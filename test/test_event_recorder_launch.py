"""ROS-graph test of the event recorder next to the obstacle_detector node.

Synthetic Helios-like clouds (ground rings, static walls, a box driving at the sensor) are
published on a private topic. Checks:
  * start-up (detector up, no data, no ego estimate yet) creates NO event,
  * the box getting CRITICAL creates an event (reason critical_risk, with the track id),
  * stopping the stream creates an event (reason no_data); no ego_unknown event appears,
  * each event folder holds a readable rosbag2 (raw input, tracks, obstacles, processing_stats,
    the detector's diagnostics) with byte-identical messages: raw header stamps / order match
    what was published, the pre window is covered, and metadata.json is complete,
  * the recorder reports through /diagnostics.
"""

import json
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import rosbag2_py
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_node_launch import make_cloud  # noqa: E402  (same synthetic scene)

NS = 'helios_event_test'
INPUT = '/helios_event_test_input/points'
OUT = tempfile.mkdtemp(prefix='helios_events_')
PRE, POST = 3.0, 2.0


@pytest.mark.launch_test
def generate_test_description():
    common = {
        'input_topic': INPUT,
        'input_reliability': 'reliable',
    }
    detector = launch_ros.actions.Node(
        package='helios_obstacle_detection', executable='obstacle_detector',
        name='obstacle_detector', namespace=NS, output='screen',
        parameters=[dict(common, **{
            'stats_period_s': 0.25, 'log_period_s': 0.0, 'input_timeout_s': 0.5,
            'ground.expected_height': -0.9, 'filter.max_range': 30.0,
            'filter.roi': [-30.0, 30.0, -30.0, 30.0, -2.5, 2.5],
            'ground.zone_boundaries': [0.0, 8.0, 16.0, 30.0], 'threads': 2,
        })])
    recorder = launch_ros.actions.Node(
        package='helios_obstacle_detection', executable='event_recorder',
        name='event_recorder', namespace=NS, output='screen',
        parameters=[dict(common, **{
            'events.output_dir': OUT, 'events.pre_event_s': PRE, 'events.post_event_s': POST,
            'events.max_event_s': 20.0, 'events.rearm_s': 1.0, 'events.min_free_disk_mb': 10.0,
        })])
    return launch.LaunchDescription([detector, recorder, launch_testing.actions.ReadyToTest()]), {}


def events():
    out = []
    for d in sorted(os.listdir(OUT)):
        meta = os.path.join(OUT, d, 'metadata.json')
        if os.path.exists(meta):
            with open(meta) as f:
                out.append((os.path.join(OUT, d), json.load(f)))
    return out


def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=os.path.join(path, 'bag'), storage_id='mcap'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    msgs = []
    while r.has_next():
        msgs.append(r.read_next())
    return types, msgs


class TestEventRecorder(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        shutil.rmtree(OUT, ignore_errors=True)

    def spin(self, node, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def test_events(self, proc_output):
        node = rclpy.create_node('helios_event_tester')
        pub = node.create_publisher(PointCloud2, INPUT, QoSProfile(depth=1))
        notes, diags = [], []
        node.create_subscription(
            String, f'/{NS}/recorder_events', lambda m: notes.append(json.loads(m.data)),
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(DiagnosticArray, '/diagnostics', diags.append,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        try:
            deadline = time.monotonic() + 15.0
            while pub.get_subscription_count() < 2 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertGreaterEqual(pub.get_subscription_count(), 2, 'detector + recorder')
            # ---- start-up: detector reports NO_DATA, no ego yet -> must not be an event
            self.spin(node, 3.0)
            self.assertEqual(events(), [], 'start-up created an event')
            self.assertEqual([n for n in notes if n['state'] == 'started'], [])
            # ---- 9 s of data: the box reaches CRITICAL at ~6 s
            t0, sent = 2000.0, []
            for k in range(90):
                stamp = t0 + 0.1 * k
                pub.publish(make_cloud(0.1 * k, stamp))
                sent.append(stamp)
                self.spin(node, 0.1)
            # ---- stream stops -> NO_DATA; wait for both events to be written
            deadline = time.monotonic() + 25.0
            while time.monotonic() < deadline:
                self.spin(node, 0.5)
                ev = events()
                reasons = {r['type'] for _, m in ev for r in m['reasons']}
                if {'critical_risk', 'no_data'} <= reasons and \
                        len([n for n in notes if n['state'] == 'closed']) >= len(ev) > 0:
                    break
            ev = events()
            reasons = [r for _, m in ev for r in m['reasons']]
            types = {r['type'] for r in reasons}
            self.assertIn('critical_risk', types, ev)
            self.assertIn('no_data', types, ev)
            self.assertNotIn('ego_unknown', types, 'ego estimate was valid all the time')
            crit = next(r for r in reasons if r['type'] == 'critical_risk')
            self.assertIsNotNone(crit['track_id'])

            sent_ns = [int(round(s * 1e9)) for s in sent]
            for path, meta in ev:
                # metadata completeness
                for key in ('event_id', 'software_version', 'trigger_wall_time', 'reasons',
                            'window', 'topics', 'integrity', 'parameters', 'replay'):
                    self.assertIn(key, meta)
                self.assertTrue(meta['software_version'].startswith('0.3'))
                self.assertEqual(meta['integrity']['sequence_gaps'], 0)
                self.assertEqual(meta['integrity']['out_of_order'], 0)
                self.assertIsNone(meta['integrity']['write_error'])
                self.assertAlmostEqual(meta['window']['pre_covered_s'], PRE, delta=0.25)
                self.assertGreaterEqual(meta['window']['event_end_s'], POST - 0.1)
                self.assertEqual(meta['parameters']['detector'].get('input_topic'), INPUT)
                self.assertEqual(meta['parameters']['recorder'].get('events.pre_event_s'), PRE)
                # bag: topics, content, stamps, order
                types_in_bag, msgs = read_bag(path)
                for t in (INPUT, f'/{NS}/tracks', f'/{NS}/obstacles',
                          f'/{NS}/processing_stats', '/diagnostics'):
                    self.assertIn(t, types_in_bag)
                counts = {}
                for topic, _, _ in msgs:
                    counts[topic] = counts.get(topic, 0) + 1
                self.assertGreater(counts.get(f'/{NS}/processing_stats', 0), 0)
                self.assertGreater(counts.get('/diagnostics', 0), 0)
                times = [t for _, _, t in msgs]
                self.assertEqual(times, sorted(times), 'bag not in time order')
                raw = [deserialize_message(d, PointCloud2) for t, d, _ in msgs if t == INPUT]
                stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in raw]
                self.assertEqual(stamps, sorted(stamps))
                self.assertEqual(len(stamps), len(set(stamps)), 'duplicate raw messages')
                for s in stamps:   # header stamps preserved exactly
                    self.assertTrue(any(abs(s - x) < 1000 for x in sent_ns), s)
                # no gap inside the recorded raw sequence (10 Hz, consecutive frames)
                idx = [min(range(len(sent_ns)), key=lambda i: abs(sent_ns[i] - s)) for s in stamps]
                self.assertEqual(idx, list(range(idx[0], idx[0] + len(idx))), 'raw frames lost')
                if meta['reasons'][0]['type'] == 'critical_risk':
                    # the raw input covers the whole pre + post window (publisher rate ~9-10 Hz)
                    raw_t = [t for tp, _, t in msgs if tp == INPUT]
                    self.assertGreaterEqual((raw_t[-1] - raw_t[0]) * 1e-9, PRE + POST - 0.4)
                    self.assertEqual(raw[0].width * raw[0].height, 1800 * 32)
            # recorder health is visible in /diagnostics
            names = {s.name for d in diags for s in d.status}
            self.assertTrue(any('Event recorder' in n for n in names), names)
        finally:
            node.destroy_node()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
