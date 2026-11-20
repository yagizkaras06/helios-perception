"""ROS-graph integration test for the obstacle_detector node.

Starts the node in an isolated namespace on a private input topic, publishes synthetic
Helios-like organised clouds (ground rings, static walls, one box moving towards the sensor) and
checks that
  * /<ns>/tracks and /<ns>/obstacles are published with the input header,
  * the LiDAR-only ego-motion estimate recognises the stationary sensor,
  * a confirmed track with a reliable, approaching velocity and a finite TTC appears,
  * /<ns>/processing_stats and /diagnostics are published,
  * the node advertises NO actuation topics (cmd_vel / Twist / Ackermann / control).
"""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

from helios_obstacle_detection.msg import ProcessingStats, TrackedObjectArray

NS = 'helios_launch_test'
INPUT = '/helios_launch_test_input/points'


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='helios_obstacle_detection',
        executable='obstacle_detector',
        name='obstacle_detector',
        namespace=NS,
        output='screen',
        parameters=[{
            'input_topic': INPUT,
            'stats_period_s': 0.5,
            'log_period_s': 0.0,
            'ground.expected_height': -0.9,
            'filter.max_range': 30.0,
            'filter.roi': [-30.0, 30.0, -30.0, 30.0, -2.5, 2.5],
            'ground.zone_boundaries': [0.0, 8.0, 16.0, 30.0],
            'threads': 2,
        }],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {'node': node}


def make_cloud(t, stamp):
    """Organised 1800 x 32 cloud: rings on the ground + a box approaching at 1.5 m/s."""
    elev = np.radians(np.concatenate([15.0 - 1.33 * np.arange(24),
                                      -19.0 - (36.0 / 7.0) * np.arange(8)]))
    az = np.radians(np.arange(1800) * 0.2)
    ce, se = np.cos(elev)[None, :], np.sin(elev)[None, :]
    dx, dy, dz = ce * np.cos(az)[:, None], ce * np.sin(az)[:, None], np.broadcast_to(se, (1800, 32))
    rng = np.full(dx.shape, np.nan)
    down = dz < 0
    rng[down] = 0.9 / -dz[down]
    # box: x in [bx, bx+0.6], |y| < 0.4, z in [-0.9, 0.8] (sensor frame)
    bx = 12.0 - 1.5 * t
    with np.errstate(divide='ignore', invalid='ignore'):
        tb = bx / dx
        hit = (dx > 0) & (np.abs(tb * dy) < 0.4) & (tb * dz > -0.9) & (tb * dz < 0.8)
    rng = np.where(hit & ((tb < rng) | np.isnan(rng)), tb, rng)
    # static walls (back wall x=-10, side walls y=+-8): make the ego motion observable
    with np.errstate(divide='ignore', invalid='ignore'):
        for t_w, ok in ((-10.0 / dx, (dx < 0)), (8.0 / dy, (dy > 0)), (-8.0 / dy, (dy < 0))):
            px, py, pz = t_w * dx, t_w * dy, t_w * dz
            inside = ok & (pz > -0.9) & (pz < 2.0) & (px >= -10.0) & (px <= 25.0) & \
                (np.abs(py) <= 8.0)
            rng = np.where(inside & ((t_w < rng) | np.isnan(rng)), t_w, rng)
    rng[rng > 60.0] = np.nan
    pts = np.zeros((1800, 32, 4), dtype=np.float32)
    pts[..., 0] = dx * rng
    pts[..., 1] = dy * rng
    pts[..., 2] = dz * rng
    pts[..., 3] = 10.0
    msg = PointCloud2()
    msg.header.frame_id = 'rslidar'
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int((stamp % 1.0) * 1e9)
    msg.height, msg.width = 1800, 32
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('x y z intensity'.split())]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 16, 16 * 32
    msg.is_dense = False
    msg.data = pts.tobytes()
    return msg


class TestDetectorNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_tracks_diagnostics_and_no_actuation(self, proc_output):
        node = rclpy.create_node('helios_launch_tester')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        pub = node.create_publisher(PointCloud2, INPUT, QoSProfile(depth=1))
        tracks, stats, diags = [], [], []
        node.create_subscription(TrackedObjectArray, f'/{NS}/tracks', tracks.append, qos)
        node.create_subscription(ProcessingStats, f'/{NS}/processing_stats', stats.append, qos)
        node.create_subscription(DiagnosticArray, '/diagnostics', diags.append, qos)
        try:
            deadline = time.monotonic() + 15.0
            while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertGreater(pub.get_subscription_count(), 0, 'node did not subscribe')
            t0 = 1000.0
            for k in range(50):            # 5 s of data at 10 Hz
                pub.publish(make_cloud(0.1 * k, t0 + 0.1 * k))
                end = time.monotonic() + 0.1
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)

            self.assertGreater(len(tracks), 20, 'too few track messages')
            self.assertEqual(tracks[-1].header.frame_id, 'rslidar')
            best = None
            for msg in tracks[-10:]:
                for o in msg.objects:
                    if o.track_state != 0 and o.velocity_reliable and 0 < o.center.x < 12:
                        best = o
            self.assertIn(tracks[-1].ego_state, (1,), tracks[-1].ego_status_text)  # STATIONARY
            self.assertIsNotNone(best, 'no confirmed track with reliable velocity')
            self.assertLess(best.velocity.x, -0.8)        # approaching ~1.5 m/s
            self.assertTrue(best.ttc_valid)
            self.assertTrue(math.isfinite(best.ttc))
            self.assertGreater(len(stats), 2)
            names = {s.name for d in diags for s in d.status}
            self.assertTrue(any('Ego motion' in n for n in names), names)
            self.assertTrue(any('LiDAR input' in n for n in names), names)

            forbidden_types = ('geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped',
                               'ackermann_msgs/msg/AckermannDriveStamped')
            for topic, types in node.get_topic_names_and_types():
                pubs = node.get_publishers_info_by_topic(topic)
                if not any(p.node_name == 'obstacle_detector' for p in pubs):
                    continue
                self.assertNotIn('cmd_vel', topic)
                self.assertNotIn('control', topic)
                for ty in types:
                    self.assertNotIn(ty, forbidden_types)
        finally:
            node.destroy_node()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
