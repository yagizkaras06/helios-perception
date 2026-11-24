"""End-to-end test of the demo launch without a sensor and without RViz.

github_demo.launch.py mode:=synthetic rviz:=false starts synthetic_lidar (simulated Helios-like
scans), a detector in /helios_sim and demo_visualizer. Checks that
  * the synthetic cloud, /helios_sim/tracks and /helios_sim/demo_markers are published,
  * a pedestrian of the scenario becomes a confirmed MOVING track,
  * every demo MarkerArray starts with DELETEALL and has no blank text marker,
  * nothing in the graph advertises an actuation topic (cmd_vel / Twist / Ackermann / control).
"""

import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from helios_obstacle_detection.msg import TrackedObjectArray

NS = 'helios_demo_test'
ACTUATION_TYPES = ('geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped',
                   'ackermann_msgs/msg/AckermannDrive', 'ackermann_msgs/msg/AckermannDriveStamped')
ACTUATION_WORDS = ('cmd_vel', 'brake', 'throttle', 'steer', 'control')


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory('helios_obstacle_detection')
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'github_demo.launch.py')),
        launch_arguments={'mode': 'synthetic', 'rviz': 'false', 'scenario': 'crossing',
                          'synthetic_namespace': NS}.items())
    return launch.LaunchDescription([demo, launch_testing.actions.ReadyToTest()]), {}


class TestGithubDemo(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('github_demo_test_probe')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_synthetic_demo_publishes_clean_markers(self):
        got = {'cloud': 0, 'tracks': [], 'markers': []}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # reliable like the detector: ~1 MB clouds are mostly lost with best effort
        self.node.create_subscription(PointCloud2, '/synthetic/rslidar_points',
                                      lambda m: got.__setitem__('cloud', got['cloud'] + 1),
                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_subscription(TrackedObjectArray, f'/{NS}/tracks',
                                      lambda m: got['tracks'].append(m), qos)
        self.node.create_subscription(MarkerArray, f'/{NS}/demo_markers',
                                      lambda m: got['markers'].append(m), qos)
        moving = False
        end = time.monotonic() + 60.0
        while time.monotonic() < end and not (moving and len(got['markers']) > 50):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            moving = any(o.track_state == 1 and o.motion_state == 2
                         for t in got['tracks'][-5:] for o in t.objects)
        self.assertGreater(got['cloud'], 10, 'no synthetic cloud')
        self.assertGreater(len(got['tracks']), 10, 'no tracks')
        self.assertTrue(moving, 'no confirmed MOVING track from the synthetic pedestrians')
        self.assertGreater(len(got['markers']), 10, 'no demo markers')
        for arr in got['markers']:
            self.assertEqual(arr.markers[0].action, Marker.DELETEALL)
            for m in arr.markers:
                if m.type == Marker.TEXT_VIEW_FACING:
                    self.assertTrue(m.text.strip(), 'blank text marker')

    def test_no_actuation_topics(self):
        time.sleep(3.0)
        for name, types in self.node.get_topic_names_and_types():
            low = name.lower()
            for word in ACTUATION_WORDS:
                self.assertNotIn(word, low, f'actuation-like topic {name}')
            for t in types:
                self.assertNotIn(t, ACTUATION_TYPES, f'{name} has actuation type {t}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
