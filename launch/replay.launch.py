"""Replay a recorded Helios bag through the perception stack, isolated from the live system.

The bag is published on /bag/rslidar_points (never on the live /rslidar_points) and the detector
runs in its own namespace (default helios_bag), so it can run next to the live stack.

  ros2 launch helios_obstacle_detection replay.launch.py bag:=$HOME/helios_ws/bags/helios_test_20s
  ros2 launch helios_obstacle_detection replay.launch.py bag:=... rviz:=true loop:=true rate:=0.5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def player(context):
    cmd = ['ros2', 'run', 'helios_obstacle_detection', 'bag_tool.py', 'play',
           LaunchConfiguration('bag').perform(context),
           '--remap', '/rslidar_points:=' + LaunchConfiguration('replay_topic').perform(context),
           '--topic', '/rslidar_points',
           '--rate', LaunchConfiguration('rate').perform(context)]
    if LaunchConfiguration('loop').perform(context).lower() in ('1', 'true', 'yes'):
        cmd.append('--loop')
    # give the detector time to start and subscribe
    return [TimerAction(period=2.0, actions=[ExecuteProcess(cmd=cmd, output='screen')])]


def generate_launch_description():
    share = get_package_share_directory('helios_obstacle_detection')
    args = [
        DeclareLaunchArgument('bag', description='bag directory (MCAP)'),
        DeclareLaunchArgument('namespace', default_value='helios_bag'),
        DeclareLaunchArgument('replay_topic', default_value='/bag/rslidar_points'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'detection.yaml')),
    ]
    detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'detection.launch.py')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'input_topic': LaunchConfiguration('replay_topic'),
            'rviz': LaunchConfiguration('rviz'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )
    return LaunchDescription(args + [detection, OpaqueFunction(function=player)])
