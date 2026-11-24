"""Replay a recorded Helios bag through the perception stack, isolated from the live system.

The bag is published on /bag/rslidar_points (never on the live /rslidar_points) and the detector
runs in its own namespace (default helios_bag), so it can run next to the live stack.

  ros2 launch helios_obstacle_detection replay.launch.py bag:=$HOME/helios_ws/bags/helios_test_20s
  ros2 launch helios_obstacle_detection replay.launch.py bag:=... rviz:=true loop:=true rate:=0.5

Event recordings (event_recorder) contain the raw input AND the outputs:
  # detector runs again on the raw input (current code / parameters)
  ros2 launch helios_obstacle_detection replay.launch.py bag:=<event>/bag rviz:=true
  # recorded outputs exactly as they were (tracks, boxes, warning, stats) under /helios_bag/...
  ros2 launch helios_obstacle_detection replay.launch.py bag:=<event>/bag mode:=recorded rviz:=true
"""

import json
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bag_tool_cmd():
    """bag_tool.py run directly (not through `ros2 run`): launch then signals the tool itself, so
    it stops with the launch instead of being left behind when the `ros2 run` wrapper is killed."""
    return [os.path.join(get_package_prefix('helios_obstacle_detection'), 'lib',
                         'helios_obstacle_detection', 'bag_tool.py')]


def player(context):
    mode = LaunchConfiguration('mode').perform(context)
    bag = LaunchConfiguration('bag').perform(context)
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    # topic names of an event recording are taken from its metadata.json (empty arguments = auto)
    src_input = LaunchConfiguration('bag_input_topic').perform(context)
    src_ns_arg = LaunchConfiguration('bag_namespace').perform(context)
    meta = os.path.join(os.path.dirname(os.path.abspath(bag.rstrip('/'))), 'metadata.json')
    auto_input, auto_ns = '/rslidar_points', 'helios'
    if os.path.exists(meta):
        with open(meta) as f:
            topics = json.load(f).get('topics', [])
        auto_input = next((t['name'] for t in topics
                           if t['type'] == 'sensor_msgs/msg/PointCloud2'), auto_input)
        auto_ns = next((t['name'][:-len('/tracks')].strip('/') for t in topics
                        if t['name'].endswith('/tracks')), auto_ns)
    src_input = src_input or auto_input
    cmd = bag_tool_cmd() + ['play', bag,
                            '--rate', LaunchConfiguration('rate').perform(context),
                            '--start', LaunchConfiguration('start').perform(context)]
    if mode == 'recorded':
        # recorded outputs exactly as they were published, under /<namespace>/...; the raw input is
        # shown through the RViz point-cloud display (/<namespace>/filtered_points)
        src_ns = (src_ns_arg or auto_ns).strip('/')
        cmd += ['--remap', f'{src_input}:=/{ns}/filtered_points', '--topic', src_input]
        for t in ('tracks', 'obstacles', 'processing_stats', 'track_markers', 'collision_warning'):
            cmd += ['--remap', f'/{src_ns}/{t}:=/{ns}/{t}', '--topic', f'/{src_ns}/{t}']
    elif mode == 'rerun':
        cmd += ['--remap', src_input + ':=' + LaunchConfiguration('replay_topic').perform(context),
                '--topic', src_input]
    else:
        raise RuntimeError(f"mode must be 'rerun' or 'recorded', not '{mode}'")
    if LaunchConfiguration('loop').perform(context).lower() in ('1', 'true', 'yes'):
        cmd.append('--loop')
    actions = [TimerAction(period=2.0, actions=[ExecuteProcess(cmd=cmd, output='screen')])]
    share = get_package_share_directory('helios_obstacle_detection')
    if mode == 'rerun':
        # the detector runs again on the raw input (current parameters / code)
        actions.insert(0, IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'detection.launch.py')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'input_topic': LaunchConfiguration('replay_topic'),
                'rviz': LaunchConfiguration('rviz'),
                'params_file': LaunchConfiguration('params_file'),
            }.items()))
    elif LaunchConfiguration('rviz').perform(context).lower() in ('1', 'true', 'yes'):
        src = os.path.join(share, 'rviz', 'detection.rviz')
        with open(src) as f:
            text = f.read().replace('/helios/', f'/{ns}/')
        dst = os.path.join('/tmp', f'helios_recorded_{ns}.rviz')
        with open(dst, 'w') as f:
            f.write(text)
        actions.append(Node(package='rviz2', executable='rviz2', name='helios_replay_rviz',
                            namespace=ns, arguments=['-d', dst], output='log'))
    return actions


def generate_launch_description():
    share = get_package_share_directory('helios_obstacle_detection')
    args = [
        DeclareLaunchArgument('bag', description='bag directory (MCAP)'),
        DeclareLaunchArgument('namespace', default_value='helios_bag'),
        DeclareLaunchArgument('replay_topic', default_value='/bag/rslidar_points'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start', default_value='0.0',
                              description='skip the first N seconds of the bag'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('mode', default_value='rerun',
                              description="rerun: detector runs again on the raw input; "
                                          "recorded: play the recorded outputs as they were"),
        DeclareLaunchArgument('bag_input_topic', default_value='',
                              description='raw LiDAR topic inside the bag (empty: from the '
                                          'event metadata.json, else /rslidar_points)'),
        DeclareLaunchArgument('bag_namespace', default_value='',
                              description='namespace of the recorded outputs, mode:=recorded '
                                          '(empty: from metadata.json, else helios)'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'detection.yaml')),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=player)])
