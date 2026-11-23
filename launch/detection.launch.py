"""Launch the Helios 32 perception stack (detection, tracking, risk; perception only).

The LiDAR driver (rslidar_sdk) is NOT started or changed here; it is assumed to be running.

Examples:
  ros2 launch helios_obstacle_detection detection.launch.py
  ros2 launch helios_obstacle_detection detection.launch.py rviz:=true
  ros2 launch helios_obstacle_detection detection.launch.py input_topic:=/other_points
  # record raw LiDAR + every output + diagnostics while running:
  ros2 launch helios_obstacle_detection detection.launch.py record:=true record_dir:=~/helios_ws/bags/run1
  # automatic event recording (5 s before / after CRITICAL risk, data loss, ego loss):
  ros2 launch helios_obstacle_detection detection.launch.py record_events:=true [events_dir:=/path]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def rviz_config_for_namespace(context):
    """RViz config with topics rewritten to the chosen namespace (written to /tmp)."""
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    src = LaunchConfiguration('rviz_config').perform(context)
    if ns == 'helios':
        return src
    with open(src) as f:
        text = f.read().replace('/helios/', f'/{ns}/')
    dst = os.path.join('/tmp', f'helios_detection_{ns}.rviz')
    with open(dst, 'w') as f:
        f.write(text)
    return dst


def launch_setup(context):
    ns = LaunchConfiguration('namespace')
    detector = Node(
        package='helios_obstacle_detection',
        executable='obstacle_detector',
        name='obstacle_detector',
        namespace=ns,
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='helios_detection_rviz',
        namespace=ns,
        arguments=['-d', rviz_config_for_namespace(context)],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )
    nsv = ns.perform(context).strip('/')
    record = ExecuteProcess(
        cmd=['ros2', 'run', 'helios_obstacle_detection', 'bag_tool.py', 'record',
             '-o', LaunchConfiguration('record_dir'), '--duration',
             LaunchConfiguration('record_duration'),
             '--topic', LaunchConfiguration('input_topic'),
             '--topic', f'/{nsv}/obstacles', '--topic', f'/{nsv}/tracks',
             '--topic', f'/{nsv}/processing_stats', '--topic', f'/{nsv}/track_markers',
             '--topic', '/diagnostics'],
        condition=IfCondition(LaunchConfiguration('record')),
        output='screen',
    )
    recorder_params = [
        LaunchConfiguration('params_file'),
        {
            'input_topic': LaunchConfiguration('input_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'events.detector_node': 'obstacle_detector',
        },
    ]
    events_dir = LaunchConfiguration('events_dir').perform(context)
    if events_dir:
        recorder_params.append({'events.output_dir': events_dir})
    recorder = Node(
        package='helios_obstacle_detection',
        executable='event_recorder',
        name='event_recorder',
        namespace=ns,
        output='screen',
        emulate_tty=True,
        parameters=recorder_params,
        condition=IfCondition(LaunchConfiguration('record_events')),
    )
    return [detector, rviz, record, recorder]


def generate_launch_description():
    share = get_package_share_directory('helios_obstacle_detection')
    args = [
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'detection.yaml')),
        DeclareLaunchArgument('namespace', default_value='helios',
                              description='Output topics: /<namespace>/...'),
        DeclareLaunchArgument('input_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'rviz', 'detection.rviz')),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('record_dir', default_value='/tmp/helios_record'),
        DeclareLaunchArgument('record_duration', default_value='60'),
        DeclareLaunchArgument('record_events', default_value='false',
                              description='start the event_recorder node (pre/post event bags)'),
        DeclareLaunchArgument('events_dir', default_value='',
                              description='event folder (default: events.output_dir in params)'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
