"""Helios Perception - presentation / demo launch (visualisation only).

mode:=live (default)
    Uses the RUNNING perception stack in /<namespace> (default helios): starts demo_visualizer,
    RViz2 with config/rviz_github_demo.rviz and optionally the status panel. A detector is started
    only if none is running in that namespace (start_detector:=auto) - never a second one, and
    never the LiDAR driver.
mode:=replay
    Plays a recording through replay.launch.py (its own detector in /helios_bag on
    /bag/rslidar_points, isolated from the live system) and shows it with the same profile.
mode:=synthetic
    No sensor needed: synthetic_lidar publishes ray-cast scans of a simulator scenario
    (scenario:=crossing|approach|ego_motion|occlusion|merge_split|wall_walk|start_stop) on
    /synthetic/rslidar_points and a detector runs on them in /helios_sim. Simulated data.

  ros2 launch helios_obstacle_detection github_demo.launch.py status_panel:=true
  ros2 launch helios_obstacle_detection github_demo.launch.py mode:=replay \\
      bag:=/path/to/recording loop:=true status_panel:=true
  ros2 launch helios_obstacle_detection github_demo.launch.py mode:=synthetic scenario:=crossing
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# GUI processes started from a snap-packaged terminal (e.g. VS Code) inherit library paths that
# break RViz / Qt ("undefined symbol: __libc_pthread_init"); unset them for the GUI only.
SNAP_GUI_VARS = ['GTK_PATH', 'GTK_EXE_PREFIX', 'GIO_MODULE_DIR', 'GTK_IM_MODULE_FILE',
                 'GSETTINGS_SCHEMA_DIR', 'LOCPATH', 'GDK_PIXBUF_MODULE_FILE',
                 'GDK_PIXBUF_MODULEDIR']


def _clean_env_cmd(cmd):
    out = ['env']
    for v in SNAP_GUI_VARS:
        out += ['-u', v]
    return out + cmd


def _detector_running(ns):
    try:
        names = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True,
                               timeout=8).stdout.split()
    except (OSError, subprocess.TimeoutExpired):
        return False
    return any(n in (f'/{ns}/obstacle_detector',) for n in names)


def setup(context):
    share = get_package_share_directory('helios_obstacle_detection')
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('live', 'replay', 'synthetic'):
        raise RuntimeError(f"mode must be 'live', 'replay' or 'synthetic', not '{mode}'")
    actions = []
    if mode == 'live':
        ns = LaunchConfiguration('namespace').perform(context).strip('/')
        raw = LaunchConfiguration('input_topic').perform(context)
        start = LaunchConfiguration('start_detector').perform(context).lower()
        running = _detector_running(ns)
        if running:
            actions.append(LogInfo(msg=f'[github_demo] using the running detector /{ns}'))
        elif start in ('auto', 'true'):
            actions.append(LogInfo(msg=f'[github_demo] no detector in /{ns}: starting one'))
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, 'launch',
                                                           'detection.launch.py')),
                launch_arguments={'namespace': ns, 'input_topic': raw}.items()))
        else:
            actions.append(LogInfo(
                msg=f'[github_demo] no detector in /{ns} (start_detector:=false)'))
    elif mode == 'synthetic':
        ns = LaunchConfiguration('synthetic_namespace').perform(context).strip('/')
        raw = '/synthetic/rslidar_points'
        actions.append(Node(
            package='helios_obstacle_detection', executable='synthetic_lidar',
            name='synthetic_lidar', output='screen',
            parameters=[{'scenario': LaunchConfiguration('scenario'), 'topic': raw,
                         'loop_s': LaunchConfiguration('synthetic_loop_s')}]))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'detection.launch.py')),
            launch_arguments={'namespace': ns, 'input_topic': raw}.items()))
    else:
        ns = LaunchConfiguration('replay_namespace').perform(context).strip('/')
        raw = LaunchConfiguration('replay_topic').perform(context)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'replay.launch.py')),
            launch_arguments={
                'bag': LaunchConfiguration('bag'), 'namespace': ns, 'replay_topic': raw,
                'rate': LaunchConfiguration('rate'), 'loop': LaunchConfiguration('loop'),
                'start': LaunchConfiguration('start'),
                'rviz': 'false',
                # replay.launch.py has its own 'mode' (rerun | recorded): launch configurations
                # are shared, so set it explicitly
                'mode': 'rerun', 'bag_input_topic': '', 'bag_namespace': ''}.items()))

    # RViz profile with the namespace / raw topic of this mode
    with open(LaunchConfiguration('rviz_config').perform(context)) as f:
        text = f.read()
    text = text.replace('/helios/', f'/{ns}/').replace('Value: /rslidar_points', f'Value: {raw}')
    cfg = os.path.join('/tmp', f'helios_github_demo_{ns}.rviz')
    with open(cfg, 'w') as f:
        f.write(text)

    actions.append(Node(
        package='helios_obstacle_detection', executable='demo_visualizer.py',
        name='demo_visualizer', namespace=ns, output='screen',
        parameters=[{'language': LaunchConfiguration('language'),
                     'show_hud': LaunchConfiguration('hud')}]))
    if LaunchConfiguration('rviz').perform(context).lower() in ('1', 'true', 'yes'):
        actions.append(ExecuteProcess(
            cmd=_clean_env_cmd(['rviz2', '-d', cfg, '--ros-args', '-r',
                                '__node:=helios_demo_rviz', '-r', f'__ns:=/{ns}']),
            output='log'))
    if LaunchConfiguration('status_panel').perform(context).lower() in ('1', 'true', 'yes'):
        exe = os.path.join(os.path.dirname(share), '..', 'lib', 'helios_obstacle_detection',
                           'status_panel.py')
        actions.append(ExecuteProcess(
            cmd=_clean_env_cmd(['python3', os.path.normpath(exe), '--ros-args',
                                '-r', f'__ns:=/{ns}']),
            output='log'))
    return actions


def generate_launch_description():
    share = get_package_share_directory('helios_obstacle_detection')
    args = [
        DeclareLaunchArgument('mode', default_value='live',
                              description='live | replay | synthetic'),
        DeclareLaunchArgument('namespace', default_value='helios',
                              description='live: namespace of the running stack'),
        DeclareLaunchArgument('input_topic', default_value='/rslidar_points',
                              description='live: raw LiDAR topic shown as background'),
        DeclareLaunchArgument('start_detector', default_value='auto',
                              description='live: auto | true | false (never a second detector)'),
        DeclareLaunchArgument('bag', default_value='', description='replay: bag directory'),
        DeclareLaunchArgument('replay_namespace', default_value='helios_bag'),
        DeclareLaunchArgument('replay_topic', default_value='/bag/rslidar_points'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start', default_value='0.0',
                              description='replay: skip the first N seconds of the bag'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('scenario', default_value='crossing',
                              description='synthetic: simulator scenario'),
        DeclareLaunchArgument('synthetic_namespace', default_value='helios_sim'),
        DeclareLaunchArgument('synthetic_loop_s', default_value='30.0',
                              description='synthetic: restart the scene every N seconds'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='false: markers only (headless / tests)'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'config',
                                                         'rviz_github_demo.rviz')),
        DeclareLaunchArgument('status_panel', default_value='false',
                              description='separate window with rates / latency / health'),
        DeclareLaunchArgument('hud', default_value='false',
                              description='diagnostics text inside the 3D view'),
        DeclareLaunchArgument('language', default_value='en', description='en | tr'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
