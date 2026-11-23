#!/usr/bin/env python3
"""rosbag2 recorder / player / inspector built on rosbag2_py (MCAP).

Exists because `ros2 bag record/play` of the installed rosbag2 fails on this machine with
"Couldn't parse log level: '--log-level info'" (CLI bug, not a data problem). Messages are stored
and replayed serialized (no deserialisation cost) with their original timing; bags remain
readable by `ros2 bag info` and the MCAP tools.

  # raw LiDAR only (default)
  ros2 run helios_obstacle_detection bag_tool.py record -o ~/helios_ws/bags/run1 --duration 30
  # raw LiDAR + every perception output + diagnostics (for later inspection in RViz)
  ros2 run helios_obstacle_detection bag_tool.py record -o ~/helios_ws/bags/run2 --preset perception
  # explicit topics
  ros2 run helios_obstacle_detection bag_tool.py record -o out \
      --topic /rslidar_points --topic /helios/tracks

  ros2 run helios_obstacle_detection bag_tool.py info ~/helios_ws/bags/run1
  # events written by the event_recorder node (reason, time, coverage, replay command)
  ros2 run helios_obstacle_detection bag_tool.py events [~/helios_ws/logs/events]
  # replay on a separate topic so it never mixes with the live sensor
  ros2 run helios_obstacle_detection bag_tool.py play ~/helios_ws/bags/run1 \
      --remap /rslidar_points:=/bag/rslidar_points [--rate 1.0] [--loop] [--start 5.0]
"""

import argparse
import json
import os
import time

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

PRESETS = {
    'raw': ['/rslidar_points'],
    'perception': [
        '/rslidar_points',
        '/helios/obstacles', '/helios/tracks', '/helios/processing_stats',
        '/helios/track_markers', '/helios/obstacle_markers', '/helios/obstacle_clusters',
        '/diagnostics',
    ],
}


def storage(uri):
    return (rosbag2_py.StorageOptions(uri=os.path.expanduser(uri), storage_id='mcap'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))


def record(a):
    topics = list(dict.fromkeys((a.topic or []) + PRESETS[a.preset]))
    out = os.path.expanduser(a.output)
    if os.path.exists(out):
        raise SystemExit(f'{out} already exists - refusing to overwrite')
    rclpy.init()
    node = Node('helios_bag_recorder')
    # wait for the topic types (topics that never appear are skipped with a warning)
    types = {}
    deadline = time.monotonic() + a.discovery_timeout
    while time.monotonic() < deadline and len(types) < len(topics):
        for name, tys in node.get_topic_names_and_types():
            if name in topics and name not in types:
                types[name] = tys[0]
        rclpy.spin_once(node, timeout_sec=0.1)
    missing = [t for t in topics if t not in types]
    if missing:
        print(f'warning: not found (skipped): {", ".join(missing)}')
    if not types:
        raise SystemExit('none of the requested topics exist')

    writer = rosbag2_py.SequentialWriter()
    writer.open(*storage(out))
    counts = {}
    for i, (name, ty) in enumerate(types.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=i, name=name, type=ty, serialization_format='cdr'))
        counts[name] = 0

        def cb(raw, name=name):
            writer.write(name, raw, node.get_clock().now().nanoseconds)
            counts[name] += 1

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(get_message(ty), name, cb, qos, raw=True)

    end = time.monotonic() + a.duration
    print(f'recording {len(types)} topic(s) for {a.duration:.0f} s -> {out}')
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    writer.close()
    for name, n in counts.items():
        print(f'  {name}: {n} messages')
    node.destroy_node()
    rclpy.try_shutdown()


def info(a):
    reader = rosbag2_py.SequentialReader()
    reader.open(*storage(a.bag))
    meta = reader.get_metadata()
    print(f'bag: {a.bag}\nduration: {meta.duration.nanoseconds / 1e9:.2f} s, '
          f'messages: {meta.message_count}')
    for t in meta.topics_with_message_count:
        print(f'  {t.topic_metadata.name} [{t.topic_metadata.type}] {t.message_count}')


def play(a):
    remap = {}
    for r in a.remap:
        src, dst = r.split(':=')
        remap[src] = dst
    reader = rosbag2_py.SequentialReader()
    reader.open(*storage(a.bag))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if a.topic:
        topics = {n: t for n, t in topics.items() if n in a.topic}
    rclpy.init()
    node = Node('helios_bag_player')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    pubs = {n: node.create_publisher(get_message(t), remap.get(n, n), qos)
            for n, t in topics.items()}
    msgs = []
    while reader.has_next():
        name, data, t = reader.read_next()
        if name in pubs:
            msgs.append((name, data, t))
    if not msgs:
        raise SystemExit('no messages to play')
    t_first = msgs[0][2]
    msgs = [m for m in msgs if (m[2] - t_first) / 1e9 >= a.start]
    time.sleep(a.discovery_wait)  # let subscribers discover the publishers
    print(f'playing {len(msgs)} messages from {a.bag} at x{a.rate} (topics: '
          f'{", ".join(remap.get(n, n) for n in topics)})')
    try:
        while rclpy.ok():
            t0_bag = msgs[0][2]
            t0 = time.monotonic()
            for name, data, t in msgs:
                wait = (t - t0_bag) / 1e9 / a.rate - (time.monotonic() - t0)
                if wait > 0:
                    time.sleep(wait)
                pubs[name].publish(data)  # serialized bytes
            if not a.loop:
                break
    except KeyboardInterrupt:
        pass
    print('playback finished')
    node.destroy_node()
    rclpy.try_shutdown()


def events(a):
    root = os.path.expanduser(a.dir)
    if not os.path.isdir(root):
        raise SystemExit(f'no events folder: {root}')
    rows = []
    for d in sorted(os.listdir(root)):
        meta_path = os.path.join(root, d, 'metadata.json')
        if not os.path.exists(meta_path):
            continue
        with open(meta_path) as f:
            m = json.load(f)
        reasons = ', '.join(
            r['type'] + (f"(track {r['track_id']})" if r.get('track_id') is not None else '')
            for r in m.get('reasons', []))
        w, i = m.get('window', {}), m.get('integrity', {})
        size = sum(os.path.getsize(os.path.join(dp, fn))
                   for dp, _, fns in os.walk(os.path.join(root, d)) for fn in fns)
        flags = []
        if w.get('pre_limited_by'):
            flags.append(f"pre {w.get('pre_covered_s', 0):.1f}s ({w['pre_limited_by']})")
        if i.get('sequence_gaps'):
            flags.append(f"{i['sequence_gaps']} dropped")
        if i.get('truncated_by_disk_budget'):
            flags.append('truncated')
        if i.get('write_error'):
            flags.append('WRITE ERROR')
        rows.append((d, m.get('trigger_wall_time', '?'), reasons,
                     w.get('recorded_to_s', 0) - w.get('recorded_from_s', 0), size, flags,
                     os.path.join(root, d, 'bag')))
    if not rows:
        print(f'no events in {root}')
        return
    for d, t, reasons, dur, size, flags, bag in rows:
        print(f'{d}\n  time {t} | {reasons} | {dur:.1f} s | {size / 1e6:.1f} MB'
              + (f" | {'; '.join(flags)}" if flags else ''))
        print('  replay: ros2 launch helios_obstacle_detection replay.launch.py '
              f'bag:={bag} rviz:=true [mode:=recorded]')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record')
    r.add_argument('-o', '--output', required=True)
    r.add_argument('--topic', action='append', help='may be repeated')
    r.add_argument('--preset', choices=sorted(PRESETS), default='raw')
    r.add_argument('--duration', type=float, default=20.0)
    r.add_argument('--discovery-timeout', type=float, default=5.0)
    i = sub.add_parser('info')
    i.add_argument('bag')
    p = sub.add_parser('play')
    p.add_argument('bag')
    p.add_argument('--remap', action='append', default=[])
    p.add_argument('--topic', action='append', help='only these topics (may be repeated)')
    p.add_argument('--rate', type=float, default=1.0)
    p.add_argument('--start', type=float, default=0.0, help='skip the first N seconds')
    p.add_argument('--loop', action='store_true')
    p.add_argument('--discovery-wait', type=float, default=1.0)
    e = sub.add_parser('events')
    e.add_argument('dir', nargs='?', default='~/helios_ws/logs/events')
    a, _ = ap.parse_known_args()   # tolerate --ros-args appended by launch
    {'record': record, 'info': info, 'play': play, 'events': events}[a.cmd](a)


if __name__ == '__main__':
    main()
