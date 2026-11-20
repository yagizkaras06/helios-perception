#!/usr/bin/env python3
"""Print obstacle candidates and processing statistics in a compact table.

Usage:
  ros2 run helios_obstacle_detection obstacle_report.py               # live, until Ctrl+C
  ros2 run helios_obstacle_detection obstacle_report.py --once        # one frame + one stats
  ros2 run helios_obstacle_detection obstacle_report.py --ns /helios --top 10
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from helios_obstacle_detection.msg import ObstacleArray, ProcessingStats, TrackedObjectArray

GROUND = {0: 'RELIABLE', 1: 'DEGRADED', 2: 'FAILED'}
TRACK = {0: 'TENT', 1: 'CONF', 2: 'COAST'}
MOTION = {0: '?', 1: 'STATIC', 2: 'MOVING'}
RISK = {0: 'NONE', 1: 'LOW', 2: 'MEDIUM', 3: 'HIGH', 4: 'CRITICAL'}
EGO = {0: 'UNKNOWN', 1: 'STATIONARY', 2: 'MOVING', 3: 'UNRELIABLE'}


class Report(Node):
    def __init__(self, ns, top, once):
        super().__init__('obstacle_report')
        self.top, self.once = top, once
        self.got_obst = self.got_stats = self.got_tracks = False
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(ObstacleArray, f'{ns}/obstacles', self.on_obst, qos)
        self.create_subscription(ProcessingStats, f'{ns}/processing_stats', self.on_stats, qos)
        self.create_subscription(TrackedObjectArray, f'{ns}/tracks', self.on_tracks, qos)

    def on_tracks(self, m):
        if self.once and self.got_tracks:
            return
        self.got_tracks = True
        ev = m.ego_velocity
        print(f'\n--- tracks | ego {EGO.get(m.ego_state)} v=({ev.x:.2f},{ev.y:.2f}) m/s '
              f'w={m.ego_yaw_rate:.3f} rad/s | max risk {RISK.get(m.max_risk_level)} | '
              f'{m.ego_status_text}')
        print('   id state motion   dist[m]  speed[m/s]      heading  TTC[s] (pess)  risk')
        objs = sorted([o for o in m.objects if o.track_state != 0],
                      key=lambda o: o.closest_distance)
        for o in objs[: self.top]:
            spd = f'{o.speed:4.2f}+-{o.speed_sigma:4.2f}' if o.velocity_reliable else '   n/a    '
            hdg = f'{math.degrees(o.heading):6.1f} deg' if o.heading_valid else '     -    '
            ttc = (f'{o.ttc:5.1f} ({o.ttc_pessimistic:4.1f})'
                   if o.ttc_valid and math.isfinite(o.ttc_pessimistic) else '     -      ')
            print(f'{o.id:5d} {TRACK.get(o.track_state):5s} {MOTION.get(o.motion_state):6s} '
                  f'{o.closest_distance:7.2f}  {spd}  {hdg}  {ttc}  {RISK.get(o.risk_level)}'
                  f'{"  LARGE" if o.large_structure else ""}')

    def on_obst(self, m):
        if self.once and self.got_obst:
            return
        self.got_obst = True
        print(f'\n--- frame {m.header.frame_id} stamp {m.header.stamp.sec}.'
              f'{m.header.stamp.nanosec:09d} | ground {GROUND.get(m.ground_status)} | '
              f'{len(m.obstacles)} obstacle candidates')
        print(' id   pts    near[m]  ctr[m]   center(x,y,z)             '
              'size L x W x H [m]      bearing')
        for o in m.obstacles[: self.top]:
            c, s = o.center, o.size
            bearing = math.degrees(math.atan2(o.closest_point.y, o.closest_point.x))
            print(f'{o.id:3d} {o.point_count:5d}  {o.closest_distance:7.2f} '
                  f'{o.center_distance:7.2f}   ({c.x:6.2f},{c.y:6.2f},{c.z:5.2f})   '
                  f'{s.x:5.2f} x {s.y:5.2f} x {s.z:4.2f}   {bearing:6.1f} deg'
                  f'{"  LARGE" if o.large_structure else ""}')

    def on_stats(self, s):
        if self.once and self.got_stats:
            return
        self.got_stats = True
        print(f'\n=== {s.status_text} | in {s.input_fps:.2f} Hz  out {s.output_fps:.2f} Hz | '
              f'rx {s.frames_received} proc {s.frames_processed} drop {s.frames_dropped} '
              f'overload {s.overload_frames}')
        print(f'    points in {s.points_input} valid {s.points_valid} roi {s.points_roi} '
              f'voxel {s.points_voxel} ground {s.points_ground} obstacle {s.points_obstacle} '
              f'| clusters {s.clusters}')
        print(f'    ground {GROUND.get(s.ground_status)} ({s.ground_method}): {s.ground_detail}')
        print(f'    CPU {s.cpu_percent:.0f}% (1 core) RSS {s.rss_mb:.0f} MB | latency rx->pub '
              f'{s.latency_rx_to_publish_mean_ms:.1f}/{s.latency_rx_to_publish_p95_ms:.1f} ms '
              f'(mean/p95) | clock synced {s.clock_synced} | health: {s.sensor_health_text}')
        print(f'    tracks {s.tracks_confirmed} confirmed, {s.tracks_moving} moving | '
              f'ego {s.ego_status_text}')
        for t in s.timings:
            print(f'    {t.name:20s} last {t.last_ms:7.2f}  mean {t.mean_ms:7.2f}  '
                  f'p95 {t.p95_ms:7.2f}  max {t.max_ms:7.2f} ms')

    def done(self):
        return self.once and self.got_obst and self.got_stats and self.got_tracks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='/helios')
    ap.add_argument('--top', type=int, default=15)
    ap.add_argument('--once', action='store_true')
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Report(a.ns.rstrip('/'), a.top, a.once)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
