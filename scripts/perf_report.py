#!/usr/bin/env python3
"""Collect /<ns>/processing_stats (+ tracks) for a while and print a Markdown performance report.

  ros2 run helios_obstacle_detection perf_report.py --duration 60 [--ns /helios] [--out r.md]

Reports measured input/output FPS, process CPU (one core = 100 %) and RSS as published by the
node, host latency (message received -> outputs published), per-stage timings, sensor health
counters, clock synchronisation and tracking statistics. Values are averages of the 1 Hz reports
(stage timings: the node's own rolling mean / p95 / max at the end of the window).
"""

import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from helios_obstacle_detection.msg import ProcessingStats, TrackedObjectArray

EGO = {0: 'UNKNOWN', 1: 'STATIONARY', 2: 'MOVING', 3: 'UNRELIABLE'}
RISK = {0: 'NONE', 1: 'LOW', 2: 'MEDIUM', 3: 'HIGH', 4: 'CRITICAL'}


class Collector(Node):
    def __init__(self, ns):
        super().__init__('helios_perf_report')
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.stats, self.track_counts, self.moving, self.risk = [], [], [], {}
        self.ego = {}
        self.create_subscription(ProcessingStats, f'{ns}/processing_stats', self.stats.append, qos)
        self.create_subscription(TrackedObjectArray, f'{ns}/tracks', self.on_tracks, qos)

    def on_tracks(self, m):
        conf = [o for o in m.objects if o.track_state != 0]
        self.track_counts.append(len(conf))
        self.moving.append(sum(1 for o in conf if o.motion_state == 2))
        self.risk[RISK[m.max_risk_level]] = self.risk.get(RISK[m.max_risk_level], 0) + 1
        self.ego[EGO.get(m.ego_state, '?')] = self.ego.get(EGO.get(m.ego_state, '?'), 0) + 1


def mean(xs):
    return statistics.fmean(xs) if xs else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='/helios')
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--out', default='')
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Collector(a.ns.rstrip('/'))
    end = time.monotonic() + a.duration
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    s = [x for x in node.stats if x.frames_processed > 0 and x.status != 1]
    lines = [f'# Helios perception performance ({a.duration:.0f} s, ns {a.ns})', '']
    if not s:
        lines.append('No processing_stats with data received (is the node / LiDAR running?).')
    else:
        last = s[-1]
        lines += [
            '| metric | value |', '|---|---|',
            f'| input FPS (mean / min) | {mean([x.input_fps for x in s]):.2f} / '
            f'{min(x.input_fps for x in s):.2f} |',
            f'| output FPS (mean / min) | {mean([x.output_fps for x in s]):.2f} / '
            f'{min(x.output_fps for x in s):.2f} |',
            f'| sensor stamp rate | {mean([x.sensor_rate_hz for x in s]):.2f} Hz |',
            f'| CPU, whole node (100 % = 1 core) mean / max | {mean([x.cpu_percent for x in s]):.1f}'
            f' / {max(x.cpu_percent for x in s):.1f} % |',
            f'| CPU normalised to all cores | {mean([x.cpu_percent_total for x in s]):.2f} % |',
            f'| RSS | {last.rss_mb:.1f} MB, {last.threads} threads |',
            f'| host latency rx->publish mean / p95 / max | {last.latency_rx_to_publish_mean_ms:.1f}'
            f' / {last.latency_rx_to_publish_p95_ms:.1f} / {last.latency_rx_to_publish_max_ms:.1f}'
            ' ms |',
            f'| clock synced | {last.clock_synced} (offset {last.clock_offset_s:.3f} s) |',
            f'| stamp age mean / p95 | ' + (f'{last.stamp_age_mean_ms:.1f} / '
                                            f'{last.stamp_age_p95_ms:.1f} ms' if last.clock_synced
                                            else 'unobservable (sensor clock not synchronised)')
            + ' |',
            f'| delivery jitter p95 | {last.delivery_jitter_p95_ms:.2f} ms |',
            f'| frames rx / processed / superseded / overload | {last.frames_received} / '
            f'{last.frames_processed} / {last.frames_dropped} / {last.overload_frames} |',
            f'| gaps / missed (est.) / time jumps / partial scans | {last.frame_gaps} / '
            f'{last.missed_frames_estimate} / {last.time_jumps} / {last.partial_scans} |',
            f'| valid return ratio / azimuth coverage | {mean([x.valid_return_ratio for x in s]):.3f}'
            f' / {mean([x.azimuth_coverage for x in s]):.3f} |',
            f'| voxel leaf (adaptive) | {last.voxel_leaf_size:.3f} m |',
            f'| ground | {last.ground_method}: {last.ground_detail} |',
            f'| confirmed tracks per frame / moving | {mean(node.track_counts):.1f} / '
            f'{mean(node.moving):.2f} |',
            f'| ego states (frames) | {node.ego} |',
            f'| max risk (frames) | {node.risk} |',
            '', '| stage | mean ms | p95 ms | max ms |', '|---|---|---|---|',
        ]
        for t in last.timings:
            lines.append(f'| {t.name} | {t.mean_ms:.2f} | {t.p95_ms:.2f} | {t.max_ms:.2f} |')
        statuses = {}
        for x in node.stats:
            statuses[x.status_text.split(':')[0]] = statuses.get(x.status_text.split(':')[0], 0) + 1
        lines += ['', f'Status reports: {statuses}']
    text = '\n'.join(lines)
    print(text)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(text + '\n')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
