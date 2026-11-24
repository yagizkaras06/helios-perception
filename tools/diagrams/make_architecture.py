"""Generate assets/diagrams/architecture.svg (data flow of helios_obstacle_detection).

  python3 tools/diagrams/make_architecture.py      # from the package root
"""
import os
W, H = 1280, 1010
out = []
A = out.append
INK, MUTED, LINE = '#1f2328', '#57606a', '#8c959f'
BLUE_F, BLUE_S = '#eef4ff', '#4a72c4'
GRN_F, GRN_S = '#eefaf0', '#3d8b50'
AMB_F, AMB_S = '#fff7e6', '#b7791f'
GRY_F, GRY_S = '#f6f8fa', '#8c959f'
RED_S = '#c93c37'


def box(x, y, w, h, title, lines=(), fill=GRY_F, stroke=GRY_S, tsize=15, dash=False):
    d = ' stroke-dasharray="6 4"' if dash else ''
    A(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{fill}" stroke="{stroke}" stroke-width="1.5"{d}/>')
    A(f'<text x="{x + 12}" y="{y + 22}" font-size="{tsize}" font-weight="600" fill="{INK}">{title}</text>')
    for k, ln in enumerate(lines):
        A(f'<text x="{x + 12}" y="{y + 42 + 17 * k}" font-size="12.5" fill="{MUTED}">{ln}</text>')


def arrow(x1, y1, x2, y2, label=None, lx=None, ly=None, color=LINE):
    A(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="1.8" marker-end="url(#arr)"/>')
    if label:
        A(f'<text x="{lx if lx is not None else (x1 + x2) / 2 + 6}" y="{ly if ly is not None else (y1 + y2) / 2}" font-size="12" fill="{MUTED}">{label}</text>')


A(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="Helvetica, Arial, sans-serif">')
A('<defs><marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#8c959f"/></marker></defs>')
A(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
A(f'<text x="40" y="44" font-size="22" font-weight="700" fill="{INK}">Helios Perception: data flow</text>')
A(f'<text x="40" y="68" font-size="13" fill="{MUTED}">ROS 2 Jazzy, C++17. Default configuration (static map on). Perception only: no actuation topics are published.</text>')

# sources
box(40, 95, 370, 88, 'RoboSense Helios 32 (rslidar_sdk)', ['/rslidar_points  sensor_msgs/PointCloud2', '10 Hz, 32 x 1800 (57,600 points), XYZI'], BLUE_F, BLUE_S)
box(455, 95, 370, 88, 'Recording (MCAP)', ['bag_tool.py play / replay.launch.py', 'republished on /bag/rslidar_points'], GRY_F, GRY_S, dash=True)
box(870, 95, 370, 88, 'synthetic_lidar (no sensor needed)', ['ray-cast Helios-32-like scans of simulator', 'scenarios on /synthetic/rslidar_points'], GRY_F, GRY_S, dash=True)
arrow(225, 183, 560, 238)
arrow(640, 183, 640, 238)
arrow(1055, 183, 720, 238)

# process frame
A(f'<rect x="30" y="240" width="1220" height="560" rx="12" fill="none" stroke="{INK}" stroke-width="1.6"/>')
A(f'<text x="48" y="266" font-size="15" font-weight="700" fill="{INK}">perception_node</text>')
A(f'<text x="196" y="266" font-size="12.5" fill="{MUTED}">input: newest frame only (reliable, depth 1, no backlog)  |  worker thread runs PerceptionPipeline (ROS-independent; tests and benchmark run the same code)</text>')

# pipeline row 1
y1 = 290
box(50, y1, 270, 108, 'CloudProcessor', ['field / NaN validation, range, ROI,', 'self-filter, azimuth coverage', '(one pass)'])
box(345, y1, 200, 108, 'Voxel grid', ['leaf size adapts to the', 'processing-time budget'])
box(570, y1, 300, 108, 'CZM ground segmentation', ['concentric zones / rings / sectors,', 'per-cell PCA, adaptive thresholds,', 'fallback plane when degraded'])
box(895, y1, 335, 108, 'Ego-motion (LiDAR only)', ['planar point-to-plane ICP + covariance;', 'areas of moving tracks excluded;', 'STATIONARY / MOVING / UNRELIABLE / UNKNOWN'], AMB_F, AMB_S)
arrow(320, y1 + 54, 345, y1 + 54)
arrow(545, y1 + 54, 570, y1 + 54)
arrow(870, y1 + 54, 895, y1 + 54)

# row 2
y2 = 432
box(50, y2, 370, 108, 'Static background map', ['background / foreground split in the world', 'frame (ego pose); learned in the first ~10 s,', 'updated after tracking (movers not absorbed)'])
box(445, y2, 370, 108, 'Clustering + boxes', ['range-adaptive Euclidean clustering', '(uniform 3D grid neighbour search),', 'AABB + L-shape oriented box'])
box(840, y2, 390, 108, 'Multi-object tracker', ['constant-velocity Kalman, gating, Hungarian;', 'TENTATIVE / CONFIRMED / OCCLUDED / LOST;', 'motion: STATIC / MOVING / UNKNOWN (onset test)'])
arrow(1062, y1 + 108, 235, y2)
arrow(420, y2 + 54, 445, y2 + 54)
arrow(815, y2 + 54, 840, y2 + 54)

# row 3
y3 = 574
box(50, y3, 570, 108, 'Risk: TTC / CPA (per track)', ['closing speed, nominal and pessimistic TTC (+2 sigma), closest point', 'of approach, collision course; NONE .. CRITICAL. Without a reliable', 'velocity only the distance grade is given (ttc_valid = false).'], GRN_F, GRN_S)
box(645, y3, 585, 108, 'Collision warning (operator display)', ['CLEAR / CAUTION / WARNING for objects coming at the sensor', '(collision course + TTC), with hysteresis.', 'Display only, not connected to vehicle control.'], GRN_F, GRN_S)
arrow(1035, y2 + 108, 335, y3)
arrow(620, y3 + 54, 645, y3 + 54)

# row 4: in-process services
y4 = 712
box(50, y4, 560, 72, 'Event recorder (record_events:=true, same process)', ['5 s before / after CRITICAL risk, data loss, ego loss: RAM pre-buffer,', 'bounded writer thread -> rosbag2 MCAP + metadata.json, disk quota, cooldown'], GRY_F, GRY_S, tsize=14)
box(635, y4, 595, 72, 'Health and timing', ['sensor health (frame gaps, valid returns, coverage), per-stage timing,', 'latency, CPU / RAM -> /processing_stats (1 Hz), /diagnostics'], GRY_F, GRY_S, tsize=14)

# outputs
yo = 835
box(40, yo, 590, 150, 'Topics  /helios/...', ['obstacles (ObstacleArray), tracks (TrackedObjectArray)', 'collision_warning (CollisionWarning)', 'processing_stats (ProcessingStats), recorder_events', 'obstacle_markers, track_markers, filtered / ground / obstacle points', ''], BLUE_F, BLUE_S)
box(655, yo, 585, 150, 'Consumers (optional)', ['RViz2: rviz/detection.rviz', 'demo_visualizer.py -> demo_markers (display only)', 'status_panel.py (PyQt5): rates, latency, health', 'helios_benchmark: offline / synthetic evaluation'], GRY_F, GRY_S)
A(f'<text x="52" y="{yo + 136}" font-size="12.5" font-weight="600" fill="{RED_S}">No cmd_vel / brake / throttle / steering output (checked by launch tests).</text>')
arrow(640, 800, 335, yo)
arrow(630, yo + 75, 655, yo + 75)
A('</svg>')
open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'assets', 'diagrams',
                  'architecture.svg'), 'w').write('\n'.join(out))
print('ok')
