#!/usr/bin/env python3
"""Presentation layer for Helios Perception (RViz2 demo profile).

Builds a clean MarkerArray (`<ns>/demo_markers`) from what the perception stack PUBLISHES:
`<ns>/tracks`, `<ns>/collision_warning` and `<ns>/processing_stats`. It runs no perception code,
starts no detector and changes nothing in the running system - it only draws:

  * 3D boxes coloured by motion state (MOVING red, UNKNOWN yellow); predicted (OCCLUDED) tracks
    dashed and translucent; translucent fill so boxes stay visible on dense point clouds
  * compact labels: track id, motion state, lifecycle, distance, speed +- sigma, heading;
    estimates that are not trusted are labelled UNRELIABLE (never shown as certain)
  * heading arrows for reliable MOVING tracks, short trails
  * range rings (5 / 10 / 15 / 20 m), sensor footprint
  * the collision warning (CLEAR / CAUTION / WARNING) as a banner placed away from the sensor
  * optional diagnostics HUD (input / output rate, latency, tracks, ego motion, sensor health)

Visibility policy is the detector's: after the static background is learned only objects that
moved (by the same displacement thresholds) are drawn; STATIC objects are not drawn.
Text markers are never empty (RViz / Ogre aborts on blank text).

  ros2 run helios_obstacle_detection demo_visualizer.py --ros-args -r __ns:=/helios
"""

import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from helios_obstacle_detection.msg import CollisionWarning, ProcessingStats, TrackedObjectArray

# ---------------------------------------------------------------------------------- appearance
COL_MOVING = (1.00, 0.18, 0.12)
COL_UNKNOWN = (1.00, 0.82, 0.10)
COL_STATIC = (0.50, 0.60, 0.90)
COL_TEXT = (0.95, 0.95, 0.95)
COL_DIM = (0.62, 0.66, 0.74)
COL_WARN = {0: (0.25, 0.90, 0.35), 1: (1.00, 0.60, 0.05), 2: (1.00, 0.12, 0.12)}
WARN_TEXT = {
    'en': {0: 'NO COLLISION RISK', 1: 'CAUTION: APPROACHING OBJECT', 2: 'COLLISION RISK'},
    'tr': {0: 'CARPISMA YOK', 1: 'DIKKAT: YAKLASAN NESNE', 2: 'CARPISMA RISKI VAR'},
}
MOTION = {0: 'UNKNOWN', 1: 'STATIC', 2: 'MOVING'}
SEP = '|'   # token separator in 3D labels (see label_lines)
LIFECYCLE = {0: 'TENTATIVE', 1: 'CONFIRMED', 2: 'OCCLUDED', 3: 'LOST'}
EGO = {0: 'UNKNOWN', 1: 'STATIONARY', 2: 'MOVING', 3: 'UNRELIABLE'}
STATUS = {0: 'OK', 1: 'NO DATA', 2: 'OVERLOAD', 3: 'GROUND DEGRADED', 4: 'GROUND FAILED',
          5: 'INVALID INPUT', 6: 'SENSOR DEGRADED', 7: 'EGO UNKNOWN'}

DEFAULTS = {
    'language': 'en',
    'label_scale': 0.24,          # [m] text height
    'box_line_width': 0.06,       # [m]
    'fill_alpha': 0.18,           # translucent box fill (0: off)
    'show_unknown': True,
    'show_trails': True,
    'show_range_rings': True,
    'show_hud': False,            # diagnostics text in the 3D view (also in the status panel)
    'show_banner': True,          # collision banner in the 3D view (off when the panel shows it)
    'only_dynamic_after_learning': True,
    'min_moving_displacement': 0.5,
    'min_dynamic_displacement': 1.0,
    'recent_motion_s': 10.0,      # stopped objects are drawn this long after they last moved
    'arrow_time_s': 1.0,
    # banner / HUD at the lower edge of the demo view (behind the sensor), away from the area in
    # front of it; with another camera set them via parameters
    'banner_x': -3.3, 'banner_y': 0.0, 'banner_scale': 0.55,
    'hud_x': -4.2, 'hud_y': 0.0, 'hud_scale': 0.24,
    'ego_radius': 0.5,
    'lifetime_s': 0.35,
}


def _pt(x, y, z):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def _marker(header, ns, mid, mtype, lifetime_s, rgb=(1.0, 1.0, 1.0), alpha=1.0):
    m = Marker()
    m.header = header
    m.ns = ns
    m.id = int(mid) & 0x7fffffff
    m.type = mtype
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.lifetime = Duration(sec=int(lifetime_s), nanosec=int((lifetime_s % 1.0) * 1e9))
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = float(alpha)
    return m


def _text(header, ns, mid, text, x, y, z, scale, rgb, lifetime_s, alpha=1.0):
    """TEXT_VIEW_FACING marker; returns None for blank text (RViz / Ogre aborts on it)."""
    if not text or not text.strip():
        return None
    m = _marker(header, ns, mid, Marker.TEXT_VIEW_FACING, lifetime_s, rgb, alpha)
    m.pose.position = _pt(x, y, z)
    m.scale.z = float(scale)
    m.text = text
    return m


def _box_corners(o):
    c, s, yaw = o.center, o.size, o.yaw
    cy, sy = math.cos(yaw), math.sin(yaw)
    hx, hy, hz = 0.5 * s.x, 0.5 * s.y, 0.5 * s.z
    pts = []
    for z in (c.z - hz, c.z + hz):
        for dx, dy in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy)):
            pts.append(_pt(c.x + cy * dx - sy * dy, c.y + sy * dx + cy * dy, z))
    return pts


def _box_edges(o, dashed=False):
    p = _box_corners(o)
    idx = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
           (0, 4), (1, 5), (2, 6), (3, 7)]
    out = []
    for a, b in idx:
        if not dashed:
            out += [p[a], p[b]]
            continue
        for q in (0, 2):   # 1st and 3rd quarter of every edge
            f0, f1 = q / 4.0, (q + 1) / 4.0
            out.append(_pt(p[a].x + f0 * (p[b].x - p[a].x), p[a].y + f0 * (p[b].y - p[a].y),
                           p[a].z + f0 * (p[b].z - p[a].z)))
            out.append(_pt(p[a].x + f1 * (p[b].x - p[a].x), p[a].y + f1 * (p[b].y - p[a].y),
                           p[a].z + f1 * (p[b].z - p[a].z)))
    return out


def visible(o, background_learning, cfg):
    """Detector's visualisation policy (tracks / risk are not affected)."""
    if o.track_state in (0, 3):           # TENTATIVE / LOST
        return False
    if o.motion_state == 1:               # STATIC: hidden (stays in /tracks and the risk)
        return False
    if o.motion_state == 0 and not cfg['show_unknown']:
        return False
    if cfg['only_dynamic_after_learning'] and not background_learning:
        moving_now = o.motion_state == 2 and o.max_displacement >= cfg['min_moving_displacement']
        moved_before = (o.ever_moved and o.max_displacement >= cfg['min_dynamic_displacement']
                        and (cfg['recent_motion_s'] <= 0
                             or o.time_since_moving_s < cfg['recent_motion_s']))
        return moving_now or moved_before
    return True


def label_lines(o):
    """Short ASCII lines; only published values, uncertain ones marked.

    RViz draws a space in TEXT_VIEW_FACING markers several characters wide, so tokens are joined
    with SEP instead (e.g. "#42|MOVING", "3.4m|1.2+-0.3m/s|45deg").
    """
    life = '' if o.track_state == 1 else SEP + LIFECYCLE.get(o.track_state, '?')
    line1 = f'#{o.id}{SEP}{MOTION.get(o.motion_state, "?")}{life}'
    dist = f'{o.closest_distance:.1f}m'
    if o.relative_motion_only:
        vel = 'v:relative-only'
    elif o.velocity_status == 2:          # RELIABLE
        vel = f'{o.speed:.1f}+-{o.speed_sigma:.1f}m/s'
        if o.heading_valid:
            vel += f'{SEP}{math.degrees(o.heading):.0f}deg'
    elif o.velocity_status == 1:          # UNRELIABLE: estimate exists but is not trusted
        vel = f'UNRELIABLE{SEP}{o.speed:.1f}+-{o.speed_sigma:.1f}m/s'
    else:
        vel = 'v:n/a'
    lines = [line1, f'{dist}{SEP}{vel}']
    # TTC of an approaching object; the operator decision (collision course + TTC) is the banner,
    # so the per-track risk grade is not repeated here (it would read as a second verdict)
    if o.ttc_valid and o.risk_level >= 2 and math.isfinite(o.ttc):
        lines.append(f'TTC{SEP}{o.ttc:.1f}s{SEP}worst{SEP}{o.ttc_pessimistic:.1f}s')
    return lines


def build_markers(header, tracks, warning, stats, cfg):
    """MarkerArray for one tracks message (warning / stats may be None)."""
    lt = cfg['lifetime_s']
    arr = MarkerArray()
    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)
    learning = bool(stats.background_learning) if stats is not None else False
    shown = 0
    for o in (tracks.objects if tracks is not None else []):
        if not visible(o, learning, cfg):
            continue
        shown += 1
        rgb = {2: COL_MOVING, 0: COL_UNKNOWN}.get(o.motion_state, COL_STATIC)
        alpha = 0.45 if o.predicted else 1.0
        edges = _marker(header, 'boxes', o.id, Marker.LINE_LIST, lt, rgb, alpha)
        edges.scale.x = cfg['box_line_width'] * (0.7 if o.predicted else 1.0)
        edges.points = _box_edges(o, dashed=o.predicted)
        arr.markers.append(edges)
        if cfg['fill_alpha'] > 0.0 and not o.predicted:
            fill = _marker(header, 'box_fill', o.id, Marker.CUBE, lt, rgb, cfg['fill_alpha'])
            fill.pose.position = _pt(o.center.x, o.center.y, o.center.z)
            fill.pose.orientation.z = math.sin(0.5 * o.yaw)
            fill.pose.orientation.w = math.cos(0.5 * o.yaw)
            fill.scale.x, fill.scale.y = max(o.size.x, 0.05), max(o.size.y, 0.05)
            fill.scale.z = max(o.size.z, 0.05)
            arr.markers.append(fill)
        top = o.center.z + 0.5 * o.size.z
        lab = _text(header, 'labels', o.id, '\n'.join(label_lines(o)), o.center.x, o.center.y,
                    top + 0.55 + 0.5 * cfg['label_scale'], cfg['label_scale'], COL_TEXT, lt,
                    0.6 if o.predicted else 1.0)
        if lab is not None:
            arr.markers.append(lab)
        if o.velocity_status == 2 and o.motion_state == 2 and o.heading_valid and o.speed > 0.2:
            ar = _marker(header, 'heading', o.id, Marker.ARROW, lt, (0.25, 0.95, 1.0), 0.95)
            length = max(0.6, o.speed * cfg['arrow_time_s'])
            ar.points = [_pt(o.center.x, o.center.y, top + 0.1),
                         _pt(o.center.x + length * math.cos(o.heading),
                             o.center.y + length * math.sin(o.heading), top + 0.1)]
            ar.scale.x, ar.scale.y, ar.scale.z = 0.10, 0.24, 0.26
            arr.markers.append(ar)
    # ---- range rings + sensor -------------------------------------------------------------
    if cfg['show_range_rings']:
        for k, r in enumerate((5.0, 10.0, 15.0, 20.0)):
            ring = _marker(header, 'range_rings', k, Marker.LINE_STRIP, lt, COL_DIM, 0.35)
            ring.scale.x = 0.035
            ring.points = [_pt(r * math.cos(a * math.pi / 60.0), r * math.sin(a * math.pi / 60.0),
                               -0.88) for a in range(121)]
            arr.markers.append(ring)
            lbl = _text(header, 'range_rings', 100 + k, f'{r:.0f}m', r * 0.7071, -r * 0.7071,
                        -0.8, 0.42, COL_DIM, lt, 0.85)
            arr.markers.append(lbl)
    level = warning.level if warning is not None else 0
    foot = _marker(header, 'sensor', 0, Marker.CYLINDER, lt, COL_WARN[level],
                   0.35 if level == 0 else 0.75)
    foot.pose.position = _pt(0.0, 0.0, -0.2)
    foot.scale.x = foot.scale.y = 2.0 * cfg['ego_radius']
    foot.scale.z = 0.05
    arr.markers.append(foot)
    # ---- collision warning ----------------------------------------------------------------
    if cfg['show_banner']:
        _banner(arr, header, warning, level, cfg, lt)
    # ---- optional diagnostics HUD -----------------------------------------------------------
    if cfg['show_hud'] and stats is not None:
        hud = (f'in {stats.input_fps:.1f}Hz out {stats.output_fps:.1f}Hz latency-p95 '
               f'{stats.latency_rx_to_publish_p95_ms:.0f}ms\ntracks {stats.tracks_confirmed} '
               f'moving {stats.tracks_moving} shown {shown} ego {EGO.get(stats.ego_state, "?")} '
               f'status {STATUS.get(stats.status, "?").replace(" ", "-")}')
        arr.markers.append(_text(header, 'hud', 0, hud, cfg['hud_x'], cfg['hud_y'], 2.0,
                                 cfg['hud_scale'], COL_TEXT, lt))
    arr.markers = [m for m in arr.markers if m is not None]
    return arr


def _banner(arr, header, warning, level, cfg, lt):
    texts = WARN_TEXT.get(cfg['language'], WARN_TEXT['en'])
    head = texts[level]
    if level == 2:
        head = '!! ' + head + ' !!'
    arr.markers.append(_text(header, 'collision_warning', 0, head, cfg['banner_x'], cfg['banner_y'],
                             2.0, cfg['banner_scale'] * (1.0 if level else 0.45), COL_WARN[level],
                             lt, 1.0 if level else 0.6))
    if level and warning is not None and warning.track_id:
        if math.isfinite(warning.ttc) and math.isfinite(warning.ttc_pessimistic):
            det = SEP.join([f'#{warning.track_id}', f'TTC{SEP}{warning.ttc:.1f}s',
                            f'worst{SEP}{warning.ttc_pessimistic:.1f}s',
                            f'{warning.distance:.1f}m', f'{warning.closing_speed:.1f}m/s'])
        else:
            det = SEP.join([f'#{warning.track_id}', f'{warning.distance:.1f}m',
                            f'{warning.closing_speed:.1f}m/s'])
        m = _text(header, 'collision_warning', 1, det, cfg['banner_x'] - 1.1 * cfg['banner_scale'],
                  cfg['banner_y'], 2.0, 0.55 * cfg['banner_scale'], COL_WARN[level], lt)
        if m is not None:
            arr.markers.append(m)


class DemoVisualizer(Node):

    def __init__(self):
        super().__init__('demo_visualizer')
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()}
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.warning = None
        self.stats = None
        self.pub = self.create_publisher(MarkerArray, 'demo_markers', qos)
        self.create_subscription(CollisionWarning, 'collision_warning', self.on_warning, qos)
        self.create_subscription(ProcessingStats, 'processing_stats', self.on_stats, qos)
        self.create_subscription(TrackedObjectArray, 'tracks', self.on_tracks, qos)
        self.get_logger().info(
            f'demo markers on {self.resolve_topic_name("demo_markers")} (display only)')

    def on_warning(self, m):
        self.warning = m

    def on_stats(self, m):
        self.stats = m
        if m.status == 1:   # NO DATA: the detector stops publishing tracks -> clear the view
            clear = Marker()
            clear.header.frame_id = 'rslidar'
            clear.action = Marker.DELETEALL
            arr = MarkerArray()
            arr.markers.append(clear)
            self.pub.publish(arr)

    def on_tracks(self, m):
        self.pub.publish(build_markers(m.header, m, self.warning, self.stats, self.cfg))


def main():
    rclpy.init()
    node = DemoVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
