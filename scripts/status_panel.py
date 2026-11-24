#!/usr/bin/env python3
"""Optional status panel for demos (PyQt5): live numbers from the perception stack's topics.

RViz2 has no screen-fixed text overlay without extra plugins, so the diagnostics (input / output
rate, latency, tracks, ego motion, sensor health, collision state) are shown in this separate
window. It only subscribes to `<ns>/processing_stats`, `<ns>/collision_warning` and
`<ns>/recorder_events`; values are exactly what the system publishes.

  ros2 run helios_obstacle_detection status_panel.py --ros-args -r __ns:=/helios \\
      -p x:=1440 -p y:=0 -p width:=480 -p height:=1080
"""

import json
import signal
import sys

import rclpy
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from helios_obstacle_detection.msg import CollisionWarning, ProcessingStats

EGO = {0: 'UNKNOWN', 1: 'STATIONARY', 2: 'MOVING', 3: 'UNRELIABLE'}
STATUS = {0: 'OK', 1: 'NO DATA', 2: 'OVERLOAD', 3: 'GROUND DEGRADED', 4: 'GROUND FAILED',
          5: 'INVALID INPUT', 6: 'SENSOR DEGRADED', 7: 'EGO UNKNOWN'}
WARN = {0: ('NO COLLISION RISK', '#2fbf55'), 1: ('CAUTION', '#ff9a0d'),
        2: ('COLLISION RISK', '#ff2020')}
BG, FG, DIM = '#16181e', '#eceff4', '#8a93a6'


class Panel(QtWidgets.QWidget):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Helios Perception - status')
        self.setStyleSheet(f'background:{BG}; color:{FG};')
        g = node.geom
        self.setGeometry(g['x'], g['y'], g['width'], g['height'])
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(24, 24, 24, 24)
        lay.setSpacing(10)
        title = QtWidgets.QLabel('HELIOS PERCEPTION')
        title.setFont(QtGui.QFont('Sans', 20, QtGui.QFont.Bold))
        sub = QtWidgets.QLabel('RoboSense Helios 32  |  LiDAR-only  |  perception only')
        sub.setStyleSheet(f'color:{DIM};')
        lay.addWidget(title)
        lay.addWidget(sub)
        self.warn = QtWidgets.QLabel('waiting for data')
        self.warn.setAlignment(QtCore.Qt.AlignCenter)
        self.warn.setFont(QtGui.QFont('Sans', 22, QtGui.QFont.Bold))
        self.warn.setMinimumHeight(90)
        lay.addSpacing(8)
        lay.addWidget(self.warn)
        self.warn_detail = QtWidgets.QLabel('')
        self.warn_detail.setAlignment(QtCore.Qt.AlignCenter)
        self.warn_detail.setStyleSheet(f'color:{DIM};')
        lay.addWidget(self.warn_detail)
        grid = QtWidgets.QGridLayout()
        grid.setVerticalSpacing(12)
        self.values = {}
        rows = ['System status', 'Input rate', 'Output rate', 'Latency rx->publish', 'Tracks',
                'Ego motion', 'Ground', 'CPU / RAM', 'Events recorded']
        for i, name in enumerate(rows):
            k = QtWidgets.QLabel(name)
            k.setStyleSheet(f'color:{DIM}; font-size:15px;')
            v = QtWidgets.QLabel('-')
            v.setStyleSheet('font-size:17px; font-weight:bold;')
            grid.addWidget(k, i, 0)
            grid.addWidget(v, i, 1)
            self.values[name] = v
        lay.addSpacing(16)
        lay.addLayout(grid)
        lay.addStretch(1)
        foot = QtWidgets.QLabel('Values as published on /processing_stats and '
                                '/collision_warning.\nNot connected to vehicle control.')
        foot.setStyleSheet(f'color:{DIM}; font-size:12px;')
        lay.addWidget(foot)
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.tick)
        timer.start(50)

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        s, w = self.node.stats, self.node.warning
        level = w.level if w is not None else None
        if level is None:
            self.warn.setText('waiting for data')
            self.warn.setStyleSheet(f'color:{DIM}; border:2px solid {DIM}; border-radius:8px;')
        else:
            text, col = WARN[level]
            self.warn.setText(text)
            self.warn.setStyleSheet(f'color:{BG if level else FG}; background:{col}; '
                                    'border-radius:8px;')
            self.warn_detail.setText(
                f'track {w.track_id}: TTC {w.ttc:.1f} s (worst {w.ttc_pessimistic:.1f} s), '
                f'{w.distance:.1f} m, {w.closing_speed:.1f} m/s' if level else '')
        if s is None:
            return
        v = self.values
        v['System status'].setText(STATUS.get(s.status, '?'))
        ok = s.status in (0, 3)   # OK / ground degraded (indoors)
        v['System status'].setStyleSheet('font-size:17px; font-weight:bold; color:%s;'
                                         % ('#2fbf55' if ok else '#ff9a0d'))
        v['Input rate'].setText(f'{s.input_fps:.1f} Hz')
        v['Output rate'].setText(f'{s.output_fps:.1f} Hz  (dropped {s.frames_dropped})')
        v['Latency rx->publish'].setText(f'{s.latency_rx_to_publish_mean_ms:.0f} ms mean, '
                                         f'{s.latency_rx_to_publish_p95_ms:.0f} ms p95')
        v['Tracks'].setText(f'{s.tracks_confirmed} confirmed, {s.tracks_moving} moving, '
                            f'{s.tracks_occluded} occluded')
        v['Ego motion'].setText(EGO.get(s.ego_state, '?'))
        v['Ground'].setText(s.ground_method)
        v['CPU / RAM'].setText(f'{s.cpu_percent:.0f} % (1 core), {s.rss_mb:.0f} MB')
        v['Events recorded'].setText(str(self.node.events) if self.node.events_seen else 'off')


class PanelNode(Node):

    def __init__(self):
        super().__init__('status_panel')
        self.geom = {k: self.declare_parameter(k, d).value
                     for k, d in (('x', 1440), ('y', 0), ('width', 466), ('height', 1080))}
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.stats = None
        self.warning = None
        self.events = 0
        self.events_seen = False
        self.create_subscription(ProcessingStats, 'processing_stats',
                                 lambda m: setattr(self, 'stats', m), qos)
        self.create_subscription(CollisionWarning, 'collision_warning',
                                 lambda m: setattr(self, 'warning', m), qos)
        self.create_subscription(
            String, 'recorder_events', self.on_event,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_event(self, m):
        self.events_seen = True
        try:
            if json.loads(m.data).get('state') == 'started':
                self.events += 1
        except ValueError:
            pass


def main():
    rclpy.init(args=sys.argv)
    node = PanelNode()
    app = QtWidgets.QApplication([a for a in sys.argv if not a.startswith('--ros-args')][:1])
    panel = Panel(node)
    panel.show()
    # Qt blocks Python signal handling while idle; the 50 ms timer of the panel lets the handler
    # run, which ends the event loop (ros2 launch stops processes with SIGINT, then SIGTERM)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    rc = app.exec_()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
