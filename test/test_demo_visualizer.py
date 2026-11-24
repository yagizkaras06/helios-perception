"""Unit tests of the demo marker builder (scripts/demo_visualizer.py), no ROS graph needed.

Checks the presentation rules the demo relies on:
  * every MarkerArray starts with DELETEALL and every ADD marker has a finite lifetime
    (no stale boxes / labels when tracks disappear or the detector stops),
  * TEXT_VIEW_FACING markers are never blank (RViz / Ogre aborts on blank text),
  * STATIC / TENTATIVE / LOST tracks are not drawn, MOVING ones are,
  * untrusted estimates are labelled as such (UNRELIABLE, relative-only, n/a),
  * the collision banner follows the published warning level, in English and Turkish.
"""

import importlib.util
import math
import os
import random

import pytest
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from helios_obstacle_detection.msg import (CollisionWarning, ProcessingStats, TrackedObject,
                                           TrackedObjectArray)

_PATH = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'demo_visualizer.py')
_spec = importlib.util.spec_from_file_location('demo_visualizer', _PATH)
dv = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(dv)


def cfg(**kw):
    c = dict(dv.DEFAULTS)
    c.update(kw)
    return c


def header():
    h = Header()
    h.frame_id = 'rslidar'
    return h


def track(i=1, state=1, motion=2, vstat=2, x=5.0, y=1.0, speed=1.2, moved=2.0, **kw):
    o = TrackedObject()
    o.id = i
    o.track_state = state
    o.motion_state = motion
    o.velocity_status = vstat
    o.velocity_reliable = vstat == 2
    o.center.x, o.center.y, o.center.z = x, y, -0.05
    o.size.x, o.size.y, o.size.z = 0.6, 0.5, 1.7
    o.closest_distance = math.hypot(x, y) - 0.3
    o.speed = speed
    o.speed_sigma = 0.2
    o.heading = 0.5
    o.heading_valid = motion == 2 and vstat == 2
    o.max_displacement = moved
    o.ever_moved = motion == 2
    o.ttc = float('inf')
    o.ttc_pessimistic = float('inf')
    for k, v in kw.items():
        setattr(o, k, v)
    return o


def tracks(*objs):
    t = TrackedObjectArray()
    t.objects = list(objs)
    return t


def stats(learning=False):
    s = ProcessingStats()
    s.background_learning = learning
    return s


def warning(level, tid=7, ttc=2.0):
    w = CollisionWarning()
    w.level = level
    w.track_id = tid
    w.ttc = ttc
    w.ttc_pessimistic = ttc * 0.7
    w.distance = 3.0
    w.closing_speed = 1.5
    return w


def drawn_ids(arr, ns='boxes'):
    return sorted(m.id for m in arr.markers if m.ns == ns and m.action == Marker.ADD)


def texts(arr):
    return [m.text for m in arr.markers if m.type == Marker.TEXT_VIEW_FACING]


def test_starts_with_deleteall_and_all_markers_expire():
    arr = dv.build_markers(header(), tracks(track()), warning(1), stats(), cfg())
    assert arr.markers[0].action == Marker.DELETEALL
    for m in arr.markers[1:]:
        assert m.action == Marker.ADD
        life = m.lifetime.sec + 1e-9 * m.lifetime.nanosec
        assert 0.0 < life <= 1.0, (m.ns, life)


def test_empty_inputs_only_clear_and_static_layers():
    arr = dv.build_markers(header(), None, None, None, cfg())
    assert arr.markers[0].action == Marker.DELETEALL
    assert drawn_ids(arr) == []
    assert all(t.strip() for t in texts(arr))


@pytest.mark.parametrize('language', ['en', 'tr'])
def test_text_markers_are_never_blank(language):
    rnd = random.Random(3)
    for _ in range(300):
        objs = [track(i=k, state=rnd.choice([0, 1, 2, 3]), motion=rnd.choice([0, 1, 2]),
                      vstat=rnd.choice([0, 1, 2]), x=rnd.uniform(-20, 20), y=rnd.uniform(-20, 20),
                      speed=rnd.uniform(0, 3), moved=rnd.uniform(0, 3),
                      relative_motion_only=rnd.random() < 0.2, predicted=rnd.random() < 0.3,
                      ttc_valid=rnd.random() < 0.5, risk_level=rnd.choice([0, 1, 2, 3, 4]),
                      ttc=rnd.choice([float('inf'), rnd.uniform(0.1, 9)]))
                for k in range(rnd.randint(0, 6))]
        w = rnd.choice([None, warning(0), warning(1), warning(2, ttc=float('inf'))])
        arr = dv.build_markers(header(), tracks(*objs), w, stats(rnd.random() < 0.3),
                               cfg(language=language, show_hud=rnd.random() < 0.5))
        for t in texts(arr):
            assert t and t.strip(), 'blank text marker'


def test_static_tentative_and_lost_tracks_are_not_drawn():
    objs = [track(i=1, motion=2), track(i=2, motion=1), track(i=3, state=0),
            track(i=4, state=3)]
    arr = dv.build_markers(header(), tracks(*objs), None, stats(), cfg())
    assert drawn_ids(arr) == [1]
    box = next(m for m in arr.markers if m.ns == 'boxes')
    assert (box.color.r, box.color.g, box.color.b) == pytest.approx(dv.COL_MOVING)


def test_after_learning_only_objects_that_went_somewhere():
    still_unknown = track(i=1, motion=0, moved=0.1)
    walker = track(i=2, motion=2, moved=1.5)
    arr = dv.build_markers(header(), tracks(still_unknown, walker), None, stats(False), cfg())
    assert drawn_ids(arr) == [2]
    arr = dv.build_markers(header(), tracks(still_unknown, walker), None, stats(True), cfg())
    assert drawn_ids(arr) == [1, 2]


def test_uncertain_estimates_are_labelled():
    lab = dv.label_lines(track(vstat=1))
    assert 'UNRELIABLE' in lab[1]
    assert 'v:relative-only' in dv.label_lines(track(relative_motion_only=True))[1]
    assert 'v:n/a' in dv.label_lines(track(vstat=0))[1]
    reliable = dv.label_lines(track(vstat=2, speed=1.2))
    assert '1.2+-0.2m/s' in reliable[1] and 'deg' in reliable[1]
    # RViz draws spaces several characters wide: labels use the separator instead
    for line in reliable + lab:
        assert ' ' not in line


def test_ttc_only_with_valid_velocity():
    t = track(ttc_valid=False, risk_level=4, ttc=1.0)
    assert len(dv.label_lines(t)) == 2
    t = track(ttc_valid=True, risk_level=3, ttc=1.0, ttc_pessimistic=0.7)
    assert dv.label_lines(t)[2] == 'TTC|1.0s|worst|0.7s'


@pytest.mark.parametrize('level,en,tr', [(0, 'NO COLLISION RISK', 'CARPISMA YOK'),
                                         (1, 'CAUTION', 'DIKKAT'),
                                         (2, 'COLLISION RISK', 'CARPISMA RISKI')])
def test_banner_follows_warning_level(level, en, tr):
    for language, word in (('en', en), ('tr', tr)):
        arr = dv.build_markers(header(), tracks(), warning(level), stats(), cfg(language=language))
        banner = [m.text for m in arr.markers if m.ns == 'collision_warning' and m.id == 0]
        assert len(banner) == 1 and word in banner[0]


def test_heading_arrow_only_for_reliable_moving_tracks():
    objs = [track(i=1, motion=2, vstat=2), track(i=2, motion=2, vstat=1, moved=2.0)]
    arr = dv.build_markers(header(), tracks(*objs), None, stats(), cfg())
    assert drawn_ids(arr, 'heading') == [1]


def test_banner_can_be_left_to_the_status_panel():
    arr = dv.build_markers(header(), tracks(), warning(2), stats(), cfg(show_banner=False))
    assert not [m for m in arr.markers if m.ns == 'collision_warning']
    foot = next(m for m in arr.markers if m.ns == 'sensor')
    assert (foot.color.r, foot.color.g, foot.color.b) == pytest.approx(dv.COL_WARN[2])


def test_stopped_objects_are_drawn_only_while_their_motion_is_recent():
    recent = track(i=1, motion=0, moved=2.0, ever_moved=True, time_since_moving_s=2.0)
    old = track(i=2, motion=0, moved=9.0, ever_moved=True, time_since_moving_s=40.0)
    arr = dv.build_markers(header(), tracks(recent, old), None, stats(False), cfg())
    assert drawn_ids(arr) == [1]
