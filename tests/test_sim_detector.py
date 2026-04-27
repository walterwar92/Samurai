"""Unit tests for compute_node.sim_detector.SimDetector (#44 phase 6)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# numpy / cv2 gated — skip cleanly without compute deps installed.
np = pytest.importorskip('numpy')
cv2 = pytest.importorskip('cv2')

from compute_node.sim_arena import SimArena
from compute_node.sim_robot import SimRobot
from compute_node.sim_detector import SimDetector


def _scene():
    a = SimArena(width=3.0, height=3.0, ball_radius=0.02, colours=())
    r = SimRobot(a)
    d = SimDetector()
    return a, r, d


def test_initial_state_empty():
    _, _, d = _scene()
    assert d.detections == []
    assert d.annotated_frame is None


def test_detects_ball_in_front():
    """Robot at (1.5, 1.5) looking +x; ball at (2.0, 1.5) → 0.5 m straight ahead."""
    a, r, d = _scene()
    a.balls.append({'x': 2.0, 'y': 1.5, 'colour': 'red',
                    'radius': 0.02, 'grabbed': False})
    d.update(r, a)
    assert any(det['colour'] == 'red' for det in d.detections)
    red = next(det for det in d.detections if det['colour'] == 'red')
    assert red['distance'] == pytest.approx(0.5, abs=0.01)
    assert red['class'] == 'sports ball'
    assert red['conf'] > 0.5


def test_skips_grabbed_balls():
    a, r, d = _scene()
    a.balls.append({'x': 2.0, 'y': 1.5, 'colour': 'red',
                    'radius': 0.02, 'grabbed': True})
    d.update(r, a)
    assert all(det['colour'] != 'red' for det in d.detections)


def test_skips_balls_behind_robot():
    a, r, d = _scene()
    # Robot at (1.5, 1.5) looking +x. Ball at (1.0, 1.5) is behind.
    a.balls.append({'x': 1.0, 'y': 1.5, 'colour': 'red',
                    'radius': 0.02, 'grabbed': False})
    d.update(r, a)
    assert all(det['colour'] != 'red' for det in d.detections)


def test_skips_balls_outside_fov():
    """Cam FOV is 60° (±30° from heading). A ball 90° to the side falls out."""
    a, r, d = _scene()
    # Robot at (1.5, 1.5) looking +x. Place ball directly to the +y side.
    a.balls.append({'x': 1.5, 'y': 2.5, 'colour': 'red',
                    'radius': 0.02, 'grabbed': False})
    d.update(r, a)
    assert all(det['colour'] != 'red' for det in d.detections)


def test_skips_balls_beyond_max_range():
    a, r, d = _scene()
    # Far ball at the edge of the 3 m max — but actually need >3 m.
    # Use a custom detector with smaller max_range to exercise the threshold
    # without enlarging the arena.
    d2 = SimDetector(max_range=0.4)
    a.balls.append({'x': 2.5, 'y': 1.5, 'colour': 'red',
                    'radius': 0.02, 'grabbed': False})
    d2.update(r, a)
    # Ball is 1.0 m away → beyond 0.4 m max_range → no detection
    assert d2.detections == []


def test_confidence_decreases_with_distance():
    a, r, d = _scene()
    a.balls.append({'x': 1.7, 'y': 1.5, 'colour': 'red',
                    'radius': 0.02, 'grabbed': False})  # near
    a.balls.append({'x': 2.8, 'y': 1.5, 'colour': 'blue',
                    'radius': 0.02, 'grabbed': False})  # far
    d.update(r, a)
    near = next(det for det in d.detections if det['colour'] == 'red')
    far = next(det for det in d.detections if det['colour'] == 'blue')
    assert near['conf'] > far['conf']


def test_annotated_frame_dimensions():
    """Frame should have shape (CAM_H, CAM_W, 3)."""
    a, r, d = _scene()
    d.update(r, a)
    assert d.annotated_frame is not None
    assert d.annotated_frame.shape == (480, 640, 3)
    assert d.annotated_frame.dtype == np.uint8


def test_get_closest_detection_filtered_by_colour():
    a, r, d = _scene()
    a.balls.extend([
        {'x': 1.7, 'y': 1.5, 'colour': 'blue', 'radius': 0.02, 'grabbed': False},
        {'x': 2.0, 'y': 1.5, 'colour': 'red',  'radius': 0.02, 'grabbed': False},
        {'x': 2.5, 'y': 1.5, 'colour': 'red',  'radius': 0.02, 'grabbed': False},
    ])
    d.update(r, a)
    red = d.get_closest_detection('red')
    assert red is not None
    assert red['colour'] == 'red'
    # 0.5 < 1.0 so the closer red ball wins
    assert red['distance'] < 0.7


def test_get_closest_detection_no_filter():
    a, r, d = _scene()
    a.balls.append({'x': 1.7, 'y': 1.5, 'colour': 'blue',
                    'radius': 0.02, 'grabbed': False})
    a.balls.append({'x': 2.5, 'y': 1.5, 'colour': 'red',
                    'radius': 0.02, 'grabbed': False})
    d.update(r, a)
    closest = d.get_closest_detection()
    assert closest is not None
    assert closest['colour'] == 'blue'    # 0.2 m < 1.0 m


def test_get_closest_detection_no_matches():
    a, r, d = _scene()
    d.update(r, a)
    assert d.get_closest_detection('purple') is None


def test_robot_facing_north_sees_north_ball():
    """Sanity check: rotate robot 90° and verify FOV pivots correctly."""
    a, r, d = _scene()
    r.theta = math.pi / 2     # facing +y
    a.balls.append({'x': 1.5, 'y': 2.5, 'colour': 'red',
                    'radius': 0.02, 'grabbed': False})
    d.update(r, a)
    assert any(det['colour'] == 'red' for det in d.detections)
