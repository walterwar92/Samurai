"""Unit tests for compute_node.sim_renderer.MapRenderer (#44 phase 5)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Skip if numpy/cv2 not available — keeps `pytest tests/` clean on a fresh
# clone that hasn't installed compute deps yet.
np = pytest.importorskip('numpy')
cv2 = pytest.importorskip('cv2')

from compute_node.sim_arena import SimArena
from compute_node.sim_robot import SimRobot
from compute_node.sim_renderer import MapRenderer


def _scene(scale: int = 100):
    a = SimArena(width=3.0, height=3.0, ball_radius=0.02, colours=())
    r = SimRobot(a)
    mr = MapRenderer(a, scale=scale)
    return a, r, mr


def test_render_returns_valid_png():
    _, r, mr = _scene()
    png = mr.render(r)
    # PNG signature
    assert png[:8] == b'\x89PNG\r\n\x1a\n'


def test_canvas_dimensions_follow_arena_and_scale():
    a, _, mr = _scene(scale=50)
    assert mr.w == int(a.width * 50)
    assert mr.h == int(a.height * 50)


def test_get_map_info_resolution_inverse_of_scale():
    _, _, mr = _scene(scale=80)
    info = mr.get_map_info()
    assert info['resolution'] == pytest.approx(1.0 / 80, rel=1e-9)
    assert info['origin_x'] == 0.0
    assert info['origin_y'] == 0.0


def test_render_with_zones_produces_red_pixels():
    """Drawing a forbidden zone leaves visibly-different pixels in the
    rendered image — encoding-decoded comparison is enough."""
    a, r, mr = _scene()
    a.add_zone(1.0, 1.0, 2.0, 2.0)
    png = mr.render(r)
    img = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_COLOR)
    # Sample a pixel in the middle of the zone (centre of arena)
    cy, cx = mr.h // 2, mr.w // 2
    pixel = img[cy, cx]
    # Should NOT be the empty-arena floor colour (240, 240, 240)
    assert tuple(int(c) for c in pixel) != (240, 240, 240)


def test_render_planned_path_overlay_changes_image():
    """Planned-path overlay should alter pixels along the route."""
    a, r, mr = _scene()
    plain = mr.render(r)
    with_path = mr.render(r, planned_path=[(0.5, 0.5), (2.5, 2.5)])
    assert plain != with_path


def test_render_scan_points_overlay():
    a, r, mr = _scene()
    plain = mr.render(r)
    with_scan = mr.render(r, scan_points=[(1.5, 1.5), (1.6, 1.5), (1.7, 1.5)])
    assert plain != with_scan


def test_grabbed_balls_not_rendered():
    """Grabbing a ball must remove it from the picture. Placed away from
    the robot so the robot's body doesn't overdraw the ball position."""
    a, r, mr = _scene()
    a.balls.append({'x': 0.5, 'y': 0.5, 'colour': 'red',
                    'radius': 0.05, 'grabbed': False})
    visible = mr.render(r)
    a.balls[-1]['grabbed'] = True
    grabbed = mr.render(r)
    assert visible != grabbed


def test_robot_arrow_orientation_changes_pixels():
    """Different headings produce different images (arrow + FOV rotate)."""
    _, r, mr = _scene()
    r.theta = 0.0
    east = mr.render(r)
    r.theta = math.pi / 2
    north = mr.render(r)
    assert east != north


def test_no_balls_no_zones_renders_blank_floor():
    """Empty world: just walls, grid, robot. Should still produce a valid PNG."""
    a = SimArena(width=2.0, height=2.0, ball_radius=0.02, colours=())
    a.balls.clear()      # remove default-spawn balls
    r = SimRobot(a)
    mr = MapRenderer(a, scale=50)
    png = mr.render(r)
    assert png[:4] == b'\x89PNG'
