"""Unit tests for compute_node.sim_arena (#44 cont.)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from compute_node.sim_arena import SimArena


def test_default_construction():
    a = SimArena()
    assert a.width == 3.0 and a.height == 3.0
    assert len(a.balls) == 5
    assert all('grabbed' in b for b in a.balls)


def test_custom_dimensions():
    a = SimArena(width=4.0, height=2.5, ball_radius=0.05,
                 colours=('red', 'blue'))
    assert a.width == 4.0 and a.height == 2.5
    assert len(a.balls) == 2
    assert a.balls[0]['radius'] == 0.05


def test_zone_add_and_query():
    a = SimArena()
    z = a.add_zone(0.0, 0.0, 1.0, 1.0)
    assert z['id'] == 1
    assert a.point_in_zone(0.5, 0.5) is True
    assert a.point_in_zone(2.0, 2.0) is False


def test_zone_id_unique_after_remove():
    """ids are auto-incremented and never reused — frontend keeps them
    as React keys, so a removed-then-readded zone must not collide with
    a stale reference."""
    a = SimArena()
    z1 = a.add_zone(0, 0, 1, 1)
    a.add_zone(1, 1, 2, 2)
    assert a.remove_zone(z1['id']) is True
    z3 = a.add_zone(0, 0, 0.5, 0.5)
    assert z3['id'] == 3   # NOT 1


def test_zone_normalises_coordinates():
    """add_zone with (x1, y1) > (x2, y2) should normalise."""
    a = SimArena()
    z = a.add_zone(2.0, 2.0, 1.0, 1.0)
    assert z['x1'] == 1.0 and z['x2'] == 2.0


def test_zone_remove_unknown_returns_false():
    a = SimArena()
    a.add_zone(0, 0, 1, 1)
    assert a.remove_zone(999) is False


def test_clear_zones():
    a = SimArena()
    a.add_zone(0, 0, 1, 1)
    a.add_zone(1, 1, 2, 2)
    a.clear_zones()
    assert a.forbidden_zones == []
    # But counter is NOT reset — next zone gets id 3
    z = a.add_zone(0, 0, 0.5, 0.5)
    assert z['id'] == 3


def test_reset_keeps_zones():
    """reset() restores ball positions but zones survive — matches the
    contract the dashboard expects (user-drawn zones aren't ephemeral)."""
    a = SimArena()
    a.add_zone(0, 0, 1, 1)
    a.balls[0]['grabbed'] = True
    a.reset()
    assert a.balls[0]['grabbed'] is False
    assert len(a.forbidden_zones) == 1


def test_point_in_zone_inclusive_boundary():
    a = SimArena()
    a.add_zone(1.0, 1.0, 2.0, 2.0)
    # Boundary points should be inside (inclusive)
    assert a.point_in_zone(1.0, 1.5) is True
    assert a.point_in_zone(2.0, 2.0) is True
    # Outside
    assert a.point_in_zone(0.99, 1.5) is False
