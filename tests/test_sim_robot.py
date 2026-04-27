"""Unit tests for compute_node.sim_robot.SimRobot (#44 phase 3)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from compute_node.sim_arena import SimArena
from compute_node.sim_robot import SimRobot


def _arena(w: float = 3.0, h: float = 3.0) -> SimArena:
    return SimArena(width=w, height=h, ball_radius=0.02, colours=())


def test_initial_state_centred():
    a = _arena()
    r = SimRobot(a)
    assert r.x == pytest.approx(1.5)
    assert r.y == pytest.approx(1.5)
    assert r.theta == 0.0
    assert r.v_linear == 0.0
    assert r.v_angular == 0.0


def test_velocity_clamping_to_max():
    r = SimRobot(_arena(), max_linear=0.30, max_angular=2.0)
    r.set_velocity(10.0, 99.0)
    assert r.v_linear == pytest.approx(0.30)
    assert r.v_angular == pytest.approx(2.0)
    r.set_velocity(-10.0, -99.0)
    assert r.v_linear == pytest.approx(-0.30)
    assert r.v_angular == pytest.approx(-2.0)


def test_tick_integrates_forward_motion():
    r = SimRobot(_arena())
    r.set_velocity(0.2, 0.0)   # 0.2 m/s along +x
    for _ in range(10):
        r.tick(0.05)            # 10 * 0.05s = 0.5s
    # Should have moved ~0.1 m in +x (0.2 m/s * 0.5s).
    assert r.x == pytest.approx(1.5 + 0.1, abs=0.01)
    assert r.y == pytest.approx(1.5, abs=0.01)


def test_tick_integrates_yaw():
    r = SimRobot(_arena())
    r.set_velocity(0.0, math.pi / 2)   # 90 deg/s
    for _ in range(20):
        r.tick(0.05)            # 1.0s total
    # Yaw should be ~ pi/2 (or near boundary)
    assert abs(r.theta) > 1.5
    assert abs(r.theta) < math.pi


def test_theta_normalised_to_pi_range():
    r = SimRobot(_arena())
    r.set_velocity(0.0, math.pi)   # spin
    for _ in range(100):
        r.tick(0.05)
        assert -math.pi <= r.theta <= math.pi


def test_wall_clamp_blocks_overshoot():
    a = _arena()
    r = SimRobot(a, robot_radius=0.12)
    # Drive hard in +x for a long time
    r.x = 1.5
    r.theta = 0.0
    r.set_velocity(0.30, 0.0)
    for _ in range(200):     # plenty of time to leave arena
        r.tick(0.05)
    # Should be clamped to (arena.width - robot_radius)
    assert r.x == pytest.approx(a.width - 0.12, abs=0.001)


def test_zone_reject_snaps_back_and_zeros_velocity():
    a = _arena()
    a.add_zone(2.0, 1.0, 2.5, 2.0)   # block in front of centre
    r = SimRobot(a, robot_radius=0.12)
    r.x, r.y, r.theta = 1.5, 1.5, 0.0
    r.set_velocity(0.30, 0.0)
    # Need ~60 ticks at 0.015 m/tick to actually reach the inflated zone
    # edge at x = 1.88. With 60 ticks the robot would otherwise reach 2.4,
    # but the reject snaps it back to just before the inflated edge.
    for _ in range(60):
        r.tick(0.05)
    margin = 0.12
    inflated_west = 2.0 - margin
    assert r.x <= inflated_west + 0.01
    # After zone reject fires v_linear should have been zeroed; the few
    # ticks after the reject all see v_linear=0 so it stays 0.
    assert r.v_linear == 0.0


def test_collision_guard_inactive_when_no_provider():
    """No range_provider — collision guard is silently disabled."""
    r = SimRobot(_arena())
    r.collision_guard = True
    r.set_velocity(0.30, 0.0)
    assert r.v_linear == pytest.approx(0.30)


def test_collision_guard_full_stop_under_close_obstacle():
    class FakeRange:
        range_m = 0.15
    r = SimRobot(_arena(), collision_stop=0.20, collision_slow=0.40)
    r.attach_range_provider(lambda: FakeRange())
    r.collision_guard = True
    r.set_velocity(0.30, 0.0)
    assert r.v_linear == 0.0


def test_collision_guard_partial_slow():
    class FakeRange:
        range_m = 0.30   # halfway between stop=0.20 and slow=0.40
    r = SimRobot(_arena(), collision_stop=0.20, collision_slow=0.40)
    r.attach_range_provider(lambda: FakeRange())
    r.collision_guard = True
    r.set_velocity(0.30, 0.0)
    # Linear factor = (0.30 - 0.20) / (0.40 - 0.20) = 0.5; clamped to max
    assert r.v_linear == pytest.approx(0.15, abs=0.01)


def test_collision_guard_inactive_for_reverse():
    """Backing away from an obstacle shouldn't be blocked."""
    class FakeRange:
        range_m = 0.10
    r = SimRobot(_arena())
    r.attach_range_provider(lambda: FakeRange())
    r.collision_guard = True
    r.set_velocity(-0.20, 0.0)
    assert r.v_linear == pytest.approx(-0.20)


def test_stop_zeroes_both_velocities():
    r = SimRobot(_arena())
    r.set_velocity(0.20, 1.0)
    r.stop()
    assert r.v_linear == 0.0
    assert r.v_angular == 0.0
