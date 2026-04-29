"""Unit tests for compute_node.sim_sensors.SimSensors (#44 phase 4)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from compute_node.sim_arena import SimArena
from compute_node.sim_robot import SimRobot
from compute_node.sim_sensors import SimSensors


def _scene(theta: float = 0.0):
    a = SimArena(width=3.0, height=3.0, ball_radius=0.02, colours=())
    r = SimRobot(a)
    r.theta = theta
    s = SimSensors(inject_noise=False)
    return a, r, s


def test_initial_range_is_max():
    _, _, s = _scene()
    assert s.range_m == pytest.approx(2.0)


def test_facing_east_wall_returns_distance_to_wall():
    a, r, s = _scene(theta=0.0)        # facing +x
    s.update(r, a)
    # Robot at (1.5, 1.5), east wall at x=3.0 → 1.5 m
    assert s.range_m == pytest.approx(1.5, abs=0.001)


def test_facing_west_wall():
    a, r, s = _scene(theta=math.pi)
    s.update(r, a)
    # Robot at (1.5, 1.5), west wall at x=0 → 1.5 m
    assert s.range_m == pytest.approx(1.5, abs=0.001)


def test_ultrasonic_clamped_to_max():
    """Even an empty arena: range_m must not exceed MAX."""
    s = SimSensors(ultrasonic_min=0.02, ultrasonic_max=2.0, inject_noise=False)
    a, r, _ = _scene()
    s.update(r, a)
    assert s.range_m <= 2.0


def test_ball_in_front_takes_priority_over_wall():
    a, r, s = _scene(theta=0.0)
    a.balls.append({'x': 2.0, 'y': 1.5, 'colour': 'red',
                    'radius': 0.02, 'grabbed': False})
    s.update(r, a)
    assert s.range_m == pytest.approx(0.5, abs=0.06)   # 2.0 - 1.5 = 0.5 m


def test_grabbed_ball_is_invisible_to_ultrasonic():
    a, r, s = _scene(theta=0.0)
    a.balls.append({'x': 2.0, 'y': 1.5, 'colour': 'red',
                    'radius': 0.02, 'grabbed': True})
    s.update(r, a)
    # Grabbed ball is ignored — should fall back to wall reading
    assert s.range_m == pytest.approx(1.5, abs=0.001)


def test_ball_off_axis_ignored():
    """Ball outside the cone half-width should not register."""
    a, r, s = _scene(theta=0.0)
    # Ball at (2.0, 1.5+0.50): perp = 0.50 m, way outside cone (~0.07 m)
    a.balls.append({'x': 2.0, 'y': 2.0, 'colour': 'red',
                    'radius': 0.02, 'grabbed': False})
    s.update(r, a)
    assert s.range_m == pytest.approx(1.5, abs=0.001)


def test_imu_yaw_tracks_robot_heading():
    a, r, s = _scene(theta=math.radians(30))
    s.update(r, a)
    assert s.imu_yaw == pytest.approx(30.0, abs=0.001)


def test_imu_gyro_tracks_angular_velocity():
    a, r, s = _scene()
    r.v_angular = 1.5
    s.update(r, a)
    assert s.imu_gyro_z == pytest.approx(1.5)


def test_accel_x_is_velocity_derivative():
    """accel_x = (v_now - v_prev) / dt"""
    a, r, s = _scene()
    s.dt = 0.05
    r._prev_v_linear = 0.10
    r.v_linear = 0.20
    s.update(r, a)
    # (0.20 - 0.10) / 0.05 = 2.0 m/s²
    assert s.accel_x == pytest.approx(2.0, abs=0.001)


def test_noise_injection_perturbs_range():
    """With inject_noise=True the reading should differ from deterministic
    case across enough samples."""
    a = SimArena(width=3.0, height=3.0, ball_radius=0.02, colours=())
    r = SimRobot(a)
    s_quiet = SimSensors(inject_noise=False)
    s_noisy = SimSensors(inject_noise=True, noise_seed=42)
    s_quiet.update(r, a)
    s_noisy.update(r, a)
    # Quiet hits 1.5 exactly; noisy should differ slightly with high prob.
    assert abs(s_noisy.range_m - 1.5) > 0.0   # any non-zero perturbation
    assert s_quiet.range_m == pytest.approx(1.5)


def test_noise_seed_reproducibility():
    """Two SimSensors seeded identically must produce identical noise."""
    a = SimArena(width=3.0, height=3.0, ball_radius=0.02, colours=())
    r = SimRobot(a)
    s1 = SimSensors(inject_noise=True, noise_seed=99)
    s2 = SimSensors(inject_noise=True, noise_seed=99)
    s1.update(r, a)
    s2.update(r, a)
    assert s1.range_m == s2.range_m
    assert s1.imu_yaw == s2.imu_yaw
