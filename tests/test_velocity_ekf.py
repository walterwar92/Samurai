"""Unit tests for pi_nodes.filters.velocity_ekf.VelocityEKF (#51).

Coverage areas:
  - Initial state
  - Friction decay (velocity decays with no input)
  - Accel update integrates measurement
  - ZUPT zeros velocity
  - Position integrates from fused velocity
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pi_nodes.filters.velocity_ekf import VelocityEKF


def _new(**kw):
    base = dict(friction_tau=0.15, q_velocity=0.08, q_bias=0.003,
                r_accel=0.12, r_zupt=0.0001)
    base.update(kw)
    return VelocityEKF(**base)


def test_initial_state_is_zero():
    e = _new()
    assert e.x == 0.0
    assert e.y == 0.0
    assert e.vx == 0.0
    assert e.vy == 0.0


def test_friction_decay_with_no_input():
    """Without any accel input, friction should bring velocity → 0."""
    e = _new()
    e.vx = 1.0
    e.vy = 0.5
    # Simulate 2 seconds of friction-only updates (no accel).
    dt = 0.02
    for _ in range(int(2.0 / dt)):
        e.predict_friction(dt)
    # Velocity should have decayed substantially. With friction_tau=0.15s,
    # exp(-2/0.15) ≈ 1.4e-6 — essentially zero.
    assert abs(e.vx) < 0.01
    assert abs(e.vy) < 0.01


def test_friction_decay_zero_dt_is_noop():
    e = _new()
    e.vx = 0.5
    e.predict_friction(0.0)
    assert e.vx == 0.5


def test_accel_update_drives_velocity():
    """Sustained constant accelerometer reading should pump velocity up."""
    e = _new(r_accel=0.001)   # tighter measurement noise → faster convergence
    # Simulate 1s of pure +0.5 m/s² acceleration in X (50 Hz).
    dt = 0.02
    for _ in range(50):
        e.predict_friction(dt)
        e.update_accel(0.5, 0.0, dt)
    # Velocity should be measurably positive (friction would be fighting it,
    # but with low r_accel the accel measurement dominates).
    assert e.vx > 0.05, f'vx did not grow: {e.vx}'


def test_zupt_zeros_velocity():
    e = _new()
    e.vx = 0.3
    e.vy = -0.2
    # ZUPT should pull velocity to zero with very low r_zupt.
    for _ in range(50):
        e.update_zupt()
    assert abs(e.vx) < 0.01
    assert abs(e.vy) < 0.01


def test_position_integrates_from_velocity():
    """integrate_position should advance x,y from vx,vy."""
    e = _new()
    e.vx = 0.5
    e.vy = 0.0
    # 1 second at 0.5 m/s → x ≈ 0.5 m
    dt = 0.02
    for _ in range(50):
        e.integrate_position(dt)
    assert e.x == pytest.approx(0.5, abs=0.01)
    assert e.y == pytest.approx(0.0, abs=0.01)


def test_reset_clears_state():
    e = _new()
    e.vx = 1.0
    e.vy = 0.5
    e.x = 2.0
    e.y = 1.0
    e.reset()
    assert e.vx == 0.0 and e.vy == 0.0
    assert e.x == 0.0 and e.y == 0.0


def test_bias_absorbs_steady_state_drift():
    """A constant accel offset under ZUPT should be absorbed into bias,
    so velocity does not drift unboundedly when the robot is actually still."""
    e = _new()
    dt = 0.02
    # Simulate static robot with a 0.05 m/s² accelerometer bias for 2 seconds.
    # ZUPT corrections should drive bias toward the measured offset, keeping
    # velocity bounded (not growing linearly with t).
    for _ in range(100):
        e.predict_friction(dt)
        e.update_accel(0.05, 0.0, dt)
        e.update_zupt()
    # Velocity should be bounded — no monotonic drift to large values.
    assert abs(e.vx) < 0.1, f'velocity drift not bounded by ZUPT: {e.vx}'
