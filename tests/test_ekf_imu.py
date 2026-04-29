"""Unit tests for pi_nodes.filters.ekf_imu.EkfImu (#51).

Coverage areas:
  - Initial / reset state
  - Yaw integration from gyro Z
  - Yaw wrapping to [-π, π]
  - ZUPT entry/exit hysteresis
  - Bias correction under sustained ZUPT
  - Accel update for roll/pitch
  - Accel gate (high-vibration samples rejected)
  - Home-orientation subtraction
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pi_nodes.filters.ekf_imu import EkfImu


def _new(**kw):
    """Construct an EkfImu, allowing override of common args."""
    base = dict(q_angle=0.001, q_bias=0.0001, r_accel=0.5,
                accel_gate=0.3, g=9.81)
    base.update(kw)
    return EkfImu(**base)


def test_initial_state_is_zero():
    e = _new()
    assert e.yaw == 0.0
    assert e.roll == 0.0
    assert e.pitch == 0.0
    assert e.gyro_bias == (0.0, 0.0, 0.0)
    assert e.stationary is False


def test_predict_zero_dt_is_noop():
    e = _new()
    e.predict(0.0, 0.0, 1.0, 0.0)
    assert e.yaw == 0.0


def test_yaw_integrates_gyro():
    e = _new()
    # 1 rad/s for 1 second → yaw ~ 1 rad. Even with q_angle process noise this
    # should be within numerical error.
    for _ in range(50):
        e.predict(0.0, 0.0, 1.0, 0.02)
    assert e.yaw == pytest.approx(1.0, abs=0.05)


def test_yaw_wraps_to_pi_range():
    e = _new()
    # Spin 4 full revolutions (8π rad) — must always be normalised.
    for _ in range(800):
        e.predict(0.0, 0.0, math.pi, 0.05)
        assert -math.pi <= e.yaw <= math.pi, f'yaw out of range: {e.yaw}'


def test_zupt_enters_after_confirm_ticks():
    """Sustained near-zero gyro should flip stationary=True."""
    e = _new()
    # Need enough ticks at < zupt_enter to confirm
    for _ in range(20):
        e.predict(0.0, 0.0, 0.001, 0.02)   # 0.001 rad/s ≪ 0.015 enter
    assert e.stationary is True


def test_zupt_exits_on_motion():
    e = _new()
    # First become stationary
    for _ in range(20):
        e.predict(0.0, 0.0, 0.001, 0.02)
    assert e.stationary
    # Now exceed exit threshold
    e.predict(0.0, 0.0, 0.5, 0.02)         # 0.5 rad/s ≫ 0.04 exit
    assert e.stationary is False


def test_zupt_bias_correction():
    """Sustained constant-bias gyro under ZUPT pulls bias toward observed value."""
    e = _new()
    # Inject a constant 0.005 rad/s bias-like signal under ZUPT regime.
    # Threshold for entry is 0.015, so 0.005 stays under enter and triggers ZUPT,
    # which then drives gz_bias toward 0.005.
    for _ in range(200):
        e.predict(0.0, 0.0, 0.005, 0.02)
    # After ZUPT corrections the bias should track the observed gz.
    assert e.gyro_bias[2] == pytest.approx(0.005, abs=0.002)


def test_accel_update_computes_roll_pitch():
    e = _new()
    # Robot tilted +30° about X (roll). Gravity vector in body:
    # ax=0, ay=g*sin(30°), az=g*cos(30°).
    g = 9.81
    e.update(0.0, g * math.sin(math.radians(30)),
             g * math.cos(math.radians(30)))
    assert e.roll == pytest.approx(math.radians(30), abs=0.02)
    assert e.pitch == pytest.approx(0.0, abs=0.02)


def test_accel_gate_rejects_high_vibration():
    """When |a| is far from g, update should be skipped (no roll/pitch change)."""
    e = _new()
    # Establish a baseline roll
    e.update(0.0, 9.81 * 0.5, 9.81 * 0.866)  # ~30° roll
    baseline_roll = e.roll
    assert baseline_roll != 0.0
    # Now feed an obviously-non-gravity sample (|a| = 30 m/s² ≫ g)
    e.update(20.0, 20.0, 10.0)
    # Roll should be unchanged because the gate rejected the sample
    assert e.roll == baseline_roll


def test_home_orientation_subtraction():
    """init_from_calibration sets a home tilt; subsequent accel-derived angles
    are reported relative to it."""
    e = _new()
    # Robot mounted with a 10° intrinsic tilt — calibrated as home.
    home_roll = math.radians(10)
    e.init_from_calibration(roll=home_roll, pitch=0.0, yaw=0.0,
                            gyro_bias=(0, 0, 0))
    # Now feed gravity vector matching the 10° tilt: should report roll=0.
    g = 9.81
    e.update(0.0, g * math.sin(home_roll), g * math.cos(home_roll))
    assert e.roll == pytest.approx(0.0, abs=0.02)


def test_reset_clears_state():
    e = _new()
    for _ in range(50):
        e.predict(0.0, 0.0, 1.0, 0.02)
    assert e.yaw != 0.0
    e.reset()
    assert e.yaw == 0.0
    assert e.gyro_bias == (0.0, 0.0, 0.0)
    assert e.stationary is False


def test_get_euler_deg_returns_degrees():
    e = _new()
    for _ in range(50):
        e.predict(0.0, 0.0, math.pi / 2, 0.02)   # quarter-turn
    roll, pitch, yaw = e.get_euler_deg()
    assert yaw == pytest.approx(90.0, abs=3.0)
    assert roll == 0.0
    assert pitch == 0.0
