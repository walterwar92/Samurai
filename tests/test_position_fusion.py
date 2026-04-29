"""Unit tests for pi_nodes.filters.position_fusion."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pi_nodes.filters.position_fusion import (  # noqa: E402
    PositionFusion,
    VALID_MODES,
    _Kalman1D,
)


# ── Mode + parameter handling ────────────────────────────────────────────────

def test_default_mode_is_wheel():
    pf = PositionFusion()
    assert pf.mode == 'wheel'


def test_invalid_mode_raises():
    with pytest.raises(ValueError):
        PositionFusion(mode='moon-phase')


def test_set_mode_returns_changed_flag():
    pf = PositionFusion(mode='wheel')
    assert pf.set_mode('imu') is True
    assert pf.set_mode('imu') is False  # same mode = no change


def test_set_mode_invalid_raises():
    pf = PositionFusion()
    with pytest.raises(ValueError):
        pf.set_mode('quantum')


def test_alpha_clamped():
    pf = PositionFusion(alpha=2.5)
    assert pf.alpha == 1.0
    pf.set_alpha(-0.5)
    assert pf.alpha == 0.0
    pf.set_alpha(0.4)
    assert pf.alpha == 0.4


def test_all_modes_listed():
    # Sanity: all 4 modes exist and can be constructed
    for m in VALID_MODES:
        pf = PositionFusion(mode=m)
        assert pf.mode == m


# ── Mode 'wheel' / 'imu' — passthroughs ──────────────────────────────────────

def test_wheel_mode_passes_wheel():
    pf = PositionFusion(mode='wheel')
    x, y = pf.update(x_wheel=1.5, y_wheel=2.5,
                     x_imu=99.0, y_imu=99.0,
                     vx_imu=0.0, vy_imu=0.0,
                     dt=0.05)
    assert x == pytest.approx(1.5)
    assert y == pytest.approx(2.5)


def test_imu_mode_passes_imu():
    pf = PositionFusion(mode='imu')
    x, y = pf.update(x_wheel=99.0, y_wheel=99.0,
                     x_imu=0.3, y_imu=-0.7,
                     vx_imu=0.0, vy_imu=0.0,
                     dt=0.05)
    assert x == pytest.approx(0.3)
    assert y == pytest.approx(-0.7)


# ── Mode 'complementary' — weighted blend ────────────────────────────────────

def test_complementary_alpha_one_equals_wheel():
    pf = PositionFusion(mode='complementary', alpha=1.0)
    x, y = pf.update(x_wheel=1.0, y_wheel=2.0,
                     x_imu=10.0, y_imu=20.0,
                     vx_imu=0.0, vy_imu=0.0,
                     dt=0.05)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(2.0)


def test_complementary_alpha_zero_equals_imu():
    pf = PositionFusion(mode='complementary', alpha=0.0)
    x, y = pf.update(x_wheel=1.0, y_wheel=2.0,
                     x_imu=10.0, y_imu=20.0,
                     vx_imu=0.0, vy_imu=0.0,
                     dt=0.05)
    assert x == pytest.approx(10.0)
    assert y == pytest.approx(20.0)


def test_complementary_alpha_half_is_average():
    pf = PositionFusion(mode='complementary', alpha=0.5)
    x, y = pf.update(x_wheel=2.0, y_wheel=4.0,
                     x_imu=4.0, y_imu=8.0,
                     vx_imu=0.0, vy_imu=0.0,
                     dt=0.05)
    assert x == pytest.approx(3.0)
    assert y == pytest.approx(6.0)


# ── Mode 'ekf' — basic convergence ───────────────────────────────────────────

def test_ekf_converges_to_consistent_measurement():
    """Both sources agree → EKF settles near that value."""
    pf = PositionFusion(mode='ekf', q_pos=0.001, q_vel=0.001)
    for _ in range(200):
        x, _ = pf.update(x_wheel=1.0, y_wheel=0.0,
                         x_imu=1.0, y_imu=0.0,
                         vx_imu=0.0, vy_imu=0.0,
                         dt=0.05)
    assert x == pytest.approx(1.0, abs=0.05)


def test_ekf_blends_when_sources_disagree():
    """Wheel and IMU disagree — EKF result is between them, not equal to either."""
    pf = PositionFusion(mode='ekf',
                        r_wheel=0.04, r_imu=0.20)
    for _ in range(50):
        x, _ = pf.update(x_wheel=2.0, y_wheel=0.0,
                         x_imu=4.0, y_imu=0.0,
                         vx_imu=0.0, vy_imu=0.0,
                         dt=0.05)
    # r_wheel < r_imu → EKF trusts wheel more → answer closer to 2 than to 4
    assert 2.0 <= x <= 3.5


def test_ekf_warms_up_in_other_modes():
    """Even in 'wheel' mode, the EKF is fed measurements so a runtime
    switch to 'ekf' inherits a sensible state, not (0, 0)."""
    pf = PositionFusion(mode='wheel', q_pos=0.001, q_vel=0.001)
    for _ in range(100):
        pf.update(x_wheel=1.0, y_wheel=0.5,
                  x_imu=1.0, y_imu=0.5,
                  vx_imu=0.0, vy_imu=0.0,
                  dt=0.05)
    # Switch to EKF and read one tick — already near 1.0, not 0.0
    pf.set_mode('ekf')
    x, y = pf.update(x_wheel=1.0, y_wheel=0.5,
                     x_imu=1.0, y_imu=0.5,
                     vx_imu=0.0, vy_imu=0.0,
                     dt=0.05)
    assert x == pytest.approx(1.0, abs=0.1)
    assert y == pytest.approx(0.5, abs=0.1)


# ── Reset ────────────────────────────────────────────────────────────────────

def test_reset_clears_fused_and_ekf():
    pf = PositionFusion(mode='ekf')
    for _ in range(50):
        pf.update(x_wheel=5.0, y_wheel=3.0,
                  x_imu=5.0, y_imu=3.0,
                  vx_imu=0.0, vy_imu=0.0,
                  dt=0.05)
    pf.reset()
    x, y = pf.update(x_wheel=0.0, y_wheel=0.0,
                     x_imu=0.0, y_imu=0.0,
                     vx_imu=0.0, vy_imu=0.0,
                     dt=0.05)
    assert abs(x) < 0.1
    assert abs(y) < 0.1


def test_reset_to_specific_value():
    pf = PositionFusion(mode='ekf')
    pf.reset(x=2.0, y=-1.5)
    # Right after reset, in 'ekf' mode the first read should reflect
    # the reset (any measurement update will pull, but with q small it
    # remains close).
    x, y = pf.update(x_wheel=2.0, y_wheel=-1.5,
                     x_imu=2.0, y_imu=-1.5,
                     vx_imu=0.0, vy_imu=0.0,
                     dt=0.0)  # dt=0 => no predict, only update
    assert x == pytest.approx(2.0, abs=0.5)
    assert y == pytest.approx(-1.5, abs=0.5)


# ── _Kalman1D primitive ──────────────────────────────────────────────────────

def test_kalman1d_predict_advances_position_with_velocity():
    kf = _Kalman1D(q_pos=0.001, q_vel=0.001, r_wheel=0.01, r_imu=0.1, r_imu_v=0.05)
    kf.p = 0.0
    kf.v = 1.0  # 1 m/s
    kf.predict(0.5)
    assert kf.p == pytest.approx(0.5)
    assert kf.v == pytest.approx(1.0)  # constant velocity model


def test_kalman1d_update_pulls_state_toward_measurement():
    kf = _Kalman1D(q_pos=0.001, q_vel=0.001, r_wheel=0.01, r_imu=0.1, r_imu_v=0.05)
    kf.p = 0.0
    kf.update_wheel(1.0)
    # With initial P00=1.0 and r_wheel=0.01, K0 ≈ 1.0/(1.0+0.01) ≈ 0.99
    # so state pulled almost all the way to 1.0
    assert kf.p > 0.9


def test_kalman1d_velocity_measurement_affects_velocity():
    kf = _Kalman1D(q_pos=0.001, q_vel=0.001, r_wheel=0.01, r_imu=0.1, r_imu_v=0.05)
    kf.update_imu_velocity(0.5)
    assert kf.v > 0.4
