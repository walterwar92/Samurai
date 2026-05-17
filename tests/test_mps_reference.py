"""Unit-tests для pi_nodes/control/mps_reference.py — генератор r(t)."""
import math
import numpy as np
import pytest

from pi_nodes.control.mps_reference import (
    ReferenceTrajectory,
    build_reference,
)


def test_build_reference_returns_trajectory():
    traj = build_reference(distance=0.30, v_target=0.15, target_heading=math.pi,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    assert isinstance(traj, ReferenceTrajectory)
    assert traj.t_drive > 0
    assert traj.t_end > traj.t_drive


def _make_drive_trap():
    return build_reference(distance=0.30, v_target=0.15, target_heading=0.0,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)


def test_drive_trapezoid_endpoints():
    traj = _make_drive_trap()
    r0 = traj.r(0.0)
    r_end = traj.r(traj.t_drive)
    # старт: s=0, v=0, θ=θ_start, ω=0
    np.testing.assert_allclose(r0, [0.0, 0.0, 0.0, 0.0, 0.0], atol=1e-9)
    # конец drive: s=D, v=0, θ=θ_start, ω=0
    np.testing.assert_allclose(r_end, [0.30, 0.0, 0.0, 0.0, 0.0], atol=1e-9)


def test_drive_trapezoid_cruise_peak():
    traj = _make_drive_trap()
    # середина cruise — v должен быть = v_target
    r_mid = traj.r(traj.t_drive / 2)
    assert abs(r_mid[1] - 0.15) < 1e-9


def test_drive_trapezoid_monotonic_s():
    traj = _make_drive_trap()
    ts = np.linspace(0, traj.t_drive, 200)
    s_vals = [traj.r(t)[0] for t in ts]
    diffs = np.diff(s_vals)
    assert np.all(diffs >= -1e-12), 'drive s_ref must be non-decreasing'


def test_drive_triangular_short_distance():
    # D=0.01 < v²/a = 0.0225/0.2 = 0.1125 → triangle
    traj = build_reference(distance=0.01, v_target=0.15, target_heading=0.0,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    r_end = traj.r(traj.t_drive)
    np.testing.assert_allclose(r_end, [0.01, 0.0, 0.0, 0.0, 0.0], atol=1e-9)
    # NB: num=101 (odd) so a sample lands exactly on the peak at t_drive/2;
    # with num=100 the closest sample misses by ~t_drive/198 ≈ 2.3e-3 s,
    # giving |Δv| ≈ a·2.3e-3 ≈ 4.5e-4 which violates the 1e-6 assertion.
    v_peak = max(traj.r(t)[1] for t in np.linspace(0, traj.t_drive, 101))
    assert v_peak < 0.15, 'triangle must not reach v_target'
    assert abs(v_peak - math.sqrt(0.01 * 0.20)) < 1e-6


def test_turn_trapezoid_endpoints():
    # φ=π = trapezoid (π > 0.25 rad threshold)
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=math.pi,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    r_end = traj.r(traj.t_end)
    # Note: _normalize_angle(+π) = -π — convention documented in mps_reference.py.
    # Robot turns CW by π, landing at θ_start − π which is the SAME orientation
    # as θ_start + π (mod 2π).
    assert abs(abs(r_end[2]) - math.pi) < 1e-9
    assert abs(r_end[3]) < 1e-9


def test_turn_positive_pi_uses_cw_per_normalize_convention():
    """Pin the documented CW convention for target_heading=+π exactly.

    _normalize_angle((π + π) % 2π − π) = 0 − π = −π. So phi_signed = −π,
    sign = -1, robot turns clockwise by π rad. End-point heading is θ_start − π.
    """
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=math.pi,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5,
                           theta_start=0.5)
    assert traj.phi_signed == pytest.approx(-math.pi)
    r_end = traj.r(traj.t_end)
    assert r_end[2] == pytest.approx(0.5 - math.pi, abs=1e-9)


def test_turn_triangular_short_angle():
    # |φ|=0.1 < ω²/α = 0.25/1.0 = 0.25 → triangle
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=0.1,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    r_end = traj.r(traj.t_end)
    assert abs(r_end[2] - 0.1) < 1e-9


def test_turn_negative_phi():
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=-0.5,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    r_end = traj.r(traj.t_end)
    assert abs(r_end[2] - (-0.5)) < 1e-9


def test_zero_distance_zero_phi():
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=0.0,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    assert traj.t_drive == 0.0
    assert traj.t_end == 0.0
    np.testing.assert_allclose(traj.r(0.0), [0.0, 0.0, 0.0, 0.0, 0.0], atol=1e-12)


def test_continuity_at_t_drive():
    traj = build_reference(distance=0.30, v_target=0.15, target_heading=math.pi,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    left = traj.r(traj.t_drive - 1e-6)
    right = traj.r(traj.t_drive + 1e-6)
    np.testing.assert_allclose(left, right, atol=1e-3)


def test_r_after_t_end_returns_final():
    traj = build_reference(distance=0.30, v_target=0.15, target_heading=math.pi,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    r_late = traj.r(traj.t_end + 100.0)
    # Final orientation = θ_start + phi_signed = 0 + (−π) = −π
    np.testing.assert_allclose(
        r_late, [0.30, 0.0, -math.pi, 0.0, 0.0], atol=1e-9)


def test_theta_start_offset():
    # Сценарий стартует не из θ=0
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=0.5,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5,
                           theta_start=1.0)
    r_end = traj.r(traj.t_end)
    assert abs(r_end[2] - 1.5) < 1e-9


def test_drive_then_turn_combined():
    """D > 0 AND φ ≠ 0: drive holds θ at θ_start; turn holds s at D."""
    traj = build_reference(distance=0.30, v_target=0.15, target_heading=0.5,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    # End of drive: arrived at D, heading unchanged from θ_start=0
    r_dr = traj.r(traj.t_drive)
    np.testing.assert_allclose(r_dr, [0.30, 0.0, 0.0, 0.0, 0.0], atol=1e-9)
    # End of turn: still at D, rotated to φ
    r_end = traj.r(traj.t_end)
    np.testing.assert_allclose(r_end, [0.30, 0.0, 0.5, 0.0, 0.0], atol=1e-9)


def test_r_negative_t_no_crash_with_d_zero():
    """r(-1.0) с D=0 не должен падать IndexError (см. clamp в r())."""
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=math.pi,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    out = traj.r(-1.0)  # должно отработать как r(0)
    np.testing.assert_allclose(out, [0.0, 0.0, 0.0, 0.0, 0.0], atol=1e-12)
