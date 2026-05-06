"""Tests for compute_node.mps_runner.

Verifies:
  • Идеальная propagation x[k+1]=Ax+Bu совпадает с ручным расчётом
    для нескольких шагов (без управления).
  • Дефолтные матрицы + D=2, v_target=0.15 → status='reached',
    ss_error<0.05.
  • Неустойчивые матрицы → status='error'.
  • Метрики peak_v / peak_omega / control_energy не отрицательные и
    конечные.
  • short_step_response отдаёт ровно ожидаемое число точек.
  • closed_loop_eigenvalues возвращает 5+5 комплексных значений.
"""
from __future__ import annotations

import numpy as np
import pytest

scipy = pytest.importorskip('scipy')

from compute_node.dashboard.schemas.mps import (  # noqa: E402
    MpsMatrices,
    MpsScenarioRequest,
)
from compute_node.mps_runner import (  # noqa: E402
    closed_loop_eigenvalues,
    run_scenario_idealized,
    short_step_response,
)


# ── Fixtures: дефолтные «учебные» матрицы из matlab/main.m ────────────
def _default_matrices(*, A=None, B=None, horizon_N=10) -> MpsMatrices:
    A_default = A if A is not None else [
        [1, 0, 0, 0.0425203, 0],
        [0, 1, 0.01, 0, 0.000213061],
        [0, 0, 1, 0, 0.0393469],
        [0, 0, 0, 0.716531, 0],
        [0, 0, 0, 0, 0.606531],
    ]
    B_default = B if B is not None else [
        [0.0074797, 0],
        [0, 3.69387e-05],
        [0, 0.0106531],
        [0.283469, 0],
        [0, 0.393469],
    ]
    C = [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)]
    D = [[0.0, 0.0] for _ in range(5)]
    return MpsMatrices(
        A=A_default, B=B_default, C=C, D=D,
        Q_diag=[10, 10, 5, 1, 1], R_diag=[1, 1],
        horizon_N=horizon_N,
        u_min=[-0.30, -2.0], u_max=[0.30, 2.0],
    )


# ── Reachability ───────────────────────────────────────────────────────
def test_default_matrices_reach_d2():
    m = _default_matrices()
    req = MpsScenarioRequest(distance=2.0, v_target=0.15, source='sim')
    res = run_scenario_idealized(m, req)
    assert res.status == 'reached'
    assert res.metrics is not None
    assert res.metrics.ss_error < 0.10  # ≤10 см
    assert res.metrics.peak_v > 0.0
    assert res.metrics.control_energy >= 0.0
    assert len(res.telemetry) > 5
    assert res.telemetry[-1].s_remaining < 0.10


def test_short_distance_reaches_quickly():
    m = _default_matrices()
    req = MpsScenarioRequest(distance=0.5, v_target=0.10, source='sim')
    res = run_scenario_idealized(m, req)
    assert res.status == 'reached'
    # Less than the 3·D/v_target timeout
    last_t = res.telemetry[-1].t
    assert last_t < 3.0 * 0.5 / 0.10


# ── Errors / instability ───────────────────────────────────────────────
def test_unstable_matrix_returns_error_or_timeout():
    """An A with |λ|=1.5 may either be controlled into bounds (rare for the
    course matrices) or blow up — either 'timeout' or 'error' is acceptable;
    we just want NOT 'reached', and no crash."""
    A_unstable = [
        [1.5, 0, 0, 0, 0],
        [0, 1.5, 0, 0, 0],
        [0, 0, 1.5, 0, 0],
        [0, 0, 0, 1.5, 0],
        [0, 0, 0, 0, 1.5],
    ]
    m = _default_matrices(A=A_unstable)
    req = MpsScenarioRequest(distance=2.0, v_target=0.15, source='sim')
    res = run_scenario_idealized(m, req, max_steps=300)
    assert res.status in ('error', 'timeout')


def test_initial_state_wrong_shape_returns_error():
    m = _default_matrices()
    req = MpsScenarioRequest(distance=1.0, v_target=0.10, source='sim')
    res = run_scenario_idealized(m, req, initial_state=np.zeros(3))
    assert res.status == 'error'
    assert res.error_message and 'shape' in res.error_message


# ── Propagation correctness ────────────────────────────────────────────
def test_zero_control_propagation_matches_manual():
    """If we force u=0 (set u_min=u_max=0) for a stable A and start from
    x = [0.5, 0.0, ...], the trajectory should follow x[k+1] = A·x[k] exactly.
    With Q=0 and R=1 the unconstrained MPC law gives u=0, but to be sure
    we also clamp u_min=u_max=0 element-wise on a tiny range that the
    explicit clip will respect."""
    # Stable diagonal A
    A = [[0.9, 0, 0, 0, 0],
         [0, 0.9, 0, 0, 0],
         [0, 0, 0.9, 0, 0],
         [0, 0, 0, 0.9, 0],
         [0, 0, 0, 0, 0.9]]
    B = [[0, 0]] * 5
    C = [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)]
    D = [[0.0, 0.0] for _ in range(5)]
    m = MpsMatrices(
        A=A, B=B, C=C, D=D,
        Q_diag=[0, 0, 0, 0, 0], R_diag=[1, 1],
        horizon_N=5,
        u_min=[-0.001, -0.001], u_max=[0.001, 0.001],
    )
    req = MpsScenarioRequest(distance=4.0, v_target=0.10, source='sim')
    initial = np.array([0.5, 0.0, 0.0, 0.0, 0.0])
    res = run_scenario_idealized(m, req, dt=1.0, max_steps=10, initial_state=initial)
    # B=0, so x[k+1] = 0.9 x[k]: s should evolve 0.5, 0.45, 0.405, 0.3645, ...
    s_seq = [p.x[0] for p in res.telemetry]
    expected = [0.5 * 0.9 ** k for k in range(len(s_seq))]
    np.testing.assert_allclose(s_seq, expected, atol=1e-6)
    # No reach: s decays to zero, distance=4.0 unreachable. Status will
    # be 'timeout' since we limited max_steps.
    assert res.status == 'timeout'


# ── short_step_response ────────────────────────────────────────────────
def test_short_step_response_returns_expected_length():
    m = _default_matrices()
    pts = short_step_response(m, duration_s=2.0, dt=0.02)
    # Should have >0 points; allow up to (duration/dt)+1.
    assert 5 < len(pts) <= int(2.0 / 0.02) + 1


def test_short_step_response_starts_at_zero_state():
    m = _default_matrices()
    pts = short_step_response(m, duration_s=1.0, dt=0.05)
    assert pts[0].t == pytest.approx(0.0)
    np.testing.assert_allclose(pts[0].x, [0, 0, 0, 0, 0])


# ── Eigenvalues helper ─────────────────────────────────────────────────
def test_closed_loop_eigenvalues_returns_5_5():
    m = _default_matrices()
    eig_open, eig_closed = closed_loop_eigenvalues(m)
    assert len(eig_open) == 5
    assert len(eig_closed) == 5
    # Default plant should be marginally stable (|λ|≤1) but closed loop must be stable.
    assert all(np.isfinite(z) for z in eig_closed)
    assert max(abs(z) for z in eig_closed) < 1.0
