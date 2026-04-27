"""Tests for pi_nodes.control.mpc_controller.

Properties verified:
  * Long horizon + Riccati terminal ⇒ MPC explicit gain ≈ LQR gain;
  * Output respects [u_min, u_max] (both 'clip' and 'qp' modes);
  * Closed-loop converges to zero;
  * Setting horizon N=1 still works (degenerate but valid).
"""

from __future__ import annotations

import numpy as np
import pytest

scipy = pytest.importorskip("scipy")

from pi_nodes.control.mpc_controller import MPCController         # noqa: E402
from pi_nodes.control.state_space_model import (                   # noqa: E402
    StateSpaceModel,
    _continuous_AB,
    _zoh_discretize,
)


@pytest.fixture
def plant_mats():
    A, B = _continuous_AB(0.2, 0.15, 0.10)
    Ad, Bd = _zoh_discretize(A, B, 0.05)
    return Ad, Bd


@pytest.fixture
def Q_R():
    Q = np.diag([10.0, 10.0, 5.0, 1.0, 1.0])
    R = np.diag([1.0, 1.0])
    return Q, R


def test_mpc_constructs_with_defaults(plant_mats, Q_R):
    Ad, Bd = plant_mats
    Q, R = Q_R
    mpc = MPCController(Ad=Ad, Bd=Bd, Q=Q, R=R, N=10,
                        u_min=[-0.3, -2], u_max=[0.3, 2])
    assert mpc.K_first.shape == (2, 5)
    assert mpc.is_stable()


def test_mpc_zero_state_gives_zero(plant_mats, Q_R):
    Ad, Bd = plant_mats
    Q, R = Q_R
    mpc = MPCController(Ad=Ad, Bd=Bd, Q=Q, R=R, N=10,
                        u_min=[-0.3, -2], u_max=[0.3, 2])
    u = mpc.step(np.zeros(5))
    assert np.allclose(u, 0.0, atol=1e-12)


def test_mpc_long_horizon_matches_lqr(plant_mats, Q_R):
    """For N≥20 with Pf=Riccati P, MPC explicit K ≈ LQR K."""
    from scipy.linalg import solve_discrete_are
    Ad, Bd = plant_mats
    Q, R = Q_R
    P = solve_discrete_are(Ad, Bd, Q, R)
    K_lqr = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)

    mpc = MPCController(Ad=Ad, Bd=Bd, Q=Q, R=R, Pf=P, N=20,
                        u_min=[-0.3, -2], u_max=[0.3, 2])

    # For large N + Riccati terminal cost, the explicit MPC gain → LQR gain
    np.testing.assert_allclose(mpc.K_first, K_lqr, atol=1e-6, rtol=1e-6)


def test_mpc_clipping(plant_mats, Q_R):
    Ad, Bd = plant_mats
    Q, R = Q_R
    u_min = np.array([-0.30, -2.0])
    u_max = np.array([+0.30, +2.0])
    mpc = MPCController(Ad=Ad, Bd=Bd, Q=Q, R=R, N=10,
                        u_min=u_min, u_max=u_max, solver="clip")

    huge = np.array([100.0, 0.0, 0.0, 0.0, 0.0])
    u = mpc.step(huge)
    assert np.all(u >= u_min - 1e-9)
    assert np.all(u <= u_max + 1e-9)


def test_mpc_qp_solver_respects_bounds(plant_mats, Q_R):
    Ad, Bd = plant_mats
    Q, R = Q_R
    mpc = MPCController(Ad=Ad, Bd=Bd, Q=Q, R=R, N=10,
                        u_min=[-0.3, -2], u_max=[0.3, 2], solver="qp")

    huge = np.array([100.0, 0.0, 0.0, 0.0, 0.0])
    u = mpc.step(huge)
    assert -0.3 - 1e-9 <= u[0] <= 0.3 + 1e-9
    assert -2.0 - 1e-9 <= u[1] <= 2.0 + 1e-9


def test_mpc_simulation_converges(plant_mats, Q_R):
    Ad, Bd = plant_mats
    Q, R = Q_R
    mpc = MPCController(Ad=Ad, Bd=Bd, Q=Q, R=R, N=10,
                        u_min=[-0.3, -2], u_max=[0.3, 2])

    x = np.array([1.0, 1.0, 0.5, 0.0, 0.0])
    n0 = np.linalg.norm(x)
    for _ in range(400):
        u = mpc.step(x)
        x = Ad @ x + Bd @ u
    assert np.linalg.norm(x) < 0.2 * n0


def test_mpc_horizon_one():
    """N=1 is a degenerate but legal case (sit on terminal cost only)."""
    A, B = _continuous_AB(0.2, 0.15, 0.10)
    Ad, Bd = _zoh_discretize(A, B, 0.05)
    Q = np.diag([10.0, 10.0, 5.0, 1.0, 1.0])
    R = np.diag([1.0, 1.0])
    mpc = MPCController(Ad=Ad, Bd=Bd, Q=Q, R=R, N=1,
                        u_min=[-0.3, -2], u_max=[0.3, 2])
    u = mpc.step(np.array([1.0, 0, 0, 0, 0]))
    assert u.shape == (2,)


def test_mpc_invalid_horizon_raises():
    A, B = _continuous_AB(0.2, 0.15, 0.10)
    Ad, Bd = _zoh_discretize(A, B, 0.05)
    Q = np.diag([10.0]*5)
    R = np.eye(2)
    with pytest.raises(ValueError):
        MPCController(Ad=Ad, Bd=Bd, Q=Q, R=R, N=0,
                      u_min=[-0.3, -2], u_max=[0.3, 2])
