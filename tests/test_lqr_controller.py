"""Tests for pi_nodes.control.lqr_controller.

Properties verified:
  * K stabilises the closed-loop (|λ(Ad - Bd K)| < 1);
  * Output is clipped to [u_min, u_max];
  * Zero state ⇒ zero control.
"""

from __future__ import annotations

import numpy as np
import pytest

scipy = pytest.importorskip("scipy")

from pi_nodes.control.lqr_controller import LQRController          # noqa: E402
from pi_nodes.control.state_space_model import (                    # noqa: E402
    StateSpaceModel,
    _continuous_AB,
    _zoh_discretize,
)


@pytest.fixture
def plant():
    A, B = _continuous_AB(0.2, 0.15, 0.10)
    Ad, Bd = _zoh_discretize(A, B, 0.05)
    return StateSpaceModel(Ad, Bd, Ts=0.05)


@pytest.fixture
def lqr_K(plant):
    """K computed from DARE for the default Q, R."""
    from scipy.linalg import solve_discrete_are
    Q = np.diag([10.0, 10.0, 5.0, 1.0, 1.0])
    R = np.diag([1.0, 1.0])
    P = solve_discrete_are(plant.Ad, plant.Bd, Q, R)
    K = np.linalg.solve(R + plant.Bd.T @ P @ plant.Bd,
                        plant.Bd.T @ P @ plant.Ad)
    return K


def test_lqr_zero_state_gives_zero_control(lqr_K):
    ctrl = LQRController(K=lqr_K, u_min=[-1, -1], u_max=[1, 1])
    u = ctrl.step(np.zeros(5))
    assert np.allclose(u, 0.0)


def test_lqr_clipping_at_limits(lqr_K):
    """Large state ⇒ unclipped LQR exceeds limits ⇒ output is at the bound."""
    u_min = np.array([-0.30, -2.0])
    u_max = np.array([+0.30, +2.0])
    ctrl = LQRController(K=lqr_K, u_min=u_min, u_max=u_max)

    huge = np.array([10.0, 10.0, 1.0, 0.0, 0.0])
    u = ctrl.step(huge)
    assert np.all(u >= u_min - 1e-9)
    assert np.all(u <= u_max + 1e-9)
    # At least one bound must be active for such a large state
    on_bound = np.isclose(u, u_min) | np.isclose(u, u_max)
    assert np.any(on_bound)


def test_lqr_closed_loop_is_stable(plant, lqr_K):
    """All closed-loop eigenvalues inside the unit circle."""
    Acl = plant.Ad - plant.Bd @ lqr_K
    eigs = np.linalg.eigvals(Acl)
    assert np.all(np.abs(eigs) < 1.0 - 1e-3), \
        f"Unstable closed-loop, eigs = {np.abs(eigs)}"


def test_lqr_simulation_converges(plant, lqr_K):
    """From a non-zero IC the closed-loop drives state toward 0."""
    ctrl = LQRController(K=lqr_K,
                         u_min=[-0.30, -2.0],
                         u_max=[+0.30, +2.0])
    x = np.array([1.0, 1.0, 0.5, 0.0, 0.0])
    norms = []
    for _ in range(400):                                 # 20 s at Ts=0.05
        u = ctrl.step(x)
        x = plant.step(x, u)
        norms.append(np.linalg.norm(x))
    # State norm should drop substantially within 20 s
    assert norms[-1] < 0.2 * norms[0]


def test_lqr_tracking_with_xref(plant, lqr_K):
    """With x_ref ≠ 0 the controller drives toward x_ref."""
    ctrl = LQRController(K=lqr_K,
                         u_min=[-0.30, -2.0],
                         u_max=[+0.30, +2.0])
    x_ref = np.array([0.5, 0.0, 0.0, 0.2, 0.0])
    x = np.zeros(5)
    for _ in range(400):
        u = ctrl.step(x, x_ref=x_ref)
        x = plant.step(x, u)
    # px should track to ~0.5 (LQR has steady-state offset for pure integrators —
    # we only check the right direction)
    assert x[0] > 0.2, f"Expected px to move toward 0.5, got {x[0]:.3f}"


def test_lqr_shape_validation():
    """K must agree with u_min/u_max in size."""
    K = np.zeros((2, 5))
    LQRController(K=K, u_min=[-1, -1], u_max=[1, 1])      # ok
    with pytest.raises(ValueError):
        LQRController(K=K, u_min=[-1, -1, -1], u_max=[1, 1, 1])
