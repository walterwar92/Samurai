"""Tests for pi_nodes.control.state_space_model.

Verify the discrete plant model matches:
  * Continuous → discrete by ZOH (eigenvalue mapping λ̃ = exp(λ Ts));
  * Hand-crafted A, B for the simple sub-blocks;
  * Open-loop properties (controllability rank, eigenvalues).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

scipy = pytest.importorskip("scipy")
from scipy.linalg import expm  # noqa: E402

from pi_nodes.control.state_space_model import (  # noqa: E402
    StateSpaceModel,
    _continuous_AB,
    zoh_discretize,
)


# ──────────────────────── Plant building ────────────────────────
def test_continuous_AB_shapes():
    A, B = _continuous_AB(v0=0.2, tau_v=0.15, tau_w=0.10)
    assert A.shape == (5, 5)
    assert B.shape == (5, 2)


def test_continuous_AB_couplings():
    """Specific entries of A reflect the kinematic + first-order motor model."""
    v0 = 0.20
    tau_v, tau_w = 0.15, 0.10
    A, B = _continuous_AB(v0, tau_v, tau_w)

    # Kinematic coupling at the operating point
    assert A[0, 3] == pytest.approx(1.0)        # dpx/dv
    assert A[1, 2] == pytest.approx(v0)         # dpy/dtheta = v0
    assert A[2, 4] == pytest.approx(1.0)        # dtheta/domega
    # Motor first-order dynamics
    assert A[3, 3] == pytest.approx(-1.0 / tau_v)
    assert A[4, 4] == pytest.approx(-1.0 / tau_w)
    # Control entries
    assert B[3, 0] == pytest.approx(1.0 / tau_v)
    assert B[4, 1] == pytest.approx(1.0 / tau_w)
    # Cross-couplings should be zero
    assert B[0, 0] == 0 and B[1, 1] == 0


def test_continuous_eigenvalues():
    """Open-loop has 3 zero eigenvalues (integrators) and -1/τ on motors."""
    A, _ = _continuous_AB(0.2, 0.15, 0.10)
    eigs = np.sort_complex(np.linalg.eigvals(A))
    # Three zero eigenvalues
    assert sum(abs(e) < 1e-9 for e in eigs) == 3
    # Two real, negative
    nonzero = sorted(e.real for e in eigs if abs(e) > 1e-9)
    assert nonzero == pytest.approx(sorted([-1.0/0.15, -1.0/0.10]), rel=1e-9)


# ──────────────────────── Discretization ────────────────────────
def testzoh_discretize_eigenvalue_mapping():
    """Discrete eigenvalues = exp(continuous × Ts)."""
    A, B = _continuous_AB(0.2, 0.15, 0.10)
    Ts = 0.05
    Ad, _ = zoh_discretize(A, B, Ts)

    cont_eigs = np.linalg.eigvals(A)
    disc_eigs = np.linalg.eigvals(Ad)
    expected = np.sort_complex([math.exp(e.real) ** Ts if abs(e.imag) < 1e-9
                                else complex(math.cos(e.imag*Ts), math.sin(e.imag*Ts))
                                     * math.exp(e.real * Ts)
                                for e in cont_eigs])
    # Just check magnitudes line up (eigenvalue ordering is delicate)
    np.testing.assert_allclose(
        np.sort(np.abs(disc_eigs)),
        np.sort(np.abs(expected)),
        atol=1e-10,
    )


def test_zoh_matches_block_exp():
    """ZOH formula matches direct expm of the lifted (n+r)×(n+r) block."""
    A, B = _continuous_AB(0.2, 0.15, 0.10)
    Ts = 0.05

    Ad, Bd = zoh_discretize(A, B, Ts)

    # Manual reference via expm
    n, r = A.shape[0], B.shape[1]
    M = np.zeros((n + r, n + r))
    M[:n, :n] = A
    M[:n, n:] = B
    Md = expm(M * Ts)
    np.testing.assert_allclose(Ad, Md[:n, :n], atol=1e-12)
    np.testing.assert_allclose(Bd, Md[:n, n:], atol=1e-12)


# ──────────────────────── StateSpaceModel ────────────────────────
def test_model_step_basic():
    A, B = _continuous_AB(0.2, 0.15, 0.10)
    Ad, Bd = zoh_discretize(A, B, 0.05)
    model = StateSpaceModel(Ad, Bd, Ts=0.05)

    x0 = np.zeros(5)
    u  = np.array([0.1, 0.0])
    x1 = model.step(x0, u)

    # v ramps up during the timestep (1st-order motor dynamics)
    assert x1[3] > 0
    # px: ZOH integrates v(s) = u_v(1 - exp(-s/τ_v)) over [0, Ts]
    #     ⇒ Δpx = u_v · (Ts - τ_v(1 - exp(-Ts/τ_v))) ≈ 0.1 · 0.00748 ≈ 7.5e-4
    expected_px = 0.1 * (0.05 - 0.15 * (1.0 - math.exp(-0.05 / 0.15)))
    assert x1[0] == pytest.approx(expected_px, rel=1e-6)
    # angular channel untouched: omega and theta both stay zero
    assert x1[2] == pytest.approx(0)
    assert x1[4] == pytest.approx(0)
    # py also stays zero (no theta/omega coupling activated)
    assert x1[1] == pytest.approx(0)


def test_model_rollout_shape():
    A, B = _continuous_AB(0.2, 0.15, 0.10)
    Ad, Bd = zoh_discretize(A, B, 0.05)
    model = StateSpaceModel(Ad, Bd, Ts=0.05)

    U = np.tile([0.1, 0.0], (20, 1))
    X = model.rollout(np.zeros(5), U)
    assert X.shape == (21, 5)
    # After 20 steps of constant 0.1 m/s command, v should approach steady-state
    assert X[-1, 3] == pytest.approx(0.1, rel=0.05)


def test_controllability():
    A, B = _continuous_AB(0.2, 0.15, 0.10)
    Ad, Bd = zoh_discretize(A, B, 0.05)
    model = StateSpaceModel(Ad, Bd, Ts=0.05)
    assert model.is_controllable()
    assert model.controllability_rank() == 5


def test_open_loop_marginally_stable():
    """Continuous open-loop has integrators → discrete has |λ|=1 at three points."""
    A, B = _continuous_AB(0.2, 0.15, 0.10)
    Ad, Bd = zoh_discretize(A, B, 0.05)
    model = StateSpaceModel(Ad, Bd, Ts=0.05)
    # Integrators stay at |λ|=1 → not strictly stable
    assert not model.is_stable()
    eigs = np.linalg.eigvals(Ad)
    n_unit = sum(1 for e in eigs if abs(abs(e) - 1.0) < 1e-9)
    assert n_unit == 3


def test_zero_velocity_breaks_controllability():
    """At v0=0 the (px, py, theta) coupling vanishes → not fully controllable."""
    A, B = _continuous_AB(v0=0.0, tau_v=0.15, tau_w=0.10)
    Ad, Bd = zoh_discretize(A, B, 0.05)
    model = StateSpaceModel(Ad, Bd, Ts=0.05)
    # py becomes uncontrollable when v0=0 (no lateral motion possible)
    assert model.controllability_rank() < 5
