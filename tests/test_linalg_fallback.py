"""Tests for pi_nodes.control._linalg — the scipy-optional numpy fallbacks.

Unlike the other control test modules this file does NOT
`pytest.importorskip('scipy')` — the whole point is that these paths must
work on the Pi, which has no scipy. The numpy fallbacks are verified
against scipy-independent oracles (analytic values, the defining
equations) and, when scipy happens to be installed, cross-checked
against it.
"""

from __future__ import annotations

import builtins
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pi_nodes.control._linalg import (  # noqa: E402
    expm,
    solve_discrete_are,
    _expm_numpy,
    _solve_discrete_are_numpy,
)

try:
    import scipy.linalg  # noqa: F401
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False


# ── helpers ────────────────────────────────────────────────────────────
def _mps_like_discrete():
    """A discretized MPS-style plant (5 states, 2 controls): controllable,
    open-loop marginally stable. Built without scipy, via _expm_numpy."""
    tau_v, tau_w, v0, Ts = 0.15, 0.10, 0.20, 0.02
    A_c = np.array([
        [0.0, 0.0, 0.0,  1.0,        0.0],
        [0.0, 0.0, v0,   0.0,        0.0],
        [0.0, 0.0, 0.0,  0.0,        1.0],
        [0.0, 0.0, 0.0, -1.0 / tau_v, 0.0],
        [0.0, 0.0, 0.0,  0.0,       -1.0 / tau_w],
    ])
    B_c = np.array([
        [0.0,        0.0],
        [0.0,        0.0],
        [0.0,        0.0],
        [1.0 / tau_v, 0.0],
        [0.0,        1.0 / tau_w],
    ])
    n, r = 5, 2
    M = np.zeros((n + r, n + r))
    M[:n, :n] = A_c
    M[:n, n:] = B_c
    Md = _expm_numpy(M * Ts)
    return Md[:n, :n], Md[:n, n:]


def _dare_residual(P, A, B, Q, R):
    """||P - (AᵀPA - AᵀPB(R+BᵀPB)⁻¹BᵀPA + Q)|| — zero at the exact solution.

    A scipy-independent oracle for the discrete algebraic Riccati equation.
    """
    BtP = B.T @ P
    BtPA = BtP @ A
    S = R + BtP @ B
    rhs = Q + A.T @ P @ A - BtPA.T @ np.linalg.solve(S, BtPA)
    return float(np.max(np.abs(P - rhs)))


def _hide_scipy(monkeypatch):
    """Make any `import scipy[...]` raise ImportError, as on the Pi."""
    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "scipy" or name.startswith("scipy."):
            raise ImportError("scipy hidden for test")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)


_Q = np.diag([10.0, 10.0, 5.0, 1.0, 1.0])
_R = np.diag([1.0, 1.0])


# ── _expm_numpy ────────────────────────────────────────────────────────
def test_expm_numpy_zero_is_identity():
    np.testing.assert_allclose(_expm_numpy(np.zeros((4, 4))), np.eye(4), atol=1e-15)


def test_expm_numpy_diagonal():
    d = [-0.2, 0.5, -1.3, 0.0]
    np.testing.assert_allclose(_expm_numpy(np.diag(d)), np.diag(np.exp(d)), atol=1e-12)


def test_expm_numpy_nilpotent_closed_form():
    """exp([[0,1],[0,0]]) = [[1,1],[0,1]] exactly."""
    N = np.array([[0.0, 1.0], [0.0, 0.0]])
    np.testing.assert_allclose(_expm_numpy(N), [[1.0, 1.0], [0.0, 1.0]], atol=1e-14)


def test_expm_numpy_scalar_zoh_value():
    """The known ZOH scalar case: exp(-10 · 0.02) = exp(-0.2)."""
    np.testing.assert_allclose(_expm_numpy(np.array([[-0.2]])), [[np.exp(-0.2)]], atol=1e-13)


def test_expm_numpy_inverse_property_triggers_scaling():
    """exp(A) @ exp(-A) == I — also exercises the scaling-and-squaring
    branch (the random matrix has inf-norm well above the 0.5 threshold)."""
    rng = np.random.default_rng(42)
    A = rng.standard_normal((6, 6))
    assert np.linalg.norm(A, np.inf) > 0.5  # scaling path is taken
    np.testing.assert_allclose(_expm_numpy(A) @ _expm_numpy(-A), np.eye(6), atol=1e-9)


def test_expm_numpy_eigenvalue_mapping():
    """exp maps eigenvalues: λ(exp(A)) = exp(λ(A))."""
    A = np.array([[-3.0, 1.0], [0.0, -7.0]])
    got = np.sort(np.linalg.eigvals(_expm_numpy(A)))
    want = np.sort(np.exp(np.linalg.eigvals(A)))
    np.testing.assert_allclose(got, want, atol=1e-12)


@pytest.mark.skipif(not HAVE_SCIPY, reason="scipy not installed")
def test_expm_numpy_matches_scipy():
    from scipy.linalg import expm as scipy_expm
    rng = np.random.default_rng(7)
    for _ in range(20):
        A = rng.standard_normal((7, 7))
        np.testing.assert_allclose(_expm_numpy(A), scipy_expm(A), atol=1e-9, rtol=1e-9)


# ── _solve_discrete_are_numpy ──────────────────────────────────────────
def test_dare_numpy_satisfies_equation():
    """The returned P must satisfy the DARE — a scipy-independent oracle."""
    Ad, Bd = _mps_like_discrete()
    P = _solve_discrete_are_numpy(Ad, Bd, _Q, _R)
    assert _dare_residual(P, Ad, Bd, _Q, _R) < 1e-8


def test_dare_numpy_symmetric_psd():
    Ad, Bd = _mps_like_discrete()
    P = _solve_discrete_are_numpy(Ad, Bd, _Q, _R)
    np.testing.assert_allclose(P, P.T, atol=1e-10)
    assert np.min(np.linalg.eigvalsh(P)) > -1e-9


def test_dare_numpy_closed_loop_stable():
    """K from the DARE solution must stabilize the plant: ρ(A - BK) < 1."""
    Ad, Bd = _mps_like_discrete()
    P = _solve_discrete_are_numpy(Ad, Bd, _Q, _R)
    K = np.linalg.solve(_R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    rho = float(np.max(np.abs(np.linalg.eigvals(Ad - Bd @ K))))
    assert rho < 1.0 - 1e-9


@pytest.mark.skipif(not HAVE_SCIPY, reason="scipy not installed")
def test_dare_numpy_matches_scipy():
    from scipy.linalg import solve_discrete_are as scipy_dare
    Ad, Bd = _mps_like_discrete()
    np.testing.assert_allclose(
        _solve_discrete_are_numpy(Ad, Bd, _Q, _R),
        scipy_dare(Ad, Bd, _Q, _R),
        atol=1e-6, rtol=1e-6,
    )


# ── public wrappers: route correctly + survive a missing scipy ─────────
def test_wrappers_work():
    Ad, Bd = _mps_like_discrete()
    np.testing.assert_allclose(expm(np.zeros((3, 3))), np.eye(3), atol=1e-15)
    assert _dare_residual(solve_discrete_are(Ad, Bd, _Q, _R), Ad, Bd, _Q, _R) < 1e-8


def test_expm_wrapper_falls_back_without_scipy(monkeypatch):
    _hide_scipy(monkeypatch)
    np.testing.assert_allclose(expm(np.array([[-0.2]])), [[np.exp(-0.2)]], atol=1e-13)


def test_dare_wrapper_falls_back_without_scipy(monkeypatch):
    _hide_scipy(monkeypatch)
    Ad, Bd = _mps_like_discrete()
    P = solve_discrete_are(Ad, Bd, _Q, _R)
    assert _dare_residual(P, Ad, Bd, _Q, _R) < 1e-8


def test_zoh_discretize_works_without_scipy(monkeypatch):
    """End-to-end regression for the original crash: zoh_discretize must
    not need scipy. _linalg imports scipy lazily (call-time), so hiding it
    forces the numpy fallback without reloading the module."""
    _hide_scipy(monkeypatch)
    from pi_nodes.control.state_space_model import zoh_discretize
    Ad, Bd = zoh_discretize(np.array([[-10.0]]), np.array([[10.0]]), 0.02)
    np.testing.assert_allclose(Ad, [[np.exp(-0.2)]], atol=1e-9)
    np.testing.assert_allclose(Bd, [[1.0 - np.exp(-0.2)]], atol=1e-9)


def test_mpc_controller_builds_without_scipy(monkeypatch):
    """The full mps_node._build_from_config boot path with scipy hidden:
    ZOH-discretize a continuous plant (needs expm), then build an
    MPCController with explicit Ad/Bd and no Pf (needs solve_discrete_are
    for the terminal penalty). Both steps crashed before this fix."""
    _hide_scipy(monkeypatch)
    from pi_nodes.control.state_space_model import zoh_discretize
    from pi_nodes.control.mpc_controller import MPCController

    tau_v, tau_w, v0 = 0.15, 0.10, 0.20
    A_c = np.array([
        [0.0, 0.0, 0.0,  1.0,        0.0],
        [0.0, 0.0, v0,   0.0,        0.0],
        [0.0, 0.0, 0.0,  0.0,        1.0],
        [0.0, 0.0, 0.0, -1.0 / tau_v, 0.0],
        [0.0, 0.0, 0.0,  0.0,       -1.0 / tau_w],
    ])
    B_c = np.array([
        [0.0, 0.0], [0.0, 0.0], [0.0, 0.0],
        [1.0 / tau_v, 0.0], [0.0, 1.0 / tau_w],
    ])
    Ad, Bd = zoh_discretize(A_c, B_c, 0.02)
    mpc = MPCController(Ad=Ad, Bd=Bd, Q=_Q, R=_R, N=10,
                        u_min=[-0.3, -2.0], u_max=[0.3, 2.0], solver="clip")
    assert mpc.K_first.shape == (2, 5)
    assert mpc.is_stable()
