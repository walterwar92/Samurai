"""_linalg — matrix exponential and discrete Riccati solver with numpy fallbacks.

The Raspberry Pi runs a deliberately lean Python environment: requirements.txt
declares only numpy, scipy is not installed there. The state-space control
modules need a matrix exponential (ZOH discretization) and a
discrete-algebraic-Riccati solver (MPC/LQR terminal cost). Both are delegated
to scipy when it is importable (laptop / CI) and to the numpy-only fallbacks
below otherwise — mirroring the scipy-optional pattern already used by
MPCController._solve_qp and _block_diag.
"""

from __future__ import annotations

import math

import numpy as np


# ── public API: scipy when available, numpy fallback otherwise ─────────

def expm(M: np.ndarray) -> np.ndarray:
    """Matrix exponential e^M.

    Uses scipy.linalg.expm when scipy is installed (laptop / CI); otherwise
    falls back to the numpy scaling-and-squaring Taylor series in
    _expm_numpy (the Pi has no scipy).
    """
    try:
        from scipy.linalg import expm as _expm_scipy
    except ImportError:
        return _expm_numpy(M)
    return _expm_scipy(M)


def solve_discrete_are(A: np.ndarray, B: np.ndarray,
                       Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Stabilizing solution P of the discrete algebraic Riccati equation

        P = A'PA - A'PB (R + B'PB)^-1 B'PA + Q.

    Uses scipy.linalg.solve_discrete_are when scipy is installed; otherwise
    falls back to the numpy Riccati fixed-point iteration in
    _solve_discrete_are_numpy (the Pi has no scipy).
    """
    try:
        from scipy.linalg import solve_discrete_are as _dare_scipy
    except ImportError:
        return _solve_discrete_are_numpy(A, B, Q, R)
    return _dare_scipy(A, B, Q, R)


# ── numpy-only fallbacks ───────────────────────────────────────────────

def _expm_numpy(M: np.ndarray) -> np.ndarray:
    """Matrix exponential via scaling-and-squaring with a Taylor series.

    exp(M) = exp(M / 2^s) ** (2^s); s is chosen so the scaled matrix has
    inf-norm <= 0.5, where a ~16-term Taylor series already reaches machine
    precision. Unconditionally convergent; the matrices in ZOH
    discretization are small (<= 7x7) so the cost is negligible.
    """
    M = np.asarray(M, dtype=float)
    norm = float(np.linalg.norm(M, np.inf))
    s = max(0, math.ceil(math.log2(norm / 0.5))) if norm > 0.5 else 0
    A = M / (2.0 ** s)

    n = A.shape[0]
    E = np.eye(n)
    term = np.eye(n)
    for k in range(1, 40):
        term = term @ A / k
        E = E + term
        if np.max(np.abs(term)) <= 1e-18 * max(1.0, float(np.max(np.abs(E)))):
            break

    for _ in range(s):
        E = E @ E
    return E


_DARE_MAX_ITER = 10_000
_DARE_TOL = 1e-12


def _solve_discrete_are_numpy(A: np.ndarray, B: np.ndarray,
                              Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """DARE via the Riccati fixed-point iteration

        P <- Q + A'PA - (B'PA)' (R + B'PB)^-1 (B'PA),   P0 = Q.

    Converges to the stabilizing PSD solution for any stabilizable +
    detectable (A, B, Q, R) — the MPS plant qualifies (controllable,
    Q positive definite). Convergence is linear; the generous iteration
    cap covers slow cases (closed-loop spectral radius near 1). Runs only
    at node bootstrap and on dashboard "Apply" — never in the 50 Hz tick
    loop.
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)

    P = Q.copy()
    for _ in range(_DARE_MAX_ITER):
        BtP = B.T @ P
        BtPA = BtP @ A
        S = R + BtP @ B
        P_next = Q + A.T @ P @ A - BtPA.T @ np.linalg.solve(S, BtPA)
        P_next = 0.5 * (P_next + P_next.T)
        delta = float(np.max(np.abs(P_next - P)))
        P = P_next
        if delta <= _DARE_TOL * max(1.0, float(np.max(np.abs(P)))):
            break
    return P
