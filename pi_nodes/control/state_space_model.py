"""state_space_model — discrete linear plant model x[k+1] = Ad x[k] + Bd u[k].

Reads matrices from config.yaml (`control.matrices.{A,B}`). The matrices
are calculated offline by `matlab/main.m` and exported via `export_to_yaml.m`.

State vector (n=5):  [px, py, theta, v, omega]
Control vector (r=2): [u_v, u_omega]

This module is intentionally minimal — a numerical convenience around the
discrete update rule. Both LQRController and MPCController take an instance
of this model so they share the same source of truth for (Ad, Bd).
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda key, default=None: default  # type: ignore


def _continuous_AB(v0: float, tau_v: float, tau_w: float) -> tuple[np.ndarray, np.ndarray]:
    """Build linearised A, B around forward motion at v0."""
    A = np.array([
        [0.0, 0.0, 0.0,        1.0,         0.0       ],
        [0.0, 0.0, v0,         0.0,         0.0       ],
        [0.0, 0.0, 0.0,        0.0,         1.0       ],
        [0.0, 0.0, 0.0,       -1.0/tau_v,   0.0       ],
        [0.0, 0.0, 0.0,        0.0,        -1.0/tau_w ],
    ])
    B = np.array([
        [0.0,        0.0       ],
        [0.0,        0.0       ],
        [0.0,        0.0       ],
        [1.0/tau_v,  0.0       ],
        [0.0,        1.0/tau_w ],
    ])
    return A, B


def _zoh_discretize(A: np.ndarray, B: np.ndarray, Ts: float) -> tuple[np.ndarray, np.ndarray]:
    """Exact ZOH discretization via block matrix exponential."""
    from scipy.linalg import expm  # local import — scipy is heavy

    n, r = A.shape[0], B.shape[1]
    M = np.zeros((n + r, n + r))
    M[:n, :n] = A
    M[:n, n:] = B
    M_d = expm(M * Ts)
    Ad = M_d[:n, :n]
    Bd = M_d[:n, n:]
    return Ad, Bd


class StateSpaceModel:
    """Discrete linear plant: x[k+1] = Ad @ x[k] + Bd @ u[k].

    Matrices come from config.yaml when constructed without arguments.
    Pass explicit Ad/Bd to override (used by tests and design scripts).
    """

    __slots__ = ("Ad", "Bd", "n", "r", "Ts")

    def __init__(
        self,
        Ad: Optional[np.ndarray] = None,
        Bd: Optional[np.ndarray] = None,
        Ts: Optional[float] = None,
    ) -> None:
        if Ad is None or Bd is None:
            Ad, Bd, Ts_cfg = self._load_or_compute()
            if Ts is None:
                Ts = Ts_cfg
        self.Ad = np.asarray(Ad, dtype=float)
        self.Bd = np.asarray(Bd, dtype=float)
        self.n, self.r = self.Bd.shape
        self.Ts = float(Ts) if Ts is not None else 0.05

        if self.Ad.shape != (self.n, self.n):
            raise ValueError(f"Ad shape {self.Ad.shape} incompatible with Bd {self.Bd.shape}")

    @staticmethod
    def _load_or_compute() -> tuple[np.ndarray, np.ndarray, float]:
        """Try loading from config; fall back to computing from plant params."""
        Ts = float(cfg("control.plant.Ts", 0.05))
        A_cfg = cfg("control.matrices.A", None)
        B_cfg = cfg("control.matrices.B", None)
        if A_cfg is not None and B_cfg is not None:
            return np.array(A_cfg, dtype=float), np.array(B_cfg, dtype=float), Ts

        v0    = float(cfg("control.plant.v0",    0.20))
        tau_v = float(cfg("control.plant.tau_v", 0.15))
        tau_w = float(cfg("control.plant.tau_w", 0.10))
        A, B = _continuous_AB(v0, tau_v, tau_w)
        Ad, Bd = _zoh_discretize(A, B, Ts)
        return Ad, Bd, Ts

    # ── Discrete propagation ───────────────────────────────────────
    def step(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Single discrete step: x_{k+1} = Ad x + Bd u (no clipping)."""
        return self.Ad @ x + self.Bd @ u

    def rollout(self, x0: np.ndarray, U: np.ndarray) -> np.ndarray:
        """Apply a sequence of controls. U is shape (N, r), returns (N+1, n)."""
        N = U.shape[0]
        X = np.zeros((N + 1, self.n))
        X[0] = x0
        for k in range(N):
            X[k + 1] = self.step(X[k], U[k])
        return X

    # ── Diagnostic helpers ─────────────────────────────────────────
    def is_stable(self) -> bool:
        """Open-loop stable iff all |λ(Ad)| < 1."""
        return bool(np.all(np.abs(np.linalg.eigvals(self.Ad)) < 1.0 - 1e-9))

    def controllability_rank(self) -> int:
        """rank([B, AB, A²B, ..., A^{n-1}B])."""
        Sy = np.hstack([np.linalg.matrix_power(self.Ad, i) @ self.Bd for i in range(self.n)])
        return int(np.linalg.matrix_rank(Sy))

    def is_controllable(self) -> bool:
        return self.controllability_rank() == self.n

    def __repr__(self) -> str:
        return f"StateSpaceModel(n={self.n}, r={self.r}, Ts={self.Ts}, stable={self.is_stable()})"


# Default robot operating point (used when no config overrides) ──────
def default_v0() -> float:
    return float(cfg("control.plant.v0", 0.20))


def default_state_for_cmd_vel(linear_x: float, angular_z: float, theta: float = 0.0) -> np.ndarray:
    """Convert a (linear_x, angular_z, theta) tuple into a state-space vector
    suitable for feeding the regulator. px, py default to 0 (controller is
    used in tracking-error coordinates)."""
    return np.array([0.0, 0.0, float(theta), float(linear_x), float(angular_z)])
