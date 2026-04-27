"""lqr_controller — discrete LQR state-feedback controller.

Reads K from config (`control.matrices.K`). If `scipy` is available and
no K is provided, falls back to solving DARE online so the module is
useful even without running the MATLAB pipeline first.

Usage
-----
>>> from pi_nodes.control.lqr_controller import LQRController
>>> ctrl = LQRController()                # K from config.yaml
>>> u = ctrl.step(x, x_ref=None)          # u = -K (x - x_ref), clipped
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda key, default=None: default  # type: ignore


class LQRController:
    """u_k = clip(-K @ (x_k - x_ref_k), u_min, u_max)."""

    __slots__ = ("K", "u_min", "u_max", "n", "r")

    def __init__(
        self,
        K: Optional[Sequence[Sequence[float]]] = None,
        u_min: Optional[Sequence[float]] = None,
        u_max: Optional[Sequence[float]] = None,
    ) -> None:
        K_arr = self._load_K(K)
        self.K = np.asarray(K_arr, dtype=float)

        self.r, self.n = self.K.shape

        if u_min is None:
            u_min = cfg("control.limits.u_min", [-0.30, -2.0])
        if u_max is None:
            u_max = cfg("control.limits.u_max", [+0.30, +2.0])
        self.u_min = np.asarray(u_min, dtype=float)
        self.u_max = np.asarray(u_max, dtype=float)

        if self.u_min.shape != (self.r,) or self.u_max.shape != (self.r,):
            raise ValueError(
                f"u_min/u_max shape mismatch: K is ({self.r}x{self.n}), "
                f"u_min={self.u_min.shape}, u_max={self.u_max.shape}"
            )

    @staticmethod
    def _load_K(K_arg: Optional[Sequence[Sequence[float]]]) -> np.ndarray:
        if K_arg is not None:
            return np.asarray(K_arg, dtype=float)

        K_cfg = cfg("control.matrices.K", None)
        if K_cfg is not None:
            return np.asarray(K_cfg, dtype=float)

        # Fallback: compute from plant params using scipy.linalg.solve_discrete_are
        return LQRController._solve_dare_from_config()

    @staticmethod
    def _solve_dare_from_config() -> np.ndarray:
        """Compute LQR K by solving DARE on the fly (used if config has no K)."""
        from scipy.linalg import solve_discrete_are

        from pi_nodes.control.state_space_model import StateSpaceModel

        plant = StateSpaceModel()
        Ad, Bd = plant.Ad, plant.Bd

        Q_diag = cfg("control.weights.Q_diag", [10.0, 10.0, 5.0, 1.0, 1.0])
        R_diag = cfg("control.weights.R_diag", [1.0, 1.0])
        Q = np.diag(np.asarray(Q_diag, dtype=float))
        R = np.diag(np.asarray(R_diag, dtype=float))

        P = solve_discrete_are(Ad, Bd, Q, R)
        # K = (R + B' P B)^-1 B' P A
        K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        return K

    # ── Online step ──────────────────────────────────────────────
    def step(self, x: np.ndarray, x_ref: Optional[np.ndarray] = None) -> np.ndarray:
        """Compute clipped LQR control."""
        x = np.asarray(x, dtype=float)
        if x_ref is None:
            dx = x
        else:
            dx = x - np.asarray(x_ref, dtype=float)
        u = -self.K @ dx
        return np.clip(u, self.u_min, self.u_max)

    def __repr__(self) -> str:
        return f"LQRController(K={self.K.shape}, u_min={self.u_min.tolist()}, u_max={self.u_max.tolist()})"
