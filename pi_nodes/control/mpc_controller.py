"""mpc_controller — Model Predictive Control with explicit-gain + clip projection.

Implements the formulation in chapter 3 of latex_doc/control_theory:
    min_U   sum_{i=0..N-1}(x_i' Q x_i + u_i' R u_i) + x_N' Pf x_N
    s.t.    x_{i+1} = Ad x_i + Bd u_i
            u_min ≤ u_i ≤ u_max

Without active inequality constraints the QP has the closed-form solution
    U* = -H^-1 f(x0) = -K_full @ x0,
and we apply only the first r rows (u_0). Constraints are enforced by
clipping (`solver: "clip"`) — fast and robust, optimal in the unconstrained
region. For full QP solve set `control.mpc.solver: qp` (requires scipy).

K_first is loaded from config (`control.matrices.K_mpc`) if available;
otherwise it is computed online from plant + weights.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda key, default=None: default  # type: ignore


class MPCController:
    """MPC with explicit unconstrained gain plus box-constraint clipping."""

    __slots__ = (
        "Ad", "Bd", "Q", "R", "Pf", "N",
        "u_min", "u_max", "n", "r",
        "Phi", "Gamma", "H", "f_mat", "K_first",
        "_solver",
    )

    def __init__(
        self,
        Ad: Optional[np.ndarray] = None,
        Bd: Optional[np.ndarray] = None,
        Q:  Optional[np.ndarray] = None,
        R:  Optional[np.ndarray] = None,
        Pf: Optional[np.ndarray] = None,
        N:  Optional[int] = None,
        u_min: Optional[Sequence[float]] = None,
        u_max: Optional[Sequence[float]] = None,
        solver: Optional[str] = None,
    ) -> None:
        # ── Resolve plant ────────────────────────────────────────
        if Ad is None or Bd is None:
            from pi_nodes.control.state_space_model import StateSpaceModel
            plant = StateSpaceModel()
            Ad = plant.Ad if Ad is None else Ad
            Bd = plant.Bd if Bd is None else Bd
        self.Ad = np.asarray(Ad, dtype=float)
        self.Bd = np.asarray(Bd, dtype=float)
        self.n, self.r = self.Bd.shape

        # ── Weights ──────────────────────────────────────────────
        if Q is None:
            Q_diag = cfg("control.weights.Q_diag", [10.0, 10.0, 5.0, 1.0, 1.0])
            Q = np.diag(np.asarray(Q_diag, dtype=float))
        if R is None:
            R_diag = cfg("control.weights.R_diag", [1.0, 1.0])
            R = np.diag(np.asarray(R_diag, dtype=float))
        self.Q = np.asarray(Q, dtype=float)
        self.R = np.asarray(R, dtype=float)

        # ── Horizon ──────────────────────────────────────────────
        if N is None:
            N = int(cfg("control.mpc.horizon_N", 10))
        self.N = int(N)
        if self.N < 1:
            raise ValueError(f"MPC horizon must be >= 1, got {self.N}")

        # ── Terminal penalty ─────────────────────────────────────
        if Pf is None:
            Pf_cfg = cfg("control.matrices.Pf", None)
            if Pf_cfg is not None:
                Pf = np.asarray(Pf_cfg, dtype=float)
            else:
                Pf = self._compute_terminal_penalty()
        self.Pf = np.asarray(Pf, dtype=float)

        # ── Limits ───────────────────────────────────────────────
        if u_min is None:
            u_min = cfg("control.limits.u_min", [-0.30, -2.0])
        if u_max is None:
            u_max = cfg("control.limits.u_max", [+0.30, +2.0])
        self.u_min = np.asarray(u_min, dtype=float)
        self.u_max = np.asarray(u_max, dtype=float)

        # ── Solver mode ──────────────────────────────────────────
        if solver is None:
            solver = cfg("control.mpc.solver", "clip")
        self._solver = str(solver).lower()
        if self._solver not in ("clip", "qp"):
            raise ValueError(f"Unknown MPC solver '{solver}'. Use 'clip' or 'qp'.")

        # ── Build lifted dynamics + QP matrices ──────────────────
        self._build_qp_matrices()

        # ── Try to load precomputed K_first from config ──────────
        K_mpc_cfg = cfg("control.matrices.K_mpc", None)
        if K_mpc_cfg is not None:
            K_pre = np.asarray(K_mpc_cfg, dtype=float)
            if K_pre.shape == (self.r, self.n):
                self.K_first = K_pre

    # ── Setup ─────────────────────────────────────────────────────
    def _compute_terminal_penalty(self) -> np.ndarray:
        """If Pf not given, solve DARE for guaranteed-stable terminal cost."""
        from scipy.linalg import solve_discrete_are

        Q_diag = cfg("control.weights.Q_diag", [10.0, 10.0, 5.0, 1.0, 1.0])
        R_diag = cfg("control.weights.R_diag", [1.0, 1.0])
        Q = np.diag(np.asarray(Q_diag, dtype=float))
        R = np.diag(np.asarray(R_diag, dtype=float))
        return solve_discrete_are(self.Ad, self.Bd, Q, R)

    def _build_qp_matrices(self) -> None:
        n, r, N = self.n, self.r, self.N
        Ad, Bd = self.Ad, self.Bd

        # Lifted dynamics: X = Phi x0 + Gamma U
        Phi = np.zeros((n * N, n))
        Gamma = np.zeros((n * N, r * N))
        Apow = np.eye(n)
        for i in range(N):
            Apow = Ad @ Apow                    # A^{i+1}
            Phi[i*n:(i+1)*n, :] = Apow
            for j in range(i + 1):
                block = np.linalg.matrix_power(Ad, i - j) @ Bd
                Gamma[i*n:(i+1)*n, j*r:(j+1)*r] = block

        # Block-diagonal weights
        if N == 1:
            Q_bar = self.Pf
        else:
            Q_bar = np.kron(np.eye(N - 1), self.Q)
            Q_bar = _block_diag(Q_bar, self.Pf)
        R_bar = np.kron(np.eye(N), self.R)

        H = 2.0 * (Gamma.T @ Q_bar @ Gamma + R_bar)
        H = 0.5 * (H + H.T)                     # symmetrise (numerical)
        f_mat = 2.0 * Gamma.T @ Q_bar @ Phi

        # Explicit unconstrained law: U* = -H^-1 f(x0); take first r rows.
        # Define K_first in LQR convention so that the user applies u = -K_first @ x
        # (matches LQRController.step). Then K_first = (H^-1 f_mat)[:r, :].
        U_full = np.linalg.solve(H, f_mat)      # (rN, n) = H^-1 f_mat
        K_first = U_full[:r, :]                 # (r, n), positive — LQR-convention

        self.Phi, self.Gamma = Phi, Gamma
        self.H, self.f_mat = H, f_mat
        self.K_first = K_first

    # ── Hot reload (between scenario runs) ───────────────────────
    def rebuild(
        self,
        *,
        Ad: Optional[np.ndarray] = None,
        Bd: Optional[np.ndarray] = None,
        Q_diag: Optional[Sequence[float]] = None,
        R_diag: Optional[Sequence[float]] = None,
        N: Optional[int] = None,
        u_min: Optional[Sequence[float]] = None,
        u_max: Optional[Sequence[float]] = None,
        Pf: Optional[np.ndarray] = None,
    ) -> None:
        """Recompute Phi/Gamma/H/K_first after model/weights/horizon change.

        Only the supplied arguments override current state — anything
        omitted is preserved. Solver mode (`clip` / `qp`) is preserved.
        Pf is recomputed via DARE if not provided AND any of (Ad/Bd/Q/R)
        changed.

        Designed to be called between scenario runs in `mps_node`. If
        the new matrices fail validation the controller is rolled back
        to the previous configuration (no half-applied state).
        """
        # Snapshot for rollback.
        snapshot = (
            self.Ad.copy(), self.Bd.copy(),
            self.Q.copy(), self.R.copy(),
            self.Pf.copy(),
            self.N,
            self.u_min.copy(), self.u_max.copy(),
            self.Phi.copy(), self.Gamma.copy(),
            self.H.copy(), self.f_mat.copy(),
            self.K_first.copy(),
        )

        try:
            # Plant
            if Ad is not None:
                Ad_arr = np.asarray(Ad, dtype=float)
                if Ad_arr.ndim != 2 or Ad_arr.shape[0] != Ad_arr.shape[1]:
                    raise ValueError(f"Ad must be square 2D, got {Ad_arr.shape}")
                self.Ad = Ad_arr
            if Bd is not None:
                Bd_arr = np.asarray(Bd, dtype=float)
                if Bd_arr.ndim != 2 or Bd_arr.shape[0] != self.Ad.shape[0]:
                    raise ValueError(
                        f"Bd shape {Bd_arr.shape} incompatible with Ad {self.Ad.shape}"
                    )
                self.Bd = Bd_arr
            self.n, self.r = self.Bd.shape

            # Weights
            weights_changed = False
            if Q_diag is not None:
                Q_arr = np.asarray(Q_diag, dtype=float)
                if Q_arr.shape != (self.n,):
                    raise ValueError(f"Q_diag must have {self.n} elements")
                self.Q = np.diag(Q_arr)
                weights_changed = True
            if R_diag is not None:
                R_arr = np.asarray(R_diag, dtype=float)
                if R_arr.shape != (self.r,):
                    raise ValueError(f"R_diag must have {self.r} elements")
                if np.any(R_arr <= 0):
                    raise ValueError("R_diag must be strictly positive")
                self.R = np.diag(R_arr)
                weights_changed = True

            # Horizon
            if N is not None:
                if int(N) < 1:
                    raise ValueError(f"horizon must be >= 1, got {N}")
                self.N = int(N)

            # Limits
            if u_min is not None:
                u_min_arr = np.asarray(u_min, dtype=float)
                if u_min_arr.shape != (self.r,):
                    raise ValueError(f"u_min must have {self.r} elements")
                self.u_min = u_min_arr
            if u_max is not None:
                u_max_arr = np.asarray(u_max, dtype=float)
                if u_max_arr.shape != (self.r,):
                    raise ValueError(f"u_max must have {self.r} elements")
                self.u_max = u_max_arr
            if np.any(self.u_min >= self.u_max):
                raise ValueError("u_min must be < u_max element-wise")

            # Terminal penalty: explicit Pf wins; otherwise re-DARE if
            # plant or weights changed.
            if Pf is not None:
                Pf_arr = np.asarray(Pf, dtype=float)
                if Pf_arr.shape != (self.n, self.n):
                    raise ValueError(f"Pf must be ({self.n}, {self.n})")
                self.Pf = Pf_arr
            elif (Ad is not None) or (Bd is not None) or weights_changed:
                from scipy.linalg import solve_discrete_are
                self.Pf = solve_discrete_are(self.Ad, self.Bd, self.Q, self.R)

            # Re-build lifted dynamics + QP matrices + K_first.
            self._build_qp_matrices()
        except Exception:
            # Rollback on ANY failure — leave the controller in its prior state.
            (
                self.Ad, self.Bd,
                self.Q, self.R,
                self.Pf,
                self.N,
                self.u_min, self.u_max,
                self.Phi, self.Gamma,
                self.H, self.f_mat,
                self.K_first,
            ) = snapshot
            self.n, self.r = self.Bd.shape
            raise

    # ── Online step ──────────────────────────────────────────────
    def step(self, x: np.ndarray, x_ref: Optional[np.ndarray] = None) -> np.ndarray:
        """Solve one MPC step and return the first control u_0."""
        x = np.asarray(x, dtype=float)
        if x_ref is None:
            dx = x
        else:
            dx = x - np.asarray(x_ref, dtype=float)

        if self._solver == "qp":
            return self._solve_qp(dx)

        # Default: explicit clip — u = -K_first @ x  (LQR-convention)
        u = -self.K_first @ dx
        return np.clip(u, self.u_min, self.u_max)

    def _solve_qp(self, dx: np.ndarray) -> np.ndarray:
        """Full QP via scipy. Returns clipped u_0."""
        try:
            from scipy.optimize import minimize
        except ImportError:
            return np.clip(self.K_first @ dx, self.u_min, self.u_max)

        N, r = self.N, self.r
        f = self.f_mat @ dx
        H = self.H
        u_lo = np.tile(self.u_min, N)
        u_hi = np.tile(self.u_max, N)
        bounds = list(zip(u_lo, u_hi))

        # Warm start from explicit solution, then clip into the box
        U0 = -np.linalg.solve(H, f)
        U0 = np.clip(U0, u_lo, u_hi)

        def obj(U):
            return 0.5 * U @ H @ U + f @ U

        def obj_grad(U):
            return H @ U + f

        res = minimize(obj, U0, jac=obj_grad, method="L-BFGS-B", bounds=bounds)
        U_star = res.x if res.success else U0
        return U_star[:r]

    # ── Diagnostics ──────────────────────────────────────────────
    def closed_loop_eigenvalues(self) -> np.ndarray:
        """|λ| < 1 ⇔ closed-loop x_{k+1} = (Ad - Bd K_first) x is stable."""
        return np.linalg.eigvals(self.Ad - self.Bd @ self.K_first)

    def is_stable(self) -> bool:
        return bool(np.all(np.abs(self.closed_loop_eigenvalues()) < 1.0 - 1e-9))

    def __repr__(self) -> str:
        return (
            f"MPCController(N={self.N}, n={self.n}, r={self.r}, "
            f"solver={self._solver}, stable={self.is_stable()})"
        )


def _block_diag(*matrices: np.ndarray) -> np.ndarray:
    """Tiny block-diagonal builder (avoids scipy dependency for hot path)."""
    rows = sum(m.shape[0] for m in matrices)
    cols = sum(m.shape[1] for m in matrices)
    out = np.zeros((rows, cols))
    r0 = c0 = 0
    for m in matrices:
        out[r0:r0 + m.shape[0], c0:c0 + m.shape[1]] = m
        r0 += m.shape[0]
        c0 += m.shape[1]
    return out
