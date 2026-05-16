"""lateral_lqr — outer LQR-loop для коррекции бокового сноса в сценарии MPS.

Работает в каскаде с inner-петлёй (`MPCController` на 5-стейтовой канонической
модели `[s, v, θ, ω, e_int]`). Inner-петля остаётся единственным источником
`cmd_vel`; outer считает оптимальную поправку курсовой ссылки δθ_ref на основе
только латеральной ошибки `e_y` и текущей ошибки курса `θ_err = θ − φ`. Затем
mps_node подаёт inner-MPC обновлённый x_ref с `θ_ref_corr = φ + δθ_ref`.

Подсистема (непрерывная, линеаризация вокруг прямолинейного движения со
скоростью v0):

    ξ = [e_y, θ_err]ᵀ,    u = δθ_ref   (скаляр)

    de_y / dt   = v0 · sin(θ_err)  ≈  v0 · θ_err
    dθ_err / dt = (δθ_ref − θ_err) / τ_inner    (первый порядок к ссылке)

    A_lat = [[0,  v0          ],   B_lat = [[0          ],
             [0, -1/τ_inner    ]]            [1/τ_inner ]]

(Ad_lat, Bd_lat) получаются ZOH-дискретизацией при Ts. K_lat ∈ ℝ^{1×2}
вычисляется через DARE в момент создания контроллера (на старте сценария,
когда известна `v0 = v_target`); горячий путь (тик 50 Hz) — единственное
умножение `δθ_ref = clip(-K_lat · ξ)`.

Подсистема неуправляема при v0 ≈ 0 (стоит на месте — повернуться можно, но
«ехать вбок» нечем). Конструктор поднимает ValueError если v0 < V0_MIN;
mps_node проверяет `v_target ≥ mps.scenario.lateral.v_min` до построения
контроллера.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from pi_nodes.control._linalg import solve_discrete_are
from pi_nodes.control.state_space_model import zoh_discretize


class LateralLqrController:
    """LQR на 2-стейтовой латеральной подсистеме [e_y, θ_err] → δθ_ref."""

    __slots__ = (
        "Ts", "v0", "tau_inner",
        "Ad", "Bd", "Q", "R",
        "K", "delta_theta_max",
    )

    # Минимальная v0 при которой подсистема управляема.
    V0_MIN = 1e-3

    def __init__(
        self,
        *,
        Ts: float,
        v0: float,
        tau_inner: float,
        Q_diag: Sequence[float] = (50.0, 5.0),
        R_diag: Sequence[float] = (1.0,),
        delta_theta_max: float = 0.30,
    ) -> None:
        Ts = float(Ts)
        v0 = float(v0)
        tau_inner = float(tau_inner)
        delta_theta_max = float(delta_theta_max)
        if Ts <= 0:
            raise ValueError(f"Ts must be > 0, got {Ts}")
        if tau_inner <= 0:
            raise ValueError(f"tau_inner must be > 0, got {tau_inner}")
        if v0 < self.V0_MIN:
            raise ValueError(
                f"v0 must be ≥ {self.V0_MIN} (lateral subsystem uncontrollable "
                f"near zero speed), got {v0}"
            )
        if delta_theta_max <= 0:
            raise ValueError(f"delta_theta_max must be > 0, got {delta_theta_max}")

        Q_arr = np.asarray(Q_diag, dtype=float)
        R_arr = np.asarray(R_diag, dtype=float)
        if Q_arr.shape != (2,):
            raise ValueError(f"Q_diag must have 2 elements, got shape {Q_arr.shape}")
        if R_arr.shape != (1,):
            raise ValueError(f"R_diag must have 1 element, got shape {R_arr.shape}")
        if np.any(R_arr <= 0):
            raise ValueError("R_diag must be strictly positive")
        if np.any(Q_arr < 0):
            raise ValueError("Q_diag must be non-negative")

        # Непрерывные A_lat, B_lat и ZOH-дискретизация.
        A_cont = np.array([
            [0.0,  v0          ],
            [0.0, -1.0 / tau_inner],
        ])
        B_cont = np.array([
            [0.0          ],
            [1.0 / tau_inner],
        ])
        Ad, Bd = zoh_discretize(A_cont, B_cont, Ts)
        Q = np.diag(Q_arr)
        R = np.diag(R_arr)

        # DARE → K = (R + B'PB)⁻¹ B'PA. Один раз в __init__, не в hot path.
        P = solve_discrete_are(Ad, Bd, Q, R)
        K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        if not np.all(np.isfinite(K)):
            raise RuntimeError(f"LateralLqr: K contains non-finite entries: {K}")

        self.Ts = Ts
        self.v0 = v0
        self.tau_inner = tau_inner
        self.Ad = Ad
        self.Bd = Bd
        self.Q = Q
        self.R = R
        self.K = K
        self.delta_theta_max = delta_theta_max

    # ── Hot path (50 Hz) ─────────────────────────────────────────────
    def step(self, e_y: float, theta_err: float) -> float:
        """δθ_ref = clip(-K @ [e_y, θ_err], ±delta_theta_max). Скаляр в радианах."""
        xi = np.array([float(e_y), float(theta_err)])
        u = float(-(self.K @ xi)[0])
        if u > self.delta_theta_max:
            return self.delta_theta_max
        if u < -self.delta_theta_max:
            return -self.delta_theta_max
        return u

    # ── Diagnostics ──────────────────────────────────────────────────
    def closed_loop_eigenvalues(self) -> np.ndarray:
        """|λ(Ad − Bd·K)| < 1 ⇔ закрытая латеральная подсистема устойчива."""
        return np.linalg.eigvals(self.Ad - self.Bd @ self.K)

    def is_stable(self) -> bool:
        return bool(np.all(np.abs(self.closed_loop_eigenvalues()) < 1.0 - 1e-9))

    def __repr__(self) -> str:
        return (
            f"LateralLqrController(v0={self.v0:.3f}, tau_inner={self.tau_inner:.3f}, "
            f"K={self.K.tolist()}, delta_max={self.delta_theta_max:.3f}, "
            f"stable={self.is_stable()})"
        )
