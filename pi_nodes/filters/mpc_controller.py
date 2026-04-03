"""
Linear MPC controller for differential-drive robot (Gosha).

Based on linearized state-space model:

    State:   X = [x, y, θ, v, ω]ᵀ   (path-local frame)
    Control: u = [v_dem, ω_dem]ᵀ
    Output:  Y = [x, y, θ]ᵀ

Continuous-time matrices:

    A = [0  0  0   1    0  ]    B = [0    0  ]    C = [1 0 0 0 0]
        [0  0  v₀  0    0  ]        [0    0  ]        [0 1 0 0 0]
        [0  0  0   0    1  ]        [0    0  ]        [0 0 1 0 0]
        [0  0  0  -1/T  0  ]        [1/T  0  ]
        [0  0  0   0  -1/T ]        [0   1/T ]

Where T = motor time constant, v₀ = linearization speed.

Discretized via Euler: Ad = I + Ac·dt, Bd = Bc·dt

Unconstrained QP solved analytically:
    U* = (Γᵀ Q̄ Γ + R̄)⁻¹ Γᵀ Q̄ (Y_ref − Φ x₀)

Constraints applied via clipping (sufficient for this scale).
"""

import numpy as np


class MPCController:
    """Model Predictive Controller for path-local robot control.

    Usage::

        mpc = MPCController(dt=0.05, Np=20, Nc=8)

        # Each control cycle:
        v_dem, omega_dem = mpc.compute(x_state, y_ref, v0)
    """

    def __init__(self,
                 dt: float = 0.05,
                 Np: int = 20,
                 Nc: int = 8,
                 motor_tau: float = 0.25,
                 q_x: float = 1.0,
                 q_y: float = 5.0,
                 q_theta: float = 3.0,
                 r_v: float = 0.1,
                 r_omega: float = 0.1,
                 v_max: float = 0.15,
                 v_min: float = -0.15,
                 omega_max: float = 1.5,
                 omega_min: float = -1.5):
        self.dt = dt
        self.Np = Np
        self.Nc = Nc
        self.T = motor_tau

        self.nx = 5
        self.nu = 2
        self.ny = 3

        # Output matrix C (constant)
        self.C = np.zeros((self.ny, self.nx))
        self.C[0, 0] = 1.0  # x
        self.C[1, 1] = 1.0  # y
        self.C[2, 2] = 1.0  # θ

        # Weight matrices
        self.Q = np.diag([q_x, q_y, q_theta])
        self.R = np.diag([r_v, r_omega])
        self.Q_bar = np.kron(np.eye(Np), self.Q)
        self.R_bar = np.kron(np.eye(Nc), self.R)

        # Constraints
        self.v_max = v_max
        self.v_min = v_min
        self.omega_max = omega_max
        self.omega_min = omega_min

        # Cache
        self._last_v0 = None
        self._Phi = None    # (Np·ny × nx)
        self._Gamma = None  # (Np·ny × Nc·nu)
        self._K_mpc = None  # gain matrix

    def _build_prediction_matrices(self, v0: float):
        """Rebuild Φ, Γ, K for a new linearization speed v₀."""
        nx, nu, ny = self.nx, self.nu, self.ny
        Np, Nc = self.Np, self.Nc
        dt, T = self.dt, self.T
        C = self.C

        # Continuous-time A, B
        Ac = np.zeros((nx, nx))
        Ac[0, 3] = 1.0           # ẋ = v
        Ac[1, 2] = v0            # ẏ = v₀·θ
        Ac[2, 4] = 1.0           # θ̇ = ω
        Ac[3, 3] = -1.0 / T     # v̇ = -v/T + v_dem/T
        Ac[4, 4] = -1.0 / T     # ω̇ = -ω/T + ω_dem/T

        Bc = np.zeros((nx, nu))
        Bc[3, 0] = 1.0 / T      # v_dem / T
        Bc[4, 1] = 1.0 / T      # ω_dem / T

        # Euler discretisation
        Ad = np.eye(nx) + Ac * dt
        Bd = Bc * dt

        # Φ = [C·Ad; C·Ad²; ...; C·Adᴺᵖ]
        Phi = np.zeros((Np * ny, nx))
        Ad_pow = Ad.copy()
        for k in range(Np):
            Phi[k * ny:(k + 1) * ny, :] = C @ Ad_pow
            Ad_pow = Ad_pow @ Ad

        # Pre-compute C·Adⁱ·Bd for i = 0 .. Np-1
        CA_Bd = []
        Ad_pow = np.eye(nx)
        for k in range(Np):
            CA_Bd.append(C @ Ad_pow @ Bd)
            Ad_pow = Ad_pow @ Ad

        # Γ — lower triangular block Toeplitz
        Gamma = np.zeros((Np * ny, Nc * nu))
        for i in range(Np):
            for j in range(min(i + 1, Nc)):
                Gamma[i * ny:(i + 1) * ny, j * nu:(j + 1) * nu] = CA_Bd[i - j]

        self._Phi = Phi
        self._Gamma = Gamma

        # Gain: K = (Γᵀ Q̄ Γ + R̄)⁻¹ Γᵀ Q̄
        GtQ = Gamma.T @ self.Q_bar
        H = GtQ @ Gamma + self.R_bar
        self._K_mpc = np.linalg.solve(H, GtQ)
        self._last_v0 = v0

    def compute(self, x_state, y_ref, v0):
        """Compute optimal [v_dem, ω_dem].

        Args:
            x_state: array(5,) — [x, y, θ, v, ω] in path-local frame.
            y_ref:   array(Np, 3) — reference [x_ref, y_ref, θ_ref] per step.
            v0:      linearization speed (use target speed or current v).

        Returns:
            (v_dem, omega_dem): first optimal control, clipped to constraints.
        """
        # Rebuild matrices if linearization point changed
        if self._last_v0 is None or abs(v0 - self._last_v0) > 0.01:
            self._build_prediction_matrices(v0)

        x = np.asarray(x_state, dtype=np.float64)
        Y_ref = y_ref.flatten()

        # Free response (no control)
        Y_free = self._Phi @ x

        # Optimal control sequence (unconstrained)
        U_opt = self._K_mpc @ (Y_ref - Y_free)

        # First control action
        v_dem = float(np.clip(U_opt[0], self.v_min, self.v_max))
        omega_dem = float(np.clip(U_opt[1], self.omega_min, self.omega_max))

        return v_dem, omega_dem
