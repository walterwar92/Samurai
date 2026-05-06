"""
mps_runner — идеальный симулятор сценария «D метров вперёд» для модуля МПС.

Модель: x[k+1] = Ad·x + Bd·u, никаких шумов / motor lag / slip. Ровно та
же модель что разворачивается на Pi (`pi_nodes/control/state_space_model.py`),
тот же регулятор (`pi_nodes/control/mpc_controller.py`). Запускается
синхронно из REST-роутера для `source="sim"` (~100 ms wall-clock на D=2 м).

Использование:
    from compute_node.dashboard.schemas.mps import MpsMatrices, MpsScenarioRequest
    from compute_node.mps_runner import run_scenario_idealized

    result = run_scenario_idealized(matrices, request)
    # result.status ∈ {reached, timeout, error}
    # result.metrics — overshoot/settling/...
    # result.telemetry — точки 50 Гц до finish

Контракт: docs/mps/api.md, единый source of truth — Pydantic-схемы.
"""
from __future__ import annotations

import math
import time
from datetime import datetime, timezone
from typing import Optional
from uuid import uuid4

import numpy as np

from compute_node.dashboard.schemas.mps import (
    MpsMatrices,
    MpsMetrics,
    MpsScenarioRequest,
    MpsScenarioResult,
    MpsTelemetryPoint,
)
from pi_nodes.control.mpc_controller import MPCController
from pi_nodes.control.state_space_model import StateSpaceModel


# Состояние на индексе 0 — пройденная дистанция s.
_S_IDX = 0
# Состояние на индексе 1 — продольная скорость v.
_V_IDX = 1
# Состояние на индексе 3 — угловая скорость ω.
_OMEGA_IDX = 3
# Tolerance settling-time: |s − D| < этого считается «пришли».
_SETTLE_TOL = 0.02


def _matrices_to_arrays(m: MpsMatrices) -> tuple[np.ndarray, ...]:
    return (
        np.asarray(m.A, dtype=float),
        np.asarray(m.B, dtype=float),
        np.asarray(m.C, dtype=float),
        np.asarray(m.D, dtype=float),
        np.asarray(m.Q_diag, dtype=float),
        np.asarray(m.R_diag, dtype=float),
        np.asarray(m.u_min, dtype=float),
        np.asarray(m.u_max, dtype=float),
    )


def _build_controller(m: MpsMatrices) -> tuple[StateSpaceModel, MPCController]:
    """Build state-space model + MPC from the supplied matrices.

    Mirrors what `mps_node` does on Pi for `source="robot"`. Both paths
    must produce identical control signals for identical initial state.
    """
    A, B, C, D, Q_diag, R_diag, u_min, u_max = _matrices_to_arrays(m)
    plant = StateSpaceModel(Ad=A, Bd=B, Cd=C, Dd=D, Ts=0.05)
    mpc = MPCController(
        Ad=A,
        Bd=B,
        Q=np.diag(Q_diag),
        R=np.diag(R_diag),
        N=int(m.horizon_N),
        u_min=u_min,
        u_max=u_max,
        solver='clip',
    )
    return plant, mpc


def _compute_metrics(
    telemetry: list[MpsTelemetryPoint],
    distance: float,
    R_diag: np.ndarray,
    dt: float,
) -> MpsMetrics:
    if not telemetry:
        return MpsMetrics(
            overshoot=0.0, settling_time=0.0, control_energy=0.0,
            ss_error=distance, peak_v=0.0, peak_omega=0.0,
        )
    s_arr = np.array([p.x[_S_IDX] for p in telemetry])
    v_arr = np.array([p.x[_V_IDX] for p in telemetry])
    omega_arr = np.array([p.x[_OMEGA_IDX] for p in telemetry])
    u_arr = np.array([p.u for p in telemetry])
    t_arr = np.array([p.t for p in telemetry])

    # overshoot = max(s) − D, обрезано снизу нулём
    overshoot = float(max(0.0, s_arr.max() - distance))

    # settling_time: первое t где |s − D| ≤ _SETTLE_TOL и остаётся таким до конца
    err = np.abs(s_arr - distance)
    inside = err <= _SETTLE_TOL
    settling_time = 0.0
    if inside.any():
        # Скан с конца: ищем самый ранний k от которого все последующие inside.
        last_outside = -1
        for k, ok in enumerate(inside):
            if not ok:
                last_outside = k
        # «Установившееся» начинается с last_outside+1
        settled_at = last_outside + 1
        if settled_at < len(t_arr):
            settling_time = float(t_arr[settled_at])
        else:
            settling_time = float(t_arr[-1])

    # control_energy = Σ uᵀRu·dt
    R_quad = np.einsum('ki,i,ki->k', u_arr, R_diag, u_arr)
    control_energy = float(np.sum(R_quad) * dt)

    ss_error = float(abs(s_arr[-1] - distance))
    peak_v = float(np.max(np.abs(v_arr)))
    peak_omega = float(np.max(np.abs(omega_arr)))

    return MpsMetrics(
        overshoot=overshoot,
        settling_time=settling_time,
        control_energy=control_energy,
        ss_error=ss_error,
        peak_v=peak_v,
        peak_omega=peak_omega,
    )


def _isfinite_all(*arrays: np.ndarray) -> bool:
    return all(np.all(np.isfinite(a)) for a in arrays)


def run_scenario_idealized(
    matrices: MpsMatrices,
    request: MpsScenarioRequest,
    dt: float = 0.02,
    max_steps: int = 10_000,
    run_id: Optional[str] = None,
    initial_state: Optional[np.ndarray] = None,
) -> MpsScenarioResult:
    """Run the «forward D meters» scenario in the ideal sim.

    Parameters
    ----------
    matrices : MpsMatrices
        Snapshot used to build the controller. Stored verbatim in the
        result for reproducibility.
    request : MpsScenarioRequest
        D, v_target, source. `source` is preserved as-is in the result
        (caller decides; this function never publishes MQTT).
    dt : float
        Tick period. Should match `mps_node.tick_dt` (default 0.02 s = 50 Hz).
    max_steps : int
        Hard cap on iterations to guard against runaway loops.
    run_id : str | None
        Override the auto-generated id (used by replay).
    initial_state : np.ndarray | None
        x[0]. Default zero — fresh start.

    Returns
    -------
    MpsScenarioResult with status one of:
        - 'reached'  — s ≥ D − 0.05
        - 'timeout'  — wall-clock budget t > 3·D / v_target
        - 'error'    — NaN/Inf in u or x, or |x| blows up
    """
    started_at = datetime.now(timezone.utc)
    rid = run_id or f"sim-{started_at.strftime('%Y%m%dT%H%M%SZ')}-{uuid4().hex[:6]}"

    distance = float(request.distance)
    v_target = float(request.v_target)
    n = 5
    r = 2

    # Initial state
    x = np.zeros(n) if initial_state is None else np.asarray(initial_state, dtype=float).copy()
    if x.shape != (n,):
        return MpsScenarioResult(
            run_id=rid,
            started_at=started_at,
            finished_at=datetime.now(timezone.utc),
            status='error',
            request=request,
            matrices_snapshot=matrices,
            telemetry=[],
            metrics=None,
            error_message=f'initial_state shape {x.shape} ≠ ({n},)',
        )

    # Build plant + controller; on failure (bad matrices, scipy error) →
    # report status='error' with the raised message rather than crashing.
    try:
        plant, mpc = _build_controller(matrices)
    except Exception as exc:
        return MpsScenarioResult(
            run_id=rid,
            started_at=started_at,
            finished_at=datetime.now(timezone.utc),
            status='error',
            request=request,
            matrices_snapshot=matrices,
            telemetry=[],
            metrics=None,
            error_message=f'controller build failed: {exc}',
        )

    R_diag = np.asarray(matrices.R_diag, dtype=float)
    timeout_t = max(1.0, 3.0 * distance / max(v_target, 1e-6))
    bound = max(50.0, 5.0 * distance)  # |x| > bound ⇒ blow-up
    epsilon_reach = 0.05               # «достиг цели» если осталось ≤5 см

    telemetry: list[MpsTelemetryPoint] = []
    status = 'timeout'
    error_message: Optional[str] = None

    t = 0.0
    for step in range(max_steps):
        # Reference: ramp s_ref to D, hold v_target, others zero.
        s_ref = min(distance, t * v_target)
        x_ref = np.array([s_ref, v_target, 0.0, 0.0, 0.0])

        try:
            u = mpc.step(x, x_ref=x_ref)
        except Exception as exc:
            status = 'error'
            error_message = f'mpc.step failed at t={t:.3f}: {exc}'
            break

        if not _isfinite_all(u, x):
            status = 'error'
            error_message = f'NaN/Inf at t={t:.3f} (u={u.tolist()}, x={x.tolist()})'
            break
        if np.max(np.abs(x)) > bound:
            status = 'error'
            error_message = f'|x| > {bound} at t={t:.3f} — instability'
            break

        # Output for UI
        try:
            y = plant.output(x, u)
        except Exception:
            y = x.copy()

        s_remaining = max(0.0, distance - x[_S_IDX])
        telemetry.append(
            MpsTelemetryPoint(
                t=round(t, 6),
                x=[float(v) for v in x],
                u=[float(v) for v in u],
                y=[float(v) for v in y],
                s_remaining=float(s_remaining),
            )
        )

        # Reached?
        if x[_S_IDX] >= distance - epsilon_reach:
            status = 'reached'
            break

        # Timeout?
        if t > timeout_t:
            status = 'timeout'
            break

        # Propagate (ideal: no noise, no slip, no motor lag)
        x = plant.step(x, u)
        t += dt

    metrics = _compute_metrics(telemetry, distance, R_diag, dt)
    finished_at = datetime.now(timezone.utc)
    return MpsScenarioResult(
        run_id=rid,
        started_at=started_at,
        finished_at=finished_at,
        status=status,  # type: ignore[arg-type]
        request=request,
        matrices_snapshot=matrices,
        telemetry=telemetry,
        metrics=metrics if status != 'error' else None,
        error_message=error_message,
    )


# ── Validate helper (used by /api/v1/mps/validate) ────────────────────
def short_step_response(matrices: MpsMatrices, duration_s: float = 2.0,
                        dt: float = 0.02) -> list[MpsTelemetryPoint]:
    """Run an unforced step from x=0 with v_target=0.15 for `duration_s`.

    Used by the validate endpoint to surface «what does the closed loop
    actually do» without running a full scenario.
    """
    request = MpsScenarioRequest(distance=5.0, v_target=0.15, source='sim')
    n_steps = max(1, int(duration_s / dt))
    result = run_scenario_idealized(
        matrices, request, dt=dt, max_steps=n_steps,
        run_id='validate-step',
    )
    return result.telemetry


def closed_loop_eigenvalues(matrices: MpsMatrices) -> tuple[list[complex], list[complex]]:
    """Return (λ(Ad), λ(Ad − Bd·K_first)) as Python complex lists."""
    A, B, _, _, Q_diag, R_diag, u_min, u_max = _matrices_to_arrays(matrices)
    eig_open = list(np.linalg.eigvals(A))
    try:
        mpc = MPCController(
            Ad=A, Bd=B,
            Q=np.diag(Q_diag), R=np.diag(R_diag),
            N=int(matrices.horizon_N),
            u_min=u_min, u_max=u_max,
            solver='clip',
        )
        eig_closed = list(np.linalg.eigvals(A - B @ mpc.K_first))
    except Exception:
        eig_closed = [complex('nan')] * len(eig_open)
    return eig_open, eig_closed


# ── Convenience: quick smoke from CLI ─────────────────────────────────
if __name__ == '__main__':
    # Default matrices from config.yaml — kept self-contained so this
    # script doesn't depend on a running dashboard.
    A_DEFAULT = [
        [1, 0, 0, 0.0425203, 0],
        [0, 1, 0.01, 0, 0.000213061],
        [0, 0, 1, 0, 0.0393469],
        [0, 0, 0, 0.716531, 0],
        [0, 0, 0, 0, 0.606531],
    ]
    B_DEFAULT = [
        [0.0074797, 0],
        [0, 3.69387e-05],
        [0, 0.0106531],
        [0.283469, 0],
        [0, 0.393469],
    ]
    m = MpsMatrices(
        A=A_DEFAULT, B=B_DEFAULT,
        C=[[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)],
        D=[[0.0, 0.0] for _ in range(5)],
        Q_diag=[10, 10, 5, 1, 1], R_diag=[1, 1],
        horizon_N=10,
        u_min=[-0.30, -2.0], u_max=[0.30, 2.0],
    )
    req = MpsScenarioRequest(distance=2.0, v_target=0.15, source='sim')
    t0 = time.time()
    res = run_scenario_idealized(m, req)
    dt_wall = time.time() - t0
    print(f"status={res.status}  steps={len(res.telemetry)}  "
          f"wall={dt_wall * 1000:.1f}ms")
    if res.metrics:
        print(f"metrics: ss_error={res.metrics.ss_error:.4f}  "
              f"overshoot={res.metrics.overshoot:.4f}  "
              f"settling={res.metrics.settling_time:.2f}s")
    if res.error_message:
        print(f"error: {res.error_message}")
