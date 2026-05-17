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
from pi_nodes.control.mps_reference import build_reference
from pi_nodes.control.state_space_model import StateSpaceModel, zoh_discretize

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda key, default=None: default  # type: ignore


def _mps_ts() -> float:
    """Период дискретизации/цикла МПС — единый источник правды."""
    return float(cfg('mps.plant.Ts', 0.02))


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


def _build_controller(m: MpsMatrices, ts: float) -> tuple[StateSpaceModel, MPCController]:
    """Build state-space model + MPC from the supplied matrices.

    `m.A`, `m.B` — НЕПРЕРЫВНЫЕ канонические матрицы (контракт
    docs/mps/api.md). Они ZOH-дискретизируются при `ts` перед передачей
    в StateSpaceModel / MPCController, которые работают с дискретными Ad/Bd.
    Mirrors what `mps_node` does on Pi for `source="robot"`.
    """
    A, B, C, D, Q_diag, R_diag, u_min, u_max = _matrices_to_arrays(m)
    Ad, Bd = zoh_discretize(A, B, ts)
    plant = StateSpaceModel(Ad=Ad, Bd=Bd, Cd=C, Dd=D, Ts=ts)
    mpc = MPCController(
        Ad=Ad,
        Bd=Bd,
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
    dt: Optional[float] = None,
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
    MpsScenarioResult with status one of (см. spec §4.1-4.3):
        - 'reached'         — все 4 координаты сошлись (|s−D|<ε_s,
                              |v|<ε_v, |θ_err|<ε_θ, |ω|<ε_ω) при t ≥ t_end
        - 'timeout_settle'  — траектория r(t) дошла до t_end, но контур
                              не уложился в settle_timeout (1.5 c)
        - 'timeout'         — run-timeout (t > 1.5·t_end + 2.0 c) ещё до
                              выхода в settling-окно
        - 'error'           — build_reference ValueError, mpc.step
                              exception, NaN/Inf в u/x, или |x| blow-up
    """
    if dt is None:
        dt = _mps_ts()
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
        plant, mpc = _build_controller(matrices, dt)
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

    # Параметры референса (хардкод дефолтов — на compute их обычно нет в
    # config; Pi передаёт свои значения через MQTT-payload для синхронизации).
    a_max_ref = 0.20
    alpha_max_ref = 1.0
    omega_max_ref = 1.0

    try:
        traj = build_reference(
            distance=distance, v_target=v_target,
            target_heading=float(request.target_heading),
            a_max=a_max_ref, alpha_max=alpha_max_ref, omega_max=omega_max_ref,
        )
    except ValueError as exc:
        return MpsScenarioResult(
            run_id=rid, started_at=started_at,
            finished_at=datetime.now(timezone.utc),
            status='error', request=request, matrices_snapshot=matrices,
            telemetry=[], metrics=None,
            error_message=f'reference build failed: {exc}',
        )

    settle_timeout = 1.5
    eps_s, eps_v, eps_theta, eps_omega = 0.005, 0.02, 0.05, 0.05
    bound = max(50.0, 5.0 * distance)
    # Anti-windup на интеграле курсовой ошибки — те же дефолты, что
    # mps_node на Pi (`mps.scenario.e_int_max`, 0.5 рад·с). Симулятор
    # должен по поведению совпадать с роботом, поэтому e_int здесь
    # тоже накапливаем извне, как ∫(−θ_err) dt, и переписываем x[4]
    # после plant.step — иначе модель `ė_int = −θ` гонит интегратор
    # к θ=0 вместо θ=θ_target (см. spec §3.3 _accumulate_eint и
    # pi_nodes/nodes/mps_node.py ~L630).
    e_int_max = 0.5

    telemetry: list[MpsTelemetryPoint] = []
    status = 'timeout'
    error_message: Optional[str] = None
    t = 0.0
    theta_target = traj.theta_start + traj.phi_signed

    for step in range(max_steps):
        r_ref = traj.r(t)
        try:
            u = mpc.step(x, x_ref=r_ref)
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
                r=[float(v) for v in r_ref],
                x_local=float(x[_S_IDX]),  # sim — straight-line: x_local = s
                y_local=0.0,
            )
        )

        if t >= traj.t_end:
            theta_err = (x[2] - theta_target + math.pi) % (2 * math.pi) - math.pi
            if (abs(x[_S_IDX] - distance) < eps_s
                    and abs(x[1]) < eps_v
                    and abs(theta_err) < eps_theta
                    and abs(x[3]) < eps_omega):
                status = 'reached'
                break
            if t > traj.t_end + settle_timeout:
                status = 'timeout_settle'
                error_message = (f'settle timeout: |s−D|={abs(x[_S_IDX]-distance):.4f}, '
                                 f'|v|={abs(x[1]):.4f}, |θ_err|={abs(theta_err):.4f}, '
                                 f'|ω|={abs(x[3]):.4f}')
                break

        if t > max(1.0, 1.5 * traj.t_end + 2.0):
            status = 'timeout'
            error_message = f'run timeout: t={t:.3f} > 1.5·t_end={1.5*traj.t_end:.3f}'
            break

        # Override e_int: mirror Pi (mps_node._accumulate_eint).
        # Канонический plant имеет `ė_int = -θ` в Ad[4,2], что после
        # plant.step добавляет `-θ·dt` к x[4]. Это работает только при
        # r[θ]=0; при non-zero θ-референсе интегратор гонит θ → 0
        # вместо θ_target. На Pi эта же проблема решена тем, что
        # mps_node ПОЛНОСТЬЮ переписывает x_meas[EINT] значением
        # `∫(-θ_err) dt` с anti-windup (spec §3.3, mps_node.py ~L630).
        # Здесь делаем то же самое: запоминаем prev_eint ДО plant.step,
        # пускаем шаг, затем override x[4] на интеграл от θ_err.
        prev_eint = float(x[4])
        x = plant.step(x, u)
        theta_err_acc = (x[2] - r_ref[2] + math.pi) % (2 * math.pi) - math.pi
        new_eint = prev_eint + (-theta_err_acc) * dt
        x[4] = max(-e_int_max, min(e_int_max, new_eint))
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
                        dt: Optional[float] = None) -> list[MpsTelemetryPoint]:
    """Run an unforced step from x=0 with v_target=0.15 for `duration_s`.

    Used by the validate endpoint to surface «what does the closed loop
    actually do» without running a full scenario.
    """
    request = MpsScenarioRequest(distance=5.0, v_target=0.15, source='sim')
    resolved_dt = dt if dt is not None else _mps_ts()
    n_steps = max(1, int(duration_s / resolved_dt))
    result = run_scenario_idealized(
        matrices, request, dt=dt, max_steps=n_steps,
        run_id='validate-step',
    )
    return result.telemetry


def closed_loop_eigenvalues(matrices: MpsMatrices) -> tuple[list[complex], list[complex]]:
    """Return (λ(Ad), λ(Ad − Bd·K_first)) as Python complex lists.

    `matrices.A/B` — НЕПРЕРЫВНЫЕ; ZOH-дискретизируются при mps.plant.Ts
    перед анализом собственных значений.
    """
    A, B, _, _, Q_diag, R_diag, u_min, u_max = _matrices_to_arrays(matrices)
    ts = _mps_ts()
    Ad, Bd = zoh_discretize(A, B, ts)
    eig_open = list(np.linalg.eigvals(Ad))
    try:
        mpc = MPCController(
            Ad=Ad, Bd=Bd,
            Q=np.diag(Q_diag), R=np.diag(R_diag),
            N=int(matrices.horizon_N),
            u_min=u_min, u_max=u_max,
            solver='clip',
        )
        eig_closed = list(np.linalg.eigvals(Ad - Bd @ mpc.K_first))
    except Exception:
        eig_closed = [complex('nan')] * len(eig_open)
    return eig_open, eig_closed


# ── Convenience: quick smoke from CLI ─────────────────────────────────
if __name__ == '__main__':
    # Default matrices from config.yaml — kept self-contained so this
    # script doesn't depend on a running dashboard.
    A_DEFAULT = [
        [0.0,  1.0,             0.0,  0.0,             0.0],
        [0.0, -1.0 / 0.15,      0.0,  0.0,             0.0],
        [0.0,  0.0,             0.0,  1.0,             0.0],
        [0.0,  0.0,             0.0, -1.0 / 0.10,      0.0],
        [0.0,  0.0,            -1.0,  0.0,             0.0],
    ]
    B_DEFAULT = [
        [0.0,         0.0],
        [1.0 / 0.15,  0.0],
        [0.0,         0.0],
        [0.0,         1.0 / 0.10],
        [0.0,         0.0],
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
