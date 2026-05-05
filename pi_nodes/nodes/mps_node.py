#!/usr/bin/env python3
"""
mps_node — Pi-side orchestrator для модуля МПС (Модель Пространства
Состояний). Выполняет сценарий «проехать D метров вперёд» под управлением
существующего MPCController, публикует телеметрию 50 Гц, поддерживает
abort и hot-reload матриц **между** прогонами.

Subscribed (samurai/{robot_id}/...):
  • mps/matrices/set     — compute → Pi: новый MpsMatrices.
                           reload() + rebuild() + ack 'mps/matrices/applied'.
  • mps/scenario/run     — compute → Pi: {run_id, request}. Pre-validate;
                           вход в FSM-state DRIVE_FORWARD_MPS, старт tick-loop.
  • mps/scenario/abort   — compute → Pi: {run_id}. Emergency stop, FSM → IDLE,
                           cmd_vel=[0,0]×3, status='aborted'.
  • odom                 — обновляет x_meas (s, v, θ, ω, e_int).

Published (samurai/{robot_id}/...):
  • mps/matrices/applied  QoS 1 — ack успешного reload+rebuild.
  • mps/telemetry         QoS 0 — 50 Гц во время RUNNING.
  • mps/scenario/finished QoS 1 — итоговый MpsScenarioResult без telemetry.
  • mps/error             QoS 1 — NaN/instability/watchdog/precondition.
  • cmd_vel               QoS 0 — управление от MPC во время RUNNING.
  • status                QoS 0 — fsm_state включая 'DRIVE_FORWARD_MPS'.

FSM:
  IDLE → DRIVE_FORWARD_MPS → IDLE
  В состоянии DRIVE_FORWARD_MPS игнорируются: voice cmds, ball detections,
  joystick. Только 'abort' переключает state (см. fsm_node.py).

Контракт payload-ов: docs/mps/api.md, схемы — compute_node/dashboard/schemas/mps.py.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from datetime import datetime, timezone
from typing import Any, Optional

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from pi_nodes.control.mpc_controller import MPCController
from pi_nodes.control.state_space_model import StateSpaceModel
from pi_nodes.mqtt_node import MqttNode

# Tolerance: «достиг цели» если осталось ≤ этого (метры).
_REACH_EPS = 0.05

# Состояние x = [s, v, θ, ω, e_int]
_S, _V, _THETA, _OMEGA, _EINT = 0, 1, 2, 3, 4

# Watchdog: сколько подряд тиков без свежей одометрии до abort.
_WATCHDOG_TICKS = 3


class _RunState:
    """Локальное состояние active run (только внутри mps_node).

    Не путать с DashboardState на ноуте — это совсем разное. Здесь только
    то, что нужно tick-loop'у.
    """
    __slots__ = (
        'run_id', 'distance', 'v_target', 'started_at',
        'telemetry', 't',
        'no_odom_ticks',
    )

    def __init__(self, run_id: str, distance: float, v_target: float):
        self.run_id = run_id
        self.distance = distance
        self.v_target = v_target
        self.started_at = datetime.now(timezone.utc)
        self.telemetry: list[dict] = []
        self.t = 0.0
        self.no_odom_ticks = 0


class MpsNode(MqttNode):
    """Orchestrator для MPS-сценариев — единственный источник cmd_vel
    в state DRIVE_FORWARD_MPS."""

    def __init__(self, **kwargs):
        super().__init__('mps_node', **kwargs)

        # Защищает state от race между MQTT-callback и timer-tick.
        self._lock = threading.RLock()

        # Source-of-truth модели: лениво грузим из config.yaml через
        # обычный StateSpaceModel(). Если matrices.A/B заданы — используем,
        # иначе пересчитываем из plant params.
        try:
            self._plant = StateSpaceModel()
            self._mpc = MPCController()
        except Exception as exc:
            self.log_error('MPS: failed to bootstrap model/controller: %s', exc)
            raise

        self._tick_dt = float(self._cfg('mps.tick_dt', 0.02))
        self._distance_max = float(self._cfg('mps.scenario.distance_max', 5.0))
        self._v_target_max = float(self._cfg('mps.scenario.v_target_max', 0.30))
        self._omega_max_fwd = float(self._cfg('mps.scenario.omega_max_in_forward', 0.5))

        # x_meas от position_fusion (через odom MQTT). Атомарно read by tick.
        self._x_meas = np.zeros(5)
        self._x_meas_ts = 0.0

        self._run: Optional[_RunState] = None
        self._fsm_state = 'IDLE'

        # Subs
        self.subscribe('mps/matrices/set', self._on_matrices_set, qos=1)
        self.subscribe('mps/scenario/run', self._on_scenario_run, qos=1)
        self.subscribe('mps/scenario/abort', self._on_scenario_abort, qos=1)
        self.subscribe('odom', self._on_odom, qos=0)

        # Tick — будет no-op в IDLE, активный в DRIVE_FORWARD_MPS.
        self.create_timer(self._tick_dt, self._tick)
        # Periodic status (полезно для UI / тестов).
        self.create_timer(1.0, self._publish_status)

        self.log_info(
            'mps_node started — tick_dt=%.3f, distance_max=%.2f, v_target_max=%.2f',
            self._tick_dt, self._distance_max, self._v_target_max,
        )

    # ── Config helper (defensive against missing config_loader) ────────
    @staticmethod
    def _cfg(key: str, default: Any) -> Any:
        try:
            from config_loader import cfg as _cfg
            return _cfg(key, default)
        except ImportError:
            return default

    # ── Public introspection ───────────────────────────────────────────
    @property
    def fsm_state(self) -> str:
        return self._fsm_state

    @property
    def is_running(self) -> bool:
        return self._fsm_state == 'DRIVE_FORWARD_MPS'

    # ── /matrices/set handler ──────────────────────────────────────────
    def _on_matrices_set(self, topic: str, payload):
        if not isinstance(payload, dict):
            self._publish_error('precondition', 'matrices/set: bad payload type')
            return
        if self.is_running:
            self._publish_error(
                'precondition',
                'matrices/set rejected: run is in progress (apply between runs)',
            )
            return
        try:
            A = np.asarray(payload['A'], dtype=float)
            B = np.asarray(payload['B'], dtype=float)
            C = np.asarray(payload['C'], dtype=float) if 'C' in payload else None
            D = np.asarray(payload['D'], dtype=float) if 'D' in payload else None
            Q = np.asarray(payload['Q_diag'], dtype=float)
            R = np.asarray(payload['R_diag'], dtype=float)
            N = int(payload['horizon_N'])
            u_min = np.asarray(payload['u_min'], dtype=float)
            u_max = np.asarray(payload['u_max'], dtype=float)
        except (KeyError, TypeError, ValueError) as exc:
            self._publish_error('precondition', f'matrices/set: {exc}')
            return

        with self._lock:
            try:
                self._plant.reload(Ad=A, Bd=B, Cd=C, Dd=D)
                self._mpc.rebuild(
                    Ad=A, Bd=B, Q_diag=Q, R_diag=R, N=N,
                    u_min=u_min, u_max=u_max,
                )
            except Exception as exc:
                self._publish_error('precondition', f'matrices apply failed: {exc}')
                return

        # Ack
        self.publish('mps/matrices/applied', {
            'matrices': payload,
            'applied_at': datetime.now(timezone.utc).isoformat(),
            'schema_version': payload.get('schema_version', '1.0'),
        }, qos=1)
        self.log_info('mps: matrices applied (N=%d, λ_open=%s)',
                      N,
                      [f'{abs(z):.3f}' for z in np.linalg.eigvals(A)])

    # ── /scenario/run handler ─────────────────────────────────────────
    def _on_scenario_run(self, topic: str, payload):
        if not isinstance(payload, dict):
            self._publish_error('precondition', 'scenario/run: bad payload')
            return
        run_id = str(payload.get('run_id', ''))
        request = payload.get('request') or {}
        try:
            distance = float(request['distance'])
            v_target = float(request['v_target'])
        except (KeyError, TypeError, ValueError) as exc:
            self._publish_error('precondition', f'scenario/run: bad request: {exc}',
                                run_id=run_id)
            return

        # Pre-validate against safety caps
        if not (0 < distance <= self._distance_max):
            self._publish_error('precondition',
                                f'distance {distance} not in (0, {self._distance_max}]',
                                run_id=run_id)
            return
        if not (0 < v_target <= self._v_target_max):
            self._publish_error('precondition',
                                f'v_target {v_target} not in (0, {self._v_target_max}]',
                                run_id=run_id)
            return

        with self._lock:
            if self.is_running:
                self._publish_error('precondition',
                                    f'run already active ({self._run.run_id if self._run else "?"})',
                                    run_id=run_id)
                return
            self._run = _RunState(run_id, distance, v_target)
            self._fsm_state = 'DRIVE_FORWARD_MPS'

        self.log_info('mps: starting run %s — D=%.2f, v_target=%.3f',
                      run_id, distance, v_target)

    # ── /scenario/abort handler ───────────────────────────────────────
    def _on_scenario_abort(self, topic: str, payload):
        with self._lock:
            run = self._run
        if run is None:
            return
        self._finish_run('aborted', None)

    # ── Odometry ──────────────────────────────────────────────────────
    def _on_odom(self, topic: str, payload):
        if not isinstance(payload, dict):
            return
        # samurai/{id}/odom — wheel/EKF combined; здесь нам важны s, v, θ, ω.
        # Маппинг: x_wheel/x → s в метрах (если в см — делим на 100).
        try:
            s = float(payload.get('s', payload.get('x', 0.0)))
            v = float(payload.get('vx', payload.get('linear_x', 0.0)))
            theta = float(payload.get('theta', 0.0))
            omega = float(payload.get('vz', payload.get('angular_z', 0.0)))
        except (TypeError, ValueError):
            return
        # If 's' looks like centimetres (>20 — odom_x обычно в см) — нормализуем.
        if abs(s) > 20.0:
            s = s / 100.0
        with self._lock:
            self._x_meas = np.array([s, v, theta, omega, self._x_meas[_EINT]])
            self._x_meas_ts = time.time()
            if self._run is not None:
                self._run.no_odom_ticks = 0

    # ── Tick (50 Hz во время RUNNING, no-op в IDLE) ───────────────────
    def _tick(self):
        with self._lock:
            run = self._run
            if run is None or self._fsm_state != 'DRIVE_FORWARD_MPS':
                return
            x = self._x_meas.copy()

        # Reference: ramp s_ref to D, hold v_target.
        s_ref = min(run.distance, run.t * run.v_target)
        x_ref = np.array([s_ref, run.v_target, 0.0, 0.0, 0.0])

        try:
            u = self._mpc.step(x, x_ref=x_ref)
        except Exception as exc:
            self.log_error('mps tick mpc.step failed: %s', exc)
            self._finish_run('error', f'mpc.step: {exc}')
            return

        if not (np.all(np.isfinite(u)) and np.all(np.isfinite(x))):
            self._finish_run('error', 'NaN/Inf in u or x')
            return

        # Hard omega cap in forward scenario — guard against accidental rotation.
        u[1] = max(-self._omega_max_fwd, min(self._omega_max_fwd, u[1]))

        # Send cmd_vel
        self.publish('cmd_vel', {
            'linear_x': float(u[0]),
            'angular_z': float(u[1]),
        }, qos=0)

        # Output for UI
        try:
            y = self._plant.output(x, u)
        except Exception:
            y = x.copy()

        s_remaining = max(0.0, run.distance - x[_S])
        point = {
            't': round(run.t, 6),
            'x': [float(v) for v in x],
            'u': [float(v) for v in u],
            'y': [float(v) for v in y],
            's_remaining': float(s_remaining),
        }
        run.telemetry.append(point)
        self.publish('mps/telemetry', {
            'run_id': run.run_id,
            'point': point,
            'schema_version': '1.0',
        }, qos=0)

        # Watchdog: нет одометрии 3 тика подряд → abort
        with self._lock:
            run.no_odom_ticks += 1
            stale = run.no_odom_ticks > _WATCHDOG_TICKS

        # Reached?
        if x[_S] >= run.distance - _REACH_EPS:
            self._finish_run('reached', None)
            return

        # Timeout?
        timeout_t = max(1.0, 3.0 * run.distance / max(run.v_target, 1e-6))
        if run.t > timeout_t:
            self._finish_run('timeout', None)
            return

        if stale and time.time() - self._x_meas_ts > 5.0 * self._tick_dt:
            self._finish_run('error', 'watchdog: no odom for >3 ticks')
            return

        run.t += self._tick_dt

    # ── Finalisation ─────────────────────────────────────────────────
    def _finish_run(self, status: str, error_message: Optional[str]):
        with self._lock:
            run = self._run
            self._run = None
            self._fsm_state = 'IDLE'

        # Failsafe: cmd_vel = 0 трижды (мотор-нода тоже сама стопит, но дублируем).
        for _ in range(3):
            self.publish('cmd_vel', {'linear_x': 0.0, 'angular_z': 0.0}, qos=0)

        if run is None:
            return

        # Compute simple metrics on Pi side too (subset — без peak_omega refs).
        n_pts = len(run.telemetry)
        if n_pts:
            s_arr = np.array([p['x'][_S] for p in run.telemetry])
            v_arr = np.array([p['x'][_V] for p in run.telemetry])
            omega_arr = np.array([p['x'][_OMEGA] for p in run.telemetry])
            metrics = {
                'overshoot': float(max(0.0, s_arr.max() - run.distance)),
                'settling_time': float(run.t),
                'control_energy': 0.0,  # compute_node пересчитает с настоящей R
                'ss_error': float(abs(s_arr[-1] - run.distance)),
                'peak_v': float(np.max(np.abs(v_arr))),
                'peak_omega': float(np.max(np.abs(omega_arr))),
            }
        else:
            metrics = None

        finished_payload = {
            'run_id': run.run_id,
            'started_at': run.started_at.isoformat(),
            'finished_at': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'request': {
                'distance': run.distance,
                'v_target': run.v_target,
                'source': 'robot',
            },
            'metrics': metrics,
            'error_message': error_message,
            'schema_version': '1.0',
        }
        self.publish('mps/scenario/finished', finished_payload, qos=1)
        self.log_info('mps: run %s finished status=%s (%d points)',
                      run.run_id, status, n_pts)
        if error_message:
            self._publish_error('other', error_message, run_id=run.run_id)

    # ── Helpers ──────────────────────────────────────────────────────
    def _publish_error(self, error_type: str, message: str,
                       run_id: Optional[str] = None) -> None:
        self.publish('mps/error', {
            'run_id': run_id,
            'error_type': error_type,
            'message': message,
            'schema_version': '1.0',
        }, qos=1)
        self.log_warn('mps error [%s]: %s', error_type, message)

    def _publish_status(self):
        with self._lock:
            run = self._run
        self.publish('status', {
            'state': self._fsm_state,
            'mps_active': run is not None,
            'mps_run_id': run.run_id if run else None,
        }, qos=0)


# ── Entry point ────────────────────────────────────────────────────────
def main():
    node = MpsNode()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
