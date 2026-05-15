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

import math
import os
import sys
import threading
import time
from datetime import datetime, timezone
from typing import Any, Optional

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from pi_nodes.control.mpc_controller import MPCController
from pi_nodes.control.state_space_model import StateSpaceModel, zoh_discretize
from pi_nodes.mqtt_node import MqttNode

# Default tolerance: «достиг цели» если осталось ≤ этого (метры).
# Перетирается через mps.scenario.reach_tolerance_m в config.yaml. Старое
# дефолтное значение 0.05 м засчитывало 84% дистанции на D=0.30 как «reached»
# и было ровно D/2 на D=0.10 — слишком грубо. Новый дефолт 0.02 м (2 см) —
# сопоставимо с разрешением dead-reckoning одометрии, не зашумит status.
_REACH_EPS_DEFAULT = 0.02

# Состояние x = [s, v, θ, ω, e_int]
_S, _V, _THETA, _OMEGA, _EINT = 0, 1, 2, 3, 4

# Watchdog: сколько подряд тиков без свежей одометрии до abort.
_WATCHDOG_TICKS = 3


def _normalize_angle(a: float) -> float:
    """Угол → [-π, π]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class _RunState:
    """Локальное состояние active run (только внутри mps_node).

    Не путать с DashboardState на ноуте — это совсем разное. Здесь только
    то, что нужно tick-loop'у.
    """
    __slots__ = (
        'run_id', 'distance', 'v_target', 'started_at',
        'telemetry', 't',
        'no_odom_ticks', 's_start', 'theta_start',
        'target_heading', 'phase', 'drive_t',
    )

    def __init__(self, run_id: str, distance: float, v_target: float,
                 s_start: float = 0.0, theta_start: float = 0.0,
                 target_heading: float = 0.0):
        self.run_id = run_id
        self.distance = distance
        self.v_target = v_target
        self.started_at = datetime.now(timezone.utc)
        self.telemetry: list[dict] = []
        self.t = 0.0
        self.no_odom_ticks = 0
        # Абсолютная позиция одометрии на момент старта сценария.
        # `s_ref` стартует с 0, поэтому позицию считаем относительно неё.
        self.s_start = s_start
        # Абсолютный курс одометрии на момент старта. Сценарий «вперёд D
        # метров» — это вперёд ОТНОСИТЕЛЬНО старта, поэтому θ считаем
        # относительно θ_start (как и s). Иначе MPC трактует x_ref[θ]=0 как
        # абсолютный 0 одометрии и доворачивает робота в одну и ту же
        # сторону вместо «ехать прямо куда смотрит».
        self.theta_start = theta_start
        # Относительный целевой курс φ (рад) — куда развернуться перед
        # движением. 0.0 = ехать прямо вперёд (сегодняшнее поведение).
        self.target_heading = target_heading
        # Фаза двухфазного сценария: 'turn' (разворот к φ на месте) →
        # 'drive' (движение N метров с удержанием курса φ).
        self.phase = 'turn'
        # Часы фазы DRIVE — начинаются с 0 при переходе TURN→DRIVE.
        # Используются для ramp s_ref и drive-timeout (тайминг движения
        # считается от начала езды, а не от старта сценария). `t` при этом
        # остаётся монотонным суммарным временем (turn + drive).
        self.drive_t = 0.0


class MpsNode(MqttNode):
    """Orchestrator для MPS-сценариев — единственный источник cmd_vel
    в state DRIVE_FORWARD_MPS."""

    def __init__(self, **kwargs):
        super().__init__('mps_node', **kwargs)

        # Защищает state от race между MQTT-callback и timer-tick.
        self._lock = threading.RLock()

        # Период дискретизации МПС — нужен до сборки контроллера.
        self._mps_ts = float(self._cfg('mps.plant.Ts', 0.02))

        # Source-of-truth модели — каноническая МПС-форма из секции `mps:`
        # config.yaml. НЕЛЬЗЯ строить безаргументными StateSpaceModel()/
        # MPCController(): они читают LEGACY-namespace control.*
        # ([px,py,θ,v,ω] + готовый gain control.matrices.K_mpc). Тогда MPC
        # в DRIVE_FORWARD_MPS получает каноническое x=[s,v,θ,ω,e_int],
        # трактует x[1]=v как поперечную координату py, видит py_ref=v_target
        # и упирает cmd_vel.angular_z → робот едет по кругу вместо «вперёд»
        # (симулятор всегда строит из канонических матриц и едет прямо).
        # Live-правки матриц по-прежнему через _on_matrices_set.
        try:
            self._plant, self._mpc = self._build_from_config()
        except Exception as exc:
            self.log_error('MPS: failed to bootstrap model/controller: %s', exc)
            raise

        self._tick_dt = float(self._cfg('mps.tick_dt', 0.02))
        self._distance_max = float(self._cfg('mps.scenario.distance_max', 5.0))
        self._v_target_max = float(self._cfg('mps.scenario.v_target_max', 0.30))
        self._omega_max_fwd = float(self._cfg('mps.scenario.omega_max_in_forward', 0.5))
        self._turn_tol = float(self._cfg('mps.scenario.turn_tolerance_rad', 0.05))
        self._turn_timeout = float(self._cfg('mps.scenario.turn_timeout_s', 10.0))
        self._omega_max_turn = float(self._cfg('mps.scenario.omega_max_in_turn', 1.0))
        # Максимальный возраст последнего odom (с) для старта сценария.
        # Если odom не пришёл вовсе или старее этого порога — _on_scenario_run
        # возвращает precondition error без перехода в DRIVE_FORWARD_MPS
        # (см. Bug A в diagnostics 2026-05-15: race-condition theta_start
        # из-за пустого _x_meas).
        self._odom_max_age = float(self._cfg('mps.scenario.odom_max_age_s', 0.5))
        self._reach_eps = float(self._cfg('mps.scenario.reach_tolerance_m',
                                           _REACH_EPS_DEFAULT))

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

    # ── Bootstrap from config `mps:` block ─────────────────────────────
    def _build_from_config(self) -> tuple[StateSpaceModel, MPCController]:
        """Построить plant + MPC из канонической секции `mps:` config.yaml.

        `mps.matrices.A/B` — НЕПРЕРЫВНЫЕ (контракт docs/mps/api.md);
        ZOH-дискретизируем при `mps.plant.Ts`. Зеркалит compute-side
        `mps_runner._build_controller` и Pi-side `_on_matrices_set` — все
        три обязаны строить ОДИН контроллер, иначе sim и робот разъезжаются.
        """
        A = self._cfg('mps.matrices.A', None)
        B = self._cfg('mps.matrices.B', None)
        C = self._cfg('mps.matrices.C', None)
        D = self._cfg('mps.matrices.D', None)
        Q_diag = self._cfg('mps.weights.Q_diag', None)
        R_diag = self._cfg('mps.weights.R_diag', None)
        N = self._cfg('mps.horizon_N', None)
        u_min = self._cfg('mps.limits.u_min', None)
        u_max = self._cfg('mps.limits.u_max', None)
        if any(v is None for v in (A, B, Q_diag, R_diag, N, u_min, u_max)):
            raise RuntimeError(
                'config.yaml: секция mps: неполна — нужны matrices.A/B, '
                'weights.Q_diag/R_diag, horizon_N, limits.u_min/u_max'
            )

        A = np.asarray(A, dtype=float)
        B = np.asarray(B, dtype=float)
        C = np.asarray(C, dtype=float) if C is not None else None
        D = np.asarray(D, dtype=float) if D is not None else None
        Ad, Bd = zoh_discretize(A, B, self._mps_ts)
        plant = StateSpaceModel(Ad=Ad, Bd=Bd, Cd=C, Dd=D, Ts=self._mps_ts)
        mpc = MPCController(
            Ad=Ad, Bd=Bd,
            Q=np.diag(np.asarray(Q_diag, dtype=float)),
            R=np.diag(np.asarray(R_diag, dtype=float)),
            N=int(N),
            u_min=np.asarray(u_min, dtype=float),
            u_max=np.asarray(u_max, dtype=float),
            solver='clip',
        )
        return plant, mpc

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

        # A/B приходят НЕПРЕРЫВНЫМИ (контракт docs/mps/api.md) —
        # ZOH-дискретизируем перед передачей в дискретные plant/MPC.
        try:
            Ad, Bd = zoh_discretize(A, B, self._mps_ts)
        except Exception as exc:
            self._publish_error('precondition', f'matrices/set: ZOH failed: {exc}')
            return

        with self._lock:
            try:
                self._plant.reload(Ad=Ad, Bd=Bd, Cd=C, Dd=D)
                self._mpc.rebuild(
                    Ad=Ad, Bd=Bd, Q_diag=Q, R_diag=R, N=N,
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
        self.log_info('mps: matrices applied (N=%d, Ts=%.3f, λ_open(Ad)=%s)',
                      N, self._mps_ts,
                      [f'{abs(z):.3f}' for z in np.linalg.eigvals(Ad)])

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
            target_heading = float(request.get('target_heading', 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            self._publish_error('precondition', f'scenario/run: bad request: {exc}',
                                run_id=run_id)
            return

        # Pre-validate against safety caps
        if not (-math.pi - 1e-6 <= target_heading <= math.pi + 1e-6):
            self._publish_error('precondition',
                                f'target_heading {target_heading} not in [-pi, pi]',
                                run_id=run_id)
            return
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

        # Reject если одометрия не пришла или устарела — без свежего snapshot'а
        # (s_start, theta_start) сценарий ловит race-condition: _x_meas остаётся
        # zeros() из __init__, theta_start=0, а к первому тику odom приходит
        # с реальным курсом → relative-θ становится огромным → робот застревает
        # в TURN-фазе (видели в diagnostics 2026-05-15: run #2 timeout 10 c
        # с theta_meas = -3.05 при target_heading=0). Reject лучше тихого failure.
        odom_age = time.time() - self._x_meas_ts
        if self._x_meas_ts == 0.0:
            self._publish_error(
                'precondition',
                'odom not received yet — start the robot stack and retry',
                run_id=run_id,
            )
            return
        if odom_age > self._odom_max_age:
            self._publish_error(
                'precondition',
                f'odom stale ({odom_age:.2f}s > {self._odom_max_age:.2f}s) — '
                f'robot may be disconnected',
                run_id=run_id,
            )
            return

        with self._lock:
            if self.is_running:
                self._publish_error('precondition',
                                    f'run already active ({self._run.run_id if self._run else "?"})',
                                    run_id=run_id)
                return
            # Снапшот позиции и курса одометрии — сценарий считает s и θ
            # относительно точки старта (s_ref и θ_ref начинаются с 0).
            s_start = float(self._x_meas[_S])
            theta_start = float(self._x_meas[_THETA])
            self._run = _RunState(run_id, distance, v_target,
                                  s_start, theta_start, target_heading)
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
        # samurai/{id}/odom — motor_node публикует:
        #   • s_body — м, signed body-frame distance (предпочитаем; см. ниже);
        #   • x      — см, world-frame координата (legacy fallback);
        #   • vx     — м/с (продольная скорость, body-frame);
        #   • theta  — рад (абсолютный курс IMU);
        #   • vz     — рад/с (угловая скорость).
        #
        # Сценарий «вперёд D метров» хочет body-frame дистанцию, а не world.x.
        # world.x = ∫v·cos(θ_abs)·dt — переворачивается в минус, если IMU
        # абсолютный курс ≈ ±π (видели в diagnostics 2026-05-15: u_v>0,
        # v_actual>0, но x шёл в минус → mps думал что робот едет назад).
        # s_body = ∫v·dt — независим от θ, всегда «сколько проехал вперёд».
        #
        # Fallback на x/100 нужен на случай старого motor_node без s_body —
        # сохраняем backward-compat. После раскатки нового motor_node на робота
        # ветка fallback станет мёртвой.
        try:
            s_body = payload.get('s_body')
            if s_body is not None:
                s = float(s_body)
            else:
                s = float(payload.get('x', 0.0)) / 100.0
            v = float(payload.get('vx', 0.0))
            theta = float(payload.get('theta', 0.0))
            omega = float(payload.get('vz', 0.0))
        except (TypeError, ValueError):
            return
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
        # Позиция и курс — относительно старта сценария (s_ref, θ_ref с 0).
        x[_S] = x[_S] - run.s_start
        x[_THETA] = _normalize_angle(x[_THETA] - run.theta_start)

        # Двухфазный сценарий: TURN (разворот к φ) → DRIVE (едем N метров).
        if run.phase == 'turn':
            if not self._tick_turn(run, x):
                return            # ещё крутимся, либо прогон завершён
            # фаза TURN завершилась этим тиком → продолжаем в DRIVE
        self._tick_drive(run, x)

    # ── Фаза TURN: разворот на месте к target_heading ─────────────────
    def _tick_turn(self, run: _RunState, x: np.ndarray) -> bool:
        """Один тик фазы разворота. `x` — уже относительный (s, θ).

        Возвращает True ровно когда поворот только что завершён — тогда
        вызывающий (`_tick`) продолжает в DRIVE тем же тиком. False —
        если ещё крутимся или прогон уже завершён (timeout / ошибка).
        """
        phi = run.target_heading

        # Курс совпал с целью → переход в DRIVE. Без публикации —
        # cmd_vel/телеметрию за этот тик опубликует _tick_drive.
        if abs(_normalize_angle(x[_THETA] - phi)) < self._turn_tol:
            run.phase = 'drive'
            run.drive_t = 0.0
            return True

        x_ref = np.array([0.0, 0.0, phi, 0.0, 0.0])
        try:
            u = self._mpc.step(x, x_ref=x_ref)
        except Exception as exc:
            self.log_error('mps turn mpc.step failed: %s', exc)
            self._finish_run('error', f'mpc.step: {exc}')
            return False

        if not (np.all(np.isfinite(u)) and np.all(np.isfinite(x))):
            self._finish_run('error', 'NaN/Inf in u or x')
            return False

        # Чистое вращение: ход — в ноль; ω — свой (более высокий) кап.
        u[0] = 0.0
        u[1] = max(-self._omega_max_turn, min(self._omega_max_turn, u[1]))

        self._publish_cmd_and_telemetry(run, x, u)

        with self._lock:
            run.no_odom_ticks += 1
            stale = run.no_odom_ticks > _WATCHDOG_TICKS

        # Turn timeout — по run.t. Валидно: TURN всегда первая фаза,
        # run.t стартует с 0, поэтому run.t == времени разворота.
        if run.t > self._turn_timeout:
            self._finish_run('timeout', None)
            return False

        if stale and time.time() - self._x_meas_ts > 5.0 * self._tick_dt:
            self._finish_run('error', 'watchdog: no odom for >3 ticks')
            return False

        run.t += self._tick_dt
        return False

    # ── Фаза DRIVE: едем N метров, удерживая курс target_heading ───────
    def _tick_drive(self, run: _RunState, x: np.ndarray) -> None:
        """Один тик фазы движения. `x` — уже относительный (s, θ).

        Логика «вперёд D», но θ_ref = target_heading (удержание выбранного
        курса, НЕ доворот к 0) и тайминг ramp/timeout по run.drive_t.
        """
        phi = run.target_heading

        # Reference: ramp s_ref to D, hold v_target, hold heading φ.
        s_ref = min(run.distance, run.drive_t * run.v_target)
        x_ref = np.array([s_ref, run.v_target, phi, 0.0, 0.0])

        try:
            u = self._mpc.step(x, x_ref=x_ref)
        except Exception as exc:
            self.log_error('mps drive mpc.step failed: %s', exc)
            self._finish_run('error', f'mpc.step: {exc}')
            return

        if not (np.all(np.isfinite(u)) and np.all(np.isfinite(x))):
            self._finish_run('error', 'NaN/Inf in u or x')
            return

        # Hard omega cap in forward scenario — guard against accidental rotation.
        u[1] = max(-self._omega_max_fwd, min(self._omega_max_fwd, u[1]))

        self._publish_cmd_and_telemetry(run, x, u)

        # Watchdog: нет одометрии 3 тика подряд → abort
        with self._lock:
            run.no_odom_ticks += 1
            stale = run.no_odom_ticks > _WATCHDOG_TICKS

        # Reached?
        if x[_S] >= run.distance - self._reach_eps:
            self._finish_run('reached', None)
            return

        # Timeout?
        timeout_t = max(1.0, 3.0 * run.distance / max(run.v_target, 1e-6))
        if run.drive_t > timeout_t:
            self._finish_run('timeout', None)
            return

        if stale and time.time() - self._x_meas_ts > 5.0 * self._tick_dt:
            self._finish_run('error', 'watchdog: no odom for >3 ticks')
            return

        run.t += self._tick_dt
        run.drive_t += self._tick_dt

    # ── Публикация cmd_vel + телеметрии (общее для обеих фаз) ──────────
    def _publish_cmd_and_telemetry(self, run: _RunState, x: np.ndarray,
                                   u: np.ndarray) -> None:
        """Опубликовать cmd_vel и точку телеметрии. Вызывается из обеих
        фаз сценария (_tick_turn, _tick_drive). `run.t` — монотонное
        суммарное время прогона, поэтому `point['t']` строго растёт."""
        self.publish('cmd_vel', {
            'linear_x': float(u[0]),
            'angular_z': float(u[1]),
        }, qos=0)

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
