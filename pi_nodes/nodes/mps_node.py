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

from pi_nodes.control.lateral_lqr import LateralLqrController
from pi_nodes.control.mpc_controller import MPCController
from pi_nodes.control.mps_reference import ReferenceTrajectory, build_reference
from pi_nodes.control.state_space_model import StateSpaceModel, zoh_discretize
from pi_nodes.mqtt_node import MqttNode

# Default tolerance: «достиг цели» если осталось ≤ этого (метры).
# Спека §4.1 / §7.1: дефолт 0.005 м (5 мм). Старое значение 0.02 м было
# выбрано в pre-pose-tracking эпохе для компенсации отсутствия decel —
# теперь референс с трапец-профилем сам сводит v к нулю, поэтому ε
# можно ужать до сопоставимого с одометрической дискретностью. Конфиг
# на конкретном железе может перекрыть через mps.scenario.reach_tolerance_m
# если dead-reckoning шумнее 5 мм (тогда задрать обратно к 0.02).
_REACH_EPS_DEFAULT = 0.005

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
    то, что нужно tick-loop'у feedforward-трекинга r(t) (см.
    docs/superpowers/specs/2026-05-17-mps-pose-tracking-design.md §3.2).
    """
    __slots__ = (
        'run_id', 'distance', 'v_target', 'target_heading',
        'started_at',
        'telemetry', 't',
        'no_odom_ticks', 's_start', 'theta_start',
        # Outer-loop LQR коррекции бокового сноса (см. lateral_lqr.py).
        'x_start_abs', 'y_start_abs', 'lateral_lqr',
        # Опорная траектория r(t) = [s_ref, v_ref, θ_ref, ω_ref, e_int_ref].
        'traj',
    )

    def __init__(self, run_id: str, distance: float, v_target: float,
                 target_heading: float,
                 traj: ReferenceTrajectory,
                 s_start: float = 0.0, theta_start: float = 0.0,
                 x_start_abs: float = 0.0, y_start_abs: float = 0.0,
                 lateral_lqr: Optional[LateralLqrController] = None):
        self.run_id = run_id
        self.distance = distance
        self.v_target = v_target
        # Относительный целевой курс φ (рад) — final heading после прибытия
        # в (D, 0). 0.0 = курс не меняется (ехать прямо вперёд).
        self.target_heading = target_heading
        self.started_at = datetime.now(timezone.utc)
        self.telemetry: list[dict] = []
        # Монотонное время прогона — координата на референсной траектории.
        self.t = 0.0
        self.no_odom_ticks = 0
        # Абсолютная позиция одометрии на момент старта сценария.
        # `s_ref` стартует с 0, поэтому позицию считаем относительно неё.
        self.s_start = s_start
        # Абсолютный курс одометрии на момент старта. Сценарий считает θ
        # относительно θ_start (как и s). Иначе MPC трактует x_ref[θ]=0 как
        # абсолютный 0 одометрии и доворачивает робота в одну и ту же
        # сторону вместо «ехать прямо куда смотрит».
        self.theta_start = theta_start
        # ── Outer LQR-петля (lateral): абсолютные (x, y) одометрии на
        # момент старта. e_y = перпендикулярное расстояние от ideal-line,
        # обновляется в _tick_run. lateral_lqr=None если outer-петля
        # выключена в config или v_target<v_min.
        self.x_start_abs = x_start_abs
        self.y_start_abs = y_start_abs
        self.lateral_lqr = lateral_lqr
        # Pre-built reference trajectory (drive + turn-after-arrival profile).
        self.traj = traj


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
        self._omega_max_turn = float(self._cfg('mps.scenario.omega_max_in_turn', 1.0))
        # Максимальный возраст последнего odom (с) для старта сценария.
        # Если odom не пришёл вовсе или старее этого порога — _on_scenario_run
        # возвращает precondition error без перехода в DRIVE_FORWARD_MPS
        # (см. Bug A в diagnostics 2026-05-15: race-condition theta_start
        # из-за пустого _x_meas).
        self._odom_max_age = float(self._cfg('mps.scenario.odom_max_age_s', 0.5))
        self._reach_eps = float(self._cfg('mps.scenario.reach_tolerance_m',
                                           _REACH_EPS_DEFAULT))
        # Тонкие допуски выхода в settling-окно по 4 координатам
        # (см. spec §4.1; зеркалит compute mps_runner._SETTLE_TOL/ε_*).
        self._eps_v = float(self._cfg('mps.scenario.reach.epsilon_v', 0.02))
        self._eps_theta = float(self._cfg('mps.scenario.reach.epsilon_theta', 0.05))
        self._eps_omega = float(self._cfg('mps.scenario.reach.epsilon_omega', 0.05))
        self._settle_timeout = float(self._cfg('mps.scenario.reach.settle_timeout_s', 1.5))
        # Параметры профиля r(t) — дефолты совпадают с compute mps_runner.
        # Payload `reference: {a_max, alpha_max}` в /scenario/run может
        # переопределять их per-run для синхронизации с sim'ом.
        self._ref_a_max = float(self._cfg('mps.scenario.reference.a_max', 0.20))
        self._ref_alpha_max = float(self._cfg('mps.scenario.reference.alpha_max', 1.0))
        # Старые ключи 2-фазного сценария (TURN→DRIVE) больше не читаются —
        # pose-tracking refactor (2026-05-17) убрал и саму фазу TURN, и
        # её таймауты/тол. Логируем явное предупреждение, чтобы старые
        # config.yaml не делали тихих сюрпризов.
        if self._cfg('mps.scenario.turn_tolerance_rad', None) is not None:
            self.log_warn('mps: config key turn_tolerance_rad is deprecated '
                          '(unused since pose-tracking refactor)')
        if self._cfg('mps.scenario.turn_timeout_s', None) is not None:
            self.log_warn('mps: config key turn_timeout_s is deprecated '
                          '(unused since pose-tracking refactor)')

        # ── Outer LQR-петля коррекции бокового сноса (см. lateral_lqr.py) ─
        # Параметры читаются один раз; контроллер строится per-scenario
        # в _on_scenario_run (зависит от v_target). При lateral_enabled=False
        # либо при v_target<lateral_v_min outer-петля молча выключается и
        # поведение возвращается к чистому inner-MPC.
        self._lateral_enabled = bool(self._cfg('mps.scenario.lateral.enabled', True))
        self._lateral_tau_inner = float(self._cfg('mps.scenario.lateral.tau_inner', 0.10))
        self._lateral_q = list(self._cfg('mps.scenario.lateral.Q_diag', [80.0, 30.0]))
        self._lateral_r = list(self._cfg('mps.scenario.lateral.R_diag', [1.0]))
        self._lateral_delta_max = float(self._cfg('mps.scenario.lateral.delta_theta_max', 0.20))
        self._lateral_v_min = float(self._cfg('mps.scenario.lateral.v_min', 0.02))

        # Anti-windup на интегральном члене e_int = ∫(−θ_err) dt. Модель
        # включает интегратор курсовой ошибки (A[4,2]=−1 в канонической MPS),
        # но «гонит θ к 0», что неправильно при non-zero θ_ref. mps_node
        # переписывает x_meas[EINT] руками в _tick_run (см. ниже).
        self._e_int_max = float(self._cfg('mps.scenario.e_int_max', 0.5))

        # x_meas от position_fusion (через odom MQTT). Атомарно read by tick.
        self._x_meas = np.zeros(5)
        self._x_meas_ts = 0.0
        # Абсолютная позиция одометрии (x, y) для outer-loop расчёта e_y.
        # Хранится отдельно от _x_meas (которое уже в относительных координатах
        # от старта сценария по `s`).
        self._x_abs = 0.0
        self._y_abs = 0.0

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

        # Outer LQR-петля строится ДО взятия lock'а: DARE может занять
        # миллисекунды на Pi с numpy fallback (без scipy), и нет смысла
        # держать tick-loop заблокированным на это время.
        lateral_lqr: Optional[LateralLqrController] = None
        if self._lateral_enabled and v_target >= self._lateral_v_min:
            try:
                lateral_lqr = LateralLqrController(
                    Ts=self._tick_dt,
                    v0=v_target,
                    tau_inner=self._lateral_tau_inner,
                    Q_diag=self._lateral_q,
                    R_diag=self._lateral_r,
                    delta_theta_max=self._lateral_delta_max,
                )
                self.log_info(
                    'mps: lateral LQR built — v0=%.3f, K=%s, stable=%s',
                    v_target, lateral_lqr.K.tolist(), lateral_lqr.is_stable(),
                )
            except Exception as exc:
                # Не критично — отвалимся в режим «без outer-петли», но залогим.
                self.log_warn('mps: lateral LQR build failed (%s) — outer loop disabled', exc)
                lateral_lqr = None

        # Снапшот позиции/курса одометрии и сборка опорной траектории
        # делаются под lock'ом ниже (snapshot должен быть атомарным с
        # переходом FSM в DRIVE_FORWARD_MPS, чтобы первый _tick видел
        # уже консистентное (run, _x_meas)).
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
            # Абсолютные (x, y) одометрии — нужны для проекции e_y на
            # ideal-line в outer LQR-петле.
            x_start_abs = float(self._x_abs)
            y_start_abs = float(self._y_abs)
            # Reset интегрального члена e_int. Без этого накопленный с
            # предыдущего прогона e_int даёт фантомную ошибку и MPC
            # «доворачивает» с первого тика.
            self._x_meas[_EINT] = 0.0

            # Build feedforward reference trajectory r(t). Параметры
            # reference.{a_max, alpha_max} можно переопределить per-run
            # через payload (для синхронизации Pi с compute sim'ом —
            # один и тот же профиль ⇒ один и тот же результат).
            ref_payload = payload.get('reference') or {}
            a_max = float(ref_payload.get('a_max', self._ref_a_max))
            alpha_max = float(ref_payload.get('alpha_max', self._ref_alpha_max))
            try:
                # ВАЖНО: theta_start=0.0 в build_reference — на Pi мы
                # отдаём MPC уже РЕЛЯТИВНЫЕ координаты (x[_THETA] = θ_meas
                # − run.theta_start, см. _tick_run). Поэтому референс
                # тоже должен быть в относительной системе: r(t)[2] = 0
                # на drive-сегменте, ramps до target_heading на turn-
                # сегменте. Иначе MPC видит θ_err = θ_start_abs (фантомные
                # 1+ рад на старте) и упирает angular_z в u_max.
                # Абсолютный θ_start робота нужен в _check_finish для
                # сборки theta_target = run.theta_start + traj.phi_signed.
                traj = build_reference(
                    distance=distance,
                    v_target=v_target,
                    target_heading=target_heading,
                    a_max=a_max,
                    alpha_max=alpha_max,
                    omega_max=self._omega_max_turn,
                    theta_start=0.0,
                )
            except ValueError as exc:
                self._publish_error('precondition',
                                    f'reference build failed: {exc}',
                                    run_id=run_id)
                return

            self._run = _RunState(
                run_id=run_id,
                distance=distance,
                v_target=v_target,
                target_heading=target_heading,
                traj=traj,
                s_start=s_start,
                theta_start=theta_start,
                x_start_abs=x_start_abs,
                y_start_abs=y_start_abs,
                lateral_lqr=lateral_lqr,
            )
            self._fsm_state = 'DRIVE_FORWARD_MPS'

        self.log_info('mps: starting run %s — D=%.2f, v_target=%.3f, '
                      'target_heading=%.3f, t_end=%.2f, lateral=%s',
                      run_id, distance, v_target, target_heading,
                      traj.t_end,
                      'on' if lateral_lqr is not None else 'off')

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
            # state[0] — body-frame distance ("сколько проехал вперёд").
            # Предпочитаем s_body = ∫v·dt (новый motor_node), fallback на
            # x/100 (старый motor_node без s_body — backward-compat).
            s_body = payload.get('s_body')
            x_world = float(payload.get('x', 0.0)) / 100.0
            y_world = float(payload.get('y', 0.0)) / 100.0
            if s_body is not None:
                s = float(s_body)
            else:
                s = x_world
            v = float(payload.get('vx', 0.0))
            theta = float(payload.get('theta', 0.0))
            omega = float(payload.get('vz', 0.0))
        except (TypeError, ValueError):
            return
        with self._lock:
            self._x_meas = np.array([s, v, theta, omega, self._x_meas[_EINT]])
            self._x_meas_ts = time.time()
            # Абсолютные (x, y) — нужны outer LQR-петле для проекции на
            # ideal-line. ИМЕННО world.x/world.y, не s_body: e_y живёт в
            # мировой системе координат, body-frame distance в неё не
            # проецируется.
            self._x_abs = x_world
            self._y_abs = y_world
            if self._run is not None:
                self._run.no_odom_ticks = 0

    # ── Tick (50 Hz во время RUNNING, no-op в IDLE) ───────────────────
    def _tick(self):
        with self._lock:
            run = self._run
            if run is None or self._fsm_state != 'DRIVE_FORWARD_MPS':
                return
        self._tick_run(run)

    # ── Один тик feedforward pose-tracking сценария ───────────────────
    def _tick_run(self, run: _RunState) -> None:
        """Один тик MPC-трекинга опорной траектории r(t).

        Поведение: MPC получает x_ref = traj.r(run.t) — пред-вычисленный
        кусок профиля accel→cruise→decel (drive) + accel→cruise→decel
        (turn-after-arrival), без явных фаз TURN/DRIVE. После plant.step
        x[4] (e_int) переписывается интегралом θ_err против ТЕКУЩЕГО
        θ_ref(t) с anti-windup — иначе при non-zero θ_ref модель
        `ė_int = −θ` гонит интегратор к θ=0 вместо θ_target (см. spec §3.3
        и compute_node/mps_runner.py).

        Если активна outer LQR-петля (`run.lateral_lqr is not None`),
        delta_theta = -K_lat·[e_y, θ_err] публикуется в телеметрию как
        diagnostic (на сам референс не добавляется — это снова прибор
        внешней косвенной коррекции; за курс отвечает r(t)).
        """
        with self._lock:
            x_meas = self._x_meas.copy()
            x_abs = self._x_abs
            y_abs = self._y_abs

        # Позиция и курс — относительно старта сценария (s_ref, θ_ref с 0).
        # θ НЕ оборачиваем в [−π, π] — MPC проектируется на линеаризованной
        # модели и ожидает «гладкое» x[θ] без дискретного скачка ±π. При
        # сценариях с target_heading=±π референс θ_ref(t) тоже ramps до
        # ±π без wrap, и контур остаётся в режиме малой ошибки. Обёртка
        # вызывала overshoot: как только робот пересекал ±π, x[θ] прыгал
        # в противоположный знак, MPC видел огромную ошибку и продолжал
        # вращение в ту же сторону (см. test_mps_node_pose_arrival).
        x = x_meas.copy()
        x[_S] = x_meas[_S] - run.s_start
        x[_THETA] = x_meas[_THETA] - run.theta_start

        r_ref = run.traj.r(run.t)

        try:
            u = self._mpc.step(x, x_ref=r_ref)
        except Exception as exc:
            self.log_error('mps mpc.step failed: %s', exc)
            self._finish_run('error', f'mpc.step: {exc}')
            return

        if not (np.all(np.isfinite(u)) and np.all(np.isfinite(x))):
            self._finish_run('error', 'NaN/Inf in u or x')
            return

        # Hard-cap по ω в drive-сегменте: v_ref ≠ 0 значит едем прямо,
        # защита от случайного крена. В turn-сегменте (v_ref = 0) кап
        # снимаем — профиль построен под omega_max_in_turn, MPC уже
        # учитывает u_max через clip, дополнительное огрубление мешает
        # отрабатывать «дотяжку» в settling-окне.
        if abs(r_ref[_V]) > 1e-3:
            u[1] = max(-self._omega_max_fwd, min(self._omega_max_fwd, u[1]))

        # Интегратор курсовой ошибки против ТЕКУЩЕГО θ_ref(t). Модель
        # включает интегратор курсовой ошибки (A[4,2]=−1), но это
        # «гонит θ к 0». При non-zero θ_ref правильный сигнал — интеграл
        # θ_err = θ − θ_ref, поэтому перепишем x[4] руками с anti-windup
        # (зеркалит compute mps_runner._accumulate_eint).
        theta_err = _normalize_angle(x[_THETA] - r_ref[_THETA])
        with self._lock:
            new_eint = self._x_meas[_EINT] + (-theta_err) * self._tick_dt
            if new_eint > self._e_int_max:
                new_eint = self._e_int_max
            elif new_eint < -self._e_int_max:
                new_eint = -self._e_int_max
            self._x_meas[_EINT] = new_eint

        # Локальные координаты для UI (см. spec §5.3). X — вдоль курса
        # робота на момент старта; Y — налево от X (правая система).
        dx = x_abs - run.x_start_abs
        dy = y_abs - run.y_start_abs
        cs = math.cos(run.theta_start)
        sn = math.sin(run.theta_start)
        x_local = dx * cs + dy * sn
        y_local = -dx * sn + dy * cs

        # Outer LQR-петля коррекции бокового сноса (см. lateral_lqr.py).
        # e_y публикуется в телеметрию даже если контроллер выключен —
        # diagnostic-значения нужны UI для построения графиков ошибки.
        e_y = y_local
        delta_theta = 0.0
        if run.lateral_lqr is not None:
            try:
                delta_theta = run.lateral_lqr.step(e_y, theta_err)
            except Exception as exc:
                self.log_warn('mps: lateral_lqr.step failed (%s) — skipping outer', exc)
                delta_theta = 0.0

        self._publish_cmd_and_telemetry(
            run, x, u,
            e_y=e_y, theta_err=theta_err, delta_theta=delta_theta,
            r=r_ref, x_local=x_local, y_local=y_local,
        )

        # Watchdog: нет одометрии 3 тика подряд → abort
        with self._lock:
            run.no_odom_ticks += 1
            stale = run.no_odom_ticks > _WATCHDOG_TICKS

        if self._check_finish(x, r_ref, run):
            return

        if stale and time.time() - self._x_meas_ts > 5.0 * self._tick_dt:
            self._finish_run('error', 'watchdog: no odom for >3 ticks')
            return

        run.t += self._tick_dt

    # ── Финиш-логика (4-координатный check вместо «доехал по s») ──────
    def _check_finish(self, x: np.ndarray, r_ref: np.ndarray,
                      run: _RunState) -> bool:
        """Решить, завершён ли прогон. Возвращает True ровно когда
        вызван _finish_run (caller должен сразу вернуться).

        Логика (см. spec §4):
          • t < t_end → разве что run timeout (контур упёрся в инстабильность
            и t убежал >> ожидаемого);
          • t ≥ t_end → 4 координаты (|s−D|<ε_s, |v|<ε_v, |θ_err|<ε_θ,
            |ω|<ε_ω) против финальной точки [D, 0, θ_start+φ, 0];
            если уложились — 'reached', иначе ждём settle_timeout
            и закрываем как 'timeout_settle' с деталью невыполнения.
        """
        if run.t < run.traj.t_end:
            # До конца профиля только run-timeout: контур взорвался и
            # t убежал ≫ ожидаемого (защита от бесконечного цикла).
            if run.t > max(1.0, 1.5 * run.traj.t_end + 2.0):
                self._finish_run('timeout', 'run timeout before t_end')
                return True
            return False

        # x[_THETA] и traj.phi_signed оба относительны run.theta_start
        # (build_reference вызывается с theta_start=0.0 — см.
        # _on_scenario_run). Целевое финальное относительное θ_target
        # = phi_signed (=_normalize_angle(target_heading)).
        theta_target_rel = run.traj.phi_signed
        theta_err = _normalize_angle(x[_THETA] - theta_target_rel)
        if (abs(x[_S] - run.distance) < self._reach_eps
                and abs(x[_V]) < self._eps_v
                and abs(theta_err) < self._eps_theta
                and abs(x[_OMEGA]) < self._eps_omega):
            self._finish_run('reached', None)
            return True

        if run.t > run.traj.t_end + self._settle_timeout:
            detail = (f'settle timeout: |s−D|={abs(x[_S]-run.distance):.4f}, '
                      f'|v|={abs(x[_V]):.4f}, |θ_err|={abs(theta_err):.4f}, '
                      f'|ω|={abs(x[_OMEGA]):.4f}')
            self._finish_run('timeout_settle', detail)
            return True
        return False

    # ── Публикация cmd_vel + телеметрии ────────────────────────────────
    def _publish_cmd_and_telemetry(
        self,
        run: _RunState,
        x: np.ndarray,
        u: np.ndarray,
        *,
        e_y: Optional[float] = None,
        theta_err: Optional[float] = None,
        delta_theta: Optional[float] = None,
        r: Optional[np.ndarray] = None,
        x_local: Optional[float] = None,
        y_local: Optional[float] = None,
    ) -> None:
        """Опубликовать cmd_vel и точку телеметрии. `run.t` — монотонное
        время прогона, поэтому `point['t']` строго растёт.

        Опциональные `e_y`/`theta_err`/`delta_theta` — diagnostic outer LQR-
        петли. `r/x_local/y_local` (schema 1.2) — опорная точка и локальные
        координаты для UI; см. compute_node/dashboard/schemas/mps.py.
        """
        self.publish('cmd_vel', {
            'linear_x': float(u[0]),
            'angular_z': float(u[1]),
        }, qos=0)

        try:
            y = self._plant.output(x, u)
        except Exception:
            y = x.copy()

        s_remaining = max(0.0, run.distance - x[_S])
        point: dict = {
            't': round(run.t, 6),
            'x': [float(v) for v in x],
            'u': [float(v) for v in u],
            'y': [float(v) for v in y],
            's_remaining': float(s_remaining),
        }
        if e_y is not None:
            point['e_y'] = float(e_y)
        if theta_err is not None:
            point['theta_err'] = float(theta_err)
        if delta_theta is not None:
            point['delta_theta'] = float(delta_theta)
        if r is not None:
            point['r'] = [float(v) for v in r]
        if x_local is not None:
            point['x_local'] = float(x_local)
        if y_local is not None:
            point['y_local'] = float(y_local)
        run.telemetry.append(point)
        self.publish('mps/telemetry', {
            'run_id': run.run_id,
            'point': point,
            'schema_version': '1.2',
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
            'schema_version': '1.1',
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
            'schema_version': '1.1',
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
