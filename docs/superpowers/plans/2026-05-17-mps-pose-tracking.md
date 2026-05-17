# MPS Pose-Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Заменить 2-фазный MPS-сценарий (`turn→drive` с `reach_tolerance=0.02м`) на pose-tracking с feedforward-референсом так, чтобы робот всегда оказывался в `(D, 0)` локального фрейма и затем разворачивался на `φ`.

**Architecture:** Один контроллер (существующий `MPCController`) + новый модуль `mps_reference.py` строит траекторию `r(t) = [s, v, θ, ω, e_int]` как трапец по `v` (drive) и трапец по `ω` (turn). Pi и compute делят модуль; sim и робот гоняют по байт-идентичному `r(t)`. Финиш — по 4 координатам `s, v, θ, ω` одновременно.

**Tech Stack:** Python (pi_nodes, compute_node, pytest), TypeScript/React (frontend, vitest), MQTT (paho), YAML (config), Pydantic v2.

**Spec:** [docs/superpowers/specs/2026-05-17-mps-pose-tracking-design.md](../specs/2026-05-17-mps-pose-tracking-design.md)

---

## File Structure

**Создаются:**
- `pi_nodes/control/mps_reference.py` — `ReferenceTrajectory` + `build_reference(...)`. Pure numpy, без MQTT/ROS импортов. Один импорт-источник для Pi и compute.
- `tests/test_mps_reference.py` — юнит-тесты профилей (trapezoid/triangle, drive/turn, edge cases, continuity).
- `docs/mps/scenario_pose.md` — пользовательская дока сценария.

**Модифицируются:**
- `compute_node/dashboard/schemas/mps.py` — поля `r`, `x_local`, `y_local` в `MpsTelemetryPoint`; статус `'timeout_settle'`; docstring `target_heading`; bump `MPS_SCHEMA_VERSION` → `'1.2'`.
- `compute_node/mps_runner.py` — `run_scenario_idealized` строит `traj = build_reference(...)`, шагает по `r(t)`, проверяет финиш per §4.1 спеки.
- `compute_node/dashboard/routers/mps.py` — MQTT-payload `mps/scenario/run` обогащается полем `reference: {a_max, alpha_max}`.
- `pi_nodes/nodes/mps_node.py` — выпиливается `_tick_turn/_tick_drive`, `_RunState.phase`, `target_heading→turn before drive`. Добавляется `_tick_run`, `_check_finish`, поля `x_local/y_local` в публикуемой телеметрии. Deprecation warnings для старых config-ключей.
- `tests/test_mps_node.py` — обновить под новый tick, добавить `test_scenario_pose_arrival`, `test_scenario_settle_timeout`.
- `tests/test_mps_runner.py` — добавить `test_runner_pose_arrival`.
- `tests/test_mps_router.py` — добавить `test_payload_includes_reference`.
- `config.yaml` — `reach_tolerance_m: 0.005`, `mps.scenario.reach.*`, `mps.scenario.reference.*`. Deprecation: `turn_tolerance_rad`, `turn_timeout_s`.
- `compute_node/frontend/src/types/mps.ts` — поля `r?`, `x_local?`, `y_local?` в `MpsTelemetryPoint`; `'timeout_settle'` в `ScenarioStatus`; `MPS_SCHEMA_VERSION = '1.2'`.
- `compute_node/frontend/src/components/mps/TrajectoryView.tsx` — путь из `x_local/y_local` (fallback на старый); пунктирный план `(0,0)→(D,0)` + стрелка φ; легенда из 4 элементов; подпись.
- `compute_node/frontend/src/components/mps/TrajectoryView.test.tsx` — новые тесты.
- `compute_node/frontend/src/components/mps/ScenarioControls.tsx` — лейбл `target_heading` → «Финальный курс φ (рад)».

**Удаляются:**
- `docs/mps/scenario_forward.md` (заменён `scenario_pose.md`).

---

## Task 1: Модуль `mps_reference.py` — структура и build

**Files:**
- Create: `pi_nodes/control/mps_reference.py`
- Test: `tests/test_mps_reference.py`

- [ ] **Step 1.1: Создать пустой test-файл с failing-импортом**

`tests/test_mps_reference.py`:
```python
"""Unit-tests для pi_nodes/control/mps_reference.py — генератор r(t)."""
import math
import numpy as np
import pytest

from pi_nodes.control.mps_reference import (
    ReferenceTrajectory,
    build_reference,
)


def test_build_reference_returns_trajectory():
    traj = build_reference(distance=0.30, v_target=0.15, target_heading=math.pi,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    assert isinstance(traj, ReferenceTrajectory)
    assert traj.t_drive > 0
    assert traj.t_end > traj.t_drive
```

- [ ] **Step 1.2: Запустить — упасть на импорте**

Run: `pytest tests/test_mps_reference.py::test_build_reference_returns_trajectory -v`
Expected: `ModuleNotFoundError: No module named 'pi_nodes.control.mps_reference'`

- [ ] **Step 1.3: Создать `pi_nodes/control/mps_reference.py` со скелетом**

```python
"""MPS reference trajectory generator.

Чистый numpy-модуль; импортируется Pi-нодой (mps_node.py) и compute-side
симулятором (mps_runner.py). Никаких импортов MQTT/ROS/dashboard — иначе
расходящиеся deps на Pi vs ноут.

Источник правды формул: docs/superpowers/specs/2026-05-17-mps-pose-tracking-design.md §3.2.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


_S, _V, _THETA, _OMEGA, _EINT = 0, 1, 2, 3, 4


def _normalize_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class _Segment:
    """Один кусок профиля (accel, cruise или decel) — для drive ИЛИ turn."""
    t0: float                # начало куска (s или rad-интегр.)
    t1: float                # конец
    s0: float                # значение интеграла в t0 (метров или радиан)
    v0: float                # скорость в t0 (m/s или rad/s)
    a: float                 # ускорение на куске (m/s² или rad/s²)


@dataclass(frozen=True)
class ReferenceTrajectory:
    """Опорная траектория r(t) для pose-tracking сценария.

    После постройки иммутабельна. Метод `r(t)` дёшев (бинпоиск +
    арифметика), безопасен в hot-loop'е tick'а MPC.
    """
    distance: float
    v_target: float
    target_heading: float        # final heading отн. θ_start
    theta_start: float           # абсолютный курс на старте сценария
    a_max: float
    alpha_max: float
    omega_max: float

    drive_segments: tuple[_Segment, ...] = field(repr=False)
    turn_segments: tuple[_Segment, ...] = field(repr=False)
    t_drive: float = 0.0
    t_end: float = 0.0
    phi_signed: float = 0.0       # _normalize_angle(target_heading)

    def r(self, t: float) -> np.ndarray:
        """Вернуть [s, v, θ, ω, e_int] на момент `t`. Для t > t_end —
        финальная точка [D, 0, θ_start+φ, 0, 0]."""
        raise NotImplementedError  # заполним в Step 1.5


def build_reference(
    *,
    distance: float,
    v_target: float,
    target_heading: float,
    a_max: float,
    alpha_max: float,
    omega_max: float,
    theta_start: float = 0.0,
) -> ReferenceTrajectory:
    """Строит ReferenceTrajectory из параметров сценария.

    Все параметры обязательны (kwargs-only). Бросает ValueError на
    некорректные входы (отрицательные D/v/a, |φ|>π, etc.) — валидация
    дублирует Pydantic на стороне compute, но нужна для прямых
    вызовов из Pi-кода и тестов.
    """
    raise NotImplementedError  # заполним в Step 1.5
```

- [ ] **Step 1.4: Запустить тест — должен упасть на NotImplementedError**

Run: `pytest tests/test_mps_reference.py::test_build_reference_returns_trajectory -v`
Expected: `NotImplementedError`

- [ ] **Step 1.5: Реализовать `build_reference` (drive + turn профили)**

Заменить `raise NotImplementedError` в `build_reference` на:
```python
    # Валидация
    if not (distance >= 0):
        raise ValueError(f'distance must be ≥ 0, got {distance}')
    if not (v_target > 0):
        raise ValueError(f'v_target must be > 0, got {v_target}')
    if not (a_max > 0 and alpha_max > 0 and omega_max > 0):
        raise ValueError('a_max, alpha_max, omega_max must be > 0')
    if abs(target_heading) > math.pi + 1e-9:
        raise ValueError(f'|target_heading| must be ≤ π, got {target_heading}')

    # Drive-сегменты
    drive_segs: list[_Segment] = []
    s_acc_full = 0.5 * v_target ** 2 / a_max
    if distance >= 2 * s_acc_full:
        # Trapezoidal
        t_acc = v_target / a_max
        t_cruise = (distance - 2 * s_acc_full) / v_target
        t_drive = 2 * t_acc + t_cruise
        drive_segs.append(_Segment(t0=0.0, t1=t_acc, s0=0.0, v0=0.0, a=a_max))
        drive_segs.append(_Segment(t0=t_acc, t1=t_acc + t_cruise,
                                   s0=s_acc_full, v0=v_target, a=0.0))
        drive_segs.append(_Segment(t0=t_acc + t_cruise, t1=t_drive,
                                   s0=distance - s_acc_full, v0=v_target,
                                   a=-a_max))
    elif distance > 0:
        # Triangular
        v_peak = math.sqrt(distance * a_max)
        t_acc = v_peak / a_max
        t_drive = 2 * t_acc
        drive_segs.append(_Segment(t0=0.0, t1=t_acc, s0=0.0, v0=0.0, a=a_max))
        drive_segs.append(_Segment(t0=t_acc, t1=t_drive,
                                   s0=distance / 2, v0=v_peak, a=-a_max))
    else:
        # distance == 0 — пустой drive
        t_drive = 0.0

    # Turn-сегменты
    phi = _normalize_angle(target_heading)
    sign = 1.0 if phi >= 0 else -1.0
    abs_phi = abs(phi)
    turn_segs: list[_Segment] = []
    theta_acc_full = 0.5 * omega_max ** 2 / alpha_max
    if abs_phi >= 2 * theta_acc_full:
        # Trapezoidal по ω
        t_acc_w = omega_max / alpha_max
        t_cruise_w = (abs_phi - 2 * theta_acc_full) / omega_max
        t_turn = 2 * t_acc_w + t_cruise_w
        turn_segs.append(_Segment(t0=0.0, t1=t_acc_w, s0=0.0, v0=0.0,
                                  a=sign * alpha_max))
        turn_segs.append(_Segment(t0=t_acc_w, t1=t_acc_w + t_cruise_w,
                                  s0=sign * theta_acc_full,
                                  v0=sign * omega_max, a=0.0))
        turn_segs.append(_Segment(t0=t_acc_w + t_cruise_w, t1=t_turn,
                                  s0=sign * (abs_phi - theta_acc_full),
                                  v0=sign * omega_max, a=-sign * alpha_max))
    elif abs_phi > 1e-9:
        # Triangular по ω
        w_peak = math.sqrt(abs_phi * alpha_max)
        t_acc_w = w_peak / alpha_max
        t_turn = 2 * t_acc_w
        turn_segs.append(_Segment(t0=0.0, t1=t_acc_w, s0=0.0, v0=0.0,
                                  a=sign * alpha_max))
        turn_segs.append(_Segment(t0=t_acc_w, t1=t_turn,
                                  s0=sign * abs_phi / 2, v0=sign * w_peak,
                                  a=-sign * alpha_max))
    else:
        t_turn = 0.0

    return ReferenceTrajectory(
        distance=distance,
        v_target=v_target,
        target_heading=target_heading,
        theta_start=theta_start,
        a_max=a_max,
        alpha_max=alpha_max,
        omega_max=omega_max,
        drive_segments=tuple(drive_segs),
        turn_segments=tuple(turn_segs),
        t_drive=t_drive,
        t_end=t_drive + t_turn,
        phi_signed=phi,
    )
```

- [ ] **Step 1.6: Запустить тест — должен пройти**

Run: `pytest tests/test_mps_reference.py::test_build_reference_returns_trajectory -v`
Expected: PASS. `traj.t_drive` ≈ 2.75, `traj.t_end` ≈ 9.53.

- [ ] **Step 1.7: Commit**

```bash
git add pi_nodes/control/mps_reference.py tests/test_mps_reference.py
git commit -m "feat(mps): скелет mps_reference.py + build_reference (drive/turn профили)"
```

---

## Task 2: Метод `ReferenceTrajectory.r(t)` (вычисление)

**Files:**
- Modify: `pi_nodes/control/mps_reference.py`
- Test: `tests/test_mps_reference.py`

- [ ] **Step 2.1: Тесты на корректность drive-trapezoid**

Добавить в `tests/test_mps_reference.py`:
```python
def _make_drive_trap():
    return build_reference(distance=0.30, v_target=0.15, target_heading=0.0,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)


def test_drive_trapezoid_endpoints():
    traj = _make_drive_trap()
    r0 = traj.r(0.0)
    r_end = traj.r(traj.t_drive)
    # старт: s=0, v=0, θ=θ_start, ω=0
    np.testing.assert_allclose(r0, [0.0, 0.0, 0.0, 0.0, 0.0], atol=1e-9)
    # конец drive: s=D, v=0, θ=θ_start, ω=0
    np.testing.assert_allclose(r_end, [0.30, 0.0, 0.0, 0.0, 0.0], atol=1e-9)


def test_drive_trapezoid_cruise_peak():
    traj = _make_drive_trap()
    # середина cruise — v должен быть = v_target
    r_mid = traj.r(traj.t_drive / 2)
    assert abs(r_mid[1] - 0.15) < 1e-9


def test_drive_trapezoid_monotonic_s():
    traj = _make_drive_trap()
    ts = np.linspace(0, traj.t_drive, 200)
    s_vals = [traj.r(t)[0] for t in ts]
    diffs = np.diff(s_vals)
    assert np.all(diffs >= -1e-12), 'drive s_ref must be non-decreasing'
```

- [ ] **Step 2.2: Запустить — упадут все три на NotImplementedError**

Run: `pytest tests/test_mps_reference.py -v -k drive_trapezoid`
Expected: 3 FAILED with NotImplementedError.

- [ ] **Step 2.3: Реализовать `r(t)` через интерполяцию сегментов**

Заменить тело `r(self, t)` в `ReferenceTrajectory`:
```python
    def r(self, t: float) -> np.ndarray:
        out = np.zeros(5)
        # Drive
        if t < self.t_drive:
            seg = self._find_segment(self.drive_segments, t)
            dt = t - seg.t0
            v = seg.v0 + seg.a * dt
            s = seg.s0 + seg.v0 * dt + 0.5 * seg.a * dt * dt
            out[_S] = s
            out[_V] = v
            out[_THETA] = self.theta_start
            out[_OMEGA] = 0.0
            return out
        # Turn (t_drive ≤ t < t_end)
        if t < self.t_end and self.turn_segments:
            seg = self._find_segment(self.turn_segments, t - self.t_drive)
            dt = (t - self.t_drive) - seg.t0
            w = seg.v0 + seg.a * dt
            dtheta = seg.s0 + seg.v0 * dt + 0.5 * seg.a * dt * dt
            out[_S] = self.distance
            out[_V] = 0.0
            out[_THETA] = self.theta_start + dtheta
            out[_OMEGA] = w
            return out
        # Settling: финальная точка
        out[_S] = self.distance
        out[_V] = 0.0
        out[_THETA] = self.theta_start + self.phi_signed
        out[_OMEGA] = 0.0
        return out

    @staticmethod
    def _find_segment(segments: tuple[_Segment, ...], t: float) -> _Segment:
        """Линейный поиск (макс 3 сегмента в каждом профиле — bisect overkill)."""
        for seg in segments:
            if t < seg.t1:
                return seg
        return segments[-1]
```

- [ ] **Step 2.4: Запустить — все 3 теста должны пройти**

Run: `pytest tests/test_mps_reference.py -v -k drive_trapezoid`
Expected: 3 PASSED.

- [ ] **Step 2.5: Тесты для треугольника, turn, edge-кейсов**

Дописать в тестовый файл:
```python
def test_drive_triangular_short_distance():
    # D=0.01 < v²/a = 0.0225/0.2 = 0.1125 → triangle
    traj = build_reference(distance=0.01, v_target=0.15, target_heading=0.0,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    r_end = traj.r(traj.t_drive)
    np.testing.assert_allclose(r_end, [0.01, 0.0, 0.0, 0.0, 0.0], atol=1e-9)
    v_peak = max(traj.r(t)[1] for t in np.linspace(0, traj.t_drive, 100))
    assert v_peak < 0.15, 'triangle must not reach v_target'
    assert abs(v_peak - math.sqrt(0.01 * 0.20)) < 1e-6


def test_turn_trapezoid_endpoints():
    # φ=π = trapezoid (π > 0.25 rad threshold)
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=math.pi,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    r_end = traj.r(traj.t_end)
    assert abs(r_end[2] - math.pi) < 1e-9
    assert abs(r_end[3]) < 1e-9


def test_turn_triangular_short_angle():
    # |φ|=0.1 < ω²/α = 0.25/1.0 = 0.25 → triangle
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=0.1,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    r_end = traj.r(traj.t_end)
    assert abs(r_end[2] - 0.1) < 1e-9


def test_turn_negative_phi():
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=-0.5,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    r_end = traj.r(traj.t_end)
    assert abs(r_end[2] - (-0.5)) < 1e-9


def test_zero_distance_zero_phi():
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=0.0,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    assert traj.t_drive == 0.0
    assert traj.t_end == 0.0
    np.testing.assert_allclose(traj.r(0.0), [0.0, 0.0, 0.0, 0.0, 0.0], atol=1e-12)


def test_continuity_at_t_drive():
    traj = build_reference(distance=0.30, v_target=0.15, target_heading=math.pi,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    left = traj.r(traj.t_drive - 1e-6)
    right = traj.r(traj.t_drive + 1e-6)
    np.testing.assert_allclose(left, right, atol=1e-3)


def test_r_after_t_end_returns_final():
    traj = build_reference(distance=0.30, v_target=0.15, target_heading=math.pi,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5)
    r_late = traj.r(traj.t_end + 100.0)
    np.testing.assert_allclose(
        r_late, [0.30, 0.0, math.pi, 0.0, 0.0], atol=1e-9)


def test_theta_start_offset():
    # Сценарий стартует не из θ=0
    traj = build_reference(distance=0.0, v_target=0.15, target_heading=0.5,
                           a_max=0.20, alpha_max=1.0, omega_max=0.5,
                           theta_start=1.0)
    r_end = traj.r(traj.t_end)
    assert abs(r_end[2] - 1.5) < 1e-9
```

- [ ] **Step 2.6: Запустить все тесты модуля**

Run: `pytest tests/test_mps_reference.py -v`
Expected: 11 PASSED.

- [ ] **Step 2.7: Commit**

```bash
git add pi_nodes/control/mps_reference.py tests/test_mps_reference.py
git commit -m "feat(mps): ReferenceTrajectory.r(t) — интерполяция drive/turn сегментов"
```

---

## Task 3: Schemas — добавить поля `r`, `x_local`, `y_local`, статус `timeout_settle`

**Files:**
- Modify: `compute_node/dashboard/schemas/mps.py:1-220`
- Modify: тестов под schema нет — Pydantic-валидация покрывается ниже в task 4/5.

- [ ] **Step 3.1: Найти `MpsTelemetryPoint` и расширить**

Открыть `compute_node/dashboard/schemas/mps.py`, найти класс `MpsTelemetryPoint` (~стр. 199). Добавить после `delta_theta`:
```python
    r: list[float] | None = Field(
        default=None,
        description='Опорный 5-вектор r(t) = [s_ref, v_ref, θ_ref, ω_ref, e_int_ref] '
                    'на этом тике. None для старой телеметрии (pre-2026-05-17).',
    )
    x_local: float | None = Field(
        default=None,
        description='Позиция робота в локальном фрейме старта (X-локальное = '
                    'курс на момент _on_scenario_run). None для старой телеметрии.',
    )
    y_local: float | None = Field(
        default=None,
        description='Позиция робота в локальном фрейме старта (Y-локальное). '
                    'None для старой телеметрии.',
    )

    @field_validator('r')
    @classmethod
    def _check_r_len(cls, v: list[float] | None) -> list[float] | None:
        if v is not None and len(v) != N_STATES:
            raise ValueError(f'r must have {N_STATES} elements')
        return v
```

- [ ] **Step 3.2: Найти `ScenarioStatus` и добавить `'timeout_settle'`**

В том же файле найти определение `ScenarioStatus` (Literal). Заменить:
```python
ScenarioStatus = Literal['running', 'reached', 'timeout', 'timeout_settle', 'aborted', 'error']
```

- [ ] **Step 3.3: Обновить docstring `target_heading`**

В `MpsScenarioRequest.target_heading` (~стр. 187-194) заменить description:
```python
        description='Финальный курс φ (рад) после прибытия в (D, 0) локального '
                    'фрейма старта. 0.0 = не разворачивается; π = разворот на '
                    '180° после доезда. Используется только при source="robot".'
```

- [ ] **Step 3.4: Bump `MPS_SCHEMA_VERSION`**

В том же файле найти `MPS_SCHEMA_VERSION = '1.1'`, заменить на:
```python
# 1.2: добавлены опциональные r/x_local/y_local в MpsTelemetryPoint;
# статус 'timeout_settle'; семантика target_heading = final heading.
MPS_SCHEMA_VERSION = '1.2'
```

- [ ] **Step 3.5: Прогнать существующие тесты — ничего не должно сломаться**

Run: `pytest tests/test_mps_router.py tests/test_mps_runner.py -v`
Expected: PASS (старые тесты не используют новые поля).

- [ ] **Step 3.6: Commit**

```bash
git add compute_node/dashboard/schemas/mps.py
git commit -m "feat(mps): schema 1.2 — r/x_local/y_local в TelemetryPoint, timeout_settle"
```

---

## Task 4: `mps_runner.py` — переход на feedforward-референс

**Files:**
- Modify: `compute_node/mps_runner.py:157-310`
- Test: `tests/test_mps_runner.py`

- [ ] **Step 4.1: Failing-тест на pose-arrival**

Дописать в `tests/test_mps_runner.py` (после существующих):
```python
import math
import numpy as np

from compute_node.mps_runner import run_scenario_idealized
from compute_node.dashboard.schemas.mps import (
    MpsMatrices, MpsScenarioRequest,
)
# default-matrices helper из этого же файла; имя — то, что уже используется в test_mps_runner.py
# (см. _default_matrices() или фикстуру; повторить тот же конструктор).

def test_runner_pose_arrival_d03_phi_pi(default_matrices):
    req = MpsScenarioRequest(distance=0.30, v_target=0.15,
                             target_heading=math.pi, source='sim')
    result = run_scenario_idealized(default_matrices, req)
    assert result.status == 'reached', \
        f"expected reached, got {result.status}: {result.error_message}"
    last = result.telemetry[-1]
    s_final = last.x[0]
    theta_final = last.x[2]
    assert abs(s_final - 0.30) < 0.005, f'final s={s_final}'
    # wrap угла перед сравнением
    err = (theta_final - math.pi + math.pi) % (2 * math.pi) - math.pi
    assert abs(err) < 0.05, f'final θ_err={err}'
```

Если в `test_mps_runner.py` нет фикстуры `default_matrices`, посмотреть как существующие тесты конструируют `MpsMatrices` и сделать аналогично (фикстура `@pytest.fixture` в том же файле).

- [ ] **Step 4.2: Запустить — упадёт (старый runner стопит за 5 см)**

Run: `pytest tests/test_mps_runner.py::test_runner_pose_arrival_d03_phi_pi -v`
Expected: FAIL. Либо `status != 'reached'`, либо `theta_final ≠ π`.

- [ ] **Step 4.3: Переписать `run_scenario_idealized` через `build_reference`**

В `compute_node/mps_runner.py` импорт сверху:
```python
from pi_nodes.control.mps_reference import build_reference
```

Заменить тело main-loop (между «build plant + controller» и `return MpsScenarioResult`) — заменяет существующие строки ~232-294:
```python
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

        x = plant.step(x, u)
        t += dt
```

Не забыть `import math` в верх файла, если ещё не импортирован.

- [ ] **Step 4.4: Запустить — тест должен пройти**

Run: `pytest tests/test_mps_runner.py::test_runner_pose_arrival_d03_phi_pi -v`
Expected: PASS. Если падает на `theta_err` — проверь, что `mpc.step` получает корректный `r_ref[2] = θ_target` после `t > t_drive` (он должен).

- [ ] **Step 4.5: Прогнать ВСЕ тесты runner'a**

Run: `pytest tests/test_mps_runner.py -v`
Expected: все PASS. Если старые тесты ломаются на изменении docstring «достиг цели» (`5 см → 5 мм`) — обновить assertions.

- [ ] **Step 4.6: Commit**

```bash
git add compute_node/mps_runner.py tests/test_mps_runner.py
git commit -m "feat(mps): runner на feedforward-референс, финиш по 4 координатам"
```

---

## Task 5: `mps_node.py` — `_tick_run`, `_check_finish`, x_local/y_local

**Files:**
- Modify: `pi_nodes/nodes/mps_node.py:1-810`
- Test: `tests/test_mps_node.py`

- [ ] **Step 5.1: Failing-тест на pose-arrival с фейк-плантом**

В `tests/test_mps_node.py` посмотреть, как существующие тесты гоняют ноду (фейк-MQTT, искусственная одометрия). Дописать:
```python
import math


def test_mps_node_pose_arrival_d03_phi_pi(mps_node_fixture):
    """D=0.30, v=0.15, φ=π → status='reached', s≈D, θ≈π."""
    node, fake_mqtt, fake_odom = mps_node_fixture
    fake_odom.start_at(x=0.0, y=0.0, theta=0.0)

    node._on_scenario_run(payload={
        'run_id': 'test-pose',
        'request': {'distance': 0.30, 'v_target': 0.15,
                    'target_heading': math.pi, 'source': 'robot'},
        'reference': {'a_max': 0.20, 'alpha_max': 1.0},
    })
    # Симулируем идеальный плант: x_meas ← интеграл cmd_vel
    for _ in range(800):  # ~16 с при tick=0.02
        node._tick()
        fake_odom.integrate_cmd_vel(fake_mqtt.last_cmd_vel, dt=node._tick_dt)
        node._on_odom(payload=fake_odom.payload())
        if node._run is None:  # сценарий финишировал
            break

    status_msg = fake_mqtt.find_published('mps/scenario/finished')
    assert status_msg['status'] == 'reached'
    last = status_msg['telemetry'][-1]
    assert abs(last['x'][0] - 0.30) < 0.01
    err = (last['x'][2] - math.pi + math.pi) % (2 * math.pi) - math.pi
    assert abs(err) < 0.05
```

Если фикстуры `mps_node_fixture` нет — посмотреть в существующем `test_mps_node.py` (и `tests/conftest.py`) на паттерн фейк-MQTT и адаптировать.

- [ ] **Step 5.2: Запустить — упадёт (старый код, две фазы)**

Run: `pytest tests/test_mps_node.py::test_mps_node_pose_arrival_d03_phi_pi -v`
Expected: FAIL. Скорее всего `status != 'reached'` или `θ_final` не π.

- [ ] **Step 5.3: Заменить `_RunState` — выпилить phase, добавить traj**

В `pi_nodes/nodes/mps_node.py` найти `class _RunState` (стр. 71-126). Заменить полностью:
```python
class _RunState:
    """Локальное состояние active run (только внутри mps_node)."""
    __slots__ = (
        'run_id', 'distance', 'v_target', 'target_heading', 'started_at',
        'telemetry', 't', 'no_odom_ticks',
        's_start', 'theta_start',
        'x_start_abs', 'y_start_abs',
        'lateral_lqr', 'traj',
    )

    def __init__(self, run_id: str, distance: float, v_target: float,
                 target_heading: float,
                 traj,  # ReferenceTrajectory
                 s_start: float = 0.0, theta_start: float = 0.0,
                 x_start_abs: float = 0.0, y_start_abs: float = 0.0,
                 lateral_lqr: Optional[LateralLqrController] = None):
        self.run_id = run_id
        self.distance = distance
        self.v_target = v_target
        self.target_heading = target_heading
        self.started_at = datetime.now(timezone.utc)
        self.telemetry: list[dict] = []
        self.t = 0.0
        self.no_odom_ticks = 0
        self.s_start = s_start
        self.theta_start = theta_start
        self.x_start_abs = x_start_abs
        self.y_start_abs = y_start_abs
        self.lateral_lqr = lateral_lqr
        self.traj = traj
```

- [ ] **Step 5.4: Импортировать `build_reference`, добавить config-чтение**

В шапке `mps_node.py` (рядом с другими импортами, стр. ~47):
```python
from pi_nodes.control.mps_reference import build_reference
```

В `__init__` после `self._reach_eps` (стр. ~170) добавить:
```python
        # Pose-tracking финиш (см. spec §4.1)
        self._eps_v = float(self._cfg('mps.scenario.reach.epsilon_v', 0.02))
        self._eps_theta = float(self._cfg('mps.scenario.reach.epsilon_theta', 0.05))
        self._eps_omega = float(self._cfg('mps.scenario.reach.epsilon_omega', 0.05))
        self._settle_timeout = float(self._cfg('mps.scenario.reach.settle_timeout_s', 1.5))
        # Параметры референса
        self._ref_a_max = float(self._cfg('mps.scenario.reference.a_max', 0.20))
        self._ref_alpha_max = float(self._cfg('mps.scenario.reference.alpha_max', 1.0))

        # Deprecation warnings
        if self._cfg('mps.scenario.turn_tolerance_rad', None) is not None:
            self.log_warn('mps: config key turn_tolerance_rad is deprecated '
                          '(unused since pose-tracking refactor)')
        if self._cfg('mps.scenario.turn_timeout_s', None) is not None:
            self.log_warn('mps: config key turn_timeout_s is deprecated')
```

- [ ] **Step 5.5: Перестроить `_on_scenario_run` — строить traj**

Найти `_on_scenario_run` (стр. ~310-410). В месте, где создаётся `_RunState`, заменить блок построения на:
```python
        try:
            traj = build_reference(
                distance=float(request['distance']),
                v_target=float(request['v_target']),
                target_heading=float(request.get('target_heading', 0.0)),
                a_max=float(payload.get('reference', {}).get('a_max', self._ref_a_max)),
                alpha_max=float(payload.get('reference', {}).get('alpha_max', self._ref_alpha_max)),
                omega_max=self._omega_max_turn,
                theta_start=theta_start,
            )
        except ValueError as exc:
            self._publish_precondition_error(run_id, f'reference build failed: {exc}')
            return

        run = _RunState(
            run_id=run_id,
            distance=float(request['distance']),
            v_target=float(request['v_target']),
            target_heading=float(request.get('target_heading', 0.0)),
            traj=traj,
            s_start=s_start, theta_start=theta_start,
            x_start_abs=x_start_abs, y_start_abs=y_start_abs,
            lateral_lqr=lateral_lqr,
        )
```

(точная подстановка зависит от существующего кода — сохранить `s_start`, `theta_start`, `x_start_abs`, `y_start_abs`, `lateral_lqr` как они вычисляются сейчас).

- [ ] **Step 5.6: Удалить `_tick_turn` и `_tick_drive`, написать `_tick_run`**

Найти `_tick_turn` (стр. ~470-520) и `_tick_drive` (стр. ~570-665). Удалить оба. Заменить вызовы в `_tick` (стр. ~440-460):
```python
    def _tick(self):
        if self._fsm_state != 'DRIVE_FORWARD_MPS' or self._run is None:
            return
        self._tick_run(self._run)
```

Добавить метод `_tick_run` (можно на место удалённого `_tick_drive`):
```python
    def _tick_run(self, run: _RunState):
        with self._lock:
            x = self._x_meas.copy()
            x_abs = self._x_abs
            y_abs = self._y_abs

        # Опорный 5-вектор
        r_ref = run.traj.r(run.t)

        # MPC step
        try:
            u = self._mpc.step(x, x_ref=r_ref)
        except Exception as exc:
            self._finish_run('error', f'mpc.step: {exc}')
            return
        if not (np.all(np.isfinite(u)) and np.all(np.isfinite(x))):
            self._finish_run('error', 'NaN/Inf in u or x')
            return

        # Hard-cap по ω: в drive-сегменте (v_ref ≠ 0) — omega_max_in_forward,
        # в turn — omega_max_in_turn (уже стоит как cap в MPC).
        if abs(r_ref[1]) > 1e-3:
            u[1] = max(-self._omega_max_fwd, min(self._omega_max_fwd, u[1]))

        # Интегратор курсовой ошибки против ТЕКУЩЕГО θ_ref(t)
        theta_err = _normalize_angle(x[_THETA] - r_ref[_THETA])
        with self._lock:
            new_eint = self._x_meas[_EINT] + (-theta_err) * self._tick_dt
            self._x_meas[_EINT] = max(-self._e_int_max,
                                       min(self._e_int_max, new_eint))

        # Локальные координаты для UI
        dx = x_abs - run.x_start_abs
        dy = y_abs - run.y_start_abs
        cs = math.cos(run.theta_start)
        sn = math.sin(run.theta_start)
        x_local = dx * cs + dy * sn
        y_local = -dx * sn + dy * cs

        # Outer LQR (как было — сохраняем для bot side comp; e_y по локальной y)
        e_y = y_local
        delta_theta = 0.0
        if run.lateral_lqr is not None:
            try:
                delta_theta = run.lateral_lqr.step(e_y, theta_err)
            except Exception as exc:
                self.log_warn('mps: lateral_lqr.step failed (%s) — skipping outer', exc)

        self._publish_cmd_and_telemetry(
            run, x, u,
            e_y=e_y, theta_err=theta_err, delta_theta=delta_theta,
            r=r_ref, x_local=x_local, y_local=y_local,
        )

        with self._lock:
            run.no_odom_ticks += 1
            stale = run.no_odom_ticks > _WATCHDOG_TICKS

        if self._check_finish(x, r_ref, run):
            return

        if stale and time.time() - self._x_meas_ts > 5.0 * self._tick_dt:
            self._finish_run('error', 'watchdog: no odom for >3 ticks')
            return

        run.t += self._tick_dt

    def _check_finish(self, x: np.ndarray, r_ref: np.ndarray, run: _RunState) -> bool:
        """True если сценарий завершён (status выставлен в _finish_run)."""
        if run.t < run.traj.t_end:
            # ещё в drive/turn — финиш невозможен
            # но run-timeout проверим
            if run.t > max(1.0, 1.5 * run.traj.t_end + 2.0):
                self._finish_run('timeout', 'run timeout')
                return True
            return False

        theta_target = run.traj.theta_start + run.traj.phi_signed
        theta_err = _normalize_angle(x[_THETA] - theta_target)
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
```

- [ ] **Step 5.7: Обновить `_publish_cmd_and_telemetry` — принимать `r/x_local/y_local`**

Найти `_publish_cmd_and_telemetry` (стр. ~666+). Расширить сигнатуру:
```python
    def _publish_cmd_and_telemetry(
        self,
        run: _RunState,
        x: np.ndarray, u: np.ndarray,
        *,
        e_y: Optional[float] = None,
        theta_err: Optional[float] = None,
        delta_theta: Optional[float] = None,
        r: Optional[np.ndarray] = None,
        x_local: Optional[float] = None,
        y_local: Optional[float] = None,
    ) -> None:
```

В формировании `point` (телеметрический dict) добавить:
```python
        if r is not None:
            point['r'] = [float(v) for v in r]
        if x_local is not None:
            point['x_local'] = float(x_local)
        if y_local is not None:
            point['y_local'] = float(y_local)
```

- [ ] **Step 5.8: Запустить тест pose-arrival**

Run: `pytest tests/test_mps_node.py::test_mps_node_pose_arrival_d03_phi_pi -v`
Expected: PASS.

- [ ] **Step 5.9: Settling-timeout тест**

Дописать в `tests/test_mps_node.py`:
```python
def test_mps_node_settle_timeout_reports_detail(mps_node_fixture):
    """Замедленный плант не сходится по v → status='timeout_settle' с деталью."""
    node, fake_mqtt, fake_odom = mps_node_fixture
    fake_odom.set_velocity_lag(0.5)  # сильно тормозит — v не успеет в 0
    fake_odom.start_at(0.0, 0.0, 0.0)
    node._on_scenario_run(payload={
        'run_id': 'test-settle',
        'request': {'distance': 0.30, 'v_target': 0.15,
                    'target_heading': 0.0, 'source': 'robot'},
        'reference': {'a_max': 0.20, 'alpha_max': 1.0},
    })
    for _ in range(2000):
        node._tick()
        fake_odom.integrate_cmd_vel(fake_mqtt.last_cmd_vel, dt=node._tick_dt)
        node._on_odom(payload=fake_odom.payload())
        if node._run is None: break
    msg = fake_mqtt.find_published('mps/scenario/finished')
    assert msg['status'] == 'timeout_settle'
    assert 'settle timeout' in (msg.get('error_message') or '')
```

(если у твоего фейка нет `set_velocity_lag` — пропусти этот тест или построй простую обёртку через `dt`-затягивание).

- [ ] **Step 5.10: Запустить ВСЕ тесты ноды**

Run: `pytest tests/test_mps_node.py -v`
Expected: все PASS. Старые тесты, ссылающиеся на `_tick_turn`/`_tick_drive`/`phase`, нужно либо удалить, либо переписать через `_tick_run`.

- [ ] **Step 5.11: Commit**

```bash
git add pi_nodes/nodes/mps_node.py tests/test_mps_node.py
git commit -m "feat(mps): pi-нода на feedforward-референс, финиш по 4 координатам"
```

---

## Task 6: `routers/mps.py` — payload `reference: {...}`

**Files:**
- Modify: `compute_node/dashboard/routers/mps.py`
- Test: `tests/test_mps_router.py`

- [ ] **Step 6.1: Тест на новый payload-поле**

В `tests/test_mps_router.py` дописать:
```python
def test_scenario_run_includes_reference_in_mqtt_payload(client, fake_mqtt):
    """POST /api/mps/scenario/run с source=robot публикует payload с reference."""
    resp = client.post('/api/mps/scenario/run', json={
        'distance': 0.30, 'v_target': 0.15, 'target_heading': 0.0,
        'source': 'robot',
    })
    assert resp.status_code == 200
    pub = fake_mqtt.find_published('mps/scenario/run')
    assert pub is not None
    assert 'reference' in pub
    assert pub['reference']['a_max'] > 0
    assert pub['reference']['alpha_max'] > 0
```

(Если фикстуры `client`/`fake_mqtt` названы иначе — адаптируй по существующим тестам в файле).

- [ ] **Step 6.2: Запустить — упадёт (поля нет)**

Run: `pytest tests/test_mps_router.py::test_scenario_run_includes_reference_in_mqtt_payload -v`
Expected: FAIL — `'reference' not in pub`.

- [ ] **Step 6.3: Расширить payload в `routers/mps.py`**

Найти место публикации `mqtt.publish('mps/scenario/run', payload, qos=1)` (стр. ~268-310). Заменить блок построения `payload`:
```python
    payload = {
        'run_id': run_id,
        'request': request.model_dump(),
        'schema_version': matrices.schema_version,
        'reference': {
            'a_max': 0.20,     # дефолты compute-side; Pi может перекрыть
            'alpha_max': 1.0,
        },
    }
```

- [ ] **Step 6.4: Запустить — должен пройти**

Run: `pytest tests/test_mps_router.py -v`
Expected: все PASS.

- [ ] **Step 6.5: Commit**

```bash
git add compute_node/dashboard/routers/mps.py tests/test_mps_router.py
git commit -m "feat(mps): router пробрасывает reference: {a_max, alpha_max} в MQTT"
```

---

## Task 7: `config.yaml` — новые ключи, дефолт ε_s = 5 мм

**Files:**
- Modify: `config.yaml` (корень репозитория)

- [ ] **Step 7.1: Прочитать текущую секцию `mps.scenario`**

Открыть `config.yaml`, найти секцию `mps.scenario`. Запомнить существующие ключи.

- [ ] **Step 7.2: Заменить секцию**

Заменить блок `mps.scenario` на:
```yaml
mps:
  # ... остальная секция mps (matrices, plant, control, etc.) — без изменений
  scenario:
    distance_max: 5.0
    v_target_max: 0.30
    omega_max_in_forward: 0.5
    omega_max_in_turn: 1.0
    odom_max_age_s: 0.5
    e_int_max: 0.5

    # Tolerance «доехал по s» (pose-tracking, см. spec §4.1)
    # Был 0.02 м (роботу разрешалось стопить за 2 см до цели —
    # без замедления это было необходимо). С feedforward-референсом
    # MPC замедляется к нулю, поэтому ε можно ужать до 5 мм.
    reach_tolerance_m: 0.005

    # Остальные ε (по v/θ/ω) + settle-timeout
    reach:
      epsilon_v: 0.02       # m/s
      epsilon_theta: 0.05   # rad
      epsilon_omega: 0.05   # rad/s
      settle_timeout_s: 1.5

    # Параметры построения опорной траектории r(t).
    # Шарятся между Pi (mps_node) и compute (mps_runner) через MQTT-payload.
    reference:
      a_max: 0.20           # m/s² (linear accel в drive-trapezoid)
      alpha_max: 1.0        # rad/s² (angular accel в turn-trapezoid)

    # ── DEPRECATED — игнорируются с 2026-05-17 (pose-tracking refactor),
    # удалить через релиз. Логируются как deprecation в mps_node.__init__.
    # turn_tolerance_rad: 0.05
    # turn_timeout_s: 10.0

    lateral:
      enabled: true
      tau_inner: 0.10
      Q_diag: [80.0, 30.0]
      R_diag: [1.0]
      delta_theta_max: 0.20
      v_min: 0.02
```

- [ ] **Step 7.3: Запустить smoke — конфиг должен парситься**

Run: `python -c "from config_loader import cfg; print(cfg('mps.scenario.reach.epsilon_v', None))"`
Expected: `0.02` (или None если `config_loader` не находит — тогда проверь правильность пути и YAML-отступы).

- [ ] **Step 7.4: Commit**

```bash
git add config.yaml
git commit -m "config(mps): pose-tracking — reach.* / reference.*; ε_s=0.005м"
```

---

## Task 8: Frontend types — `MpsTelemetryPoint` + `ScenarioStatus`

**Files:**
- Modify: `compute_node/frontend/src/types/mps.ts`

- [ ] **Step 8.1: Обновить `MPS_SCHEMA_VERSION` и `MpsTelemetryPoint`**

В `compute_node/frontend/src/types/mps.ts`:
- Заменить `MPS_SCHEMA_VERSION = '1.1'` на `'1.2'`, обновить комментарий выше.
- В интерфейсе `MpsTelemetryPoint` (стр. 56-76) добавить после `delta_theta`:
```typescript
  /** Опорный 5-вектор r(t) = [s_ref, v_ref, θ_ref, ω_ref, e_int_ref] на этом тике.
   *  null/undefined для старой телеметрии (pre-2026-05-17, pre-1.2). */
  r?: number[]
  /** Позиция в локальном фрейме старта (X-локальное вдоль курса на старте). */
  x_local?: number
  y_local?: number
```

- [ ] **Step 8.2: Обновить `ScenarioStatus`**

Заменить (стр. ~93):
```typescript
export type ScenarioStatus = 'running' | 'reached' | 'timeout' | 'timeout_settle' | 'aborted' | 'error'
```

- [ ] **Step 8.3: Обновить docstring `target_heading` в `MpsScenarioRequest`**

Заменить комментарий перед `target_heading?: number` (стр. 50-52) на:
```typescript
  /** Финальный курс φ (рад, −π…π) после прибытия в (D, 0) локального фрейма
   *  старта. 0.0 = не разворачивается; π = разворот на 180° после доезда. */
  target_heading?: number
```

- [ ] **Step 8.4: tsc должен проходить**

Run: `cd compute_node/frontend && npx tsc --noEmit`
Expected: 0 errors.

- [ ] **Step 8.5: Commit**

```bash
git add compute_node/frontend/src/types/mps.ts
git commit -m "feat(mps/ui): types — schema 1.2 (r/x_local/y_local, timeout_settle)"
```

---

## Task 9: `TrajectoryView.tsx` — путь из x_local/y_local + пунктирный план + стрелка

**Files:**
- Modify: `compute_node/frontend/src/components/mps/TrajectoryView.tsx`
- Test: `compute_node/frontend/src/components/mps/TrajectoryView.test.tsx`

- [ ] **Step 9.1: Failing-тесты на план + arrow + 4-элементную легенду**

В `TrajectoryView.test.tsx` дописать:
```tsx
import { render, screen } from '@testing-library/react'
import { TrajectoryView } from '../TrajectoryView'
import type { MpsScenarioResult, MpsTelemetryPoint } from '@/types/mps'

function makeResult(telemetry: MpsTelemetryPoint[], distance = 0.30): MpsScenarioResult {
  return {
    run_id: 't', started_at: '', finished_at: null,
    status: 'reached',
    request: { distance, v_target: 0.15, target_heading: Math.PI, source: 'robot' },
    matrices_snapshot: {} as any,
    telemetry,
    metrics: null,
    schema_version: '1.2',
  }
}

test('renders dashed plan line from (0,0) to (D,0)', () => {
  const { container } = render(<TrajectoryView result={makeResult([])} />)
  const dashed = container.querySelector('path[stroke-dasharray]')
  expect(dashed).toBeInTheDocument()
})

test('renders heading arrow at target', () => {
  const { container } = render(<TrajectoryView result={makeResult([])} />)
  const arrow = container.querySelector('[data-testid="target-heading-arrow"]')
  expect(arrow).toBeInTheDocument()
})

test('legend has 4 entries (план, путь, target, текущая)', () => {
  render(<TrajectoryView result={makeResult([])} />)
  expect(screen.getByText('план')).toBeInTheDocument()
  expect(screen.getByText('путь')).toBeInTheDocument()
  expect(screen.getByText('target')).toBeInTheDocument()
  expect(screen.getByText('текущая')).toBeInTheDocument()
})

test('uses x_local/y_local when present in telemetry', () => {
  const t: MpsTelemetryPoint[] = [
    { t: 0, x: [0,0,0,0,0], u: [0,0], y: [], s_remaining: 0.3,
      x_local: 0, y_local: 0 },
    { t: 1, x: [0.15,0.15,0,0,0], u: [0,0], y: [], s_remaining: 0.15,
      x_local: 0.15, y_local: 0 },
  ]
  const { container } = render(<TrajectoryView result={makeResult(t)} />)
  const path = container.querySelector('path[stroke="#2563eb"]')
  expect(path?.getAttribute('d')).toContain('L')  // более одной точки
})
```

- [ ] **Step 9.2: Запустить — все 4 упадут**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/TrajectoryView.test.tsx`
Expected: 4 FAIL (нет dashed-path, нет arrow, нет «план» в легенде, путь рисуется через старую формулу).

- [ ] **Step 9.3: Переписать `points2d` + добавить план/стрелку**

В `compute_node/frontend/src/components/mps/TrajectoryView.tsx` заменить `points2d` (стр. 19-27):
```typescript
function points2d(telemetry: MpsTelemetryPoint[]): { x: number; y: number }[] {
  return telemetry.map((p) => {
    // Если телеметрия из 1.2+ — есть x_local/y_local, рисуем геометрически верно
    if (p.x_local !== undefined && p.y_local !== undefined) {
      return { x: p.x_local, y: p.y_local }
    }
    // Fallback для pre-1.2 прогонов из History
    return {
      x: (p.x[0] ?? 0) * Math.cos(p.x[2] ?? 0),
      y: (p.x[0] ?? 0) * Math.sin(p.x[2] ?? 0),
    }
  })
}
```

В компоненте `TrajectoryView` после вычисления `target` (стр. ~57) добавить:
```typescript
  const phi = result?.request.target_heading ?? 0
  // Пунктирный план — прямая (0,0)→(D,0) в локальном фрейме
  const planStart = project(0, 0)
  const planEnd = project(distance, 0)
  // Короткая стрелка финального курса в (D,0), длина ~0.08·distance в мирe
  const arrowLen = Math.max(0.05, distance * 0.15)
  const arrowTip = project(distance + arrowLen * Math.cos(phi),
                            arrowLen * Math.sin(phi))
```

В JSX svg (между axes и target, стр. ~70-80) добавить:
```tsx
          {/* план: пунктир (0,0)→(D,0) */}
          <line x1={planStart[0]} y1={planStart[1]}
                x2={planEnd[0]} y2={planEnd[1]}
                stroke="#94a3b8" strokeWidth={1.5}
                strokeDasharray="4 3" />
          {/* стрелка финального курса в target */}
          <line data-testid="target-heading-arrow"
                x1={target[0]} y1={target[1]}
                x2={arrowTip[0]} y2={arrowTip[1]}
                stroke="#16a34a" strokeWidth={1.5}
                markerEnd="url(#arrowhead)" />
```

Перед `</svg>` (в самом верху svg, после `<rect>`) добавить marker для стрелки:
```tsx
          <defs>
            <marker id="arrowhead" markerWidth="6" markerHeight="6"
                    refX="5" refY="3" orient="auto">
              <path d="M0,0 L6,3 L0,6 z" fill="#16a34a" />
            </marker>
          </defs>
```

В легенде (стр. 103-111) добавить первым элементом «план»:
```tsx
          <g transform={`translate(${PADDING + 4}, 14)`} fontSize={10} fontFamily="monospace">
            <line x1={0} y1={0} x2={20} y2={0} stroke="#94a3b8" strokeWidth={1.5} strokeDasharray="4 3" />
            <text x={26} y={3} fill="#94a3b8">план</text>
            <line x1={70} y1={0} x2={90} y2={0} stroke="#2563eb" strokeWidth={1.5} />
            <text x={96} y={3} fill="#2563eb">путь</text>
            <circle cx={140} cy={0} r={4} fill="none" stroke="#16a34a" strokeWidth={1.5} />
            <text x={150} y={3} fill="#16a34a">target</text>
            <circle cx={200} cy={0} r={3} fill="#dc2626" />
            <text x={208} y={3} fill="#dc2626">текущая</text>
          </g>
```

Заменить подпись внизу карточки (стр. 127-129):
```tsx
        <div className="mt-1 text-xs text-muted-foreground">
          Локальный фрейм старта; план показан штрихом.
        </div>
```

- [ ] **Step 9.4: Запустить — все 4 теста должны пройти**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/TrajectoryView.test.tsx`
Expected: все PASS.

- [ ] **Step 9.5: tsc + lint**

Run: `cd compute_node/frontend && npx tsc --noEmit && npm run lint -- src/components/mps/TrajectoryView.tsx`
Expected: 0 errors.

- [ ] **Step 9.6: Commit**

```bash
git add compute_node/frontend/src/components/mps/TrajectoryView.tsx compute_node/frontend/src/components/mps/TrajectoryView.test.tsx
git commit -m "feat(mps/ui): план + стрелка курса в TrajectoryView; x_local/y_local"
```

---

## Task 10: `ScenarioControls.tsx` — обновить лейбл `target_heading`

**Files:**
- Modify: `compute_node/frontend/src/components/mps/ScenarioControls.tsx`

- [ ] **Step 10.1: Найти и заменить лейбл**

Открыть `ScenarioControls.tsx`, найти лейбл и хелпер для `target_heading` (поиск по `target_heading` или «разворот»). Заменить лейбл на:
```tsx
<Label>Финальный курс φ (рад)</Label>
<HelpText>
  Куда робот будет смотреть после прибытия в (D, 0). φ=0 — не разворачивается;
  π — разворот на 180° после доезда.
</HelpText>
```

(Точная структура — Label/HelpText/что-то — зависит от существующих компонентов в файле; сохрани стиль.)

- [ ] **Step 10.2: Если есть snapshot-тест для ScenarioControls — обновить**

Run: `cd compute_node/frontend && npx vitest run -u src/components/mps/ScenarioControls`
Expected: PASS (либо snapshot обновится).

- [ ] **Step 10.3: tsc**

Run: `cd compute_node/frontend && npx tsc --noEmit`
Expected: 0 errors.

- [ ] **Step 10.4: Commit**

```bash
git add compute_node/frontend/src/components/mps/ScenarioControls.tsx
git commit -m "feat(mps/ui): лейбл target_heading → «Финальный курс φ»"
```

---

## Task 11: Документация — `scenario_pose.md`, удалить `scenario_forward.md`

**Files:**
- Create: `docs/mps/scenario_pose.md`
- Delete: `docs/mps/scenario_forward.md`

- [ ] **Step 11.1: Создать `docs/mps/scenario_pose.md`**

```markdown
# MPS-сценарий «Pose-tracking» (`scenario_pose`)

> Текущий и единственный сценарий MPS-модуля. Заменил `scenario_forward.md`
> 2026-05-17 (см. [спека](../superpowers/specs/2026-05-17-mps-pose-tracking-design.md)).

## Что делает

Принимает три параметра:

| Поле | Тип | Диапазон | Смысл |
|---|---|---|---|
| `distance` | float, м | `(0, 5.0]` | Перемещение по X локального фрейма старта |
| `v_target` | float, м/с | `(0, 0.30]` | Крейс-скорость в drive-сегменте |
| `target_heading` | float, рад | `[-π, π]` | **Финальный** курс после прибытия |

Робот:
1. Едет вперёд на `distance` метров, разгоняясь до `v_target` и тормозя в
   конце (трапец/треугольник в зависимости от `distance`).
2. Поворачивается на месте в `(D, 0)` к курсу `θ_start + target_heading`.

В коде один MPC закрывает петлю по `r(t) − x(t)`; «фаз» нет.

## Финиш

`status='reached'` когда **одновременно**:
- `|s_meas − D| < 0.005 м`
- `|v_meas| < 0.02 м/с`
- `|θ_meas − (θ_start + φ)| < 0.05 рад`
- `|ω_meas| < 0.05 рад/с`

Иначе через `1.5 с` после `t_end` — `status='timeout_settle'` с указанием,
по какой координате не сошлось.

## Smoke-тесты на железе

| D | v_target | φ | Ожидаемое t | s_final | θ_final |
|---|---|---|---|---|---|
| 0.30 | 0.15 | π | ≈ 9.5 c ± 1 | 0.295…0.305 | π ± 0.05 |
| 0.30 | 0.15 | 0 | ≈ 2.75 c ± 0.5 | 0.295…0.305 | 0 ± 0.05 |
| 0 | 0.15 | π/2 | ≈ 2.6 c ± 0.5 | 0 ± 0.005 | π/2 ± 0.05 |

## Внутренности

- Опорная траектория: `pi_nodes/control/mps_reference.py`.
- Контроллер: `pi_nodes/control/mpc_controller.py` (без изменений).
- Pi-нода: `pi_nodes/nodes/mps_node.py` (`_tick_run`).
- Sim: `compute_node/mps_runner.py` (`run_scenario_idealized`).
- Визуализация: `compute_node/frontend/src/components/mps/TrajectoryView.tsx`.
- Конфиг: `config.yaml` → `mps.scenario.{reach,reference}`.

## Что было до этого

Раньше `target_heading` значил «куда разворачиваемся ПЕРЕД движением», и
сценарий стопил за 2 см до D. Подробности — в коммите этого изменения и
в спеке выше.
```

- [ ] **Step 11.2: Удалить старый `scenario_forward.md`**

Run: `git rm docs/mps/scenario_forward.md`

- [ ] **Step 11.3: Поправить ссылки на старый файл, если есть**

Run: `git grep -l scenario_forward docs/ README.md 2>/dev/null || true`
Если что-то нашлось — заменить ссылки на `scenario_pose.md`.

- [ ] **Step 11.4: Commit**

```bash
git add docs/mps/scenario_pose.md docs/mps/scenario_forward.md
git commit -m "docs(mps): scenario_pose.md заменяет scenario_forward.md"
```

---

## Task 12: Финальная sanity-проверка

- [ ] **Step 12.1: Прогнать весь тест-сюит Python**

Run: `pytest tests/ -v --tb=short`
Expected: всё PASS. Если что-то сломалось — пофиксить, не маскировать.

- [ ] **Step 12.2: Прогнать весь frontend-сюит**

Run: `cd compute_node/frontend && npx vitest run`
Expected: всё PASS.

- [ ] **Step 12.3: Поднять стенд (sim) и руками прогнать сценарий**

```bash
./samurai.sh sim &
# В UI: открыть MPS-страницу, поставить D=0.30, v=0.15, target_heading=π,
# source=sim, нажать «Старт».
# Ожидаемое: статус reached, синяя кривая упирается в зелёный круг,
# стрелка курса смотрит влево (φ=π в локальном фрейме = −X).
```

Если поведение совпало — сделать скриншот для подтверждения.

- [ ] **Step 12.4: Документировать результат прогона**

Если поведение не такое — НЕ помечать план как готовый. Сначала
диагностировать и пофиксить.

---

## Notes для исполнителя

- **Не амендить старые коммиты** даже если хук падает — фикси и делай новый коммит.
- **Перед каждым коммитом** проверь `git status`, что коммитятся только
  ожидаемые файлы (никаких случайных `.env`, `node_modules` диффов).
- **Тесты должны падать перед тем, как их «исправит» имплементация** —
  если test passes без кода, тест ничего не проверяет.
- **Без Co-Authored-By Claude и без AI-упоминаний в коммитах** — этого
  требует устав проекта (см. memory_root).
- **Конфиг тестируем «как пользователь»** — `python -c "from config_loader
  import cfg; ..."` лучше, чем ручной yaml.safe_load.
