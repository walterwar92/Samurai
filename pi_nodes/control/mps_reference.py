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
