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


# Wrap convention: returns angle in [-π, π). At the boundary +π collapses to -π
# because (π + π) % 2π == 0, so 0 − π = −π. This means build_reference(target_heading=+π)
# produces a clockwise turn (sign = -1), landing at θ_start − π — same orientation
# as θ_start + π mod 2π. Test: test_turn_positive_pi_uses_cw_per_normalize_convention.
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

    После постройки иммутабельна. Метод `r(t)` дёшев (линейный поиск +
    арифметика; ≤3 сегмента в каждом профиле), безопасен в hot-loop'е
    tick'а MPC.
    """
    distance: float
    v_target: float
    target_heading: float        # final heading отн. θ_start
    # Used only by r(t) to build absolute θ_ref; segments themselves store
    # Δθ relative to start (multiplied by sign for direction).
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
        # Clamp negative t to 0 — спецификация неявно подразумевает t ∈ [0, ∞).
        # Без clamp при D=0 + t<0 _find_segment вызывается на пустом tuple
        # и падает IndexError. Сценарии r(t) запускаются от t=0, негативный
        # вход — только из REPL/debug.
        if t < 0.0:
            t = 0.0
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
        """Линейный поиск (макс 3 сегмента в каждом профиле — bisect overkill).
        Использует строгое `t < seg.t1` — на стыке сегментов (включая
        zero-length cruise при D = v²/a) выбирает следующий, а не предыдущий
        (см. follow-up #3 из code review Task 1)."""
        for seg in segments:
            if t < seg.t1:
                return seg
        return segments[-1]


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
    for _name, _val in (('a_max', a_max), ('alpha_max', alpha_max), ('omega_max', omega_max)):
        if not (_val > 0):
            raise ValueError(f'{_name} must be > 0, got {_val}')
    if abs(target_heading) > math.pi + 1e-9:  # numerical epsilon for π boundary
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
    elif abs_phi > 1e-9:  # numerical epsilon — not control tolerance (see ε_θ in spec §4.1)
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
