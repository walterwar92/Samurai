"""Tests for pi_nodes.control.lateral_lqr.LateralLqrController.

Свойства, которые проверяются:
  * Контроллер строится с дефолтными параметрами, закрытая подсистема устойчива.
  * Знак коррекции: e_y > 0 (робот сместился влево от линии) → δθ < 0 (поворот
    направо), и наоборот.
  * Выход всегда клипуется к ±delta_theta_max.
  * Деградации входов: v0=0, tau=0, R≤0, неверный размер Q отвергаются.
  * Симуляция замкнутой подсистемы из ненулевого ξ₀ сходится к нулю.
  * K имеет форму (1, 2).
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from pi_nodes.control.lateral_lqr import LateralLqrController  # noqa: E402


# Параметры по умолчанию матчат планируемый блок mps.scenario.lateral в config.yaml.
_DEFAULT_KW = dict(
    Ts=0.02,
    v0=0.15,
    tau_inner=0.10,
    Q_diag=[50.0, 5.0],
    R_diag=[1.0],
    delta_theta_max=0.30,
)


def test_construct_defaults_gives_stable_closed_loop():
    ctrl = LateralLqrController(**_DEFAULT_KW)
    eigs = ctrl.closed_loop_eigenvalues()
    assert ctrl.is_stable(), f"closed-loop unstable: |λ|={np.abs(eigs)}"


def test_K_shape_is_1x2():
    ctrl = LateralLqrController(**_DEFAULT_KW)
    assert ctrl.K.shape == (1, 2)


def test_zero_state_zero_control():
    ctrl = LateralLqrController(**_DEFAULT_KW)
    assert ctrl.step(e_y=0.0, theta_err=0.0) == pytest.approx(0.0)


def test_positive_e_y_gives_negative_delta_theta():
    """Робот съехал влево от линии (e_y > 0) → поворот направо (δθ < 0)."""
    ctrl = LateralLqrController(**_DEFAULT_KW)
    delta = ctrl.step(e_y=0.05, theta_err=0.0)
    assert delta < 0, f"ожидаем δθ<0 для e_y>0, получили {delta}"


def test_negative_e_y_gives_positive_delta_theta():
    """Робот съехал вправо от линии (e_y < 0) → поворот налево (δθ > 0)."""
    ctrl = LateralLqrController(**_DEFAULT_KW)
    delta = ctrl.step(e_y=-0.05, theta_err=0.0)
    assert delta > 0


def test_output_clipped_at_delta_max_positive():
    ctrl = LateralLqrController(**_DEFAULT_KW)
    # Большая отрицательная e_y → большая положительная δθ → клип к +delta_max.
    delta = ctrl.step(e_y=-10.0, theta_err=0.0)
    assert delta == pytest.approx(ctrl.delta_theta_max)


def test_output_clipped_at_delta_max_negative():
    ctrl = LateralLqrController(**_DEFAULT_KW)
    delta = ctrl.step(e_y=10.0, theta_err=0.0)
    assert delta == pytest.approx(-ctrl.delta_theta_max)


def test_v0_zero_rejected():
    kw = dict(_DEFAULT_KW, v0=0.0)
    with pytest.raises(ValueError, match="v0"):
        LateralLqrController(**kw)


def test_v0_below_min_rejected():
    kw = dict(_DEFAULT_KW, v0=1e-6)
    with pytest.raises(ValueError, match="v0"):
        LateralLqrController(**kw)


def test_tau_zero_rejected():
    kw = dict(_DEFAULT_KW, tau_inner=0.0)
    with pytest.raises(ValueError, match="tau_inner"):
        LateralLqrController(**kw)


def test_ts_zero_rejected():
    kw = dict(_DEFAULT_KW, Ts=0.0)
    with pytest.raises(ValueError, match="Ts"):
        LateralLqrController(**kw)


def test_negative_r_rejected():
    kw = dict(_DEFAULT_KW, R_diag=[0.0])
    with pytest.raises(ValueError, match="R_diag"):
        LateralLqrController(**kw)


def test_q_wrong_dim_rejected():
    kw = dict(_DEFAULT_KW, Q_diag=[1.0, 1.0, 1.0])
    with pytest.raises(ValueError, match="Q_diag"):
        LateralLqrController(**kw)


def test_delta_max_zero_rejected():
    kw = dict(_DEFAULT_KW, delta_theta_max=0.0)
    with pytest.raises(ValueError, match="delta_theta_max"):
        LateralLqrController(**kw)


def test_simulation_converges_from_lateral_disturbance():
    """ξ₀ = [0.10, 0]: 10 см off-line, прямо вперёд. Замкнутая подсистема должна
    сойтись к ≈0 за 10 с (≤1 см и убывание на порядок). Это валидирует, что
    K выбран как стабилизирующий с практически разумной скоростью."""
    ctrl = LateralLqrController(**_DEFAULT_KW)
    xi = np.array([0.10, 0.0])
    initial_norm = float(np.linalg.norm(xi))
    for _ in range(500):                  # 10 секунд при Ts=0.02
        delta = ctrl.step(e_y=xi[0], theta_err=xi[1])
        u = np.array([delta])
        xi = ctrl.Ad @ xi + ctrl.Bd @ u
    final_norm = float(np.linalg.norm(xi))
    assert final_norm < 0.01, f"|ξ| должно упасть <0.01 за 10 с, получили {final_norm:.4f}"
    assert final_norm < 0.10 * initial_norm, "ожидаем падение норм на порядок"


def test_step_works_with_python_floats():
    """e_y/theta_err не обязаны быть numpy — int/float тоже принимается."""
    ctrl = LateralLqrController(**_DEFAULT_KW)
    delta = ctrl.step(e_y=0.05, theta_err=0.01)
    assert isinstance(delta, float)


def test_repr_includes_v0_and_stability():
    ctrl = LateralLqrController(**_DEFAULT_KW)
    s = repr(ctrl)
    assert "v0=0.150" in s
    assert "stable=True" in s
