"""Tests for the MPS extensions of pi_nodes.control.state_space_model.

Covers Cd, Dd, output(), reload(). The base discretisation tests live in
tests/test_state_space_model.py — here we only exercise the new surface
introduced for `feat/mps`.
"""
from __future__ import annotations

import numpy as np
import pytest

scipy = pytest.importorskip("scipy")  # state_space_model uses scipy.linalg.expm

from pi_nodes.control.state_space_model import StateSpaceModel  # noqa: E402


def _stable_5x5() -> tuple[np.ndarray, np.ndarray]:
    """A diagonal stable Ad (|λ|<1) and a non-trivial Bd of shape (5, 2)."""
    Ad = np.diag([0.9, 0.85, 0.95, 0.7, 0.6])
    Bd = np.array(
        [
            [0.01, 0.0],
            [0.0, 0.0],
            [0.0, 0.01],
            [0.28, 0.0],
            [0.0, 0.39],
        ]
    )
    return Ad, Bd


# ── Defaults ───────────────────────────────────────────────────────────
def test_default_cd_is_identity_dd_is_zero():
    Ad, Bd = _stable_5x5()
    m = StateSpaceModel(Ad=Ad, Bd=Bd, Ts=0.05)
    assert m.k == 5
    assert np.allclose(m.Cd, np.eye(5))
    assert np.allclose(m.Dd, np.zeros((5, 2)))


def test_output_default_equals_state():
    Ad, Bd = _stable_5x5()
    m = StateSpaceModel(Ad=Ad, Bd=Bd, Ts=0.05)
    x = np.array([1.0, -2.0, 0.5, 3.0, -0.7])
    u = np.array([0.1, -0.2])
    # default Cd = I, Dd = 0  ⇒  y == x for ANY u
    np.testing.assert_allclose(m.output(x, u), x)


def test_output_with_explicit_cd_dd():
    Ad, Bd = _stable_5x5()
    Cd = np.array([[1, 0, 0, 0, 0], [0, 0, 0, 1, 0]], dtype=float)  # (2, 5)
    Dd = np.array([[0.0, 0.5], [0.1, 0.0]])
    m = StateSpaceModel(Ad=Ad, Bd=Bd, Cd=Cd, Dd=Dd, Ts=0.05)
    assert m.k == 2
    x = np.array([2.0, 0.0, 0.0, 0.5, 0.0])
    u = np.array([1.0, 1.0])
    y = m.output(x, u)
    # y[0] = 2.0 + 0.5*1 = 2.5;  y[1] = 0.5 + 0.1*1 = 0.6
    np.testing.assert_allclose(y, [2.5, 0.6])


def test_output_no_u_drops_dd_term():
    Ad, Bd = _stable_5x5()
    Cd = np.eye(5)
    Dd = np.full((5, 2), 99.0)  # would explode if used
    m = StateSpaceModel(Ad=Ad, Bd=Bd, Cd=Cd, Dd=Dd, Ts=0.05)
    x = np.arange(5, dtype=float)
    np.testing.assert_allclose(m.output(x), x)


def test_output_default_dd_when_only_cd_given():
    Ad, Bd = _stable_5x5()
    Cd = np.eye(5) * 2.0
    m = StateSpaceModel(Ad=Ad, Bd=Bd, Cd=Cd, Ts=0.05)
    assert m.Dd.shape == (5, 2)
    np.testing.assert_allclose(m.Dd, 0.0)


# ── Constructor validation ─────────────────────────────────────────────
def test_invalid_cd_cols_raises():
    Ad, Bd = _stable_5x5()
    Cd_bad = np.eye(4)  # 4 cols ≠ n=5
    with pytest.raises(ValueError, match="Cd shape"):
        StateSpaceModel(Ad=Ad, Bd=Bd, Cd=Cd_bad, Ts=0.05)


def test_invalid_dd_shape_raises():
    Ad, Bd = _stable_5x5()
    Dd_bad = np.zeros((5, 3))  # cols=3 ≠ r=2
    with pytest.raises(ValueError, match="Dd shape"):
        StateSpaceModel(Ad=Ad, Bd=Bd, Dd=Dd_bad, Ts=0.05)


# ── reload() ───────────────────────────────────────────────────────────
def test_reload_swaps_ad_atomically():
    Ad, Bd = _stable_5x5()
    m = StateSpaceModel(Ad=Ad, Bd=Bd, Ts=0.05)
    Ad_new = Ad * 0.5
    m.reload(Ad=Ad_new)
    np.testing.assert_allclose(m.Ad, Ad_new)
    # Bd untouched
    np.testing.assert_allclose(m.Bd, Bd)


def test_reload_keeps_cd_dd_when_n_unchanged():
    Ad, Bd = _stable_5x5()
    Cd = np.eye(5) * 3.0
    m = StateSpaceModel(Ad=Ad, Bd=Bd, Cd=Cd, Ts=0.05)
    m.reload(Ad=Ad * 0.9)
    np.testing.assert_allclose(m.Cd, Cd)


def test_reload_invalid_shape_leaves_model_untouched():
    Ad, Bd = _stable_5x5()
    m = StateSpaceModel(Ad=Ad, Bd=Bd, Ts=0.05)
    Ad_orig = m.Ad.copy()
    Bd_orig = m.Bd.copy()
    Ad_bad = np.eye(4)  # 4×4 incompatible with Bd 5×2
    with pytest.raises(ValueError, match="reload"):
        m.reload(Ad=Ad_bad)
    # State preserved
    np.testing.assert_allclose(m.Ad, Ad_orig)
    np.testing.assert_allclose(m.Bd, Bd_orig)


def test_reload_swaps_cd_dd_together():
    Ad, Bd = _stable_5x5()
    m = StateSpaceModel(Ad=Ad, Bd=Bd, Ts=0.05)
    Cd_new = np.array([[1, 0, 0, 0, 0]], dtype=float)  # (1, 5)
    Dd_new = np.array([[0.5, 0.0]])                    # (1, 2)
    m.reload(Cd=Cd_new, Dd=Dd_new)
    assert m.k == 1
    np.testing.assert_allclose(m.Cd, Cd_new)
    np.testing.assert_allclose(m.Dd, Dd_new)


def test_reload_mismatched_cd_dd_rows_rejected():
    Ad, Bd = _stable_5x5()
    m = StateSpaceModel(Ad=Ad, Bd=Bd, Ts=0.05)
    Cd_new = np.eye(5)[:2]                  # (2, 5)
    Dd_new = np.zeros((3, 2))               # rows mismatch
    with pytest.raises(ValueError, match="Dd"):
        m.reload(Cd=Cd_new, Dd=Dd_new)


def test_reload_unstable_eigvals_then_is_stable_false():
    Ad, Bd = _stable_5x5()
    m = StateSpaceModel(Ad=Ad, Bd=Bd, Ts=0.05)
    assert m.is_stable() is True
    # |λ|=1.5 ⇒ unstable
    m.reload(Ad=np.diag([1.5, 1.5, 1.5, 1.5, 1.5]))
    assert m.is_stable() is False
