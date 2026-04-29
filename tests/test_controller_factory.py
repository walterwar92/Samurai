"""Tests for pi_nodes.control.controller_factory.

Verify that mode strings dispatch to the right controller and that
unknown / broken configs degrade gracefully to a passthrough.
"""

from __future__ import annotations

import importlib

import numpy as np
import pytest

scipy = pytest.importorskip("scipy")

from pi_nodes.control.controller_factory import (                  # noqa: E402
    PassthroughController,
    make_controller,
)


def test_off_mode_returns_passthrough():
    ctrl = make_controller("off")
    assert isinstance(ctrl, PassthroughController)


def test_passthrough_returns_zero_without_ref():
    ctrl = PassthroughController()
    u = ctrl.step(np.zeros(5))
    assert np.allclose(u, 0)


def test_passthrough_passes_reference_velocities():
    ctrl = PassthroughController()
    x_ref = np.array([0, 0, 0, 0.15, -0.5])
    u = ctrl.step(np.zeros(5), x_ref=x_ref)
    assert u[0] == pytest.approx(0.15)
    assert u[1] == pytest.approx(-0.5)


def test_unknown_mode_falls_back_to_passthrough():
    ctrl = make_controller("does-not-exist")
    assert isinstance(ctrl, PassthroughController)


def test_lqr_mode_constructs():
    """With config-driven scipy fallback, lqr mode should always succeed."""
    ctrl = make_controller("lqr")
    # not a passthrough (real regulator)
    assert not isinstance(ctrl, PassthroughController)
    u = ctrl.step(np.array([1.0, 0, 0, 0, 0]))
    assert u.shape == (2,)


def test_mpc_mode_constructs():
    ctrl = make_controller("mpc")
    assert not isinstance(ctrl, PassthroughController)
    u = ctrl.step(np.array([1.0, 0, 0, 0, 0]))
    assert u.shape == (2,)
