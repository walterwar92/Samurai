"""Unit tests for pi_nodes.control.imu_push_detector."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pi_nodes.control.imu_push_detector import IMUPushDetector, PushParams


def _params(**override):
    base = dict(
        speed=0.12,
        deviation_threshold=0.20,
        confirm_samples=2,
        max_move_ticks=30,
        bias_alpha_slow=0.02,
        bias_alpha_fast=0.15,
        fast_adapt_ticks=20,
        bias_init_alpha=0.1,
        bias_init_samples=40,
    )
    base.update(override)
    return PushParams(**base)


def test_motors_active_outputs_zero_and_resets():
    d = IMUPushDetector(_params())
    # Even with strong push, motors_idle=False → no output
    vx, vy = d.update(raw_lin_bx=1.0, raw_lin_by=0.0, theta=0.0,
                      motors_idle=False)
    assert vx == 0.0 and vy == 0.0
    assert d.dev_count == 0
    assert d.cont_ticks == 0
    assert d.active is False


def test_quiet_input_outputs_zero():
    d = IMUPushDetector(_params())
    for _ in range(50):
        vx, vy = d.update(0.0, 0.0, 0.0, motors_idle=True)
    assert vx == 0.0 and vy == 0.0


def test_sustained_push_triggers_after_confirm_samples():
    d = IMUPushDetector(_params(confirm_samples=2, bias_init_samples=0))
    # First sample over threshold — dev_count=1, not yet triggered
    v1 = d.update(0.5, 0.0, 0.0, motors_idle=True)
    assert v1 == (0.0, 0.0)
    # Second sample — dev_count=2, triggers
    vx, vy = d.update(0.5, 0.0, 0.0, motors_idle=True)
    assert vx == pytest.approx(0.12, abs=0.001)
    assert abs(vy) < 1e-6


def test_push_direction_locks_at_first_trigger():
    d = IMUPushDetector(_params(confirm_samples=2, bias_init_samples=0))
    d.update(0.5, 0.0, 0.0, motors_idle=True)
    d.update(0.5, 0.0, 0.0, motors_idle=True)
    # Now reverse the input — direction should be locked from the trigger
    vx, vy = d.update(-0.5, 0.5, 0.0, motors_idle=True)
    # Once active, magnitude is fixed at speed and direction stays locked
    assert abs(vx**2 + vy**2 - 0.12**2) < 1e-4
    assert vx > 0  # original locked direction was +x


def test_world_frame_rotation():
    """A body-frame +x push at theta=π/2 should produce world-frame +y velocity."""
    d = IMUPushDetector(_params(confirm_samples=1, bias_init_samples=0))
    vx, vy = d.update(0.5, 0.0, theta=math.pi / 2, motors_idle=True)
    # vx in world should be ~0, vy should be ~+0.12
    assert abs(vx) < 0.01
    assert vy == pytest.approx(0.12, abs=0.001)


def test_sustained_push_eventually_treated_as_drift():
    p = _params(confirm_samples=1, max_move_ticks=3, bias_init_samples=0)
    d = IMUPushDetector(p)
    seen_active = False
    seen_inactive_after_active = False
    for i in range(10):
        vx, vy = d.update(0.5, 0.0, 0.0, motors_idle=True)
        if d.active:
            seen_active = True
        elif seen_active:
            seen_inactive_after_active = True
            break
    assert seen_active, 'detector should have activated'
    assert seen_inactive_after_active, ('detector should have deactivated after '
                                        'max_move_ticks (drift recovery)')


def test_reset_clears_state():
    d = IMUPushDetector(_params(confirm_samples=1, bias_init_samples=0))
    d.update(0.5, 0.0, 0.0, motors_idle=True)
    d.update(0.5, 0.0, 0.0, motors_idle=True)
    assert d.active is True
    d.reset()
    assert d.active is False
    assert d.bias_x == 0.0
    assert d.dev_count == 0
