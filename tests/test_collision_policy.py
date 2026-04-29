"""Unit tests for pi_nodes.control.collision_policy.

Verifies behaviour extracted from motor_node._control_loop matches the
original semantics. Run: pytest tests/test_collision_policy.py
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pi_nodes.control.collision_policy import CollisionPolicy


@pytest.fixture
def policy():
    return CollisionPolicy(stop_m=0.20, slow_m=0.40, avoid_angular=0.6)


def _apply(p, lin, ang=0.0, range_m=2.0, age=0.0, manual=False, t=0.0):
    return p.apply(lin, ang, range_m, age, manual, t)


def test_clear_path_no_change(policy):
    d = _apply(policy, lin=0.2, range_m=2.0)
    assert d.linear == pytest.approx(0.2)
    assert d.source is None


def test_reverse_motion_never_blocks(policy):
    d = _apply(policy, lin=-0.2, range_m=0.05)
    assert d.linear == pytest.approx(-0.2)
    assert d.source is None


def test_zero_motion_no_override(policy):
    d = _apply(policy, lin=0.0, range_m=0.05)
    assert d.linear == 0.0
    assert d.source is None


def test_slow_zone_scales_linearly(policy):
    # range = 0.30 is exactly halfway between stop=0.2 and slow=0.4
    d = _apply(policy, lin=0.2, range_m=0.30)
    assert d.linear == pytest.approx(0.1, abs=0.001)
    assert d.source == 'collision'


def test_stop_zone_zeros_linear(policy):
    d = _apply(policy, lin=0.2, range_m=0.10)
    assert d.linear == 0.0
    assert d.source == 'collision'


def test_stop_zone_steers_when_autonomous(policy):
    d = _apply(policy, lin=0.2, range_m=0.10, manual=False)
    assert d.angular == pytest.approx(0.6)  # avoid_angular * dir(1.0)
    assert d.avoidance_active is True


def test_stop_zone_does_not_steer_in_manual(policy):
    d = _apply(policy, lin=0.2, ang=0.0, range_m=0.10, manual=True)
    assert d.angular == 0.0    # caller's angular preserved
    # source still 'collision'? In manual mode, the original code only sets
    # source='collision' when not manual_active — so None here.
    assert d.source is None


def test_avoid_dir_alternates_after_clearance(policy):
    # Trigger stop zone twice with a clear pass between
    _apply(policy, lin=0.2, range_m=0.10)
    assert policy.avoid_dir == 1.0
    _apply(policy, lin=0.2, range_m=2.0)   # clear
    assert policy.avoid_active is False
    assert policy.avoid_dir == -1.0        # flipped on clearance
    d = _apply(policy, lin=0.2, range_m=0.10)
    assert d.angular == pytest.approx(-0.6)


def test_stale_data_skips_guard(policy):
    # Sensor stale (age >= 1.0) — guard should NOT block movement
    d = _apply(policy, lin=0.2, range_m=0.05, age=2.0)
    assert d.linear == pytest.approx(0.2)
    assert d.source is None


def test_negative_age_treated_as_stale(policy):
    # age=-1 means "no valid reading yet" — guard should skip
    d = _apply(policy, lin=0.2, range_m=0.05, age=-1.0)
    assert d.linear == pytest.approx(0.2)


def test_stale_warning_throttled():
    # Two stale calls within warn_period should warn only once.
    p = CollisionPolicy(warn_period_s=5.0)
    warnings = []
    for t in (0.0, 1.0, 2.0, 3.0):
        p.apply(0.2, 0.0, 0.05, 2.0, False, t, log_warn=warnings.append)
    assert len(warnings) == 1   # only the first triggers a warning


def test_stale_warning_fires_after_period():
    p = CollisionPolicy(warn_period_s=5.0)
    warnings = []
    p.apply(0.2, 0.0, 0.05, 2.0, False, 0.0, log_warn=warnings.append)
    p.apply(0.2, 0.0, 0.05, 2.0, False, 6.0, log_warn=warnings.append)
    assert len(warnings) == 2


def test_stop_m_must_be_less_than_slow_m():
    with pytest.raises(ValueError):
        CollisionPolicy(stop_m=0.5, slow_m=0.3)
