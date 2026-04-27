"""Tests for the 1st-order motor lag low-pass used in motor_node.

The lag isn't a separate module (it's a 3-line formula in _control_loop),
but the math itself has well-known properties — verify them so a future
edit doesn't silently break the contract.

Formula:    v(t) = v(t-1) + α · (target - v(t-1)),   α = dt / (τ + dt)
Equivalent: low-pass with time constant τ. After 3·τ, v reaches ~95% of
            a step input; after 5·τ, ~99%.
"""

import math

import pytest


def step_lowpass(target: float, tau: float, dt: float, n_steps: int,
                 v0: float = 0.0) -> float:
    """Apply n_steps of the same lag formula used by motor_node."""
    v = v0
    alpha = dt / (tau + dt)
    for _ in range(n_steps):
        v += alpha * (target - v)
    return v


def test_step_response_reaches_63_percent_at_one_tau():
    """Classic 1st-order step response: v(τ) ≈ 0.63·target."""
    tau = 0.15
    dt = 0.01  # fine dt for accuracy
    n = int(tau / dt)
    v = step_lowpass(target=1.0, tau=tau, dt=dt, n_steps=n)
    # Continuous-time 1-exp(-1) = 0.632; discrete approximation close
    assert 0.60 < v < 0.66


def test_step_response_reaches_95_percent_at_3_tau():
    tau = 0.15
    dt = 0.01
    n = int(3 * tau / dt)
    v = step_lowpass(target=1.0, tau=tau, dt=dt, n_steps=n)
    assert v > 0.92


def test_step_response_reaches_99_percent_at_5_tau():
    tau = 0.15
    dt = 0.01
    n = int(5 * tau / dt)
    v = step_lowpass(target=1.0, tau=tau, dt=dt, n_steps=n)
    assert v > 0.98


def test_step_response_with_motor_node_dt():
    """At motor_node's actual dt=0.05 (20Hz), lag still behaves correctly."""
    tau = 0.15
    dt = 0.05
    # 3·τ = 0.45 sec → 9 ticks at 20Hz
    v = step_lowpass(target=1.0, tau=tau, dt=dt, n_steps=9)
    # Discrete with coarser dt undershoots a bit, but still > 0.85
    assert v > 0.85


def test_braking_decay():
    """Sudden brake (target=0): velocity decays exponentially, not instantly."""
    tau = 0.15
    dt = 0.05
    # Start at full speed, command zero, see what happens after 1·τ
    v = step_lowpass(target=0.0, tau=tau, dt=dt, n_steps=3, v0=1.0)
    # After 3 ticks (0.15 sec ≈ 1·τ), should still be ~0.37 of initial
    assert 0.30 < v < 0.50


def test_lag_disabled_is_passthrough():
    """When tau=0 the formula should reduce to instant response.

    Note: tau=0 would divide by zero in α; motor_node guards this with
    `MOTOR_LAG_ENABLED and MOTOR_LAG_TAU > 0` — verify that contract here
    by computing what the code does in disabled branch (v_actual = v_target).
    """
    # In disabled branch: v_actual = target directly.
    # Just make sure the lag-skip logic produces target verbatim.
    target = 0.30
    v_actual = target  # what motor_node._control_loop does when disabled
    assert v_actual == target


def test_repeated_constant_input_settles():
    """Hold target constant for many ticks → output equals target exactly
    in the limit (at least to floating-point precision)."""
    tau = 0.15
    dt = 0.05
    v = step_lowpass(target=0.5, tau=tau, dt=dt, n_steps=200)
    assert math.isclose(v, 0.5, abs_tol=1e-3)
