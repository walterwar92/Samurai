"""Unit tests for compute_node.sim_fsm_states (#44 phase 7a)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from compute_node.sim_fsm_states import State, ALL_STATES


def test_all_seven_states_present():
    """Locks the FSM contract — adding/removing a state requires deliberate
    test update so the simulator's behaviour set can't drift silently."""
    expected = {'IDLE', 'SEARCHING', 'TARGETING', 'APPROACHING',
                'GRABBING', 'CALLING', 'RETURNING'}
    actual = {s.value for s in ALL_STATES}
    assert actual == expected


def test_state_string_equality():
    """str-Enum: comparison to bare string still passes."""
    assert State.IDLE == 'IDLE'
    assert State.SEARCHING == 'SEARCHING'
    assert State.RETURNING == 'RETURNING'


def test_str_repr_returns_value_not_enum_name():
    """f-strings and str() should print 'IDLE', not 'State.IDLE'."""
    assert str(State.IDLE) == 'IDLE'
    assert f'{State.SEARCHING}' == 'SEARCHING'


def test_isinstance_str_works():
    """Because State inherits from str, instances are real strings."""
    assert isinstance(State.IDLE, str)
    assert State.IDLE.upper() == 'IDLE'


def test_lookup_by_value():
    """State('IDLE') must find the enum member."""
    assert State('IDLE') is State.IDLE
    assert State('CALLING') is State.CALLING


def test_invalid_value_raises():
    import pytest
    with pytest.raises(ValueError):
        State('UNKNOWN_STATE')


def test_all_states_tuple_is_immutable():
    """ALL_STATES is a tuple, not a list — callers can't mutate it
    accidentally."""
    assert isinstance(ALL_STATES, tuple)
    assert len(ALL_STATES) == 7


def test_simulator_reexport():
    """Backward compat: importing State from simulator must yield the
    same enum members."""
    from compute_node import simulator
    assert simulator.State is State
    assert simulator.State.IDLE is State.IDLE
