"""Unit tests for pi_nodes.control.drive_state."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pi_nodes.control.drive_state import DriveState, StateTracker, ALLOWED


def test_initial_state():
    t = StateTracker(initial=DriveState.IDLE)
    assert t.state is DriveState.IDLE
    assert t.current == 'idle'


def test_legal_transition():
    warns, infos = [], []
    t = StateTracker(initial=DriveState.IDLE,
                     log_info=lambda *a: infos.append(a),
                     log_warn=lambda *a: warns.append(a))
    ok = t.transition_to(DriveState.ALIGNING, reason='start')
    assert ok is True
    assert t.state is DriveState.ALIGNING
    assert t.previous is DriveState.IDLE
    assert len(infos) == 1
    assert len(warns) == 0


def test_illegal_transition_logged_but_allowed():
    warns, infos = [], []
    t = StateTracker(initial=DriveState.DONE,
                     log_info=lambda *a: infos.append(a),
                     log_warn=lambda *a: warns.append(a))
    # done → driving is not in ALLOWED but should still execute (log only)
    ok = t.transition_to(DriveState.DRIVING)
    assert ok is False
    assert t.state is DriveState.DRIVING
    assert t.illegal_count == 1
    assert len(warns) == 1


def test_force_bypasses_validation():
    warns = []
    t = StateTracker(initial=DriveState.DONE,
                     log_warn=lambda *a: warns.append(a))
    ok = t.transition_to(DriveState.DRIVING, force=True)
    # Forced transitions don't increment illegal_count even when illegal
    assert t.state is DriveState.DRIVING
    assert t.illegal_count == 0


def test_noop_on_same_state():
    infos = []
    t = StateTracker(initial=DriveState.IDLE,
                     log_info=lambda *a: infos.append(a))
    t.transition_to(DriveState.IDLE)
    assert len(infos) == 0   # no-op transitions don't log


def test_restore_previous():
    t = StateTracker(initial=DriveState.DRIVING)
    t.transition_to(DriveState.PAUSED_USER, reason='user pause')
    assert t.state is DriveState.PAUSED_USER
    t.restore_previous()
    assert t.state is DriveState.DRIVING


def test_is_in_accepts_enum_and_str():
    t = StateTracker(initial=DriveState.DRIVING)
    assert t.is_in(DriveState.DRIVING)
    assert t.is_in('driving')
    assert t.is_in(DriveState.IDLE, DriveState.DRIVING)
    assert not t.is_in(DriveState.IDLE, DriveState.ALIGNING)


def test_on_transition_hook():
    seen = []
    t = StateTracker(initial=DriveState.IDLE,
                     on_transition=lambda old, new, reason: seen.append((old, new, reason)))
    t.transition_to(DriveState.DRIVING, reason='go')
    assert seen == [(DriveState.IDLE, DriveState.DRIVING, 'go')]


def test_on_transition_hook_does_not_break_fsm():
    def bad_hook(*_a):
        raise RuntimeError('hook broke')
    t = StateTracker(initial=DriveState.IDLE, on_transition=bad_hook)
    t.transition_to(DriveState.DRIVING)
    assert t.state is DriveState.DRIVING


def test_string_value_compatibility():
    """Existing code reads self._state as a string — DriveState extends str."""
    t = StateTracker(initial=DriveState.DRIVING)
    assert t.current == 'driving'
    # And DriveState members are == their string values (str enum)
    assert DriveState.DRIVING == 'driving'


def test_all_states_have_outgoing_transitions():
    for s in DriveState:
        assert s in ALLOWED, f'{s} has no outgoing transitions defined'
