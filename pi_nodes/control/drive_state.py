"""
DriveState — enum + transition-validating state tracker for precision_drive_node.

Why not python-statemachine: precision_drive is 800 lines of safety-critical
control code with many implicit transitions. A full migration to a third-party
state machine library would mean a major rewrite with regression risk; the
visible benefit (auto-logging, illegal-transition catches) can be achieved
with a thin validator that wraps a string field.

Usage::

    self._fsm = StateTracker(initial=DriveState.IDLE, log=self.log_warn,
                             log_info=self.log_info)
    self._fsm.transition_to(DriveState.ALIGNING, reason='start scenario')
    if self._fsm.is_in(DriveState.IDLE, DriveState.DONE):
        ...
    self._state = self._fsm.current  # string for backward compat
"""

from enum import Enum
from typing import Callable, Optional


class DriveState(str, Enum):
    IDLE = 'idle'
    ALIGNING = 'aligning'
    DRIVING = 'driving'
    TURNING = 'turning'
    SETTLING = 'settling'
    PAUSED_USER = 'paused_user'
    PAUSED_LIFT = 'paused_lift'
    DONE = 'done'

    def __str__(self):
        return self.value


# Allowed transitions. Anything not listed here is logged as illegal but not
# blocked — original code did `self._state = 'X'` everywhere with no checks,
# so blocking would be a behavioural change. Logging surfaces the bug; if a
# logged "illegal" transition turns out to be needed, add it here.
ALLOWED: dict[DriveState, set[DriveState]] = {
    DriveState.IDLE: {
        DriveState.ALIGNING, DriveState.DRIVING, DriveState.TURNING,
        DriveState.SETTLING, DriveState.DONE,
    },
    DriveState.ALIGNING: {
        DriveState.DRIVING, DriveState.SETTLING, DriveState.IDLE,
        DriveState.PAUSED_USER, DriveState.PAUSED_LIFT,
    },
    DriveState.DRIVING: {
        DriveState.SETTLING, DriveState.IDLE, DriveState.DONE,
        DriveState.PAUSED_USER, DriveState.PAUSED_LIFT,
    },
    DriveState.TURNING: {
        DriveState.SETTLING, DriveState.IDLE, DriveState.DONE,
        DriveState.PAUSED_USER, DriveState.PAUSED_LIFT,
    },
    DriveState.SETTLING: {
        DriveState.IDLE, DriveState.DRIVING, DriveState.TURNING,
        DriveState.ALIGNING, DriveState.DONE,
        DriveState.PAUSED_USER, DriveState.PAUSED_LIFT,
    },
    DriveState.PAUSED_USER: {
        DriveState.IDLE, DriveState.ALIGNING, DriveState.DRIVING,
        DriveState.TURNING, DriveState.SETTLING,
    },
    DriveState.PAUSED_LIFT: {
        DriveState.IDLE, DriveState.SETTLING, DriveState.ALIGNING,
        DriveState.DRIVING, DriveState.TURNING,
    },
    DriveState.DONE: {
        DriveState.IDLE,
    },
}


class StateTracker:
    """Validating state container. Logs every transition and flags illegal ones."""

    def __init__(self, initial: DriveState = DriveState.IDLE,
                 log_info: Optional[Callable[[str, ...], None]] = None,
                 log_warn: Optional[Callable[[str, ...], None]] = None,
                 on_transition: Optional[Callable[[DriveState, DriveState, str], None]] = None):
        self._state = initial
        self._prev = initial
        self._log_info = log_info
        self._log_warn = log_warn
        self._on_transition = on_transition
        self.illegal_count = 0
        self.transition_count = 0

    @property
    def current(self) -> str:
        return self._state.value

    @property
    def state(self) -> DriveState:
        return self._state

    @property
    def previous(self) -> DriveState:
        return self._prev

    def is_in(self, *states) -> bool:
        # Accept either DriveState enum members or their string values for
        # ergonomic call sites that haven't fully migrated.
        cur = self._state.value
        for s in states:
            if isinstance(s, DriveState):
                if self._state is s:
                    return True
            elif cur == s:
                return True
        return False

    def transition_to(self, new_state: DriveState, reason: str = '',
                      force: bool = False) -> bool:
        """Perform a transition with validation. Returns True on legal transition.

        force=True bypasses the legality check (still logs). Use for emergency
        transitions like abort() that are intentionally allowed from any state.
        """
        if not isinstance(new_state, DriveState):
            new_state = DriveState(new_state)

        if new_state is self._state:
            return True   # no-op, common in control loops

        legal = new_state in ALLOWED.get(self._state, set())
        suffix = f' ({reason})' if reason else ''

        if not legal and not force:
            self.illegal_count += 1
            if self._log_warn:
                self._log_warn('FSM ILLEGAL transition %s -> %s%s',
                               self._state.value, new_state.value, suffix)
            # Allow it anyway — old code did, blocking would be a behavioural
            # change. But the warning surfaces the bug for inspection.

        if self._log_info:
            tag = 'forced ' if force and not legal else ''
            self._log_info('FSM: %s%s -> %s%s',
                           tag, self._state.value, new_state.value, suffix)

        if self._on_transition:
            try:
                self._on_transition(self._state, new_state, reason)
            except Exception:
                pass   # never let a hook break the FSM

        self._prev = self._state
        self._state = new_state
        self.transition_count += 1
        return legal

    def restore_previous(self) -> None:
        """Swap to the previously-stored state (e.g. paused_user → resume)."""
        target = self._prev
        self.transition_to(target, reason='restore previous', force=True)
