"""
SimFSM state enumeration (#44 phase 7a).

Extracted from compute_node/simulator.py — the lightweight half of the
SimFSM split. The full SimFSM class (~570 LoC) stays in simulator.py
for now; only the state-name constants are pulled out here so they
can be imported by future helper extractions and unit tests without
loading the whole Flask + sim stack.

`State(str, Enum)` keeps backward compatibility for code that compares
against the bare string constant: `state == State.IDLE` and
`state == 'IDLE'` are both true. The legacy class-of-string-constants
form (`class State: IDLE = 'IDLE'`) was used because the old simulator
was written before py3.4 enums were widely adopted; the str-Enum form
is the modern equivalent and gives free isinstance() checks.
"""
from __future__ import annotations

from enum import Enum


class State(str, Enum):
    """High-level behavioural state of the simulated robot's FSM.

    Mirrors the FSM defined in pi_nodes/nodes/fsm_node.py — keep these
    symbols aligned across both files until the simulator FSM is
    rewritten on top of the BehaviourTree (which is the planned
    direction; see backlog #1 transformation).
    """
    IDLE = 'IDLE'
    SEARCHING = 'SEARCHING'
    TARGETING = 'TARGETING'
    APPROACHING = 'APPROACHING'
    GRABBING = 'GRABBING'
    CALLING = 'CALLING'
    RETURNING = 'RETURNING'

    def __str__(self) -> str:
        # Preserve the legacy string representation so code paths that
        # do `f'{state}'` or `str(state)` continue to print 'IDLE' etc.,
        # not the verbose 'State.IDLE'.
        return self.value


# Convenience: a tuple of all state names, useful for UI dropdowns and
# transition validators that need to enumerate possibilities.
ALL_STATES: tuple[State, ...] = tuple(State)
