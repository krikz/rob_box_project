"""DialogueStateMachine — formal transition wrapper around the existing
``DialogueManager``.

The point of this module (per ADR-0001 / P0.3 of the refactoring plan) is to
make the state-machine contract explicit and verifiable:

  * every state has an enumerated set of legal next states,
  * any illegal transition raises instead of silently mutating,
  * consumers can introspect the transition graph for documentation / tests.

We deliberately do NOT touch the existing ``DialogueManager`` (which already
ships with the live dialogue node). ``DialogueStateMachine`` is a thin,
additive facade — once we migrate callers in P1, ``DialogueManager`` itself
will be re-implemented on top of this class.
"""

from __future__ import annotations

from enum import Enum
from typing import Callable, Mapping

# ---------------------------------------------------------------------------
# Reuse the existing state enum so values stay byte-identical with what
# DialogueManager already writes on the wire (P0: no behavioural change).
# ---------------------------------------------------------------------------


class DialogueState(str, Enum):
    """Dialogue state machine states.

    Inherits ``str`` so the value is still usable wherever DialogueManager
    expects a string (e.g. ROS message fields).
    """

    IDLE = "IDLE"
    LISTENING = "LISTENING"
    DIALOGUE = "DIALOGUE"
    SILENCED = "SILENCED"


# ---------------------------------------------------------------------------
# Transition table — single source of truth.
# ---------------------------------------------------------------------------


_VALID_TRANSITIONS: Mapping[DialogueState, frozenset[DialogueState]] = {
    DialogueState.IDLE: frozenset({DialogueState.LISTENING, DialogueState.SILENCED}),
    DialogueState.LISTENING: frozenset({DialogueState.DIALOGUE, DialogueState.IDLE, DialogueState.SILENCED}),
    DialogueState.DIALOGUE: frozenset({DialogueState.IDLE, DialogueState.LISTENING, DialogueState.SILENCED}),
    DialogueState.SILENCED: frozenset({DialogueState.IDLE}),
}


class IllegalTransitionError(RuntimeError):
    """Raised when a caller tries to move between two states that aren't
    connected in ``_VALID_TRANSITIONS``."""


class DialogueStateMachine:
    """Pure-Python state machine with explicit transition rules.

    Usage::

        sm = DialogueStateMachine()
        sm.transition_to(DialogueState.LISTENING)
        assert sm.state == DialogueState.LISTENING
        sm.transition_to(DialogueState.IDLE)  # legal

        sm.transition_to(DialogueState.DIALOGUE)  # from IDLE: not legal → raises
    """

    def __init__(self, *, initial: DialogueState = DialogueState.IDLE, clock: Callable[[], float] | None = None) -> None:
        self._state: DialogueState = initial
        # clock is optional — exists so the state machine can grow time-based
        # exits in P1 without forcing the constructor signature to change later.
        self._clock = clock or (lambda: 0.0)
        self._on_enter: dict[DialogueState, Callable[[], None]] = {}
        self._on_exit: dict[DialogueState, Callable[[], None]] = {}
        self.history: list[tuple[DialogueState, DialogueState]] = []  # (from, to)

    # -- introspection ------------------------------------------------------

    @property
    def state(self) -> DialogueState:
        return self._state

    @staticmethod
    def legal_next_states(current: DialogueState) -> frozenset[DialogueState]:
        """Return the set of states ``current`` may transition to."""
        return _VALID_TRANSITIONS[current]

    def can_transition_to(self, target: DialogueState) -> bool:
        return target in _VALID_TRANSITIONS[self._state]

    # -- mutation -----------------------------------------------------------

    def transition_to(self, target: DialogueState) -> None:
        """Move to ``target``. Raises ``IllegalTransitionError`` on illegal moves.

        Hooks (``on_exit`` / ``on_enter``) run only when the transition is
        legal — failures inside a hook propagate, the state stays put.
        """
        if not isinstance(target, DialogueState):
            raise TypeError(f"target must be DialogueState, got {type(target).__name__}")
        if not self.can_transition_to(target):
            raise IllegalTransitionError(
                f"illegal transition {self._state.value} -> {target.value}; "
                f"legal targets: {sorted(s.value for s in _VALID_TRANSITIONS[self._state])}"
            )
        previous = self._state
        exit_hook = self._on_exit.get(previous)
        enter_hook = self._on_enter.get(target)
        if exit_hook is not None:
            exit_hook()
        self._state = target
        if enter_hook is not None:
            enter_hook()
        self.history.append((previous, target))

    def reset(self) -> None:
        """Drop back to IDLE regardless of current state. Useful for tests."""
        previous = self._state
        self._state = DialogueState.IDLE
        self.history.append((previous, DialogueState.IDLE))

    # -- hooks --------------------------------------------------------------

    def on_enter(self, state: DialogueState, callback: Callable[[], None]) -> None:
        self._on_enter[state] = callback

    def on_exit(self, state: DialogueState, callback: Callable[[], None]) -> None:
        self._on_exit[state] = callback


__all__ = [
    "DialogueState",
    "DialogueStateMachine",
    "IllegalTransitionError",
]
