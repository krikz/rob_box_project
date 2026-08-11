"""Tests for `DialogueStateMachine` (P0.3)."""

from __future__ import annotations

import pytest

from rob_box_core.dialogue_state import (
    DialogueState,
    DialogueStateMachine,
    IllegalTransitionError,
)


def test_starts_in_idle():
    sm = DialogueStateMachine()
    assert sm.state == DialogueState.IDLE


def test_legal_transitions_from_idle():
    sm = DialogueStateMachine()
    assert sm.can_transition_to(DialogueState.LISTENING)
    assert sm.can_transition_to(DialogueState.SILENCED)
    assert not sm.can_transition_to(DialogueState.DIALOGUE)


def test_legal_transitions_from_listening():
    sm = DialogueStateMachine(initial=DialogueState.LISTENING)
    assert sm.can_transition_to(DialogueState.DIALOGUE)
    assert sm.can_transition_to(DialogueState.IDLE)
    assert sm.can_transition_to(DialogueState.SILENCED)


def test_legal_transitions_from_dialogue():
    sm = DialogueStateMachine(initial=DialogueState.DIALOGUE)
    assert sm.can_transition_to(DialogueState.IDLE)
    assert sm.can_transition_to(DialogueState.LISTENING)
    assert sm.can_transition_to(DialogueState.SILENCED)


def test_legal_transitions_from_silenced():
    sm = DialogueStateMachine(initial=DialogueState.SILENCED)
    assert sm.can_transition_to(DialogueState.IDLE)
    assert not sm.can_transition_to(DialogueState.LISTENING)
    assert not sm.can_transition_to(DialogueState.DIALOGUE)


def test_illegal_transition_raises():
    sm = DialogueStateMachine()
    with pytest.raises(IllegalTransitionError):
        sm.transition_to(DialogueState.DIALOGUE)


def test_legal_transition_succeeds_and_updates_state():
    sm = DialogueStateMachine()
    sm.transition_to(DialogueState.LISTENING)
    assert sm.state == DialogueState.LISTENING
    sm.transition_to(DialogueState.DIALOGUE)
    assert sm.state == DialogueState.DIALOGUE
    sm.transition_to(DialogueState.IDLE)
    assert sm.state == DialogueState.IDLE


def test_transition_to_non_state_raises_type_error():
    sm = DialogueStateMachine()
    with pytest.raises(TypeError):
        sm.transition_to("DIALOGUE")  # type: ignore[arg-type]


def test_silenced_only_returns_to_idle():
    sm = DialogueStateMachine(initial=DialogueState.SILENCED)
    sm.transition_to(DialogueState.IDLE)
    assert sm.state == DialogueState.IDLE


def test_history_records_every_transition():
    sm = DialogueStateMachine()
    sm.transition_to(DialogueState.LISTENING)
    sm.transition_to(DialogueState.DIALOGUE)
    sm.transition_to(DialogueState.IDLE)
    assert sm.history == [
        (DialogueState.IDLE, DialogueState.LISTENING),
        (DialogueState.LISTENING, DialogueState.DIALOGUE),
        (DialogueState.DIALOGUE, DialogueState.IDLE),
    ]


def test_hooks_fire_on_legal_transition():
    entered: list[DialogueState] = []
    exited: list[DialogueState] = []
    sm = DialogueStateMachine()
    sm.on_enter(DialogueState.LISTENING, lambda: entered.append(DialogueState.LISTENING))
    sm.on_exit(DialogueState.IDLE, lambda: exited.append(DialogueState.IDLE))
    sm.transition_to(DialogueState.LISTENING)
    assert entered == [DialogueState.LISTENING]
    assert exited == [DialogueState.IDLE]


def test_hooks_do_not_fire_on_illegal_transition():
    entered: list[DialogueState] = []
    sm = DialogueStateMachine()
    sm.on_enter(DialogueState.DIALOGUE, lambda: entered.append(DialogueState.DIALOGUE))
    with pytest.raises(IllegalTransitionError):
        sm.transition_to(DialogueState.DIALOGUE)
    assert entered == []  # hook not called


def test_illegal_transition_does_not_update_state():
    sm = DialogueStateMachine()
    with pytest.raises(IllegalTransitionError):
        sm.transition_to(DialogueState.DIALOGUE)
    assert sm.state == DialogueState.IDLE


def test_reset_returns_to_idle_from_any_state():
    for start in DialogueState:
        sm = DialogueStateMachine(initial=start)
        sm.reset()
        assert sm.state == DialogueState.IDLE


def test_legal_next_states_is_static_helper():
    assert DialogueStateMachine.legal_next_states(DialogueState.IDLE) == frozenset(
        {DialogueState.LISTENING, DialogueState.SILENCED}
    )


def test_dialogue_state_is_string_compatible():
    """DialogueState inherits str — values must round-trip through .value."""
    assert DialogueState.IDLE.value == "IDLE"
    assert DialogueState.IDLE == "IDLE"  # str mixin
    assert f"{DialogueState.DIALOGUE}" == "DialogueState.DIALOGUE"
