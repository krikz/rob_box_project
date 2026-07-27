"""Unit tests for :class:`DialogueStateMachine` — pure state machine for dialog lifecycle.

Tests the full state transition table, input classification, silence timeout,
and lifecycle. All tests are synchronous (no asyncio) because the DSM has no I/O.
"""

from __future__ import annotations

import time

import pytest

from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
    DialogState,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_dsm(
    initial: DialogueStateKind = DialogueStateKind.IDLE,
    silence_timeout: float = 300.0,
) -> DialogueStateMachine:
    return DialogueStateMachine(initial=initial, silence_timeout=silence_timeout)


def _make_state(**kwargs: object) -> DialogState:
    defaults: dict[str, object] = {
        "wake_active": True,
        "silenced": False,
        "silenced_until": 0.0,
        "dialogue_id": "test-001",
        "user_id": "test_user",
        "last_stt_text": "",
        "last_response": "",
        "turn_count": 0,
    }
    defaults.update(kwargs)
    return DialogState(**{k: v for k, v in defaults.items() if k in DialogState.__dataclass_fields__})  # type: ignore[arg-type]


class TestInitialState:
    """Verify the state machine starts in the expected state."""

    def test_default_initial_is_idle(self) -> None:
        dsm = _make_dsm()
        assert dsm.state == DialogueStateKind.IDLE

    def test_custom_initial_state(self) -> None:
        dsm = _make_dsm(initial=DialogueStateKind.LISTENING)
        assert dsm.state == DialogueStateKind.LISTENING

    def test_silence_timeout_property(self) -> None:
        dsm = _make_dsm(silence_timeout=120.0)
        assert dsm.silence_timeout == 120.0


class TestIdleTransitions:
    """IDLE state transitions."""

    def test_idle_wake_word_to_listening(self) -> None:
        dsm = _make_dsm()
        new_state = dsm.on_event(DialogueEvent.WAKE_WORD)
        assert new_state == DialogueStateKind.LISTENING
        assert dsm.state == DialogueStateKind.LISTENING

    def test_idle_stt_result_noop(self) -> None:
        dsm = _make_dsm()
        new_state = dsm.on_event(DialogueEvent.STT_RESULT)
        assert new_state == DialogueStateKind.IDLE
        assert dsm.state == DialogueStateKind.IDLE

    def test_idle_dialogue_end_noop(self) -> None:
        dsm = _make_dsm()
        new_state = dsm.on_event(DialogueEvent.DIALOGUE_END)
        assert new_state == DialogueStateKind.IDLE

    def test_idle_timeout_noop(self) -> None:
        dsm = _make_dsm()
        new_state = dsm.on_event(DialogueEvent.TIMEOUT)
        assert new_state == DialogueStateKind.IDLE

    def test_idle_silence_command_to_silenced(self) -> None:
        dsm = _make_dsm()
        new_state = dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        assert new_state == DialogueStateKind.SILENCED
        assert dsm.state == DialogueStateKind.SILENCED


class TestListeningTransitions:
    """LISTENING state transitions."""

    def test_listening_stt_result_to_dialogue(self) -> None:
        dsm = _make_dsm()
        dsm.on_event(DialogueEvent.WAKE_WORD)  # IDLE → LISTENING
        assert dsm.state == DialogueStateKind.LISTENING
        new_state = dsm.on_event(DialogueEvent.STT_RESULT)
        assert new_state == DialogueStateKind.DIALOGUE
        assert dsm.state == DialogueStateKind.DIALOGUE

    def test_listening_timeout_to_idle(self) -> None:
        dsm = _make_dsm()
        dsm.on_event(DialogueEvent.WAKE_WORD)  # IDLE → LISTENING
        new_state = dsm.on_event(DialogueEvent.TIMEOUT)
        assert new_state == DialogueStateKind.IDLE

    def test_listening_wake_word_noop(self) -> None:
        dsm = _make_dsm()
        dsm.on_event(DialogueEvent.WAKE_WORD)  # IDLE → LISTENING
        new_state = dsm.on_event(DialogueEvent.WAKE_WORD)
        assert new_state == DialogueStateKind.LISTENING  # stays

    def test_listening_silence_command_to_silenced(self) -> None:
        dsm = _make_dsm()
        dsm.on_event(DialogueEvent.WAKE_WORD)  # IDLE → LISTENING
        new_state = dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        assert new_state == DialogueStateKind.SILENCED


class TestDialogueTransitions:
    """DIALOGUE state transitions."""

    def test_dialogue_end_to_idle(self) -> None:
        dsm = _make_dsm()
        dsm.on_event(DialogueEvent.WAKE_WORD)  # IDLE → LISTENING
        dsm.on_event(DialogueEvent.STT_RESULT)  # LISTENING → DIALOGUE
        assert dsm.state == DialogueStateKind.DIALOGUE
        new_state = dsm.on_event(DialogueEvent.DIALOGUE_END)
        assert new_state == DialogueStateKind.IDLE

    def test_dialogue_wake_word_barge_in(self) -> None:
        """Barge-in: wake word during dialogue → restart listening."""
        dsm = _make_dsm()
        dsm.on_event(DialogueEvent.WAKE_WORD)  # IDLE → LISTENING
        dsm.on_event(DialogueEvent.STT_RESULT)  # LISTENING → DIALOGUE
        new_state = dsm.on_event(DialogueEvent.WAKE_WORD)
        assert new_state == DialogueStateKind.LISTENING

    def test_dialogue_stt_result_noop(self) -> None:
        dsm = _make_dsm()
        dsm.on_event(DialogueEvent.WAKE_WORD)  # IDLE → LISTENING
        dsm.on_event(DialogueEvent.STT_RESULT)  # LISTENING → DIALOGUE
        new_state = dsm.on_event(DialogueEvent.STT_RESULT)
        assert new_state == DialogueStateKind.DIALOGUE  # stays

    def test_dialogue_silence_command_to_silenced(self) -> None:
        dsm = _make_dsm()
        dsm.on_event(DialogueEvent.WAKE_WORD)  # IDLE → LISTENING
        dsm.on_event(DialogueEvent.STT_RESULT)  # LISTENING → DIALOGUE
        new_state = dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        assert new_state == DialogueStateKind.SILENCED


class TestSilencedTransitions:
    """SILENCED state transitions and timeout."""

    def test_silenced_timeout_to_idle(self) -> None:
        dsm = _make_dsm(silence_timeout=0.01)
        dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        assert dsm.state == DialogueStateKind.SILENCED
        time.sleep(0.02)  # exceed timeout
        timeout_event = dsm.check_silence_timeout()
        assert timeout_event == DialogueEvent.TIMEOUT
        dsm.on_event(timeout_event)
        assert dsm.state == DialogueStateKind.IDLE

    def test_silenced_unsilence_to_idle(self) -> None:
        dsm = _make_dsm()
        dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        assert dsm.state == DialogueStateKind.SILENCED
        new_state = dsm.on_event(DialogueEvent.UNSILENCE)
        assert new_state == DialogueStateKind.IDLE

    def test_silenced_ignores_other_events(self) -> None:
        dsm = _make_dsm()
        dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        assert dsm.state == DialogueStateKind.SILENCED
        # Wake word, STT result, dialogue end — all ignored while silenced
        for event in (DialogueEvent.WAKE_WORD, DialogueEvent.STT_RESULT, DialogueEvent.DIALOGUE_END):
            new_state = dsm.on_event(event)
            assert new_state == DialogueStateKind.SILENCED

    def test_check_silence_timeout_when_not_silenced(self) -> None:
        dsm = _make_dsm()
        assert dsm.check_silence_timeout() is None

    def test_check_silence_timeout_before_expiry(self) -> None:
        dsm = _make_dsm(silence_timeout=300.0)
        dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        assert dsm.check_silence_timeout() is None  # not expired yet


class TestUserInputClassification:
    """Verify on_user_input correctly classifies raw text into events."""

    def test_wake_word_robbox(self) -> None:
        dsm = _make_dsm()
        assert dsm.on_user_input("роббокс") == DialogueEvent.WAKE_WORD

    def test_wake_word_rob_box(self) -> None:
        dsm = _make_dsm()
        assert dsm.on_user_input("роб бокс") == DialogueEvent.WAKE_WORD

    def test_wake_word_robot(self) -> None:
        dsm = _make_dsm()
        assert dsm.on_user_input("робот") == DialogueEvent.WAKE_WORD

    def test_silence_tikho(self) -> None:
        dsm = _make_dsm()
        assert dsm.on_user_input("тихо") == DialogueEvent.SILENCE_COMMAND

    def test_silence_molchi(self) -> None:
        dsm = _make_dsm()
        assert dsm.on_user_input("молчи") == DialogueEvent.SILENCE_COMMAND

    def test_silence_zamolchi(self) -> None:
        dsm = _make_dsm()
        assert dsm.on_user_input("замолчи") == DialogueEvent.SILENCE_COMMAND

    def test_silence_khvatit(self) -> None:
        dsm = _make_dsm()
        assert dsm.on_user_input("хватит") == DialogueEvent.SILENCE_COMMAND

    def test_silence_stop(self) -> None:
        dsm = _make_dsm()
        assert dsm.on_user_input("стоп") == DialogueEvent.SILENCE_COMMAND

    def test_silence_priority_over_wake_word(self) -> None:
        """Silence command in text takes priority over wake word."""
        dsm = _make_dsm()
        assert dsm.on_user_input("роббокс замолчи") == DialogueEvent.SILENCE_COMMAND

    def test_normal_text_stt_result(self) -> None:
        dsm = _make_dsm()
        assert dsm.on_user_input("привет как дела") == DialogueEvent.STT_RESULT

    def test_empty_input_timeout(self) -> None:
        dsm = _make_dsm()
        assert dsm.on_user_input("") == DialogueEvent.TIMEOUT

    def test_whitespace_only_timeout(self) -> None:
        dsm = _make_dsm()
        assert dsm.on_user_input("   ") == DialogueEvent.TIMEOUT


class TestLifecycle:
    """Reset and lifecycle behaviour."""

    def test_reset_to_idle_from_listening(self) -> None:
        dsm = _make_dsm()
        dsm.on_event(DialogueEvent.WAKE_WORD)
        assert dsm.state == DialogueStateKind.LISTENING
        dsm.reset(DialogueStateKind.IDLE)
        assert dsm.state == DialogueStateKind.IDLE

    def test_reset_to_silenced(self) -> None:
        dsm = _make_dsm()
        dsm.reset(DialogueStateKind.SILENCED)
        assert dsm.state == DialogueStateKind.SILENCED


class TestFullSessionFlow:
    """End-to-end state machine flow for a typical dialogue session."""

    def test_normal_dialogue_flow(self) -> None:
        dsm = _make_dsm()
        state = _make_state()

        # 1. User says wake word
        event = dsm.on_user_input("роббокс", state)
        assert event == DialogueEvent.WAKE_WORD
        dsm.on_event(event, state)
        assert dsm.state == DialogueStateKind.LISTENING

        # 2. User speaks normally
        event = dsm.on_user_input("какая сегодня погода", state)
        assert event == DialogueEvent.STT_RESULT
        dsm.on_event(event, state)
        assert dsm.state == DialogueStateKind.DIALOGUE

        # 3. Assistant finishes responding
        dsm.on_event(DialogueEvent.DIALOGUE_END, state)
        assert dsm.state == DialogueStateKind.IDLE

    def test_silence_interrupt_flow(self) -> None:
        dsm = _make_dsm(silence_timeout=0.01)
        state = _make_state()

        # Start dialogue
        dsm.on_user_input("роббокс", state)
        dsm.on_event(DialogueEvent.WAKE_WORD, state)
        dsm.on_user_input("привет", state)
        dsm.on_event(DialogueEvent.STT_RESULT, state)
        assert dsm.state == DialogueStateKind.DIALOGUE

        # User says "тихо" mid-dialogue
        dsm.on_event(DialogueEvent.SILENCE_COMMAND, state)
        assert dsm.state == DialogueStateKind.SILENCED

        # Wait for timeout
        time.sleep(0.02)
        timeout_event = dsm.check_silence_timeout()
        assert timeout_event == DialogueEvent.TIMEOUT
        dsm.on_event(timeout_event, state)
        assert dsm.state == DialogueStateKind.IDLE

    def test_barge_in_flow(self) -> None:
        dsm = _make_dsm()
        state = _make_state()

        # Start dialogue
        dsm.on_user_input("роббокс", state)
        dsm.on_event(DialogueEvent.WAKE_WORD, state)
        dsm.on_user_input("расскажи анекдот", state)
        dsm.on_event(DialogueEvent.STT_RESULT, state)
        assert dsm.state == DialogueStateKind.DIALOGUE

        # User interrupts with wake word
        dsm.on_event(DialogueEvent.WAKE_WORD, state)
        assert dsm.state == DialogueStateKind.LISTENING

        # New speech input
        dsm.on_user_input("нет, лучше расскажи историю", state)
        dsm.on_event(DialogueEvent.STT_RESULT, state)
        assert dsm.state == DialogueStateKind.DIALOGUE
