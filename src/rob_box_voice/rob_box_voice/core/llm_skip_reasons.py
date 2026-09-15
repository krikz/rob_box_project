"""Canonical reasons why dialogue processing can skip an LLM dispatch."""

from __future__ import annotations

from enum import Enum


class LLMSkipReason(str, Enum):
    """Stable metric keys used by ``DialogueNode._llm_skipped_counter``.

    Single source of truth — :class:`rob_box_voice.core.stt_admission`
    bumps these keys (and only these) via the central ``evaluate``
    counter write. Issue #1389 regression test
    ``test_counter_keys_match_constant`` enforces that every increment
    site in ``dialogue_node.py`` AND ``core/stt_admission.py`` matches
    one of the members below.
    """

    NO_WAKE_WORD = "no_wake_word"
    SILENCED = "silenced"
    SILENCE_COMMAND = "silence_command"
    EMPTY_TEXT = "empty_text"
    EMPTY_AFTER_STRIP = "empty_after_strip"
    STT_REJECTED = "stt_rejected"
    MUSIC_STOP = "music_stop"
    COMMAND_INTENT = "command_intent"
    NEW_SESSION = "new_session"
    QUICK_DECIDE_IGNORE = "quick_decide_ignore"
    PENDING_LLM = "pending_llm"
    BACKLOG_ACCUMULATED = "backlog_accumulated"
    UNSILENCE = "unsilence"


def new_llm_skip_counter() -> dict[str, int]:
    """Return a strict, zero-initialized counter for every known reason."""

    return {reason.value: 0 for reason in LLMSkipReason}
