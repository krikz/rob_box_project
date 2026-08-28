"""Canonical reasons why dialogue processing can skip an LLM dispatch."""

from __future__ import annotations

from enum import Enum


class LLMSkipReason(str, Enum):
    """Stable metric keys used by ``DialogueNode._llm_skipped_counter``."""

    NO_WAKE_WORD = "no_wake_word"
    SILENCED = "silenced"
    SILENCE_COMMAND = "silence_command"
    EMPTY_AFTER_STRIP = "empty_after_strip"
    STT_REJECTED = "stt_rejected"
    MUSIC_STOP = "music_stop"
    COMMAND_INTENT = "command_intent"
    NEW_SESSION = "new_session"


def new_llm_skip_counter() -> dict[str, int]:
    """Return a strict, zero-initialized counter for every known reason."""

    return {reason.value: 0 for reason in LLMSkipReason}
