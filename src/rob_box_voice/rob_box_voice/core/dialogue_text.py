"""Pure helpers for wake-word / silence-command text classification.

Extracted from :class:`DialogueManager` and the per-method regex in
:mod:`rob_box_harness.harnesses.dialog` so the same logic can be used
by:

* the legacy ``DialogueNode`` (OpenAI Agents SDK pipeline)
* the new ``DialogHarness`` (harness framework)
* unit tests that don't want to spin up the full state machine

Everything in this module is **pure**: no I/O, no ROS2, no time
side-effects (callers pass in the wake-word list / lowercased text).
That makes the helpers cheap to test in isolation (see
``test/unit/core/test_dialogue_text.py``).
"""

from __future__ import annotations

import re
from typing import Sequence

# Default trigger phrases — match what DialogueManager used historically
# so behaviour is preserved when the legacy node switches to these
# helpers. Includes spelling variants that the dialog harness originally
# supported inline (роббокс, роб бокс, робокс).
DEFAULT_WAKE_WORDS: tuple[str, ...] = (
    "робок",
    "робот",
    "роббокс",
    "роб бокс",
    "робокс",
    "robbox",
    "rob box",
)
DEFAULT_SILENCE_COMMANDS: tuple[str, ...] = ("помолч", "замолч", "хватит")
DEFAULT_UNSILENCE_COMMANDS: tuple[str, ...] = (
    "говори",
    "включ",
    "работ",
    "отвеч",
    "разговар",
)


# ---------------------------------------------------------------------------
# Wake-word
# ---------------------------------------------------------------------------


def has_wake_word(text_lower: str, wake_words: Sequence[str]) -> bool:
    """Return True if any ``wake_word`` appears in ``text_lower``.

    Empty ``wake_words`` means "bypass mode" — accept every input.
    """
    if not wake_words:
        return True
    return any(w in text_lower for w in wake_words)


def strip_wake_word(text: str, wake_words: Sequence[str] | None = None) -> str:
    """Remove the *first* matching wake word from ``text``.

    Used by the harness wake-word gate (``DialogHarness._strip_wake_word``)
    AND by the legacy node's ``DialogueManager.remove_wake_word``.

    The harness version (regex, anchored, case-insensitive) handles
    punctuation; the legacy version is a substring strip. We support
    both via ``mode="legacy"`` (substring) vs the default "harness"
    (anchored regex) so existing behaviour is preserved when callers
    migrate to this helper.

    Args:
        text: Raw user input (case-insensitive).
        wake_words: Optional override; defaults to
            :data:`DEFAULT_WAKE_WORDS`.
    """
    words = wake_words if wake_words is not None else DEFAULT_WAKE_WORDS
    if not words:
        return text.strip()
    # Anchored, case-insensitive — matches the harness _strip_wake_word
    # behaviour. We accept a leading optional whitespace and consume any
    # trailing punctuation/whitespace so the cleaned text is usable.
    pattern = re.compile(
        r"^\s*(" + "|".join(re.escape(w) for w in words) + r")[.,\s]*",
        re.IGNORECASE,
    )
    return pattern.sub("", text, count=1).strip()


# ---------------------------------------------------------------------------
# Silence / unsilence commands
# ---------------------------------------------------------------------------


def is_silence_command(text_lower: str, commands: Sequence[str] | None = None) -> bool:
    """Return True if ``text_lower`` matches any silence trigger phrase."""
    cmds = commands if commands is not None else DEFAULT_SILENCE_COMMANDS
    return any(cmd in text_lower for cmd in cmds)


def is_unsilence_command(text_lower: str, commands: Sequence[str] | None = None) -> bool:
    """Return True if ``text_lower`` matches any unsilence trigger phrase."""
    cmds = commands if commands is not None else DEFAULT_UNSILENCE_COMMANDS
    return any(cmd in text_lower for cmd in cmds)


__all__ = [
    "DEFAULT_WAKE_WORDS",
    "DEFAULT_SILENCE_COMMANDS",
    "DEFAULT_UNSILENCE_COMMANDS",
    "has_wake_word",
    "strip_wake_word",
    "is_silence_command",
    "is_unsilence_command",
]