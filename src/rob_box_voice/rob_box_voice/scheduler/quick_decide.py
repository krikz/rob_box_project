"""quick_decide.py — S4.1 rules-level input classification (issue #968).

Ревизия v5 (SCHEDULER_DESIGN.md §4.7): **никакой второй LLM**. Level 1
is a narrow, fast (< 50ms) rules pass that catches only obvious noise
and explicit imperatives. Everything else — including any
conjunction/pronoun-bearing follow-up like "и ещё про X" — is
``PENDING_LLM``: the main LLM cycle sees ``[ACTIVE TASKS]``/
``[SEGMENT PLAN]`` and decides MERGE/REPLACE/ordinary reply itself.
Rules deliberately do NOT try to parse continuations — §4.7.2's
own table warns against exactly that: "Никаких regex на союзы/«и
ещё» — это контекстно-зависимо, пусть решает LLM".

Pure Python, no ROS, no scheduler import — mirrors the style of
:mod:`rob_box_voice.core.dialogue_text`. Same as that module,
``clock``/``previous_text``/``previous_ts`` are passed in rather than
tracked internally, so the function stays a pure, side-effect-free
classifier that is trivial to unit test.
"""

from __future__ import annotations

from enum import Enum
from typing import Callable, Optional

from rob_box_voice.core.dialogue_guards import MUSIC_STOP_OVERRIDES


class QuickVerdict(str, Enum):
    """Level-1 rules verdict (§4.7.3, v5 — only these three)."""

    IGNORE = "IGNORE"
    REPLACE = "REPLACE"
    PENDING_LLM = "PENDING_LLM"


#: §4.7.2 table row: "Императив с модальным глаголом и одним
#: существительным → REPLACE". Kept as an explicit whitelist (not a
#: broader heuristic) so it stays precise and fast.
_REPLACE_IMPERATIVES: tuple[str, ...] = ("хватит", "стоп", "замолчи", "отставить")

#: §4.7.2 table row: interjections/fillers → IGNORE. Deliberately a
#: small, exact-match set (not substring) — these are short enough
#: that substring matching would false-positive too easily.
_INTERJECTIONS: frozenset[str] = frozenset({
    "угу", "ага", "мхм", "ну", "хм", "мм", "эм", "да", "неа",
})

#: §4.7.2 table row: exact repeat of the previous phrase within a
#: short window → IGNORE (dedup echo).
DEDUP_WINDOW_S: float = 0.5

#: Confidence floor for the "неразборчиво" IGNORE rule.
CONFIDENCE_FLOOR: float = 0.4


def _is_music_stop_override(text_lower: str) -> bool:
    return any(kw in text_lower for kw in MUSIC_STOP_OVERRIDES)


def quick_decide(
    text: str,
    *,
    source: str = "stt",
    active_group: Optional[str] = None,
    clock: Callable[[], float],
    confidence: Optional[float] = None,
    previous_text: Optional[str] = None,
    previous_ts: Optional[float] = None,
) -> QuickVerdict:
    """Classify *text* as IGNORE / REPLACE / PENDING_LLM (§4.7.3, v5).

    Args:
        text: Raw utterance (already wake-word-stripped upstream).
        source: Where *text* came from (``"stt"``, ``"telegram"``, ...).
            Not used by the rules yet — kept for parity with the
            design's signature and for future source-specific rules.
        active_group: The currently-active segment group_id, if any.
            Not used by the rules yet (level 1 does not need scheduler
            state to catch noise/imperatives) — kept so callers don't
            need a different call shape once FROZEN/LIVE (S9) rules
            land here.
        clock: Zero-arg callable returning the current time (seconds,
            monotonic) — used only for the dedup-repeat window.
        confidence: ASR confidence for *text*, if known.
        previous_text: The immediately preceding utterance, if any —
            caller-tracked (this function has no memory of its own).
        previous_ts: ``clock()`` value at *previous_text*, if any.

    Returns:
        ``QuickVerdict.IGNORE`` or ``QuickVerdict.REPLACE`` when a rule
        matches with high confidence; ``QuickVerdict.PENDING_LLM``
        otherwise — the normal, expected outcome for anything that
        isn't obvious noise or an explicit imperative.
    """
    stripped = (text or "").strip()
    if not stripped:
        return QuickVerdict.IGNORE

    if confidence is not None and confidence < CONFIDENCE_FLOOR:
        return QuickVerdict.IGNORE

    text_lower = stripped.lower().rstrip(".,!?")

    if (
        previous_text is not None
        and previous_ts is not None
        and text_lower == previous_text.strip().lower().rstrip(".,!?")
        and (clock() - previous_ts) <= DEDUP_WINDOW_S
    ):
        return QuickVerdict.IGNORE

    if text_lower in _INTERJECTIONS:
        return QuickVerdict.IGNORE

    # Imperatives are checked BEFORE the word-count rule — most of
    # them ("хватит", "стоп") are themselves a single word and would
    # otherwise be swallowed by "< 2 слов → IGNORE".
    if (
        any(kw in text_lower for kw in _REPLACE_IMPERATIVES)
        and not _is_music_stop_override(text_lower)
    ):
        return QuickVerdict.REPLACE

    if len(text_lower.split()) < 2:
        return QuickVerdict.IGNORE

    return QuickVerdict.PENDING_LLM
