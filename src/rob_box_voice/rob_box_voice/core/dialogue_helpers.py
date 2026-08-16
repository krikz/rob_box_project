"""dialogue_helpers.py — pure helper functions for DialogueNode (TD-1).

Extracted from :class:`rob_box_voice.dialogue_node.DialogueNode` so the
stateless string/dict helpers can be unit-tested without a ROS2 node.
Pure Python: no rclpy, no I/O.

Owns several families of helpers:

* :func:`map_emotion_to_animation` — emotion label → LED animation key.
* :func:`generate_fallback_response` — static offline replies when the
  LLM is unavailable.
* :func:`detect_volume_intent` / :func:`detect_pitch_intent` — keyword
  intent detection for volume/pitch adjustment commands.
* :func:`sanitize_speaker_name` — guard against junk speaker names
  (issue #1077 / #1101: resemblyzer may store "Null"/"null"/"None").
* :func:`format_llm_skipped_summary` — one-line diagnostics summary of
  LLM-skip counters (issue #1101).
"""

from __future__ import annotations

from typing import Mapping, Optional

#: Speaker names that are not real names — resemblyzer may store junk
#: values like "Null", "null", "None" in the DB (live 12.08 fix).
INVALID_SPEAKER_NAMES: frozenset = frozenset(
    {"null", "none", "undefined", "unknown", ""}
)

EMOTION_TO_ANIMATION: dict = {
    "happy": "happy",
    "sad": "sad",
    "angry": "angry",
    "surprised": "surprised",
    "thinking": "thinking",
    "excited": "victory",
    "confused": "thinking",
    "worried": "sad",
    "neutral": "idle",
    "calm": "idle",
}


def map_emotion_to_animation(emotion: str) -> str:
    """Map an emotion label to its corresponding LED animation key.

    Lookup is case-insensitive; unknown emotions default to ``"idle"``.
    """
    if not emotion:
        return "idle"
    return EMOTION_TO_ANIMATION.get(str(emotion).lower(), "idle")


def generate_fallback_response(text: str) -> str:
    """Generate a static fallback reply when the LLM is unavailable."""
    low = (text or "").lower()
    greetings = (
        "привет", "здравствуй", "hello", "hi ", "доброе утро",
        "добрый день", "добрый вечер",
    )
    if any(g in low for g in greetings):
        return (
            "Привет! К сожалению, интернет сейчас недоступен, "
            "но я могу выполнять базовые команды."
        )
    if any(w in low for w in ("как дела", "how are")):
        return (
            "У меня всё хорошо! Но интернет сейчас недоступен, "
            "часть функций ограничена."
        )
    thanks = ("спасибо", "благодарю", "thanks", "thank you")
    if any(w in low for w in thanks):
        return "Пожалуйста!"
    farewells = ("пока", "до свидания", "bye", "goodbye", "прощай")
    if any(w in low for w in farewells):
        return "До свидания!"
    return "Интернет сейчас недоступен, я не могу ответить на этот вопрос."


def detect_volume_intent(text: str):
    """Detect volume adjustment intent from user text."""
    if not text:
        return None
    low = text.lower()
    if any(w in low for w in ("максимальная громкость", "максимум громк")):
        return "max"
    if any(w in low for w in ("нормальная громкость", "стандартная громкость", "обычная громкость")):
        return "normal"
    if "говори громко" in low:
        return "max"
    if any(w in low for w in ("громче", "громко")):
        return "louder"
    if any(w in low for w in ("тише", "потише")):
        return "quieter"
    return None


def detect_pitch_intent(text: str):
    """Detect pitch adjustment intent from user text."""
    if not text:
        return None
    low = text.lower()
    if any(w in low for w in ("нормальный голос", "говори нормально", "обычный голос")):
        return "normal"
    if any(w in low for w in ("выше", "повысь")):
        return "higher"
    if any(w in low for w in ("ниже",)):
        return "lower"
    return None


def sanitize_speaker_name(name: Optional[str]) -> str:
    """Return the cleaned speaker name, or ``""`` for junk values.

    resemblyzer may store garbage like ``"Null"``/``"null"``/``"None"``
    in the speaker DB (live 12.08 fix, issues #1077/#1101). Such values
    are not names — callers should treat them as "unknown speaker".

    Args:
        name: Raw speaker name from the speaker DB (may be None).

    Returns:
        Trimmed name with junk values normalized to ``""``.
    """
    if name is None:
        return ""
    cleaned = str(name).strip()
    if cleaned.lower() in INVALID_SPEAKER_NAMES:
        return ""
    return cleaned


def format_llm_skipped_summary(
    counter: Mapping[str, int],
    window_s: float = 300.0,
) -> Optional[str]:
    """Build the one-line diagnostics summary for LLM-skip counters.

    Issue #1101 — periodically log why the robot is silent. Returns
    ``None`` when there is nothing to report (no skips), so callers can
    skip logging entirely without spamming empty windows.

    Args:
        counter: Mapping of skip reason -> occurrence count.
        window_s: Reporting window in seconds (used only in the message).

    Returns:
        Formatted summary string, or ``None`` when ``total == 0``.
    """
    total = sum(counter.values())
    if total == 0:
        return None
    breakdown = ", ".join(
        f"{k}={v}"
        for k, v in counter.items()
        if v > 0
    )
    return (
        f"📊 [diagnostics] llm_skipped_total={total} (since startup, "
        f"last {window_s:.0f}s): {breakdown}"
    )
