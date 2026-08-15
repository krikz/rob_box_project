"""dialogue_helpers.py — pure helper functions for DialogueNode (TD-1).

Extracted from :class:`rob_box_voice.dialogue_node.DialogueNode` so the
stateless string/dict helpers can be unit-tested without a ROS2 node.
Pure Python: no rclpy, no I/O.

Owns four families of helpers:

* :func:`map_emotion_to_animation` — emotion label → LED animation key.
* :func:`generate_fallback_response` — static offline replies when the
  LLM is unavailable.
* :func:`detect_volume_intent` / :func:`detect_pitch_intent` — keyword
  intent detection for volume/pitch adjustment commands.
"""

from __future__ import annotations

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
