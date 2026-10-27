"""LED animation vocabulary — one declaration for every tool that uses it.

The robot's face draws procedural scenes (``face_node``, ADR-0150 §5.5):
every name here resolves to a scene of ``rob_box_animations.face.states``
(``SCENES`` / ``LEGACY``), and ``tests/unit/docker/test_animation_assets.py``
fails the build when a name has nothing behind it. The scene catalogue is
wider than this list; extending the tool vocabulary is PR-3c (#3691).

Before this module the same vocabulary existed three times and disagreed:

* ``SpeakTextTool`` had a 21-name list inside ``execute()`` plus a private
  alias map — correct, and the only one that normalised its input.
* ``PlayAnimationTool.AVAILABLE_ANIMATIONS`` had 22 names including
  ``excited``, which is an alias rather than an animation — it has no
  manifest. The tool advertised it as a valid value and then rewrote it
  with a hand-written ``if animation == "excited"``, so every further word
  from the model's emotion vocabulary needed its own special case here,
  separately from the alias map ``speak_text`` already had.
* The parameter description spelled a third, prose version of the list.

Aliases stay *outside* the enum on purpose. They exist because models
reliably emit words from the general emotion vocabulary (``excited``,
``neutral``, ``confused``) and because Russian input reaches the tools
directly; normalising them is friendlier than rejecting them. But they are
not animations, so they must never be advertised as valid values.
"""

from __future__ import annotations

__all__ = [
    "ANIMATION_ALIASES",
    "KNOWN_ANIMATIONS",
    "SHOW_MAX_S",
    "SHOW_MIN_S",
    "clamp_show_seconds",
    "normalize_animation",
]

#: ``play_animation``: допустимая длительность показа, секунды; вне диапазона - ``SHOW_MIN_S``.
SHOW_MIN_S, SHOW_MAX_S = 2.0, 30.0

#: Every name the ``play_animation`` tool offers; each resolves to a face scene.
KNOWN_ANIMATIONS: tuple[str, ...] = (
    "accelerating",
    "ambulance",
    "angry",
    "braking",
    "charging",
    "error",
    "fire_truck",
    "happy",
    "idle",
    "low_battery",
    "police_lights",
    "road_service",
    "sad",
    "sleep",
    "surprised",
    "talking",
    "thinking",
    "turn_left",
    "turn_right",
    "victory",
    "wakeup",
)

#: Names that are *not* animations but are normalised to one. Russian labels
#: come from direct speech; the English ones from the LLM's emotion
#: vocabulary (MiniMax M3 emits ``excited`` constantly).
ANIMATION_ALIASES: dict[str, str] = {
    # Русские названия
    "нейтрально": "idle",
    "нейтральная": "idle",
    "нейтральный": "idle",
    "радость": "happy",
    "радостный": "happy",
    "счастливый": "happy",
    "грустный": "sad",
    "грусть": "sad",
    "печаль": "sad",
    "злой": "angry",
    "злость": "angry",
    "возбужденный": "happy",
    "возбуждение": "happy",
    "смущенный": "thinking",
    "смущение": "thinking",
    "растерянный": "thinking",
    # Эмоции из общего словаря LLM, которых нет среди анимаций
    "neutral": "idle",
    "excited": "happy",
    "confused": "thinking",
    "laughing": "happy",
    "smiling": "happy",
    # ``dancing`` раньше маппился в "excited" — такой анимации не
    # существует, поэтому нормализация всё равно скатывалась в fallback.
    "dancing": "happy",
    "singing": "happy",
    # LLM часто пишет "talk" вместо "talking"
    "talk": "talking",
}


def normalize_animation(animation: str | None, *, fallback: str = "idle") -> str:
    """Resolve *animation* to a name the player can actually render.

    Applies the alias map case-insensitively; anything still unknown
    becomes *fallback*. Callers that need to report the substitution should
    compare the result against their input.
    """
    if not animation:
        return fallback
    candidate = str(animation).strip().lower()
    candidate = ANIMATION_ALIASES.get(candidate, candidate)
    return candidate if candidate in KNOWN_ANIMATIONS else fallback


def clamp_show_seconds(duration: float | None) -> float | None:
    """Длительность показа ``play_animation``: ``None`` - по умолчанию лица; вне ``[SHOW_MIN_S, SHOW_MAX_S]`` - минимум.

    Правило тула: его применяют ответ тула модели и диалог v1 (``turn_tools.show_seconds``). Диалог v2 приводит
    время из данных тула к своему правилу показа ``rob_box_dialog.expression.show_seconds`` (#4083: не меньше
    ``FACE_SHOW_MIN_S``), поэтому 60 с у v2 дают 2 с тула, а на лице 5 с, а не 30. Свести в одно правило - после
    удаления v1 (PR-15): тест v1 (``test_turn_stages``) держит нынешнее «вне диапазона - минимум».
    """
    if duration is None:
        return None
    seconds = float(duration)
    return seconds if SHOW_MIN_S <= seconds <= SHOW_MAX_S else SHOW_MIN_S
