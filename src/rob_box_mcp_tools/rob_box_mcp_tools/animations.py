"""LED animation vocabulary — one declaration for every tool that uses it.

The robot's animations are defined by the manifests in
``src/rob_box_animations/animations/manifests/*.yaml``; that directory is
the source of truth, and ``test_animation_catalog_matches_manifests``
fails the build when this list drifts from it.

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
    "normalize_animation",
]

#: Every animation with a manifest, i.e. every name the player can render.
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
