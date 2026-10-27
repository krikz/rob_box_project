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

#4103: the dialogue needs the same vocabulary — it builds the queue of
scenes for «покажи по очереди радость, грусть и удивление» from the
person's words — and ``rob_box_dialog`` must not import this ROS package.
So the table itself (``KNOWN_ANIMATIONS``, ``ANIMATION_ALIASES``,
``animation_name``) lives in ``rob_box_dialog.knowledge``; this module
re-exports it and keeps the tool-side rules (fallback, show seconds).
"""

from __future__ import annotations

from rob_box_dialog.knowledge import ANIMATION_ALIASES, KNOWN_ANIMATIONS, animation_name

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


def normalize_animation(animation: str | None, *, fallback: str = "idle") -> str:
    """Resolve *animation* to a name the player can actually render.

    One rule with the dialogue (``rob_box_dialog.knowledge.animation_name``):
    the alias map applies case-insensitively; anything still unknown becomes
    *fallback*. Callers that need to report the substitution should compare
    the result against their input.
    """
    return animation_name(animation) or fallback


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
