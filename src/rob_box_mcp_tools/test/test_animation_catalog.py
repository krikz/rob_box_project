"""The animation vocabulary of the tools is self-consistent.

That every name resolves to a scene of the procedural face is checked next
to the face itself (``tests/unit/docker/test_animation_assets.py``): a name
the LLM is offered with nothing behind it is a matrix that shows nothing —
which is how ``excited`` ended up hard-coded as a special case in
``play_animation``.
"""

from __future__ import annotations

from rob_box_mcp_tools.animations import (
    ANIMATION_ALIASES,
    KNOWN_ANIMATIONS,
    normalize_animation,
)


def test_aliases_never_shadow_a_real_animation() -> None:
    """An alias must not share a name with an animation, and must resolve to one.

    A name in both places is ambiguous: the enum would offer it while the
    normaliser silently rewrote it to something else.
    """
    for alias, target in ANIMATION_ALIASES.items():
        assert alias not in KNOWN_ANIMATIONS, (
            f"{alias!r} is both an alias and a real animation"
        )
        assert target in KNOWN_ANIMATIONS, (
            f"alias {alias!r} resolves to {target!r}, which is not an animation"
        )


def test_normalize_animation_handles_the_cases_that_reach_it() -> None:
    """Aliases, Russian labels, unknown names and empty input."""
    assert normalize_animation("happy") == "happy"
    assert normalize_animation("HAPPY") == "happy"
    # MiniMax M3 emits this constantly; it is not an animation.
    assert normalize_animation("excited") == "happy"
    assert normalize_animation("радость") == "happy"
    assert normalize_animation("talk") == "talking"
    assert normalize_animation("") == "idle"
    assert normalize_animation(None) == "idle"
    assert normalize_animation("no_such_animation") == "idle"
    assert normalize_animation("no_such_animation", fallback="talking") == "talking"


def test_vocabulary_is_the_one_table_of_the_dialogue() -> None:
    """#4103: the dialogue builds the show queue from the same table — one source, re-exported here, not a copy."""
    from rob_box_dialog import knowledge as kn

    assert KNOWN_ANIMATIONS is kn.KNOWN_ANIMATIONS
    assert ANIMATION_ALIASES is kn.ANIMATION_ALIASES
    # Russian words a person uses for the show queue normalise for the tool as well.
    assert normalize_animation("удивление") == "surprised"
    assert normalize_animation("Нейтральное") == "idle"
    assert normalize_animation("удивлённое") == "surprised"
