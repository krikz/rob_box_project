"""The animation vocabulary must match what the player can actually render.

``rob_box_animations`` renders one animation per manifest in
``animations/manifests/*.yaml``. Any name a tool advertises without a
manifest behind it is a tool the LLM will happily call and a matrix that
shows nothing — which is how ``excited`` ended up hard-coded as a special
case in ``play_animation``.
"""

from __future__ import annotations

import pathlib

import pytest

from rob_box_mcp_tools.animations import (
    ANIMATION_ALIASES,
    KNOWN_ANIMATIONS,
    normalize_animation,
)

MANIFEST_DIR = (
    pathlib.Path(__file__).resolve().parents[2]
    / "rob_box_animations"
    / "animations"
    / "manifests"
)


def _manifest_names() -> set[str]:
    return {p.stem for p in MANIFEST_DIR.glob("*.yaml")}


@pytest.mark.skipif(
    not MANIFEST_DIR.is_dir(),
    reason="rob_box_animations sources unavailable (installed package layout)",
)
def test_animation_catalog_matches_manifests() -> None:
    """Every advertised animation exists, and every animation is advertised."""
    manifests = _manifest_names()
    assert manifests, f"no manifests found under {MANIFEST_DIR}"
    advertised = set(KNOWN_ANIMATIONS)
    assert not advertised - manifests, (
        "animations advertised to the LLM with no manifest behind them: "
        f"{sorted(advertised - manifests)}"
    )
    assert not manifests - advertised, (
        "animations the robot can play but no tool offers: "
        f"{sorted(manifests - advertised)}"
    )


@pytest.mark.skipif(
    not MANIFEST_DIR.is_dir(),
    reason="rob_box_animations sources unavailable (installed package layout)",
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
            f"alias {alias!r} resolves to {target!r}, which has no manifest"
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
