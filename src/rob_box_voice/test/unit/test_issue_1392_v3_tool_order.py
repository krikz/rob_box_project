"""Regression test for issue #1392 v3 — music_skill prompt content.

Live regression 18.08.2026 17:05 MSK (Vision Pi): LLM-Compositor-M3
responded "🎵 Достаю генератор!" but returned tools=[], never calling the
music-generation tools despite the user asking to "сгенерируй песню про
космос".

The tool-ordering half of this test covered ``music_skill.py``, the local
Compositor sub-agent that was retired during the harness migration and has
now been deleted along with the rest of the dead skill package. What
remains is the live contract: ``music_skill_prompt.txt`` is read verbatim
by ``dialogue_node._load_system_prompt``, so drift there is user-visible.

Checks that music_skill_prompt.txt contains:
  - DJ MODE — RULE #1 (Renardo is alive, not deleted!)
  - MiniMax free plan cost mention (3 requests/minute awareness)
  - speak_text.animation enum
  - the {renardo_ref} placeholder
"""
from __future__ import annotations

from pathlib import Path

MUSIC_SKILL_PROMPT_PATH = Path(
    "src/rob_box_voice/prompts/skills/music_skill_prompt.txt"
)


def test_music_skill_prompt_has_renardo_dj_mode_rule() -> None:
    """Regression for issue #1392 v3 user complaint — Renardo DJ rule preserved."""
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")
    assert "DJ MODE" in content, "DJ MODE section missing from music_skill_prompt"
    assert "set_dj_mode" in content, "set_dj_mode reference missing"
    assert "Clock.clear()" in content, "Clock.clear() rule missing"


def test_music_skill_prompt_has_minimax_cost_awareness() -> None:
    """Cost-awareness: MiniMax free plan has 3 requests/minute limit."""
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")
    # Must mention rate limit and 429 handling
    assert "3 requests/minute" in content or "3 req" in content, (
        "MiniMax rate-limit mention missing from music_skill_prompt"
    )
    assert "429" in content, "429 error handling missing from prompt"


def test_music_skill_prompt_has_animation_enum() -> None:
    """Required by test_prompt_animation_enum — outer Compositor picks animation."""
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")
    assert "speak_text.animation" in content, (
        "speak_text.animation enum missing from music_skill_prompt"
    )


def test_music_skill_prompt_renardo_reference_placeholder_preserved() -> None:
    """The {renardo_ref} placeholder must remain for prompt substitution."""
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")
    assert "{renardo_ref}" in content, (
        "{renardo_ref} placeholder missing — RENARDO_REFERENCE.md won't be substituted"
    )
    assert "{music_library_enabled}" in content, (
        "{music_library_enabled} placeholder missing"
    )
