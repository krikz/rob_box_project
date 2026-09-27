"""Regression test for issue #1392 v3 — music_skill tool ordering + prompt content.

Live regression 18.08.2026 17:05 MSK (Vision Pi):
LLM-Compositor-M3 responded "🎵 Достаю генератор!" but returned tools=[],
never calling handle_music/generate_music despite user asking to
"сгенерируй песню про космос".

Two root causes locked in here:
  1. music_skill._make_tools() must return generate_music FIRST so the
     sub-agent's tool list has MiniMax tools in the first 7 slots.
  2. generate_music docstring must contain explicit ★ USE THIS TOOL WHEN ★
     triggers listing Russian vocal/song request phrasings.

Also checks music_skill_prompt.txt contains:
  - DJ MODE — RULE #1 (Renardo is alive, not deleted!)
  - MiniMax free plan cost mention (3 requests/minute awareness)
  - speak_text.animation enum (required by test_prompt_animation_enum)
"""
from __future__ import annotations

import re
from pathlib import Path

MUSIC_SKILL_PATH = Path(
    "src/rob_box_voice/rob_box_voice/skills/music_skill.py"
)
MUSIC_SKILL_PROMPT_PATH = Path(
    "src/rob_box_voice/prompts/skills/music_skill_prompt.txt"
)


def _extract_tool_order(source: str) -> list[str]:
    """Return the order of @function_tool definitions in music_skill.py."""
    return re.findall(
        r"@function_tool\s*\n\s+(?:async )?def\s+(\w+)\(",
        source,
    )


def test_generate_music_is_first_tool_in_music_skill() -> None:
    """LLM picks the FIRST tool in a function-calling list. MiniMax first."""
    source = MUSIC_SKILL_PATH.read_text(encoding="utf-8")
    tools = _extract_tool_order(source)
    assert tools, "No @function_tool definitions found in music_skill.py"
    assert tools[0] == "generate_music", (
        f"generate_music must be FIRST tool, got order: {tools[:5]}"
    )


def test_gen_library_tools_immediately_after_generate_music() -> None:
    """All 6 gen_* library tools should be in slots 2-7 (right after generate_music)."""
    source = MUSIC_SKILL_PATH.read_text(encoding="utf-8")
    tools = _extract_tool_order(source)
    gen_tools = [t for t in tools if t.startswith("gen_")]
    assert len(gen_tools) >= 6, f"Expected 6+ gen_* tools, got: {gen_tools}"
    # First 7 should be: generate_music + 6 gen_*
    assert tools[:7] == [
        "generate_music",
        "gen_list_library",
        "gen_search_library",
        "gen_save_to_library",
        "gen_play_from_library",
        "gen_delete_from_library",
        "gen_get_track_info",
    ], f"First 7 tools wrong: {tools[:7]}"


def test_generate_music_docstring_lists_russian_vocal_triggers() -> None:
    """The docstring must include Russian vocal/song request phrasings."""
    source = MUSIC_SKILL_PATH.read_text(encoding="utf-8")
    # Locate generate_music's docstring
    m = re.search(
        r'@function_tool\s*\n\s+async def generate_music\([^)]*\)[^"]*"""(.*?)"""',
        source,
        re.S,
    )
    assert m, "generate_music docstring not found"
    docstring = m.group(1)
    # Must mention these triggers
    for trigger in ("спой", "сочини", "сгенерируй", "USE THIS TOOL WHEN"):
        assert trigger in docstring, (
            f"Missing trigger {trigger!r} in generate_music docstring"
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
    """The {renardo_ref} placeholder must remain for skill_factory substitution."""
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")
    assert "{renardo_ref}" in content, (
        "{renardo_ref} placeholder missing — RENARDO_REFERENCE.md won't be substituted"
    )
    assert "{music_library_enabled}" in content, (
        "{music_library_enabled} placeholder missing"
    )
