"""Regression tests for music runtime startup assets and prompt guardrails."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[5]
FOXDOT_INIT_PATH = REPO_ROOT / "docker" / "vision" / "voice_assistant" / "foxdot_init.sc"
START_VOICE_ASSISTANT_PATH = REPO_ROOT / "docker" / "vision" / "scripts" / "voice_assistant" / "start_voice_assistant.sh"
MASTER_PROMPT_PATH = REPO_ROOT / "src" / "rob_box_voice" / "prompts" / "master_prompt_compact.txt"
MUSIC_SKILL_PROMPT_PATH = REPO_ROOT / "src" / "rob_box_voice" / "prompts" / "skills" / "music_skill_prompt.txt"


def test_foxdot_init_uses_distinct_placeholder_guard_and_no_pathname_exists() -> None:
    content = FOXDOT_INIT_PATH.read_text(encoding="utf-8")

    assert "__RENARDO_SCLANG_DIR_PLACEHOLDER__" in content
    assert "renardoSynthDir == renardoSynthDirPlaceholder" in content
    assert ".exists" not in content


def test_foxdot_init_preloads_pianovel_for_runtime_safe_piano_usage() -> None:
    content = FOXDOT_INIT_PATH.read_text(encoding="utf-8")

    assert '"pianovel"' in content


def test_start_voice_assistant_validates_pianovel_startup_health() -> None:
    content = START_VOICE_ASSISTANT_PATH.read_text(encoding="utf-8")

    assert "--critical-synth pianovel" in content


def test_master_prompt_bans_extra_players_and_random_effect_samples() -> None:
    content = MASTER_PROMPT_PATH.read_text(encoding="utf-8")

    assert "NEVER use d4, d5, p4, p5" in content
    assert 'NEVER use `play("k"' in content or 'NEVER use "k"' in content
    assert "spack=1" in content
    assert 'NEVER invent sample letters like "A"' in content or 'NEVER invent sample letters like `A`' in content
    assert 'search_samples("kick", case="upper")' in content or "search_samples('kick', case='upper')" in content


def test_music_skill_prompt_bans_extra_players_and_random_effect_samples() -> None:
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")

    assert "NEVER use d4, d5, p4, p5" in content
    assert 'NEVER use `play("k"' in content or 'NEVER use "k"' in content
    assert "spack=1" in content
    assert 'NEVER invent sample letters like "A"' in content or 'NEVER invent sample letters like `A`' in content
    assert 'search_samples("kick", case="upper")' in content or "search_samples('kick', case='upper')" in content