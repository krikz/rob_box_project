"""Regression tests for music runtime startup assets and prompt guardrails."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[5]
FOXDOT_INIT_PATH = REPO_ROOT / "docker" / "vision" / "voice_assistant" / "foxdot_init.sc"
START_VOICE_ASSISTANT_PATH = REPO_ROOT / "docker" / "vision" / "scripts" / "voice_assistant" / "start_voice_assistant.sh"
CUSTOM_SYNTHDEF_DIR = REPO_ROOT / "docker" / "vision" / "voice_assistant" / "custom_synthdefs"
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


def test_foxdot_init_preloads_sc_only_custom_synthdefs_for_stranger_things_palette() -> None:
    content = FOXDOT_INIT_PATH.read_text(encoding="utf-8")

    assert '"warmpad"' in content
    assert '"retrobass"' in content
    assert '"supersawlead"' in content
    assert '/ws/custom_synthdefs' in content


def test_foxdot_init_preloads_imperial_march_sc_only_custom_synthdefs() -> None:
    content = FOXDOT_INIT_PATH.read_text(encoding="utf-8")

    assert '"imperialbrass"' in content
    assert '"marchstrings"' in content


def test_foxdot_init_preloads_expanded_stranger_things_sc_only_custom_synthdefs() -> None:
    content = FOXDOT_INIT_PATH.read_text(encoding="utf-8")

    assert '"strangerpulsepad"' in content
    assert '"strangerarp"' in content
    assert '"strangerbrass"' in content


def test_start_voice_assistant_validates_pianovel_startup_health() -> None:
    content = START_VOICE_ASSISTANT_PATH.read_text(encoding="utf-8")

    assert "--critical-synth pianovel" in content


def test_start_voice_assistant_validates_sc_only_custom_synthdefs_startup_health() -> None:
    content = START_VOICE_ASSISTANT_PATH.read_text(encoding="utf-8")

    assert "--critical-synth warmpad" in content
    assert "--critical-synth retrobass" in content
    assert "--critical-synth supersawlead" in content


def test_start_voice_assistant_validates_imperial_march_sc_only_custom_synthdefs() -> None:
    content = START_VOICE_ASSISTANT_PATH.read_text(encoding="utf-8")

    assert "--critical-synth imperialbrass" in content
    assert "--critical-synth marchstrings" in content


def test_start_voice_assistant_validates_expanded_stranger_things_sc_only_custom_synthdefs() -> None:
    content = START_VOICE_ASSISTANT_PATH.read_text(encoding="utf-8")

    assert "--critical-synth strangerpulsepad" in content
    assert "--critical-synth strangerarp" in content
    assert "--critical-synth strangerbrass" in content


def test_sc_only_custom_synthdef_files_exist_for_repo_owned_palette() -> None:
    for synth_name in (
        "warmpad",
        "retrobass",
        "supersawlead",
        "imperialbrass",
        "marchstrings",
        "strangerpulsepad",
        "strangerarp",
        "strangerbrass",
    ):
        synth_path = CUSTOM_SYNTHDEF_DIR / f"{synth_name}.scd"
        assert synth_path.exists()
        content = synth_path.read_text(encoding="utf-8")
        assert f"SynthDef.new(\\{synth_name}" in content


def test_master_prompt_contains_stranger_things_structure_guidance() -> None:
    content = MASTER_PROMPT_PATH.read_text(encoding="utf-8")

    assert "Stranger Things" in content
    assert "strangerpulsepad" in content
    assert "strangerarp" in content
    assert "strangerbrass" in content
    assert "heartbeat" in content.lower()
    assert "fixed bass ostinato" in content.lower() or "deterministic bass ostinato" in content.lower()
    assert "same pitch sequence" in content.lower() or "reuse the same pitch sequence" in content.lower()


def test_music_skill_prompt_contains_stranger_things_structure_guidance() -> None:
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")

    assert "Stranger Things" in content
    assert "strangerpulsepad" in content
    assert "strangerarp" in content
    assert "strangerbrass" in content
    assert "heartbeat" in content.lower()
    assert "fixed bass ostinato" in content.lower() or "deterministic bass ostinato" in content.lower()
    assert "same pitch sequence" in content.lower() or "reuse the same pitch sequence" in content.lower()


def test_master_prompt_bans_extra_players_and_random_effect_samples() -> None:
    content = MASTER_PROMPT_PATH.read_text(encoding="utf-8")

    assert "NEVER use d4, d5, p4, p5" in content
    assert 'NEVER use `play("k"' in content or 'NEVER use "k"' in content
    assert "spack=1" in content
    assert 'NEVER invent sample letters like "A"' in content or 'NEVER invent sample letters like `A`' in content
    assert 'search_samples("kick", case="upper")' in content or "search_samples('kick', case='upper')" in content


def test_master_prompt_contains_imperial_march_sc_only_guidance() -> None:
    content = MASTER_PROMPT_PATH.read_text(encoding="utf-8")

    assert "Imperial March" in content or "Star Wars march" in content
    assert "imperialbrass" in content
    assert "marchstrings" in content
    assert "midinote" in content
    assert "first phrase alone is incomplete" in content.lower() or "first phrase alone" in content.lower()
    assert "bridge" in content.lower()
    assert "answer phrase" in content.lower() or "b answer phrase" in content.lower()


def test_music_skill_prompt_bans_extra_players_and_random_effect_samples() -> None:
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")

    assert "NEVER use d4, d5, p4, p5" in content
    assert 'NEVER use `play("k"' in content or 'NEVER use "k"' in content
    assert "spack=1" in content
    assert 'NEVER invent sample letters like "A"' in content or 'NEVER invent sample letters like `A`' in content
    assert 'search_samples("kick", case="upper")' in content or "search_samples('kick', case='upper')" in content


def test_music_skill_prompt_contains_imperial_march_sc_only_guidance() -> None:
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")

    assert "Imperial March" in content or "Star Wars march" in content
    assert "imperialbrass" in content
    assert "marchstrings" in content
    assert "midinote" in content
    assert "first phrase alone is incomplete" in content.lower() or "do not use the opening motif alone" in content.lower()
    assert "bridge" in content.lower()
    assert "answer phrase" in content.lower() or "b answer phrase" in content.lower()