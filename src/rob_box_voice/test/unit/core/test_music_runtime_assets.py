"""Regression tests for music runtime startup assets and prompt guardrails."""

from pathlib import Path


# Resolve repo root by walking up the tree until we find the ``docker/`` and
# ``src/`` siblings. This avoids brittle parents[N] indexing that breaks when
# the test is copied into a colcon workspace (test_ws/src/rob_box_voice/...).
def _resolve_repo_root(start: Path) -> Path:
    for parent in [start, *start.parents]:
        if (parent / "docker").is_dir() and (parent / "src").is_dir():
            return parent
    # Fallback: original semantics (5 parents up from this test file).
    return start.parents[5]


_THIS_FILE = Path(__file__).resolve()
REPO_ROOT = _resolve_repo_root(_THIS_FILE)
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
    assert "one execute_music_code" in content.lower() or "single execute_music_code" in content.lower()
    assert "do not add a second atmospheric pass" in content.lower() or "do not send a second atmospheric pass" in content.lower()
    assert "stay within d1 and p1-p3" in content.lower() or "do not use p4 or d4" in content.lower()
    assert "dur=0.25" in content.lower()


def test_music_skill_prompt_contains_stranger_things_structure_guidance() -> None:
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")

    assert "Stranger Things" in content
    assert "strangerpulsepad" in content
    assert "strangerarp" in content
    assert "strangerbrass" in content
    assert "heartbeat" in content.lower()
    assert "fixed bass ostinato" in content.lower() or "deterministic bass ostinato" in content.lower()
    assert "same pitch sequence" in content.lower() or "reuse the same pitch sequence" in content.lower()
    assert "one execute_music_code" in content.lower() or "single execute_music_code" in content.lower()
    assert "do not add a second atmospheric pass" in content.lower() or "do not send a second atmospheric pass" in content.lower()
    assert "stay within d1 and p1-p3" in content.lower() or "do not use p4 or d4" in content.lower()
    assert "dur=0.25" in content.lower()


def test_master_prompt_bans_extra_players_and_random_effect_samples() -> None:
    content = MASTER_PROMPT_PATH.read_text(encoding="utf-8")

    assert "NEVER use d4, d5, p4, p5" in content
    assert 'NEVER use `play("k"' in content or 'NEVER use "k"' in content
    assert "spack=1" in content
    assert 'NEVER invent sample letters like "A"' in content or 'NEVER invent sample letters like `A`' in content
    assert 'search_samples("kick", case="upper")' in content or "search_samples('kick', case='upper')" in content


def test_master_prompt_contains_tb303_safety_guidance() -> None:
    content = MASTER_PROMPT_PATH.read_text(encoding="utf-8")

    assert "tb303" in content.lower()
    assert "attack=0.01" in content.lower()
    assert "crack-prone" in content.lower() or "click-prone" in content.lower()
    assert "do not combine tb303" in content.lower() or "never combine tb303" in content.lower()
    assert "crush" in content.lower()
    assert "bits" in content.lower()
    assert "echo" in content.lower()


def test_master_prompt_contains_imperial_march_sc_only_guidance() -> None:
    content = MASTER_PROMPT_PATH.read_text(encoding="utf-8")

    assert "Imperial March" in content or "Star Wars march" in content
    assert "imperialbrass" in content
    assert "marchstrings" in content
    assert "midinote" in content
    assert "first phrase alone is incomplete" in content.lower() or "first phrase alone" in content.lower()
    assert "bridge" in content.lower()
    assert "answer phrase" in content.lower() or "b answer phrase" in content.lower()
    assert "one execute_music_code" in content.lower() or "single execute_music_code" in content.lower()
    assert "do not use clock.future" in content.lower()
    assert "do not use p4 or d4" in content.lower() or "stay within d1 and p1-p3" in content.lower()
    assert "76,75,74,70,66,63,70,67" in content.replace(" ", "")
    assert "brass" in content.lower()
    assert "avoid organ" in content.lower() or "prefer strings over organ" in content.lower()
    assert "a -> a' -> bridge -> answer phrase" in content.lower() or "a -> a' -> bridge -> b answer" in content.lower()
    assert "exact midinote contour" in content.lower() or "preserve the exact contour" in content.lower()
    assert "do not use prand for the main melody" in content.lower() or "never use prand for the main melody" in content.lower()
    assert "folk instruments" in content.lower() or "народн" in content.lower()


def test_music_skill_prompt_bans_extra_players_and_random_effect_samples() -> None:
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")

    assert "NEVER use d4, d5, p4, p5" in content
    assert 'NEVER use `play("k"' in content or 'NEVER use "k"' in content
    assert "spack=1" in content
    assert 'NEVER invent sample letters like "A"' in content or 'NEVER invent sample letters like `A`' in content
    assert 'search_samples("kick", case="upper")' in content or "search_samples('kick', case='upper')" in content


def test_music_skill_prompt_contains_tb303_safety_guidance() -> None:
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")

    assert "tb303" in content.lower()
    assert "attack=0.01" in content.lower()
    assert "crack-prone" in content.lower() or "click-prone" in content.lower()
    assert "do not combine tb303" in content.lower() or "never combine tb303" in content.lower()
    assert "crush" in content.lower()
    assert "bits" in content.lower()
    assert "echo" in content.lower()
    assert "retrobass" in content.lower() or "wobblebass" in content.lower()


def test_music_skill_prompt_contains_imperial_march_sc_only_guidance() -> None:
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")

    assert "Imperial March" in content or "Star Wars march" in content
    assert "imperialbrass" in content
    assert "marchstrings" in content
    assert "midinote" in content
    assert "first phrase alone is incomplete" in content.lower() or "do not use the opening motif alone" in content.lower()
    assert "bridge" in content.lower()
    assert "answer phrase" in content.lower() or "b answer phrase" in content.lower()
    assert "one execute_music_code" in content.lower() or "single execute_music_code" in content.lower()
    assert "do not use clock.future" in content.lower()
    assert "do not use p4 or d4" in content.lower() or "stay within d1 and p1-p3" in content.lower()
    assert "76,75,74,70,66,63,70,67" in content.replace(" ", "")
    assert "brass" in content.lower()
    assert "avoid organ" in content.lower() or "prefer strings over organ" in content.lower()
    assert "a -> a' -> bridge -> answer phrase" in content.lower() or "a -> a' -> bridge -> b answer" in content.lower()
    assert "exact midinote contour" in content.lower() or "preserve the exact contour" in content.lower()
    assert "do not use prand for the main melody" in content.lower() or "never use prand for the main melody" in content.lower()
    assert "folk instruments" in content.lower() or "народн" in content.lower()
    assert "timbral remix example" in content.lower() or "folk-style remix example" in content.lower()
    assert "marimba" in content.lower()
    assert "karp" in content.lower()
    assert "same melody contour" in content.lower() or "do not rewrite the tune" in content.lower()

def test_master_prompt_mentions_estimate_tts_duration() -> None:
    """#949 AC4 — the LLM must know estimate_tts_duration exists."""
    content = MASTER_PROMPT_PATH.read_text(encoding="utf-8")

    assert "estimate_tts_duration" in content
    assert "segments" in content


def test_music_skill_prompt_mentions_estimate_tts_duration() -> None:
    """#949 AC4 — the music skill lists estimate_tts_duration as a tool."""
    content = MUSIC_SKILL_PROMPT_PATH.read_text(encoding="utf-8")

    assert "estimate_tts_duration(text)" in content
    assert "30 симв/с" in content or "30 chars" in content
