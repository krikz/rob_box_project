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
COMPOSER_PROMPT_PATH = REPO_ROOT / "src" / "rob_box_voice" / "prompts" / "skills" / "composer.txt"


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


def test_start_voice_assistant_reports_honest_failure_not_degraded_but_usable() -> None:
    """Issue #2716: a critical SynthDef that never confirmed into scsynth
    means the preset built on it plays silently, not "a bit worse". Calling
    that "non-critical errors (degraded but usable)" hid the failure behind
    a reassuring label and let it ride along in deploy auto-reports for
    weeks (#2693, #2707) instead of getting looked at.
    """
    content = START_VOICE_ASSISTANT_PATH.read_text(encoding="utf-8")

    assert "degraded but usable" not in content
    assert "non-critical errors" not in content
    assert "Music stack validation FAILED" in content
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


# ── issue #1810 — the music skill could not look anything up ─────────────
#
# The skill prompt told the model to research artists with
# ``search_artist_style(...)`` and to pick samples with
# ``renardo_search_samples(...)``. Neither tool has ever existed: the real
# names are ``search_web`` and ``search_samples``. ``_validate_tools_in_prompt``
# did not catch it — it only warns about registered tools *missing* from the
# prompt, never about invented ones present in it. So the model was told to
# call a tool that would fail, and fell back to inventing a melody instead of
# looking one up. These tests pin the real names down.

_PHANTOM_TOOL_NAMES = (
    "search_artist_style",
    "renardo_search_samples",
    "renardo_list_tracks",
    "renardo_save_track",
    "renardo_load_track",
    "renardo_delete_track",
)


def test_composer_prompt_names_no_unregistered_tools() -> None:
    """#1810 — every tool the prompt orders must actually be registered."""
    content = COMPOSER_PROMPT_PATH.read_text(encoding="utf-8")

    for phantom in _PHANTOM_TOOL_NAMES:
        assert phantom not in content, (
            f"composer.txt orders '{phantom}', which is not a "
            f"registered MCP tool (see _tool_catalog_data.py). The LLM will "
            f"get a tool-not-found error and fall back to prose or to "
            f"inventing music."
        )


def test_search_web_description_does_not_ban_music_research() -> None:
    """#1810 — the tool's own description used to wave the music skill off.

    «НЕ используй для музыкального ресёрча» applied to sample picking, but
    the model read it as "not for music", which is the opposite of what the
    melody-lookup path needs.
    """
    catalog_path = (
        REPO_ROOT / "src" / "rob_box_core" / "rob_box_core" / "_tool_catalog_data.py"
    )
    content = catalog_path.read_text(encoding="utf-8")
    start = content.index("'name': 'search_web'")
    entry = content[start:start + 2000]

    assert "музыкального ресёрча" not in entry, (
        "search_web still tells the LLM not to use it for music research"
    )
    assert "ноты" in entry
