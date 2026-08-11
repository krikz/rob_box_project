from pathlib import Path


VALID_SPEAK_TEXT_ANIMATIONS = (
    "idle",
    "talking",
    "wakeup",
    "sleep",
    "happy",
    "sad",
    "angry",
    "surprised",
    "thinking",
    "victory",
    "error",
    "low_battery",
    "charging",
    "police_lights",
    "ambulance",
    "fire_truck",
    "road_service",
    "turn_left",
    "turn_right",
    "accelerating",
    "braking",
    "neutral",
    "excited",
    "confused",
)


PROMPT_PATHS = (
    Path(__file__).resolve().parents[2] / "prompts" / "master_prompt_compact.txt",
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "skills"
    / "music_skill_prompt.txt",
)


def test_speak_text_animation_enum_is_explicit_in_all_llm_prompts() -> None:
    """Keep prompt guidance aligned with SpeakTextTool's schema enum."""
    enum_line = (
        "**`speak_text.animation` enum (use ONLY one of these values):** "
        + ", ".join(f"`{animation}`" for animation in VALID_SPEAK_TEXT_ANIMATIONS)
        + "."
    )

    for prompt_path in PROMPT_PATHS:
        content = prompt_path.read_text(encoding="utf-8")
        assert enum_line in content, f"Missing animation enum in {prompt_path}"


def test_speak_text_animation_enum_does_not_suggest_unregistered_styles() -> None:
    """The prompt must not teach the LLM arbitrary animation/style names."""
    enum_line = (
        "**`speak_text.animation` enum (use ONLY one of these values):** "
        + ", ".join(f"`{animation}`" for animation in VALID_SPEAK_TEXT_ANIMATIONS)
        + "."
    )

    for prompt_path in PROMPT_PATHS:
        content = prompt_path.read_text(encoding="utf-8")
        enum_start = content.index(enum_line)
        enum_end = enum_start + len(enum_line)
        assert "hiphop" not in content[enum_start:enum_end]
        assert "rap" not in content[enum_start:enum_end]
        assert "sing" not in content[enum_start:enum_end]
