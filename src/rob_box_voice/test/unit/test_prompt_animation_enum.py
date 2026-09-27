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


_MASTER_PROMPT = Path(__file__).resolve().parents[2] / "prompts" / "master_prompt_compact.txt"
_SKILLS_DIR = Path(__file__).resolve().parents[2] / "prompts" / "skills"


def _enum_line() -> str:
    return (
        "**`speak_text.animation` enum (use ONLY one of these values):** "
        + ", ".join(f"`{animation}`" for animation in VALID_SPEAK_TEXT_ANIMATIONS)
        + "."
    )


def test_speak_text_animation_enum_lives_only_in_the_master_prompt() -> None:
    """Единственный источник enum анимаций — мастер-промпт (§4 TOOL REFERENCE).

    Раньше эта строка дублировалась в ``music_skill_prompt.txt`` и сверялась
    «идентична в двух файлах» — то есть тест требовал ДУБЛЬ. После удаления
    мёртвого файла инвариант строже: enum обязан быть в мастер-промпте и не
    должен дублироваться ни в одном скилл-файле — вторая копия это ровно тот
    дрейф, ради которого писался исходный тест.
    """
    enum_line = _enum_line()

    master = _MASTER_PROMPT.read_text(encoding="utf-8")
    assert enum_line in master, "Missing animation enum in master_prompt_compact.txt"

    for skill_path in sorted(_SKILLS_DIR.glob("*.txt")):
        content = skill_path.read_text(encoding="utf-8")
        assert enum_line not in content, (
            f"{skill_path.name} дублирует enum анимаций — единственный "
            f"источник это master_prompt_compact.txt"
        )


def test_speak_text_animation_enum_does_not_suggest_unregistered_styles() -> None:
    """The prompt must not teach the LLM arbitrary animation/style names."""
    enum_line = _enum_line()

    content = _MASTER_PROMPT.read_text(encoding="utf-8")
    enum_start = content.index(enum_line)
    enum_end = enum_start + len(enum_line)
    assert "hiphop" not in content[enum_start:enum_end]
    assert "rap" not in content[enum_start:enum_end]
    assert "sing" not in content[enum_start:enum_end]
