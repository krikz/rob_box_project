"""Промпт Личности робота = основа ядра + персона профиля робота, байт в байт как до переноса (ADR-0167 Ш1б-3).

До Ш1б-3 промпт Личности лежал целиком в ``src/rob_box_voice/prompts/master_prompt_compact.txt`` (с именем робота).
Теперь основа без имени продукта - данные пакета ядра (``rob_box_dialog/prompts/personality.txt``), персона - в
профиле робота (``docker/sound/config/persona.txt``). Эталон - текст, который ``load_system_prompt`` отдавал из
старого файла на develop 4d9c774 (``fixtures/personality_prompt_develop_4d9c774.txt``): поведение робота не меняется.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

from rob_box_dialog.profile import ProfileError, load_profile
from rob_box_dialog.prompts import base, personality_prompt

REPO = Path(__file__).resolve().parents[3]
CONFIG = REPO / "docker" / "sound" / "config"
GOLDEN = Path(__file__).resolve().parent / "fixtures" / "personality_prompt_develop_4d9c774.txt"
DIALOG_NODE = REPO / "src" / "rob_box_voice" / "rob_box_voice" / "dialog_node.py"
YAMLS = (
    CONFIG / "voice_assistant" / "dialog_node.yaml",
    REPO / "src" / "rob_box_voice" / "config" / "dialog_node.yaml",
)

pytestmark = pytest.mark.skipif(not (CONFIG / "assistant.yaml").is_file(), reason="нет docker/ (без профиля робота)")


def _robot_prompt() -> str:
    return personality_prompt(load_profile(CONFIG).persona)


def test_robot_prompt_is_byte_for_byte_the_old_master_prompt():
    golden = GOLDEN.read_text(encoding="utf-8")
    assert _robot_prompt().encode("utf-8") == golden.encode("utf-8")


def test_robot_system_prompt_is_the_prompt_plus_knowledge_rules():
    """Мост дописывает правила знания к промпту так же, как к старому файлу: ``промпт + пустая строка + правила``."""
    ports = pytest.importorskip("rob_box_dialog_host.dialog_v2_ports", reason="мосту нужен rob_box_llm")
    full = ports.system_prompt(_robot_prompt(), "personality")
    assert full.startswith(GOLDEN.read_text(encoding="utf-8") + "\n\n")


def test_robot_name_lives_only_in_the_robot_persona():
    assert "РОББОКС" not in base()
    persona = load_profile(CONFIG).persona
    assert "РОББОКС" in persona.identity and "РОББОКС" in persona.character


def test_old_prompt_file_is_gone():
    """Две реализации не держим: файла промпта Личности у робота больше нет, параметра ноды - тоже."""
    assert not (REPO / "src" / "rob_box_voice" / "prompts" / "master_prompt_compact.txt").exists()
    for yaml_file in YAMLS:
        text = yaml_file.read_text(encoding="utf-8")
        assert "master_prompt_compact" not in text and "\n    system_prompt_file:" not in text, yaml_file


def test_dialog_node_builds_the_personality_prompt_from_the_profile():
    calls = [n for n in ast.walk(ast.parse(DIALOG_NODE.read_text(encoding="utf-8-sig"))) if isinstance(n, ast.Call)]
    declared = {
        n.args[0].value
        for n in calls
        if getattr(n.func, "attr", "") == "declare_parameter" and n.args and isinstance(n.args[0], ast.Constant)
    }
    # Ш1б-4: и у телеграма своего файла промпта нет (основа ядра с name профиля).
    assert "system_prompt_file" not in declared and "telegram_system_prompt_file" not in declared
    prompts = [ast.unparse(n.args[0]) for n in calls if ast.unparse(n.func) == "personality_prompt"]
    assert prompts == ["node.product_profile.persona"]


def test_stand_builds_the_same_prompt():
    pytest.importorskip("openai", reason="сухому стенду нужен настоящий мост команд, а ему - клиент openai")
    sys.path.insert(0, str(REPO / "scripts" / "dialog"))
    import dry_stand  # noqa: PLC0415

    assert dry_stand.robot_prompt() == _robot_prompt()


def test_robot_profile_without_persona_does_not_start(tmp_path):
    (tmp_path / "wake_words.yaml").write_text((CONFIG / "wake_words.yaml").read_text(encoding="utf-8"), "utf-8")
    (tmp_path / "assistant.yaml").write_text(
        "format: 4\nname: РОББОКС\nwake_words: wake_words.yaml\npersona: persona.txt\noperator_persona: o.txt\n"
        "owners: [speech]\n",
        encoding="utf-8",
    )
    with pytest.raises(ProfileError, match="файла персоны"):
        load_profile(tmp_path)
