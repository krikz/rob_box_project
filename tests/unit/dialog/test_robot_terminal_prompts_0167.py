"""Промпты чата Telegram и ТАРС робота = основа ядра + профиль робота, байт в байт как до переноса (ADR-0167 Ш1б-4).

До Ш1б-4 промпт чата лежал целиком в ``src/rob_box_voice/prompts/telegram_chat_prompt.txt``, правила ТАРС - в
``src/rob_box_supervisor/prompts/operator_system_prompt.txt`` (оба с именами). Теперь основы без имён - данные пакета
ядра (``rob_box_dialog/prompts/telegram.txt``, ``operator.txt``), имя продукта - ``name`` профиля робота, персона ТАРС -
``docker/sound/config/operator_persona.txt``. Эталоны - текст, который ``load_system_prompt`` отдавал из старых файлов
на develop e8bfb47 (``fixtures/*_develop_e8bfb47.txt``): поведение робота не меняется.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from rob_box_dialog.profile import load_profile
from rob_box_dialog.prompts import operator_base, operator_rules, telegram_base, telegram_prompt

REPO = Path(__file__).resolve().parents[3]
CONFIG = REPO / "docker" / "sound" / "config"
FIXTURES = Path(__file__).resolve().parent / "fixtures"
GOLDEN_TELEGRAM = FIXTURES / "telegram_prompt_develop_e8bfb47.txt"
GOLDEN_OPERATOR = FIXTURES / "operator_rules_develop_e8bfb47.txt"
DIALOG_NODE = REPO / "src" / "rob_box_voice" / "rob_box_voice" / "dialog_node.py"
OPERATOR_TURNS = REPO / "src" / "rob_box_supervisor" / "rob_box_supervisor" / "operator_turns.py"
SUPERVISOR_NODE = REPO / "src" / "rob_box_supervisor" / "rob_box_supervisor" / "supervisor_node.py"
YAMLS = (
    CONFIG / "voice_assistant" / "dialog_node.yaml",
    REPO / "src" / "rob_box_voice" / "config" / "dialog_node.yaml",
)

pytestmark = pytest.mark.skipif(not (CONFIG / "assistant.yaml").is_file(), reason="нет docker/ (без профиля робота)")


def _profile():
    return load_profile(CONFIG)


def test_robot_telegram_prompt_is_byte_for_byte_the_old_file():
    golden = GOLDEN_TELEGRAM.read_text(encoding="utf-8")
    assert telegram_prompt(_profile().name).encode("utf-8") == golden.encode("utf-8")


def test_robot_operator_rules_are_byte_for_byte_the_old_file():
    golden = GOLDEN_OPERATOR.read_text(encoding="utf-8")
    assert operator_rules(_profile().operator_persona).encode("utf-8") == golden.encode("utf-8")


def test_full_prompts_are_built_on_the_same_rules():
    """Мост дописывает правила знания к промпту чата, хост ТАРС - раздел инструментов к правилам - как к старым
    файлам."""
    ports = pytest.importorskip("rob_box_voice.core.dialog_v2_ports", reason="мосту нужен rob_box_llm")
    full = ports.system_prompt(telegram_prompt(_profile().name), "telegram")
    assert full.startswith(GOLDEN_TELEGRAM.read_text(encoding="utf-8") + "\n\n")
    op = pytest.importorskip("rob_box_supervisor.operator_prompt", reason="нужен rob_box_supervisor")
    assert op.operator_prompt(operator_rules(_profile().operator_persona)).startswith(
        GOLDEN_OPERATOR.read_text(encoding="utf-8") + "\n\n" + op.TOOLS_HEADER
    )


def test_names_live_only_in_the_robot_profile():
    assert "РОББОКС" not in telegram_base() and "ТАРС" not in operator_base()
    profile = _profile()
    assert profile.name == "РОББОКС"
    assert "ТАРС" in profile.operator_persona.identity


def test_old_prompt_files_and_params_are_gone():
    """Две реализации не держим: файлов промптов у хостов и параметров, которые на них указывали, больше нет."""
    assert not (REPO / "src" / "rob_box_voice" / "prompts" / "telegram_chat_prompt.txt").exists()
    assert not (REPO / "src" / "rob_box_supervisor" / "prompts" / "operator_system_prompt.txt").exists()
    for yaml_file in YAMLS:
        assert "telegram_system_prompt_file" not in yaml_file.read_text(encoding="utf-8"), yaml_file
    for host in (DIALOG_NODE, SUPERVISOR_NODE):
        text = host.read_text(encoding="utf-8-sig")
        assert "telegram_chat_prompt" not in text and "operator_system_prompt" not in text, host


def _calls(path: Path):
    return [n for n in ast.walk(ast.parse(path.read_text(encoding="utf-8-sig"))) if isinstance(n, ast.Call)]


def test_hosts_build_the_prompts_from_the_profile():
    telegram = [ast.unparse(n.args[0]) for n in _calls(DIALOG_NODE) if ast.unparse(n.func) == "telegram_prompt"]
    assert telegram == ["node.product_profile.name"]
    operator = [ast.unparse(n.args[0]) for n in _calls(OPERATOR_TURNS) if ast.unparse(n.func) == "operator_rules"]
    assert operator == ["profile.operator_persona"]
    assert not [n for n in _calls(OPERATOR_TURNS) if ast.unparse(n.func) == "load_system_prompt"]
