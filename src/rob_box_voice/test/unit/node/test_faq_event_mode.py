from pathlib import Path
from unittest.mock import MagicMock

import pytest

from rob_box_voice.dialogue_node import DialogueNode


def _make_node(parameters: dict | None = None) -> DialogueNode:
    values = parameters or {}
    node = object.__new__(DialogueNode)
    logger = MagicMock()
    node._logger = logger
    node.get_logger = lambda: logger
    node.get_parameter = lambda name: type(
        "Parameter", (), {"value": values.get(name)}
    )()
    node._faq_store = None
    node._event_profile = None
    return node


def test_load_event_profile_from_yaml(tmp_path: Path) -> None:
    config_path = tmp_path / "event_mode.yaml"
    config_path.write_text(
        "event:\n"
        '  name: "День открытых дверей 2026"\n'
        '  organization: "Президентская академия"\n'
        '  location: "Москва"\n'
        '  date: "2026-04-12"\n'
        '  description: "Презентация бакалавриата"\n'
        '  robot_role: "РОББОКС — ровер-помощник"\n'
        f'  faq_file: "{tmp_path / "faq.xlsx"}"\n',
        encoding="utf-8",
    )

    node = _make_node(
        {
            "faq_mode_enabled": True,
            "faq_event_config_file": str(config_path),
        }
    )

    profile = node._load_event_profile()

    assert profile["name"] == "День открытых дверей 2026"
    assert profile["organization"] == "Президентская академия"
    assert profile["robot_role"] == "РОББОКС — ровер-помощник"


def test_render_event_instructions_includes_role_and_context() -> None:
    node = _make_node()
    node._event_profile = {
        "name": "День открытых дверей 2026",
        "organization": "Президентская академия",
        "location": "Москва",
        "date": "2026-04-12",
        "description": "Презентация бакалавриата",
        "robot_role": "РОББОКС — ровер-помощник",
    }

    rendered = node._render_event_instructions("BASE PROMPT")

    assert "РОББОКС — ровер-помощник" in rendered
    assert "День открытых дверей 2026" in rendered
    assert rendered.endswith("BASE PROMPT")


def test_build_skills_adds_faq_tool_when_event_mode_ready(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import rob_box_voice.dialogue_node as dialogue_node_module

    class FakeSkill:
        def __init__(self, *args, **kwargs):
            pass

        def as_tool(self, tool_name: str, tool_description: str):
            return tool_name

    monkeypatch.setattr(dialogue_node_module, "MusicSkill", FakeSkill)
    monkeypatch.setattr(dialogue_node_module, "NavigationSkill", FakeSkill)
    monkeypatch.setattr(dialogue_node_module, "MemorySkill", FakeSkill)
    monkeypatch.setattr(dialogue_node_module, "StatusSkill", FakeSkill)
    monkeypatch.setattr(dialogue_node_module, "FAQSkill", FakeSkill, raising=False)

    node = _make_node()
    node._mcp = MagicMock()
    node._faq_store = MagicMock()
    node._event_profile = {
        "name": "День открытых дверей 2026",
        "robot_role": "РОББОКС — ровер-помощник",
    }
    node._load_prompt_file = lambda _: "prompt"

    tools = node._build_skills(model=MagicMock())

    assert "handle_faq" in tools
