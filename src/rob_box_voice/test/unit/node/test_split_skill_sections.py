"""Проводка фазы 5 в ноде: ``DialogueNode._split_skill_sections``.

Парсер разметки проверен отдельно (``rob_box_core/test/test_prompt_sections``),
а мастер-промпт — в ``test_prompt_skill_sections.py``. Здесь остаётся стык:
нода обязана

* при ``skills_enabled=false`` отдать промпт как раньше и НЕ трогать
  фрагменты (дефолтная конфигурация неотличима от доскилловой);
* при ``true`` вынуть доменные секции из системного промпта и приклеить их
  к фрагментам своих скиллов — чтобы AgentCore доставил их вплотную к
  текущему ходу;
* пережить сломанную разметку, не потеряв ни одного правила: контейнер на
  роботе должен подняться и говорить.

Не требует ROS2 — rclpy замокан в conftest.py.
"""

from unittest.mock import MagicMock

import pytest

from rob_box_voice.dialogue_node import DialogueNode

_PROMPT = "\n".join(
    [
        "шапка",
        "<<<SKILL-OFF>>>",
        "- «поезжай на кухню» → §6 WAYPOINTS.",
        "<<<SKILL-OFF-END>>>",
        "<<<SKILL-ON>>>",
        "- точки приезжают блоком скилла.",
        "<<<SKILL-ON-END>>>",
        "<<<SKILL-MOVE navigation>>>",
        "# 6. WAYPOINTS",
        "navigate_to_waypoint(«кухня»)",
        "<<<SKILL-MOVE-END>>>",
        "подвал",
        "",
    ]
)


def _node(*, skills_enabled: bool):
    node = object.__new__(DialogueNode)
    logger = MagicMock()
    node._logger = logger
    node.get_logger = lambda: logger
    node._skills_enabled = lambda: skills_enabled  # type: ignore[method-assign]
    return node


def _errors(node) -> list[str]:
    return [str(call.args[0]) for call in node.get_logger().error.call_args_list]


class TestSkillsDisabled:
    def test_sections_stay_in_the_system_prompt(self) -> None:
        prompt, fragments = _node(skills_enabled=False)._split_skill_sections(
            _PROMPT, {}
        )
        assert "# 6. WAYPOINTS" in prompt
        assert "navigate_to_waypoint" in prompt
        assert fragments == {}

    def test_markers_do_not_reach_the_llm(self) -> None:
        prompt, _ = _node(skills_enabled=False)._split_skill_sections(_PROMPT, {})
        assert "<<<" not in prompt

    def test_skills_on_text_is_not_shown(self) -> None:
        prompt, _ = _node(skills_enabled=False)._split_skill_sections(_PROMPT, {})
        assert "точки приезжают блоком скилла" not in prompt
        assert "§6 WAYPOINTS" in prompt


class TestSkillsEnabled:
    def test_sections_move_from_the_prompt_to_the_fragment(self) -> None:
        prompt, fragments = _node(skills_enabled=True)._split_skill_sections(
            _PROMPT, {"navigation": "[СКИЛЛ navigation]"}
        )
        assert "# 6. WAYPOINTS" not in prompt
        assert fragments["navigation"].startswith("[СКИЛЛ navigation]")
        assert "navigate_to_waypoint" in fragments["navigation"]

    def test_routing_stops_lying_about_removed_sections(self) -> None:
        prompt, _ = _node(skills_enabled=True)._split_skill_sections(_PROMPT, {})
        assert "§6 WAYPOINTS" not in prompt
        assert "точки приезжают блоком скилла" in prompt

    def test_missing_fragment_file_does_not_swallow_the_section(self) -> None:
        """Файла фрагмента нет — правила всё равно доезжают до LLM."""
        _, fragments = _node(skills_enabled=True)._split_skill_sections(_PROMPT, {})
        assert "navigate_to_waypoint" in fragments["navigation"]

    def test_other_fragments_are_untouched(self) -> None:
        _, fragments = _node(skills_enabled=True)._split_skill_sections(
            _PROMPT, {"memory": "текст memory"}
        )
        assert fragments["memory"] == "текст memory"


class TestBrokenMarkup:
    """Сломанная разметка не роняет ноду и не теряет правила."""

    broken = "шапка\n<<<SKILL-MOVE navigation>>>\n# 6. WAYPOINTS\n"

    def test_node_survives_and_keeps_the_rules(self) -> None:
        node = _node(skills_enabled=True)
        prompt, fragments = node._split_skill_sections(self.broken, {})
        assert "# 6. WAYPOINTS" in prompt
        assert fragments == {}

    def test_operator_sees_what_broke(self) -> None:
        node = _node(skills_enabled=True)
        node._split_skill_sections(self.broken, {})
        assert any("разметка секций" in msg.lower() for msg in _errors(node))


@pytest.mark.parametrize("enabled", [False, True])
def test_prompt_without_markup_passes_through(enabled: bool) -> None:
    """Старый промпт без разметки едет как есть в обоих режимах."""
    plain = "строка один\nстрока два\n"
    prompt, fragments = _node(skills_enabled=enabled)._split_skill_sections(
        plain, {"memory": "m"}
    )
    assert prompt == plain
    assert fragments == {"memory": "m"}
