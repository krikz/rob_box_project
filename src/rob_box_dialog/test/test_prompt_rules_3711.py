"""Правила промпта агентов — таблица знания, а не текст файла (#3711, PR-15 15-rule; ADR-0148).

Проверяется поведение модуля: что правила у агентов есть и приходят в порядке таблицы, что тулы, которые правила
называют, у агента правда есть, а числа выступления согласованы с пределами схемы команды. Текст промпта здесь не
сверяется.
"""

import re

import pytest

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import knowledge as kn
from rob_box_dialog.agents import agent_spec, tools_for

CATALOG = {t.name for t in TOOL_CATALOG}
#: Имя в обратных кавычках, за которым идёт вызов (``name(`` / ``name=``) или конец кавычек.
_BACKTICKED = re.compile(r"`([a-z][a-z_]*)(?:\(|`)")


def _named(agent):
    return {m for rule in kn.PROMPT_RULES if agent in rule.agents for m in _BACKTICKED.findall(rule.text)}


@pytest.mark.parametrize("agent", [kn.AGENTS[0], kn.TEXT_CHANNEL_AGENT])
def test_agent_gets_its_rules_in_table_order(agent):
    texts = [rule.text for rule in kn.PROMPT_RULES if agent in rule.agents]
    block = kn.prompt_rules(agent)
    assert texts and block.startswith("# RULES\n")
    assert block.splitlines()[1:] == [f"- {t}" for t in texts]


def test_agent_without_rules_gets_no_block():
    """Оператор (ТАРС) на свой реестр промптов: правил таблицы у него нет, блока в промпте нет."""
    assert kn.AGENTS[1] == "operator" and kn.prompt_rules("operator") == ""
    assert kn.prompt_rules("nobody") == ""


def test_rule_belongs_to_known_agents_and_is_not_repeated_for_one_agent():
    seen = set()
    for rule in kn.PROMPT_RULES:
        assert rule.agents and set(rule.agents) <= set(kn.AGENTS)
        for agent in rule.agents:
            assert (rule.key, agent) not in seen, f"правило {rule.key} дважды у {agent}"
            seen.add((rule.key, agent))


@pytest.mark.parametrize("agent", [kn.AGENTS[0], kn.TEXT_CHANNEL_AGENT])
def test_tools_named_by_rules_exist_for_the_agent(agent):
    """Правило, которое зовёт тул, которого у агента нет, учит модель выдумке (#1810): тулы правил — из его среза."""
    tools = {n for n in _named(agent) if n in CATALOG}
    assert tools, "правила не называют ни одного тула"
    assert tools <= set(tools_for(agent_spec(agent)))
    assert not _named(agent) & {"say", "speak_text"}  # транспорт речи и тул оператора правилам не нужны


def test_chat_agent_rule_calls_the_aloud_tool_the_executor_knows():
    assert kn.ALOUD_TOOL in _named(kn.TEXT_CHANNEL_AGENT)
    assert kn.ALOUD_TOOL in tools_for(agent_spec(kn.TEXT_CHANNEL_AGENT))


def test_performance_length_advice_fits_the_command_schema():
    """Совет о длине анекдота и сказки — внутри пределов схемы команды: правило не просит невалидного."""
    assert kn.SEGMENTS_MIN <= kn.JOKE_SEGMENTS_MIN <= kn.SEGMENTS_MAX
    low, high = kn.TALE_SEGMENTS
    assert kn.SEGMENTS_MIN <= low <= high <= kn.SEGMENTS_MAX
    text = " ".join(rule.text for rule in kn.PROMPT_RULES if rule.key == "perform")
    assert str(kn.JOKE_SEGMENTS_MIN) in text and f"{low}–{high}" in text
