"""Спецификации агентов на общем движке (ADR-0150 §9)."""

from rob_box_core.utterance import Sink
from rob_box_dialog import knowledge as kn
from rob_box_dialog.agents import AGENT_SPECS, GRAMMAR_GROUPS, agent_spec, tools_for


def test_one_spec_per_agent():
    assert set(AGENT_SPECS) == set(kn.AGENTS)
    for name, spec in AGENT_SPECS.items():
        assert spec.name == name and set(spec.grammar) <= set(GRAMMAR_GROUPS)


def test_silence_grammar_is_only_for_personality():
    """Решение О2: «хватит» действует на диалог Личности, а не на телеграм и ТАРС."""
    assert [n for n, s in AGENT_SPECS.items() if "silence" in s.grammar] == ["personality"]


def test_default_sinks():
    assert agent_spec("personality").sink is Sink.SPEAKERS
    assert agent_spec("operator").sink is Sink.HEADSET
    assert agent_spec("telegram").sink is None


def test_tools_view_respects_action_classes():
    personality, operator = tools_for(agent_spec("personality")), tools_for(agent_spec("operator"))
    assert "say" not in personality and "say" in operator
    assert "speak_text" not in personality  # транспорт речи, не действие (§4.4)
    assert "memory_save" in personality and "memory_save" in tools_for(agent_spec("telegram"))
