"""Гард #3298: промпт ТАРС не расходится с реальным реестром инструментов.

Промпт велел звать ``say`` и ``set_voice_preset``, которых в реестре не было, и молчал про ``show_metrics`` /
``get_perception_context``. С PR-13b (#3708) список инструментов в промпте строит код
(:mod:`rob_box_supervisor.operator_prompt`) из реестра оператора ``operator_visible_tools()``; файл промпта держит
только правила и ни одного имени тула — второй копии знания нет.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from rob_box_core.tool_catalog import operator_visible_tools, tool_names
from rob_box_supervisor.metrics_source import (
    METRIC_DICTIONARY,
    QUERY_ALIASES,
    metric_dictionary_text,
)
from rob_box_supervisor.operator_prompt import CONFIRM_RULE, TOOLS_HEADER, operator_prompt

_PROMPT = (
    Path(__file__).resolve().parents[2] / "prompts" / "operator_system_prompt.txt"
)
_WORD_RE = re.compile(r"[a-z][a-z0-9_]*")


def _rules() -> str:
    return _PROMPT.read_text(encoding="utf-8")


def _registry() -> set[str]:
    return {e.name for e in operator_visible_tools()}


def _named(text: str) -> set[str]:
    return set(_WORD_RE.findall(text)) & set(tool_names())


def test_rules_file_names_no_tool() -> None:
    """Имена тулов — только в разделе, который строит код."""
    assert _named(_rules()) == set()


@pytest.mark.parametrize("engine", ["v1", "v2"])
def test_rendered_prompt_names_only_registry_tools(engine: str) -> None:
    prompt = operator_prompt(_rules(), engine)
    assert prompt.startswith(_rules().strip()) and TOOLS_HEADER in prompt
    assert _named(prompt) <= _registry()
    assert CONFIRM_RULE[engine] in prompt


def test_every_operator_only_tool_is_described() -> None:
    """``show_metrics``, ``say`` — у каждого тула «только у оператора» строка с описанием."""
    prompt = operator_prompt(_rules())
    own = {e.name for e in operator_visible_tools() if not e.llm_visible}
    assert {"say", "show_metrics", "read_logs", "ros2_node_status", "container_status"} <= own
    for name in own:
        assert f"- {name} — " in prompt


def test_say_is_the_aloud_tool_and_speak_text_is_banned() -> None:
    prompt = operator_prompt(_rules())
    assert "вслух людям рядом — say" in prompt
    assert "speak_text — инструмент личности, его не вызывай" in prompt


def test_prompt_carries_metric_dictionary_and_stale_failure_rule() -> None:
    prompt = operator_prompt(_rules())
    assert metric_dictionary_text() in prompt
    assert "в этом ходе" in prompt.lower() and "журнал" in prompt.lower()


def test_empty_rules_give_no_prompt() -> None:
    assert operator_prompt("  ") == ""


@pytest.mark.parametrize(
    ("alias", "needle"),
    [
        ("cpu", "process_cpu_seconds_total"),
        ("процессор", "process_cpu_seconds_total"),
        ("memory", "process_resident_memory_bytes"),
        ("память", "process_resident_memory_bytes"),
        ("llm_latency", "voice_llm_request_duration_seconds"),
        ("stt_latency", "voice_stt_recognize_duration_seconds"),
        ("tts_latency", "voice_tts_synthesize_duration_seconds"),
        ("barge_in", "voice_barge_in_total"),
        ("перебивания", "voice_barge_in_total"),
        ("voice_confidence", "voice_speaker_recognize_confidence"),
        ("уверенность голоса", "voice_speaker_recognize_confidence"),
        ("services_up", "up"),
        ("статус сервисов", "up"),
    ],
)
def test_show_metrics_aliases_resolve(alias: str, needle: str) -> None:
    assert needle in QUERY_ALIASES[alias]


def test_cpu_alias_is_percent_of_a_rate_not_cumulative_seconds() -> None:
    expr = QUERY_ALIASES["cpu"]
    assert expr.startswith("100 * rate(") and "[5m]" in expr


@pytest.mark.parametrize(
    "alias", ["llm_latency", "stt_latency", "tts_latency", "voice_confidence"]
)
def test_latency_aliases_are_mean_via_sum_over_count(alias: str) -> None:
    expr = QUERY_ALIASES[alias]
    assert "_sum[5m]" in expr and "_count[5m]" in expr and " / " in expr


def test_dictionary_text_lists_every_primary_alias() -> None:
    text = metric_dictionary_text()
    for key, *_rest in METRIC_DICTIONARY:
        assert f"'{key}'" in text
