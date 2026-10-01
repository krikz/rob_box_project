"""Гард #3298: промпт ТАРС не расходится с реальным реестром инструментов.

Промпт велел звать ``say`` и ``set_voice_preset``, которых в реестре
оператора нет (``say`` скрыт от LLM: ``llm_visible=False``), и молчал про
``show_metrics`` / ``get_perception_context``. Модель либо вызывала
несуществующее, либо отвечала «инструмента нет» про существующее.

Каталог берём там же, где реестр оператора: ``llm_visible`` записи
``rob_box_core.tool_catalog`` (их же предзаполняет ``ToolRegistry``) плюс
два инструмента, которые добавляются не из каталога: ``show_metrics``
(``TarsPanelDispatcher.register_tool``) и ``load_skill`` (``AgentCore``).
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from rob_box_core.tool_catalog import llm_visible_tools, tool_names
from rob_box_supervisor.metrics_source import (
    METRIC_DICTIONARY,
    QUERY_ALIASES,
    metric_dictionary_text,
)

_PROMPT = (
    Path(__file__).resolve().parents[2] / "prompts" / "operator_system_prompt.txt"
)

#: Не из каталога: регистрируются кодом супервизора / ядра агента.
_EXTRA_OPERATOR_TOOLS = {"show_metrics", "load_skill"}

_DOUBLE_BACKTICK_RE = re.compile(r"``([^`]+)``")
_TOOL_NAME_RE = re.compile(r"([a-z][a-z0-9_]*)(?:\(.*\))?")


def _operator_registry() -> set[str]:
    return {e.name for e in llm_visible_tools()} | _EXTRA_OPERATOR_TOOLS


def _prompt_tool_names() -> set[str]:
    """Имена в двойных бэктиках; ``operator.admin`` и т.п. (с точкой) — не тулы."""
    names: set[str] = set()
    for token in _DOUBLE_BACKTICK_RE.findall(_PROMPT.read_text(encoding="utf-8")):
        match = _TOOL_NAME_RE.fullmatch(token.strip())
        if match:
            names.add(match.group(1))
    return names


def test_prompt_mentions_tools() -> None:
    """Гард не вырождается: в промпте есть что проверять."""
    assert {"show_metrics", "get_perception_context"} <= _prompt_tool_names()


def test_every_tool_named_in_prompt_is_in_operator_registry() -> None:
    missing = _prompt_tool_names() - _operator_registry()
    assert not missing, (
        f"промпт ТАРС называет инструменты, которых нет в реестре оператора: "
        f"{sorted(missing)}"
    )


def test_prompt_names_no_hidden_or_gone_tools() -> None:
    """``say`` есть в каталоге, но скрыт от LLM — упоминать его нельзя."""
    named = _prompt_tool_names()
    hidden = {n for n in tool_names() if n not in _operator_registry()}
    assert not (named & hidden)
    for gone in ("say", "set_voice_preset", "set_voice_language", "preview_voice"):
        assert gone not in named


def test_prompt_keeps_speak_text_ban_and_stale_failure_rule() -> None:
    text = _PROMPT.read_text(encoding="utf-8")
    assert "Никогда не вызывай ``speak_text``" in text
    assert "в этом ходе" in text.lower()
    assert "журнал" in text.lower()


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
