"""degrade.py: режим команд по возможностям и порядок провайдеров (ADR-0150 §4.5, §8; #3686)."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import pytest

from rob_box_dialog.degrade import MODE_ORDER, ProviderState, provider_order, select_mode


@dataclass(frozen=True)
class Caps:
    tools: bool = False
    tool_choice: bool = False
    json_mode: bool = False
    structured_output: bool = False
    thinking: bool = False


@pytest.mark.parametrize(
    "caps, expected",
    [
        (Caps(structured_output=True, tool_choice=True, tools=True, json_mode=True), "structured_output"),
        (Caps(tool_choice=True, tools=True, json_mode=True), "tool_choice"),
        (Caps(tool_choice=True, tools=False, json_mode=True), "json_mode"),  # tool_choice без tools — не режим
        # #3798: модель в режиме размышлений отвечает 400 на tool_choice="required" — команда JSON-ом
        (Caps(tool_choice=True, tools=True, json_mode=True, thinking=True), "json_mode"),
        (Caps(tool_choice=True, tools=True, thinking=True), "json_in_text"),
        (Caps(json_mode=True, tools=True), "json_mode"),
        (Caps(tools=True), "json_in_text"),
        (Caps(), "json_in_text"),
    ],
)
def test_select_mode_follows_declared_capabilities(caps, expected):
    assert select_mode(caps) == expected


def test_mode_order_matches_adr_section_4_5():
    assert MODE_ORDER == ("structured_output", "tool_choice", "json_mode", "json_in_text")


def test_provider_order_keeps_priority_and_prefers_healthy():
    states = [
        ProviderState("minimax", "llm", "degraded"),
        ProviderState("deepseek", "llm", "ok"),
    ]
    assert provider_order(states, ["minimax", "deepseek"]) == ("deepseek", "minimax")


def test_provider_order_drops_exhausted_and_dead():
    states = [
        ProviderState("minimax", "llm", "quota", last_error_code="2056"),
        ProviderState("deepseek", "llm", "auth"),
        ProviderState("mimo", "llm", "down"),
    ]
    assert provider_order(states, ["minimax", "deepseek", "mimo"]) == ()


def test_provider_order_unknown_provider_is_assumed_ok_and_other_kinds_ignored():
    states = [ProviderState("deepseek", "tts", "down")]  # TTS-состояние не касается LLM
    assert provider_order(states, ["deepseek", "minimax"]) == ("deepseek", "minimax")


def test_dialog_package_does_not_import_adapters():
    """ADR §4.5: пакет не знает адаптеров. Эквивалент `grep -rn "providers\\." src/rob_box_dialog/rob_box_dialog` = 0."""
    root = Path(__file__).resolve().parents[1] / "rob_box_dialog"
    pattern = re.compile(r"providers\.")
    hits = [
        f"{p.name}:{n}"
        for p in root.glob("*.py")
        for n, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1)
        if pattern.search(line)
    ]
    assert hits == []
