"""adapter_benchmark.py: офлайн-часть (FakeLLMProvider; живые адаптеры без ключей не вызываются). #3686."""

from __future__ import annotations

import asyncio
import json
from collections import Counter

import pytest
from rob_box_dialog.command import ToolCall as CommandCall
from rob_box_llm.provider import LLMResponse, ProviderCapabilities, ToolCall
from rob_box_llm.providers.fake import FakeLLMProvider
from scripts.dialog import adapter_benchmark as ab


def _fake(payload: str) -> FakeLLMProvider:
    async def on_complete(msgs, tools, settings):
        return LLMResponse(content=payload)

    return FakeLLMProvider(on_complete=on_complete)


def test_counts_delivered_and_invalid_and_sums_to_calls():
    good = asyncio.run(ab.run_adapter(_fake(json.dumps({"speech": "Привет"})), "json_in_text", ["a", "b"], 2))
    assert (good["calls"], good["delivered"], good["invalid"]) == (4, 4, 0)
    bad = asyncio.run(ab.run_adapter(_fake("не json"), "json_in_text", ["a"], 3))
    assert (bad["calls"], bad["delivered"], bad["invalid"]) == (3, 0, 3)
    empty = asyncio.run(ab.run_adapter(_fake("{}"), "json_mode", ["a"], 1))
    assert (empty["delivered"], empty["invalid"]) == (0, 1)


def test_missing_key_is_reported_not_skipped(monkeypatch):
    for var in ab.KEY_ENV.values():
        monkeypatch.delenv(var, raising=False)
    assert ab.build_provider("deepseek") is None
    table = ab.render([("deepseek", "auto", None)], "rob_box_dialog.command.parse", 20)
    assert "не прогнано: нет ключа" in table


def test_phrases_exclude_tier1_and_respect_limit():
    data = ab.yaml.safe_load((ab.REPO / "scripts" / "dialog" / "reference_set.yaml").read_text(encoding="utf-8"))
    not_tier1 = [p["text"] for p in data["phrases"] if "tier1" not in p["expected"]]
    assert ab.load_phrases(0) == not_tier1 and len(not_tier1) < len(data["phrases"])
    assert len(ab.load_phrases(20)) == 20


# --- путь моста (--cells, #3698 ч.7d) ---------------------------------------------------------------------------


def _phrase(kind="speech_only", actions=()):
    return ab.Phrase("r00", "Робот, привет", kind, tuple(actions))


def test_bridge_tool_offered_sends_command_tool_without_tool_choice():
    seen = []

    async def on_complete(msgs, tools, settings):
        seen.append((tuple(t["function"]["name"] for t in tools), settings.tool_choice))
        return LLMResponse(content="", tool_calls=(ToolCall("c1", "command", {"speech": "Привет!"}),))

    fake = FakeLLMProvider(on_complete=on_complete, capabilities=ProviderCapabilities(tools=True))
    row = asyncio.run(ab.bridge_turn(fake, "tool_offered", {}, _phrase(), [], 0))
    assert seen == [(("command",), None)]
    assert row["first"]["ok"] and row["final"]["ok"] and not row["retried"]


def test_bridge_counts_retry_and_two_command_calls_as_several_commands():
    replies = [
        LLMResponse(content="", tool_calls=(ToolCall("c1", "command", {"speech": "а"}),
                                            ToolCall("c2", "command", {"speech": "б"}))),
        LLMResponse(content="", tool_calls=(ToolCall("c3", "command", {"speech": "Привет"}),)),
    ]
    fake = FakeLLMProvider(responses=replies, capabilities=ProviderCapabilities(tools=True, tool_choice=True))
    row = asyncio.run(ab.bridge_turn(fake, "tool_choice", {}, _phrase(), [], 0))
    assert row["first"]["reason"] == "several_commands" and row["retried"] and row["final"]["ok"]
    summary = ab.summarize_cell("fake/tool_choice", [row], Counter(), 1)
    assert (summary["delivered_first"], summary["delivered"], summary["fixed"], summary["multi_command"]) == (0, 1, 1, 1)


def test_bridge_phantom_is_claim_without_actions():
    phrase = _phrase("actions", ["play_animation"])
    claim = ab.turn_verdict(phrase, ab.Command(speech="Готово, включил"))
    assert claim["phantom"] and not claim["ok"]
    done = ab.turn_verdict(phrase, ab.Command(actions=(CommandCall("play_animation", {}),)))
    assert done["ok"] and not done["phantom"]


def test_parse_cell_rejects_unknown_mode_or_variant():
    assert ab.parse_cell("minimax:MiniMax-M3/tool_offered/adaptive") == ("minimax:MiniMax-M3", "tool_offered", "adaptive")
    assert ab.parse_cell("deepseek") == ("deepseek", "auto", "robot")
    for bad in ("minimax/nope", "minimax/tool_choice/hot"):
        with pytest.raises(ValueError):
            ab.parse_cell(bad)
