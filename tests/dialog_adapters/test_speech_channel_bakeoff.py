"""speech_channel_bakeoff.py: офлайн-часть (FakeLLMProvider, ключи не нужны). #3697."""

from __future__ import annotations

import asyncio

from rob_box_llm.provider import LLMResponse, ToolCall
from rob_box_llm.providers.fake import FakeLLMProvider
from scripts.dialog import adapter_benchmark as ab
from scripts.dialog import speech_channel_bakeoff as bo

SAY = bo.Phrase("t1", "Робот, привет", "speech_only")
ACT = bo.Phrase("t2", "Робот, покажи анимацию", "actions", ("play_animation",))
SIL = bo.Phrase("t3", "ну вот", "silence")


def _fake(*responses: LLMResponse) -> FakeLLMProvider:
    return FakeLLMProvider(responses=list(responses))


def _turn(provider, channel, mode, phrase):
    return asyncio.run(bo.run_turn(provider, channel, mode, phrase, bo.stub_tools(["play_animation"]), ["play_animation"]))


def test_field_speech_in_slot_is_delivered_without_leak():
    out = _turn(_fake(LLMResponse(content='{"speech": "Привет!"}')), "field", "json_in_text", SAY)
    j = bo.judge(SAY, out)
    assert out.speeches == ["Привет!"]
    assert (j["delivered"], j["leak"], j["double"], j["phantom"], j["stray"]) == (True, False, False, False, False)


def test_field_text_around_json_is_stray_not_spoken():
    out = _turn(_fake(LLMResponse(content='Конечно! {"speech": "Привет!"}')), "field", "json_in_text", SAY)
    j = bo.judge(SAY, out)
    assert j["stray"] and j["delivered"] and not j["lost"]


def test_speech_in_content_instead_of_slot_is_lost():
    out = _turn(_fake(LLMResponse(content="Привет, я Робби")), "tool", "tool_choice", SAY)
    j = bo.judge(SAY, out)
    assert j["lost"] and not j["delivered"]


def test_service_text_and_execution_claim_in_speech():
    leaky = bo.Outcome(speeches=['{"speech": "x"}'])
    claim = bo.Outcome(speeches=["Готово, включил"])
    assert bo.judge(SAY, leaky)["leak"] and bo.judge(ACT, claim)["phantom"]
    assert not bo.judge(SAY, bo.Outcome(speeches=["Привет"]))["phantom"]


def test_tool_channel_ignores_content_and_counts_second_call():
    first = LLMResponse(content="Привет всем", tool_calls=(ToolCall("1", "speak_text", {"text": "Привет!"}),))
    out = _turn(_fake(first, LLMResponse(content="Готово")), "tool", "tool_choice", SAY)
    j = bo.judge(SAY, out)
    assert out.llm_calls == 2 and out.speeches == ["Привет!"]
    assert j["delivered"] and j["stray"] and not j["double"]  # текст вне слота не озвучивается


def test_tool_channel_double_speech_on_second_call():
    first = LLMResponse(tool_calls=(ToolCall("1", "speak_text", {"text": "Привет!"}),))
    second = LLMResponse(tool_calls=(ToolCall("2", "speak_text", {"text": "Привет, друг!"}),))
    assert bo.judge(SAY, _turn(_fake(first, second), "tool", "tool_choice", SAY))["double"] is True


def test_silence_with_speech_is_spurious_and_stay_silent_is_clean():
    spoke_call = LLMResponse(tool_calls=(ToolCall("1", "speak_text", {"text": "А?"}),))
    spoke = _turn(_fake(spoke_call, LLMResponse()), "tool", "tool_choice", SIL)
    assert bo.judge(SIL, spoke)["spurious"] is True
    quiet = _turn(_fake(LLMResponse(tool_calls=(ToolCall("1", "stay_silent", {}),))), "tool", "tool_choice", SIL)
    j = bo.judge(SIL, quiet)
    assert quiet.llm_calls == 1 and not j["spurious"] and not j["expects"]


def test_action_phrase_needs_action_and_invalid_is_not_delivered():
    speech_only = _turn(_fake(LLMResponse(content='{"speech": "Ок"}')), "field", "json_in_text", ACT)
    assert bo.judge(ACT, speech_only)["delivered"] is False
    ok = LLMResponse(tool_calls=(ToolCall("1", "play_animation", {"value": "sleep"}),))
    assert bo.judge(ACT, _turn(_fake(ok, LLMResponse()), "tool", "tool_choice", ACT))["delivered"] is True
    bad = _turn(_fake(LLMResponse(content="просто текст")), "field", "json_in_text", SAY)
    assert bo.judge(SAY, bad)["invalid"] is True


def test_provider_failure_on_every_call_is_not_run_not_zero_percent():
    async def boom(msgs, tools, settings):
        raise RuntimeError("402 Payment Required")

    provider = FakeLLMProvider(on_complete=boom)
    cell = asyncio.run(bo.run_cell(provider, "field", "json_in_text", [SAY, ACT], 2, 2, ["play_animation"]))
    res = bo.summarize(cell, 4)
    assert "not_run" in res and "402" in res["not_run"]
    assert "не прогнан: " in bo.render([("mimo", "field", "json_in_text", res)], 2, 2)


def test_phrases_exclude_tier1_and_cover_all_kinds():
    phrases = bo.load_phrases()
    assert len(phrases) == 58 and {p.kind for p in phrases} == {"actions", "ask", "silence", "speech_only"}
    assert "speak_text" not in bo.action_tool_names(phrases)


def test_adapter_registry_has_new_adapters_and_jev_is_reported(monkeypatch):
    for var in set(ab.KEY_ENV.values()):
        monkeypatch.setenv(var, "k" * 12)
    assert ab.build_provider("jev") is None and "404" in ab.unavailable("jev")
    assert ab.build_provider("qwen-flash")._default_model == "qwen3.8-flash"
    assert ab.build_provider("minimax-flash")._default_model == "MiniMax-M3.1-Flash"
    assert ab.build_provider("deepseek-pro")._default_model == "deepseek-v4-pro"
    assert "k" * 12 not in ab.redact("boom " + "k" * 12)


def test_rescore_rebuilds_table_from_raw_and_keeps_not_run(tmp_path):
    import json

    row = {"cell": "deepseek/field/tool_choice", "id": "r12", "run": 0, "speeches": ["Жили-были"], "actions": [],
           "passed": False, "leaks": [], "invalid": None, "llm_calls": 1, "ms": 900, "served": "deepseek-flash",
           "round2_error": None}
    raw, summary = tmp_path / "raw.jsonl", tmp_path / "s.json"
    raw.write_text(json.dumps(row, ensure_ascii=False) + "\n", encoding="utf-8")
    summary.write_text(json.dumps([["mimo", "field", "json_in_text", {"calls": 3, "not_run": "402"}]]), encoding="utf-8")
    table = bo.render(bo.rescore(raw, summary), 58, 3)
    assert "deepseek-flash" in table and "1/1 (100 %)" in table and "не прогнан: 402" in table
