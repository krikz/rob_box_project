"""adapter_benchmark.py: офлайн-часть (FakeLLMProvider; живые адаптеры без ключей не вызываются). #3686."""

from __future__ import annotations

import asyncio
import json

from rob_box_llm.provider import LLMResponse
from rob_box_llm.providers.fake import FakeLLMProvider
from scripts.dialog import adapter_benchmark as ab
from scripts.dialog.stub_parse import stub_parse


def _fake(payload: str) -> FakeLLMProvider:
    async def on_complete(msgs, tools, settings):
        return LLMResponse(content=payload)

    return FakeLLMProvider(on_complete=on_complete)


def test_counts_delivered_and_invalid_and_sums_to_calls():
    good = asyncio.run(ab.run_adapter(_fake(json.dumps({"speech": "Привет"})), "json_in_text", ["a", "b"], 2, stub_parse))
    assert (good["calls"], good["delivered"], good["invalid"]) == (4, 4, 0)
    bad = asyncio.run(ab.run_adapter(_fake("не json"), "json_in_text", ["a"], 3, stub_parse))
    assert (bad["calls"], bad["delivered"], bad["invalid"]) == (3, 0, 3)
    empty = asyncio.run(ab.run_adapter(_fake("{}"), "json_mode", ["a"], 1, stub_parse))
    assert (empty["delivered"], empty["invalid"]) == (0, 1)


def test_missing_key_is_reported_not_skipped(monkeypatch):
    for var in ab.KEY_ENV.values():
        monkeypatch.delenv(var, raising=False)
    assert ab.build_provider("deepseek") is None
    table = ab.render([("deepseek", "auto", None)], "stub", 20)
    assert "не прогнано: нет ключа" in table


def test_phrases_exclude_tier1_and_respect_limit():
    assert len(ab.load_phrases(0)) == 58
    assert len(ab.load_phrases(20)) == 20
