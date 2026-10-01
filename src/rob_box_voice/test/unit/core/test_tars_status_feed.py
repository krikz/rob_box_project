"""Тесты core.tars_status_feed (issue #3253, Ш3б)."""

from __future__ import annotations

import json
from types import SimpleNamespace

from rob_box_voice.core.tars_status_feed import (
    StatusFeed,
    describe_llm,
    describe_wake,
)


class _Pub:
    def __init__(self) -> None:
        self.sent: list = []

    def publish(self, msg) -> None:
        self.sent.append(msg)


def _msg(data: str):
    return SimpleNamespace(data=data)


def test_describe_llm_single_provider() -> None:
    d = describe_llm(SimpleNamespace(name="deepseek"))
    assert d is not None
    assert d["provider"] == "deepseek"
    assert d["chain"] == ["deepseek"]
    assert d["source"] == "configured"


def test_describe_llm_chain_takes_primary_first() -> None:
    llm = SimpleNamespace(
        name="health-aware-fallback",
        _providers=[SimpleNamespace(name="MiniMax"), SimpleNamespace(name="deepseek")],
    )
    d = describe_llm(llm)
    assert d["provider"] == "minimax"
    assert d["chain"] == ["minimax", "deepseek"]


def test_describe_llm_unknown_model_means_no_model_key() -> None:
    d = describe_llm(SimpleNamespace(name="no-such-provider"))
    assert d is not None and "model" not in d


def test_describe_llm_without_name_is_none() -> None:
    assert describe_llm(object()) is None


def test_describe_wake() -> None:
    assert describe_wake(["робот", " робокс ", "", 5]) == {"words": ["робот", "робокс"]}
    assert describe_wake([]) is None


def test_feed_publishes_json_and_skips_repeat() -> None:
    pub = _Pub()
    feed = StatusFeed(pub, _msg)
    assert feed.publish({"words": ["робот"]}) is True
    assert feed.publish({"words": ["робот"]}) is False
    assert feed.publish({"words": ["робот", "робокс"]}) is True
    assert feed.publish(None) is False
    assert [json.loads(m.data) for m in pub.sent] == [
        {"words": ["робот"]},
        {"words": ["робот", "робокс"]},
    ]
    assert "робот" in pub.sent[0].data  # ensure_ascii=False
