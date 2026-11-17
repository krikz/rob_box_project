"""Статус LLM и вейков для ТАРС (#3253): перенос v1 ``core/tars_status_feed.py`` в ``rob_box_dialog`` (#3912)."""

from __future__ import annotations

import json
from types import SimpleNamespace

from rob_box_dialog.status_feed import StatusFeed, describe_chain, describe_wake


class _Pub:
    def __init__(self) -> None:
        self.sent: list = []

    def publish(self, msg) -> None:
        self.sent.append(msg)


def test_chain_primary_first_lowercase_configured() -> None:
    assert describe_chain(["MiniMax", " deepseek ", ""]) == {
        "provider": "minimax", "chain": ["minimax", "deepseek"], "source": "configured"}


def test_model_only_when_catalog_knows_it() -> None:
    assert describe_chain(["deepseek"], lambda name: "deepseek-chat")["model"] == "deepseek-chat"
    assert "model" not in describe_chain(["deepseek"], lambda name: None)
    assert "model" not in describe_chain(["deepseek"])


def test_empty_chain_is_none() -> None:
    assert describe_chain([]) is None and describe_chain(["", 5]) is None


def test_describe_wake() -> None:
    assert describe_wake(["робот", " робокс ", "", 5]) == {"words": ["робот", "робокс"]}
    assert describe_wake([]) is None


def test_feed_publishes_json_and_skips_repeat() -> None:
    pub = _Pub()
    feed = StatusFeed(pub, lambda data: SimpleNamespace(data=data))
    assert feed.publish({"words": ["робот"]}) is True
    assert feed.publish({"words": ["робот"]}) is False
    assert feed.publish({"words": ["робот", "робокс"]}) is True
    assert feed.publish(None) is False
    assert [json.loads(m.data) for m in pub.sent] == [{"words": ["робот"]}, {"words": ["робот", "робокс"]}]
    assert "робот" in pub.sent[0].data  # ensure_ascii=False
