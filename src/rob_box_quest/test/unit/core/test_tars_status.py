"""Тесты core.tars_status (issue #3253, Ш3): формирование, троттлинг, прочерки."""

from __future__ import annotations

import json

from rob_box_quest.core.tars_status import (
    HEARTBEAT_S,
    MIN_INTERVAL_S,
    NEARBY_TTL_S,
    TarsStatusAggregator,
    TarsStatusRelay,
    format_tts,
)


def test_no_sources_means_no_event() -> None:
    agg = TarsStatusAggregator()
    assert agg.poll(0.0, 1, None, None) is None
    assert agg.poll(0.0, 1, "", "  ") is None


def test_missing_fields_are_absent_not_invented() -> None:
    agg = TarsStatusAggregator()
    ev = agg.poll(0.0, 5, "minimax", "voice1")
    assert ev == {"type": "tars_status", "tts": "minimax · voice1", "ts_ms": 5}
    # llm и wake — нет источника, ключей нет никогда.
    assert "llm" not in ev and "wake" not in ev
    assert "topic" not in ev and "nearby" not in ev


def test_format_tts() -> None:
    assert format_tts("yandex", "alena") == "yandex · alena"
    assert format_tts("yandex", None) == "yandex"
    assert format_tts(None, "alena") is None


def test_dj_mode_topic() -> None:
    agg = TarsStatusAggregator()
    agg.note_dj_mode({"enabled": True, "theme": " рок 80-х "})
    assert agg.snapshot(0.0, None, None) == {"topic": "DJ · рок 80-х"}
    agg.note_dj_mode({"enabled": True})
    assert agg.snapshot(0.0, None, None) == {"topic": "DJ"}
    agg.note_dj_mode({"enabled": False, "theme": "старая"})
    assert agg.snapshot(0.0, None, None) == {"topic": "обычный режим"}


def test_dj_mode_garbage_ignored() -> None:
    agg = TarsStatusAggregator()
    agg.note_dj_mode(None)
    agg.note_dj_mode("x")
    agg.note_dj_mode({"enabled": "yes"})
    assert agg.snapshot(0.0, None, None) == {}


def test_nearby_known_only_and_ttl() -> None:
    agg = TarsStatusAggregator()
    agg.note_speaker({"is_known": False, "name": "Кто-то"}, 0.0)
    assert "nearby" not in agg.snapshot(0.0, None, None)
    agg.note_speaker({"is_known": True, "name": "Борис", "confidence": 0.9}, 10.0)
    assert agg.snapshot(11.0, None, None)["nearby"] == "Борис"
    # inconclusive и неузнанный кадры не стирают узнанного
    agg.note_speaker({"is_known": False, "inconclusive": True, "reason": "short"}, 20.0)
    agg.note_speaker({"is_known": False}, 21.0)
    assert agg.snapshot(22.0, None, None)["nearby"] == "Борис"
    # устарел → прочерк
    assert "nearby" not in agg.snapshot(10.0 + NEARBY_TTL_S + 1, None, None)


def test_throttle_changes_not_faster_than_min_interval() -> None:
    agg = TarsStatusAggregator()
    assert agg.poll(0.0, 1, "minimax", "a") is not None
    assert agg.poll(0.5, 2, "yandex", "b") is None  # изменилось, но рано
    ev = agg.poll(MIN_INTERVAL_S, 3, "yandex", "b")
    assert ev is not None and ev["tts"] == "yandex · b"


def test_unchanged_is_silent_until_heartbeat() -> None:
    agg = TarsStatusAggregator()
    assert agg.poll(0.0, 1, "minimax", "a") is not None
    assert agg.poll(HEARTBEAT_S - 0.1, 2, "minimax", "a") is None
    assert agg.poll(HEARTBEAT_S, 3, "minimax", "a") is not None


def test_field_disappearing_is_reported_after_interval() -> None:
    agg = TarsStatusAggregator()
    agg.note_speaker({"is_known": True, "name": "Борис"}, 0.0)
    assert agg.poll(0.0, 1, "minimax", "a")["nearby"] == "Борис"
    ev = agg.poll(NEARBY_TTL_S + 1, 2, "minimax", "a")
    assert ev is not None and "nearby" not in ev


# ───────────── TarsStatusRelay: колбэки подписок + таймер ─────────────


class _Msg:
    def __init__(self, data: str) -> None:
        self.data = data


def _relay(tts=("minimax", "v1")):
    sent: list = []
    logs: list = []
    relay = TarsStatusRelay(lambda: tts, sent.append, logs.append)
    return relay, sent, logs


def test_relay_timer_broadcasts_status_from_topics() -> None:
    relay, sent, _ = _relay()
    relay.on_dj_mode(_Msg(json.dumps({"enabled": True, "theme": "джаз"})))
    relay.on_speaker_result(_Msg(json.dumps({"is_known": True, "name": "Борис"})))
    relay.on_timer()
    assert len(sent) == 1
    ev = sent[0]
    assert ev["type"] == "tars_status"
    assert ev["tts"] == "minimax · v1"
    assert ev["topic"] == "DJ · джаз"
    assert ev["nearby"] == "Борис"
    assert "llm" not in ev and "wake" not in ev
    relay.on_timer()  # без изменений — молчим (троттлинг)
    assert len(sent) == 1


def test_relay_silent_without_data_and_survives_bad_json() -> None:
    relay, sent, _ = _relay(tts=("", ""))
    relay.on_dj_mode(_Msg("не json"))
    relay.on_speaker_result(_Msg(""))
    relay.on_timer()
    assert sent == []


def test_relay_broadcast_failure_does_not_raise() -> None:
    logs: list = []

    def boom(_ev):
        raise RuntimeError("ws down")

    relay = TarsStatusRelay(lambda: ("minimax", "v1"), boom, logs.append)
    relay.on_timer()
    assert logs and "ws down" in logs[0]


def test_llm_and_wake_appear_only_after_latched_topics() -> None:
    from rob_box_quest.core.tars_status import format_llm, format_wake

    assert format_llm({"provider": "minimax", "model": "M2"}) == "minimax · M2"
    assert format_llm({"provider": "deepseek"}) == "deepseek"
    assert format_llm({"model": "M2"}) is None
    assert format_llm("x") is None
    assert format_wake({"words": ["робот", " робокс ", 5, ""]}) == "робот, робокс"
    assert format_wake({"words": []}) is None
    assert format_wake({"words": "робот"}) is None

    agg = TarsStatusAggregator()
    agg.note_llm({"provider": "minimax", "model": "M2", "chain": ["minimax"]})
    agg.note_wake({"words": ["робот", "робокс"]})
    ev = agg.poll(0.0, 7, None, None)
    assert ev == {"type": "tars_status", "llm": "minimax · M2",
                  "wake": "робот, робокс", "ts_ms": 7}


def test_relay_routes_llm_and_wake_callbacks() -> None:
    from types import SimpleNamespace

    sent: list = []
    relay = TarsStatusRelay(lambda: (None, None), sent.append, lambda _t: None)
    relay.on_llm_status(SimpleNamespace(data=json.dumps({"provider": "deepseek", "model": "v4"})))
    relay.on_wake_words(SimpleNamespace(data=json.dumps({"words": ["робот"]})))
    relay.on_wake_words(SimpleNamespace(data="не json"))
    relay.on_timer()
    assert len(sent) == 1
    assert sent[0]["llm"] == "deepseek · v4" and sent[0]["wake"] == "робот"
