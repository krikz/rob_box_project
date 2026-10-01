"""Тесты core.tars_console (issue #3253, Ш4): фразы оператора и события хода."""

from __future__ import annotations

import array
import json

from rob_box_quest.core import tars_console as tc


def test_operator_event_from_stt_payload() -> None:
    ev = tc.operator_event({"text": "  включи музыку ", "client_id": "q1", "ts_ms": 1234})
    assert ev is not None
    assert ev["type"] == "tars_console" and ev["kind"] == "operator"
    assert ev["text"] == "включи музыку"
    assert ev["request_id"] == "q1:1234"
    assert ev["ts_ms"] == 1234
    json.dumps(ev)


def test_operator_event_empty_or_bad_payload() -> None:
    assert tc.operator_event({"text": "   "}) is None
    assert tc.operator_event({}) is None
    assert tc.operator_event("text") is None
    assert tc.operator_event(None) is None


def test_operator_events_list_form() -> None:
    assert tc.operator_events({"text": "стоп"})[0]["text"] == "стоп"
    assert tc.operator_events({"text": ""}) == []
    assert tc.operator_events(None) == []


def test_operator_text_is_capped() -> None:
    ev = tc.operator_event({"text": "а" * 5000})
    assert ev is not None and len(ev["text"]) == tc.MAX_OPERATOR_TEXT


def test_turn_events_only_tool_names_no_params() -> None:
    payload = {
        "request_id": "r1",
        "ok": True,
        "summary": "готово",
        "tool_calls": [
            {"name": "play_music", "args": {"signature": "SECRET"}},
            {"name": "set_volume", "arguments": "token=abc"},
        ],
    }
    evs = tc.turn_events(payload)
    assert [e["text"] for e in evs] == ["tool: play_music", "tool: set_volume"]
    assert all(e["kind"] == "event" and e["request_id"] == "r1" for e in evs)
    blob = json.dumps(evs)
    assert "SECRET" not in blob and "token" not in blob


def test_turn_events_failure_has_no_error_text() -> None:
    evs = tc.turn_events({"request_id": "r2", "ok": False, "error": "key=sk-123", "tool_calls": []})
    assert [e["text"] for e in evs] == ["ошибка хода"]
    assert "sk-123" not in json.dumps(evs)


def test_turn_events_ok_without_tools_is_silent() -> None:
    assert tc.turn_events({"ok": True, "summary": "привет", "tool_calls": []}) == []
    assert tc.turn_events({"summary": "нет ok"}) == []
    assert tc.turn_events([]) == []


def test_turn_events_tolerate_junk_and_arrays() -> None:
    # tool_calls бывает не только list: строки, мусор, пустые имена.
    evs = tc.turn_events({"tool_calls": ("a", 5, {"name": ""}, {"name": "x" * 200}, None)})
    assert [e["text"] for e in evs] == ["tool: a", "tool: " + "x" * tc.MAX_TOOL_NAME]
    # ndarray/array.array не должны ронять разбор (в rclpy массивы не list).
    assert tc.turn_events({"tool_calls": array.array("i", [1, 2])}) == []


def test_relay_events_survives_broadcast_failure() -> None:
    sent: list[dict] = []
    logs: list[str] = []

    def broadcast(ev: dict) -> None:
        if ev["text"] == "tool: a":
            raise RuntimeError("ws down")
        sent.append(ev)

    evs = tc.turn_events({"tool_calls": [{"name": "a"}, {"name": "b"}]})
    tc.relay_events(evs, broadcast, logs.append)
    assert [e["text"] for e in sent] == ["tool: b"]
    assert len(logs) == 1 and "ws down" in logs[0]
