"""voice_request: ожидание voice_changed по request_id и данные исхода для фразы диалога (PR-4 ч.3, #3693)."""

from __future__ import annotations

import json
import sys
import threading
import types

import pytest


class _Pub:
    def __init__(self, on_publish) -> None:
        self._on_publish = on_publish

    def publish(self, msg) -> None:
        self._on_publish(msg)


class _SpeechEvent:
    KIND_VOICE_CHANGED = 6

    def __init__(self, kind=6, turn_id="", requested="", applied="", provider="", reason="") -> None:
        self.kind, self.turn_id, self.voice_requested = kind, turn_id, requested
        self.voice_applied, self.provider, self.reason = applied, provider, reason


class _String:
    data = ""


@pytest.fixture
def vr(monkeypatch):
    msgs = types.ModuleType("rob_box_dialog_msgs.msg")
    msgs.SpeechEvent = _SpeechEvent
    monkeypatch.setitem(sys.modules, "rob_box_dialog_msgs", types.ModuleType("rob_box_dialog_msgs"))
    monkeypatch.setitem(sys.modules, "rob_box_dialog_msgs.msg", msgs)
    std = types.ModuleType("std_msgs.msg")
    std.String = _String
    monkeypatch.setitem(sys.modules, "std_msgs", types.ModuleType("std_msgs"))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std)
    import importlib

    module = importlib.import_module("rob_box_mcp_tools.voice_request")
    return importlib.reload(module)


class _Node:
    def __init__(self, vr, reply) -> None:
        self.sub = None
        self.sent = []
        self._reply = reply
        self._vr = vr

    def create_publisher(self, typ, topic, depth):
        assert topic == "/voice/tts/voice_request"
        return _Pub(self._on_publish)

    def create_subscription(self, typ, topic, cb, depth):
        assert topic == "/voice/speech/event"
        self.sub = cb

    def _on_publish(self, msg) -> None:
        body = json.loads(msg.data)
        self.sent.append(body)
        reply = self._reply(body)
        if reply is not None:
            threading.Thread(target=self.sub, args=(reply,)).start()


def test_applied_outcome_matches_by_request_id(vr):
    node = _Node(vr, lambda b: _SpeechEvent(turn_id=b["request_id"], requested=b["voice"], applied=b["voice"], provider="minimax"))
    out = vr.VoiceRequests(node).request("female-shaonv")
    assert out.status == "applied" and out.applied == "female-shaonv"
    data = vr.outcome_data(out)
    assert (data["status"], data["voice_applied"], data["voice_default"]) == ("applied", "female-shaonv", False)
    assert "reason" not in data


def test_fallback_outcome_data_names_both_voices(vr):
    node = _Node(vr, lambda b: _SpeechEvent(turn_id=b["request_id"], requested="alena", applied="male-qn-qingse",
                                            provider="minimax", reason="voice_unavailable"))
    out = vr.VoiceRequests(node).request("alena")
    assert out.status == "fallback"
    data = vr.outcome_data(out)
    assert (data["requested_name"], data["voice_name"], data["reason"]) == ("alena", "male-qn-qingse", "voice_unavailable")
    assert data["voice_default"] is True  # применён обычный голос робота (minimax/male-qn-qingse): диалог так и скажет


def test_unknown_provider_outcome_data(vr):
    node = _Node(vr, lambda b: _SpeechEvent(turn_id=b["request_id"], requested="x", applied="", provider="eleven",
                                            reason="unknown_provider"))
    out = vr.VoiceRequests(node).request("x", "eleven")
    data = vr.outcome_data(out)
    assert out.status == "fallback" and data["provider_label"] == "eleven" and "voice_default" not in data


def test_other_requests_event_is_ignored_then_timeout(vr):
    node = _Node(vr, lambda b: _SpeechEvent(turn_id="someone-else", requested="a", applied="a", provider="minimax"))
    out = vr.VoiceRequests(node).request("a", wait_s=0.2)
    assert out.status == "timeout"
    data = vr.outcome_data(out)
    assert (data["status"], data["reason"]) == ("timeout", "timeout") and "voice_default" not in data


def test_non_voice_events_are_ignored(vr):
    node = _Node(vr, lambda b: _SpeechEvent(kind=4, turn_id=b["request_id"]))
    assert vr.VoiceRequests(node).request("a", wait_s=0.2).status == "timeout"


def test_voice_requests_is_one_per_node(vr):
    node = _Node(vr, lambda b: None)
    assert vr.voice_requests(node) is vr.voice_requests(node)


def test_speak_voice_resolution(vr):
    assert vr.resolve_speak_voice("minimax", None) == ("", False)
    assert vr.resolve_speak_voice("minimax", "alena") == ("male-qn-qingse", True)


@pytest.mark.parametrize("args, status, expected", [
    (("applied", "male-qn-qingse", "male-qn-qingse", "minimax"), "done", "Говорю своим обычным голосом."),
    (("applied", "zahar", "zahar", "yandex"), "done", "Говорю голосом zahar."),
    (("fallback", "alena", "male-qn-qingse", "minimax", "voice_unavailable"), "rejected",
     "Голос alena недоступен, остаюсь на обычном."),
    (("fallback", "alena", "zahar", "yandex", "voice_unavailable"), "rejected",
     "Голос alena недоступен, остаюсь на zahar."),
    (("fallback", "x", "", "eleven", "unknown_provider"), "rejected",
     "Не получилось сменить голос: провайдер eleven не подходит."),
    (("timeout", "zahar", "", "yandex"), "rejected", "Не дождался подтверждения смены голоса, не знаю, какой сейчас."),
])
def test_voice_outcome_data_contract(vr, args, status, expected):
    """Данные исхода тула - ровно те поля, из которых диалог строит фразу (``knowledge.VOICE_DONE``/``VOICE_FAILED``):
    слот, которого тул не отдал, уронил бы фразу в общее «Не вышло.» (``turn.outcome_phrase``)."""
    from rob_box_dialog import respond
    from rob_box_dialog.respond import ActionResult

    data = vr.outcome_data(vr.VoiceOutcome(*args))
    result = ActionResult("t/0", 0, "personality", "set_voice", "action.local", status, data.get("reason"), data)
    assert respond.phrase_from_result(result).text == expected
