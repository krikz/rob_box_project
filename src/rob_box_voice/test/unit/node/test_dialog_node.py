"""PR-8 ч.8e2 (#3702): ROS-хост диалога v2 ``dialog_node`` на заглушках rclpy (conftest) и фейковых LLM/тулах."""

from __future__ import annotations

import json
import sys
import types
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()
try:  # настоящий rosidl-пакет (CI) или заглушка сервиса DialogControl (Windows)
    import rob_box_dialog_msgs.srv  # noqa: F401
except ImportError:
    class _Request:
        ACTION_HOLD, ACTION_RESUME, ACTION_RESET = 0, 1, 2

        def __init__(self, action=0, set_by="", ttl_s=0.0):
            self.action, self.set_by, self.ttl_s = action, set_by, ttl_s

    _srv = types.ModuleType("rob_box_dialog_msgs.srv")
    _srv.DialogControl = type("DialogControl", (), {"Request": _Request,
                                                    "Response": lambda: types.SimpleNamespace()})
    sys.modules["rob_box_dialog_msgs.srv"] = _srv
    sys.modules["rob_box_dialog_msgs"].srv = _srv

from rob_box_dialog_msgs.srv import DialogControl  # noqa: E402
from rob_box_llm.provider import LLMResponse, ToolResult  # noqa: E402
from rob_box_voice import dialog_node as dn  # noqa: E402

TIME = type(M.TurnEvent().stamp)


class _LLM:
    def __init__(self, responses):
        self.responses, self.calls = list(responses), 0
        self.answered_by = "deepseek"

    async def complete(self, messages, tools=(), settings=None):
        self.calls += 1
        return self.responses.pop(0)


class _Tools:
    def __init__(self):
        self.calls = []

    async def execute(self, call):
        self.calls.append(call.name)
        return ToolResult(tool_call_id=call.id, content="ok", data={"status": "done"})


@pytest.fixture()
def node(monkeypatch):
    llm = _LLM([LLMResponse(content="Привет, я робот.")])
    tools = _Tools()
    monkeypatch.setattr(dn, "build_llm", lambda node, counter: (llm, "Ты робот."))
    monkeypatch.setattr(dn, "build_tools", lambda node: tools)
    made = dn.DialogNode()
    stamp = TIME(sec=1, nanosec=0)
    made.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))
    made.test_llm, made.test_tools = llm, tools
    yield made
    made.shutdown()


def _topics(made, kind):
    return [t for t, _ in getattr(made, kind)]


def _utterance(text):
    msg = M.Utterance()
    msg.text, msg.source, msg.utterance_id = text, "respeaker", "u1"
    return msg


def _published(made, topic):
    pubs = [p for t, p in made._publishers if t == topic]
    return [c.args[0] for p in pubs for c in p.publish.call_args_list]


def test_wiring_v2_topics_and_no_v1_dialogue_topics(node):
    subs = set(_topics(node, "_subscribers"))
    assert {"/voice/stt/utterance", "/voice/speaker/result", "/voice/speech/event", "/voice/sound/event",
            "/voice/music/state", "/voice/music/event", "/voice/providers/state",
            "/dialog/interrupt_request"} <= subs
    pubs = set(_topics(node, "_publishers"))
    assert {"/dialog/session", "/dialog/intent"} <= pubs  # /dialog/plan_event — plan_host в build_tools (подменён)
    for legacy in ("/voice/dialogue/response", "/voice/tts/request", "/dialogue/control", "/dialogue/control_ack"):
        assert legacy not in subs | pubs, legacy
    assert node.engine_flag == "v1"  # параметр есть; какую ноду стартовать, решает launch по dialog_engine.yaml


def test_addressed_utterance_runs_turn_and_speaks(node):
    node._on_utterance(_utterance("Робот, как дела"))
    node.last_turn.result(timeout=10)
    said = _published(node, "/voice/speech/request")
    assert [s.text for s in said] == ["<speak>Привет, я робот.</speak>"]
    assert node.test_llm.calls == 1


def test_unaddressed_utterance_goes_to_backlog_without_turn(node):
    node._on_utterance(_utterance("ну вот покажи анимацию"))
    assert node.last_turn is None and node.test_llm.calls == 0
    assert [e.text for e in node._backlog._entries] == ["ну вот покажи анимацию"]


def test_speech_end_puts_spoken_part_into_window(node):
    node._on_utterance(_utterance("Робот, как дела"))
    node.last_turn.result(timeout=10)
    said = _published(node, "/voice/speech/request")[0]
    event = M.SpeechEvent()
    event.kind, event.speech_id, event.agent, event.spoken_chars = M.SpeechEvent.KIND_FINISHED, said.speech_id, \
        "personality", 7
    node._on_speech_event(event)
    assert [e.content for e in node._engine.log.window()] == ["как дела", "Привет,"]


def test_control_reset_hold_resume(node):
    response = types.SimpleNamespace()
    out = node._on_control(DialogControl.Request(action=DialogControl.Request.ACTION_RESET, set_by="operator"),
                           response)
    assert out.ok and out.epoch == 1
    node._on_control(DialogControl.Request(action=DialogControl.Request.ACTION_HOLD, set_by="operator", ttl_s=60.0),
                     types.SimpleNamespace())
    assert node._session.snapshot.top("OperatorHold") is not None
    node._on_control(DialogControl.Request(action=DialogControl.Request.ACTION_RESUME, set_by="operator"),
                     types.SimpleNamespace())
    assert node._session.snapshot.top("OperatorHold") is None
    assert _published(node, "/dialog/session"), "снимок сессии latched-топиком на каждое изменение"


def test_music_state_goes_to_session_snapshot(node):
    msg = MagicMock()
    msg.data = json.dumps({"state": "playing", "track_id": "t9", "dj": {"enabled": True, "title": "Сет"}})
    node._on_music_state(msg)
    music = node._session.snapshot.music
    assert (music.state, music.track_id, music.dj) == ("playing", "t9", True)


def test_clear_stop_interrupt_cancels_speech_of_epoch(node):
    msg = M.InterruptRequest()
    msg.text_partial, msg.utterance_id = "Робот, стоп", "u2"
    node._on_interrupt(msg)
    assert _published(node, "/voice/speech/cancel")


def test_e2e_db_outside_data_only(node, tmp_path):
    node._parameters["e2e_mode"] = True
    node._declared["e2e_mode"].value = True
    node._declared["e2e_db_path"].value = "/data/x.db"
    assert node._open_events() is None
    node._declared["e2e_db_path"].value = str(tmp_path / "e2e.db")
    events = node._open_events()
    assert events is not None and events.path.endswith("e2e.db")
    events.close()
