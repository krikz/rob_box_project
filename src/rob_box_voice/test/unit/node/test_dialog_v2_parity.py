"""#3912: паритет ``dialog_node`` v2 с v1 — приветствие по встрече лицом, стартовая фраза, фид статуса ТАРС.

Хост на заглушках rclpy (conftest) с фейковыми LLM/тулами, как ``test_dialog_node.py``. Правила — в
``rob_box_dialog`` (``test_meeting.py``, ``test_startup.py``, ``test_status_feed.py``); здесь — что хост их проводит:
подписка, речь владельцу речи, кадр окна ответа, latched-топики, таймеры.
"""

from __future__ import annotations

import json
import sys
import types

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()
try:  # настоящий rosidl-пакет (CI) или заглушка сервиса DialogControl (Windows) — как в test_dialog_node.py
    import rob_box_dialog_msgs.srv  # noqa: F401
except ImportError:
    class _Request:
        ACTION_HOLD, ACTION_RESUME, ACTION_RESET = 0, 1, 2

    _srv = types.ModuleType("rob_box_dialog_msgs.srv")
    _srv.DialogControl = type("DialogControl", (), {"Request": _Request,
                                                    "Response": lambda: types.SimpleNamespace()})
    sys.modules["rob_box_dialog_msgs.srv"] = _srv
    sys.modules["rob_box_dialog_msgs"].srv = _srv

from rob_box_llm.provider import LLMResponse  # noqa: E402
from rob_box_voice import dialog_node as dn  # noqa: E402
from rob_box_voice.core import dialog_v2_parity as parity  # noqa: E402

TIME = type(M.TurnEvent().stamp)


class _LLM:
    def __init__(self):
        self.calls = 0

    async def complete(self, messages, tools=(), settings=None):
        self.calls += 1
        return LLMResponse(content="И тебе привет.")


class _Tools:
    async def execute(self, call):  # pragma: no cover — ходы этих тестов тулов не зовут
        raise AssertionError(call.name)


@pytest.fixture()
def node(monkeypatch):
    try:
        import rob_box_perception_msgs.msg  # noqa: F401
    except ImportError:  # Windows: лицевых сообщений нет — подписка должна всё равно появиться
        pkg, msg = types.ModuleType("rob_box_perception_msgs"), types.ModuleType("rob_box_perception_msgs.msg")
        msg.VisionEvent = type("VisionEvent", (), {})
        pkg.msg = msg
        monkeypatch.setitem(sys.modules, "rob_box_perception_msgs", pkg)
        monkeypatch.setitem(sys.modules, "rob_box_perception_msgs.msg", msg)
    llm, tools = _LLM(), _Tools()
    monkeypatch.setattr(dn, "build_llm", lambda node, counter, agent="personality": (llm, "Ты робот."))
    monkeypatch.setattr(dn, "build_tools", lambda node: tools)
    monkeypatch.setattr(dn, "build_chat_tools", lambda node, personality_tools: tools)
    made = dn.DialogNode()
    stamp = TIME(sec=1, nanosec=0)
    made.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))
    made.test_llm = llm
    yield made
    made.shutdown()


def _published(made, topic):
    return [c.args[0] for t, p in made._publishers if t == topic for c in p.publish.call_args_list]


def _said(made):
    return _published(made, "/voice/speech/request")


def _face(name="Денис", person_id="p1", count=5, camera="oak_d"):
    attrs = {"encounter": "start", "person_id": person_id, "name": name, "is_new": False, "similarity": 0.9,
             "encounter_count": count}
    return types.SimpleNamespace(event_type="face", attributes_json=json.dumps(attrs), source_camera=camera)


def _finished(speech_id):
    event = M.SpeechEvent()
    event.kind, event.speech_id, event.agent = M.SpeechEvent.KIND_FINISHED, speech_id, "personality"
    return event


# -- фид ТАРС ----------------------------------------------------------------------------------------------------

def test_tars_feeds_are_latched_and_carry_configured_chain_and_wake(node):
    llm = [json.loads(m.data) for m in _published(node, "/voice/llm_status")]
    wake = [json.loads(m.data) for m in _published(node, "/voice/wake_words")]
    assert len(llm) == 1 and llm[0]["chain"] == ["deepseek", "minimax"] and llm[0]["provider"] == "deepseek"
    assert llm[0]["source"] == "configured"
    assert wake == [{"words": list(node._wake)}] and wake[0]["words"]
    pubs = {t: p for t, p in node._publishers}
    assert {"/voice/llm_status", "/voice/wake_words"} <= set(pubs)


# -- встреча лицом -----------------------------------------------------------------------------------------------

def test_meeting_subscription_exists(node):
    assert "/vision/hailo/events" in {t for t, _ in node._subscribers}


def test_known_face_after_restart_gets_one_happy_phrase_and_reply_window(node):
    phrase = parity.on_vision_event(node, _face())
    said = _said(node)
    assert phrase and "Денис" in phrase
    assert len(said) == 1 and "Денис" in said[0].text and said[0].emotion == "happy"
    assert said[0].agent == "personality"
    frame = node._session.snapshot.top("GreetingReply")
    assert frame is not None and frame.payload["speech_id"] == said[0].speech_id
    assert parity.on_vision_event(node, _face()) is None  # тот же день — молчим
    assert len(_said(node)) == 1


def test_stranger_unnamed_stub_and_first_meeting_are_silent(node):
    assert parity.on_vision_event(node, _face(name="", person_id="x")) is None
    assert parity.on_vision_event(node, _face(person_id="new", count=1)) is None
    assert parity.on_vision_event(node, _face(person_id="fake", camera="stub")) is None
    person = types.SimpleNamespace(event_type="person", attributes_json="{}", source_camera="oak_d")
    assert parity.on_vision_event(node, person) is None
    assert _said(node) == []


@pytest.mark.parametrize("kind,setter", [("Silence", "user"), ("OperatorHold", "operator")])
def test_silence_or_operator_hold_mutes_greeting_and_retries_later(node, kind, setter):
    node._session.push_scene(kind, setter, ttl_s=60.0)
    assert parity.on_vision_event(node, _face()) is None
    node._session.pop_scene(kind)
    assert parity.on_vision_event(node, _face()) is not None  # не «сгорело»: следующая встреча здоровается


def test_turn_in_flight_mutes_greeting(node, monkeypatch):
    monkeypatch.setattr(parity, "turn_busy", lambda _node: True)
    assert parity.on_vision_event(node, _face()) is None and _said(node) == []


def test_reply_without_wake_after_greeting_is_a_turn(node):
    parity.on_vision_event(node, _face())
    node._on_speech_event(_finished(_said(node)[0].speech_id))
    frame = node._session.snapshot.top("GreetingReply")
    assert frame is not None and frame.payload["asked_at"] is not None
    reply = M.Utterance()
    reply.text, reply.source, reply.utterance_id = "привет", "respeaker", "u2"
    reply.ended_at = TIME(sec=int(frame.payload["asked_at"]) + 1, nanosec=0)
    node._on_utterance(reply)
    assert node.last_turn is not None
    node.last_turn.result(timeout=10)
    assert node.test_llm.calls == 1
    assert node._session.snapshot.top("GreetingReply") is None  # окно сняла первая адресованная фраза


def test_wakeless_phrase_without_greeting_stays_in_backlog(node):
    msg = M.Utterance()
    msg.text, msg.source, msg.utterance_id = "привет", "respeaker", "u3"
    node._on_utterance(msg)
    assert node.last_turn is None and node.test_llm.calls == 0


# -- стартовая фраза ---------------------------------------------------------------------------------------------

def _startup_timer(made):
    return [t for t in made._timers if t.period == 12.0]


def test_startup_greeting_earcons_then_phrase(node):
    (timer,) = _startup_timer(node)
    timer.callback()
    earcons = [r.earcon for r in _published(node, "/voice/expression/request")]
    assert earcons[-1:] == ["heard"]
    step2 = node._timers[-1]
    assert step2.period == 2.0
    step2.callback()
    assert [r.earcon for r in _published(node, "/voice/expression/request")][-1:] == ["done"]
    step3 = node._timers[-1]
    assert step3.period == 1.5
    step3.callback()
    said = _said(node)
    assert len(said) == 1 and node._startup_greeting.phrase in said[0].text
    timer.callback()  # повторный тик — не вторая фраза
    assert len(_said(node)) == 1


def test_startup_greeting_skipped_when_silenced(node):
    node._session.push_scene("Silence", "user", ttl_s=60.0)
    _startup_timer(node)[0].callback()
    assert _published(node, "/voice/expression/request") == [] and _said(node) == []


def test_startup_greeting_off_by_zero(monkeypatch):
    node_cls = dn.DialogNode
    made = object.__new__(node_cls)
    params = {"startup_greeting_sec": 0.0}
    made.get_parameter = lambda name: types.SimpleNamespace(value=params[name])
    assert parity.arm_startup_greeting(made) is None
