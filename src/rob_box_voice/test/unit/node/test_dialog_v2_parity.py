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
from rob_box_dialog import phrases  # noqa: E402
from rob_box_dialog.startup import ReadinessWatch, assess  # noqa: E402

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
        msg.PerceptionEvent = type("PerceptionEvent", (), {})
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


# -- стартовая фраза по готовности узлов (#3913) -----------------------------------------------------------------

VOICE = ["/audio_node", "/stt_node", "/tts_node"]
REST = ["/context_aggregator", "/camera/camera", "/lslidar_driver_node", "/rtabmap/rtabmap"]


def _startup_timer(made):
    return [t for t in made._timers if t.period == 12.0]


def _report(active=(), failed=(), missing=()):
    return types.SimpleNamespace(active_nodes=list(active), failed_nodes=list(failed), missing_nodes=list(missing))


def _feed(made, report):
    (cb,) = [f.callback for t, f in made._subscribers if t == parity.READINESS_TOPIC]
    cb(report)


def _run_to_phrase(made):
    """12 с -> (опрос готовности) -> heard -> 2 с -> done -> 1,5 с -> фраза; возвращает сказанное."""
    _startup_timer(made)[0].callback()
    for period in (2.0, 1.5):
        step = made._timers[-1]
        assert step.period == period
        step.callback()
    return _said(made)


def test_readiness_subscription_exists(node):
    assert parity.READINESS_TOPIC == "/perception/context_update"
    assert parity.READINESS_TOPIC in {t for t, _ in node._subscribers}


def test_startup_greeting_all_up_says_ready_to_work(node):
    _feed(node, _report(active=["/dialog_node", *VOICE, *REST]))
    (timer,) = _startup_timer(node)
    timer.callback()
    assert [r.earcon for r in _published(node, "/voice/expression/request")][-1:] == ["heard"]
    step2 = node._timers[-1]
    assert step2.period == 2.0
    step2.callback()
    assert [r.earcon for r in _published(node, "/voice/expression/request")][-1:] == ["done"]
    step3 = node._timers[-1]
    assert step3.period == 1.5
    step3.callback()
    said = _said(node)
    assert len(said) == 1 and node._startup_greeting.phrase in said[0].text
    assert node._startup_greeting.verdict == "ready"
    assert node._startup_greeting.phrase in phrases.catalog()["startup.ready"]
    timer.callback()  # повторный тик - не вторая фраза
    assert len(_said(node)) == 1


def test_startup_greeting_waits_while_not_ready_then_says_voice_only_at_timeout(node, monkeypatch):
    now = [1000.0]
    greeting = node._startup_greeting
    monkeypatch.setattr(greeting, "_clock", lambda: now[0])
    greeting._deadline = now[0] + 90.0
    _feed(node, _report(active=["/dialog_node", *VOICE], missing=REST))
    _startup_timer(node)[0].callback()
    assert _published(node, "/voice/expression/request") == [] and _said(node) == []  # ждём: не всё поднялось
    poll = node._timers[-1]
    assert poll.period == 1.0
    now[0] += 30.0
    poll.callback()
    assert _published(node, "/voice/expression/request") == []  # 30 с - ещё ждём
    now[0] += 61.0  # вышло 90 с
    node._timers[-1].callback()
    assert [r.earcon for r in _published(node, "/voice/expression/request")] == ["heard"]
    node._timers[-1].callback()
    node._timers[-1].callback()
    said = _said(node)
    assert len(said) == 1 and "только голос" in said[0].text.lower()
    assert node._startup_greeting.verdict == "voice_only"


def test_startup_greeting_speaks_as_soon_as_everything_comes_up(node, monkeypatch):
    now = [1000.0]
    monkeypatch.setattr(node._startup_greeting, "_clock", lambda: now[0])
    node._startup_greeting._deadline = now[0] + 90.0
    _feed(node, _report(active=VOICE, missing=REST))
    _startup_timer(node)[0].callback()
    assert _said(node) == []
    now[0] += 5.0
    _feed(node, _report(active=VOICE + REST))
    node._timers[-1].callback()  # опрос: теперь всё поднялось - heard без ожидания 90 с
    assert [r.earcon for r in _published(node, "/voice/expression/request")] == ["heard"]


def test_startup_greeting_without_any_report_is_honest_unknown_after_timeout(node, monkeypatch):
    now = [1000.0]
    monkeypatch.setattr(node._startup_greeting, "_clock", lambda: now[0])
    node._startup_greeting._deadline = now[0] + 90.0
    _startup_timer(node)[0].callback()
    assert _said(node) == []
    now[0] += 91.0
    node._timers[-1].callback()
    for _ in range(2):
        node._timers[-1].callback()
    said = _said(node)
    assert len(said) == 1 and node._startup_greeting.verdict == "unknown"
    assert "готов к работе" not in said[0].text.lower()


def test_stale_report_is_not_a_fact():
    got = ReadinessWatch()
    parity.on_node_report(got, _report(active=VOICE + REST), clock=lambda: 100.0)
    assert assess(got.snapshot(105.0)).kind == "ready"
    assert assess(got.snapshot(200.0)).kind == "unknown"


def test_operator_text_overrides_the_verdict(node):
    node._startup_greeting._text = "Всем привет, я на связи"
    _feed(node, _report(active=VOICE, missing=REST))
    node._startup_greeting._deadline = 0.0  # время ожидания вышло
    said = _run_to_phrase(node)
    assert len(said) == 1 and "Всем привет, я на связи" in said[0].text and node._startup_greeting.verdict == "voice_only"


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
