"""#4076: ``dialog_node`` ведёт журнал смены — голос собеседника, снимок плеера, исходы хода, встреча лицом — и отдаёт
его модели разделом ``shift_log`` на вопрос о смене (марафон 27.10, n1001 «подведи итог ночи»).

Хост на заглушках rclpy (conftest) с фейковыми LLM/тулами, как ``test_dialog_node.py``; правила журнала —
``rob_box_dialog/test/test_shift_log.py``, здесь — что хост их проводит.
"""

from __future__ import annotations

import json
import sys
import time
import types
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()
try:  # настоящий rosidl-пакет (CI) или заглушка сервиса DialogControl (Windows) — как в test_dialog_node.py
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

from rob_box_dialog.respond import ActionResult  # noqa: E402
from rob_box_dialog_msgs.srv import DialogControl  # noqa: E402
from rob_box_llm.provider import LLMResponse, ToolResult  # noqa: E402
from rob_box_voice import dialog_node as dn  # noqa: E402
from rob_box_voice.core import dialog_v2_parity as parity  # noqa: E402

TIME = type(M.TurnEvent().stamp)


class _LLM:
    def __init__(self, responses):
        self.responses, self.messages = list(responses), []
        self.answered_by = "deepseek"

    async def complete(self, messages, tools=(), settings=None):
        self.messages.append(messages)
        return self.responses.pop(0)


class _Tools:
    async def execute(self, call):
        return ToolResult(tool_call_id=call.id, content="ok", data={"status": "done"})


@pytest.fixture()
def node(monkeypatch):
    llm = _LLM([LLMResponse(content="Ночью приходил Борис."), LLMResponse(content="Хорошо.")])
    tools = _Tools()
    monkeypatch.setattr(dn, "build_llm", lambda node, counter, agent="personality": (llm, "Ты робот."))
    monkeypatch.setattr(dn, "build_tools", lambda node: tools)
    monkeypatch.setattr(dn, "build_chat_tools", lambda node, personality_tools: tools)
    made = dn.DialogNode()
    stamp = TIME(sec=1, nanosec=0)
    made.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))
    made.test_llm = llm
    yield made
    made.shutdown()


def _voice(payload):
    from rob_box_core.speaker_wire import fill_speaker_result
    return fill_speaker_result(M.SpeakerResult(), payload)


def _boris(uid):
    return _voice({"is_known": True, "speaker_id": "spk-boris", "name": "Борис", "confidence": 0.9,
                   "utterance_id": uid})


def _utterance(text, uid):
    msg = M.Utterance()
    msg.text, msg.source, msg.utterance_id = text, "respeaker", uid
    end = time.time()
    msg.started_at, msg.ended_at = TIME(sec=int(end - 3), nanosec=0), TIME(sec=int(end), nanosec=0)
    return msg


def _music(state, track_id=None, **dj):
    msg = MagicMock()
    msg.data = json.dumps({"state": state, "track_id": track_id, "dj": {"enabled": False, **dj}, "ts": time.time()})
    return msg


def _asked(made, text, uid):
    made._on_utterance(_utterance(text, uid))
    made.last_turn.result(timeout=10)
    return json.dumps(made.test_llm.messages[-1], ensure_ascii=False, default=str)


def test_night_question_gets_the_shift_journal_even_after_a_new_session(node):
    """Борис говорил, играл трек, потом «новая сессия» — на «подведи итог ночи» модель видит журнал смены: Бориса и
    трек, с фразой-итогом кода."""
    node._on_speaker(_boris("u-b"))
    node._on_music_state(_music("playing", "t1", name="Песня про енота", source="synth"))
    node._on_music_state(_music("idle"))
    node._on_control(DialogControl.Request(action=DialogControl.Request.ACTION_RESET, set_by="operator"),
                     types.SimpleNamespace())
    node._on_speaker(_boris("u-q"))
    blob = _asked(node, "Робот, подведи итог ночи: кто к нам приходил и что мы слушали?", "u-q")
    journal = blob.split("<shift_log>")[1].split("</shift_log>")[0]
    assert "Борис" in journal and "Песня про енота" in journal and "synth" in journal
    assert "Журнал смены за последние" in journal


def test_ordinary_turn_does_not_carry_the_journal(node):
    node._on_speaker(_boris("u-b"))
    blob = _asked(node, "Робот, как дела?", "u-b")
    assert "shift_log" not in blob


def test_turn_outcomes_and_face_meetings_are_written_to_the_shift(node):
    """Исход действия хода (журнал ходов → ``on_outcome``) и встреча лицом (``parity.on_vision_event``) — записи смены."""
    node._engine.log.begin("turn-x", 0, "включи музыку")
    node._engine.log.result("turn-x", ActionResult("turn-x/0", 0, "personality", "request_music", "action.media",
                                                   "done"))
    face = {"encounter": "start", "person_id": "f-grisha", "name": "Гриша", "is_new": False, "similarity": 0.9,
            "encounter_count": 1}
    parity.on_vision_event(node, types.SimpleNamespace(event_type="face", attributes_json=json.dumps(face),
                                                       source_camera="oak_d"))
    view = node._shift.view()
    assert view["actions"] == {"request_music": {"done": 1}}
    assert [(p["name"], p["via"]) for p in view["people"]] == [("Гриша", "face")]
