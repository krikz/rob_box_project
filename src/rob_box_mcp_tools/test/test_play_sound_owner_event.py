"""play_sound: успех - событие владельца звука, а не публикация триггера (ADR-0150 §5.2, PR-7 ч.7b, #3698).

Раньше тул отвечал успехом сразу после ``publish``: занятый канал (``skipped{busy}``) или упавший
``sound_node`` LLM видела как успех. Теперь тул шлёт ``{sound, request_id}`` и ждёт ``SoundEvent`` с этим id.
"""

import json
import threading

from rob_box_dialog import knowledge as kn
from rob_box_dialog_msgs.msg import SoundEvent

from ._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools import sound as sound_module
    from rob_box_mcp_tools.tools.sound import PlaySoundTool
_ros_stubs = _ros.fixture()


class _Node:
    """Нода-заглушка: ``reply(body)`` - что ответит владелец звука на триггер (``None`` - молчит)."""

    def __init__(self, reply):
        self._reply, self.sent, self.subs = reply, [], {}

    def create_publisher(self, _typ, topic, _depth):
        node = self

        class _Pub:
            def publish(self, msg):
                body = json.loads(msg.data)
                node.sent.append((topic, body))
                for event in node._reply(body) or ():
                    threading.Thread(target=node.subs[sound_module.EVENT_TOPIC], args=(event,)).start()
        return _Pub()

    def create_subscription(self, _typ, topic, cb, _depth):
        self.subs[topic] = cb

    def get_logger(self):
        class _Log:
            def info(self, *_a, **_k):
                pass
            warning = error = debug = info
        return _Log()


def _event(kind, request_id, sound="robot_happy", reason=""):
    ev = SoundEvent()
    ev.kind, ev.request_id, ev.sound, ev.reason = kind, request_id, sound, reason
    return ev


def _tool(reply, monkeypatch, deadline=2.0):
    monkeypatch.setattr(sound_module, "sound_deadline_s", lambda: deadline)
    node = _Node(reply)
    tool = PlaySoundTool(node)
    tool._available_sounds, tool._duration_map = ["robot_happy", "ui_confirm"], {"robot_happy": 0.8}
    return tool, node


def test_trigger_carries_sound_and_request_id_and_started_is_success(monkeypatch):
    tool, node = _tool(lambda b: [_event(SoundEvent.KIND_STARTED, b["request_id"])], monkeypatch)
    out = tool.execute(sound="robot_happy")
    ((topic, body),) = node.sent
    assert topic == sound_module.TRIGGER_TOPIC and body["sound"] == "robot_happy" and body["request_id"]
    assert out.success and out.data["status"] == "done" and out.data["request_id"] == body["request_id"]
    assert out.message == "Звук robot_happy заиграл."


def test_busy_channel_is_an_honest_refusal_with_reason(monkeypatch):
    tool, _ = _tool(lambda b: [_event(SoundEvent.KIND_SKIPPED, b["request_id"], reason="busy")], monkeypatch)
    out = tool.execute(sound="robot_happy")
    assert not out.success and out.error == "busy" and out.data["status"] == "rejected"
    assert out.message == "Звук robot_happy не сыграл: другой звук ещё играет."


def test_unknown_reason_falls_back_to_the_generic_refusal(monkeypatch):
    tool, _ = _tool(lambda b: [_event(SoundEvent.KIND_SKIPPED, b["request_id"], reason="cosmic_rays")], monkeypatch)
    assert tool.execute(sound="robot_happy").message == "Звук robot_happy не сыграл."


def test_silent_owner_is_timeout_not_success(monkeypatch):
    tool, _ = _tool(lambda b: None, monkeypatch, deadline=0.2)
    out = tool.execute(sound="robot_happy")
    assert not out.success and out.data["status"] == "timeout"
    assert out.message == "Не дождался ответа от звука robot_happy — не знаю, сыграл ли."


def test_event_of_another_request_is_not_ours(monkeypatch):
    tool, _ = _tool(lambda b: [_event(SoundEvent.KIND_STARTED, "someone-else")], monkeypatch, deadline=0.2)
    assert tool.execute(sound="robot_happy").data["status"] == "timeout"


def test_finished_without_seen_started_still_means_it_played(monkeypatch):
    tool, _ = _tool(lambda b: [_event(SoundEvent.KIND_FINISHED, b["request_id"])], monkeypatch)
    assert tool.execute(sound="robot_happy").data["status"] == "done"


def test_unknown_sound_is_rejected_without_asking_the_owner(monkeypatch):
    tool, node = _tool(lambda b: None, monkeypatch)
    out = tool.execute(sound="kazoo")
    assert not out.success and node.sent == []


def test_deadline_is_the_class_deadline_from_knowledge():
    assert sound_module.sound_deadline_s() == kn.ACTION_CLASSES[kn.TOOL_CLASSES["play_sound"]].timeout_s


def test_catalog_names_the_owner_event_source():
    from rob_box_core.tool_catalog import get_tool

    assert get_tool("play_sound").event_source == sound_module.EVENT_TOPIC == "/voice/sound/event"
