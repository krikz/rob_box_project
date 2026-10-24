"""``say_aloud`` - озвучка из чата Telegram с итогом по ``SpeechEvent`` (ADR-0150 §5.1, §9.2.3; PR-12b #3707).

Успех - событие владельца речи, а не публикация запроса: началась - ``pending`` с ``event_ref=speech_id``, отказ -
``rejected{reason}``; реплика не прерываема перебиванием других агентов; срез ``telegram`` доступен только агенту чата.
"""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

try:
    import rclpy  # noqa: F401
except ImportError:
    for _mod in ("rob_box_mcp_tools.tools.navigation", "rob_box_mcp_tools.tools.system",
                 "rob_box_mcp_tools.tools.perception", "rob_box_mcp_tools.tools.mapping",
                 "rob_box_mcp_tools.tools.memory", "rob_box_mcp_tools.tools.music"):
        sys.modules.setdefault(_mod, Mock())

from rob_box_mcp_tools.mcp_auth import DEFAULT_ALLOWED_SENDERS, PERSONALITY_SENDER, TELEGRAM_SENDER  # noqa: E402
from rob_box_mcp_tools.slice_authority import load_default_authority  # noqa: E402
from rob_box_mcp_tools.tools.say import SayAloudTool  # noqa: E402

M = install_dialog_msgs_stub()
STARTED, FINISHED, REJECTED = 2, 4, 7


class Owner:
    """Владелец речи в тесте: на каждый ``SpeechRequest`` отвечает заданными событиями (синхронно, как поток ROS)."""

    def __init__(self, events):
        self.events, self.requests, self.callback = events, [], None

    def on_publish(self, req):
        self.requests.append(req)
        for kind, reason, seconds in self.events:
            self.callback(SimpleNamespace(kind=kind, speech_id=req.speech_id, agent="telegram", reason=reason,
                                          seconds=seconds))


def make_tool(events):
    owner = Owner(events)
    node = MagicMock()
    pub = MagicMock()
    pub.publish.side_effect = owner.on_publish
    node.create_publisher.return_value = pub

    def subscribe(msg_type, topic, callback, *_a, **_k):
        owner.callback = callback

    node.create_subscription.side_effect = subscribe
    stamp = type(M.SpeechRequest().stamp)()
    node.get_clock.return_value = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))
    return SayAloudTool(node), owner


def test_started_means_pending_with_event_ref_not_done():
    tool, owner = make_tool([(STARTED, "", 0.0)])

    result = tool.execute(text="Жил-был робот")

    req = owner.requests[0]
    assert result.success is True
    assert result.data["status"] == "pending"
    assert result.data["event_ref"] == req.speech_id
    assert req.agent == "telegram" and req.interruptible is False
    assert req.text == "<speak>Жил-был робот</speak>"


def test_rejected_by_owner_is_a_failure_with_the_owner_reason():
    tool, _owner = make_tool([(REJECTED, "unsupported_script", 0.0)])

    result = tool.execute(text="Hello")

    assert result.success is False
    assert result.data["status"] == "rejected" and result.data["reason"] == "unsupported_script"
    assert "event_ref" not in result.data


def test_very_short_reply_finishing_at_once_is_done_with_seconds():
    tool, _owner = make_tool([(FINISHED, "", 2.4)])

    result = tool.execute(text="Привет")

    assert result.data["status"] == "done" and result.data["seconds"] == 2


def test_no_event_in_time_is_pending_not_a_claim_of_success(monkeypatch):
    monkeypatch.setattr("rob_box_mcp_tools.tools.say.START_WAIT_S", 0.01)
    tool, _owner = make_tool([])

    result = tool.execute(text="Привет")

    assert result.data["status"] == "pending" and result.data["event_ref"]


def test_empty_text_publishes_nothing():
    tool, owner = make_tool([])

    result = tool.execute(text="  ")

    assert result.success is False and result.data["status"] == "rejected"
    assert owner.requests == []


def test_voice_goes_into_the_request():
    tool, owner = make_tool([(STARTED, "", 0.0)])

    tool.execute(text="Привет", voice="alena")

    assert owner.requests[0].voice == "alena"


def test_telegram_slice_belongs_to_the_chat_agent_only():
    authority = load_default_authority()
    assert authority.is_allowed(TELEGRAM_SENDER, "say_aloud").allowed
    assert authority.is_allowed(TELEGRAM_SENDER, "get_current_time").allowed
    assert not authority.is_allowed(PERSONALITY_SENDER, "say_aloud").allowed
    assert not authority.is_allowed("avatar_supervisor", "say_aloud").allowed
    assert TELEGRAM_SENDER in DEFAULT_ALLOWED_SENDERS
    assert not authority.is_allowed(TELEGRAM_SENDER, "say").allowed


@pytest.mark.parametrize("flag", ["telegram_visible"])
def test_catalog_marks_say_aloud_chat_only(flag):
    from rob_box_core.tool_catalog import get_tool, llm_visible_tools, operator_visible_tools, telegram_visible_tools

    entry = get_tool("say_aloud")
    assert getattr(entry, flag) is True and entry.llm_visible is False and entry.operator_visible is False
    assert "say_aloud" in {e.name for e in telegram_visible_tools()}
    assert "say_aloud" not in {e.name for e in llm_visible_tools()}
    assert "say_aloud" not in {e.name for e in operator_visible_tools()}
