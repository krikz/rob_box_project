"""#3853: v2-личность (ROS-нода ``dialog_node``) ходит в MCP под именем личности — подпись и срез те же, что у v1."""

from __future__ import annotations

from rob_box_mcp_tools.mcp_auth import DEFAULT_ALLOWED_SENDERS, PERSONALITY_SENDER, RequestAuthenticator
from rob_box_mcp_tools.slice_authority import load_default_authority

_TOKEN = "t" * 32


def _signed(sender: str) -> dict:
    request = {"tool_name": "get_current_time", "parameters": {}, "request_id": f"r-{sender}"}
    return RequestAuthenticator(_TOKEN, sender=sender).sign(request)


def test_verify_accepts_the_personality_sender_used_by_dialog_v2() -> None:
    server = RequestAuthenticator(_TOKEN)
    ok, error = server.verify(_signed(PERSONALITY_SENDER))
    assert (ok, error) == (True, "")


def test_ros_node_name_is_not_a_second_identity() -> None:
    assert "dialog_node" not in DEFAULT_ALLOWED_SENDERS
    assert "dialog_node" not in load_default_authority().known_senders


def test_personality_slice_covers_v2_tools() -> None:
    authority = load_default_authority()
    for tool in ("get_current_time", "get_battery_level"):
        assert authority.is_allowed(PERSONALITY_SENDER, tool).allowed, tool
    assert not authority.is_allowed(PERSONALITY_SENDER, "say").allowed
