"""JSON_CMD nav_goal / nav_cancel (issue #3151): гейт, ack/nack, отмена."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from rob_box_quest.core.nav_goal import NavGoalRequest
from rob_box_quest.protocol.frame import FrameType
from rob_box_quest.server.session import ClientSession
from rob_box_quest.server.ws_server import JSON_CMD_HANDLERS, NoOpBridge, WSSServer


class _NavBridge(NoOpBridge):
    def __init__(self, goal_reason=None, had_goal=True):
        self.goals: list[tuple[str, NavGoalRequest]] = []
        self.cancels: list[str] = []
        self._goal_reason = goal_reason
        self._had_goal = had_goal

    def nav_goal(self, client_id, req):
        self.goals.append((client_id, req))
        return self._goal_reason

    def nav_cancel(self, client_id):
        self.cancels.append(client_id)
        return self._had_goal


def _server(bridge, *, require_floor=False):
    server = WSSServer(bridge=bridge, pin="123456", require_teleop_floor=require_floor)
    server._send = AsyncMock()
    session = ClientSession(session_id="sess-nav")
    session.mark_authenticated("test", [])
    return server, SimpleNamespace(), session


def _goal(**over):
    body = {"cmd": "nav_goal", "ts_ms": 1, "seq": 4, "x": 1.0, "y": 2.0, "yaw": 0.25, "frame": "map"}
    body.update(over)
    return body


def _sent_event(server):
    args = server._send.await_args.args
    assert args[1] == FrameType.JSON_EVENT
    return args[3]


def test_nav_commands_registered():
    assert JSON_CMD_HANDLERS["nav_goal"].__name__ == "_json_cmd_nav_goal"
    assert JSON_CMD_HANDLERS["nav_cancel"].__name__ == "_json_cmd_nav_cancel"


@pytest.mark.asyncio
async def test_nav_goal_ack_and_forward():
    bridge = _NavBridge()
    server, ws, session = _server(bridge)
    await server._on_json_cmd(ws, session, _goal())
    assert bridge.goals == [(session.server_client_id, NavGoalRequest(seq=4, x=1.0, y=2.0, yaw=0.25))]
    ev = _sent_event(server)
    assert ev["type"] == "nav_goal_ack" and ev["seq"] == 4


@pytest.mark.asyncio
async def test_nav_goal_bad_frame_nack_not_forwarded():
    bridge = _NavBridge()
    server, ws, session = _server(bridge)
    await server._on_json_cmd(ws, session, _goal(frame="odom"))
    assert bridge.goals == []
    ev = _sent_event(server)
    assert ev == {"type": "nav_goal_nack", "reason": "bad_frame", "seq": 4, "ts_ms": ev["ts_ms"]}


@pytest.mark.asyncio
async def test_nav_goal_bad_payload_without_seq():
    server, ws, session = _server(_NavBridge())
    await server._on_json_cmd(ws, session, _goal(seq="x"))
    ev = _sent_event(server)
    assert ev["reason"] == "bad_payload"
    assert "seq" not in ev


@pytest.mark.asyncio
async def test_nav_goal_floor_gate_same_as_teleop():
    bridge = _NavBridge()
    server, ws, session = _server(bridge, require_floor=True)
    # Руль у другой сессии.
    server._avatar_arbiter.try_acquire_floor("other-session")
    await server._on_json_cmd(ws, session, _goal())
    assert bridge.goals == []
    assert _sent_event(server)["reason"] == "floor_held"


@pytest.mark.asyncio
async def test_nav_goal_passes_gate_when_floor_held_by_session():
    bridge = _NavBridge()
    server, ws, session = _server(bridge, require_floor=True)
    server._avatar_arbiter.try_acquire_floor(session.session_id)
    await server._on_json_cmd(ws, session, _goal())
    assert len(bridge.goals) == 1
    assert _sent_event(server)["type"] == "nav_goal_ack"


@pytest.mark.asyncio
async def test_nav_goal_bridge_reason_becomes_nack():
    server, ws, session = _server(_NavBridge(goal_reason="emergency_active"))
    await server._on_json_cmd(ws, session, _goal())
    assert _sent_event(server)["reason"] == "emergency_active"


@pytest.mark.asyncio
async def test_nav_goal_noop_bridge_is_honest_nack():
    server, ws, session = _server(NoOpBridge())
    await server._on_json_cmd(ws, session, _goal())
    assert _sent_event(server)["reason"] == "nav2_unavailable"


@pytest.mark.asyncio
async def test_nav_goal_bridge_exception_is_nack():
    class _Boom(NoOpBridge):
        def nav_goal(self, client_id, req):
            raise RuntimeError("x")

    server, ws, session = _server(_Boom())
    await server._on_json_cmd(ws, session, _goal())
    assert _sent_event(server)["reason"] == "nav2_unavailable"


@pytest.mark.asyncio
async def test_nav_cancel_bypasses_floor_gate():
    bridge = _NavBridge(had_goal=True)
    server, ws, session = _server(bridge, require_floor=True)
    server._avatar_arbiter.try_acquire_floor("other-session")
    await server._on_json_cmd(ws, session, {"cmd": "nav_cancel", "ts_ms": 1})
    assert bridge.cancels == [session.server_client_id]
    ev = _sent_event(server)
    assert ev["type"] == "nav_cancel_ack" and ev["had_goal"] is True


@pytest.mark.asyncio
async def test_nav_cancel_without_bridge_support():
    class _Old:
        def feed_client_alive(self):
            pass

    server, ws, session = _server(_Old())
    await server._on_json_cmd(ws, session, {"cmd": "nav_cancel", "ts_ms": 1})
    assert _sent_event(server)["had_goal"] is False
