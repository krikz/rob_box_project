"""Regression tests for the table-driven JSON_CMD dispatcher (issue #2195)."""

from __future__ import annotations

import ast
import inspect
import logging
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

import pytest

from rob_box_quest.server.session import ClientSession, ErrorCode
from rob_box_quest.server.ws_server import JSON_CMD_HANDLERS, NoOpBridge, WSSServer


EXPECTED_JSON_COMMANDS = {
    "ping",
    "stream_list",
    "stream_select",
    "teleop_twist",
    "teleop_heartbeat",
    "stop_emergency",
    "voice_ptt_start",
    "voice_ptt_stop",
    "voice_mode",
    "voice_listen_start",
    "voice_listen_stop",
    "supervisor_set_mode",
    "supervisor_acquire_floor",
    "supervisor_release_floor",
    "supervisor_get_state",
    "list_voices",
    "set_voice",
    "voice_pipeline",
    "preview_voice",
}


@pytest.fixture
def dispatcher():
    server = WSSServer(bridge=NoOpBridge(), pin="123456")
    server._send = AsyncMock()
    server._send_error = AsyncMock()
    session = ClientSession(session_id="session-json-cmd")
    return server, SimpleNamespace(), session


def test_json_cmd_handler_table_covers_existing_dispatch_contract() -> None:
    assert set(JSON_CMD_HANDLERS) == EXPECTED_JSON_COMMANDS
    assert len(set(JSON_CMD_HANDLERS.values())) >= 15


def test_on_json_cmd_is_only_a_table_dispatcher() -> None:
    import textwrap

    source = textwrap.dedent(inspect.getsource(WSSServer._on_json_cmd))
    node = ast.parse(source)
    decisions = sum(
        isinstance(
            item,
            (
                ast.If,
                ast.For,
                ast.While,
                ast.Try,
                ast.BoolOp,
                ast.IfExp,
                ast.Match,
            ),
        )
        for item in ast.walk(node)
    )
    assert decisions <= 2


@pytest.mark.asyncio
async def test_json_cmd_dispatches_registered_handler(dispatcher, monkeypatch) -> None:
    server, ws, session = dispatcher
    handler = AsyncMock()
    monkeypatch.setitem(JSON_CMD_HANDLERS, "ping", handler)

    await server._on_json_cmd(ws, session, {"cmd": "ping", "ts_ms": 1})

    handler.assert_awaited_once_with(server, ws, session, {"cmd": "ping", "ts_ms": 1})


@pytest.mark.asyncio
async def test_unknown_json_cmd_returns_terminal_error_and_warning(
    dispatcher, caplog
) -> None:
    server, ws, session = dispatcher

    with caplog.at_level(logging.WARNING):
        await server._on_json_cmd(ws, session, {"cmd": "not_registered"})

    server._send_error.assert_awaited_once_with(
        ws,
        0,
        ErrorCode.UNKNOWN_COMMAND,
        "unknown JSON_CMD: 'not_registered'",
    )
    assert any(
        "not_registered" in record.message
        and "session-json-cmd" in record.message
        for record in caplog.records
    )


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("mode", "expected_bridge_method"),
    [
        ("voice", "set_voice"),
        ("style", "set_voice_preset"),
    ],
)
async def test_set_voice_uses_explicit_mode(
    dispatcher, monkeypatch, mode, expected_bridge_method
) -> None:
    server, ws, session = dispatcher
    monkeypatch.setattr(server, "_voice_rate_limit_check", lambda *_args: True)
    server.bridge.set_voice = MagicMock(return_value=(True, "technical", "", []))
    server.bridge.set_voice_preset = MagicMock()
    server.bridge.set_voice_language = MagicMock()
    payload = {
        "cmd": "set_voice",
        "mode": mode,
        "voice_id": "technical",
        "preset": "technical",
    }

    await JSON_CMD_HANDLERS["set_voice"](server, ws, session, payload)

    getattr(server.bridge, expected_bridge_method).assert_called_once()
    other_method = "set_voice_preset" if expected_bridge_method == "set_voice" else "set_voice"
    getattr(server.bridge, other_method).assert_not_called()


@pytest.mark.asyncio
async def test_set_voice_rejects_missing_explicit_mode(dispatcher) -> None:
    server, ws, session = dispatcher

    await JSON_CMD_HANDLERS["set_voice"](
        server,
        ws,
        session,
        {"cmd": "set_voice", "voice_id": "alena"},
    )

    server._send_error.assert_awaited_once_with(
        ws,
        0,
        ErrorCode.BAD_PAYLOAD,
        "set_voice: mode must be 'voice' or 'style'",
    )
