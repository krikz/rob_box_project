"""Integration: JSON_CMD{list_voices} → JSON_EVENT{voices_list}.

Источник истины: задача t_7eba64d9 §1.
"""

import json
import time

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.server.ws_server import NoOpBridge, WSSServer, build_app


pytestmark = pytest.mark.asyncio


@pytest.fixture
def fixed_pin(monkeypatch):
    pin = "111111"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


@pytest.fixture
async def client(fixed_pin):
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        yield client, server


async def _open_ws(client):
    return await client.ws_connect("/quest")


async def _send_hello(ws, pin):
    payload = json.dumps({
        "client_version": "0.1.0",
        "capabilities": ["webxr"],
        "session_pin": pin,
    }).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.HELLO, 0, payload))


async def _drain_until_welcome(ws, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = await ws.receive()
        if msg.type == WSMsgType.CLOSE:
            return False
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, payload = decode_frame(msg.data)
            if ftype == FrameType.WELCOME:
                return True
    return False


async def _next_event(ws, event_type, timeout=1.0):
    """Читаем JSON_EVENT пока не получим нужный type."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = await ws.receive(timeout=0.1)
        if msg.type == WSMsgType.CLOSE:
            return None
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, payload = decode_frame(msg.data)
            if ftype == FrameType.JSON_EVENT:
                body = json.loads(payload.decode("utf-8"))
                if body.get("type") == event_type:
                    return body
    return None


async def _next_error(ws, code=None, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = await ws.receive(timeout=0.1)
        if msg.type == WSMsgType.CLOSE:
            return None
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, payload = decode_frame(msg.data)
            if ftype == FrameType.ERROR:
                body = json.loads(payload.decode("utf-8"))
                if code is None or body.get("code") == code:
                    return body
    return None


async def test_list_voices_returns_at_least_4_voices(client, fixed_pin):
    """Acceptance: list_voices → ≥4 голосов + 4 пресета."""
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "list_voices"}).encode("utf-8"),
        ))
        evt = await _next_event(ws, "voices_list")
        assert evt is not None
        assert isinstance(evt["voices"], list)
        assert len(evt["voices"]) >= 4
        assert isinstance(evt["presets"], list)
        assert len(evt["presets"]) >= 4
        # Структура voice dict
        v0 = evt["voices"][0]
        assert {"id", "name", "gender", "language", "description"} <= v0.keys()
        # Структура preset dict
        p0 = evt["presets"][0]
        assert {"id", "name", "description"} <= p0.keys()
        # Пресеты включают 4 обязательных
        preset_ids = {p["id"] for p in evt["presets"]}
        assert {"standard", "friendly", "authoritative", "whisper"} <= preset_ids
    finally:
        try:
            await ws.close()
        except Exception:
            pass


async def test_list_voices_is_idempotent(client, fixed_pin):
    """Двойной вызов — одинаковый ответ."""
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        for _ in range(2):
            await ws.send_bytes(encode_frame(
                FrameType.JSON_CMD, 0,
                json.dumps({"cmd": "list_voices"}).encode("utf-8"),
            ))
            evt = await _next_event(ws, "voices_list")
            assert evt is not None
            assert len(evt["voices"]) >= 4
    finally:
        try:
            await ws.close()
        except Exception:
            pass