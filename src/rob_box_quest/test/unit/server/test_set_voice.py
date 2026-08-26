"""Integration: JSON_CMD{set_voice} → JSON_EVENT{voice_changed}.

Источник истины: задача t_7eba64d9 §2.
"""

import asyncio
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
    pin = "222222"
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
    """Читаем JSON_EVENT пока не получим нужный type.

    Параллельно шлёт JSON_EVENT{type:"ping"} чтобы watchdog (0.6с) не
    убил сессию во время долгих wait'ов (>0.6с).
    """
    import asyncio

    deadline = time.monotonic() + timeout
    ping_task = asyncio.create_task(
        _fire_ping_periodically(ws, timeout, 0.2)
    )
    try:
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
    finally:
        ping_task.cancel()
        try:
            await ping_task
        except (asyncio.CancelledError, Exception):
            pass
    return None


async def _fire_ping_periodically(ws, duration, interval):
    """Шлёт JSON_EVENT{type:"ping"} каждые interval секунд пока не истечёт duration."""
    import asyncio
    end = asyncio.get_event_loop().time() + duration
    try:
        while True:
            now = asyncio.get_event_loop().time()
            if now >= end:
                break
            await asyncio.sleep(min(interval, end - now))
            payload = json.dumps({"type": "ping", "ts_ms": 0}).encode("utf-8")
            await ws.send_bytes(encode_frame(FrameType.JSON_EVENT, 0, payload))
    except asyncio.CancelledError:
        pass


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


async def test_set_voice_valid_returns_ack(client, fixed_pin):
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "set_voice", "voice_id": "anton",
                        "preset": "friendly"}).encode("utf-8"),
        ))
        evt = await _next_event(ws, "voice_changed")
        assert evt is not None
        assert evt["voice_id"] == "anton"
        assert evt["preset"] == "friendly"
        assert "ts_ms" in evt
    finally:
        try:
            await ws.close()
        except Exception:
            pass


async def test_set_voice_default_preset(client, fixed_pin):
    """Без preset → default 'standard'."""
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "set_voice", "voice_id": "alena"}).encode("utf-8"),
        ))
        evt = await _next_event(ws, "voice_changed")
        assert evt is not None
        assert evt["voice_id"] == "alena"
        assert evt["preset"] == "standard"
    finally:
        try:
            await ws.close()
        except Exception:
            pass


async def test_set_voice_unknown_returns_voice_unknown_error(client, fixed_pin):
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "set_voice", "voice_id": "no_such_voice_xyz"}
                      ).encode("utf-8"),
        ))
        err = await _next_error(ws, code="VOICE_UNKNOWN")
        assert err is not None
        assert "not in catalog" in err["message"]
    finally:
        try:
            await ws.close()
        except Exception:
            pass


async def test_set_voice_missing_voice_id_returns_bad_payload(client, fixed_pin):
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "set_voice"}).encode("utf-8"),
        ))
        err = await _next_error(ws, code="BAD_PAYLOAD")
        assert err is not None
    finally:
        try:
            await ws.close()
        except Exception:
            pass


async def test_set_voice_unknown_preset_returns_bad_payload(client, fixed_pin):
    """Неизвестный preset → BAD_PAYLOAD (не VOICE_UNKNOWN)."""
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "set_voice", "voice_id": "anton",
                        "preset": "bogus_preset"}).encode("utf-8"),
        ))
        err = await _next_error(ws, code="BAD_PAYLOAD")
        assert err is not None
    finally:
        try:
            await ws.close()
        except Exception:
            pass


async def test_set_voice_persists_in_state_for_voice_state_loop(client, fixed_pin):
    """После set_voice, сервер шлёт немедленный voice_state event.

    Не ждём 1 Hz tick (он бы занял ~1с и под нагрузкой watchdog мог
    сработать) — сервер шлёт event сразу после apply.
    """
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "set_voice", "voice_id": "zahar",
                        "preset": "whisper"}).encode("utf-8"),
        ))
        # Первый event — voice_changed ACK, второй — voice_state (немедленный).
        # _next_event(voice_state) пропустит heartbeat/voice_changed и поймает
        # именно voice_state.
        evt = await _next_event(ws, "voice_state", timeout=2.0)
        assert evt is not None
        assert evt["active_voice_id"] == "zahar"
        assert evt["active_preset"] == "whisper"
        assert evt["listening"] is False
        assert evt["last_error"] is None
    finally:
        try:
            await ws.close()
        except Exception:
            pass