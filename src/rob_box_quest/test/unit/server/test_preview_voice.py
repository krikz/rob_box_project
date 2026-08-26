"""Integration: JSON_CMD{preview_voice} → BINARY_FRAME c opus payload.

Источник истины: задача t_7eba64d9 §3 + rate limit 3/10sec.
"""

import json
import time

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.protocol.topics import TOPIC_IDS
from rob_box_quest.server.ws_server import NoOpBridge, WSSServer, build_app
from rob_box_quest.voice import (
    HardcodedVoiceProvider,
    VoiceCatalog,
    VoicePreset,
    VoiceProvider,
)


pytestmark = pytest.mark.asyncio


class StubTTSProvider:
    """Подменяем Yandex: возвращаем фиксированный opus-payload."""

    def __init__(self, payload=b"\x01\x02\x03\x04opus_stub"):
        self._payload = payload
        self.calls: list[tuple[str, str, str]] = []

    def synthesize(self, *, voice_id, text, preset):
        self.calls.append((voice_id, text, preset.id))
        return self._payload


@pytest.fixture
def fixed_pin(monkeypatch):
    pin = "333333"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


@pytest.fixture
async def client_with_stub_tts(fixed_pin):
    """WS-server с подменённым TTS-провайдером (возвращает предсказуемый opus)."""
    stub = StubTTSProvider()
    server = WSSServer(
        bridge=NoOpBridge(),
        pin=fixed_pin,
        voice_provider=stub,
    )
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        yield client, server, stub


@pytest.fixture
async def client_with_failing_tts(fixed_pin):
    """WS-server с TTS-провайдером, который возвращает None."""
    class FailingProvider:
        def synthesize(self, *, voice_id, text, preset):
            return None

    server = WSSServer(
        bridge=NoOpBridge(),
        pin=fixed_pin,
        voice_provider=FailingProvider(),
    )
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


async def _next_binary(ws, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = await ws.receive(timeout=0.1)
        if msg.type == WSMsgType.CLOSE:
            return None
        if msg.type == WSMsgType.BINARY:
            return msg.data
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


async def test_preview_voice_returns_audio_with_topic_tag(
    client_with_stub_tts, fixed_pin
):
    http_client, _server, stub = client_with_stub_tts
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "preview_voice", "voice_id": "anton",
                        "text": "Привет", "preset": "standard"}
                      ).encode("utf-8"),
        ))
        raw = await _next_binary(ws)
        assert raw is not None
        ftype, sid, payload = decode_frame(raw)
        assert ftype == FrameType.BINARY_FRAME
        # stream_id = 0 (control-stream для preview).
        assert sid == 0
        # payload = [4 bytes topic_id LE][opus bytes]
        topic_id = int.from_bytes(payload[:4], "little")
        assert topic_id == TOPIC_IDS["voice_audio_preview"]
        assert payload[4:] == b"\x01\x02\x03\x04opus_stub"
        # Провайдер был вызван с правильными аргументами.
        assert stub.calls == [("anton", "Привет", "standard")]
    finally:
        try:
            await ws.close()
        except Exception:
            pass


async def test_preview_voice_unknown_voice_returns_voice_unknown(
    client_with_stub_tts, fixed_pin
):
    http_client, _server, _stub = client_with_stub_tts
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "preview_voice", "voice_id": "no_such_xyz",
                        "text": "x"}).encode("utf-8"),
        ))
        err = await _next_error(ws, code="VOICE_UNKNOWN")
        assert err is not None
    finally:
        try:
            await ws.close()
        except Exception:
            pass


async def test_preview_voice_missing_text_returns_bad_payload(
    client_with_stub_tts, fixed_pin
):
    http_client, _server, _stub = client_with_stub_tts
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "preview_voice", "voice_id": "anton"}
                      ).encode("utf-8"),
        ))
        err = await _next_error(ws, code="BAD_PAYLOAD")
        assert err is not None
    finally:
        try:
            await ws.close()
        except Exception:
            pass


async def test_preview_voice_provider_failure_returns_internal(
    client_with_failing_tts, fixed_pin
):
    """Если TTS-провайдер вернул None — ERROR{INTERNAL}, сессия жива."""
    http_client, _server = client_with_failing_tts
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "preview_voice", "voice_id": "anton",
                        "text": "Привет"}).encode("utf-8"),
        ))
        err = await _next_error(ws, code="INTERNAL")
        assert err is not None
        # Сокет НЕ закрыт — сервер остался жив.
        # Проверяем что можем слать другую команду.
        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "list_voices"}).encode("utf-8"),
        ))
        # Должны получить ping heartbeat или voices_list — сокет живой.
        import asyncio
        msg = await asyncio.wait_for(ws.receive(), timeout=2.0)
        assert msg.type != WSMsgType.CLOSE
    finally:
        try:
            await ws.close()
        except Exception:
            pass


async def test_preview_voice_rate_limit_3_per_10sec(
    client_with_stub_tts, fixed_pin
):
    """Acceptance: rate limit 3/10sec — 4-й запрос → ERROR{RATE_LIMIT}."""
    http_client, _server, _stub = client_with_stub_tts
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        assert await _drain_until_welcome(ws)

        # 3 запроса подряд — все успешны (читаем audio или drain до BINARY).
        for i in range(3):
            await ws.send_bytes(encode_frame(
                FrameType.JSON_CMD, 0,
                json.dumps({"cmd": "preview_voice", "voice_id": "anton",
                            "text": f"req{i}"}).encode("utf-8"),
            ))
            raw = await _next_binary(ws, timeout=2.0)
            assert raw is not None, f"no audio for request {i}"

        # 4-й — RATE_LIMIT.
        await ws.send_bytes(encode_frame(
            FrameType.JSON_CMD, 0,
            json.dumps({"cmd": "preview_voice", "voice_id": "anton",
                        "text": "req4"}).encode("utf-8"),
        ))
        err = await _next_error(ws, code="RATE_LIMIT")
        assert err is not None
        assert "limit" in err["message"].lower()
    finally:
        try:
            await ws.close()
        except Exception:
            pass