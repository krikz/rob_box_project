"""Integration-тесты голосового passthrough (рация) в WS-сервере.

Проверяет контракт (план P3, Task 3.2):
- voice_ptt_start → Bridge.publish_voice_barge_in() (STOP TTS + sound)
- VOICE_AUDIO frame → Bridge.publish_voice_audio(payload)
- voice_ptt_stop → Bridge.publish_voice_stop()

Без rclpy/ROS/Zenoh — подменяем Bridge на записывающий stub.
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


class RecordingBridge(NoOpBridge):
    """Stub, записывающий вызовы голосовых методов Bridge."""

    def __init__(self) -> None:
        self.barge_in_calls = 0
        self.voice_audio_payloads: list[bytes] = []
        self.voice_stop_calls = 0

    def publish_voice_barge_in(self) -> None:
        self.barge_in_calls += 1

    def publish_voice_audio(self, payload: bytes) -> None:
        self.voice_audio_payloads.append(payload)

    def publish_voice_stop(self) -> None:
        self.voice_stop_calls += 1


@pytest.fixture
def bridge() -> RecordingBridge:
    return RecordingBridge()


@pytest.fixture
def fixed_pin(monkeypatch) -> str:
    pin = "123456"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


@pytest.fixture
async def client(fixed_pin, bridge):
    server = WSSServer(bridge=bridge, pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        yield client, server, bridge


async def _open_and_hello(client, pin):
    """Открыть WS, HELLO, дождаться WELCOME."""
    ws = await client.ws_connect("/quest")
    payload = json.dumps(
        {"client_version": "0.1.0", "capabilities": ["webxr"], "session_pin": pin}
    ).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.HELLO, 0, payload))
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        msg = await ws.receive()
        if msg.type == WSMsgType.CLOSE:
            pytest.fail("closed before WELCOME")
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, _p = decode_frame(msg.data)
            if ftype == FrameType.WELCOME:
                return ws
    pytest.fail("WELCOME not received")


async def _send_json_cmd(ws, cmd_obj) -> None:
    payload = json.dumps(cmd_obj).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))


async def test_voice_ptt_start_triggers_barge_in(client, fixed_pin):
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_ptt_start", "ts_ms": 0})
        await asyncio.sleep(0.05)
        assert bridge.barge_in_calls == 1
    finally:
        await ws.close()


async def test_voice_audio_frame_publishes_payload(client, fixed_pin):
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        pcm = b"\x00\x00\xff\x7f\x00\x80"  # int16 LE: 0, 32767, -32768
        await ws.send_bytes(encode_frame(FrameType.VOICE_AUDIO, 0, pcm))
        await asyncio.sleep(0.05)
        assert bridge.voice_audio_payloads == [pcm]
    finally:
        await ws.close()


async def test_voice_ptt_stop_calls_stop(client, fixed_pin):
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_ptt_stop", "ts_ms": 0})
        await asyncio.sleep(0.05)
        assert bridge.voice_stop_calls == 1
    finally:
        await ws.close()
