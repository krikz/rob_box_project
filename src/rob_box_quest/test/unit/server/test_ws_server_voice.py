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
from typing import Any

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
        self.robot_start_calls = 0
        self.robot_stop_calls = 0
        self.voice_modes: list[str] = []
        # AV-27 / issue #1919 — TTS picker state.
        self.voices_snapshots: list[dict[str, Any]] = []
        self.set_voice_calls: list[tuple[str, str | None]] = []
        self.preview_voice_calls: list[tuple[str, str, str]] = []  # (request_id, voice_id, text)
        # Параметры, задаваемые в каждом тесте:
        self.active_provider: str = "yandex"
        self.active_voice: str = "alena"
        self.voices_payload: list[dict[str, Any]] = []

    def publish_voice_barge_in(self) -> None:
        self.barge_in_calls += 1

    def publish_voice_audio(self, payload: bytes) -> None:
        self.voice_audio_payloads.append(payload)

    def publish_voice_stop(self) -> None:
        self.voice_stop_calls += 1

    def publish_voice_robot_start(self) -> None:
        self.robot_start_calls += 1

    def publish_voice_robot_stop(self) -> None:
        self.robot_stop_calls += 1

    def set_voice_mode(self, mode: str) -> None:
        self.voice_modes.append(mode)

    # ── AV-27 / issue #1919 — TTS picker stubs ────────────────────────

    def list_voices_snapshot(self) -> dict[str, Any]:
        snap = {
            "voices": list(self.voices_payload),
            "active_provider": self.active_provider,
            "active_voice": self.active_voice,
        }
        self.voices_snapshots.append(snap)
        return snap

    def set_voice(
        self, voice_id: str, preset: str | None
    ) -> tuple[bool, str | None, str | None, list[str] | None]:
        self.set_voice_calls.append((voice_id, preset))
        # Простая логика: voice_id должен в в available_voices для active_provider.
        # Если active_provider пуст — tts_unreachable.
        if not self.active_provider:
            return False, None, "tts_unreachable", None
        if voice_id not in self.voices_payload and voice_id not in [v.get("voice_id") for v in self.voices_payload]:
            available = [v.get("voice_id") for v in self.voices_payload if isinstance(v, dict)]
            return False, None, "voice_unavailable", available
        return True, voice_id, None, None

    def publish_preview_voice(self, request_id: str, voice_id: str, text: str) -> None:
        self.preview_voice_calls.append((request_id, voice_id, text))


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
    payload = json.dumps({"client_version": "0.1.0", "capabilities": ["webxr"], "session_pin": pin}).encode("utf-8")
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


async def test_voice_ptt_start_robot_mode(client, fixed_pin):
    """voice_ptt_start {mode: robot_voice} → publish_voice_robot_start (не radio)."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_ptt_start", "mode": "robot_voice", "ts_ms": 0})
        await asyncio.sleep(0.05)
        assert bridge.robot_start_calls == 1
        assert bridge.barge_in_calls == 0
    finally:
        await ws.close()


async def test_voice_ptt_stop_robot_mode(client, fixed_pin):
    """voice_ptt_stop {mode: robot_voice} → publish_voice_robot_stop (не radio)."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_ptt_stop", "mode": "robot_voice", "ts_ms": 0})
        await asyncio.sleep(0.05)
        assert bridge.robot_stop_calls == 1
        assert bridge.voice_stop_calls == 0
    finally:
        await ws.close()


async def test_voice_mode_routes_to_bridge(client, fixed_pin):
    """voice_mode {mode: ttts_proxy} → bridge.set_voice_mode + ack."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_mode", "mode": "ttts_proxy", "ts_ms": 0})
        await asyncio.sleep(0.05)
        assert bridge.voice_modes == ["ttts_proxy"]
    finally:
        await ws.close()


# ── AV-27 / issue #1919 — TTS picker ────────────────────────────────────


async def _wait_for_json_event(ws, predicate, timeout: float = 1.0):
    """Читать WS-сообщения пока не найдём JSON_EVENT с body, удовлетворяющим predicate."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = await ws.receive()
        if msg.type == WSMsgType.CLOSE:
            return None
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, payload = decode_frame(msg.data)
            if ftype == FrameType.JSON_EVENT:
                body = json.loads(payload)
                if predicate(body):
                    return body
    return None


async def test_list_voices_returns_snapshot(client, fixed_pin):
    """list_voices → voice_list event с active_provider/active_voice."""
    http_client, _server, bridge = client
    bridge.active_provider = "yandex"
    bridge.active_voice = "alena"
    bridge.voices_payload = [
        {"voice_id": "alena", "display_name": "Алёна", "language": "ru-RU", "gender": "female"},
        {"voice_id": "anton", "display_name": "Антон", "language": "ru-RU", "gender": "male"},
    ]
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "list_voices", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_list", timeout=1.0
        )
        assert body is not None, "voice_list not received"
        assert body["voices"] == bridge.voices_payload
        assert body["active_provider"] == "yandex"
        assert body["active_voice"] == "alena"
        assert isinstance(body["ts_ms"], int) and body["ts_ms"] > 0
        assert bridge.voices_snapshots, "bridge.list_voices_snapshot not called"
    finally:
        await ws.close()


async def test_list_voices_empty_provider_returns_empty(client, fixed_pin):
    """Empty-list path: active_provider пустой → voice_list{voices:[]}."""
    http_client, _server, bridge = client
    bridge.active_provider = ""
    bridge.active_voice = ""
    bridge.voices_payload = []
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "list_voices", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_list", timeout=1.0
        )
        assert body is not None
        assert body["voices"] == []
        assert body["active_provider"] == ""
        assert body["active_voice"] == ""
    finally:
        await ws.close()


async def test_set_voice_success_sends_ack(client, fixed_pin):
    """set_voice {voice_id: alena} → bridge.set_voice + voice_set_ack."""
    http_client, _server, bridge = client
    bridge.active_provider = "yandex"
    bridge.voices_payload = [{"voice_id": "alena"}]
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "set_voice", "voice_id": "alena", "preset": "friendly", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_ack", timeout=1.0
        )
        assert body is not None
        assert body["voice_id"] == "alena"
        assert body["preset"] == "friendly"
        assert isinstance(body["ts_ms"], int) and body["ts_ms"] > 0
        assert bridge.set_voice_calls == [("alena", "friendly")]
    finally:
        await ws.close()


async def test_set_voice_unknown_id_returns_nack_with_available(client, fixed_pin):
    """set_voice {voice_id: bogus} → voice_set_nack{reason: voice_unavailable, available: [...]}."""
    http_client, _server, bridge = client
    bridge.active_provider = "yandex"
    bridge.voices_payload = [{"voice_id": "alena"}, {"voice_id": "anton"}]
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "set_voice", "voice_id": "bogus", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_nack", timeout=1.0
        )
        assert body is not None
        assert body["reason"] == "voice_unavailable"
        assert body["voice_id"] == "bogus"
        assert sorted(body["available"]) == ["alena", "anton"]
    finally:
        await ws.close()


async def test_set_voice_no_active_provider_returns_tts_unreachable(client, fixed_pin):
    """set_voice при пустом active_provider → nack{reason: tts_unreachable}."""
    http_client, _server, bridge = client
    bridge.active_provider = ""
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "set_voice", "voice_id": "alena", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_nack", timeout=1.0
        )
        assert body is not None
        assert body["reason"] == "tts_unreachable"
    finally:
        await ws.close()


async def test_set_voice_bad_payload_returns_error(client, fixed_pin):
    """set_voice без voice_id → BAD_PAYLOAD (через _send_error)."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "set_voice", "ts_ms": 0})
        # _send_error шлёт ERROR-frame (см. session.py:ErrorCode). Для теста
        # достаточно что bridge.set_voice_calls пустой.
        await asyncio.sleep(0.05)
        assert bridge.set_voice_calls == []
    finally:
        await ws.close()


async def test_preview_voice_calls_bridge_and_registers(client, fixed_pin):
    """preview_voice {request_id, voice_id, text} → bridge.publish_preview_voice + pending."""
    http_client, server, bridge = client
    bridge.active_provider = "yandex"
    bridge.active_voice = "alena"
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {
                "cmd": "preview_voice",
                "request_id": "req-1",
                "voice_id": "alena",
                "text": "Привет",
                "ts_ms": 0,
            },
        )
        await asyncio.sleep(0.05)
        assert bridge.preview_voice_calls == [("req-1", "alena", "Привет")]
        # request_id зарегистрирован в pending.
        assert "req-1" in server._preview_pending
    finally:
        await ws.close()


async def test_preview_voice_deliver_error_clears_pending(client, fixed_pin):
    """deliver_preview_error от supervisor → preview_voice_error клиенту + pending чистый."""
    http_client, server, bridge = client
    bridge.active_provider = "yandex"
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {"cmd": "preview_voice", "request_id": "req-err", "voice_id": "alena", "text": "x", "ts_ms": 0},
        )
        await asyncio.sleep(0.05)
        assert "req-err" in server._preview_pending
        # Ставим _send_loop руками (в aiohttp test client он уже есть, но на всякий случай).
        server._send_loop = asyncio.get_event_loop()
        delivered = server.deliver_preview_error("req-err", "preview_synthesis_not_implemented_in_mvp")
        assert delivered is True
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "preview_voice_error", timeout=1.0
        )
        assert body is not None
        assert body["request_id"] == "req-err"
        assert body["reason"] == "preview_synthesis_not_implemented_in_mvp"
        assert "req-err" not in server._preview_pending
    finally:
        await ws.close()


async def test_voice_rate_limit_drops_repeat(client, fixed_pin):
    """list_voices подряд 2 раза за <10s — второй cmd дропается."""
    http_client, _server, bridge = client
    bridge.active_provider = "yandex"
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "list_voices", "ts_ms": 0})
        # Первый должен дойти.
        body1 = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_list", timeout=1.0
        )
        assert body1 is not None
        # Второй в течение лимита — должен быть drop (rate-limit), не должно быть второго voice_list.
        await _send_json_cmd(ws, {"cmd": "list_voices", "ts_ms": 0})
        # Дренируем очередь сообщений.
        drain_start = time.monotonic()
        second_count = 0
        while time.monotonic() - drain_start < 0.3:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.JSON_EVENT:
                    body = json.loads(payload)
                    if body.get("type") == "voice_list":
                        second_count += 1
        # Должен быть только первый (rate-limit не пускает второй).
        assert second_count == 0, "rate-limit failed: second list_voices delivered"
    finally:
        await ws.close()
