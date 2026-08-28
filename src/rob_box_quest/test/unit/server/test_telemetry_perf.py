"""Phase 2.2 telemetry tests (ADR-0032 §3.5).

Покрывает:
- WSS 0x40 TELEMETRY_PERF frame type добавлен в protocol/frame.py;
- _on_telemetry_perf вызывает Bridge.publish_quest_perf с raw payload;
- Rate-limit: > 5 Hz → дроп после 10 быстрых;
- Bridge contract: NoOpBridge + mock-Bridge оба реализуют publish_quest_perf.
- Session fields last_telemetry_ms / telemetry_fast_count инициализируются.

Без rclpy / aiohttp — чистая логика через прямой вызов _on_telemetry_perf.
"""

from __future__ import annotations

import asyncio
import json
import time

import pytest

from rob_box_quest.protocol.frame import FrameType, encode_frame
from rob_box_quest.server.session import ClientSession, SessionState
from rob_box_quest.server.ws_server import NoOpBridge, WSSServer


class FakeBridge:
    """Тестовый Bridge — записывает все вызовы publish_quest_perf."""

    def __init__(self) -> None:
        self.calls: list[bytes] = []
        self.fail_next = False

    def publish_quest_perf(self, payload: bytes) -> None:
        if self.fail_next:
            self.fail_next = False
            raise RuntimeError("simulated failure")
        self.calls.append(bytes(payload))

    # NoOp stubs — нужны только для контракта.
    def publish_quest(self, linear: float, angular: float) -> None: ...
    def publish_emergency(self) -> None: ...
    def feed_client_alive(self) -> None: ...
    def reset(self) -> None: ...
    def emergency_stop(self) -> None: ...
    def publish_frame(self, ui_name: str, payload: bytes) -> None: ...
    def available_streams(self) -> list: return []
    def publish_voice_barge_in(self) -> None: ...
    def publish_voice_audio(self, payload: bytes) -> None: ...
    def publish_voice_stop(self) -> None: ...
    def publish_voice_robot_start(self) -> None: ...
    def publish_voice_robot_stop(self) -> None: ...
    def set_voice_mode(self, mode: str) -> None: ...


def _authed_session() -> ClientSession:
    s = ClientSession()
    s.mark_authenticated("test", ["webxr"])
    return s


def test_frame_type_telemetry_perf_exists():
    """FrameType.TELEMETRY_PERF = 0x40 (ADR-0032 §3.5)."""
    assert int(FrameType.TELEMETRY_PERF) == 0x40


def test_bridge_publish_quest_perf_in_noop():
    """NoOpBridge.publish_quest_perf есть и no-op."""
    b = NoOpBridge()
    b.publish_quest_perf(b"\x00\x01\x02")  # не должен бросить


@pytest.mark.asyncio
async def test_on_telemetry_perf_calls_bridge():
    """_on_telemetry_perf пересылает CBOR payload в Bridge без декодирования."""
    bridge = FakeBridge()
    server = WSSServer(bridge=bridge, pin="000000")
    session = _authed_session()
    # Произвольные CBOR bytes (не декодируем на server).
    payload = b"\xa2\x63\x66\x70\x73\x18\x58\x67\x70\x61\x79\x6c\x6f\x61\x64\x00"
    await server._on_telemetry_perf(ws=None, session=session, payload=payload)
    assert bridge.calls == [payload]
    assert session.last_telemetry_ms is not None
    assert session.telemetry_fast_count == 0


@pytest.mark.asyncio
async def test_on_telemetry_perf_handles_bridge_failure():
    """Если Bridge падает, сокет НЕ рвётся (warning в логе, продолжаем)."""
    bridge = FakeBridge()
    bridge.fail_next = True
    server = WSSServer(bridge=bridge, pin="000000")
    session = _authed_session()
    # Должно проглотить исключение и не выкинуть.
    await server._on_telemetry_perf(ws=None, session=session, payload=b"\x00")
    assert bridge.calls == []  # bridge выкинул до добавления в calls


@pytest.mark.asyncio
async def test_on_telemetry_perf_rate_limit_drops_after_10():
    """> 5 Hz (delta < 200 мс) → после 10 быстрых — дроп."""
    bridge = FakeBridge()
    server = WSSServer(bridge=bridge, pin="000000")
    session = _authed_session()
    # Симулируем 15 быстрых подряд пакетов (delta ~0).
    session.last_telemetry_ms = int(time.time() * 1000)
    for _ in range(15):
        await server._on_telemetry_perf(ws=None, session=session, payload=b"\x00")
    # Первые 10 должны пройти, после 11-го — дроп.
    # Точнее: telemetry_fast_count инкрементируется на каждом быстром, и
    # дроп начинается когда count > 10. После 15 вызовов: 10 в calls + 5 дропов.
    assert len(bridge.calls) <= 11  # ~10 до rate-limit триггера
    assert len(bridge.calls) >= 9  # с допуском на тайминг


@pytest.mark.asyncio
async def test_on_telemetry_perf_no_rate_limit_at_1hz():
    """1 Hz (delta >= 200 мс) — все пакеты проходят."""
    bridge = FakeBridge()
    server = WSSServer(bridge=bridge, pin="000000")
    session = _authed_session()
    # 3 пакета с интервалом 250 мс (реальный wall-clock).
    for _ in range(3):
        await server._on_telemetry_perf(ws=None, session=session, payload=b"\x00")
        await asyncio.sleep(0.25)
    assert len(bridge.calls) == 3


def test_session_has_telemetry_fields():
    """ClientSession инициализирует Phase 2.2 telemetry-поля."""
    s = ClientSession()
    assert s.last_telemetry_ms is None
    assert s.telemetry_fast_count == 0


def test_telemetry_perf_frame_format():
    """encodeFrame(TELEMETRY_PERF, 0, payload) → валидный header."""
    payload = b"\xa2\x63\x66\x70\x73\x18\x58"  # пример CBOR {fps: 88}, 7 байт
    frame = encode_frame(FrameType.TELEMETRY_PERF, 0, payload)
    assert frame[0] == 0x40
    assert frame[1:5] == b"\x00\x00\x00\x00"  # stream_id = 0
    # LEB128(7) = 0x07, потом 7 байт payload.
    assert frame[5] == 7
    assert frame[6:13] == payload
    assert len(frame) == 13
