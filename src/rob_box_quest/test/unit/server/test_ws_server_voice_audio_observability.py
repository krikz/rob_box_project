"""issue #1992 observability: приём VOICE_AUDIO в ws_server раньше не
оставлял никакого следа в логах. Симптом на роботе (владелец надел шлем,
сказал «ТАРС ты здесь» несколько раз): в ``docker logs rob-box-quest`` — ни
одной строки по wake/voice_listen/VOICE_AUDIO. Причина оказалась на
клиенте (main.ts не запускал захват микрофона вне грипа, см.
webxr_client/tests/voice_capture_wiring.test.ts), но чтобы СЛЕДУЮЩИЙ раз
отладка не была слепой — сервер обязан логировать сам факт приёма кадра,
независимо от того, публикует ли мост его дальше в ROS.

Эти тесты проверяют ``WSSServer._note_voice_audio_rx`` (см. ws_server.py):
- первый принятый пакет per stream_id логируется сразу;
- дальше — не чаще раза в ``VOICE_AUDIO_LOG_INTERVAL_S`` секунд (иначе на
  потоке ~16 кГц / 20мс-чанках лог захлебнётся, до 50 пакетов/сек);
- реальный WS-фрейм VOICE_AUDIO(stream_id=2) действительно триггерит лог
  через полный путь ``_ws_handler`` (не только прямой вызов private-метода).
"""

import asyncio
import json
import logging
import time

import pytest
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.server.ws_server import (
    VOICE_AUDIO_LOG_INTERVAL_S,
    NoOpBridge,
    WSSServer,
    build_app,
)

# Только один тест здесь реально async (полный WS round-trip); остальные —
# синхронные unit-тесты приватного метода. Маркируем точечно, а не через
# module-level pytestmark, иначе pytest-asyncio ругается на sync-тесты.


@pytest.fixture
def fixed_pin(monkeypatch) -> str:
    pin = "191992"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


async def _open_and_hello(client, pin):
    from aiohttp import WSMsgType

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


# ── Unit-level: _note_voice_audio_rx напрямую (детерминированный time) ──


def test_first_packet_logs_immediately(caplog):
    server = WSSServer(bridge=NoOpBridge(), pin="000000")
    with caplog.at_level(logging.INFO, logger="rob_box_quest.server.ws_server"):
        server._note_voice_audio_rx(2, 640, "sess-1")
    assert any(
        "first packet" in r.message and "stream_id=2" in r.message
        for r in caplog.records
    )


def test_second_packet_within_window_does_not_log_again(caplog, monkeypatch):
    server = WSSServer(bridge=NoOpBridge(), pin="000000")
    now = [1000.0]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    with caplog.at_level(logging.INFO, logger="rob_box_quest.server.ws_server"):
        server._note_voice_audio_rx(2, 640, "sess-1")
        caplog.clear()
        now[0] += 0.02  # один чанк (20 мс) позже — далеко до интервала сводки
        server._note_voice_audio_rx(2, 640, "sess-1")
    assert caplog.records == []


def test_summary_logged_after_interval_elapses(caplog, monkeypatch):
    server = WSSServer(bridge=NoOpBridge(), pin="000000")
    now = [1000.0]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    with caplog.at_level(logging.INFO, logger="rob_box_quest.server.ws_server"):
        server._note_voice_audio_rx(2, 640, "sess-1")  # first packet, count=1
        caplog.clear()
        for _ in range(9):
            now[0] += 0.02
            server._note_voice_audio_rx(2, 640, "sess-1")  # count=2..10, no log yet
        assert caplog.records == []
        now[0] += VOICE_AUDIO_LOG_INTERVAL_S  # окно истекло
        server._note_voice_audio_rx(2, 640, "sess-1")  # count=11 -> summary
    summaries = [r.message for r in caplog.records if "rx summary" in r.message]
    assert len(summaries) == 1
    msg = summaries[0]
    assert "stream_id=2" in msg
    # 10 пакетов накопилось в окне (9 молчаливых + этот) до сводки.
    assert "10 packets" in msg
    assert "6400 bytes" in msg
    assert "total 11 packets" in msg


def test_stream_ids_are_counted_independently(caplog, monkeypatch):
    """ptt (sid=1) и wake (sid=2) не должны делить один счётчик/окно —
    иначе живой ptt-трафик маскирует полное отсутствие wake."""
    server = WSSServer(bridge=NoOpBridge(), pin="000000")
    now = [2000.0]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    with caplog.at_level(logging.INFO, logger="rob_box_quest.server.ws_server"):
        for _ in range(50):
            now[0] += 0.02
            server._note_voice_audio_rx(1, 320, "sess-1")  # ptt: живой поток
        # wake ни разу не пришёл — ровно симптом issue #1992.
    ptt_first = [
        r.message
        for r in caplog.records
        if "first packet" in r.message and "stream_id=1" in r.message
    ]
    wake_any = [r.message for r in caplog.records if "stream_id=2" in r.message]
    assert len(ptt_first) == 1
    assert wake_any == []  # честная тишина по wake — не замаскирована ptt


# ── End-to-end: реальный WS-фрейм действительно доходит до логгера ──────


@pytest.mark.asyncio
async def test_real_voice_audio_frame_triggers_rx_log(fixed_pin, caplog):
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as http_client:
        ws = await _open_and_hello(http_client, fixed_pin)
        try:
            with caplog.at_level(
                logging.INFO, logger="rob_box_quest.server.ws_server"
            ):
                pcm = b"\x00\x00\xff\x7f\x00\x80\x01\x00"
                await ws.send_bytes(encode_frame(FrameType.VOICE_AUDIO, 2, pcm))
                await asyncio.sleep(0.05)
            assert any(
                "first packet" in r.message and "stream_id=2" in r.message
                for r in caplog.records
            )
        finally:
            await ws.close()
