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
        self.robot_start_calls = 0
        self.robot_stop_calls = 0
        self.voice_modes: list[str] = []

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


@pytest.fixture
def bridge() -> RecordingBridge:
    return RecordingBridge()


@pytest.fixture
def fixed_pin(monkeypatch) -> str:
    pin = "123456"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


@pytest.fixture
def long_watchdog(monkeypatch) -> None:
    """Отключить watchdog для floor-тестов (они ждут ответ дольше 0.6 с).

    В проде WATCHDOG_TIMEOUT_S = 0.6 (meta-quest-api.md §7 + ADR-0027 §3.3).
    Здесь поднимаем до 30 с, чтобы тесты не зависели от реального RTT
    event-loop'а. Поведение server-логики voice-floor не затронуто.
    """
    import rob_box_quest.server.session as _session_mod

    monkeypatch.setattr(_session_mod, "WATCHDOG_TIMEOUT_S", 30.0)


@pytest.fixture
async def client(fixed_pin, long_watchdog, bridge):
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


# --------------------------------------------------------------------------
# Voice-floor: серверный mutex (двух квестов быть не должно).
# Acceptance (t_3c27c1da):
#  - при занятом floor второй voice_ptt_start → отказ + voice_state{denied};
#  - при единственном клиенте поведение совпадает с до-изменения;
#  - отвал клиента освобождает floor (force_release_for).
# --------------------------------------------------------------------------


async def _next_voice_state_event(ws, timeout_s: float = 1.0):
    """Дождаться JSON_EVENT{type:'voice_state',...}, пропуская прочие события
    (subscribe_ack, heartbeat, pong, ...). Возвращает decoded payload dict или
    None при таймауте.

    NB: heartbeat идёт каждые 200 мс (meta-quest-api.md §7) — он придёт
    раньше voice_state если тест не успеет. Поэтому фильтруем по type.

    NB2: WATCHDOG_TIMEOUT_S = 0.6 с (session.py). Тест должен послать ping
    непосредственно перед этой функцией (см. ``_send_ping``), иначе
    watchdog отключит клиента до того, как мы успеем прочитать ответ.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            msg = await asyncio.wait_for(ws.receive(), timeout=min(remaining, 0.15))
        except asyncio.TimeoutError:
            continue
        if msg.type == WSMsgType.CLOSE:
            return None
        if msg.type != WSMsgType.BINARY:
            continue
        ftype, _sid, payload = decode_frame(msg.data)
        if ftype != FrameType.JSON_EVENT:
            continue
        ev = json.loads(payload.decode("utf-8"))
        if ev.get("type") == "voice_state":
            return ev
    return None


async def _try_receive(ws):
    """Луч-эффект receive с коротким таймаутом; None если ничего нет."""
    fut = asyncio.ensure_future(ws.receive())
    try:
        return await asyncio.wait_for(fut, timeout=0.05)
    except asyncio.TimeoutError:
        fut.cancel()
        try:
            await fut
        except (asyncio.CancelledError, Exception):
            pass
        return None


async def test_voice_floor_acquire_first_client(client, fixed_pin):
    """Один клиент: voice_ptt_start → barge_in + voice_state{listening} + holder_id."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {"cmd": "voice_ptt_start", "client_id": "operator-quest", "ts_ms": 0},
        )
        ev = await _next_voice_state_event(ws)
        assert ev is not None
        assert ev["type"] == "voice_state"
        assert ev["state"] == "listening"
        assert "holder_id" in ev and ev["holder_id"].startswith("operator-quest:")
        assert bridge.barge_in_calls == 1
    finally:
        await ws.close()


async def test_voice_floor_second_client_gets_denied(client, fixed_pin):
    """Два WS-клиента, второй start при занятом floor → denied + bridge НЕ вызван."""
    http_client, _server, bridge = client
    ws1 = await _open_and_hello(http_client, fixed_pin)
    ws2 = await _open_and_hello(http_client, fixed_pin)
    try:
        # 1) operator захватывает floor.
        await _send_json_cmd(
            ws1,
            {"cmd": "voice_ptt_start", "client_id": "operator-quest", "ts_ms": 0},
        )
        ev1 = await _next_voice_state_event(ws1)
        assert ev1 is not None
        assert ev1["state"] == "listening"
        # ws1 уже прочитал listening; голосовые события на ws2 ещё не пришли.

        # 2) telegram-bridge пытается — должен получить denied + busy holder.
        await _send_json_cmd(
            ws2,
            {"cmd": "voice_ptt_start", "client_id": "telegram-bridge", "ts_ms": 0},
        )
        ev2 = await _next_voice_state_event(ws2)
        assert ev2 is not None, "ws2 должен получить voice_state{denied}"
        assert ev2["type"] == "voice_state"
        assert ev2["state"] == "denied"
        assert ev2["holder_id"].startswith("operator-quest:")
        assert ev2["detail"].startswith("busy: operator-quest:")
        # bridge не получал второй publish_voice_barge_in (только первый).
        assert bridge.barge_in_calls == 1
    finally:
        await ws1.close()
        await ws2.close()


async def test_voice_floor_stop_releases_and_emits_idle(client, fixed_pin):
    """voice_ptt_stop от держателя → voice_state{idle}."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws, {"cmd": "voice_ptt_start", "client_id": "op", "ts_ms": 0}
        )
        ev = await _next_voice_state_event(ws)
        assert ev is not None and ev["state"] == "listening"

        await _send_json_cmd(ws, {"cmd": "voice_ptt_stop", "ts_ms": 0})
        ev_idle = await _next_voice_state_event(ws)
        assert ev_idle is not None
        assert ev_idle["type"] == "voice_state"
        assert ev_idle["state"] == "idle"
        assert "holder_id" not in ev_idle or ev_idle.get("holder_id") is None
        assert bridge.voice_stop_calls == 1
    finally:
        await ws.close()


async def test_voice_floor_stop_by_non_holder_does_not_emit_idle(client, fixed_pin):
    """voice_ptt_stop от НЕ-держателя → bridge вызван, но voice_state{idle} НЕ шлём."""
    http_client, _server, bridge = client
    ws1 = await _open_and_hello(http_client, fixed_pin)
    ws2 = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws1, {"cmd": "voice_ptt_start", "client_id": "op", "ts_ms": 0}
        )
        await _next_voice_state_event(ws1)

        # ws2 шлёт stop без удержания floor — никакого idle event не должно быть.
        await _send_json_cmd(ws2, {"cmd": "voice_ptt_stop", "ts_ms": 0})
        leaked = await _try_receive(ws2)
        if leaked is not None:
            # Если событие всё-таки пришло — это не должен быть voice_state{idle}
            if leaked.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(leaked.data)
                if ftype == FrameType.JSON_EVENT:
                    ev = json.loads(payload.decode("utf-8"))
                    assert not (
                        ev.get("type") == "voice_state" and ev.get("state") == "idle"
                    ), "non-holder не должен слать voice_state{idle}"
        # bridge всё равно вызвал stop (идемпотентная команда).
        assert bridge.voice_stop_calls == 1
    finally:
        await ws1.close()
        await ws2.close()


async def test_voice_floor_disconnect_releases_floor(client, fixed_pin):
    """Отвал держателя (close WS) → другой клиент может захватить floor."""
    http_client, _server, bridge = client
    ws1 = await _open_and_hello(http_client, fixed_pin)
    ws2 = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws1, {"cmd": "voice_ptt_start", "client_id": "op", "ts_ms": 0}
        )
        await _next_voice_state_event(ws1)
        # ws2 пытается — denied.
        await _send_json_cmd(
            ws2, {"cmd": "voice_ptt_start", "client_id": "tg", "ts_ms": 0}
        )
        ev_denied = await _next_voice_state_event(ws2)
        assert ev_denied is not None and ev_denied["state"] == "denied"

        # ws1 отваливается (имитация watchdog/disconnect).
        await ws1.close()
        # ws2 теперь должен иметь возможность захватить floor.
        await _send_json_cmd(
            ws2, {"cmd": "voice_ptt_start", "client_id": "tg", "ts_ms": 0}
        )
        ev_ok = await _next_voice_state_event(ws2)
        assert ev_ok is not None, (
            "ws2 должен получить voice_state после force_release_for(ws1)"
        )
        assert ev_ok["type"] == "voice_state"
        assert ev_ok["state"] == "listening"
        assert ev_ok["holder_id"].startswith("tg:")
    finally:
        await ws1.close()
        await ws2.close()


async def test_voice_floor_subscribe_to_voice_state_emits_idle_snapshot(client, fixed_pin):
    """SUBSCRIBE voice_state без держателя → voice_state{idle} сразу."""
    http_client, _server, _bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        payload = json.dumps({"topic": "voice_state", "quality": "med"}).encode("utf-8")
        await ws.send_bytes(encode_frame(FrameType.SUBSCRIBE, 0, payload))

        ev = await _next_voice_state_event(ws, timeout_s=1.5)
        assert ev is not None
        assert ev["type"] == "voice_state"
        assert ev["state"] == "idle"
        assert ev.get("holder_id") is None
    finally:
        await ws.close()


async def test_voice_floor_subscribe_to_voice_state_emits_listening_snapshot(client, fixed_pin):
    """SUBSCRIBE voice_state при занятом floor → voice_state{listening} с holder_id."""
    http_client, _server, bridge = client
    ws1 = await _open_and_hello(http_client, fixed_pin)
    ws2 = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws1, {"cmd": "voice_ptt_start", "client_id": "op", "ts_ms": 0}
        )
        await _next_voice_state_event(ws1)

        # ws2 подписывается на voice_state — должен сразу увидеть busy.
        payload = json.dumps({"topic": "voice_state", "quality": "med"}).encode("utf-8")
        await ws2.send_bytes(encode_frame(FrameType.SUBSCRIBE, 0, payload))

        ev = await _next_voice_state_event(ws2, timeout_s=1.5)
        assert ev is not None
        assert ev["type"] == "voice_state"
        assert ev["state"] == "listening"
        assert ev["holder_id"] is not None
        assert ev["holder_id"].startswith("op:")
        # bridge не трогали — это просто snapshot.
        assert bridge.barge_in_calls == 1
    finally:
        await ws1.close()
        await ws2.close()


async def test_voice_floor_baseline_single_client_unchanged(client, fixed_pin):
    """Штатный сценарий (1 клиент): поведение совпадает с до-изменения
    — bridge.barge_in/voice_stop вызываются как раньше, плюс voice_state
    events идут как новый side-effect."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_ptt_start", "ts_ms": 0})
        ev_start = await _next_voice_state_event(ws)
        assert ev_start is not None and ev_start["state"] == "listening"
        assert bridge.barge_in_calls == 1

        # VOICE_AUDIO всё ещё работает.
        pcm = b"\x00\x00\xff\x7f"
        await ws.send_bytes(encode_frame(FrameType.VOICE_AUDIO, 0, pcm))
        await asyncio.sleep(0.05)
        assert bridge.voice_audio_payloads == [pcm]

        await _send_json_cmd(ws, {"cmd": "voice_ptt_stop", "ts_ms": 0})
        ev_stop = await _next_voice_state_event(ws)
        assert ev_stop is not None and ev_stop["state"] == "idle"
        assert bridge.voice_stop_calls == 1
    finally:
        await ws.close()
