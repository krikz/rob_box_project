"""E2E-сценарий «два голоса — отказ второму» для voice_floor.

Контекст (см. issue #1912, docs/plans/2026-08-27-quest-voice-passthrough-design.md §5,
meta-quest-api.md §4 + §6):

  Когда AV-23 (Telegram-рация) приземлится, два WS-клиента
  (operator-quest в Meta Quest + telegram-bridge) смогут одновременно
  слать голос. VoiceFloor — серверный mutex, ровно один держатель
  на /avatar/voice_in. Acceptance из родительской карточки t_53a576a4:

    1. operator держит PTT → voice_state{listening, holder_id=operator} виден ему;
    2. telegram пытается → voice_state{denied, holder_id=operator, detail=...}
       виден telegram и (через snapshot подписки) operator;
    3. operator отпускает PTT → voice_state{idle};
    4. telegram повторно шлёт start → voice_state{listening, holder_id=telegram}.

Этот файл — e2e-сценарий для PR #1933 (AV-25 voice_floor). Он собирает в
одном тесте весь flow, который родительский test_ws_server_voice.py
покрывает разрозненно, чтобы любой регресс floor-а был виден сразу
как «сценарий G8/G19 сломан», а не как зелёные отдельные юниты.

Без rclpy/ROS/Zenoh — подменяем Bridge на NoOpBridge-наследника (как
в test_ws_server_voice.py). Чистый aiohttp test-utils, без реальных
сетевых портов.
"""

from __future__ import annotations

import asyncio
import json
import time
from typing import Optional

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.server.ws_server import NoOpBridge, WSSServer, build_app


pytestmark = pytest.mark.asyncio


# ---------------------------------------------------------------------------
# Фикстуры (mirror test_ws_server_voice.py, чтобы поведение было одинаковым).
# ---------------------------------------------------------------------------


class RecordingBridge(NoOpBridge):
    """Stub, записывающий bridge-вызовы и считающий voice_state-эмиссии."""

    def __init__(self) -> None:
        self.barge_in_calls = 0
        self.voice_stop_calls = 0
        self.voice_audio_payloads: list[bytes] = []
        self.robot_start_calls = 0
        self.robot_stop_calls = 0

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
    """Поднимаем watchdog до 30 с, чтобы e2e не зависел от RTT event-loop'а."""
    import rob_box_quest.server.session as _session_mod

    monkeypatch.setattr(_session_mod, "WATCHDOG_TIMEOUT_S", 30.0)


@pytest.fixture
async def client(fixed_pin, long_watchdog, bridge):
    server = WSSServer(bridge=bridge, pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        yield client, server, bridge


# ---------------------------------------------------------------------------
# Хелперы.
# ---------------------------------------------------------------------------


async def _open_and_hello(client, pin: str):
    """Открыть WS, выполнить HELLO, дождаться WELCOME. Возвращает ws."""
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


async def _send_cmd(ws, cmd_obj: dict) -> None:
    payload = json.dumps(cmd_obj).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))


async def _subscribe(ws, topic: str) -> None:
    payload = json.dumps({"topic": topic, "quality": "med"}).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.SUBSCRIBE, 0, payload))


async def _next_voice_state(ws, timeout_s: float = 1.0) -> Optional[dict]:
    """Дождаться JSON_EVENT{type:'voice_state',...}, фильтруя heartbeat/ack.

    Возвращает payload dict или None при таймауте.
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


# ---------------------------------------------------------------------------
# E2E-сценарий из issue #1912 / t_53a576a4 (acceptance criterion).
# ---------------------------------------------------------------------------


async def test_two_clients_floor_denied_then_handoff(client, fixed_pin):
    """Полный сценарий «два голоса — отказ второму» в одном тесте.

    Шаги (один тест = один аудит-сценарий G8/G19):

      1. operator + telegram открывают WS, оба SUBSCRIBE voice_state
         (для snapshot-индикатора WebXR).
      2. operator шлёт voice_ptt_start → получает voice_state{listening,
         holder_id=operator:...}.
      3. telegram шлёт voice_ptt_start → получает voice_state{denied,
         holder_id=operator:..., detail='busy: operator:...'}.
         bridge.publish_voice_barge_in НЕ вызван для telegram.
      4. operator SUBSCRIBE уже сделал; его snapshot-индикатор показывает
         listening (через SUBSCRIBE snapshot при занятом floor — это
         покрыто отдельным тестом test_voice_floor_subscribe_to_voice_state_*
         в test_ws_server_voice.py; здесь мы подтверждаем, что telegram
         через SUBSCRIBE тоже видит busy).
      5. operator шлёт voice_ptt_stop → получает voice_state{idle}.
      6. telegram повторно шлёт voice_ptt_start → получает
         voice_state{listening, holder_id=telegram:...}.
         bridge.publish_voice_barge_in теперь вызван 2 раза (operator + telegram).

    Acceptance:
      - оба клиента «видят» floor-состояние через voice_state канал;
      - denied приходит именно denied-получателю, не holder-у;
      - handoff (operator stop → telegram acquire) работает;
      - bridge вызывается только для фактического держателя floor.
    """
    http_client, _server, bridge = client

    # 1) Открываем двух клиентов и оба подписываются на voice_state.
    ws_op = await _open_and_hello(http_client, fixed_pin)
    ws_tg = await _open_and_hello(http_client, fixed_pin)
    try:
        # subscribe_ack + heartbeat идут параллельно, фильтруем по типу события.
        await _subscribe(ws_op, "voice_state")
        await _subscribe(ws_tg, "voice_state")
        # Считываем snapshot'ы — должны быть voice_state{idle} (ещё никто не говорит).
        snap_op = await _next_voice_state(ws_op, timeout_s=1.5)
        snap_tg = await _next_voice_state(ws_tg, timeout_s=1.5)
        assert snap_op is not None and snap_op["state"] == "idle"
        assert snap_tg is not None and snap_tg["state"] == "idle"

        # 2) operator захватывает floor.
        await _send_cmd(
            ws_op,
            {"cmd": "voice_ptt_start", "client_id": "operator", "ts_ms": 0},
        )
        ev_listening = await _next_voice_state(ws_op, timeout_s=1.5)
        assert ev_listening is not None
        assert ev_listening["state"] == "listening"
        assert ev_listening["holder_id"].startswith("operator:")
        # bridge получил ровно один barge_in (от operator).
        assert bridge.barge_in_calls == 1

        # 3) telegram пытается — должен получить denied.
        await _send_cmd(
            ws_tg,
            {"cmd": "voice_ptt_start", "client_id": "telegram-bridge", "ts_ms": 0},
        )
        ev_denied = await _next_voice_state(ws_tg, timeout_s=1.5)
        assert ev_denied is not None, (
            "telegram должен получить voice_state{denied}, иначе floor не работает"
        )
        assert ev_denied["type"] == "voice_state"
        assert ev_denied["state"] == "denied"
        assert ev_denied["holder_id"].startswith("operator:")
        assert ev_denied["detail"].startswith("busy: operator:")
        # bridge не получал второй barge_in.
        assert bridge.barge_in_calls == 1, (
            "denied-клиент НЕ должен публиковать в /avatar/voice_in"
        )

        # 4) operator отпускает PTT.
        await _send_cmd(ws_op, {"cmd": "voice_ptt_stop", "ts_ms": 0})
        ev_idle = await _next_voice_state(ws_op, timeout_s=1.5)
        assert ev_idle is not None
        assert ev_idle["state"] == "idle"
        assert bridge.voice_stop_calls == 1

        # 5) telegram повторно — теперь должен захватить floor.
        await _send_cmd(
            ws_tg,
            {"cmd": "voice_ptt_start", "client_id": "telegram-bridge", "ts_ms": 0},
        )
        ev_listening_tg = await _next_voice_state(ws_tg, timeout_s=1.5)
        assert ev_listening_tg is not None, (
            "после operator stop telegram должен иметь возможность захватить floor"
        )
        assert ev_listening_tg["state"] == "listening"
        assert ev_listening_tg["holder_id"].startswith("telegram-bridge:")
        # bridge получил второй barge_in (от telegram).
        assert bridge.barge_in_calls == 2

        # 6) Финальная проверка: floor действительно занят telegram, а
        # operator (если попробует снова) тоже получит denied.
        await _send_cmd(
            ws_op,
            {"cmd": "voice_ptt_start", "client_id": "operator", "ts_ms": 0},
        )
        ev_denied_op = await _next_voice_state(ws_op, timeout_s=1.5)
        assert ev_denied_op is not None
        assert ev_denied_op["state"] == "denied"
        assert ev_denied_op["holder_id"].startswith("telegram-bridge:")
        # bridge всё ещё 2 (не 3) — denied не публикуется.
        assert bridge.barge_in_calls == 2

    finally:
        await ws_op.close()
        await ws_tg.close()


async def test_two_clients_floor_with_subscribed_indicator_snapshots(
    client, fixed_pin
):
    """Гарантия для UI: подписанный на voice_state клиент видит **актуальный**
    busy-снимок, даже если он сам start не отправлял.

    Это нужно WebXR-индикатору: после reconnect (или просто при
    повторном SUBSCRIBE) он должен увидеть «у робота говорит <X>»,
    а не «idle», пока floor занят.

    Шаги:
      1. operator захватывает floor.
      2. telegram подписывается на voice_state → получает snapshot
         {listening, holder_id=operator:...} (а не idle).
      3. operator stop → telegram подписка не обновляется автоматически
         (voice_state event-driven, не broadcast); для нового snapshot
         telegram снова шлёт SUBSCRIBE.
      4. telegram SUBSCRIBE снова → получает {idle} (floor свободен).
    """
    http_client, _server, bridge = client

    ws_op = await _open_and_hello(http_client, fixed_pin)
    ws_tg = await _open_and_hello(http_client, fixed_pin)
    try:
        # 1) operator держит floor.
        await _send_cmd(
            ws_op,
            {"cmd": "voice_ptt_start", "client_id": "operator", "ts_ms": 0},
        )
        ev = await _next_voice_state(ws_op, timeout_s=1.5)
        assert ev is not None and ev["state"] == "listening"

        # 2) telegram подписывается — должен увидеть busy snapshot.
        await _subscribe(ws_tg, "voice_state")
        snap_busy = await _next_voice_state(ws_tg, timeout_s=1.5)
        assert snap_busy is not None
        assert snap_busy["state"] == "listening"
        assert snap_busy["holder_id"].startswith("operator:")
        # UI сразу может нарисовать «у робота говорит operator».

        # 3) operator отпускает.
        await _send_cmd(ws_op, {"cmd": "voice_ptt_stop", "ts_ms": 0})
        ev_idle = await _next_voice_state(ws_op, timeout_s=1.5)
        assert ev_idle is not None and ev_idle["state"] == "idle"

        # 4) telegram re-subscribes — должен увидеть idle.
        # NB: ws уже подписан — повторный SUBSCRIBE в текущей реализации
        # эмитит subscribe_ack + snapshot заново (см. ws_server.py §_on_subscribe).
        await _subscribe(ws_tg, "voice_state")
        snap_idle = await _next_voice_state(ws_tg, timeout_s=1.5)
        assert snap_idle is not None
        assert snap_idle["state"] == "idle"
    finally:
        await ws_op.close()
        await ws_tg.close()
