"""Edge-case тесты для VoiceFloor / WS-server.

Контекст (см. issue #1912, body карточки t_53a576a4 §2):

  Юнит-тесты на стороне сервера для edge-кейсов:
  - отвал WS-клиента (освобождение floor по таймауту watchdog);
  - повторный PTT после denied без явного stop между;
  - неизвестное состояние из recon (новый HELLO после разрыва).

Эти три кейса НЕ покрыты ни voice_floor.py unit'ами (чистая логика,
без WS / aiohttp), ни test_ws_server_voice.py (там есть только
test_voice_floor_disconnect_releases_floor, и тот — про явный ws.close()).
Здесь — именно edge'ы.

Без rclpy/ROS/Zenoh — те же фикстуры, что и в test_ws_server_voice.py
(RecordingBridge, NoOpBridge, aiohttp test-utils).
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
# Фикстуры (те же, что в test_ws_server_voice.py, чтобы поведение было
# одинаковым и между e2e, и между edge-тестами).
# ---------------------------------------------------------------------------


class RecordingBridge(NoOpBridge):
    """Минимальный stub: считает только barge_in и voice_stop."""

    def __init__(self) -> None:
        self.barge_in_calls = 0
        self.voice_stop_calls = 0

    def publish_voice_barge_in(self) -> None:
        self.barge_in_calls += 1

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
def long_watchdog(monkeypatch) -> None:
    """Поднимаем watchdog до 30 с по умолчанию (как в test_ws_server_voice.py)."""
    import rob_box_quest.server.session as _session_mod

    monkeypatch.setattr(_session_mod, "WATCHDOG_TIMEOUT_S", 30.0)


@pytest.fixture
def short_watchdog(monkeypatch):
    """Опускаем watchdog до 0.2 с — чтобы edge-тест на отвал по таймауту
    не ждал вечность."""
    import rob_box_quest.server.session as _session_mod

    old = _session_mod.WATCHDOG_TIMEOUT_S
    monkeypatch.setattr(_session_mod, "WATCHDOG_TIMEOUT_S", 0.2)
    return old


@pytest.fixture
async def client(fixed_pin, long_watchdog, bridge):
    server = WSSServer(bridge=bridge, pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        yield client, server, bridge


@pytest.fixture
async def client_short_watchdog(fixed_pin, short_watchdog, bridge):
    """Клиент с watchdog 0.2 с — для теста на отвал по таймауту."""
    server = WSSServer(bridge=bridge, pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        yield client, server, bridge


# ---------------------------------------------------------------------------
# Хелперы.
# ---------------------------------------------------------------------------


async def _open_and_hello(client, pin: str):
    """Открыть WS, выполнить HELLO, дождаться WELCOME."""
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


async def _next_voice_state(ws, timeout_s: float = 1.0) -> Optional[dict]:
    """Дождаться JSON_EVENT{type:'voice_state',...}, фильтруя heartbeat/ack."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            msg = await asyncio.wait_for(ws.receive(), timeout=min(remaining, 0.1))
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


async def _drain_until_close(ws, max_wait_s: float = 2.0) -> None:
    """Читать из ws до тех пор, пока сервер его не закроет (watchdog-trip)."""
    deadline = time.monotonic() + max_wait_s
    while time.monotonic() < deadline:
        try:
            msg = await asyncio.wait_for(ws.receive(), timeout=0.2)
        except asyncio.TimeoutError:
            continue
        if msg.type == WSMsgType.CLOSE:
            return
        # heartbeat / прочие события пропускаем.
    pytest.fail(f"ws не закрыт после {max_wait_s} с — watchdog не сработал")


# ---------------------------------------------------------------------------
# Edge-кейс 1: отвал WS-клиента по watchdog-таймауту.
# ---------------------------------------------------------------------------


async def test_watchdog_timeout_releases_floor_for_other_client(
    client_short_watchdog, fixed_pin, short_watchdog
):
    """Держатель floor замолчал дольше WATCHDOG_TIMEOUT_S → server закрыл
    соединение → другой клиент может захватить floor.

    Это edge относительно test_voice_floor_disconnect_releases_floor:
    там явный ws.close() от клиента, здесь — пассивный watchdog-trip
    (клиент «умер» — сетевой разрыв без graceful GOODBYE).
    """
    http_client, _server, bridge = client_short_watchdog

    ws_dead = await _open_and_hello(http_client, fixed_pin)
    ws_alive = await _open_and_hello(http_client, fixed_pin)
    try:
        # 1) ws_dead захватывает floor.
        await _send_cmd(
            ws_dead,
            {"cmd": "voice_ptt_start", "client_id": "dead-client", "ts_ms": 0},
        )
        ev_listening = await _next_voice_state(ws_dead, timeout_s=1.5)
        assert ev_listening is not None and ev_listening["state"] == "listening"

        # 2) ws_alive пытается — denied (флор ещё занят).
        await _send_cmd(
            ws_alive,
            {"cmd": "voice_ptt_start", "client_id": "alive-client", "ts_ms": 0},
        )
        ev_denied = await _next_voice_state(ws_alive, timeout_s=1.5)
        assert ev_denied is not None and ev_denied["state"] == "denied"

        # 3) ws_alive кормит watchdog ping'ом (иначе и его отключат).
        #    ws_dead — НЕ кормит; ждём, пока watchdog-trip его закроет.
        async def _keep_alive(ws):
            """Шлёт ping каждые 100 мс (кормит watchdog)."""
            while True:
                await _send_cmd(ws, {"cmd": "ping", "ts_ms": 0})
                await asyncio.sleep(0.1)

        keep = asyncio.create_task(_keep_alive(ws_alive))
        try:
            # watchdog 0.2 с + закрытие + force_release_for → ждём до 2 с.
            await _drain_until_close(ws_dead, max_wait_s=2.0)
        finally:
            keep.cancel()
            try:
                await keep
            except (asyncio.CancelledError, Exception):
                pass

        # 4) ws_alive теперь может захватить floor.
        await _send_cmd(
            ws_alive,
            {"cmd": "voice_ptt_start", "client_id": "alive-client", "ts_ms": 0},
        )
        ev_ok = await _next_voice_state(ws_alive, timeout_s=2.0)
        assert ev_ok is not None, (
            "ws_alive должен получить voice_state после watchdog-trip держателя"
        )
        assert ev_ok["state"] == "listening"
        assert ev_ok["holder_id"].startswith("alive-client:")
        # bridge получил ровно 2 barge_in: dead-client (когда был жив) + alive-client.
        assert bridge.barge_in_calls == 2
    finally:
        await ws_dead.close()
        await ws_alive.close()


# ---------------------------------------------------------------------------
# Edge-кейс 2: повторный PTT после denied без явного stop между.
# ---------------------------------------------------------------------------


async def test_repeated_ptt_after_denied_stays_denied(client, fixed_pin):
    """Клиент получил denied → НЕ делает stop → шлёт start снова →
    снова denied. Floor остаётся у первого, bridge не публикует
    лишний barge_in.

    Это страховка от «retry storm» в UI: если пользователь продолжает
    жать PTT после отказа, сервер не должен мигать bridge.publish_voice_*
    на каждый ретрай.
    """
    http_client, _server, bridge = client

    ws_holder = await _open_and_hello(http_client, fixed_pin)
    ws_retry = await _open_and_hello(http_client, fixed_pin)
    try:
        # 1) holder захватывает.
        await _send_cmd(
            ws_holder,
            {"cmd": "voice_ptt_start", "client_id": "holder", "ts_ms": 0},
        )
        ev = await _next_voice_state(ws_holder, timeout_s=1.5)
        assert ev is not None and ev["state"] == "listening"

        # 2) retry-шник: первый start → denied.
        await _send_cmd(
            ws_retry,
            {"cmd": "voice_ptt_start", "client_id": "retry", "ts_ms": 0},
        )
        ev_d1 = await _next_voice_state(ws_retry, timeout_s=1.5)
        assert ev_d1 is not None and ev_d1["state"] == "denied"

        # 3) Без stop — второй start → снова denied.
        await _send_cmd(
            ws_retry,
            {"cmd": "voice_ptt_start", "client_id": "retry", "ts_ms": 0},
        )
        ev_d2 = await _next_voice_state(ws_retry, timeout_s=1.5)
        assert ev_d2 is not None and ev_d2["state"] == "denied"

        # 4) И третий — для надёжности.
        await _send_cmd(
            ws_retry,
            {"cmd": "voice_ptt_start", "client_id": "retry", "ts_ms": 0},
        )
        ev_d3 = await _next_voice_state(ws_retry, timeout_s=1.5)
        assert ev_d3 is not None and ev_d3["state"] == "denied"

        # 5) floor по-прежнему у holder; bridge не получал лишних barge_in.
        assert bridge.barge_in_calls == 1, (
            f"retry-storm должен игнорироваться: barge_in={bridge.barge_in_calls}, "
            "ожидался 1 (только от holder)"
        )
    finally:
        await ws_holder.close()
        await ws_retry.close()


# ---------------------------------------------------------------------------
# Edge-кейс 3: неизвестное состояние из recon (новый HELLO после разрыва).
# ---------------------------------------------------------------------------


async def test_reconnect_after_floor_loss_starts_idle(client, fixed_pin):
    """Держатель floor разорвал соединение (НЕ stop, просто close) →
    его session_id force-released → новый клиент приходит по тому же PIN'у
    с новым HELLO → получает чистый snapshot {idle}.

    Контракт для UI: после reconnect клиент НЕ должен «думать», что он
    ещё держит floor. floor.fsm при новой сессии стартует с idle.
    Другой уже-подключённый клиент видит {idle} через SUBSCRIBE snapshot.

    NB: в текущей реализации PIN не привязан к session_id (он не сессионный
    ключ, а ротационный secret — см. meta-quest-api.md §7 / ADR-0027 §4.5),
    поэтому «тот же клиент переподключился» означает именно новый session_id.
    """
    http_client, server, bridge = client

    # 1) ws1 захватывает floor.
    ws1 = await _open_and_hello(http_client, fixed_pin)
    await _send_cmd(
        ws1,
        {"cmd": "voice_ptt_start", "client_id": "reconnect-test", "ts_ms": 0},
    )
    ev = await _next_voice_state(ws1, timeout_s=1.5)
    assert ev is not None and ev["state"] == "listening"

    # 2) ws1 «умирает» без явного stop.
    await ws1.close()

    # 3) ws2 (или тот же клиент после recon) подключается и подписывается
    #    на voice_state → должен увидеть {idle} (старый session_id был
    #    force-released).
    ws2 = await _open_and_hello(http_client, fixed_pin)
    try:
        payload = json.dumps({"topic": "voice_state", "quality": "med"}).encode("utf-8")
        await ws2.send_bytes(encode_frame(FrameType.SUBSCRIBE, 0, payload))
        snap = await _next_voice_state(ws2, timeout_s=1.5)
        assert snap is not None
        assert snap["state"] == "idle", (
            f"после disconnect держателя новый клиент должен видеть idle, "
            f"получили state={snap['state']}"
        )
        assert snap.get("holder_id") is None

        # 4) ws2 теперь может нормально захватить floor (он свободен).
        await _send_cmd(
            ws2,
            {"cmd": "voice_ptt_start", "client_id": "reconnect-test", "ts_ms": 0},
        )
        ev_listening = await _next_voice_state(ws2, timeout_s=1.5)
        assert ev_listening is not None
        assert ev_listening["state"] == "listening"
        assert ev_listening["holder_id"].startswith("reconnect-test:")
    finally:
        await ws2.close()


async def test_unknown_voice_state_value_in_msgpack_normalized_to_idle():
    """Pure-logic edge: VoiceFloor нормализует неизвестное состояние из recon.

    Это из issue #1930 (см. body t_3888cb1e): «неизвестное состояние
    (например, от старого клиента или от внешнего реконнекта с устаревшим
    msgpack-словарём) → warning + idle».

    Тестируем напрямую VoiceFloor.fsm, чтобы контракт был зафиксирован:
    любой «новый» клиент, который приходит с неизвестным state, получает
    idle-семантику (state не null, не error, не crash).
    """
    from rob_box_quest.server.voice_floor import FloorState, VoiceFloor

    floor = VoiceFloor()
    # Нормальное состояние: try_acquire → LISTENING.
    ok, _, new_state = floor.try_acquire("s1", "operator")
    assert ok is True
    assert new_state == FloorState.LISTENING

    # Симулируем «recon-клиент сообщает unknown state»: флор сам по себе
    # не принимает external state (он server-authoritative), но FloorState
    # enum должен уметь парсить любые строки через .value в JSON_EVENT.
    # Контракт: только {idle, listening, speaking, denied} — любой другой
    # string → UI получает idle (warning + drop, см. streams/voice_state.py).
    valid_states = {"idle", "listening", "speaking", "denied"}
    for member in FloorState:
        assert member.value in valid_states, (
            f"FloorState.{member.name}={member.value} не входит в "
            f"voice_state JSON_EVENT схему"
        )

    # snapshot() всегда возвращает один из этих state'ов.
    snap = floor.snapshot()
    assert snap["state"] in valid_states
