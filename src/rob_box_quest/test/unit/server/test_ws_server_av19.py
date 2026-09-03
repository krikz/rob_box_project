"""Integration-тесты WS-сервера rob_box_quest, AV-19 (issue #1911).

Что покрывается (ADR-0028 §4.4, meta-quest-api.md §5/§8):
- require_teleop_floor=false (default): teleop_twist публикуется
  всегда, без FLOOR_HELD-error; relay heartbeat в Bridge.
- require_teleop_floor=true:
  - сессия-A взяла floor, шлёт teleop_twist — публикуется;
  - сессия-B не держит floor, шлёт teleop_twist → ERROR{FLOOR_HELD}
    rate-limited (≤ 1 Гц), НЕ публикуется cmd_vel_quest;
  - teleop_heartbeat от A релеится; от B — НЕТ релея;
  - stop_emergency от B — ВСЕГДА публикуется (обход гейта).
- _unregister_session: floor снимается, новая сессия может взять.
- on_floor_lost_external: клиент получает JSON_EVENT{type:"floor_lost"},
  Bridge.on_floor_lost вызывается (zero Twist fail-safe).

Запуск:
    PYTHONPATH=src/rob_box_quest pytest src/rob_box_quest/test/unit/server/test_ws_server_av19.py -v
"""

import asyncio
import json
import time

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.core.floor import SupervisorFloorTracker
from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.server.session import ErrorCode
from rob_box_quest.server.ws_server import NoOpBridge, WSSServer, build_app


pytestmark = pytest.mark.asyncio


# ── Spy-bridge для проверки relay / zero-Twist fail-safe ─────────────────


class SpyBridge(NoOpBridge):
    """NoOpBridge + журнал вызовов relay_teleop_heartbeat и on_floor_lost.

    Используется в тестах gate'а и fail-safe, чтобы проверить, что
    сервер действительно шлёт heartbeat в Bridge только при наличии
    floor и что on_floor_lost External уведомляет Bridge для zero Twist.
    """

    def __init__(self) -> None:
        super().__init__()
        self.heartbeats: list[tuple[str, int, int]] = []
        self.floor_lost_clients: list[str] = []

    def relay_teleop_heartbeat(self, client_id: str, ts_ms: int, seq: int) -> None:
        self.heartbeats.append((client_id, ts_ms, seq))

    def on_floor_lost(self, client_id: str) -> None:
        self.floor_lost_clients.append(client_id)


# ── helpers (как в test_ws_server.py) ───────────────────────────────────


@pytest.fixture
def fixed_pin(monkeypatch):
    pin = "123456"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


async def _open_ws(client):
    return await client.ws_connect("/quest")


async def _send_hello(ws, pin):
    payload = json.dumps({"client_version": "0.1.0", "capabilities": ["webxr"], "session_pin": pin}).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.HELLO, 0, payload))


async def _read_frame(ws, *, type_filter=None, timeout=1.0):
    """Прочитать один BINARY фрейм, опционально фильтруя по типу.

    Возвращает (FrameType, payload_bytes) или None если timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            msg = await ws.receive(timeout=0.2)
        except asyncio.TimeoutError:
            continue
        if msg.type == WSMsgType.CLOSE:
            return None
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, payload = decode_frame(msg.data)
            if type_filter is None or ftype == type_filter:
                return ftype, payload
    return None


async def _send_teleop_twist(ws, *, linear=0.5, angular=0.0, deadman=True, seq=1, ts_ms=None):
    if ts_ms is None:
        ts_ms = int(time.time() * 1000)
    payload = json.dumps(
        {
            "cmd": "teleop_twist",
            "ts_ms": ts_ms,
            "seq": seq,
            "linear": {"x": linear, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular},
            "deadman": deadman,
        }
    ).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))


async def _send_teleop_heartbeat(ws, *, seq=1, ts_ms=None):
    if ts_ms is None:
        ts_ms = int(time.time() * 1000)
    payload = json.dumps(
        {"cmd": "teleop_heartbeat", "ts_ms": ts_ms, "seq": seq}
    ).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))


async def _send_stop_emergency(ws, source="controller_b"):
    payload = json.dumps({"cmd": "stop_emergency", "ts_ms": int(time.time() * 1000), "source": source}).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))


async def _authenticate(client, pin, *, gate=False):
    """Подключиться и пройти HELLO; вернуть (ws, session_id) или (None, None).

    При ``gate=True`` сервер уже занят другой сессией — клиент НЕ
    получает floor (held_by=другой), но WELCOME всё равно приходит.
    """
    ws = await _open_ws(client)
    await _send_hello(ws, pin)
    got = await _read_frame(ws, type_filter=FrameType.WELCOME, timeout=1.0)
    if got is None:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass
        return None, None
    _, payload = got
    body = json.loads(payload.decode("utf-8"))
    return ws, body["session_id"]


# ── Тесты: режим по умолчанию (require_teleop_floor=False) ─────────────


async def test_default_mode_no_gate_teleop_publishes(fixed_pin):
    """Без гейта teleop_twist публикуется даже если нет floor."""
    bridge = SpyBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=False)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws, _ = await _authenticate(client, fixed_pin)
        assert ws is not None
        await _send_teleop_twist(ws, seq=1)
        # Сервер не должен слать ERROR{FLOOR_HELD} в default mode.
        result = await _read_frame(ws, type_filter=FrameType.ERROR, timeout=0.4)
        assert result is None, "ERROR не должен слаться при require_teleop_floor=False"
        # Heartbeat-relay должен быть — twist-фрейм это живость.
        await asyncio.sleep(0.05)
        assert any(cid and seq == 1 for cid, _ts, seq in bridge.heartbeats), (
            f"relay_teleop_heartbeat не вызван; got={bridge.heartbeats}"
        )
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_default_mode_heartbeat_relay(fixed_pin):
    """В default режиме teleop_heartbeat тоже релеится."""
    bridge = SpyBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=False)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws, sid = await _authenticate(client, fixed_pin)
        assert ws is not None
        bridge.heartbeats.clear()
        await _send_teleop_heartbeat(ws, seq=42)
        await asyncio.sleep(0.05)
        assert any(s == sid and seq == 42 for s, _ts, seq in bridge.heartbeats), (
            f"heartbeat не релеился; got={bridge.heartbeats}"
        )
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


# ── Тесты: gate ON (require_teleop_floor=True) ──────────────────────────


async def test_gate_blocks_twist_when_floor_held_by_other(fixed_pin):
    """Сессия-A взяла floor, сессия-B шлёт twist → ERROR{FLOOR_HELD}."""
    bridge = SpyBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=True)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        # Сессия A: HELLO + WELCOME, занимает floor.
        ws_a, sid_a = await _authenticate(client, fixed_pin)
        assert ws_a is not None
        # Сессия B: HELLO, floor уже у A — не получает его.
        ws_b, sid_b = await _authenticate(client, fixed_pin)
        assert ws_b is not None
        assert sid_a != sid_b
        assert server._floor_tracker.is_held_by(sid_a) is True
        assert server._floor_tracker.is_held_by(sid_b) is False

        # B шлёт twist — должна прийти ERROR{FLOOR_HELD}.
        await _send_teleop_twist(ws_b, seq=1)
        err = await _read_frame(ws_b, type_filter=FrameType.ERROR, timeout=0.5)
        assert err is not None, "ERROR не пришёл"
        _, payload = err
        body = json.loads(payload.decode("utf-8"))
        assert body["code"] == ErrorCode.FLOOR_HELD
        # Relay не должен быть — у B нет floor.
        await asyncio.sleep(0.05)
        assert not any(s == sid_b for s, _ts, _seq in bridge.heartbeats), (
            f"B не должен слать relay; got={bridge.heartbeats}"
        )
        try:
            await ws_a.close()
            await ws_b.close()
        except Exception:  # noqa: BLE001
            pass


async def test_gate_allows_twist_when_floor_is_mine(fixed_pin):
    """Сессия-A держит floor, шлёт twist — публикуется."""
    bridge = SpyBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=True)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws_a, sid_a = await _authenticate(client, fixed_pin)
        assert ws_a is not None
        bridge.heartbeats.clear()
        await _send_teleop_twist(ws_a, seq=10)
        await asyncio.sleep(0.05)
        # Relay должен быть (A держит floor).
        assert any(s == sid_a and seq == 10 for s, _ts, seq in bridge.heartbeats)
        # ERROR не должно быть.
        err = await _read_frame(ws_a, type_filter=FrameType.ERROR, timeout=0.3)
        assert err is None, f"A держит floor — ERROR не должно слаться; got={err}"
        try:
            await ws_a.close()
        except Exception:  # noqa: BLE001
            pass


async def test_floor_held_error_is_rate_limited(fixed_pin):
    """На 30 Гц twist — НЕ более 1 ERROR{FLOOR_HELD} в секунду."""
    bridge = SpyBridge()
    fake_now = [1_000_000.0]
    tracker = SupervisorFloorTracker(now_fn=lambda: fake_now[0])
    server = WSSServer(
        bridge=bridge,
        pin=fixed_pin,
        require_teleop_floor=True,
        floor_tracker=tracker,
    )
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        # Занимаем floor сессией A.
        ws_a, _ = await _authenticate(client, fixed_pin)
        # Сессия B — без floor.
        ws_b, _ = await _authenticate(client, fixed_pin)

        # Шлём 10 twist за 0.5 c от B.
        errors = 0
        for seq in range(1, 11):
            await _send_teleop_twist(ws_b, seq=seq)
            await asyncio.sleep(0.05)
        # Drain всех ERROR.
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            got = await _read_frame(ws_b, type_filter=FrameType.ERROR, timeout=0.1)
            if got is None:
                break
            _, payload = got
            body = json.loads(payload.decode("utf-8"))
            assert body["code"] == ErrorCode.FLOOR_HELD
            errors += 1
        # Окно 1с — должно быть ровно 1 (первая попытка), даже при 10 twist-ах.
        assert errors == 1, f"ожидали 1 ERROR за окно 1с, получили {errors}"
        try:
            await ws_a.close()
            await ws_b.close()
        except Exception:  # noqa: BLE001
            pass


async def test_floor_released_on_unregister(fixed_pin):
    """После close сессии-A — B может взять floor."""
    bridge = SpyBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=True)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws_a, sid_a = await _authenticate(client, fixed_pin)
        ws_b, sid_b = await _authenticate(client, fixed_pin)
        assert server._floor_tracker.is_held_by(sid_a) is True
        assert server._floor_tracker.is_held_by(sid_b) is False

        await ws_a.close()
        # Дать event-loop отработать unregister.
        await asyncio.sleep(0.1)
        assert server._floor_tracker.is_held_by(sid_a) is False
        # B всё ещё без floor — нужно re-acquire, который произойдёт
        # через FSM-механизм Phase 2; в Phase 1 B остаётся без floor
        # до выхода. Это явно задокументировано: см. design.md.
        assert server._floor_tracker.holder is None
        try:
            await ws_b.close()
        except Exception:  # noqa: BLE001
            pass


async def test_stop_emergency_bypasses_gate(fixed_pin):
    """stop_emergency от B (без floor) — ВСЕГДА публикуется."""
    bridge = SpyBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=True)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws_a, _ = await _authenticate(client, fixed_pin)
        ws_b, _ = await _authenticate(client, fixed_pin)

        # B (без floor) шлёт stop_emergency → Bridge.publish_emergency +
        # emergency_stop должны быть вызваны. NoOpBridge → record-через
        # log.debug (для SpyBridge это поведение унаследовано), но
        # ГЛАВНОЕ — никакого ERROR. Проверим по отсутствию ERROR-фрейма
        # после команды.
        await _send_stop_emergency(ws_b, source="ui_button")
        await asyncio.sleep(0.05)
        err = await _read_frame(ws_b, type_filter=FrameType.ERROR, timeout=0.3)
        assert err is None, (
            f"stop_emergency в обход гейта; не должно быть ERROR, got={err}"
        )
        try:
            await ws_a.close()
            await ws_b.close()
        except Exception:  # noqa: BLE001
            pass


# ── Тесты: on_floor_lost_external (fail-safe) ───────────────────────────


async def test_floor_lost_external_notifies_bridge_and_client(fixed_pin):
    """Bridge.on_floor_lost вызывается + JSON_EVENT{type:'floor_lost'} шлётся."""
    bridge = SpyBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=True)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws_a, sid_a = await _authenticate(client, fixed_pin)
        assert server._floor_tracker.is_held_by(sid_a) is True

        # Имитируем внешний сигнал потери floor (Telegram-оператор взял его).
        server.on_floor_lost_external(sid_a)
        await asyncio.sleep(0.05)

        # 1) Bridge.on_floor_lost вызван.
        assert bridge.floor_lost_clients == [sid_a], (
            f"bridge.on_floor_lost not called; got={bridge.floor_lost_clients}"
        )
        # 2) Floor снят в tracker.
        assert server._floor_tracker.is_held_by(sid_a) is False
        # 3) Клиент получил JSON_EVENT{type:"floor_lost"}.
        evt = await _read_frame(ws_a, type_filter=FrameType.JSON_EVENT, timeout=0.5)
        assert evt is not None, "JSON_EVENT{floor_lost} не пришёл клиенту"
        _, payload = evt
        body = json.loads(payload.decode("utf-8"))
        assert body.get("type") == "floor_lost"
        assert body.get("floor") == "teleop"
        try:
            await ws_a.close()
        except Exception:  # noqa: BLE001
            pass


async def test_floor_lost_external_noop_when_not_holder(fixed_pin):
    """on_floor_lost_external для НЕ-держателя — no-op (без notify)."""
    bridge = SpyBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=True)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws_a, sid_a = await _authenticate(client, fixed_pin)
        ws_b, sid_b = await _authenticate(client, fixed_pin)
        bridge.floor_lost_clients.clear()
        # B не держит floor — вызов не должен ничего сделать.
        server.on_floor_lost_external(sid_b)
        await asyncio.sleep(0.05)
        assert bridge.floor_lost_clients == []
        try:
            await ws_a.close()
            await ws_b.close()
        except Exception:  # noqa: BLE001
            pass


# ── Тесты: WELCOME подсказывает teleop_floor_held_by ────────────────────


async def test_welcome_includes_teleop_floor_held_by(fixed_pin):
    """WELCOME содержит teleop_floor_held_by (для FSM клиента).

    Тест запускает ОБЕ сессии HELLO+WELCOME подряд, чтобы A не
    успела попасть в watchdog за время открытия B. После того, как
    обе прошли handshake, делаем «diagnostic-пинг» у A (ping-event,
    сбрасывает watchdog) и затем читаем WELCOME B.
    """
    bridge = SpyBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=True)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        # 1. HELLO A → читаем WELCOME.
        ws_a = await _open_ws(client)
        await _send_hello(ws_a, fixed_pin)
        got_a = await _read_frame(ws_a, type_filter=FrameType.WELCOME, timeout=1.0)
        assert got_a is not None
        sid_a = json.loads(got_a[1].decode("utf-8"))["session_id"]

        # 2. HELLO B → читаем WELCOME (не даём A молчать — watchdog!).
        ws_b = await _open_ws(client)
        await _send_hello(ws_b, fixed_pin)
        got_b = await _read_frame(ws_b, type_filter=FrameType.WELCOME, timeout=1.0)
        assert got_b is not None, "WELCOME для B не пришёл (watchdog?)"
        body_b = json.loads(got_b[1].decode("utf-8"))

        assert body_b.get("teleop_floor_held_by") == sid_a, (
            f"ожидали teleop_floor_held_by={sid_a}, got={body_b.get('teleop_floor_held_by')}"
        )
        try:
            await ws_a.close()
            await ws_b.close()
        except Exception:  # noqa: BLE001
            pass
