"""Unit-тесты для новой функциональности issue #2190 (voice-vr 05).

Покрывает:
- FloorView.update → FloorViewUpdate diff (AvatarStateFloorCache.update).
- WSSServer.update_floor_cache → возвращает diff.
- WSSServer.notify_floor_lost_external → JSON_EVENT{floor_lost} в сокет.
- E2E сценарий «avatar_supervisor снял floor» (через update_floor_cache):
  клиент-держатель получает floor_lost event.

Запуск:
    PYTHONPATH=src/rob_box_quest pytest src/rob_box_quest/test/unit/server/test_voice_vr_05_floor_view.py -v
"""

from __future__ import annotations

import asyncio
import json
import time

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.core.floor import (
    AvatarFloorSnapshot,
    AvatarStateFloorCache,
    FloorViewUpdate,
    make_server_client_id,
)
from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.server.ws_server import NoOpBridge, WSSServer, build_app


# ── helpers (copied из test_ws_server_av19 — избегаем cross-test imports) ─


@pytest.fixture
def fixed_pin(monkeypatch):
    pin = "123456"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


async def _open_ws(client):
    return await client.ws_connect("/quest")


async def _send_hello(ws, pin):
    payload = json.dumps(
        {
            "client_version": "0.1.0",
            "capabilities": ["webxr"],
            "session_pin": pin,
        }
    ).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.HELLO, 0, payload))


async def _send_client_ping(ws):
    payload = json.dumps({"cmd": "ping"}).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))


async def _keepalive_pings(ws, interval_s: float = 0.2) -> None:
    try:
        while True:
            await asyncio.sleep(interval_s)
            await _send_client_ping(ws)
    except asyncio.CancelledError:
        return


async def _read_frame(ws, *, type_filter=None, event_type=None, timeout=1.0):
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
            if type_filter is not None and ftype != type_filter:
                continue
            if event_type is not None and ftype == FrameType.JSON_EVENT:
                try:
                    inner = json.loads(payload.decode("utf-8"))
                except Exception:  # noqa: BLE001
                    continue
                if inner.get("type") != event_type:
                    continue
            return ftype, payload
    return None


async def _authenticate(client, pin):
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


# ── FloorViewUpdate diff — pure unit test ────────────────────────────────


def test_floor_view_update_no_change():
    """Идемпотентный update с тем же snapshot → diff с пустыми флагами."""
    cache = AvatarStateFloorCache(
        initial_snapshot=AvatarFloorSnapshot(teleop_holder="quest:alice")
    )
    diff = cache.update(AvatarFloorSnapshot(teleop_holder="quest:alice"))
    assert isinstance(diff, FloorViewUpdate)
    assert diff.teleop_holder_changed is False
    assert diff.teleop_lost is False
    assert diff.teleop_replaced is False
    assert diff.prev_teleop_holder == "quest:alice"
    assert diff.next_teleop_holder == "quest:alice"


def test_floor_view_update_lost():
    """Holder → None: prev держал, теперь никто. teleop_lost=True."""
    cache = AvatarStateFloorCache(
        initial_snapshot=AvatarFloorSnapshot(teleop_holder="quest:alice")
    )
    diff = cache.update(AvatarFloorSnapshot(teleop_holder=None))
    assert diff.teleop_holder_changed is True
    assert diff.teleop_lost is True
    assert diff.teleop_replaced is False
    assert diff.prev_teleop_holder == "quest:alice"
    assert diff.next_teleop_holder is None


def test_floor_view_update_replaced():
    """Holder alice → holder bob: кто-то держал, стал другой. teleop_replaced=True."""
    cache = AvatarStateFloorCache(
        initial_snapshot=AvatarFloorSnapshot(teleop_holder="quest:alice")
    )
    diff = cache.update(AvatarFloorSnapshot(teleop_holder="quest:bob"))
    assert diff.teleop_holder_changed is True
    assert diff.teleop_lost is False
    assert diff.teleop_replaced is True
    assert diff.prev_teleop_holder == "quest:alice"
    assert diff.next_teleop_holder == "quest:bob"


def test_floor_view_update_acquired():
    """None → holder: floor стал занят. teleop_lost=False, replaced=False."""
    cache = AvatarStateFloorCache()
    diff = cache.update(AvatarFloorSnapshot(teleop_holder="quest:carol"))
    assert diff.teleop_holder_changed is True
    assert diff.teleop_lost is False
    assert diff.teleop_replaced is False
    assert diff.prev_teleop_holder is None
    assert diff.next_teleop_holder == "quest:carol"


# ── WSSServer.update_floor_cache — integration ───────────────────────────


@pytest.mark.asyncio
async def test_update_floor_cache_returns_diff():
    """update_floor_cache через WSSServer — возвращает FloorViewUpdate."""
    cache = AvatarStateFloorCache()
    server = WSSServer(
        bridge=NoOpBridge(),
        pin="123456",
        require_teleop_floor=True,
        floor_cache=cache,
    )
    diff = server.update_floor_cache(
        AvatarFloorSnapshot(teleop_holder="quest:alice")
    )
    assert isinstance(diff, FloorViewUpdate)
    assert diff.next_teleop_holder == "quest:alice"
    # Cache обновился.
    assert server._floor_cache.holder == "quest:alice"


# ── notify_floor_lost_external — клиент получает JSON_EVENT{floor_lost} ──


@pytest.mark.asyncio
async def test_notify_floor_lost_external_sends_event_to_holder(fixed_pin):
    """E2E: avatar_supervisor снимает floor → клиент-держатель получает floor_lost.

    Сценарий issue #2190 acceptance:
      - Клиент держит teleop_floor (server_client_id = "quest:<uuid>").
      - avatar_supervisor снимает floor (или перехватывает) → /avatar/state
        приходит с teleop_holder=None.
      - QuestBridge.on_avatar_state зовёт update_floor_cache → diff.teleop_lost
        с prev_holder = наш client_id.
      - ws_server.notify_floor_lost_external шлёт JSON_EVENT{floor_lost} в
        сокет клиента. Клиент DISARM-ит (см. main.ts:1586).
    """
    bridge = NoOpBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=True)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws, sid = await _authenticate(client, fixed_pin)
        assert server._avatar_arbiter.floor_holder == make_server_client_id(sid)
        # Watchdog-loop не даёт сессии умереть, пока фильтруем events.
        ping_task = asyncio.create_task(_keepalive_pings(ws))
        try:
            # Имитируем «avatar_supervisor перехватил floor»: cache
            # переходит с нашего server_client_id на None.
            prev_holder = make_server_client_id(sid)
            diff = server.update_floor_cache(
                AvatarFloorSnapshot(teleop_holder=None)
            )
            assert diff.teleop_lost is True
            assert diff.prev_teleop_holder == prev_holder

            # QuestBridge.on_avatar_state вызвал бы это:
            server.notify_floor_lost_external(
                prev_holder, reason="avatar_supervisor_released"
            )

            evt = await _read_frame(
                ws,
                type_filter=FrameType.JSON_EVENT,
                event_type="floor_lost",
                timeout=1.0,
            )
            assert evt is not None, "JSON_EVENT{floor_lost} не пришёл клиенту"
            _, payload = evt
            body = json.loads(payload.decode("utf-8"))
            assert body.get("type") == "floor_lost"
            assert body.get("floor") == "teleop"
            assert body.get("reason") == "avatar_supervisor_released"
        finally:
            ping_task.cancel()
            try:
                await ping_task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


@pytest.mark.asyncio
async def test_notify_floor_lost_external_noop_for_unknown_client(fixed_pin):
    """notify_floor_lost_external для неизвестного client_id — no-op."""
    bridge = NoOpBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=True)
    # Без активной сессии вообще.
    server.notify_floor_lost_external(
        "quest:nonexistent", reason="test_noop"
    )  # просто не должно кидать исключение


@pytest.mark.asyncio
async def test_notify_floor_lost_external_noop_when_holder_unchanged(fixed_pin):
    """Если diff не показывает потерю — notify не вызывается.

    Идемпотентный /avatar/state (тот же holder) → diff.teleop_lost=False →
    QuestBridge не зовёт notify_floor_lost_external.
    """
    bridge = NoOpBridge()
    server = WSSServer(bridge=bridge, pin=fixed_pin, require_teleop_floor=True)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws, sid = await _authenticate(client, fixed_pin)
        holder = make_server_client_id(sid)
        # Сначала cache приводим в соответствие с arbiter.
        server._floor_cache.update(AvatarFloorSnapshot(teleop_holder=holder))
        ping_task = asyncio.create_task(_keepalive_pings(ws))
        try:
            # Повторный /avatar/state с тем же holder → diff без изменений.
            diff = server.update_floor_cache(
                AvatarFloorSnapshot(teleop_holder=holder)
            )
            assert diff.teleop_holder_changed is False
            assert diff.teleop_lost is False
            # QuestBridge НЕ зовёт notify — имитируем это и проверяем,
            # что клиент НЕ получает лишний floor_lost event.
            evt = await _read_frame(
                ws,
                type_filter=FrameType.JSON_EVENT,
                event_type="floor_lost",
                timeout=0.3,
            )
            assert evt is None, (
                f"клиент НЕ должен получать floor_lost на идемпотентном "
                f"/avatar/state; got={evt}"
            )
        finally:
            ping_task.cancel()
            try:
                await ping_task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass
