"""Integration-тесты WS-сервера rob_box_quest.

Использует aiohttp.test_utils (родной механизм aiohttp, не pytest-плагин).
Без rclpy/ROS/Zenoh — подменяем Bridge на NoOpBridge.
"""

import asyncio
import json
import time

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.server.session import WATCHDOG_TIMEOUT_S
from rob_box_quest.server.ws_server import NoOpBridge, WSSServer, build_app


pytestmark = pytest.mark.asyncio


@pytest.fixture
def fixed_pin(monkeypatch):
    """Фиксируем PIN чтобы тест был детерминирован."""
    pin = "123456"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


@pytest.fixture
async def client(fixed_pin):
    """aiohttp test client без server._ws_handler прямой обработки.
    Запускаем build_app(TestServer) и получаем WS endpoint.
    """
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        yield client, server


async def _open_ws(client):
    """Открыть WS к /quest, ждать пока ready."""
    ws = await client.ws_connect("/quest")
    return ws


async def _send_hello(ws, pin):
    payload = json.dumps({"client_version": "0.1.0", "capabilities": ["webxr"], "session_pin": pin}).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.HELLO, 0, payload))


async def _send_subscribe(ws, topic, quality="med"):
    payload = json.dumps({"topic": topic, "quality": quality}).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.SUBSCRIBE, 0, payload))


async def _send_ping_event(ws, nonce="abc"):
    payload = json.dumps({"type": "ping", "ts_ms": 0, "nonce": nonce}).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.JSON_EVENT, 0, payload))


# --- Tests -----------------------------------------------------------------


async def test_healthz_returns_ok(client):
    http_client, _server = client
    resp = await http_client.get("/healthz")
    assert resp.status == 200
    body = await resp.json()
    assert body["status"] == "ok"


async def test_hello_with_correct_pin_returns_welcome(client, fixed_pin):
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        # Пропустим возможный heartbeat (JSON_EVENT) и ищем WELCOME.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.CLOSE:
                break
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.WELCOME:
                    body = json.loads(payload.decode("utf-8"))
                    assert body["server_version"] == "0.1.0"
                    assert body["session_id"]
                    return
            # иначе продолжаем
        pytest.fail("WELCOME not received")
    finally:
        await ws.close()


async def test_hello_with_wrong_pin_returns_auth_fail_and_closes(client):
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, "000000")  # wrong
        # Должен прийти ERROR{AUTH_FAIL}, затем close.
        deadline = time.monotonic() + 1.0
        got_error = False
        while time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.CLOSE:
                break
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.ERROR:
                    body = json.loads(payload.decode("utf-8"))
                    assert body["code"] == "AUTH_FAIL"
                    got_error = True
        assert got_error, "ERROR{AUTH_FAIL} not received"
    finally:
        # WS уже закрыт сервером; наша close может бросить, ок.
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_subscribe_returns_ack_with_stream_id(client, fixed_pin):
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        # Сначала съедаем WELCOME + возможные heartbeat'ы.
        deadline = time.monotonic() + 1.0
        welcomed = False
        while not welcomed and time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.WELCOME:
                    welcomed = True
            elif msg.type == WSMsgType.CLOSE:
                pytest.fail("closed before WELCOME")
        assert welcomed

        await _send_subscribe(ws, "camera_rear", quality="med")
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.CLOSE:
                pytest.fail("closed before subscribe_ack")
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.JSON_EVENT:
                    body = json.loads(payload.decode("utf-8"))
                    if body.get("type") == "subscribe_ack":
                        assert body["topic"] == "camera_rear"
                        # stream_id — server-initiated (0x1000..0xFFFF),
                        # не обязан совпадать с topic_id.
                        assert 0x1000 <= body["stream_id"] < 0x10000
                        assert body["quality"] == "med"
                        return
        pytest.fail("subscribe_ack not received")
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_subscribe_unknown_topic_returns_error(client, fixed_pin):
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        # Drain WELCOME.
        welcomed = False
        deadline = time.monotonic() + 1.0
        while not welcomed and time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.WELCOME:
                    welcomed = True
        assert welcomed

        # Subscribe на несуществующий топик.
        payload = json.dumps({"topic": "bogus_topic_xyz", "quality": "med"}).encode()
        await ws.send_bytes(encode_frame(FrameType.SUBSCRIBE, 0, payload))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.CLOSE:
                break
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.ERROR:
                    body = json.loads(payload.decode("utf-8"))
                    assert body["code"] == "TOPIC_UNKNOWN"
                    return
        pytest.fail("ERROR{TOPIC_UNKNOWN} not received")
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_stream_list_returns_catalog(client, fixed_pin):
    """JSON_CMD{cmd:stream_list} → JSON_EVENT{type:stream_list, items:[...]}."""
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        welcomed = False
        deadline = time.monotonic() + 1.0
        while not welcomed and time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.WELCOME:
                    welcomed = True
        assert welcomed

        payload = json.dumps({"cmd": "stream_list"}).encode()
        await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, body_bytes = decode_frame(msg.data)
                if ftype == FrameType.JSON_EVENT:
                    body = json.loads(body_bytes.decode("utf-8"))
                    if body.get("type") == "stream_list":
                        items = body["items"]
                        assert isinstance(items, list)
                        topics = {it["topic"] for it in items}
                        assert "lidar_2d" in topics
                        assert "camera_oak_color" in topics
                        assert "camera_ceiling" in topics
                        return
        pytest.fail("stream_list not received")
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_stream_select_unknown_topic_returns_error(client, fixed_pin):
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        welcomed = False
        deadline = time.monotonic() + 1.0
        while not welcomed and time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.WELCOME:
                    welcomed = True
        assert welcomed

        payload = json.dumps({"cmd": "stream_select", "topic": "bogus_camera"}).encode()
        await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, body_bytes = decode_frame(msg.data)
                if ftype == FrameType.ERROR:
                    body = json.loads(body_bytes.decode("utf-8"))
                    assert body["code"] == "TOPIC_UNKNOWN"
                    return
        pytest.fail("ERROR not received")
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_stream_select_known_topic_returns_ack(client, fixed_pin):
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        welcomed = False
        deadline = time.monotonic() + 1.0
        while not welcomed and time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.WELCOME:
                    welcomed = True
        assert welcomed

        payload = json.dumps({"cmd": "stream_select", "topic": "camera_oak_color"}).encode()
        await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, body_bytes = decode_frame(msg.data)
                if ftype == FrameType.JSON_EVENT:
                    body = json.loads(body_bytes.decode("utf-8"))
                    if body.get("type") == "stream_select_ack":
                        assert body["topic"] == "camera_oak_color"
                        assert body["kind"] == "camera_direct"
                        # Не подписан → stream_id=None → клиент делает SUBSCRIBE
                        assert body["stream_id"] is None
                        return
        pytest.fail("stream_select_ack not received")
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_heartbeat_is_sent_periodically(client, fixed_pin):
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        # Drain WELCOME.
        welcomed = False
        deadline = time.monotonic() + 1.0
        while not welcomed and time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.WELCOME:
                    welcomed = True
        assert welcomed
        # Продолжаем читать — должны получить heartbeat в течение ~400 мс.
        deadline = time.monotonic() + 1.0
        got_heartbeat = False
        while not got_heartbeat and time.monotonic() < deadline:
            msg = await ws.receive(timeout=0.5)
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.JSON_EVENT:
                    body = json.loads(payload.decode("utf-8"))
                    if body.get("type") == "heartbeat":
                        assert "ts_ms" in body
                        got_heartbeat = True
        assert got_heartbeat, "heartbeat not received within 1s"
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_watchdog_closes_socket_without_ping(client, fixed_pin):
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        # Drain WELCOME.
        welcomed = False
        deadline = time.monotonic() + 1.0
        while not welcomed and time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.WELCOME:
                    welcomed = True
        assert welcomed
        # Теперь молчим. Сервер должен закрыть сокет по watchdog.
        # WATCHDOG_TIMEOUT_S = 0.6 с, плюс check раз в 0.3 с.
        deadline = time.monotonic() + WATCHDOG_TIMEOUT_S + 1.0
        closed = False
        while time.monotonic() < deadline:
            try:
                msg = await ws.receive(timeout=0.5)
            except asyncio.TimeoutError:
                continue
            if msg.type == WSMsgType.CLOSE:
                closed = True
                break
        assert closed, "server did not close socket on watchdog timeout"
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_ping_resets_watchdog(client, fixed_pin):
    """Если клиент шлёт ping, watchdog сбрасывается и close не происходит."""
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        welcomed = False
        deadline = time.monotonic() + 1.0
        while not welcomed and time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.WELCOME:
                    welcomed = True
        assert welcomed

        # Шлём ping каждые 200 мс в течение 1.2 с (больше одного watchdog окна).
        start = time.monotonic()
        while time.monotonic() - start < 1.2:
            await _send_ping_event(ws)
            await asyncio.sleep(0.2)
            # Drain любых incoming binary чтобы receive не блокировал.
            try:
                msg = await ws.receive(timeout=0.05)
                if msg.type == WSMsgType.CLOSE:
                    pytest.fail("closed despite active pings")
            except asyncio.TimeoutError:
                pass
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass
