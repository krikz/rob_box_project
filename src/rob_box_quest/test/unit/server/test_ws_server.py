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


async def test_ping_gets_pong_with_echoed_ts(client, fixed_pin):
    """Wave 3.A: pong с эхом ts_ms — клиент считает RTT по своим часам."""
    http_client, _server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        welcomed = False
        deadline = time.monotonic() + 1.0
        while not welcomed and time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, _payload = decode_frame(msg.data)
                if ftype == FrameType.WELCOME:
                    welcomed = True
        assert welcomed

        sent_ts = 1_700_000_000_123
        payload = json.dumps({"type": "ping", "ts_ms": sent_ts}).encode("utf-8")
        await ws.send_bytes(encode_frame(FrameType.JSON_EVENT, 0, payload))

        got_pong = False
        deadline = time.monotonic() + 1.0
        while not got_pong and time.monotonic() < deadline:
            msg = await ws.receive(timeout=0.5)
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.JSON_EVENT:
                    body = json.loads(payload.decode("utf-8"))
                    if body.get("type") == "pong":
                        assert body["ts_ms"] == sent_ts
                        assert body["server_ts_ms"] > 1_700_000_000_000
                        got_pong = True
        assert got_pong, "pong not received within 1s"
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


# --- broadcast_json_event (robot_alert fan-out) ----------------------------


async def test_broadcast_json_event_reaches_authenticated_session(client, fixed_pin):
    """JSON_EVENT{type:robot_alert} → доставляется ВСЕМ открытым сессиям
    без подписки на topic (control-frame, stream_id=0)."""
    http_client, server = client
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

        # broadcast_json_event без подписки — control-frame.
        server.broadcast_json_event({
            "type": "robot_alert",
            "code": "BATTERY_LOW",
            "active": True,
            "level": "warn",
            "args": {"pct": 12},
            "ts_ms": 1_700_000_000_000,
        })

        deadline = time.monotonic() + 1.0
        got_alert = False
        while not got_alert and time.monotonic() < deadline:
            msg = await ws.receive(timeout=0.5)
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(msg.data)
                if ftype == FrameType.JSON_EVENT:
                    body = json.loads(payload.decode("utf-8"))
                    if body.get("type") == "robot_alert":
                        assert body["code"] == "BATTERY_LOW"
                        assert body["active"] is True
                        assert body["level"] == "warn"
                        assert body["args"] == {"pct": 12}
                        got_alert = True
        assert got_alert, "robot_alert not delivered"
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_broadcast_json_event_uses_stream_id_zero(client, fixed_pin):
    """JSON_EVENT от broadcast_json_event всегда stream_id=0 (control)."""
    http_client, server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        welcomed = False
        deadline = time.monotonic() + 1.0
        while not welcomed and time.monotonic() < deadline:
            msg = await ws.receive()
            if msg.type == WSMsgType.BINARY:
                ftype, _sid, _ = decode_frame(msg.data)
                if ftype == FrameType.WELCOME:
                    welcomed = True
        assert welcomed

        server.broadcast_json_event({"type": "robot_alert", "code": "WIFI_WEAK"})

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            msg = await ws.receive(timeout=0.5)
            if msg.type == WSMsgType.BINARY:
                ftype, sid, payload = decode_frame(msg.data)
                if ftype == FrameType.JSON_EVENT:
                    body = json.loads(payload.decode("utf-8"))
                    if body.get("type") == "robot_alert":
                        assert sid == 0  # control-frame
                        return
        pytest.fail("robot_alert not received")
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_broadcast_json_event_no_sessions_returns_zero():
    """Если никто не подключён — broadcast возвращает 0 и не падает."""
    from rob_box_quest.server.ws_server import WSSServer
    server = WSSServer(bridge=NoOpBridge(), pin="000000")
    count = server.broadcast_json_event({"type": "robot_alert", "code": "X"})
    assert count == 0


# ── issue #2099/#2100 — один упавший cmd-хендлер НЕ должен убивать сессию ──
#
# Диагностика (2026-09-07): гипотеза issue #2100 «register_audio_session
# (operator_tts) возвращает False на дефолтном лимите» ОПРОВЕРГНУТА —
# ``test_register_audio_session_acceptance_default_limit`` в
# ``test_ws_server_deliver_audio.py`` зелёный на дефолтном
# ``VOICE_PREVIEW_MAX_CONCURRENT``. Реальный разрыв обратного канала —
# в ``_ws_handler`` (этот файл, было: строки 2216-2300 одним большим
# if/elif БЕЗ try/except вокруг диспетчера, весь блок ловился только
# внешним ``except Exception`` вместе с ``finally: self._unregister_session``).
# Любое необработанное исключение внутри ЛЮБОГО cmd-хендлера — например,
# ``NameError: name '_voices_for' is not defined`` из issue #2099
# (``bridge.set_voice``, реальный траблшут с робота) — убивало ВСЮ
# WS-сессию, а не только упавшую команду. ``deliver_audio(stream=
# "operator_tts", ...)`` адресуется по ``ws``, зарегистрированному ЗА ЭТУ
# сессию (``register_audio_session``) — если сессия умерла, реплика ТАРС
# в шлем пропадает без единого предупреждения выше по стеку супервизора.
#
# Фикс — ``ws_server.py:_ws_handler``: try/except обёрнут вокруг ОДНОГО
# кадра (не вокруг всего ``async for``), так что кривой cmd логируется,
# клиенту уходит ``ERROR{INTERNAL}``, но сессия и все её audio-регистрации
# остаются живы.


async def _send_set_voice_cmd(ws, voice_id: str = "ru-RU-voice") -> None:
    payload = json.dumps({"cmd": "set_voice", "mode": "voice", "voice_id": voice_id}).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))


async def _wait_for_welcome(ws) -> None:
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        msg = await ws.receive()
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, _payload = decode_frame(msg.data)
            if ftype == FrameType.WELCOME:
                return
    pytest.fail("WELCOME not received")


async def _drain_until(ws, want_ftype, timeout: float = 2.0, body_type: str = None):
    """Читает кадры пока не встретит ``want_ftype`` (или CLOSE → fail).

    Если ``body_type`` задан — для ``FrameType.JSON_EVENT`` фильтрует ещё и
    по ``body["type"]`` (иначе наши собственные keep-alive ``pong``,
    которые шлёт этот же helper, ложно совпали бы по ftype).

    Шлёт ping каждые ~150 мс, чтобы долгое ожидание в тесте (под pytest +
    aiohttp TestServer это иногда медленнее, чем 0.6с) не словило
    watchdog (``WATCHDOG_TIMEOUT_S``) и не закрыло сессию по ПОСТОРОННЕЙ
    для теста причине. Возвращает декодированный payload (dict).
    """
    deadline = time.monotonic() + timeout
    last_ping = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_ping > 0.15:
            await _send_ping_event(ws)
            last_ping = now
        try:
            msg = await ws.receive(timeout=0.1)
        except asyncio.TimeoutError:
            continue
        if msg.type == WSMsgType.CLOSE:
            pytest.fail(f"socket closed while waiting for ftype={want_ftype}")
        if msg.type != WSMsgType.BINARY:
            continue
        ftype, _sid, payload = decode_frame(msg.data)
        if ftype != want_ftype:
            continue
        body = {}
        if payload:
            try:
                body = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                body = {}
        if body_type is not None and body.get("type") != body_type:
            continue
        return body
    pytest.fail(f"ftype={want_ftype} (type={body_type}) not received within {timeout}s")


async def test_crashing_cmd_handler_does_not_kill_ws_session(
    client, fixed_pin, monkeypatch
):
    """Issue #2099 regression: NameError в bridge.set_voice не рвёт сессию.

    До фикса это ронялось в ``ws_handler crashed`` (внешний except) и
    ``_unregister_session`` закрывал сокет — ровно траблшут с робота
    (2026-09-07, 16:48 и 16:55).
    """
    http_client, server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        await _wait_for_welcome(ws)
        assert server.get_active_sessions() == 1

        # Симулируем ровно баг issue #2099.
        def _boom(voice_id, preset):
            raise NameError("name '_voices_for' is not defined")

        monkeypatch.setattr(server.bridge, "set_voice", _boom)
        await _send_set_voice_cmd(ws)

        err_body = await _drain_until(ws, FrameType.ERROR)
        assert err_body.get("code") == "INTERNAL"

        # Сессия осталась зарегистрированной и живой...
        assert server.get_active_sessions() == 1

        # ...и продолжает обслуживать дальнейшие кадры как ни в чём не бывало.
        pong_body = await _drain_until(ws, FrameType.JSON_EVENT, body_type="pong")
        assert "ts_ms" in pong_body
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_operator_tts_audio_survives_unrelated_crashing_cmd(
    client, fixed_pin, monkeypatch
):
    """Issue #2100 — операторский аудиоканал переживает баг issue #2099.

    Прямой сквозной регресс-тест на связку #2099→#2100: регистрируем
    ``operator_tts`` request_id → ws (как это делает
    ``quest_node._on_avatar_tts_request_meta``), затем ломаем сессию
    падающим ``set_voice`` (issue #2099), и проверяем, что
    ``deliver_audio(stream="operator_tts", ...)`` для РАНЕЕ
    зарегистрированного request_id всё ещё долетает клиенту — сессия и
    её audio-регистрация не были стёрты падением постороннего cmd.
    """
    http_client, server = client
    ws = await _open_ws(http_client)
    try:
        await _send_hello(ws, fixed_pin)
        await _wait_for_welcome(ws)
        assert server.get_active_sessions() == 1

        # deliver_audio шлёт fire-and-forget через _send_loop (в проде
        # quest_node зовёт set_send_loop() на старте — deliver_audio
        # вызывается из ROS-потока, не из aiohttp-loop). Тестовый клиент
        # это не делает сам — воспроизводим ту же проводку явно.
        server.set_send_loop(asyncio.get_running_loop())

        # ws_server-сторонний объект ws (тот самый, что видит deliver_audio).
        server_ws = next(iter(server._ws_by_session.values()))
        assert server.register_audio_session("operator_tts", "req-tars-1", server_ws)

        # Ломаем сессию не связанным с TTS багом (issue #2099).
        def _boom(voice_id, preset):
            raise NameError("name '_voices_for' is not defined")

        monkeypatch.setattr(server.bridge, "set_voice", _boom)
        await _send_set_voice_cmd(ws)

        # Дренируем ERROR{INTERNAL} — не требуем его в этом тесте, фокус
        # на том, что audio-регистрация не пострадала. Хелпер сам держит
        # watchdog живым пингами, так что дальнейшее ожидание безопасно.
        await _drain_until(ws, FrameType.ERROR)

        assert server.get_active_sessions() == 1

        delivered = server.deliver_audio(
            stream="operator_tts",
            request_id="req-tars-1",
            audio_bytes=b"\x01\x02\x03\x04",
            audio_format="pcm_s16le",
            content_type="audio/pcm",
            seq=0,
            total=0,
        )
        assert delivered, (
            "deliver_audio(operator_tts) failed for a request_id registered "
            "BEFORE the crashing cmd — the session/registry must survive an "
            "unrelated command's exception (issue #2100 regression)"
        )

        audio_body = await _drain_until(
            ws, FrameType.JSON_EVENT, body_type="operator_tts_audio"
        )
        assert audio_body.get("request_id") == "req-tars-1"
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass
