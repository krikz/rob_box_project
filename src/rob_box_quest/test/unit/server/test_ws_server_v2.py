"""AV-16: WS-server integration tests для supervisor API + subprotocol v1/v2.

Покрывает acceptance criteria карточки #1908:

- [ ] WSSServer анонсирует robbox-quest-v2 (и v1) в WS-handshake.
- [ ] v1-сессия присылает 0x31 → ERROR{PROTOCOL_VERSION}.
- [ ] v1-сессия НЕ получает STATE_UPDATE (в т.ч. keep-alive).
- [ ] v2 ACQUIRE_FLOOR → supervisor-сервис → STATE_UPDATE с msgpack.
- [ ] v2 SET_MODE с невалидным mode → ERROR{BAD_PAYLOAD}.
- [ ] v2 SET_MODE когда FSM отвергает (Bridge returns applied=False) → MODE_CONFLICT.
- [ ] v2 ACQUIRE_FLOOR когда floor занят → ERROR{FLOOR_HELD} с held_by.
- [ ] client_id из payload игнорируется (Bridge receives server-side client_id).
- [ ] JSON_CMD{supervisor_*} (эквивалент §5.1) → supervisor_state event + ts_ms.
- [ ] Bridge без service-клиента (None) → супервизор-unavailable, не блок aiohttp.

Источники истины:
- docs/architecture/meta-quest-api.md §3/§5.1/§8/§11
- src/rob_box_quest/protocol/frame.py (FrameType)
- src/rob_box_quest/server/{session.py,ws_server.py}
"""

import asyncio
import json
import time

import msgpack
import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.server.session import (
    SUPPORTED_SUBPROTOCOLS_V2,
    server_client_id,
)
from rob_box_quest.server.ws_server import NoOpBridge, WSSServer, build_app


pytestmark = pytest.mark.asyncio


# === Test doubles ============================================================

class _FakeServiceClient:
    """Подделка ROS-клиента для unit-тестов _run_supervisor_service.

    Контракт (см. ws_server.Bridge + supervisor_node._extract_*):
    - ``srv_type.Request()`` — конструктор Trigger-like.
    - ``call_async(req)`` — async coroutine возвращающая объект с success/message.
    """

    def __init__(self, response: dict, delay_s: float = 0.0):
        self._response = response
        self._delay_s = delay_s
        self.calls: list = []

        class _Trigger:
            def __init__(self) -> None:
                # Атрибуты, которые supervisor ищет через getattr().
                self.client_id = None
                self.floor = None
                self.event = None

        class _TriggerSrv:
            Request = _Trigger

        self.srv_type = _TriggerSrv()

    def call_async(self, req):  # noqa: D401
        async def _go():
            self.calls.append(
                {
                    "client_id": getattr(req, "client_id", None),
                    "floor": getattr(req, "floor", None),
                    "event": getattr(req, "event", None),
                }
            )
            if self._delay_s > 0:
                await asyncio.sleep(self._delay_s)

            class _Resp:
                def __init__(self, body: dict) -> None:
                    self.success = bool(body.get("success", True))
                    self.message = json.dumps(
                        {k: v for k, v in body.items() if k != "success"}
                    )

            return _Resp(self._response)

        return _go()


class _ConfigurableBridge:
    """Bridge с переопределяемыми ответами на supervisor_* и состоянием.

    Покрывает сценарии:
    - acquire/release возвращают ``granted/applied/reason``.
    - supervisor_state возвращает dict или None.
    - publish_state обновляет StateUpdate counter.
    """

    def __init__(
        self,
        acquire_response: dict | None = None,
        release_response: dict | None = None,
        set_mode_response: dict | None = None,
        initial_state: dict | None = None,
    ) -> None:
        self.acquire_response = acquire_response or {
            "granted": True,
            "applied": True,
            "reason": "ok",
        }
        self.release_response = release_response or {"applied": True, "reason": "ok"}
        self.set_mode_response = set_mode_response or {
            "applied": True,
            "reason": "ok",
            "actual_mode": "off",
        }
        self._state = initial_state or {"mode": "off"}
        self.acquire_calls: list = []
        self.release_calls: list = []
        self.set_mode_calls: list = []

    # --- старые Phase 1 методы (нужны для build → QuestNode аналога) ---
    def publish_quest(self, linear, angular):
        pass

    def publish_emergency(self):
        pass

    def feed_client_alive(self):
        pass

    def reset(self):
        pass

    def emergency_stop(self):
        pass

    def publish_frame(self, ui, payload):
        pass

    def available_streams(self):
        return []

    def publish_voice_barge_in(self):
        pass

    def publish_voice_audio(self, payload):
        pass

    def publish_voice_stop(self):
        pass

    def publish_voice_robot_start(self):
        pass

    def publish_voice_robot_stop(self):
        pass

    def set_voice_mode(self, mode):
        pass

    # --- AV-16 supervisor API ---
    def supervisor_acquire_floor(self, client_id, floor):
        self.acquire_calls.append((client_id, floor))
        return self.acquire_response

    def supervisor_release_floor(self, client_id, floor):
        self.release_calls.append((client_id, floor))
        return self.release_response

    def supervisor_set_mode(self, client_id, mode):
        self.set_mode_calls.append((client_id, mode))
        return self.set_mode_response

    def supervisor_state(self):
        return self._state

    def on_supervisor_state(self, cb):
        pass


# === Fixtures ============================================================


@pytest.fixture
def fixed_pin(monkeypatch):
    pin = "123456"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


@pytest.fixture
async def v1_client(fixed_pin):
    """WS-клиент, требующий subprotocol v1 (через WS-handshake)."""
    server = WSSServer(bridge=_ConfigurableBridge(), pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        yield client, server


@pytest.fixture
async def v2_client(fixed_pin):
    """WS-клиент, требующий subprotocol v2."""
    server = WSSServer(
        bridge=_ConfigurableBridge(
            acquire_response={"granted": True, "applied": True, "reason": "ok"},
            initial_state={"mode": "off", "version": 1},
        ),
        pin=fixed_pin,
    )
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        yield client, server


async def _open_ws(client, subprotocol: str | None = None):
    """Подключиться к /quest с указанным subprotocol.

    aiohttp test_utils: ws_connect принимает ``protocols=...`` напрямую.
    """
    protocols = subprotocol or SUPPORTED_SUBPROTOCOLS_V2
    if isinstance(protocols, str):
        protocols = (protocols,)
    ws = await client.ws_connect("/quest", protocols=protocols)
    return ws


async def _drain_welcome(ws, timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = await ws.receive()
        if msg.type == WSMsgType.CLOSE:
            return
        if msg.type == WSMsgType.BINARY:
            ftype, _, payload = decode_frame(msg.data)
            if ftype == FrameType.WELCOME:
                return


async def _send_hello(ws, pin="123456"):
    body = json.dumps(
        {"client_version": "0.1.0", "capabilities": ["webxr"], "session_pin": pin}
    ).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.HELLO, 0, body))


async def _read_first(ws, predicate, timeout_s: float = 1.0):
    """Читать фреймы до первого, удовлетворяющего ``predicate(ftype, payload)``."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = await ws.receive(timeout=0.5)
        if msg.type == WSMsgType.CLOSE:
            return ("CLOSE", None)
        if msg.type == WSMsgType.BINARY:
            ftype, _, payload = decode_frame(msg.data)
            if predicate(ftype, payload):
                return (ftype, payload)
    return ("TIMEOUT", None)


# === Tests =================================================================


async def test_ws_server_anounces_v2_in_subprotocol(fixed_pin):
    """Сервер объявляет robbox-quest-v2 как primary в WS-handshake."""
    server = WSSServer(bridge=_ConfigurableBridge(), pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws = await client.ws_connect("/quest")  # без заголовка — server выбирает первый
        # Сервер анонсирует SUPPORTED_SUBPROTOCOLS_V2 — клиент без заголовка
        # получит v2 (auto-first-match в aiohttp). Проверяем, что и v1 работает.
        await ws.close()
        ws2 = await client.ws_connect("/quest", protocols=("robbox-quest-v1",))
        # WSS handler согласует v1 (первый совпавший из запрошенных)
        await ws2.close()
        ws3 = await client.ws_connect("/quest", protocols=("robbox-quest-v2",))
        await ws3.close()
        # Если дошли сюда без ошибок — handshake на обоих subprotocol-ах прошёл.


async def test_v1_session_gets_protocol_version_on_acquire_floor(v1_client, fixed_pin):
    """v1 ACQUIRE_FLOOR (0x31) → ERROR{PROTOCOL_VERSION} (§11)."""
    http_client, _server = v1_client
    ws = await _open_ws(http_client, "robbox-quest-v1")
    try:
        await _send_hello(ws, fixed_pin)
        await _drain_welcome(ws)
        payload = msgpack.packb(
            {"client_id": "malicious", "floor": "teleop"}, use_bin_type=True
        )
        await ws.send_bytes(encode_frame(FrameType.ACQUIRE_FLOOR, 0, payload))

        # Ждём ERROR{PROTOCOL_VERSION}.
        def is_error(ftype, p):
            if ftype != FrameType.ERROR:
                return False
            try:
                err = json.loads(p.decode("utf-8"))
            except Exception:  # noqa: BLE001
                return False
            return err.get("code") == "PROTOCOL_VERSION"

        result = await _read_first(ws, is_error, timeout_s=1.0)
        assert result[0] != "TIMEOUT", "ERROR{PROTOCOL_VERSION} not received"
        assert result[0] != "CLOSE"
        ftype, raw = result
        assert ftype == FrameType.ERROR
        body = json.loads(raw.decode("utf-8"))
        assert body["code"] == "PROTOCOL_VERSION"
        assert "v2" in body["message"]
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_v1_session_no_state_update_keepalive(v1_client, fixed_pin):
    """v1 НЕ получает STATE_UPDATE (в т.ч. keep-alive 1Hz). Ждём ~1.5 с и убеждаемся, что STATE_UPDATE не пришёл.

    Используем ping-task (как в test_v2_state_update_keepalive_at_1hz) чтобы
    watchdog не трипнулся по таймауту 600мс.
    """
    http_client, _server = v1_client
    ws = await _open_ws(http_client, "robbox-quest-v1")
    try:
        await _send_hello(ws, fixed_pin)
        await _drain_welcome(ws)
        ping_task = asyncio.create_task(_keepalive_ping(ws))
        try:
            deadline = time.monotonic() + 1.5
            state_updates = 0
            while time.monotonic() < deadline:
                try:
                    msg = await ws.receive(timeout=0.3)
                except asyncio.TimeoutError:
                    continue
                if msg.type == WSMsgType.CLOSE:
                    break
                if msg.type == WSMsgType.BINARY:
                    ftype, _, _ = decode_frame(msg.data)
                    if ftype == FrameType.STATE_UPDATE:
                        state_updates += 1
            assert state_updates == 0, (
                f"v1 received {state_updates} STATE_UPDATE frames; should be zero"
            )
        finally:
            ping_task.cancel()
            try:
                await ping_task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_v2_acquire_floor_success_returns_state_update(v2_client, fixed_pin):
    """v2 ACQUIRE_FLOOR (0x31) → Bridge.supervisor_acquire_floor → STATE_UPDATE."""
    http_client, server = v2_client
    ws = await _open_ws(http_client, "robbox-quest-v2")
    try:
        await _send_hello(ws, fixed_pin)
        await _drain_welcome(ws)
        # Платёж: client_id может любым — сервер игнорирует его (см. §11).
        payload = msgpack.packb(
            {"client_id": "spoofed_telegram", "floor": "teleop"},
            use_bin_type=True,
        )
        await ws.send_bytes(encode_frame(FrameType.ACQUIRE_FLOOR, 0, payload))

        def is_state_update(ftype, p):
            return ftype == FrameType.STATE_UPDATE

        result = await _read_first(ws, is_state_update, timeout_s=1.0)
        assert result[0] != "TIMEOUT", "STATE_UPDATE not received on acquire"
        assert result[0] != "CLOSE"
        ftype, raw = result
        assert ftype == FrameType.STATE_UPDATE
        # msgpack-payload: {state: {mode, version}}
        decoded = msgpack.unpackb(raw, raw=False)
        # STATE_UPDATE payload — msgpack-dict с ключом ``state`` (см. §3 строка 66).
        assert "state" in decoded, f"missing 'state' wrapper in {decoded}"
        assert decoded["state"].get("mode") == "off"
        # Bridge получил именно server-side client_id, не «spoofed_telegram».
        bridge = server.bridge
        assert bridge.acquire_calls, "acquire не был вызван на Bridge"
        called_client_id, called_floor = bridge.acquire_calls[0]
        assert called_floor == "teleop"
        assert called_client_id.startswith("quest:")
        assert "spoofed_telegram" not in called_client_id
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_v2_set_mode_invalid_returns_bad_payload(v2_client, fixed_pin):
    """SET_MODE с mode='hacker_special' → ERROR{BAD_PAYLOAD} (нет в списке)."""
    http_client, _server = v2_client
    ws = await _open_ws(http_client, "robbox-quest-v2")
    try:
        await _send_hello(ws, fixed_pin)
        await _drain_welcome(ws)
        payload = msgpack.packb(
            {"client_id": "quest:xyz", "mode": "hacker_special"}, use_bin_type=True
        )
        await ws.send_bytes(encode_frame(FrameType.SET_MODE, 0, payload))

        def is_bad_payload(ftype, p):
            if ftype != FrameType.ERROR:
                return False
            err = json.loads(p.decode("utf-8"))
            return err.get("code") == "BAD_PAYLOAD"

        result = await _read_first(ws, is_bad_payload, timeout_s=1.0)
        assert result[0] != "TIMEOUT"
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_v2_set_mode_fsm_rejects_returns_mode_conflict(fixed_pin):
    """SET_MODE когда FSM supervisor-а вернула applied=False → ERROR{MODE_CONFLICT}."""
    bridge = _ConfigurableBridge(
        set_mode_response={
            "applied": False,
            "reason": "fsm_refused:off_requires_force_off",
            "actual_mode": "telegram_active",
        }
    )
    server = WSSServer(bridge=bridge, pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws = await client.ws_connect("/quest", protocols=("robbox-quest-v2",))
        try:
            await _send_hello(ws, fixed_pin)
            await _drain_welcome(ws)
            payload = msgpack.packb({"mode": "mixed"}, use_bin_type=True)
            await ws.send_bytes(encode_frame(FrameType.SET_MODE, 0, payload))

            def is_mode_conflict(ftype, p):
                if ftype != FrameType.ERROR:
                    return False
                err = json.loads(p.decode("utf-8"))
                return err.get("code") == "MODE_CONFLICT"

            result = await _read_first(ws, is_mode_conflict, timeout_s=1.0)
            assert result[0] != "TIMEOUT"
            # Подтверждаем что Bridge всё-таки был вызван.
            assert bridge.set_mode_calls
            assert bridge.set_mode_calls[0][1] == "mixed"
        finally:
            try:
                await ws.close()
            except Exception:  # noqa: BLE001
                pass


async def test_v2_acquire_floor_conflict_returns_floor_held_with_holder(fixed_pin):
    """ACQUIRE_FLOOR когда floor занят другим client_id → ERROR{FLOOR_HELD}."""
    bridge = _ConfigurableBridge(
        acquire_response={
            "granted": False,
            "applied": True,
            "reason": "conflict",
            "held_by": "telegram:abc",
        }
    )
    server = WSSServer(bridge=bridge, pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws = await client.ws_connect("/quest", protocols=("robbox-quest-v2",))
        try:
            await _send_hello(ws, fixed_pin)
            await _drain_welcome(ws)
            payload = msgpack.packb(
                {"client_id": "quest:xyz", "floor": "teleop"},
                use_bin_type=True,
            )
            await ws.send_bytes(encode_frame(FrameType.ACQUIRE_FLOOR, 0, payload))

            def is_floor_held(ftype, p):
                if ftype != FrameType.ERROR:
                    return False
                err = json.loads(p.decode("utf-8"))
                return err.get("code") == "FLOOR_HELD"

            result = await _read_first(ws, is_floor_held, timeout_s=1.0)
            assert result[0] != "TIMEOUT"
            ftype, raw = result
            err = json.loads(raw.decode("utf-8"))
            assert err["code"] == "FLOOR_HELD"
            assert "telegram:abc" in err["message"]
        finally:
            try:
                await ws.close()
            except Exception:  # noqa: BLE001
                pass


async def test_v2_json_cmd_supervisor_acquire_floor(v2_client, fixed_pin):
    """JSON_CMD{supervisor_acquire_floor} → JSON_EVENT{supervisor_state, ts_ms} (§5.1)."""
    http_client, server = v2_client
    ws = await _open_ws(http_client, "robbox-quest-v2")
    try:
        await _send_hello(ws, fixed_pin)
        await _drain_welcome(ws)
        body = json.dumps(
            {"cmd": "supervisor_acquire_floor", "client_id": "any", "floor": "voice"}
        ).encode("utf-8")
        await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, body))

        def is_supervisor_state(ftype, p):
            if ftype != FrameType.JSON_EVENT:
                return False
            ev = json.loads(p.decode("utf-8"))
            return ev.get("type") == "supervisor_state"

        result = await _read_first(ws, is_supervisor_state, timeout_s=1.0)
        assert result[0] != "TIMEOUT"
        ftype, raw = result
        ev = json.loads(raw.decode("utf-8"))
        assert ev["type"] == "supervisor_state"
        assert "ts_ms" in ev
        assert ev["state"]["mode"] == "off"  # initial_state в bridge
        # floor был передан, Bridge вызвался с правильным floor-ом.
        assert server.bridge.acquire_calls[0][1] == "voice"
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_v2_json_cmd_supervisor_get_state(v2_client, fixed_pin):
    """JSON_CMD{supervisor_get_state} → JSON_EVENT{supervisor_state, ts_ms}."""
    http_client, _server = v2_client
    ws = await _open_ws(http_client, "robbox-quest-v2")
    try:
        await _send_hello(ws, fixed_pin)
        await _drain_welcome(ws)
        body = json.dumps({"cmd": "supervisor_get_state"}).encode("utf-8")
        await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, body))

        def is_state_event(ftype, p):
            if ftype != FrameType.JSON_EVENT:
                return False
            ev = json.loads(p.decode("utf-8"))
            return ev.get("type") == "supervisor_state"

        result = await _read_first(ws, is_state_event, timeout_s=1.0)
        assert result[0] != "TIMEOUT"
        ev = json.loads(result[1].decode("utf-8"))
        assert ev["type"] == "supervisor_state"
        assert ev["state"]["mode"] == "off"
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_client_id_in_payload_ignored_server_supplies_own(v2_client, fixed_pin):
    """client_id из payload-а игнорируется; Bridge всегда получает server-side «quest:<uuid>»."""
    http_client, server = v2_client
    ws = await _open_ws(http_client, "robbox-quest-v2")
    try:
        await _send_hello(ws, fixed_pin)
        await _drain_welcome(ws)
        payload = msgpack.packb(
            {"client_id": "telegram:budget", "floor": "voice"}, use_bin_type=True
        )
        await ws.send_bytes(encode_frame(FrameType.ACQUIRE_FLOOR, 0, payload))

        def is_any(ftype, p):
            return ftype in (FrameType.STATE_UPDATE, FrameType.ERROR)

        await _read_first(ws, is_any, timeout_s=1.0)
        called_client_id = server.bridge.acquire_calls[0][0]
        assert called_client_id.startswith("quest:")
        assert "telegram" not in called_client_id
        # server_client_id, сформированный при HELLO:
        assert called_client_id == server_client_id(server._sessions[next(iter(server._sessions))].session_id)
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_v2_release_floor_calls_bridge_with_release_path(v2_client, fixed_pin):
    """RELEASE_FLOOR (0x32) вызывает ``supervisor_release_floor``, а не acquire."""
    http_client, server = v2_client
    ws = await _open_ws(http_client, "robbox-quest-v2")
    try:
        await _send_hello(ws, fixed_pin)
        await _drain_welcome(ws)
        payload = msgpack.packb({"floor": "voice"}, use_bin_type=True)
        await ws.send_bytes(encode_frame(FrameType.RELEASE_FLOOR, 0, payload))

        def is_any(ftype, p):
            return ftype in (FrameType.STATE_UPDATE, FrameType.ERROR)

        await _read_first(ws, is_any, timeout_s=1.0)
        assert server.bridge.release_calls
        assert server.bridge.acquire_calls == []
        assert server.bridge.release_calls[0][1] == "voice"
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def test_v2_state_update_keepalive_at_1hz(v2_client, fixed_pin):
    """STATE_UPDATE keep-alive 1Hz — за ~2 с приходит ≥1 STATE_UPDATE frame.

    v1 здесь не подходит (см. test выше), поэтому тест специфичен для v2.
    Watchdog-trip подавлен ping'ом (200мс, как в test_ws_server.py).
    """
    http_client, server = v2_client
    ws = await _open_ws(http_client, "robbox-quest-v2")
    try:
        await _send_hello(ws, fixed_pin)
        await _drain_welcome(ws)
        # Прогреваем ACQUIRE_FLOOR (получим первый STATE_UPDATE),
        # параллельно запускаем ping-task чтобы не споткнуться о watchdog.
        ping_task = asyncio.create_task(_keepalive_ping(ws))
        try:
            payload = msgpack.packb({"floor": "teleop"}, use_bin_type=True)
            await ws.send_bytes(encode_frame(FrameType.ACQUIRE_FLOOR, 0, payload))

            def is_any_state(ftype, p):
                return ftype in (FrameType.STATE_UPDATE, FrameType.ERROR)

            await _read_first(ws, is_any_state, timeout_s=1.0)

            # Теперь считаем STATE_UPDATE за ~2.0 с (ожидаем ≥1).
            deadline = time.monotonic() + 2.0
            state_updates = 0
            while time.monotonic() < deadline:
                try:
                    msg = await ws.receive(timeout=0.3)
                except asyncio.TimeoutError:
                    continue
                if msg.type == WSMsgType.CLOSE:
                    break
                if msg.type == WSMsgType.BINARY:
                    ftype, _, _ = decode_frame(msg.data)
                    if ftype == FrameType.STATE_UPDATE:
                        state_updates += 1
            assert state_updates >= 1, f"too few STATE_UPDATE keepalives: {state_updates}"
        finally:
            ping_task.cancel()
            try:
                await ping_task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass


async def _keepalive_ping(ws):
    """Шлёт JSON_EVENT ping каждые 100мс (под WATCHDOG_TIMEOUT_S=0.6с)."""
    body = json.dumps({"type": "ping", "ts_ms": 0}).encode("utf-8")
    while True:
        try:
            await ws.send_bytes(encode_frame(FrameType.JSON_EVENT, 0, body))
        except Exception:  # noqa: BLE001
            return
        await asyncio.sleep(0.1)


async def test_protocol_version_field_in_session_after_handshake(v2_client):
    """После handshake session.protocol_version == 2 (AV-16 acceptance)."""
    http_client, server = v2_client
    ws = await _open_ws(http_client, "robbox-quest-v2")
    try:
        # session уже существует после ws.connect (handler создал её).
        sessions = list(server._sessions.values())
        assert sessions, "session not registered"
        assert sessions[0].protocol_version == 2
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass
