"""voice-vr 17 (t_af606d97) — unit-тесты диспетчеризации supervisor-API.

Покрывает карточку #2202 [voice-vr 17] — декомпозицию
``WSSServer._handle_supervisor_command`` (CC=24 → 3):

* ``_handle_supervisor_command`` с FrameType, отсутствующим в
  ``SUPERVISOR_HANDLERS``, но валидным на уровне msgpack → ERROR
  с явным указанием ftype (а не молчаливый drop).
* v1-сессия → ``ERROR{PROTOCOL_VERSION}`` (regression-acceptance
  из карточки #1908 / DoD voice-vr 17).
* Битый msgpack → ERROR{BAD_PAYLOAD}, bridge-метод НЕ вызывается
  (pre-guard msgpack unpack).

Эти тесты — прямые unit-вызовы ``_handle_supervisor_command``,
минуя aiohttp WS-handshake. Через реальный WS попасть в default-ветку
диспетчера нельзя — ``_ws_handler:2582-2591`` маршрутизирует только
``SET_MODE / ACQUIRE_FLOOR / RELEASE_FLOOR`` (все зарегистрированы).
Default-ветка сработает, если разработчик расширит ``SUPERVISOR_FRAME_TYPES``
в module-level, но забудет добавить handler — этот тест ловит такую
ошибку на CI.
"""

from __future__ import annotations

import json
from typing import Any

import msgpack
import pytest

from rob_box_quest.protocol.frame import FrameType, decode_frame
from rob_box_quest.server.session import (
    SessionState,
    ClientSession,
    server_client_id,
)
from rob_box_quest.server.ws_server import (
    NoOpBridge,
    SUPERVISOR_HANDLERS,
    SUPERVISOR_FRAME_TYPES,
    WSSServer,
    _handle_acquire_floor,
    _handle_release_floor,
    _handle_set_mode,
)


pytestmark = pytest.mark.asyncio


class _RecordingBridge(NoOpBridge):
    """Мини-bridge для direct-вызовов _handle_supervisor_command.

    Наследует NoOpBridge, чтобы соответствовать Bridge-протоколу, и
    переопределяет supervisor-методы с записью вызовов (для ассертов
    «pre-guard отбил → bridge не вызывался»).
    """

    def __init__(self) -> None:
        super().__init__()
        self.acquire_calls: list = []
        self.release_calls: list = []
        self.set_mode_calls: list = []
        self._state: dict = {"mode": "off", "version": 1}

    def supervisor_acquire_floor(self, client_id, floor):
        self.acquire_calls.append((client_id, floor))
        return {"granted": True, "applied": True, "reason": "ok"}

    def supervisor_release_floor(self, client_id, floor):
        self.release_calls.append((client_id, floor))
        return {"applied": True, "reason": "ok"}

    def supervisor_set_mode(self, client_id, mode):
        self.set_mode_calls.append((client_id, mode))
        return {"applied": True, "reason": "ok", "actual_mode": mode}

    def supervisor_state(self):
        return self._state


class _FakeWS:
    """Записывает все отправленные binary-фреймы, как реальный ws-объект.

    Метод ``send_bytes`` — корутина (как у aiohttp WebSocketResponse).
    """

    def __init__(self) -> None:
        self.frames: list[bytes] = []

    async def send_bytes(self, data: bytes) -> None:
        self.frames.append(data)

    async def close(self) -> None:  # pragma: no cover
        pass


def _make_session(protocol_version: int = 2) -> ClientSession:
    """Свежая аутентифицированная сессия с заполненным server_client_id."""
    session_id = "test-session-001"
    return ClientSession(
        session_id=session_id,
        state=SessionState.AUTHENTICATED,
        client_version="0.1.0",
        capabilities=["webxr"],
        protocol_version=protocol_version,
        subscribed={},
        last_ping_monotonic=0.0,
        last_heartbeat_monotonic=None,
        created_monotonic=0.0,
        server_client_id=server_client_id(session_id),
    )


def _decode_error(ws: _FakeWS) -> dict:
    """Достать и распарсить ERROR-фрейм из ws.frames (последний)."""
    assert ws.frames, "ws.frames is empty — no ERROR was sent"
    ftype, _sid, payload = decode_frame(ws.frames[-1])
    assert ftype == FrameType.ERROR, f"expected ERROR, got {ftype.name}"
    return json.loads(payload.decode("utf-8"))


# === Tests =================================================================


async def test_supervisor_handlers_table_covers_all_supervisor_frame_types():
    """Защита от «добавил ftype в SUPERVISOR_FRAME_TYPES — забыл handler»."""
    missing = SUPERVISOR_FRAME_TYPES - set(SUPERVISOR_HANDLERS.keys())
    assert not missing, (
        "SUPERVISOR_FRAME_TYPES содержит ftype без handler-а в "
        f"SUPERVISOR_HANDLERS: {missing}"
    )


async def test_unknown_supervisor_frame_returns_bad_payload_error():
    """FrameType, не зарегистрированный в SUPERVISOR_HANDLERS, но с
    валидным msgpack → ERROR{BAD_PAYLOAD} с явным именем ftype.

    Имитирует сценарий «разработчик добавил ftype в SUPERVISOR_FRAME_TYPES
    и _ws_handler диспетчер, но забыл прописать handler». Диспетчер
    supervisor-API не должен молча проглатывать такие фреймы.
    """
    bridge = _RecordingBridge()
    server = WSSServer(bridge=bridge, pin="000000")
    session = _make_session()
    ws = _FakeWS()

    # Используем FrameType.JSON_CMD (0x10) — гарантированно не в
    # SUPERVISOR_HANDLERS, но supervisor_pre_guard (msgpack unpack +
    # version + auth) пропустит.
    unknown_ftype = FrameType.JSON_CMD
    assert unknown_ftype not in SUPERVISOR_HANDLERS

    payload = msgpack.packb({"some": "data"}, use_bin_type=True)
    await server._handle_supervisor_command(ws, session, unknown_ftype, payload)

    err = _decode_error(ws)
    assert err["code"] == "BAD_PAYLOAD"
    # В message должно быть имя ftype (для диагностики клиента)
    # + hex-код для логов.
    assert unknown_ftype.name in err["message"], err
    assert f"0x{unknown_ftype.value:02x}" in err["message"], err


async def test_v1_session_gets_protocol_version_on_unregistered_frame():
    """v1-сессия → ERROR{PROTOCOL_VERSION}.

    Карточка #1908 acceptance: даже если frame-type не зарегистрирован,
    pre-guard protocol_version срабатывает раньше dispatch.
    """
    bridge = _RecordingBridge()
    server = WSSServer(bridge=bridge, pin="000000")
    session = _make_session(protocol_version=1)
    ws = _FakeWS()

    payload = msgpack.packb({"floor": "teleop"}, use_bin_type=True)
    await server._handle_supervisor_command(
        ws, session, FrameType.ACQUIRE_FLOOR, payload
    )

    err = _decode_error(ws)
    assert err["code"] == "PROTOCOL_VERSION"
    assert "v2" in err["message"]


async def test_msgpack_unpack_failure_returns_bad_payload_error():
    """Битый msgpack → ERROR{BAD_PAYLOAD} (msgpack unpack guard)."""
    bridge = _RecordingBridge()
    server = WSSServer(bridge=bridge, pin="000000")
    session = _make_session()
    ws = _FakeWS()

    # Просто невалидный msgpack-байтовый поток.
    await server._handle_supervisor_command(
        ws, session, FrameType.SET_MODE, b"\xc1\x00not-msgpack"
    )

    err = _decode_error(ws)
    assert err["code"] == "BAD_PAYLOAD"
    # Bridge не должен был быть вызван — pre-guard отбил.
    assert bridge.set_mode_calls == []


async def test_acquire_floor_dispatches_to_registered_handler():
    """SET_MODE / ACQUIRE_FLOOR / RELEASE_FLOOR handler'ы зарегистрированы.

    Smoke-test: dispatcher не подменяет handler-ы; handler-ы — module-level
    async-функции (НЕ bound-методы WSSServer), потому что dict инициализируется
    в module-scope до создания WSSServer instance.
    """
    import inspect
    import types

    for ftype, expected in (
        (FrameType.SET_MODE, _handle_set_mode),
        (FrameType.ACQUIRE_FLOOR, _handle_acquire_floor),
        (FrameType.RELEASE_FLOOR, _handle_release_floor),
    ):
        handler = SUPERVISOR_HANDLERS[ftype]
        # Module-level async function (не bound-method): types.CoroutineType
        # при вызове, но сам callable — types.FunctionType.
        assert isinstance(handler, types.FunctionType), (
            f"handler for {ftype.name} is not a plain function: {type(handler)}"
        )
        assert handler is expected, (
            f"handler for {ftype.name} does not point to module-level "
            f"function {expected.__name__}"
        )
        # async функция: inspect.iscoroutinefunction вернёт True
        assert inspect.iscoroutinefunction(handler), (
            f"handler for {ftype.name} is not async"
        )
