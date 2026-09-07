"""Unit-тесты обобщённого audio-канала ws_server (ADR-0055, issue #1993).

Покрывает commit C1 из impl-плана 0055:

* ``deliver_audio(stream="operator_tts", ...)`` шлёт JSON_EVENT
  ``{type:"operator_tts_audio", ...}`` + BINARY_FRAME.
* ``register_audio_session(stream, request_id, ws)`` + ``deliver_audio``
  с явным ws-параметром (для path ``operator_tts``: quest_node сам
  достаёт ws из реестра по сессии).
* Per-stream лимит: заполнить preview до потолка → register для
  operator_tts остаётся ``True`` (стримы не делят слот).
* Неизвестный stream → ``False``, без побочных эффектов.
* ``deliver_preview_audio`` через обёртку — регресс (AV-27 контракт
  НЕ меняется, поведение прежнее).
* Backward-compat: ``server._preview_pending`` всё ещё работает для
  существующих тестов AV-19/AV-27.

Запуск:
    PYTHONPATH=src/rob_box_quest pytest \\
        src/rob_box_quest/test/unit/server/test_ws_server_deliver_audio.py -v
"""

from __future__ import annotations

import asyncio
import json
from typing import Any
from unittest.mock import MagicMock

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.server.ws_server import (
    NoOpBridge,
    WSSServer,
    _AUDIO_STREAMS,
    build_app,
)

# pytestmark только для async-тестов; sync-тесты ниже не помечены
# (pytest-asyncio ругается на маркер у sync-функций).


# ── helpers (как в test_ws_server.py/test_ws_server_voice.py) ──────────


@pytest.fixture
def fixed_pin(monkeypatch):
    pin = "123456"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


async def _open_ws(client):
    return await client.ws_connect("/quest")


async def _send_hello(ws, pin):
    payload = json.dumps(
        {"client_version": "0.1.0", "capabilities": ["webxr"], "session_pin": pin}
    ).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.HELLO, 0, payload))


async def _read_frame(ws, *, type_filter=None, timeout=1.0):
    deadline_loop = asyncio.get_event_loop()
    deadline = deadline_loop.time() + timeout
    while deadline_loop.time() < deadline:
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


async def _wait_for_json_event(ws, predicate, *, timeout=2.0):
    """Ждём первый JSON_EVENT, для которого ``predicate(payload_dict) == True``.

    Проглатываем служебные события (heartbeat, welcome-ack, ping/pong)
    и не выходим по ним. Возвращаем dict payload, или None если timeout.
    """
    deadline_loop = asyncio.get_event_loop()
    deadline = deadline_loop.time() + timeout
    while deadline_loop.time() < deadline:
        try:
            msg = await ws.receive(timeout=0.2)
        except asyncio.TimeoutError:
            continue
        if msg.type == WSMsgType.CLOSE:
            return None
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, payload = decode_frame(msg.data)
            if ftype != FrameType.JSON_EVENT:
                continue
            try:
                body = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if predicate(body):
                return body
    return None


async def _send_ping(ws) -> None:
    """Отправить клиентский ping (JSON_EVENT) — сбрасывает watchdog."""
    # Используем JSON_CMD{cmd:"ping"} — он точно сбрасывает watchdog
    # (см. ws_server._on_json_cmd ветка ``cmd == "ping"``); JSON_EVENT
    # {type:"ping"} тоже работает, но идёт через длинный путь _on_json_event.
    payload = json.dumps({"cmd": "ping", "ts_ms": 0}).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))


async def _keep_alive(ws, *, interval_s: float = 0.2):
    """Background-task: пингуем сервер чаще WATCHDOG_TIMEOUT_S/2 (~0.3 с).

    Без этого TestClient+сервер закрывают соединение по watchdog, и
    доставленные JSON_EVENT'ы теряются (мы их просто не успеваем прочитать).
    Запускается через ``asyncio.create_task`` в начале теста и
    cancel'ится в finally.
    """
    try:
        while True:
            await asyncio.sleep(interval_s)
            try:
                await _send_ping(ws)
            except Exception:
                return
    except asyncio.CancelledError:
        return


async def _authenticate(client, pin):
    ws = await _open_ws(client)
    await _send_hello(ws, pin)
    got = await _read_frame(ws, type_filter=FrameType.WELCOME, timeout=1.0)
    if got is None:
        try:
            await ws.close()
        except Exception:
            pass
        return None, None
    _, payload = got
    body = json.loads(payload.decode("utf-8"))
    return ws, body["session_id"]


# ── тесты обобщённого deliver_audio (ADR-0055) ────────────────────────


@pytest.mark.asyncio
async def test_deliver_audio_operator_tts_sends_binary_frame(fixed_pin):
    """deliver_audio(stream="operator_tts") → JSON_EVENT + BINARY_FRAME клиенту.

    Контракт: meta["type"] == "operator_tts_audio", затем BINARY_FRAME
    с PCM-байтами (НЕ base64 в JSON — это инвариант §7.4 архитектуры).
    """
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws, _ = await _authenticate(client, fixed_pin)
        assert ws is not None
        # Берём именно SERVER-side ws (из реестра сессии), а не клиентский.
        # В TestClient+TestServer aiohttp client_ws.send_bytes пишет в
        # СЕРВЕРНУЮ сторону сокета (а не наоборот); чтобы доставка шла
        # клиенту, ws_server должен звать send_bytes на server_ws.
        # Это та же логика, что в проде: ``deliver_audio`` вызывается
        # из ROS-callback-а и достаёт ws из _ws_by_session на стороне
        # quest_node.
        server_session_id = list(server._sessions.keys())[0]
        server_ws = server._ws_by_session[server_session_id]
        server._send_loop = asyncio.get_event_loop()
        ping_task = asyncio.create_task(_keep_alive(ws))
        try:
            assert server.register_audio_session(
                "operator_tts", "req-operator-1", server_ws
            )
            pcm = b"\x00\x01\x02\x03\x04\x05"  # 3 int16-семпла
            delivered = server.deliver_audio(
                stream="operator_tts",
                request_id="req-operator-1",
                audio_bytes=pcm,
                audio_format="pcm_s16le",
                content_type="audio/pcm",
                seq=0,
                total=0,
            )
            assert delivered is True
            # 1) JSON_EVENT-мета — пропускаем heartbeat и пр.
            body = await _wait_for_json_event(
                ws,
                lambda b: b.get("type") == "operator_tts_audio",
                timeout=2.0,
            )
            assert body is not None, "JSON_EVENT для operator_tts_audio не пришёл"
            assert body["type"] == "operator_tts_audio"
            assert body["request_id"] == "req-operator-1"
            assert body["format"] == "pcm_s16le"
            assert body["content_type"] == "audio/pcm"
            assert body["seq"] == 0
            assert body["total"] == 0
            assert "ts_ms" in body
            # 2) BINARY_FRAME с PCM-байтами.
            binary = await _read_frame(ws, type_filter=FrameType.BINARY_FRAME, timeout=1.0)
            assert binary is not None, "BINARY_FRAME не пришёл"
            # _read_frame уже декодировал фрейм через decode_frame и вернул
            # (ftype, payload); stream_id не нужен (всегда 0 для audio).
            ftype, payload = binary
            assert ftype == FrameType.BINARY_FRAME
            assert payload == pcm
        finally:
            ping_task.cancel()
            try:
                await ping_task
            except (asyncio.CancelledError, Exception):
                pass
            try:
                await ws.close()
            except Exception:
                pass


@pytest.mark.asyncio
async def test_deliver_audio_unknown_stream_is_dropped(fixed_pin):
    """Неизвестный stream → False + WARNING, без побочных эффектов."""
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    # Не заводим сессию — нас интересует именно «не зашёл в реестр».
    delivered = server.deliver_audio(
        stream="bogus",
        request_id="r",
        audio_bytes=b"\x00\x01",
        audio_format="pcm_s16le",
        content_type="audio/pcm",
        seq=0,
        total=0,
    )
    assert delivered is False
    # Реестр для неизвестного stream не должен появиться.
    assert "bogus" not in server._audio_pending
    # А whitelisted-stream'ы инициализированы на старте.
    assert "preview" in server._audio_pending
    assert "operator_tts" in server._audio_pending

@pytest.mark.asyncio

async def test_register_audio_session_too_many(fixed_pin):
    """Per-stream лимит: заполнить operator_tts до VOICE_PREVIEW_MAX_CONCURRENT.

    Реестр — per-stream (ADR-0055 §quest_node): заполненный operator_tts
    НЕ мешает preview.
    """
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    # Подменяем лимит на маленький, чтобы тест был компактным.
    from rob_box_quest.server import ws_server as ws_mod

    original = ws_mod.VOICE_PREVIEW_MAX_CONCURRENT
    ws_mod.VOICE_PREVIEW_MAX_CONCURRENT = 2
    try:
        mock_ws = MagicMock()
        mock_ws.closed = False
        # 2 регистрации — лимит.
        assert server.register_audio_session("operator_tts", "r1", mock_ws)
        assert server.register_audio_session("operator_tts", "r2", mock_ws)
        # 3-я — отказ.
        assert server.register_audio_session("operator_tts", "r3", mock_ws) is False
    finally:
        ws_mod.VOICE_PREVIEW_MAX_CONCURRENT = original
@pytest.mark.asyncio


async def test_deliver_preview_audio_still_works_after_refactor(fixed_pin):
    """Регресс: ``deliver_preview_audio`` через обёртку (AV-27 контракт).

    Сигнатура и meta["type"] == "preview_voice_audio" НЕ меняются
    (тесты AV-19/AV-27 остаются зелёными без правок).
    """
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws, _ = await _authenticate(client, fixed_pin)
        assert ws is not None
        server_session_id = list(server._sessions.keys())[0]
        server_ws = server._ws_by_session[server_session_id]
        server._send_loop = asyncio.get_event_loop()
        ping_task = asyncio.create_task(_keep_alive(ws))
        try:
            assert server.start_preview_session("req-preview-1", server_ws)
            delivered = server.deliver_preview_audio(
                request_id="req-preview-1",
                audio_bytes=b"mp3-bytes",
                audio_format="mp3",
                content_type="audio/mpeg",
                seq=0,
                total=1,
            )
            assert delivered is True
            body = await _wait_for_json_event(
                ws,
                lambda b: b.get("type") == "preview_voice_audio",
                timeout=2.0,
            )
            assert body is not None
            assert body["type"] == "preview_voice_audio"
            assert body["request_id"] == "req-preview-1"
            assert body["format"] == "mp3"
            # BINARY_FRAME.
            binary = await _read_frame(ws, type_filter=FrameType.BINARY_FRAME, timeout=1.0)
            assert binary is not None
        finally:
            ping_task.cancel()
            try:
                await ping_task
            except (asyncio.CancelledError, Exception):
                pass
            try:
                await ws.close()
            except Exception:
                pass


@pytest.mark.asyncio
async def test_deliver_preview_concurrent_limit_isolated_per_stream(fixed_pin):
    """Заполненный preview НЕ мешает operator_tts (стримы не делят слот)."""
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    from rob_box_quest.server import ws_server as ws_mod

    original = ws_mod.VOICE_PREVIEW_MAX_CONCURRENT
    ws_mod.VOICE_PREVIEW_MAX_CONCURRENT = 1
    try:
        mock_ws = MagicMock()
        mock_ws.closed = False
        # 1 preview — лимит.
        assert server.register_audio_session("preview", "p1", mock_ws)
        assert server.register_audio_session("preview", "p2", mock_ws) is False
        # operator_tts — ОК (изолированный слот; в нём пусто).
        assert server.register_audio_session("operator_tts", "o1", mock_ws) is True
        # operator_tts теперь заполнен (1 == лимит).
        assert server.register_audio_session("operator_tts", "o2", mock_ws) is False
        # И наоборот: освобождаем preview (deliver_done) — снова ОК.
        assert server._send_audio_done(
            "preview", "preview_voice_done", "p1"
        ) is True
        # Теперь preview свободен → новая регистрация ОК.
        assert server.register_audio_session("preview", "p3", mock_ws) is True
    finally:
        ws_mod.VOICE_PREVIEW_MAX_CONCURRENT = original


def test_audio_streams_whitelist_is_static():
    """``_AUDIO_STREAMS`` — frozenset {"preview", "operator_tts"}.

    Guard против «широкого шва»: добавить stream можно только явной
    правкой константы + ADR + тестов. Это намеренно.
    """
    assert _AUDIO_STREAMS == frozenset({"preview", "operator_tts"})
    assert isinstance(_AUDIO_STREAMS, frozenset)


def test_register_audio_session_rejects_unknown_stream(fixed_pin):
    """register_audio_session(stream="bogus", ...) → False (no side effects)."""
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    mock_ws = MagicMock()
    mock_ws.closed = False
    assert server.register_audio_session("bogus", "r1", mock_ws) is False
    # Реестр не появился.
    assert "bogus" not in server._audio_pending


@pytest.mark.asyncio
async def test_deliver_audio_uses_explicit_ws_param_when_provided(fixed_pin):
    """deliver_audio(ws=...) берёт переданный ws, не из реестра.

    Это path для ADR-0055 §quest_node: ``register_audio_session`` хранит
    request_id → ws (сессионная привязка делается через side-channel
    ``/avatar/tts/request`` подписку в quest_node). После — ROS-callback
    ``_on_avatar_tts_audio`` зовёт ``deliver_audio(stream="operator_tts",
    ws=self._current_avatar_ws, ...)`` с явным ws.

    NB: явный ``ws`` — это **server-side** ``WebSocketResponse`` (тот же,
    что в ``_ws_by_session``), а не client-side ``ws``. Сервер может
    писать только в server-side сокет; client-side — для receive.
    """
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        ws, _ = await _authenticate(client, fixed_pin)
        assert ws is not None
        server_session_id = list(server._sessions.keys())[0]
        server_ws = server._ws_by_session[server_session_id]
        server._send_loop = asyncio.get_event_loop()
        ping_task = asyncio.create_task(_keep_alive(ws))
        try:
            assert server.register_audio_session(
                "operator_tts", "req-explicit-ws", server_ws
            )
            # Подменим запись в реестре на «не тот ws» (например, закрытый),
            # чтобы проверить, что deliver_audio берёт **явный** ws, не из реестра.
            mock_garbage_ws = MagicMock()
            mock_garbage_ws.closed = True
            with server._voice_state_lock:
                server._audio_pending["operator_tts"]["req-explicit-ws"] = (
                    mock_garbage_ws,
                    0.0,
                )
            # Вызов с явным ws=server_ws (живым, server-side) должен сработать
            # даже при closed-моке в реестре.
            delivered = server.deliver_audio(
                stream="operator_tts",
                request_id="req-explicit-ws",
                audio_bytes=b"\x10\x20",
                audio_format="pcm_s16le",
                content_type="audio/pcm",
                seq=0,
                total=0,
                ws=server_ws,
            )
            assert delivered is True
            body = await _wait_for_json_event(
                ws,
                lambda b: b.get("type") == "operator_tts_audio",
                timeout=2.0,
            )
            assert body is not None
        finally:
            ping_task.cancel()
            try:
                await ping_task
            except (asyncio.CancelledError, Exception):
                pass
            try:
                await ws.close()
            except Exception:
                pass


def test_backward_compat_preview_pending_property(fixed_pin):
    """``server._preview_pending`` — back-compat alias на ``_audio_pending['preview']``.

    Существующие тесты AV-19/AV-27 читают ``server._preview_pending``
    напрямую. Удалить после их миграции (коммит-фикс).
    """
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    # На старте — пустой dict для preview.
    assert server._preview_pending == {}
    mock_ws = MagicMock()
    mock_ws.closed = False
    # Зарегистрировали через register_audio_session — preview-алиас видит.
    server.register_audio_session("preview", "p1", mock_ws)
    assert "p1" in server._preview_pending
    # А оператор-стрим НЕ виден через preview-алиас.
    server.register_audio_session("operator_tts", "o1", mock_ws)
    assert "o1" not in server._preview_pending
    assert "o1" in server._audio_pending["operator_tts"]