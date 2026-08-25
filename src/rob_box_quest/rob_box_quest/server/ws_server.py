"""aiohttp WS handler + app builder for rob_box_quest.

Phase 1.2: чистая server-логика без ROS-ноды.
- HELLO с PIN → WELCOME / ERROR{AUTH_FAIL}
- SUBSCRIBE → JSON_EVENT{subscribe_ack} c выделенным stream_id
- heartbeat каждые 200 мс (JSON_EVENT{type:"heartbeat"})
- watchdog: нет ping > 600 мс → close + GOODBYE{reason:"timeout"}
- /healthz endpoint (как у voice-action-server)

ROS/Zenoh-мост подключается в Phase 1.3 через DI: WSSServer.__init__
принимает интерфейс `Bridge` (publish/subscribe методы) — это позволяет
тестировать handler в изоляции.
"""

from __future__ import annotations

import asyncio
import json
import logging
import secrets
import time
from typing import Any, Awaitable, Callable, Optional, Protocol

from ..protocol.frame import FrameType, decode_frame, encode_frame
from ..protocol.topics import TOPIC_IDS
from .session import (
    ErrorCode,
    HEARTBEAT_INTERVAL_S,
    WATCHDOG_TIMEOUT_S,
    ClientSession,
    generate_pin,
)


log = logging.getLogger(__name__)


class Bridge(Protocol):
    """Минимальный контракт для Phase 1.3 Zenoh-моста.

    Phase 1.2: not implemented, подменяется NoOpBridge в тестах.
    Phase 1.3: реальная реализация через rclpy.
    """

    async def publish_quest(self, linear: float, angular: float) -> None: ...

    async def publish_emergency(self) -> None: ...

    def subscribe_topic(self, topic_name: str, callback: Callable[[bytes], Awaitable[None]]) -> None: ...


class NoOpBridge:
    """Заглушка для тестов Phase 1.2. Phase 1.3 заменит на Zenoh-мост."""

    async def publish_quest(self, linear: float, angular: float) -> None:
        return None

    async def publish_emergency(self) -> None:
        return None

    def subscribe_topic(self, topic_name: str, callback: Callable[[bytes], Awaitable[None]]) -> None:
        return None


# Текущий PIN — генерится один раз на старте контейнера, логируется.
# Phase 1.6 в start_quest.sh выводит его в docker logs.
ACTIVE_PIN: str = generate_pin()


# Текущий набор занятых server stream_id'ов — шарён между всеми сессиями.
# В PoC один клиент, но контракт позволяет несколько.
_stream_ids_in_use: set[int] = set()


class WSSServer:
    """Серверная логика — собирает app + принимает aiohttp WS-коннекты."""

    def __init__(self, bridge: Bridge, pin: Optional[str] = None) -> None:
        self.bridge = bridge
        self.pin = pin or ACTIVE_PIN
        # Текущие сессии (session_id → ClientSession) для отладки/healthcheck.
        self._sessions: dict[str, ClientSession] = {}

    def get_active_sessions(self) -> int:
        return sum(1 for s in self._sessions.values() if s.is_open())

    async def _send(
        self,
        ws,
        ftype: FrameType,
        stream_id: int,
        payload_obj: dict[str, Any],
    ) -> None:
        raw = json.dumps(payload_obj, separators=(",", ":")).encode("utf-8")
        await ws.send_bytes(encode_frame(ftype, stream_id, raw))

    async def _send_error(
        self,
        ws,
        stream_id: int,
        code: str,
        message: str,
    ) -> None:
        await self._send(
            ws,
            FrameType.ERROR,
            stream_id,
            {"code": code, "message": message},
        )

    async def _send_binary(
        self,
        ws,
        stream_id: int,
        payload: bytes,
    ) -> None:
        await ws.send_bytes(encode_frame(FrameType.BINARY_FRAME, stream_id, payload))

    def _register_session(self, session: ClientSession) -> None:
        self._sessions[session.session_id] = session

    def _unregister_session(self, session: ClientSession) -> None:
        # Освободить stream_id'ы этой сессии.
        for sid in session.subscribed.values():
            _stream_ids_in_use.discard(sid)
        session.close()
        self._sessions.pop(session.session_id, None)

    async def _on_hello(
        self,
        ws,
        session: ClientSession,
        payload_obj: dict[str, Any],
    ) -> bool:
        """Обработать HELLO. True если authenticated, False если нужно закрыть."""
        pin = payload_obj.get("session_pin")
        if not isinstance(pin, str):
            await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "session_pin required")
            return False
        # constant-time compare против тайминг-атак.
        if not secrets.compare_digest(pin, self.pin):
            await self._send_error(ws, 0, ErrorCode.AUTH_FAIL, "wrong PIN")
            log.warning("quest: AUTH_FAIL from peer")
            return False

        client_version = str(payload_obj.get("client_version", "0.0.0"))
        capabilities = payload_obj.get("capabilities") or []
        if not isinstance(capabilities, list):
            capabilities = []
        session.mark_authenticated(client_version, capabilities)

        await self._send(
            ws,
            FrameType.WELCOME,
            0,
            {
                "server_version": "0.1.0",
                "session_id": session.session_id,
                "server_time_ms": int(time.time() * 1000),
            },
        )
        log.info(
            "quest: WELCOME session_id=%s client=%s caps=%s",
            session.session_id,
            client_version,
            capabilities,
        )
        return True

    async def _on_subscribe(
        self,
        ws,
        session: ClientSession,
        payload_obj: dict[str, Any],
    ) -> None:
        if session.state.value != "authenticated":
            await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "subscribe before HELLO")
            return
        topic = payload_obj.get("topic")
        if not isinstance(topic, str) or topic not in TOPIC_IDS:
            await self._send_error(
                ws,
                0,
                ErrorCode.TOPIC_UNKNOWN,
                f"topic '{topic}' not in registry",
            )
            return
        quality = payload_obj.get("quality", "med")
        if quality not in ("low", "med", "high"):
            quality = "med"
        if topic in session.subscribed:
            # идемпотентно: повторный SUBSCRIBE → ack с тем же stream_id.
            sid = session.subscribed[topic]
        else:
            sid = session.allocate_stream_id(_stream_ids_in_use)
            _stream_ids_in_use.add(sid)
            session.subscribed[topic] = sid
        await self._send(
            ws,
            FrameType.JSON_EVENT,
            0,
            {"type": "subscribe_ack", "topic": topic, "stream_id": sid, "quality": quality},
        )

    async def _on_unsubscribe(
        self,
        ws,
        session: ClientSession,
        payload_obj: dict[str, Any],
    ) -> None:
        topic = payload_obj.get("topic")
        if not isinstance(topic, str):
            return
        sid = session.subscribed.pop(topic, None)
        if sid is not None:
            _stream_ids_in_use.discard(sid)

    async def _on_json_event(
        self,
        ws,
        session: ClientSession,
        payload_obj: dict[str, Any],
    ) -> None:
        event_type = payload_obj.get("type")
        if event_type == "ping":
            session.feed_ping()

    async def _on_json_cmd(
        self,
        ws,
        session: ClientSession,
        payload_obj: dict[str, Any],
    ) -> None:
        # Phase 1.2: минимальный роутинг — реальная логика в Phase 1.3.
        cmd = payload_obj.get("cmd")
        if cmd == "stop_emergency":
            await self.bridge.publish_emergency()

    async def _ws_handler(self, request) -> Any:
        """aiohttp WebSocket handler."""
        from aiohttp import web as _aiohttp_web

        ws = _aiohttp_web.WebSocketResponse()
        await ws.prepare(request)
        session = ClientSession()
        self._register_session(session)
        # Отправить heartbeat сразу для clock baseline.
        session.feed_heartbeat()

        # Heartbeat loop (отдельная task).
        heartbeat_task = asyncio.create_task(self._heartbeat_loop(ws, session))
        # Watchdog loop.
        watchdog_task = asyncio.create_task(self._watchdog_loop(ws, session))

        try:
            async for msg in ws:
                if msg.type != msg.type.BINARY:
                    continue  # text frames вне контракта
                try:
                    ftype, sid, payload = decode_frame(msg.data)
                except ValueError as e:
                    await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, str(e))
                    continue
                if ftype == FrameType.HELLO:
                    try:
                        payload_obj = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError) as e:
                        await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, f"bad HELLO json: {e}")
                        continue
                    if not await self._on_hello(ws, session, payload_obj):
                        # AUTH_FAIL → закрыть сокет после отправки ERROR.
                        await ws.close(code=4001, message=b"auth_fail")
                        return ws
                    continue
                if session.state.value != "authenticated":
                    await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "HELLO required first")
                    continue
                if ftype == FrameType.SUBSCRIBE:
                    try:
                        payload_obj = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "bad SUBSCRIBE json")
                        continue
                    await self._on_subscribe(ws, session, payload_obj)
                elif ftype == FrameType.UNSUBSCRIBE:
                    try:
                        payload_obj = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    await self._on_unsubscribe(ws, session, payload_obj)
                elif ftype == FrameType.JSON_EVENT:
                    try:
                        payload_obj = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    await self._on_json_event(ws, session, payload_obj)
                elif ftype == FrameType.JSON_CMD:
                    try:
                        payload_obj = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "bad JSON_CMD json")
                        continue
                    await self._on_json_cmd(ws, session, payload_obj)
                elif ftype == FrameType.GOODBYE:
                    await ws.close(code=1000, message=b"goodbye")
                    return ws
                else:
                    await self._send_error(
                        ws,
                        0,
                        ErrorCode.BAD_PAYLOAD,
                        f"frame type {ftype} not supported in Phase 1.2",
                    )
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001 — логируем и рвём сокет
            log.exception("quest: ws_handler crashed: %s", e)
            await self._send_error(ws, 0, ErrorCode.INTERNAL, str(e))
        finally:
            heartbeat_task.cancel()
            watchdog_task.cancel()
            for task in (heartbeat_task, watchdog_task):
                try:
                    await task
                except (asyncio.CancelledError, Exception):  # noqa: BLE001
                    pass
            self._unregister_session(session)
        return ws

    async def _heartbeat_loop(self, ws, session: ClientSession) -> None:
        try:
            while session.is_open():
                await asyncio.sleep(HEARTBEAT_INTERVAL_S)
                if not session.is_open():
                    return
                await self._send(
                    ws,
                    FrameType.JSON_EVENT,
                    0,
                    {"type": "heartbeat", "ts_ms": int(time.time() * 1000)},
                )
                session.feed_heartbeat()
        except asyncio.CancelledError:
            return
        except Exception as e:  # noqa: BLE001
            log.debug("heartbeat loop ended: %s", e)

    async def _watchdog_loop(self, ws, session: ClientSession) -> None:
        """Проверяет last_ping_monotonic; > WATCHDOG_TIMEOUT_S → close."""
        try:
            while session.is_open():
                await asyncio.sleep(WATCHDOG_TIMEOUT_S / 2.0)
                if session.watchdog_tripped():
                    log.warning(
                        "quest: watchdog trip session=%s",
                        session.session_id,
                    )
                    try:
                        await ws.close(code=4002, message=b"watchdog_timeout")
                    finally:
                        session.close()
                    return
        except asyncio.CancelledError:
            return


# Проверяем наличие aiohttp лениво, чтобы тесты могли мокать.
def _import_aiohttp_web():
    try:
        from aiohttp import web
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "rob_box_quest.ws_server requires aiohttp. "
            "Install it via `pip install aiohttp` "
            "(declared in package.xml as python3-aiohttp)."
        ) from exc
    return web


def build_app(server: WSSServer):
    """Собрать aiohttp Application: /healthz + /quest WS endpoint."""
    web = _import_aiohttp_web()

    async def healthz(_request):
        return web.json_response(
            {
                "status": "ok",
                "sessions_active": server.get_active_sessions(),
                "server_version": "0.1.0",
            }
        )

    async def quest_ws(request):
        # Передаём управление в server._ws_handler, но в нём уже
        # подготовлен сокет. Чтобы не дублировать prepare — вынесу в helper.
        # Phase 1.3 здесь будет подставляться Bridge.
        return await server._ws_handler(request)

    app = web.Application()
    app.router.add_get("/healthz", healthz)
    app.router.add_get("/quest", quest_ws)
    return app


__all__ = [
    "Bridge",
    "NoOpBridge",
    "WSSServer",
    "build_app",
    "ACTIVE_PIN",
]
