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
import os
import secrets
import threading
import time
from typing import Any, Optional, Protocol

from ..protocol.frame import FrameType, decode_frame, encode_frame
from ..streams.registry import STREAM_CATALOG, get_stream
from .session import (
    ErrorCode,
    HEARTBEAT_INTERVAL_S,
    WATCHDOG_TIMEOUT_S,
    ClientSession,
    generate_pin,
)


log = logging.getLogger(__name__)


class Bridge(Protocol):
    """Контракт между WS-сервером и capture/ROS-источниками (Phase 1.4 v2).

    Реализация:
    - NoOpBridge — для тестов.
    - QuestBridge (quest_node.py) — реальная, держит TeleopController +
      Watchdog, подписывается на ROS-топики (lidar/status) и получает
      кадры от CameraProvider (camera_oak_color/depth, camera_ceiling).

    Все методы sync (rclpy thread-safe + capture-loop thread). Можно
    вызывать прямо из aiohttp event-loop без await.
    """

    def publish_quest(self, linear: float, angular: float) -> None: ...

    def publish_emergency(self) -> None: ...

    def feed_client_alive(self) -> None:
        """Клиент активен (HELLO/SUBSCRIBE/ping) — сбросить watchdog."""
        ...

    def reset(self) -> None:
        """Новый HELLO / operator ack — снять emergency lock и edge-флаги."""
        ...

    def emergency_stop(self) -> None:
        """Зафиксировать emergency lock (safe stop + close session)."""
        ...

    def publish_frame(self, ui_name: str, payload: bytes) -> None:
        """Bridge публикует payload (JPEG/H.264/msgpack) для всех
        подписанных клиентов на стрим ui_name. Если никто не подписан — no-op.

        Вызывается из:
        - ROS-подписок (lidar_2d, robot_status, voice_state) — payload из
          protocol/topics.py.
        - capture-loop'ов CameraProvider (camera_oak_*, camera_ceiling) —
          payload это JPEG/H.264 bytes с камеры.
        """
        ...

    def available_streams(self) -> list[dict[str, Any]]:
        """JSON-payload для stream_select list cmd (Phase 2 R10)."""
        ...

    def publish_voice_barge_in(self) -> None:
        """PTT start: STOP в /voice/tts/control + /voice/sound/stop (barge-in)."""
        ...

    def publish_voice_audio(self, payload: bytes) -> None:
        """VOICE_AUDIO: publish AudioData в /avatar/voice_in (int16 PCM 16 kHz)."""
        ...

    def publish_voice_stop(self) -> None:
        """PTT stop: STOP в /voice/sound/stop → sound_node закрывает стрим."""
        ...

    def publish_voice_robot_start(self) -> None:
        """PTT start (robot-voice): barge-in + начать буферизацию PCM для STT."""
        ...

    def publish_voice_robot_stop(self) -> None:
        """PTT stop (robot-voice): буфер → AudioData в /audio/quest_in (STT)."""
        ...

    def set_voice_mode(self, mode: str) -> None:
        """voice_mode cmd → сменить режим голоса (через супервизор, ADR-0028 S5)."""
        ...

    # ── TTS picker (AV-27 / issue #1919) ─────────────────────────────────
    # Эти методы вызываются из JSON_CMD-хэндлеров. Контракт:
    # - list_voices: вернуть (sync) последний кэш /voice/tts/voices + active provider/voice;
    #   если кэш пуст — значит tts_node ещё ни разу не опубликовал latched-топик
    #   (см. design t_5b9d5d0c §128-150), отдаём честно пустой список.
    # - set_voice: синхронная валидация по текущему активному провайдеру (tts_node
    #   уже держит его в кэше). Если валидно — публикуем запрос в /avatar/set_voice
    #   и шлём ack; финальное подтверждение (или rollback) придёт через
    #   /voice/tts/provider_state (republish tts_node на смену параметра).
    # - preview_voice: публикуем в /avatar/preview_voice — supervisor гонит синтез
    #   и публикует результат в /avatar/preview_voice/result + /avatar/preview_voice/audio.
    # См. docs/architecture/tts-picker-ros-path.md.
    def list_voices_snapshot(self) -> dict[str, Any]:
        """Sync-снимок кэша: {voices:[VoiceInfo], active_provider, active_voice, ts_ms}.

        QuestBridge сам решает, протух ли кэш (voices_cache_ttl_sec). WS-сервер
        только форвардит: ни валидации, ни TTL тут нет.
        """
        ...

    def set_voice(self, voice_id: str, preset: str | None) -> tuple[bool, str | None, str | None, list[str] | None]:
        """Sync-валидация + публикация /avatar/set_voice.

        Returns:
            (ok, applied_voice_id, reason, available)
            * ok=True, applied_voice_id=voice_id (фактически подтверждённый голос
              у текущего активного провайдера), reason=None, available=None —
              нормальный ack;
            * ok=False, reason="voice_unavailable"|"tts_unreachable"|..., available=[...]
              — для nack; available заполняется когда валидно провайдер не знает
              запрошенный голос (для UI-подсказки).
        """
        ...

    def publish_preview_voice(self, request_id: str, voice_id: str, text: str) -> None:
        """Опубликовать запрос на синтез-preview. Ответ придёт асинхронно в
        /avatar/preview_voice/{audio,result,error} → ws_server биндится на
        request_id и шлёт клиенту preview_voice_{audio,done,error}.
        """
        ...


class NoOpBridge:
    """Заглушка для тестов. Реальная реализация в quest_node.py."""

    def publish_quest(self, linear: float, angular: float) -> None:
        return None

    def publish_emergency(self) -> None:
        return None

    def feed_client_alive(self) -> None:
        return None

    def reset(self) -> None:
        return None

    def emergency_stop(self) -> None:
        return None

    def publish_frame(self, ui_name: str, payload: bytes) -> None:
        return None

    def available_streams(self) -> list[dict[str, Any]]:
        # NoOpBridge: возвращаем весь каталог (тестам нужны имена для проверки).
        items: list[dict[str, Any]] = []
        for name, spec in STREAM_CATALOG.items():
            items.append(
                {
                    "topic": name,
                    "topic_id": spec.topic_id,
                    "kind": spec.kind.value,
                    "source": spec.source,
                    "default_quality": spec.default_quality,
                    "description": spec.description,
                }
            )
        return items

    def publish_voice_barge_in(self) -> None:
        return None

    def publish_voice_audio(self, payload: bytes) -> None:
        return None

    def publish_voice_stop(self) -> None:
        return None

    def publish_voice_robot_start(self) -> None:
        return None

    def publish_voice_robot_stop(self) -> None:
        return None

    def set_voice_mode(self, mode: str) -> None:
        return None

    # ── TTS picker stubs (AV-27) ────────────────────────────────────────
    def list_voices_snapshot(self) -> dict[str, Any]:
        # NoOpBridge: пустой кэш. UI получает voices_list с voices=[] и
        # видит «провайдер не отдаёт список голосов» (issue #1919 acceptance).
        return {
            "voices": [],
            "active_provider": "",
            "active_voice": "",
            "ts_ms": 0,
        }

    def set_voice(
        self, voice_id: str, preset: str | None
    ) -> tuple[bool, str | None, str | None, list[str] | None]:
        # Тестовая среда не публикует ничего; возвращаем nack чтобы WS-тесты
        # видели честный «no-op без моста».
        return False, None, "tts_unreachable", None

    def publish_preview_voice(self, request_id: str, voice_id: str, text: str) -> None:
        # NoOpBridge: без ROS-стека preview-синтез невозможен. WS-тесты
        # проверяют ack/nack через явные вызовы bridge — этот stub только
        # обеспечивает совместимость сигнатуры.
        return None


# Текущий PIN — генерится один раз на старте контейнера, логируется.
# Phase 1.6 в start_quest.sh выводит его в docker logs.
# Если задан ENV QUEST_PIN (например, в docker-compose.yaml) — используется
# фиксированный PIN (удобно для дев-сессий, когда не хочется каждый раз
# лезть в docker logs). Иначе — генерируется 6-значный случайный.
ACTIVE_PIN: str = os.environ.get("QUEST_PIN") or generate_pin()


# Текущий набор занятых server stream_id'ов — шарён между всеми сессиями.
# В PoC один клиент, но контракт позволяет несколько.
_stream_ids_in_use: set[int] = set()


# AV-27 / issue #1919 — rate-limit policy (docs/architecture/meta-quest-api.md §9):
# list_voices ≤ 1/10s, set_voice ≤ 1/2s, preview_voice ≤ 1/5s + ≤3 параллельных.
# Реализуется через in-memory last-ts per ws (не per session) — соединение
# одно, но политика прибита к клиенту.
VOICE_LIST_MIN_INTERVAL_S: float = 10.0
VOICE_SET_MIN_INTERVAL_S: float = 2.0
VOICE_PREVIEW_MIN_INTERVAL_S: float = 5.0
VOICE_PREVIEW_MAX_CONCURRENT: int = 3


def _consume_future_exception(fut: "asyncio.Future[Any]") -> None:
    """Глушим исключение из Future (иначе asyncio пишет "never retrieved")."""
    if not fut.cancelled():
        fut.exception()


class WSSServer:
    """Серверная логика — собирает app + принимает aiohttp WS-коннекты."""

    def __init__(self, bridge: Bridge, pin: Optional[str] = None) -> None:
        self.bridge = bridge
        self.pin = pin or ACTIVE_PIN
        # Текущие сессии (session_id → ClientSession) для отладки/healthcheck.
        self._sessions: dict[str, ClientSession] = {}
        # session_id → активный ws (для broadcast_frame из capture-loops).
        # Хранится ОТДЕЛЬНО от ClientSession чтобы можно было отвязать
        # (без race на is_open() во время unregister).
        self._ws_by_session: dict[str, Any] = {}
        # aiohttp event-loop для потокобезопасной отправки BINARY_FRAME.
        # broadcast_frame вызывается из ROS/capture-потоков, где нет running
        # loop — раньше кадр молча терялся (чёрный экран). Устанавливается
        # quest_node через set_send_loop().
        self._send_loop: Optional[asyncio.AbstractEventLoop] = None
        # AV-27 / issue #1919 — state для preview_voice + rate-limit.
        # request_id → (ws, opened_at) — отдаём клиенту ТОЛЬКО если ws ещё
        # живой; иначе дропаем ответ. lock — потому что ROS callback'и
        # зовут deliver_* из другого потока, а cmd-handler — из aiohttp-loop.
        self._preview_pending: dict[str, tuple[Any, float]] = {}
        self._voice_state_lock = threading.Lock()
        # ws_id(id(ws)) → last-ts (монотонный) per cmd для rate-limit.
        # key = id(ws) (а не сам ws, потому что ws не hashable).
        self._last_voice_cmd_ts: dict[int, dict[str, float]] = {}

    def get_active_sessions(self) -> int:
        return sum(1 for s in self._sessions.values() if s.is_open())

    def set_send_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Установить aiohttp-loop для потокобезопасной отправки кадров."""
        self._send_loop = loop

    # ── AV-27 / issue #1919 — TTS picker helpers ────────────────────────

    def _voice_rate_limit_check(self, ws: Any, cmd: str, min_interval_s: float) -> bool:
        """True если cmd разрешён (лимит не превышен). False = drop + log."""
        ws_id = id(ws)
        now = time.monotonic()
        with self._voice_state_lock:
            per_ws = self._last_voice_cmd_ts.setdefault(ws_id, {})
            last = per_ws.get(cmd, 0.0)
            if now - last < min_interval_s:
                return False
            per_ws[cmd] = now
        return True

    def start_preview_session(self, request_id: str, ws: Any) -> bool:
        """Зарегистрировать request_id → ws для preview.

        Returns False если уже есть `VOICE_PREVIEW_MAX_CONCURRENT` активных
        request_id'ов — клиент получит preview_voice_error{reason: "too_many"}.
        """
        with self._voice_state_lock:
            # Чистим зависшие (старше 60 с) — на случай если supervisor упал.
            now = time.monotonic()
            stale = [k for k, (_, t) in self._preview_pending.items() if now - t > 60.0]
            for k in stale:
                self._preview_pending.pop(k, None)
            if len(self._preview_pending) >= VOICE_PREVIEW_MAX_CONCURRENT:
                return False
            self._preview_pending[request_id] = (ws, now)
        return True

    def deliver_preview_audio(
        self,
        request_id: str,
        audio_bytes: bytes,
        audio_format: str,
        content_type: str,
        seq: int,
        total: int,
    ) -> bool:
        """Опубликовать audio preview в WS клиента + JSON_EVENT{type:preview_voice_audio}.

        Returns True если request_id был зарегистрирован и ws ещё живой.
        Sync; работает как из aiohttp-loop, так и из ROS-thread (через
        _send_loop.call_soon_threadsafe — fire-and-forget, не блокирует loop).
        """
        with self._voice_state_lock:
            entry = self._preview_pending.get(request_id)
            if entry is None:
                return False
            ws, _ = entry
        if ws.closed:
            with self._voice_state_lock:
                self._preview_pending.pop(request_id, None)
            return False
        ts_ms = int(time.time() * 1000)
        meta = {
            "type": "preview_voice_audio",
            "request_id": request_id,
            "format": audio_format,
            "content_type": content_type,
            "seq": seq,
            "total": total,
            "ts_ms": ts_ms,
        }
        self._schedule_ws_send(ws, meta)
        if audio_bytes:
            self._schedule_ws_send_binary(ws, audio_bytes)
        return True

    def deliver_preview_done(self, request_id: str) -> bool:
        """Финальный preview_voice_done → клиенту. Чистит pending."""
        with self._voice_state_lock:
            entry = self._preview_pending.pop(request_id, None)
            if entry is None:
                return False
            ws, _ = entry
        if ws.closed:
            return False
        body = {
            "type": "preview_voice_done",
            "request_id": request_id,
            "ts_ms": int(time.time() * 1000),
        }
        self._schedule_ws_send(ws, body)
        return True

    def deliver_preview_error(self, request_id: str, reason: str) -> bool:
        """Ошибка preview → preview_voice_error. Чистит pending."""
        with self._voice_state_lock:
            entry = self._preview_pending.pop(request_id, None)
            if entry is None:
                return False
            ws, _ = entry
        if ws.closed:
            return False
        body = {
            "type": "preview_voice_error",
            "request_id": request_id,
            "reason": reason,
            "ts_ms": int(time.time() * 1000),
        }
        self._schedule_ws_send(ws, body)
        return True

    def _schedule_ws_send(self, ws: Any, body: dict[str, Any]) -> None:
        """Потокобезопасно запланировать отправку JSON_EVENT в aiohttp-loop.

        Fire-and-forget: в ROS-потоке зовём ``_send_loop.call_soon_threadsafe``,
        из aiohttp-loop (тесты) — планируем через ``loop.call_soon``. Если
        loop'а нет (юнит-тесты ws_server без aiohttp) — drop + debug-лог.
        """
        loop = self._send_loop
        if loop is None or not loop.is_running():
            log.debug("ws_server: no event loop; deliver dropped (test_voice=%s)", body.get("type"))
            return
        try:
            loop.call_soon_threadsafe(self._send_async, ws, body)
        except RuntimeError as exc:  # loop closed
            log.debug("ws_server: schedule failed: %s", exc)

    def _schedule_ws_send_binary(self, ws: Any, payload: bytes) -> None:
        loop = self._send_loop
        if loop is None or not loop.is_running():
            return
        try:
            loop.call_soon_threadsafe(self._send_binary_async, ws, payload)
        except RuntimeError:  # noqa: BLE001
            return

    def _send_async(self, ws: Any, body: dict[str, Any]) -> None:
        """Алиас над ``_send`` для call_soon_threadsafe (loop уже наш)."""
        loop = self._send_loop
        if loop is None:
            return
        try:
            coro = self._send(ws, FrameType.JSON_EVENT, 0, body)
            # В loop'е можно прямо awaitить через create_task.
            loop.create_task(coro)
        except Exception as exc:  # noqa: BLE001
            log.debug("ws_server: _send_async failed: %s", exc)

    def _send_binary_async(self, ws: Any, payload: bytes) -> None:
        loop = self._send_loop
        if loop is None:
            return
        try:
            loop.create_task(self._send_binary(ws, payload))
        except Exception as exc:  # noqa: BLE001
            log.debug("ws_server: _send_binary_async failed: %s", exc)

    async def _send_binary(self, ws: Any, payload: bytes) -> None:
        """Sync-wrapper для отправки raw bytes в WS (для preview-аудио).

        BINARY_FRAME с stream_id=0 — отдельный от метаданных JSON_EVENT. См.
        meta-quest-api.md §4.2 + messages.ts:148-156."""
        if ws.closed:
            return
        await ws.send_bytes(encode_frame(FrameType.BINARY_FRAME, 0, payload))

    def broadcast_frame(self, ui_name: str, payload: bytes) -> int:
        """Слать BINARY_FRAME всем сессиям, подписанным на ui_name.

        Вызывается из Bridge.publish_frame (ROS-callback'и + capture-loops).
        Sync: encode + send_bytes из aiohttp event-loop thread. Если вызов
        из capture-thread — schedule через loop.call_soon_threadsafe.

        Returns: количество клиентов которым доставлено.
        """
        if not self._sessions:
            return 0
        count = 0
        for sid, session in list(self._sessions.items()):
            if not session.is_open():
                continue
            stream_id = session.subscribed.get(ui_name)
            if stream_id is None:
                continue
            ws = self._ws_by_session.get(sid)
            if ws is None:
                continue
            frame = encode_frame(FrameType.BINARY_FRAME, stream_id, payload)
            # ws.send_bytes — coroutine. Из aiohttp-loop — await напрямую;
            # из другого потока — call_soon_threadsafe (Phase 1.5).
            self._schedule_send(ws, frame)
            count += 1
        return count

    def _schedule_send(self, ws, frame: bytes) -> None:
        """Отправить frame в ws из любого потока.

        broadcast_frame вызывается из ROS-потока (rclpy executor) и
        capture-потоков, где `asyncio.get_running_loop()` кидает RuntimeError —
        раньше кадр молча терялся (чёрный экран). Теперь шлём через
        run_coroutine_threadsafe на сохранённом aiohttp-loop.
        """
        loop = self._send_loop
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            pass
        if loop is None:
            return
        try:
            fut = asyncio.run_coroutine_threadsafe(ws.send_bytes(frame), loop)
        except RuntimeError:
            return  # loop закрыт/не запущен — кадр теряем, не роняем ноду
        fut.add_done_callback(_consume_future_exception)

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

    def _register_session(self, session: ClientSession, ws) -> None:
        self._sessions[session.session_id] = session
        self._ws_by_session[session.session_id] = ws

    def _unregister_session(self, session: ClientSession) -> None:
        # Освободить stream_id'ы этой сессии.
        for sid in session.subscribed.values():
            _stream_ids_in_use.discard(sid)
        session.close()
        self._sessions.pop(session.session_id, None)
        self._ws_by_session.pop(session.session_id, None)

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

        # Новый HELLO: снять emergency lock от прошлой сессии / stop_emergency
        # и взвести bridge-watchdog (дальше его кормит клиентский ping).
        self.bridge.reset()
        self.bridge.feed_client_alive()

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
        spec = get_stream(topic) if isinstance(topic, str) else None
        if spec is None:
            await self._send_error(
                ws,
                0,
                ErrorCode.TOPIC_UNKNOWN,
                f"topic '{topic}' not in registry",
            )
            return
        quality = payload_obj.get("quality", spec.default_quality)
        if quality not in ("low", "med", "high"):
            quality = spec.default_quality
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
            {
                "type": "subscribe_ack",
                "topic": topic,
                "stream_id": sid,
                "quality": quality,
                "kind": spec.kind.value,
            },
        )

    async def _on_json_cmd(
        self,
        ws,
        session: ClientSession,
        payload_obj: dict[str, Any],
    ) -> None:
        """JSON_CMD → Bridge + meta-commands.

        Контракт:
        - ping → session.feed_ping() (клиент шлёт JSON_CMD{cmd:"ping"})
                 или JSON_EVENT{type:"ping"} — обрабатывается в _on_json_event
        - teleop_twist → Bridge.publish_quest + feed_client_alive
        - stop_emergency → Bridge.publish_emergency + emergency_stop
        - stream_select → переключение активного camera-стрима
        - stream_list → JSON_EVENT{type: stream_list, items: [...]}
        - voice_ptt_start/stop → Bridge barge-in / voice stop (рация, P3)
        - voice_mode / ui_button → Phase 2
        """
        cmd = payload_obj.get("cmd")
        if cmd == "ping":
            # Клиентский webxr_client/src/wire/connection.ts шлёт ping как
            # JSON_CMD{cmd:"ping"} (отступление от контракта meta-quest-api.md
            # §7, который говорит JSON_EVENT{type:"ping"}). Сбрасываем watchdog
            # в обоих случаях чтобы не терять сессию.
            session.feed_ping()
            self.bridge.feed_client_alive()
            return
        if cmd == "stream_list":
            items = []
            for s in self.bridge.available_streams():
                items.append(s)
            await self._send(
                ws,
                FrameType.JSON_EVENT,
                0,
                {"type": "stream_list", "items": items},
            )
            return
        if cmd == "teleop_twist":
            try:
                linear = float(payload_obj.get("linear", {}).get("x", 0.0))
                angular = float(payload_obj.get("angular", {}).get("z", 0.0))
            except (TypeError, ValueError):
                await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "teleop_twist: bad linear/angular")
                return
            deadman = bool(payload_obj.get("deadman", False))
            self.bridge.publish_quest(linear, angular)
            self.bridge.feed_client_alive()
            _ = deadman  # Phase 1.5: telemetry через deadman-события
            return
        if cmd == "stop_emergency":
            self.bridge.publish_emergency()
            self.bridge.emergency_stop()
            return
        if cmd == "voice_ptt_start":
            # PTT start: mode "robot_voice" (левый grip) → STT → LLM → TTS;
            # иначе "radio" (правый grip) → barge-in (прервать TTS + музыку).
            if payload_obj.get("mode") == "robot_voice":
                self.bridge.publish_voice_robot_start()
            else:
                self.bridge.publish_voice_barge_in()
            return
        if cmd == "voice_ptt_stop":
            # Правый grip (рация) → закрыть голосовой стрим; левый grip
            # (робот-голос) → вытолкнуть буфер PCM в STT.
            if payload_obj.get("mode") == "robot_voice":
                self.bridge.publish_voice_robot_stop()
            else:
                self.bridge.publish_voice_stop()
            return
        if cmd == "voice_mode":
            # Смена режима голоса (off/passthrough/ttts_proxy/stt_llm/...).
            # Маршрутизация — через супервизор (ADR-0028 S5): bridge переводит
            # wire-режим в voice_input_mode и публикует запрос супервизору.
            mode = payload_obj.get("mode")
            self.bridge.set_voice_mode(mode)
            await self._send(
                ws,
                FrameType.JSON_EVENT,
                0,
                {"type": "voice_mode_ack", "mode": mode, "ts_ms": int(time.time() * 1000)},
            )
            return
        if cmd == "stream_select":
            # Meta-command: UI запросил смену активного стрима.
            # Сервер подтверждает что стрим есть в registry, и возвращает
            # текущий stream_id (если уже подписан) или подсказывает
            # SUBSCRIBE. Клиент сам решает — UNSUBSCRIBE+SUBSCRIBE.
            topic = payload_obj.get("topic")
            spec = get_stream(topic) if isinstance(topic, str) else None
            if spec is None:
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.TOPIC_UNKNOWN,
                    f"topic '{topic}' not in registry",
                )
                return
            sid = session.subscribed.get(topic)
            await self._send(
                ws,
                FrameType.JSON_EVENT,
                0,
                {
                    "type": "stream_select_ack",
                    "topic": topic,
                    "stream_id": sid,  # может быть None → клиент делает SUBSCRIBE
                    "kind": spec.kind.value,
                },
            )
            return

        # ── AV-27 / issue #1919 — TTS picker ──────────────────────────
        if cmd == "list_voices":
            if not self._voice_rate_limit_check(ws, "list_voices", VOICE_LIST_MIN_INTERVAL_S):
                return  # drop + log внутри
            snap = self.bridge.list_voices_snapshot()
            await self._send(
                ws,
                FrameType.JSON_EVENT,
                0,
                {
                    "type": "voice_list",
                    "voices": snap["voices"],
                    "active_provider": snap["active_provider"],
                    "active_voice": snap["active_voice"],
                    "ts_ms": int(time.time() * 1000),
                },
            )
            return
        if cmd == "set_voice":
            if not self._voice_rate_limit_check(ws, "set_voice", VOICE_SET_MIN_INTERVAL_S):
                return
            voice_id = payload_obj.get("voice_id")
            preset = payload_obj.get("preset")
            if not isinstance(voice_id, str) or not voice_id:
                await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "set_voice: voice_id required")
                return
            if preset is not None and not isinstance(preset, str):
                await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "set_voice: preset must be string")
                return
            ok, applied_voice, reason, available = self.bridge.set_voice(voice_id, preset)
            if not ok:
                err_payload = {
                    "type": "voice_set_nack",
                    "voice_id": voice_id,
                    "reason": reason or "unknown",
                    "ts_ms": int(time.time() * 1000),
                }
                if available:
                    err_payload["available"] = available
                await self._send(ws, FrameType.JSON_EVENT, 0, err_payload)
                return
            await self._send(
                ws,
                FrameType.JSON_EVENT,
                0,
                {
                    "type": "voice_set_ack",
                    "voice_id": applied_voice or voice_id,
                    "preset": preset or "standard",
                    "ts_ms": int(time.time() * 1000),
                },
            )
            return
        if cmd == "preview_voice":
            if not self._voice_rate_limit_check(ws, "preview_voice", VOICE_PREVIEW_MIN_INTERVAL_S):
                return
            request_id = payload_obj.get("request_id")
            voice_id = payload_obj.get("voice_id")
            text = payload_obj.get("text")
            if not isinstance(request_id, str) or not request_id:
                await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "preview_voice: request_id required")
                return
            if not isinstance(voice_id, str) or not voice_id:
                await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "preview_voice: voice_id required")
                return
            if not isinstance(text, str) or not text:
                await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "preview_voice: text required")
                return
            if not self.start_preview_session(request_id, ws):
                await self._send(
                    ws,
                    FrameType.JSON_EVENT,
                    0,
                    {
                        "type": "preview_voice_error",
                        "request_id": request_id,
                        "reason": "too_many_concurrent_previews",
                        "ts_ms": int(time.time() * 1000),
                    },
                )
                return
            self.bridge.publish_preview_voice(request_id, voice_id, text)
            return

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
            # Клиентский ping (JSON_EVENT) — он же keepalive bridge-watchdog'а:
            # иначе при простое (без teleop_twist) watchdog ложно триггерит
            # emergency_stop и блокирует телеоп навсегда.
            self.bridge.feed_client_alive()
            # pong с эхом клиентского ts_ms (meta-quest-api.md §6/§7): клиент
            # считает RTT по своим часам, без синхронизации с сервером.
            await self._send(
                ws,
                FrameType.JSON_EVENT,
                0,
                {
                    "type": "pong",
                    "ts_ms": payload_obj.get("ts_ms"),
                    "server_ts_ms": int(time.time() * 1000),
                },
            )

    # Управление _on_json_cmd перенесено в новый метод выше (см. Phase 1.4):
    # stream_select / stream_list + teleop_twist / stop_emergency.

    async def _ws_handler(self, request) -> Any:
        """aiohttp WebSocket handler."""
        from aiohttp import web as _aiohttp_web

        # Echo negotiated subprotocol (Sec-WebSocket-Protocol). Without this,
        # Chrome refuses the handshake with:
        #   "Sent non-empty 'Sec-WebSocket-Protocol' header but no response
        #    was received"
        # Per docs/architecture/meta-quest-api.md §3, only "robbox-quest-v1"
        # is supported; aiohttp will pick the first match from `protocols`.
        ws = _aiohttp_web.WebSocketResponse(protocols=("robbox-quest-v1",))
        await ws.prepare(request)
        session = ClientSession()
        self._register_session(session, ws)
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
                elif ftype == FrameType.VOICE_AUDIO:
                    # Рация: голос оператора → /avatar/voice_in (int16 PCM 16 kHz).
                    self.bridge.publish_voice_audio(payload)
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
