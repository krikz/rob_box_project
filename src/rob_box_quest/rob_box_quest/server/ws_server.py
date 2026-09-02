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
import time
from typing import Any, Optional, Protocol

from ..core.floor import FLOOR_HELD_RATE_LIMIT_S, SupervisorFloorTracker
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

    AV-19 (issue #1911, ADR-0028 §4.4 S10): добавлены методы для
    relay-логики teleop-heartbeat и обработки потери teleop_floor.
    Это симметрично ``SupervisorClient`` из ``rob_box_telegram`` —
    единая точка живости клиента для супервизора.
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

    def relay_teleop_heartbeat(self, client_id: str, ts_ms: int, seq: int) -> None:
        """Опубликовать TeleopHeartbeat в ``/teleop_heartbeat`` от ``client_id``.

        Контракт (ADR-0028 §4.4 S10, meta-quest-api.md): payload —
        msgpack-encoded dict ``{client_id, ts_ms, seq}`` в ``std_msgs/String``.
        Вызывается из ws_server._on_json_cmd при получении
        ``teleop_heartbeat`` или ``teleop_twist`` от клиента. Сервер НЕ
        генерирует heartbeat-ы «на автомате» — это обнулит dead-man.

        Параметр ``seq`` — монотонный sequence от клиента; ``ts_ms`` — его
        локальное время (``Date.now()``). Это для диагностики и
        метрик ``dead_man_trips_total``, но НЕ для синхронизации часов.
        """
        ...

    def on_floor_lost(self, client_id: str) -> None:
        """Уведомление: ``teleop_floor`` для ``client_id`` потерян.

        Вызывается из ws_server:
        1) При выходе из режима ``require_teleop_floor=true`` (например,
           кончилось окно dead-man 500 мс — супервизор снял floor).
        2) При явном release в результате FSM-перехода супервизора
           (Telegram-клиент взял teleop_floor).
        3) При рестарте супервизора.

        Bridge обязан немедленно опубликовать ``Twist(0,0)`` в
        ``cmd_vel_quest`` — робот не должен продолжать ехать по инерции
        последнего фрейма. Это fail-safe из ADR-0028 §4.4 «если
        клиент замолчал — супервизор снимет floor».
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

    def relay_teleop_heartbeat(self, client_id: str, ts_ms: int, seq: int) -> None:
        # NoOpBridge: ничего не публикуем, но логируем для тестов.
        log.debug("NoOpBridge: relay_teleop_heartbeat client_id=%s seq=%d", client_id, seq)

    def on_floor_lost(self, client_id: str) -> None:
        # NoOpBridge: no-op для тестов; реальная реализация в QuestBridge.
        log.debug("NoOpBridge: on_floor_lost client_id=%s", client_id)


# Текущий PIN — генерится один раз на старте контейнера, логируется.
# Phase 1.6 в start_quest.sh выводит его в docker logs.
# Если задан ENV QUEST_PIN (например, в docker-compose.yaml) — используется
# фиксированный PIN (удобно для дев-сессий, когда не хочется каждый раз
# лезть в docker logs). Иначе — генерируется 6-значный случайный.
ACTIVE_PIN: str = os.environ.get("QUEST_PIN") or generate_pin()


# Текущий набор занятых server stream_id'ов — шарён между всеми сессиями.
# В PoC один клиент, но контракт позволяет несколько.
_stream_ids_in_use: set[int] = set()


def _consume_future_exception(fut: "asyncio.Future[Any]") -> None:
    """Глушим исключение из Future (иначе asyncio пишет "never retrieved")."""
    if not fut.cancelled():
        fut.exception()


class WSSServer:
    """Серверная логика — собирает app + принимает aiohttp WS-коннекты.

    AV-19 (issue #1911, ADR-0028 §4.4): добавлен локальный floor-tracker
    и параметр ``require_teleop_floor`` — см. design meta-quest-api.md
    §5 (gate teleop_twist + FLOOR_HELD + relay heartbeat + fail-safe).
    """

    def __init__(
        self,
        bridge: Bridge,
        pin: Optional[str] = None,
        require_teleop_floor: bool = False,
        floor_tracker: Optional[SupervisorFloorTracker] = None,
    ) -> None:
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
        # AV-19: gate teleop_floor (Phase 1 — локальный tracker; Phase 2 —
        # proxy на avatar_supervisor service, ADR-0028 §4.4).
        self._require_teleop_floor: bool = require_teleop_floor
        self._floor_tracker: SupervisorFloorTracker = (
            floor_tracker if floor_tracker is not None else SupervisorFloorTracker()
        )
        # client_id (= session_id) → True если ws_server уже сообщил
        # клиенту о FLOOR_HELD-error в текущем окне rate-limit. Нужно,
        # чтобы при HOLD-окне не слать ERROR повторно (rate-limit — на
        # уровне tracker, но здесь дополнительно дедуплим, чтобы один
        # клиент не получал > 1 ошибки в FLOOR_HELD_RATE_LIMIT_S даже
        # если в ws_server прилетают разные teleop_twist-ы).
        self._floor_held_warned_session: dict[str, float] = {}

    def get_active_sessions(self) -> int:
        return sum(1 for s in self._sessions.values() if s.is_open())

    def set_send_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Установить aiohttp-loop для потокобезопасной отправки кадров."""
        self._send_loop = loop

    def _should_send_floor_held_error(self, session_id: str) -> bool:
        """Обёртка над floor_tracker.should_send_floor_held_error.

        Удобно иметь в одном месте, чтобы при смене стратегии
        rate-limit (например, вынести в ROS-параметр) менять одну
        функцию, а не искать по всем ``_on_json_cmd``.
        """
        return self._floor_tracker.should_send_floor_held_error(session_id)

    def on_floor_lost_external(self, client_id: str) -> None:
        """Внешнее уведомление (от Bridge/avatar_supervisor) о потере floor.

        В Phase 2, когда avatar_supervisor будет публиковать
        ``/avatar/state`` с ``teleop_floor != client_id``, эта точка
        будет вызываться из подписки. Сейчас используется из
        QuestBridge через Bridge.on_floor_lost (см. _unregister_session
        для случая закрытия сессии).

        Внутри:
        1) Сбрасываем локальный tracker (если ещё держит — значит
           состояния разошлись, но fail-safe приоритетнее).
        2) Уведомляем Bridge — он опубликует Twist(0,0) в cmd_vel_quest.
        3) Шлём клиенту JSON_EVENT{type:"floor_lost"} с held_by=None —
           клиент DISARM-ит и показывает тост (teleop_fsm).
        """
        if not self._floor_tracker.is_held_by(client_id):
            # Tracker уже не считает client_id держателем — likely двойной
            # уведомление (release в _unregister_session + supervisor
            # подтвердил). Ничего не делаем.
            return
        self._floor_tracker.force_release()
        self._floor_tracker.reset_rate_limit(client_id)
        self._floor_held_warned_session.pop(client_id, None)
        # Уведомить Bridge (QuestBridge опубликует zero Twist).
        self.bridge.on_floor_lost(client_id)
        # Уведомить активные WS-сессии с этим client_id, если ещё открыты.
        session = self._sessions.get(client_id)
        if session is None or not session.is_open():
            return
        ws = self._ws_by_session.get(client_id)
        if ws is None:
            return
        # JSON_EVENT шлём в event-loop (он же вызвал эту функцию).
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            loop = None
        if loop is None:
            return

        async def _notify() -> None:
            try:
                payload = json.dumps(
                    {
                        "type": "floor_lost",
                        "floor": "teleop",
                        "reason": "external_supervisor_or_lost",
                        "ts_ms": int(time.time() * 1000),
                    },
                    separators=(",", ":"),
                ).encode("utf-8")
                await ws.send_bytes(encode_frame(FrameType.JSON_EVENT, 0, payload))
            except Exception as exc:  # noqa: BLE001 — уведомление best-effort
                log.warning("quest: floor_lost notify failed: %s", exc)

        fut = asyncio.run_coroutine_threadsafe(_notify(), loop)
        fut.add_done_callback(_consume_future_exception)
        log.info("quest: floor_lost session_id=%s (external)", client_id)

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
        # AV-19: освободить teleop_floor, если эта сессия его держала.
        # Это симметрично acquire в _on_hello. Если в Phase 2 этот код
        # пойдёт через avatar_supervisor service — здесь будет
        # release_floor service-call.
        was_held = self._floor_tracker.release(session.session_id)
        if was_held:
            self._floor_tracker.reset_rate_limit(session.session_id)
            self._floor_held_warned_session.pop(session.session_id, None)
            log.info("quest: teleop_floor released session_id=%s", session.session_id)
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

        # AV-19: попытаться взять teleop_floor от имени новой сессии.
        # В Phase 1 локальный tracker; в Phase 2 тут будет service-call
        # к /avatar_supervisor/acquire_floor (см. design.md, meta-quest-api.md
        # §5 «Supervisor-команды Phase 2»). Сейчас — best-effort: если
        # floor уже занят другой сессией — мы НЕ отказываем в WELCOME
        # (это убьёт UX при попытке Telegram-op перехватить), но помечаем
        # сессию «не держит» — teleop_twist гейт не пройдёт и шлёт
        # ERROR{FLOOR_HELD} rate-limited (см. _on_json_cmd).
        acquire = self._floor_tracker.acquire(session.session_id)
        if not acquire.granted:
            log.info(
                "quest: HELLO session_id=%s teleop_floor held_by=%s — режим %s",
                session.session_id,
                acquire.held_by,
                "read_only" if self._require_teleop_floor else "silent_gate",
            )

        await self._send(
            ws,
            FrameType.WELCOME,
            0,
            {
                "server_version": "0.1.0",
                "session_id": session.session_id,
                "server_time_ms": int(time.time() * 1000),
                # AV-19: подсказка клиенту о статусе floor (нужно для FSM,
                # чтобы сразу отрисовать «возьми руль» без ожидания
                # первого FLOOR_HELD).
                "teleop_floor_held_by": acquire.held_by,
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
            # AV-19: gate teleop_twist (ADR-0028 §4.4, meta-quest-api.md §5).
            # Если включён require_teleop_floor и эта сессия НЕ держит
            # teleop_floor — НЕ публикуем cmd_vel_quest и отдаём
            # ERROR{FLOOR_HELD} (rate-limited, не чаще 1 раза в секунду;
            # иначе на 30 Гц teleop_twist зальём сокет ошибками).
            # stop_emergency (см. ниже) — ВСЕГДА в обход гейта, по
            # ADR-0028 §4.4 / карточке: «аварийная остановка работает
            # всегда, у кого бы ни был floor».
            ts_ms_raw = payload_obj.get("ts_ms", int(time.time() * 1000))
            seq_raw = payload_obj.get("seq", 0)
            try:
                ts_ms = int(ts_ms_raw) if isinstance(ts_ms_raw, (int, float)) else int(time.time() * 1000)
                seq = int(seq_raw) if isinstance(seq_raw, (int, float)) else 0
            except (TypeError, ValueError):
                ts_ms, seq = int(time.time() * 1000), 0
            if self._require_teleop_floor and not self._floor_tracker.is_held_by(
                session.session_id
            ):
                if self._should_send_floor_held_error(session.session_id):
                    await self._send_error(
                        ws,
                        0,
                        ErrorCode.FLOOR_HELD,
                        f"teleop_floor held by {self._floor_tracker.holder!r}",
                    )
                # НЕ публикуем cmd_vel_quest, но feed_client_alive всё
                # равно вызываем — клиент жив, watchdog должен крутиться.
                self.bridge.feed_client_alive()
                _ = deadman  # намерение: deadman игнорируется пока gate закрыт.
                return
            # Floor наш (или gate выключен) — публикуем.
            self.bridge.publish_quest(linear, angular)
            self.bridge.feed_client_alive()
            # AV-19: relay teleop_heartbeat (ADR-0028 §4.4 S10). Twist-фрейм
            # — тоже живость клиента (он активен), шлём heartbeat-reley.
            # Источник живости — клиент: сервер НЕ генерирует heartbeat-ы
            # на автомате (см. design.md §4.4 «не обнулит dead-man»).
            try:
                self.bridge.relay_teleop_heartbeat(session.session_id, ts_ms, seq)
            except Exception as exc:  # noqa: BLE001 — relay не должен ронять сессию
                log.warning("quest: relay_teleop_heartbeat failed: %s", exc)
            _ = deadman  # Phase 1.5: telemetry через deadman-события
            return
        if cmd == "teleop_heartbeat":
            # AV-19: клиентский heartbeat (ADR-0028 §4.4 S10). Сервер
            # релеит в /teleop_heartbeat от имени этой сессии. Это
            # ВТОРОЙ источник живости (первый — teleop_twist); оба
            # пробрасываются одинаково. Никакого периодического
            # self-loop на сервере: dead-man ловит именно молчание
            # клиента, поэтому heartbeat нельзя слать «на автомате».
            ts_ms_raw = payload_obj.get("ts_ms", int(time.time() * 1000))
            seq_raw = payload_obj.get("seq", 0)
            try:
                ts_ms = int(ts_ms_raw) if isinstance(ts_ms_raw, (int, float)) else int(time.time() * 1000)
                seq = int(seq_raw) if isinstance(seq_raw, (int, float)) else 0
            except (TypeError, ValueError):
                ts_ms, seq = int(time.time() * 1000), 0
            # Если gate включён и floor чужой — relay НЕ шлём (клиент
            # не должен жить в логе супервизора как владелец floor).
            # Можно было бы слать всё равно (heartbeat не вредный), но
            # тогда супервизор может ошибочно «оживить» чужой сессии
            # клиента, что противоречит §4.4 «источник живости — клиент».
            if self._require_teleop_floor and not self._floor_tracker.is_held_by(
                session.session_id
            ):
                # Тем не менее feed_client_alive — watchdog WSS-сессии
                # крутится по любой живости клиента.
                self.bridge.feed_client_alive()
                return
            try:
                self.bridge.relay_teleop_heartbeat(session.session_id, ts_ms, seq)
            except Exception as exc:  # noqa: BLE001
                log.warning("quest: relay_teleop_heartbeat failed: %s", exc)
            self.bridge.feed_client_alive()
            return
        if cmd == "stop_emergency":
            # AV-19: stop_emergency ВСЕГДА в обход гейта (ADR-0028 §4.4,
            # карточка: «аварийная остановка работает всегда, у кого бы
            # ни был floor»). Это страховка от зависшего Quest-клиента
            # в момент, когда floor держит Telegram-оператор: B-кнопка
            # или UI-кнопка в очках ОБЯЗАНА остановить робота.
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
