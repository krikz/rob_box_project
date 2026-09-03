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

from ..core.floor import FLOOR_HELD_RATE_LIMIT_S, SupervisorFloorTracker
from ..protocol.frame import FrameType, decode_frame, encode_frame
from ..streams.registry import STREAM_CATALOG, get_stream
from .session import (
    ErrorCode,
    HEARTBEAT_INTERVAL_S,
    SUPPORTED_SUBPROTOCOLS_V2,
    WATCHDOG_TIMEOUT_S,
    ClientSession,
    generate_pin,
)
from .voice_floor import FloorHolder, FloorState, VoiceFloor

# AV-28 §P7 (issue #1920): список допустимых voice-preset ID и языков вывода.
# Синхронизирован с src/rob_box_voice/config/voice_presets.yaml (PR #1931)
# и meta-quest-api.md §P7. Сервер не выдумывает — если клиент прислал
# не-whitelisted preset/language → NACK (UI откатывает optimistic update).
# Расширение списка = правка YAML + сюда, без правок dialogue_node.
VOICE_PRESET_IDS: tuple[str, ...] = (
    "technical",
    "street",
    "caveman",
    "business",
    "philosopher",
    "lenin",
)
VOICE_LANGUAGES: tuple[str, ...] = ("ru", "en")

# msgpack — payload supervisor-API (0x30..0x33). Импорт ленив: в некоторых
# dev-env модуль может отсутствовать (как у нас на билд-машине для пары
# тестов status). Бинарные фреймы с msgpack-payload идут через отдельный
# encode_helpers.
try:
    import msgpack as _msgpack  # type: ignore[import-untyped]
except ImportError:  # pragma: no cover — dev-env only
    _msgpack = None


# === AV-16: supervisor-API helpers =============================================
# Эти helper-ы живут в ws_server потому что Bridge-supervisor-state snapshot
# отдаётся через msgpack-encoded payload в STATE_UPDATE-фрейме (§3 строка 66).
# Helper-ы — чистая логика, тестируются прямо в protocol/protocol/ тестах.

VALID_FLOORS_V2: tuple[str, ...] = ("teleop", "voice")
VALID_MODES_V2: tuple[str, ...] = (
    "off",
    "telegram_active",
    "avatar_present",
    "mixed",
    "teleop_only",
    "voice_only",
)


def _pack_msgpack(payload: dict) -> bytes:
    """Serialize dict → msgpack bytes (bin-type=True для bytes-полей)."""
    if _msgpack is None:
        raise RuntimeError(
            "msgpack not available — supervisor API requires python3-msgpack"
        )
    return _msgpack.packb(payload, use_bin_type=True)  # type: ignore[union-attr]


def _unpack_msgpack(data: bytes) -> dict:
    if _msgpack is None:
        raise RuntimeError(
            "msgpack not available — supervisor API requires python3-msgpack"
        )
    raw = _msgpack.unpackb(data, raw=False, strict_map_key=False)  # type: ignore[union-attr]
    if not isinstance(raw, dict):
        raise ValueError(
            f"supervisor payload: expected msgpack map, got {type(raw).__name__}"
        )
    return raw


# Keep-alive STATE_UPDATE для v2-сессий (§3 строка 66: «1 Hz keep-alive»).
STATE_UPDATE_KEEPALIVE_S: float = 1.0


log = logging.getLogger(__name__)


def _validate_voice_set_payload(
    preset: Optional[str], language: Optional[str]
) -> Optional[str]:
    """Whitelist preset/language для AV-28 §P7 (тестируется без rclpy).

    Возвращает ``None`` если payload валиден, иначе строку-причину для
    ``voice_set_nack.reason``. Оба поля опциональны — пустой payload
    (ничего не меняем) трактуется как валидный (UI получит ack с ``null``
    в обеих позициях и mode_manager сохранит предыдущие значения).
    """
    if preset is not None and preset not in VOICE_PRESET_IDS:
        return f"invalid_voice_preset: {preset!r}"
    if language is not None and language not in VOICE_LANGUAGES:
        return f"invalid_voice_language: {language!r}"
    return None


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

    def set_voice(
        self, voice_id: str, preset: str | None
    ) -> tuple[bool, str | None, str | None, list[str] | None]:
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

    # ── AV-28 §P7 (issue #1920) — voice style preset + language ──────────────
    # Эти методы отвечают за смену СТИЛЯ речи (technical / street / caveman /
    # business / philosopher / lenin) и языка вывода на ``dialogue_node``.
    # ВНИМАНИЕ: «preset» здесь — это стиль речи (style preset), а НЕ
    # TTS-вариант из ``set_voice(voice_id, preset)`` выше. Контракт
    # разный: AV-27 «preset» — на стороне tts_node, AV-28 — на стороне
    # dialogue_node. Никакого пересечения в рантайме.
    def set_voice_preset(self, preset: str) -> None:
        """AV-28 §P7: выставить ``voice_preset`` на ``dialogue_node``.

        Публикует запрос в ``/avatar/set_voice_preset``; супервизор
        делает ``SetParameters(voice_preset=<preset>)`` (ADR-0028 S5).
        """
        ...

    def set_voice_language(self, language: str) -> None:
        """AV-28 §P7: выставить ``voice_output_language`` на ``dialogue_node``.

        Публикует запрос в ``/avatar/set_voice_language``; супервизор
        делает ``SetParameters(voice_output_language=<language>)``
        (ADR-0028 S5). Без рестарта dialogue_node — параметр
        подхватывается на следующей фразе.
        """
        ...

    def publish_preview_voice(self, request_id: str, voice_id: str, text: str) -> None:
        """Опубликовать запрос на синтез-preview. Ответ придёт асинхронно в
        /avatar/preview_voice/{audio,result,error} → ws_server биндится на
        request_id и шлёт клиенту preview_voice_{audio,done,error}.
        """
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

    # === AV-16: supervisor API (§3 строки 63-66 + §5.1 + §11). ===============
    # Все методы sync (требование соглашения с WS-handler: вызываются
    # из aiohttp event-loop thread без await). Реализация в ``QuestBridge``
    # (quest_node.py) использует run_coroutine_threadsafe на ROS-executor,
    # чтобы НЕ блокировать event-loop aiohttp на синхронном ROS-сервисе.
    #
    # Контракты ответов — простые dict-ы в формате §5.1:
    #   {"granted": bool, "applied": bool, "reason": str, "held_by"?: client_id}
    # Никакой собственный msgpack-словарь — caller пакует payload сам.

    def supervisor_acquire_floor(self, client_id: str, floor: str) -> dict:
        """Запросить ``floor`` (teleop|voice) для ``client_id``.

        Returns: dict с ключами granted/applied/reason (и опц. held_by).
        Никогда НЕ блокирует aiohttp event-loop — реализация обязана
        маршалить запрос в ROS-поток и вернуть управление мгновенно
        (см. ws_server._on_supervisor_acquire_floor; см. acceptance
        «< 100 мс при зависшем сервисе»).
        """
        ...

    def supervisor_release_floor(self, client_id: str, floor: str) -> dict:
        """Отпустить свой ``floor``. Идемпотентно для своего, иначе
        ``applied=false/reason=permission_denied``.

        Returns: dict с ключами applied/reason.
        """
        ...

    def supervisor_set_mode(self, client_id: str, mode: str) -> dict:
        """``SET_MODE`` через FSM ModeManager (§3/§4.1).

        Returns: dict с ключами applied/reason/actual_mode (опц.).
        """
        ...

    def supervisor_state(self) -> "object | None":
        """Текущий снапшот ``/avatar/state`` (msgpack bytes или уже
        распакованный dict — caller использует ; см. AV-16 §3 строка 66).

        None если снапшота ещё нет (мост не подключился). Sync — никаких
        await внутри.
        """
        ...

    def on_supervisor_state(self, cb) -> None:
        """Подписка на изменения ``/avatar/state``.

        ``cb(statesnapshot)`` вызывается из ROS-callback-а (не из aiohttp);
        реализация ``Bridge`` обязана маршалить ``cb`` в aiohttp-loop через
        ``WSSServer._schedule_send_state`` (см. broadcast_frame паттерн).
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

    # ── AV-28 §P7 (issue #1920) — voice style stubs (NoOpBridge) ────────────
    # Симметрично ``set_voice_preset``/``set_voice_language`` в Protocol:
    # NoOpBridge для unit-тестов ws_server без ROS — ничего не публикует,
    # но держит сигнатуру, чтобы isinstance(bridge, Bridge) работал.
    def set_voice_preset(self, preset: str) -> None:
        # NoOpBridge: см. set_voice_mode ниже — фиксируется в логе для теста.
        log.debug("NoOpBridge: set_voice_preset preset=%s", preset)
        return None

    def set_voice_language(self, language: str) -> None:
        # NoOpBridge: фиксируется в логе для теста.
        log.debug("NoOpBridge: set_voice_language language=%s", language)
        return None

    def publish_preview_voice(self, request_id: str, voice_id: str, text: str) -> None:
        # NoOpBridge: без ROS-стека preview-синтез невозможен. WS-тесты
        # проверяют ack/nack через явные вызовы bridge — этот stub только
        # обеспечивает совместимость сигнатуры.
        return None

    def relay_teleop_heartbeat(self, client_id: str, ts_ms: int, seq: int) -> None:
        # NoOpBridge: ничего не публикуем, но логируем для тестов.
        log.debug(
            "NoOpBridge: relay_teleop_heartbeat client_id=%s seq=%d", client_id, seq
        )

    def on_floor_lost(self, client_id: str) -> None:
        # NoOpBridge: no-op для тестов; реальная реализация в QuestBridge.
        log.debug("NoOpBridge: on_floor_lost client_id=%s", client_id)

    # === AV-16: supervisor API заглушки (для unit-тестов ws_server без ROS). ===

    def supervisor_acquire_floor(self, client_id: str, floor: str) -> dict:
        # NoOpBridge: всегда «granted», без held_by — тестам нужны оба пути.
        return {"granted": True, "applied": True, "reason": "noop_granted"}

    def supervisor_release_floor(self, client_id: str, floor: str) -> dict:
        return {"applied": True, "reason": "noop_released"}

    def supervisor_set_mode(self, client_id: str, mode: str) -> dict:
        return {
            "applied": True,
            "reason": "noop_mode_set",
            "actual_mode": mode,
        }

    def supervisor_state(self):
        """NoOpBridge: всегда возвращает минимальный вменяемый снапшот."""
        return {
            "mode": "off",
            "teleop_floor": None,
            "voice_floor": None,
            "last_event": None,
            "since_ms": 0,
            "version": 1,
        }

    def on_supervisor_state(self, cb) -> None:
        # NoOpBridge: подписки нет, тестовый WSSServer руками дёргает cb
        # через subscribe_state-fake по сценарию в тестах.
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
        # AV-27 / issue #1919 — state для preview_voice + rate-limit.
        # request_id → (ws, opened_at) — отдаём клиенту ТОЛЬКО если ws ещё
        # живой; иначе дропаем ответ. lock — потому что ROS callback'и
        # зовут deliver_* из другого потока, а cmd-handler — из aiohttp-loop.
        self._preview_pending: dict[str, tuple[Any, float]] = {}
        self._voice_state_lock = threading.Lock()
        # ws_id(id(ws)) → last-ts (монотонный) per cmd для rate-limit.
        # key = id(ws) (а не сам ws, потому что ws не hashable).
        self._last_voice_cmd_ts: dict[int, dict[str, float]] = {}
        # Voice floor — серверный mutex на голосовой поток (см. voice_floor.py).
        # Один держатель на все WS-сессии, чтобы два квеста (оператор +
        # AV-23 telegram-bridge) не микшировали голос в /avatar/voice_in.
        self._voice_floor = VoiceFloor()
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
            log.debug(
                "ws_server: no event loop; deliver dropped (test_voice=%s)",
                body.get("type"),
            )
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

    # === AV-16: STATE_UPDATE broadcast =========================================
    # 0x33 STATE_UPDATE — broadcast для ВСЕХ v2-сессий (не stream: подписок
    # не требуется). Подписки на состояние — на стороне Quest-клиента
    # через WSS в handler-е on-message (см. §6). STATE_UPDATE — отдельный
    # frame-type, чтобы клиент мог маршрутизировать по типу без SUBSCRIBE.
    def broadcast_state_update(self, state_payload: bytes) -> int:
        """Слать STATE_UPDATE (msgpack bytes) всем v2-сессиям с активным ws.

        Returns: количество клиентов которым доставлено (v1 не получает).
        Вызывается из Bridge-подписки на /avatar/state (ROS-callback-а);
        потому вызов sync, а отправка — через _schedule_send (точно так же,
        как broadcast_frame для BINARY).
        """
        if not self._sessions:
            return 0
        frame = encode_frame(FrameType.STATE_UPDATE, 0, state_payload)
        count = 0
        for sid, session in list(self._sessions.items()):
            if not session.is_open():
                continue
            # v1-сессии STATE_UPDATE не получают (см. §11.1).
            if session.protocol_version != 2:
                continue
            ws = self._ws_by_session.get(sid)
            if ws is None:
                continue
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

    def broadcast_json_event(self, payload_obj: dict[str, Any]) -> int:
        """Слать JSON_EVENT (control notification) всем открытым сессиям.

        В отличие от ``broadcast_frame`` (BINARY_FRAME, привязан к stream_id
        через ``subscribed``), JSON_EVENT — control-frame (stream_id=0) и
        не требует подписки: клиент видит все JSON_EVENT'ы потому что
        соединён. Сейчас используется для ``robot_alert`` (AV-26 / R7);
        ``safety_stop`` уже шлётся через ``_send`` напрямую в сессионном
        цикле.

        Sync (вызывается из ROS/capture-потоков). Возвращает количество
        клиентов которым доставлено. Loop-потокобезопасность — как у
        ``broadcast_frame``.
        """
        if not self._sessions:
            return 0
        raw = json.dumps(payload_obj, separators=(",", ":")).encode("utf-8")
        frame = encode_frame(FrameType.JSON_EVENT, 0, raw)
        count = 0
        for sid, session in list(self._sessions.items()):
            if not session.is_open():
                continue
            ws = self._ws_by_session.get(sid)
            if ws is None:
                continue
            self._schedule_send(ws, frame)
            count += 1
        return count

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

    async def _send_voice_state(
        self,
        ws,
        state: str,
        ts_ms: int,
        holder_id: Optional[str] = None,
        detail: Optional[str] = None,
    ) -> None:
        """Эмиттить voice_state-событие через JSON_EVENT.

        Формат совпадает с meta-quest-api.md §6 (voice_state JSON_EVENT):
            {"type":"voice_state", "state":"<s>", "ts_ms":<n>, ...}

        Доп. поля ``holder_id``/``detail`` — локальное расширение для
        UI чтобы показать «у робота говорит <client>». Схема
        meta-quest-api.md §4 их не запрещает (``utterance_id?`` уже
        опциональный, паттерн тот же).

        Args:
            ws: получатель (обычно requester).
            state: "idle" | "listening" | "speaking" | "denied".
            ts_ms: server clock (мс, int).
            holder_id: кто держит floor (``FloorHolder.label()``), None
                если state == "idle".
            detail: человеко-читаемая подсказка для UI (например,
                "busy: operator-quest:abcd1234").
        """
        payload: dict[str, Any] = {
            "type": "voice_state",
            "state": state,
            "ts_ms": ts_ms,
        }
        if holder_id is not None:
            payload["holder_id"] = holder_id
        if detail is not None:
            payload["detail"] = detail
        await self._send(ws, FrameType.JSON_EVENT, 0, payload)

    def _register_session(self, session: ClientSession, ws) -> None:
        self._sessions[session.session_id] = session
        self._ws_by_session[session.session_id] = ws

    def _unregister_session(self, session: ClientSession) -> None:
        # Освободить stream_id'ы этой сессии.
        for sid in session.subscribed.values():
            _stream_ids_in_use.discard(sid)
        # Освободить voice floor, если эта сессия его держала
        # (watchdog/GOODBYE/disconnect → без явного voice_ptt_stop).
        if self._voice_floor.force_release_for(session.session_id):
            log.info(
                "quest: voice floor released by disconnect session=%s",
                session.session_id,
            )
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
            await self._send_error(
                ws, 0, ErrorCode.BAD_PAYLOAD, "subscribe before HELLO"
            )
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
        # voice_state: эмитим актуальный snapshot floor-а сразу при подписке
        # (а не ждём первого события) — UI должен сразу показать «робот
        # говорит» если кто-то уже держит floor. meta-quest-api.md §4
        # описывает voice_state как event-driven; snapshot — локальное
        # расширение quest-сервера, дешевле (один event на SUBSCRIBE).
        if topic == "voice_state":
            floor_snap = self._voice_floor.snapshot()
            await self._send_voice_state(
                ws,
                state=floor_snap["state"],
                ts_ms=int(time.time() * 1000),
                holder_id=(
                    FloorHolder(
                        session_id=floor_snap["holder"]["session_id"],
                        client_id=floor_snap["holder"]["client_id"],
                    ).label()
                    if floor_snap["holder"] is not None
                    else None
                ),
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
                await self._send_error(
                    ws, 0, ErrorCode.BAD_PAYLOAD, "teleop_twist: bad linear/angular"
                )
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
                ts_ms = (
                    int(ts_ms_raw)
                    if isinstance(ts_ms_raw, (int, float))
                    else int(time.time() * 1000)
                )
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
                ts_ms = (
                    int(ts_ms_raw)
                    if isinstance(ts_ms_raw, (int, float))
                    else int(time.time() * 1000)
                )
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
            #
            # Voice-floor (server-side mutex): два квеста (оператор + AV-23
            # telegram-bridge) НЕ должны одновременно публиковать PCM в
            # /avatar/voice_in. При попытке второго voice_ptt_start шлём
            # requester-у JSON_EVENT{type:"voice_state", state:"denied"} и
            # НЕ вызываем bridge. Подробности см. voice_floor.py + план
            # docs/plans/2026-08-27-quest-voice-passthrough-design.md §5.
            client_id = payload_obj.get("client_id")
            if not isinstance(client_id, str):
                client_id = None
            acquired, busy_holder, _new_state = self._voice_floor.try_acquire(
                session.session_id, client_id
            )
            if not acquired:
                busy_label = (
                    busy_holder.label() if busy_holder is not None else "unknown"
                )
                log.info(
                    "quest: voice floor DENIED session=%s busy=%s",
                    session.session_id,
                    busy_label,
                )
                await self._send_voice_state(
                    ws,
                    state="denied",
                    ts_ms=int(time.time() * 1000),
                    holder_id=busy_label,
                    detail=f"busy: {busy_label}",
                )
                return
            log.info(
                "quest: voice floor ACQUIRED session=%s client_id=%s",
                session.session_id,
                client_id or "anon",
            )
            if payload_obj.get("mode") == "robot_voice":
                self.bridge.publish_voice_robot_start()
            else:
                self.bridge.publish_voice_barge_in()
            await self._send_voice_state(
                ws,
                state="listening",
                ts_ms=int(time.time() * 1000),
                holder_id=(
                    self._voice_floor.holder.label()
                    if self._voice_floor.holder
                    else None
                ),
            )
            return
        if cmd == "voice_ptt_stop":
            # Правый grip (рация) → закрыть голосовой стрим; левый grip
            # (робот-голос) → вытолкнуть буфер PCM в STT.
            #
            # Voice-floor: если requester — текущий держатель, освобождаем
            # и публикуем voice_state{idle}. Иначе — bridge-вызов как
            # прежде (идемпотентен), но floor не трогаем (защита от
            # двойного stop от не-держателя).
            was_holder = self._voice_floor.release(session.session_id)[0]
            if was_holder:
                log.info(
                    "quest: voice floor RELEASED session=%s",
                    session.session_id,
                )
            if payload_obj.get("mode") == "robot_voice":
                self.bridge.publish_voice_robot_stop()
            else:
                self.bridge.publish_voice_stop()
            if was_holder:
                await self._send_voice_state(
                    ws,
                    state="idle",
                    ts_ms=int(time.time() * 1000),
                )
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
                {
                    "type": "voice_mode_ack",
                    "mode": mode,
                    "ts_ms": int(time.time() * 1000),
                },
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
        # === AV-16: supervisor_* JSON-эквиваленты (§5.1) =====================
        # Доступно только v2-сессиям; v1 → ERROR{PROTOCOL_VERSION} тем же
        # поведенческим контрактом, что и в _handle_supervisor_command.
        if cmd in (
            "supervisor_set_mode",
            "supervisor_acquire_floor",
            "supervisor_release_floor",
            "supervisor_get_state",
        ):
            if session.protocol_version != 2:
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.PROTOCOL_VERSION,
                    f"{cmd} requires subprotocol v2 (negotiated: {session.protocol_version}); "
                    f"update client (docs §11)",
                )
                return
            if session.server_client_id is None:
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.AUTH_FAIL,
                    "session not authenticated",
                )
                return
            client_id = session.server_client_id
            # client-supplied client_id игнорируется точно так же как в
            # бинарных фреймах (см. _handle_supervisor_command).
            payload_client_id = payload_obj.get("client_id")
            if payload_client_id is not None and payload_client_id != client_id:
                log.warning(
                    "supervisor_api (JSON): client_id mismatch session=%s "
                    "payload=%s (ignored; using server-side)",
                    session.session_id,
                    payload_client_id,
                )

            if cmd == "supervisor_get_state":
                # poll-эквивалент STATE_UPDATE (§5.1): синхронный ответ.
                snapshot = self.bridge.supervisor_state()
                if snapshot is None:
                    await self._send(
                        ws,
                        FrameType.JSON_EVENT,
                        0,
                        {
                            "type": "supervisor_state",
                            "state": None,
                            "ts_ms": int(time.time() * 1000),
                        },
                    )
                    return
                if isinstance(snapshot, (bytes, bytearray)):
                    # bytes-форма от моста — отдаём как base64-meta в message.
                    # Кандидат на AV-16+ streaming — пока упростим: маленький
                    # inline msgpack через hex-строку в message не нужен,
                    # у клиента есть бинарный STATE_UPDATE-фрейм.
                    await self._send_error(
                        ws,
                        0,
                        ErrorCode.INTERNAL,
                        "supervisor_state snapshot is msgpack bytes; "
                        "use binary STATE_UPDATE frame instead",
                    )
                    return
                await self._send(
                    ws,
                    FrameType.JSON_EVENT,
                    0,
                    {
                        "type": "supervisor_state",
                        "state": snapshot,
                        "ts_ms": int(time.time() * 1000),
                    },
                )
                return

            # supervisor_set_mode / supervisor_acquire_floor / supervisor_release_floor
            if cmd == "supervisor_set_mode":
                mode = payload_obj.get("mode")
                if not isinstance(mode, str) or mode not in VALID_MODES_V2:
                    await self._send_error(
                        ws,
                        0,
                        ErrorCode.BAD_PAYLOAD,
                        f"mode must be one of {list(VALID_MODES_V2)}; got {mode!r}",
                    )
                    return
                try:
                    body = self.bridge.supervisor_set_mode(client_id, mode)
                except Exception as exc:  # noqa: BLE001
                    log.exception("supervisor_set_mode bridge crashed: %s", exc)
                    await self._send_error(
                        ws,
                        0,
                        ErrorCode.INTERNAL,
                        f"supervisor_set_mode: {exc}",
                    )
                    return
                if not body.get("applied"):
                    await self._send_error(
                        ws,
                        0,
                        ErrorCode.MODE_CONFLICT,
                        str(body.get("reason", "refused")),
                    )
                    return
                await self._send(
                    ws,
                    FrameType.JSON_EVENT,
                    0,
                    {
                        "type": "supervisor_state",
                        "state": self.bridge.supervisor_state(),
                        "ts_ms": int(time.time() * 1000),
                    },
                )
                return

            # floor-операции
            floor = payload_obj.get("floor")
            if not isinstance(floor, str) or floor not in VALID_FLOORS_V2:
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.BAD_PAYLOAD,
                    f"floor must be one of {list(VALID_FLOORS_V2)}; got {floor!r}",
                )
                return
            try:
                if cmd == "supervisor_acquire_floor":
                    body = self.bridge.supervisor_acquire_floor(client_id, floor)
                else:  # supervisor_release_floor
                    body = self.bridge.supervisor_release_floor(client_id, floor)
            except Exception as exc:  # noqa: BLE001
                log.exception("supervisor_floor bridge crashed: %s", exc)
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.INTERNAL,
                    f"supervisor floor: {exc}",
                )
                return
            granted = bool(body.get("granted", body.get("applied")))
            if not granted:
                held_by = body.get("held_by")
                reason = body.get("reason", "refused")
                err_message = (
                    reason if held_by is None else f"{reason}; held_by={held_by}"
                )
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.FLOOR_HELD,
                    err_message,
                )
                return
            await self._send(
                ws,
                FrameType.JSON_EVENT,
                0,
                {
                    "type": "supervisor_state",
                    "state": self.bridge.supervisor_state(),
                    "ts_ms": int(time.time() * 1000),
                },
            )
            return

        # ── AV-27 / issue #1919 — TTS picker ──────────────────────────
        if cmd == "list_voices":
            if not self._voice_rate_limit_check(
                ws, "list_voices", VOICE_LIST_MIN_INTERVAL_S
            ):
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
            # ── AV-27 (TTS picker) + AV-28 (style preset + language) ─────
            # Один cmd обслуживает обе фичи. Диспетчер:
            # 1) если в payload есть preset ∈ VOICE_PRESET_IDS или
            #    language ∈ VOICE_LANGUAGES — это AV-28 style/language
            #    запрос → bridge.set_voice_preset / set_voice_language
            #    → супервизор → SetParameters на dialogue_node
            #    (см. ADR-0028 §S5, meta-quest-api.md §P7).
            # 2) иначе (нет style-preset) — это AV-27 voice_id запрос
            #    → bridge.set_voice(voice_id, preset) (TTS picker).
            # Один rate-limit slot «set_voice» шарится между фичами — это
            # сознательно, чтобы UI не мог flood-ить через разные поля.
            if not self._voice_rate_limit_check(
                ws, "set_voice", VOICE_SET_MIN_INTERVAL_S
            ):
                return
            voice_id = payload_obj.get("voice_id")
            preset = payload_obj.get("preset")
            language = payload_obj.get("language")
            ts_ms = int(time.time() * 1000)
            # ── AV-28 §P7: style preset / language flow ────────────────────
            # Развилка по ЗНАЧЕНИЮ, а не по наличию поля: имя `preset`
            # делят две фичи. У AV-27 это пресет провайдера
            # (standard|friendly|authoritative|whisper), у AV-28 — id стиля
            # речи из VOICE_PRESET_IDS. Роутинг «есть preset → значит
            # AV-28» отправлял легитимный запрос picker'а в whitelist
            # стилей и отвечал NACK вместо ACK.
            #
            # Признак AV-28 — либо preset из списка стилей, либо language
            # (у AV-27 такого поля нет вовсе). Всё прочее идёт в AV-27,
            # где preset валидирует сам bridge.
            av28_language = language if isinstance(language, str) else None
            av28_preset = (
                preset
                if isinstance(preset, str) and preset in VOICE_PRESET_IDS
                else None
            )
            # Третий признак: preset без voice_id. У picker'а voice_id
            # обязателен и непуст, у панели стилей он может быть пустым —
            # значит запрос с preset, но без voice_id это AV-28 с
            # неизвестным стилем, и ответить на него надо NACK
            # invalid_voice_preset, а не BAD_PAYLOAD про voice_id.
            style_without_voice = (
                isinstance(preset, str)
                and preset
                and not (isinstance(voice_id, str) and voice_id)
            )
            is_av28_request = (
                av28_preset is not None
                or av28_language is not None
                or bool(style_without_voice)
            )
            if style_without_voice and av28_preset is None:
                av28_preset = preset
            if is_av28_request:
                nack_reason = _validate_voice_set_payload(
                    preset=av28_preset, language=av28_language
                )
                if nack_reason:
                    await self._send(
                        ws,
                        FrameType.JSON_EVENT,
                        0,
                        {
                            "type": "voice_set_nack",
                            "preset": av28_preset,
                            "language": av28_language,
                            "reason": nack_reason,
                            "ts_ms": ts_ms,
                        },
                    )
                    return
                # Применяем: preset → /avatar/set_voice_preset; language →
                # /avatar/set_voice_language. Супервизор делает SetParameters
                # на dialogue_node (см. supervisor_node.py _apply_voice_*).
                if av28_preset is not None:
                    self.bridge.set_voice_preset(av28_preset)
                if av28_language is not None:
                    self.bridge.set_voice_language(av28_language)
                await self._send(
                    ws,
                    FrameType.JSON_EVENT,
                    0,
                    {
                        "type": "voice_set_ack",
                        "preset": av28_preset,
                        "language": av28_language,
                        "ts_ms": ts_ms,
                    },
                )
                return
            # ── AV-27 / issue #1919: TTS picker flow ───────────────────────
            if not isinstance(voice_id, str) or not voice_id:
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.BAD_PAYLOAD,
                    "set_voice: voice_id or preset/language required",
                )
                return
            if preset is not None and not isinstance(preset, str):
                await self._send_error(
                    ws, 0, ErrorCode.BAD_PAYLOAD, "set_voice: preset must be string"
                )
                return
            ok, applied_voice, reason, available = self.bridge.set_voice(
                voice_id, preset
            )
            if not ok:
                err_payload = {
                    "type": "voice_set_nack",
                    "voice_id": voice_id,
                    "reason": reason or "unknown",
                    "ts_ms": ts_ms,
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
                    "ts_ms": ts_ms,
                },
            )
            return
        if cmd == "preview_voice":
            if not self._voice_rate_limit_check(
                ws, "preview_voice", VOICE_PREVIEW_MIN_INTERVAL_S
            ):
                return
            request_id = payload_obj.get("request_id")
            voice_id = payload_obj.get("voice_id")
            text = payload_obj.get("text")
            if not isinstance(request_id, str) or not request_id:
                await self._send_error(
                    ws, 0, ErrorCode.BAD_PAYLOAD, "preview_voice: request_id required"
                )
                return
            if not isinstance(voice_id, str) or not voice_id:
                await self._send_error(
                    ws, 0, ErrorCode.BAD_PAYLOAD, "preview_voice: voice_id required"
                )
                return
            if not isinstance(text, str) or not text:
                await self._send_error(
                    ws, 0, ErrorCode.BAD_PAYLOAD, "preview_voice: text required"
                )
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
        # Per docs/architecture/meta-quest-api.md §11, после перехода на v2 мы
        # поддерживаем ОБА варианта — aiohttp выберет первый совпавший из
        # `protocols` (см. SUPPORTED_SUBPROTOCOLS_V2). Это даёт поэтапный
        # rollout: новый клиент сразу подключается на v2, старый продолжает
        # работать на v1 (monitor-only, см. §11.1).
        ws = _aiohttp_web.WebSocketResponse(protocols=SUPPORTED_SUBPROTOCOLS_V2)
        await ws.prepare(request)
        session = ClientSession()
        # Фиксируем согласованный subprotocol-version: client может прислать
        # ``Sec-WebSocket-Protocol: v2`` или ``v1`` (или ничего — fallback v1).
        # ``ws.ws_protocol`` доступен только ПОСЛЕ ws.prepare().
        if ws.ws_protocol is not None:
            session.apply_subprotocol(ws.ws_protocol)
        else:
            # Клиент не объявил subprotocol. Мы попросили v2/v1, aiohttp
            # выбрал бы первый, но если клиент был без заголовка — оставляем
            # None: ниже в handlers ветка «не v2 → PROTOCOL_VERSION» даст
            # явный сигнал при попытке supervisor-команд.
            session.protocol_version = 1
        self._register_session(session, ws)
        # Отправить heartbeat сразу для clock baseline.
        session.feed_heartbeat()

        # Heartbeat loop (отдельная task).
        heartbeat_task = asyncio.create_task(self._heartbeat_loop(ws, session))
        # Watchdog loop.
        watchdog_task = asyncio.create_task(self._watchdog_loop(ws, session))
        # STATE_UPDATE keep-alive 1 Hz (только для v2-сессий; для v1 — no-op).
        state_update_task = asyncio.create_task(
            self._state_update_keepalive_loop(ws, session)
        )

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
                        await self._send_error(
                            ws, 0, ErrorCode.BAD_PAYLOAD, f"bad HELLO json: {e}"
                        )
                        continue
                    if not await self._on_hello(ws, session, payload_obj):
                        # AUTH_FAIL → закрыть сокет после отправки ERROR.
                        await ws.close(code=4001, message=b"auth_fail")
                        return ws
                    continue
                if session.state.value != "authenticated":
                    await self._send_error(
                        ws, 0, ErrorCode.BAD_PAYLOAD, "HELLO required first"
                    )
                    continue
                if ftype == FrameType.SUBSCRIBE:
                    try:
                        payload_obj = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        await self._send_error(
                            ws, 0, ErrorCode.BAD_PAYLOAD, "bad SUBSCRIBE json"
                        )
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
                        await self._send_error(
                            ws, 0, ErrorCode.BAD_PAYLOAD, "bad JSON_CMD json"
                        )
                        continue
                    await self._on_json_cmd(ws, session, payload_obj)
                elif ftype == FrameType.GOODBYE:
                    await ws.close(code=1000, message=b"goodbye")
                    return ws
                elif ftype == FrameType.VOICE_AUDIO:
                    # Рация: голос оператора → /avatar/voice_in (int16 PCM 16 kHz).
                    self.bridge.publish_voice_audio(payload)
                elif ftype in (
                    FrameType.SET_MODE,
                    FrameType.ACQUIRE_FLOOR,
                    FrameType.RELEASE_FLOOR,
                ):
                    # Supervisor-API (§3 + §11 + AV-16). Только v2-сессии;
                    # v1 присылает 0x30..0x32 → ERROR{PROTOCOL_VERSION}.
                    await self._handle_supervisor_command(ws, session, ftype, payload)
                elif ftype == FrameType.STATE_UPDATE:
                    # Сервер-инициируемый frame; клиент НИКОГДА не должен
                    # слать STATE_UPDATE → ERROR{BAD_PAYLOAD}.
                    await self._send_error(
                        ws,
                        0,
                        ErrorCode.BAD_PAYLOAD,
                        "STATE_UPDATE is server→client only (§3)",
                    )
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
            state_update_task.cancel()
            for task in (heartbeat_task, watchdog_task, state_update_task):
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

    async def _state_update_keepalive_loop(self, ws, session: ClientSession) -> None:
        """Для v2-сессий: каждые ``STATE_UPDATE_KEEPALIVE_S`` шлёт STATE_UPDATE.

        Цель — keep-alive + сообщить клиенту «текущее состояние супервизора»
        даже если FSM/floor-ы не менялись. Payload берётся из
        ``Bridge.supervisor_state()`` (msgpack bytes), мост в quest_node.py
        наполняет его через ROS-подписку на /avatar/state.

        Для v1-сессий — no-op (STATE_UPDATE на v1 не идёт, см. §11.1).
        """
        try:
            while session.is_open():
                await asyncio.sleep(STATE_UPDATE_KEEPALIVE_S)
                if not session.is_open():
                    return
                if session.protocol_version != 2:
                    # v1 — намеренно молчим: STATE_UPDATE в v1 не поддержан.
                    continue
                snapshot = self.bridge.supervisor_state()
                if snapshot is None:
                    # Мост ещё не подключился к /avatar/state — снапшота нет.
                    continue
                # snapshot может быть dict (decoded) или bytes (raw msgpack).
                # Bytes-форма предпочтительна (минуем re-pack overhead), но
                # для совместимости с NoOpBridge (даёт dict) пакуем сами.
                if isinstance(snapshot, (bytes, bytearray)):
                    payload = bytes(snapshot)
                elif isinstance(snapshot, dict):
                    payload = _pack_msgpack({"state": snapshot})
                else:
                    continue
                frame = encode_frame(FrameType.STATE_UPDATE, 0, payload)
                try:
                    await ws.send_bytes(frame)
                except Exception as exc:  # noqa: BLE001
                    # ws закрыт (watchdog-trip или close handshake) — выходим.
                    log.debug("state_update_keepalive: send failed: %s", exc)
                    return
        except asyncio.CancelledError:
            return
        except Exception as e:  # noqa: BLE001
            log.debug("state_update_keepalive ended: %s", e)

    # === AV-16: supervisor-command handler =======================================
    async def _handle_supervisor_command(
        self,
        ws,
        session: ClientSession,
        ftype: FrameType,
        payload: bytes,
    ) -> None:
        """0x30/0x31/0x32 от клиента. Доступно только v2-сессиям.

        v1 → ``ERROR{PROTOCOL_VERSION}`` (наш выбранный поведенческий контракт,
        см. design.md / meta-quest-api.md §11 строки 451-462): молча игнорировать
        — тот же механизм, что прятал баги в AV-14 («клиент шлёт лишнее —
        сервер молча ест»), поэтому v1-клинт СРАЗУ получит явный сигнал
        обновиться через ``ERROR{PROTOCOL_VERSION}`` (см. §8 коды).
        """
        if session.protocol_version != 2:
            await self._send_error(
                ws,
                0,
                ErrorCode.PROTOCOL_VERSION,
                f"{ftype.name} requires subprotocol v2 (negotiated: "
                f"{session.protocol_version}); update client (docs §11)",
            )
            return

        if session.server_client_id is None:
            # Пре-аутентификация, теоретически не должно случиться (выше в
            # _ws_handler есть защита «session.state != authenticated»), но
            # defensive: ошибка аутентификации, не падать.
            await self._send_error(
                ws, 0, ErrorCode.AUTH_FAIL, "session not authenticated"
            )
            return

        # unpack msgpack
        try:
            data = _unpack_msgpack(payload)
        except (ValueError, Exception) as exc:
            # msgpack.exceptions.* наследуются от Exception; не ввозим тип
            # чтобы не ловить ImportError если msgpack-a нет в dev-env.
            await self._send_error(
                ws,
                0,
                ErrorCode.BAD_PAYLOAD,
                f"supervisor payload: {exc}",
            )
            return

        # Игнорируем client-supplied client_id (см. §11 + AV-16 «клиент не
        # должен уметь представиться Telegram'ом»). Сервер подставляет свой
        # server_client_id; расхождение — лог-warning.
        client_id = session.server_client_id
        payload_client_id = data.get("client_id")
        if payload_client_id is not None and payload_client_id != client_id:
            log.warning(
                "supervisor_api: client_id mismatch session=%s payload=%s "
                "(ignored; using server-side)",
                session.session_id,
                payload_client_id,
            )

        if ftype == FrameType.SET_MODE:
            mode = data.get("mode")
            if not isinstance(mode, str) or mode not in VALID_MODES_V2:
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.BAD_PAYLOAD,
                    f"mode must be one of {list(VALID_MODES_V2)}; got {mode!r}",
                )
                return
            try:
                body = self.bridge.supervisor_set_mode(client_id, mode)
            except Exception as exc:  # noqa: BLE001
                # Мост не должен валить event-loop; если падает — это баг
                # реализации Bridge и его надо исправлять, но клиент получит
                # INTERNAL и сможет retry.
                log.exception("supervisor_set_mode bridge crashed: %s", exc)
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.INTERNAL,
                    f"supervisor_set_mode: {exc}",
                )
                return
            # Контракт: applied=False → FSM не пропустила → MODE_CONFLICT;
            # иначе → успех → STATE_UPDATE со свежим снапшотом.
            if not body.get("applied"):
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.MODE_CONFLICT,
                    str(body.get("reason", "refused")),
                )
                return
            # Успех: шлём клиенту свежий STATE_UPDATE с msgpack {state: ...}.
            snapshot = self.bridge.supervisor_state()
            if snapshot is None:
                # мост ещё не подключился — клиент пусть ждёт keep-alive.
                return
            if isinstance(snapshot, (bytes, bytearray)):
                payload_bytes = bytes(snapshot)
            elif isinstance(snapshot, dict):
                payload_bytes = _pack_msgpack({"state": snapshot})
            else:
                return
            await ws.send_bytes(encode_frame(FrameType.STATE_UPDATE, 0, payload_bytes))
            return

        if ftype in (FrameType.ACQUIRE_FLOOR, FrameType.RELEASE_FLOOR):
            floor = data.get("floor")
            if not isinstance(floor, str) or floor not in VALID_FLOORS_V2:
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.BAD_PAYLOAD,
                    f"floor must be one of {list(VALID_FLOORS_V2)}; got {floor!r}",
                )
                return
            try:
                if ftype == FrameType.ACQUIRE_FLOOR:
                    body = self.bridge.supervisor_acquire_floor(client_id, floor)
                else:
                    body = self.bridge.supervisor_release_floor(client_id, floor)
            except Exception as exc:  # noqa: BLE001
                log.exception("supervisor_floor bridge crashed: %s", exc)
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.INTERNAL,
                    f"supervisor floor: {exc}",
                )
                return

            granted = bool(body.get("granted", body.get("applied")))
            if not granted:
                # Конфликт: floor занят другим client_id или другой
                # permission_denied reason. Per §8 код FLOOR_HELD — единый
                # код для обоих сценариев; ``held_by`` в message.
                held_by = body.get("held_by")
                reason = body.get("reason", "refused")
                err_message = (
                    reason if held_by is None else f"{reason}; held_by={held_by}"
                )
                await self._send_error(
                    ws,
                    0,
                    ErrorCode.FLOOR_HELD,
                    err_message,
                )
                return

            # Успех: STATE_UPDATE со свежим snapshot (аналогично SET_MODE).
            snapshot = self.bridge.supervisor_state()
            if snapshot is None:
                return
            if isinstance(snapshot, (bytes, bytearray)):
                payload_bytes = bytes(snapshot)
            elif isinstance(snapshot, dict):
                payload_bytes = _pack_msgpack({"state": snapshot})
            else:
                return
            await ws.send_bytes(encode_frame(FrameType.STATE_UPDATE, 0, payload_bytes))
            return

        # Unreachable: elif chain выше покрывает все три frame-type.
        await self._send_error(
            ws,
            0,
            ErrorCode.BAD_PAYLOAD,
            f"unknown supervisor frame {ftype}",
        )


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
