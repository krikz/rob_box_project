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

from ..core.avatar_arbiter import (
    FLOOR_HELD_RATE_LIMIT_S,
    AcquireResult,
    LocalAvatarArbiterClient,
)
from ..core.floor import AvatarFloorSnapshot, AvatarStateFloorCache, FloorViewUpdate
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
from .voice_floor import FloorHolder, FloorState, VoiceFloorCache

# AV-28 §P7 (issue #1920): whitelist voice-preset ID и языков вывода.
# Single source of truth — ``rob_box_core.bridge_protocol`` (туда же
# импортирует supervisor_node). Если кто-то добавляет 8-й пресет в
# voice_presets.yaml + bridge_protocol, ws_server подхватит его
# автоматически. Раньше тут была копия-tuple — issue #2240 фиксирует
# третий инцидент с расхождением копий (валидация NACK'ала реальный
# preset). Не возвращаемся к локальному объявлению: conformance-тест
# ``test_ws_server_voice_presets`` всё равно проверит, что
# ``ws_server.VOICE_PRESET_IDS`` — это тот же объект, что в каталоге.
from rob_box_core.bridge_protocol import (
    VOICE_LANGUAGES,  # noqa: F401  (re-export для обратной совместимости)
    VOICE_PRESET_IDS,  # noqa: F401  (re-export для обратной совместимости)
)

# msgpack — payload supervisor-API (0x30..0x33). Импорт ленив: в некоторых
# dev-env модуль может отсутствовать (как у нас на билд-машине для пары
# тестов status). Бинарные фреймы с msgpack-payload идут через отдельный
# encode_helpers.
try:
    import msgpack as _msgpack  # type: ignore[import-untyped]
except ImportError:  # pragma: no cover — dev-env only
    _msgpack = None


# === JSON_CMD dispatch table =================================================
# Handler implementations are defined after WSSServer; this annotation keeps
# the table's interface explicit while allowing handlers to call server seams.
JSON_CMD_HANDLERS: dict[str, Any] = {}



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

# Каркас supervisor-API frame-типов. Расширение (новый supervisor frame)
# = добавить FrameType в этот набор + handler в SUPERVISOR_HANDLERS ниже.
# Сейчас только бинарные supervisor-фреймы; cmd-flow (supervisor_* через
# JSON_CMD) живёт отдельно в _on_json_cmd.
SUPERVISOR_FRAME_TYPES: frozenset[FrameType] = frozenset(
    {FrameType.SET_MODE, FrameType.ACQUIRE_FLOOR, FrameType.RELEASE_FLOOR}
)


# === AV-16: FRAME_HANDLERS table (ADR-0021 R1, ADR-0080 §2.2) ==================
# Frame-handlers с одинаковой сигнатурой
# ``async (self, ws, session, payload) -> bool`` (плюс sid для voice_audio
# и ftype для supervisor).
# Возвращают ``False`` чтобы попросить ``_ws_handler`` закрыть сокет
# после отправки ответа (HELLO auth-fail, GOODBYE).
#
# Исключения из таблицы (особая семантика):
#   * HELLO     — может закрыть сокет на AUTH_FAIL.
#   * GOODBYE   — нормальное закрытие (code 1000).
#   * STATE_UPDATE — server→client only, приход от клиента = ERROR{BAD_PAYLOAD}.
#
# Эта таблица используется в ``_handle_frame`` (issue #2201, voice-vr 16).
# Сами frame-handler'ы (``_dispatch_*``) — тонкие адаптеры, которые парсят
# payload и зовут существующие бизнес-методы (``_on_subscribe``,
# ``_on_unsubscribe`` и т.д.). Сложная бизнес-логика остаётся в
# ``_on_json_cmd`` / ``_on_hello`` / etc. — out of scope этой карточки.
async def _dispatch_subscribe(self, ws, session, payload) -> bool:
    try:
        payload_obj = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, f"bad SUBSCRIBE json: {e}")
        return True
    await self._on_subscribe(ws, session, payload_obj)
    return True


async def _dispatch_unsubscribe(self, ws, session, payload) -> bool:
    try:
        payload_obj = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return True  # UNSUBSCRIBE с битым json — noop (как было в исходнике)
    await self._on_unsubscribe(ws, session, payload_obj)
    return True


async def _dispatch_json_event(self, ws, session, payload) -> bool:
    try:
        payload_obj = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return True  # JSON_EVENT с битым json — noop (как было в исходнике)
    await self._on_json_event(ws, session, payload_obj)
    return True


async def _dispatch_json_cmd(self, ws, session, payload) -> bool:
    try:
        payload_obj = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, f"bad JSON_CMD json: {e}")
        return True
    await self._on_json_cmd(ws, session, payload_obj)
    return True


async def _dispatch_voice_audio(self, ws, session, sid, payload) -> bool:
    # ADR-0071 step 5a: stream_id==2 → wake-канал (publish_quest_wake_audio),
    # иначе — PTT/radio (publish_voice_audio, текущее поведение).
    # Back-compat: stream_id==0 тоже идёт в radio-канал
    # (исторически клиенты слали sid=0).
    # issue #1992 observability: см. _note_voice_audio_rx — без этого приём
    # молчит одинаково что при живом потоке без подписчика, что при клиенте,
    # который вообще ничего не шлёт.
    self._note_voice_audio_rx(sid, len(payload), session.session_id)
    if sid == 2:
        self.bridge.publish_quest_wake_audio(payload)
    else:
        self.bridge.publish_voice_audio(payload)
    return True


async def _dispatch_supervisor(self, ws, session, ftype, payload) -> bool:
    # Supervisor-API (§3 + §11 + AV-16). Только v2-сессии; v1 присылает
    # 0x30..0x32 → ERROR{PROTOCOL_VERSION} (возвращается из самого
    # _handle_supervisor_command).
    await self._handle_supervisor_command(ws, session, ftype, payload)
    return True


async def _dispatch_hello(self, ws, session, payload) -> bool:
    try:
        payload_obj = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        await self._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, f"bad HELLO json: {e}")
        return True
    if not await self._on_hello(ws, session, payload_obj):
        # AUTH_FAIL → закрыть сокет после отправки ERROR.
        await ws.close(code=4001, message=b"auth_fail")
        return False
    return True


async def _dispatch_goodbye(self, ws) -> bool:
    await ws.close(code=1000, message=b"goodbye")
    return False


async def _dispatch_state_update(self, ws) -> bool:
    # Сервер-инициируемый frame; клиент НИКОГДА не должен слать
    # STATE_UPDATE → ERROR{BAD_PAYLOAD}.
    await self._send_error(
        ws, 0, ErrorCode.BAD_PAYLOAD, "STATE_UPDATE is server→client only (§3)"
    )
    return True


async def _dispatch_unknown(self, ws, ftype) -> bool:
    await self._send_error(
        ws, 0, ErrorCode.BAD_PAYLOAD, f"frame type {ftype} not supported"
    )
    return True


FRAME_HANDLERS: dict = {
    FrameType.SUBSCRIBE: _dispatch_subscribe,
    FrameType.UNSUBSCRIBE: _dispatch_unsubscribe,
    FrameType.JSON_EVENT: _dispatch_json_event,
    FrameType.JSON_CMD: _dispatch_json_cmd,
    FrameType.HELLO: _dispatch_hello,
    FrameType.GOODBYE: lambda self, ws, session, payload: _dispatch_goodbye(ws),
    FrameType.STATE_UPDATE: lambda self, ws, session, payload: _dispatch_state_update(
        ws
    ),
    FrameType.VOICE_AUDIO: lambda self, ws, session, sid, payload: _dispatch_voice_audio(
        self, ws, session, sid, payload
    ),
    FrameType.SET_MODE: lambda self, ws, session, payload: _dispatch_supervisor(
        self, ws, session, FrameType.SET_MODE, payload
    ),
    FrameType.ACQUIRE_FLOOR: lambda self, ws, session, payload: _dispatch_supervisor(
        self, ws, session, FrameType.ACQUIRE_FLOOR, payload
    ),
    FrameType.RELEASE_FLOOR: lambda self, ws, session, payload: _dispatch_supervisor(
        self, ws, session, FrameType.RELEASE_FLOOR, payload
    ),
}


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

# issue #1992 observability: до этой правки приём VOICE_AUDIO не логировался
# вовсе — «клиент не шлёт» и «мост не публикует» выглядели на сервере
# одинаково (тишина в обоих случаях). Первый пакет per stream_id — сразу
# INFO, дальше сводка раз в это окно (поток ~16 кГц / чанк 20мс = до 50
# пакетов/сек — без троттлинга лог захлебнётся).
VOICE_AUDIO_LOG_INTERVAL_S: float = 10.0


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


# Шаг 4б (t_80e7aa1e): дефолтный язык пайплайна грипа до первой
# синхронизации с панели. Должен совпадать с ``GRIP_DEFAULT_LANGUAGE``
# в supervisor_node.py:362 — иначе оптимистичная подсветка клиента
# разъедется с тем, что реально применилось.
VOICE_PIPELINE_DEFAULT_LANGUAGE: str = "ru"


def _validate_voice_pipeline_payload(
    llm_enabled: bool, preset: str, language: str
) -> Optional[str]:
    """Whitelist для voice_pipeline (тестируется без rclpy/WS).

    Возвращает ``None`` если payload валиден, иначе строку-причину для
    ``voice_pipeline_nack.reason``.

    Семантика «без стиля» = ``llm_enabled=False`` ИЛИ ``preset in
    {"", "none", "off"}`` — грип идёт в TTS дословно, 0 вызовов LLM
    (см. supervisor_node.classify_preset). Сервер не выдумывает
    экзотические комбинации: пустой preset при llm_enabled=True — это
    явный сигнал «не стилизуем» (id «none»), и он валиден.
    """
    if preset and preset not in VOICE_PRESET_IDS:
        return f"invalid_voice_pipeline_preset: {preset!r}"
    if language and language not in VOICE_LANGUAGES:
        return f"invalid_voice_pipeline_language: {language!r}"
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
        - ROS-подписок (lidar_2d, map_2d, robot_status, voice_state) — payload
          из protocol/topics.py и streams/*.
        - ROS-подписок на image_transport (camera_rear, camera_ceiling) —
          payload это JPEG bytes, форвардятся as-is.
        - capture-loop'ов CameraProvider (camera_oak_*) — payload это
          JPEG/H.264 bytes с камеры.
        """
        ...

    def available_streams(self) -> list[dict[str, Any]]:
        """JSON-payload для stream_select list cmd (Phase 2 R10)."""
        ...

    def publish_voice_barge_in(self) -> None:
        """PTT start: STOP в /voice/tts/control + /voice/sound/stop (barge-in)."""
        ...

    def publish_voice_audio(self, payload: bytes) -> None:
        """VOICE_AUDIO (stream_id=1, PTT): publish AudioData в /avatar/voice_in
        (int16 PCM 16 kHz)."""
        ...

    def publish_quest_wake_audio(self, payload: bytes) -> None:
        """VOICE_AUDIO (stream_id=2, wake-channel): always-on микрофон с
        client-side RMS VAD (ADR-0071 step 5а) → AudioData в /audio/quest_wake.

        ``payload`` — один кадр 20 мс / 640 байт, НЕ фраза. Реализация
        (QuestBridge, issue #2135) копит кадры и публикует ОДНУ AudioData на
        фразу, закрывая её по паузе в потоке (см. `core/wake_segmenter.py`):
        stt_node гоняет полный цикл распознавания на каждое сообщение, и на
        20 мс оно всегда возвращает пусто. Дальше stt_node маршрутизирует в
        /avatar/stt/result только при вейке «ТАРС» (целевая §7.1/§9.1).

        Тестовая реализация — NoOpBridge — остаётся no-op: unit-тесты на
        routing stream_id проверяют только то, что WS-сервер вызывает этот
        метод с правильным payload, без участия ROS-стека.
        """
        ...

    def reset_wake_audio(self) -> None:
        """WS-сессия закончилась → выбросить недособранную wake-фразу.

        issue #2135: сегментатор wake-канала (QuestBridge) держит буфер
        кадров между вызовами ``publish_quest_wake_audio``. Буфер принадлежит
        сессии: без сброса хвост фразы ушедшего оператора склеится с первыми
        кадрами следующей. NoOpBridge — no-op (буфера нет).
        """
        ...

    def set_wake_stream_state(self, active: bool) -> None:
        """JSON_CMD {cmd: voice_listen_start/stop} → синхронизировать
        серверный флаг wake-канала.

        Поведение реализации:
        - Запоминает ``_wake_active = active`` (для diagnostic/observability).
        - Опционально публикует latched-топик ``/avatar/wake_stream{state}``
          для дашборда и e2e-наблюдателей (ADR-0071 §2.4).

        Контракт идемпотентен: повторный start при уже активном — no-op.
        """
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

    def publish_voice_pipeline(
        self, llm_enabled: bool, preset: str, language: str
    ) -> None:
        """Шаг 4б (issue #1989): опубликовать конфиг пайплайна грипа в /avatar/voice_pipeline.

        Отдельный канал от AV-28 set_voice_preset/language: те меняют
        voice_preset на dialogue_node (личность), а этот меняет _pipeline_*
        на СУПЕРВИЗОРЕ (грип-трансформация, см. supervisor_node.py:2634).
        Если ``llm_enabled=False`` или ``preset=""`` — грип произносит
        дословно (0 вызовов LLM). Сервер уже провалидировал whitelist (см.
        ``_validate_voice_pipeline_payload``); здесь — только публикация.
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

    def publish_quest_wake_audio(self, payload: bytes) -> None:
        # NoOpBridge: ADR-0071 step 5a, stream_id=2 → wake-канал.
        # Реальная маршрутизация в /audio/quest_wake → stt_node реализована
        # в QuestBridge (quest_node.py, issue #1992). NoOpBridge — тестовый
        # ROS-free double, остаётся no-op намеренно.
        return None

    def set_wake_stream_state(self, active: bool) -> None:
        # NoOpBridge: фиксируем в логе для unit-тестов.
        log.debug("NoOpBridge: set_wake_stream_state active=%s", active)
        return None

    def reset_wake_audio(self) -> None:
        # NoOpBridge: буфера wake-фразы нет (сегментация живёт в QuestBridge,
        # issue #2135) — сбрасывать нечего.
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

    # ── Шаг 4б grip-pipeline config (t_80e7aa1e) ────────────────────────
    # Симметрично ``set_voice_preset``/``set_voice_language``: в Protocol
    # описана как ``publish_voice_pipeline``. NoOpBridge нужен, чтобы WS-тесты
    # без ROS (test_ws_server_voice.py и др.) видели NoOpBridge-совместимый
    # контракт — иначе isinstance(bridge, Bridge) падает и unit-тесты роняются.
    def publish_voice_pipeline(
        self, llm_enabled: bool, preset: str, language: str
    ) -> None:
        # NoOpBridge: ничего не публикуем, только лог для отладки теста.
        log.debug(
            "NoOpBridge: publish_voice_pipeline llm_enabled=%s preset=%r language=%r",
            llm_enabled,
            preset,
            language,
        )
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
# list_voices ≤ 1/10s, set_voice ≤ 1/2s, preview_voice ≤ 1/5s + ≤3 параллельных;
# AV-28 (стиль/язык) — свой слот set_voice_style ≤ 1/0.5s.
# Реализуется через in-memory last-ts per ws (не per session) — соединение
# одно, но политика прибита к клиенту.
VOICE_LIST_MIN_INTERVAL_S: float = 10.0
VOICE_SET_MIN_INTERVAL_S: float = 2.0
VOICE_PREVIEW_MIN_INTERVAL_S: float = 5.0
VOICE_PREVIEW_MAX_CONCURRENT: int = 3

# ADR-0055 / issue #1993 — два стрима аудио-WS-канала делят один приватный
# канал сессии, но считают слоты НЕЗАВИСИМО (preview и operator_tts). Whitelist
# в deliver_audio() / register_audio_session() статичный — расширение = правка
# тут, ADR и ws_server-тестов; «широким швом» канал делать нельзя.
_AUDIO_STREAMS: frozenset[str] = frozenset({"preview", "operator_tts"})

# Сколько секунд считать request_id «зависшим» (для чистки реестра).
# Один и тот же потолок для preview и operator_tts — на Quest один оператор.
_AUDIO_PENDING_STALE_S: float = 60.0

# AV-28 (стиль речи + язык вывода) считает СВОЙ слот, а не делит слот с
# AV-27. Раньше слот был общий, и это ломало обычную работу оператора:
# выбрал стиль в панели пайплайна → через секунду выбрал язык (или
# применил голос в picker'е) → второй запрос молча падал в rate-limit, а
# UI уже показывал новое значение. Два клика подряд — это не флуд, это
# нормальный сценарий; флуд по-прежнему режется, но по каждой фиче
# отдельно. Интервал меньше: AV-28 — это SetParameters на dialogue_node,
# без синтеза и без похода к TTS-провайдеру.
VOICE_STYLE_MIN_INTERVAL_S: float = 0.5


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
        avatar_arbiter: Optional[LocalAvatarArbiterClient] = None,
        floor_cache: Optional[AvatarStateFloorCache] = None,
        voice_cache: Optional[VoiceFloorCache] = None,
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
        # ADR-0055 / issue #1993: общий per-stream реестр активных audio-запросов.
        # Внутри: stream → {request_id: (ws, opened_at)}. Два стрима
        # (``preview`` и ``operator_tts``) считают слоты НЕЗАВИСИМО — голос
        # оператора в шлем не должен конкурировать с превью в picker'е.
        # request_id → ws — отдаём клиенту ТОЛЬКО если ws ещё живой; иначе
        # дропаем ответ. lock — потому что ROS callback'и зовут deliver_* из
        # другого потока, а cmd-handler — из aiohttp-loop.
        self._audio_pending: dict[str, dict[str, tuple[Any, float]]] = {
            stream: {} for stream in _AUDIO_STREAMS
        }
        self._voice_state_lock = threading.Lock()
        # ws_id(id(ws)) → last-ts (монотонный) per cmd для rate-limit.
        # key = id(ws) (а не сам ws, потому что ws не hashable).
        self._last_voice_cmd_ts: dict[int, dict[str, float]] = {}
        # ADR-0051 §2.2 (issue #1999, C2): avatar_arbiter service client
        # (локальный stub на Phase 1) + два read-only кэша для UI/логов.
        # Tracker-объекты (``SupervisorFloorTracker`` / ``VoiceFloor``)
        # удалены — вместо них ``LocalAvatarArbiterClient`` (mutex + cache
        # push) и ``AvatarStateFloorCache`` / ``VoiceFloorCache`` (read-only
        # mirror из /avatar/state). Когда avatar_arbiter service будет
        # подключён через ROS 2 client, заменится только avatar_arbiter —
        # ws_server код не поменяется.
        self._floor_cache: AvatarStateFloorCache = (
            floor_cache if floor_cache is not None else AvatarStateFloorCache()
        )
        self._voice_cache: VoiceFloorCache = (
            voice_cache if voice_cache is not None else VoiceFloorCache()
        )
        if avatar_arbiter is not None:
            self._avatar_arbiter: LocalAvatarArbiterClient = avatar_arbiter
        else:
            self._avatar_arbiter = LocalAvatarArbiterClient(
                cache=self._floor_cache,
            )
        # AV-19: gate teleop_floor (Phase 1 — локальный tracker; Phase 2 —
        # proxy на avatar_supervisor service, ADR-0028 §4.4). После
        # #1999 / C2 — через LocalAvatarArbiterClient.
        self._require_teleop_floor: bool = require_teleop_floor
        # Backward-compat alias для ws_server кода, который исторически
        # обращался к ``_floor_tracker`` / ``_voice_floor``. Эти алиасы
        # указывают на один и тот же объект (avatar_arbiter client) —
        # упрощает миграцию call-sites без семантики-потери. После
        # #1999 follow-up будут удалены.
        self._floor_tracker = self._avatar_arbiter
        self._voice_floor = self._avatar_arbiter
        # session_id → True если ws_server уже сообщил
        # клиенту о FLOOR_HELD-error в текущем окне rate-limit. Нужно,
        # чтобы при HOLD-окне не слать ERROR повторно (rate-limit — на
        # уровне avatar_arbiter клиента, но здесь дополнительно дедуплим,
        # чтобы один клиент не получал > 1 ошибки в FLOOR_HELD_RATE_LIMIT_S
        # даже если в ws_server прилетают разные teleop_twist-ы).
        self._floor_held_warned_session: dict[str, float] = {}
        # issue #1992 observability: счётчики приёма VOICE_AUDIO по
        # stream_id (1=ptt/radio, 2=wake). См. _note_voice_audio_rx().
        self._voice_audio_rx_count: dict[int, int] = {}
        self._voice_audio_rx_bytes: dict[int, int] = {}
        self._voice_audio_rx_window_count: dict[int, int] = {}
        self._voice_audio_rx_window_bytes: dict[int, int] = {}
        self._voice_audio_rx_last_log_ts: dict[int, float] = {}

    def get_active_sessions(self) -> int:
        return sum(1 for s in self._sessions.values() if s.is_open())

    def set_send_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Установить aiohttp-loop для потокобезопасной отправки кадров."""
        self._send_loop = loop

    def _note_voice_audio_rx(self, sid: int, nbytes: int, session_id: str) -> None:
        """issue #1992 observability — приём VOICE_AUDIO по ``stream_id``.

        До этой правки приём кадра ничем не логировался: если шлем не
        шлёт wake-канал (``stream_id=2``), в логах моста ровно та же
        тишина, что и при живом потоке, который просто некуда
        публиковать. Считает пакеты/байты per stream_id и логирует:
        первый принятый пакет — сразу (подтверждает, что WS вообще что-то
        получил), дальше — сводка не чаще раза в
        :data:`VOICE_AUDIO_LOG_INTERVAL_S` секунд, чтобы не забить лог на
        потоке ~16 кГц (чанк 20 мс -> до 50 пакетов/сек).
        """
        now = time.monotonic()
        count = self._voice_audio_rx_count.get(sid, 0) + 1
        self._voice_audio_rx_count[sid] = count
        self._voice_audio_rx_bytes[sid] = self._voice_audio_rx_bytes.get(sid, 0) + nbytes
        window_count = self._voice_audio_rx_window_count.get(sid, 0) + 1
        self._voice_audio_rx_window_count[sid] = window_count
        window_bytes = self._voice_audio_rx_window_bytes.get(sid, 0) + nbytes
        self._voice_audio_rx_window_bytes[sid] = window_bytes

        last_log = self._voice_audio_rx_last_log_ts.get(sid)
        if last_log is None:
            self._voice_audio_rx_last_log_ts[sid] = now
            self._voice_audio_rx_window_count[sid] = 0
            self._voice_audio_rx_window_bytes[sid] = 0
            log.info(
                "VOICE_AUDIO stream_id=%d first packet received "
                "(session=%s, %d bytes)",
                sid,
                session_id,
                nbytes,
            )
            return

        elapsed = now - last_log
        if elapsed >= VOICE_AUDIO_LOG_INTERVAL_S:
            log.info(
                "VOICE_AUDIO stream_id=%d rx summary: %d packets / %d bytes "
                "in %.1fs (session=%s, total %d packets)",
                sid,
                window_count,
                window_bytes,
                elapsed,
                session_id,
                count,
            )
            self._voice_audio_rx_last_log_ts[sid] = now
            self._voice_audio_rx_window_count[sid] = 0
            self._voice_audio_rx_window_bytes[sid] = 0

    # ── AV-27 / issue #1919 — TTS picker helpers ────────────────────────

    def _voice_rate_limit_check(self, ws: Any, cmd: str, min_interval_s: float) -> bool:
        """True если cmd разрешён (лимит не превышен), иначе False.

        Логирует и решает, что делать с отказом, ВЫЗЫВАЮЩИЙ: у каждой
        команды свой честный ответ (voice_set_nack / ответ из кэша), а
        молчание — не ответ (ADR-0018).
        """
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
        """Тонкая обёртка: зарегистрировать request_id → ws для preview.

        Реализация — общий ``register_audio_session(stream="preview", ...)``.
        Returns False если уже есть ``VOICE_PREVIEW_MAX_CONCURRENT``
        активных preview-request_id'ов — клиент получит
        ``preview_voice_error{reason: "too_many"}``.
        """
        return self.register_audio_session("preview", request_id, ws)

    def register_audio_session(
        self, stream: str, request_id: str, ws: Any
    ) -> bool:
        """Зарегистрировать request_id → ws для ``stream``.

        Per-stream лимит (``VOICE_PREVIEW_MAX_CONCURRENT`` для каждого
        стрима — на Quest один оператор, стримы не делят слот). Неизвестный
        stream → логируем WARNING и возвращаем ``False`` (защита от
        «широкого шва» — канал не должен стать open-endpoint'ом).

        Returns False если для этого stream уже лимит активных request_id'ов
        — supervisor/quest_node увидит ``/voice/tts/finished{success=False}``
        и реплика ТАРС честно теряется (без спама retry).
        """
        if stream not in _AUDIO_STREAMS:
            log.warning(
                "ws_server.register_audio_session: unknown stream=%r "
                    "(allowed=%s)",
                stream,
                sorted(_AUDIO_STREAMS),
            )
            return False
        with self._voice_state_lock:
            per_stream = self._audio_pending.get(stream)
            if per_stream is None:
                # Stream не из числа whitelisted — допустимо только если
                # реестр ещё не инициализирован в __init__ (защита от
                # гонки на старте; на практике всегда присутствует).
                log.warning(
                    "ws_server.register_audio_session: stream=%r "
                        "отсутствует в _audio_pending",
                    stream,
                )
                return False
            # Чистим зависшие (старше _AUDIO_PENDING_STALE_S) — на случай
            # если supervisor упал. Per-stream чистка (preview/operator_tts
            # не делят слот, но «зависшие» друг друга не касаются).
            now = time.monotonic()
            stale = [
                k for k, (_, t) in per_stream.items()
                if now - t > _AUDIO_PENDING_STALE_S
            ]
            for k in stale:
                per_stream.pop(k, None)
            if len(per_stream) >= VOICE_PREVIEW_MAX_CONCURRENT:
                return False
            per_stream[request_id] = (ws, now)
        return True

    def deliver_audio(
        self,
        *,
        stream: str,
        request_id: str,
        audio_bytes: bytes,
        audio_format: str,
        content_type: str,
        seq: int,
        total: int,
        sample_rate: Optional[int] = None,
        ws: Optional[Any] = None,
    ) -> bool:
        """Обобщённая доставка аудио в WS клиента (ADR-0055, issue #1993).

        Контракт сообщения:
          * ``meta["type"]`` — ``preview_voice_audio`` для ``stream="preview"``,
            ``operator_tts_audio`` для ``stream="operator_tts"``.
          * ``meta`` шлётся через ``_schedule_ws_send`` (JSON_EVENT),
            аудио-байты — через ``_schedule_ws_send_binary`` (BINARY_FRAME,
            stream_id=0).
          * Unknown stream → log.warning + ``False``, без побочных эффектов.
          * ADR-0078 §3.1: для ``stream="operator_tts"`` в meta добавляется
            поле ``sample_rate`` (int, Гц) — обязательное для ручной
            сборки ``AudioBuffer`` из int16-LE PCM на стороне клиента.
            Для ``stream="preview"`` поле НЕ добавляется (preview использует
            ``decodeAudioData`` и частоту не передаёт).

        Args:
            stream: один из ``_AUDIO_STREAMS``. Иначе — дроп.
            request_id: должен быть зарегистрирован через
                ``register_audio_session(stream, request_id, ws)`` или
                ``start_preview_session(request_id, ws)`` ДО ``deliver_audio``.
                ``ws`` можно передать явно (для path ``operator_tts``: quest_node
                сам достаёт ws из реестра по сессии). Если ``ws`` не передан —
                достаём из ``_audio_pending[stream][request_id]``.

        Returns:
            True если request_id был зарегистрирован и ws ещё живой;
            False если stream неизвестен, request_id не зарегистрирован или
            ws закрыт. В False-ветке реестр чистится от зависшего request_id.
        """
        if stream not in _AUDIO_STREAMS:
            log.warning(
                "ws_server.deliver_audio: unknown stream=%r (allowed=%s)",
                stream,
                sorted(_AUDIO_STREAMS),
            )
            return False
        with self._voice_state_lock:
            per_stream = self._audio_pending.get(stream)
            if per_stream is None:
                return False
            entry = per_stream.get(request_id)
            if entry is None:
                return False
            ws_resolved = ws if ws is not None else entry[0]
        if ws_resolved is None or getattr(ws_resolved, "closed", False):
            with self._voice_state_lock:
                per_stream = self._audio_pending.get(stream)
                if per_stream is not None:
                    per_stream.pop(request_id, None)
            return False
        ts_ms = int(time.time() * 1000)
        meta = {
            "type": "preview_voice_audio" if stream == "preview" else "operator_tts_audio",
            "request_id": request_id,
            "format": audio_format,
            "content_type": content_type,
            "seq": seq,
            "total": total,
            "ts_ms": ts_ms,
        }
        # ADR-0078 §3.1: sample_rate только для operator_tts (preview не
        # использует — там decodeAudioData и частота вшита в контейнер).
        if stream == "operator_tts" and sample_rate is not None:
            meta["sample_rate"] = int(sample_rate)
        self._schedule_ws_send(ws_resolved, meta)
        if audio_bytes:
            self._schedule_ws_send_binary(ws_resolved, audio_bytes)
        return True

    def deliver_preview_audio(
        self,
        request_id: str,
        audio_bytes: bytes,
        audio_format: str,
        content_type: str,
        seq: int,
        total: int,
        sample_rate: Optional[int] = None,
    ) -> bool:
        """Тонкая обёртка для AV-27 preview-канала (обратная совместимость).

        Реализация — ``deliver_audio(stream="preview", ...)``. Сигнатура
        и поведение НЕ меняются (тесты AV-19/AV-27 остаются зелёными без
        правок); ADR-0055 ввёл обобщённый канал, а preview — первый стрим
        на нём.

        ``sample_rate`` принят для симметрии сигнатуры, но НЕ попадает в
        meta preview'а (ADR-0078 §3.1: для preview частота вшита в
        контейнер mp3/wav/opus, ``decodeAudioData`` сам знает).
        """
        return self.deliver_audio(
            stream="preview",
            request_id=request_id,
            audio_bytes=audio_bytes,
            audio_format=audio_format,
            content_type=content_type,
            seq=seq,
            total=total,
            sample_rate=sample_rate,
        )

    def deliver_preview_done(self, request_id: str) -> bool:
        """Финальный preview_voice_done → клиенту. Чистит preview-pending.

        Тонкая обёртка над ``_send_audio_done("preview", "preview_voice_done")``.
        """
        return self._send_audio_done("preview", "preview_voice_done", request_id)

    def deliver_preview_error(self, request_id: str, reason: str) -> bool:
        """Ошибка preview → preview_voice_error. Чистит preview-pending.

        Тонкая обёртка над ``_send_audio_error("preview", ...)``.
        """
        return self._send_audio_error(
            "preview", "preview_voice_error", request_id, reason
        )

    def deliver_operator_tts_done(self, request_id: str) -> bool:
        """Финал оператор-TTS реплики → operator_tts_done. Чистит pending.

        ADR-0078 §3.1: клиент играет каждый чанк сразу по приходу байт
        (очередь), поэтому ``done`` сейчас НЕ публикуется сервером в норме
        (нет финального синхронизирующего маркера). Метод оставлен для
        forward-compat: явный flush/сброс (например, при supervised shutdown,
        supervisor-level error или в e2e-тестах).
        """
        return self._send_audio_done(
            "operator_tts", "operator_tts_done", request_id
        )

    def deliver_operator_tts_error(self, request_id: str, reason: str) -> bool:
        """Ошибка оператор-TTS → operator_tts_error. Чистит pending.

        ADR-0078 §3.1: симметрично ``deliver_preview_error``, для flush
        при error-path (supervisor не смог синтезировать, ws закрылся,
        и т.п.).
        """
        return self._send_audio_error(
            "operator_tts", "operator_tts_error", request_id, reason
        )

    def _send_audio_done(
        self, stream: str, type_name: str, request_id: str
    ) -> bool:
        """Обобщённый «done»: достаём ws из ``_audio_pending[stream]``,
        шлём ``{type, request_id, ts_ms}`` и чистим pending.

        Для preview (``type="preview_voice_done"``) — финал синтеза.
        Для operator_tts (``type="operator_tts_done"``, ADR-0055 — пока
        не шлётся сервером, но метод готов для follow-up) — финал
        реплики ТАРС в шлем. Неизвестный stream → False.
        """
        if stream not in _AUDIO_STREAMS:
            log.warning(
                "ws_server._send_audio_done: unknown stream=%r", stream
            )
            return False
        with self._voice_state_lock:
            per_stream = self._audio_pending.get(stream)
            if per_stream is None:
                return False
            entry = per_stream.pop(request_id, None)
            if entry is None:
                return False
            ws, _ = entry
        if ws is None or getattr(ws, "closed", False):
            return False
        body = {
            "type": type_name,
            "request_id": request_id,
            "ts_ms": int(time.time() * 1000),
        }
        self._schedule_ws_send(ws, body)
        return True

    def _send_audio_error(
        self, stream: str, type_name: str, request_id: str, reason: str
    ) -> bool:
        """Обобщённый «error»: достаём ws из ``_audio_pending[stream]``,
        шлём ``{type, request_id, reason, ts_ms}`` и чистим pending.

        Используется и для ``preview_voice_error`` (синтез preview'а упал),
        и для ``operator_tts_error`` (синтез реплики ТАРС упал;
        ADR-0055 — пока не шлётся сервером, но метод готов).
        """
        if stream not in _AUDIO_STREAMS:
            log.warning(
                "ws_server._send_audio_error: unknown stream=%r", stream
            )
            return False
        with self._voice_state_lock:
            per_stream = self._audio_pending.get(stream)
            if per_stream is None:
                return False
            entry = per_stream.pop(request_id, None)
            if entry is None:
                return False
            ws, _ = entry
        if ws is None or getattr(ws, "closed", False):
            return False
        body = {
            "type": type_name,
            "request_id": request_id,
            "reason": reason,
            "ts_ms": int(time.time() * 1000),
        }
        self._schedule_ws_send(ws, body)
        return True

    # Backward-compat alias для тестов AV-19/AV-27, которые читают
    # ``server._preview_pending`` напрямую. По завершении тестов можно
    # удалить (коммит-фикс).
    @property
    def _preview_pending(self) -> dict[str, tuple[Any, float]]:
        """Back-compat: ``server._preview_pending`` → ``_audio_pending["preview"]``.

        Удалить после миграции существующих тестов на ``_audio_pending``.
        """
        return self._audio_pending.get("preview", {})

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
            log.debug(
                "ws_server._schedule_ws_send_binary: no loop; payload (%d bytes) dropped",
                len(payload),
            )
            return
        try:
            loop.call_soon_threadsafe(self._send_binary_async, ws, payload)
        except RuntimeError as exc:  # noqa: BLE001
            log.debug("ws_server._schedule_ws_send_binary failed: %s", exc)
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
        return self._avatar_arbiter.should_send_floor_held_error(session_id)

    def _find_session_by_client_id(
        self, client_id: str
    ) -> "Optional[ClientSession]":
        """Найти активную :class:`ClientSession` по server_client_id.

        ``_sessions`` keyed by ``session_id`` (UUID), а внешний API
        (avatar_arbiter, supervisor_*, JSON_EVENT клиенту) оперирует
        ``server_client_id`` (issue #2190, ``"quest:<uuid>"``).
        Линейный поиск по всем сессиям — их обычно < 5, поэтому
        overhead не критичен (gate и heartbeat идут по
        ``_avatar_arbiter.floor_holder`` напрямую, без поиска).
        """
        if not client_id:
            return None
        for sess in self._sessions.values():
            if sess.server_client_id == client_id:
                return sess
        return None

    def update_floor_cache(
        self, snapshot: "AvatarFloorSnapshot"
    ) -> "FloorViewUpdate":
        """Обновить :py:attr:`AvatarStateFloorCache` из /avatar/state.

        Issue #2190 (voice-vr 05): «живой путь floor_lost». Раньше
        кеш жил только через ``LocalAvatarArbiterClient._push_to_cache``
        (мутации из try_acquire/release) — внешний ``/avatar/state``
        из avatar_supervisor никак не доходил до ws_server. Теперь
        QuestBridge.on_avatar_state парсит msgpack → ``AvatarFloorSnapshot``
        и зовёт этот метод; он же возвращает diff (``FloorViewUpdate``)
        — на его основании QuestBridge шлёт ``JSON_EVENT{floor_lost}``
        бывшему держателю, если avatar_supervisor перехватил/освободил
        floor.

        Метод вызывается из aiohttp-loop (см. ``_dispatch_state_update``),
        поэтому single-thread инвариант кеша соблюдён.
        """
        return self._floor_cache.update(snapshot)

    def notify_floor_lost_external(self, client_id: str, reason: str) -> None:
        """Шлёт ``JSON_EVENT{floor_lost}`` в сокет указанного client_id.

        Issue #2190: ``QuestBridge.on_avatar_state`` зовёт этот метод,
        когда diff из ``AvatarStateFloorCache.update`` показал, что
        бывший держатель floor-а (server_client_id) был одной из
        наших Quest-сессий. avatar_supervisor (внешний) перехватил
        или освободил floor — клиент должен DISARM-нуть и показать тост.

        ``client_id`` — server_client_id (``"quest:<uuid>"``).
        Метод НЕ трогает ``_avatar_arbiter.floor_holder`` —
        avatar_supervisor остаётся источником истины; ws_server только
        оповещает UI клиента.

        Если у клиента ещё нет активной WS-сессии (например, отключился
        пока avatar_supervisor перехватывал) — метод no-op. Это
        безопасно, потому что кеш уже обновлён и при следующем
        ``subscribe``/переподключении клиент увидит правильный state.
        """
        if not client_id:
            return
        session = self._find_session_by_client_id(client_id)
        if session is None or not session.is_open():
            return
        ws = self._ws_by_session.get(session.session_id)
        if ws is None:
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return
        if loop is None:
            return

        async def _notify() -> None:
            try:
                payload = json.dumps(
                    {
                        "type": "floor_lost",
                        "floor": "teleop",
                        "reason": reason,
                        "ts_ms": int(time.time() * 1000),
                    },
                    separators=(",", ":"),
                ).encode("utf-8")
                await ws.send_bytes(encode_frame(FrameType.JSON_EVENT, 0, payload))
            except Exception as exc:  # noqa: BLE001
                log.warning("quest: floor_lost notify failed: %s", exc)

        fut = asyncio.run_coroutine_threadsafe(_notify(), loop)
        fut.add_done_callback(_consume_future_exception)
        log.info("quest: floor_lost notify client_id=%s reason=%s", client_id, reason)

    def on_floor_lost_external(self, client_id: str) -> None:
        """Внешнее уведомление (от Bridge/avatar_supervisor) о потере floor.

        В Phase 2, когда avatar_supervisor будет публиковать
        ``/avatar/state`` с ``teleop_floor != client_id``, эта точка
        будет вызываться из подписки. Сейчас используется из
        QuestBridge через Bridge.on_floor_lost (см. _unregister_session
        для случая закрытия сессии).

        ``client_id`` — server_client_id формата ``"quest:<session_uuid>"``
        (issue #2190). Это совпадает с тем, что avatar_arbiter публикует
        в ``/avatar/state.teleop_floor.client_id``, и с тем, что ws_server
        хранит в ``_avatar_arbiter.floor_holder``. Поэтому сравнение
        ``floor_holder != client_id`` корректно без дополнительной
        конвертации.

        Внутри:
        1) Сбрасываем локальный tracker (если ещё держит — значит
           состояния разошлись, но fail-safe приоритетнее).
        2) Уведомляем Bridge — он опубликует Twist(0,0) в cmd_vel_quest.
        3) Шлём клиенту JSON_EVENT{type:"floor_lost"} с held_by=None —
           клиент DISARM-ит и показывает тост (teleop_fsm).
        """
        # ADR-0051 §2.2 (issue #1999, C2): проверяем avatar_arbiter
        # (источник истины по floor-ам) напрямую. Раньше это был
        # tracker.is_held_by — теперь его роль исполняет клиент.
        # issue #2190: оба аргумента в формате server_client_id, никакой
        # конвертации не нужно.
        if self._avatar_arbiter.floor_holder != client_id:
            # avatar_arbiter уже не считает client_id держателем —
            # likely двойное уведомление (release в _unregister_session
            # + supervisor подтвердил). Ничего не делаем.
            return
        self._avatar_arbiter.force_release_floor()
        self._avatar_arbiter.reset_floor_held_rate_limit(client_id)
        self._floor_held_warned_session.pop(client_id, None)
        # Уведомить Bridge (QuestBridge опубликует zero Twist).
        self.bridge.on_floor_lost(client_id)
        # Уведомить активные WS-сессии с этим client_id, если ещё открыты.
        # issue #2190: client_id — server_client_id, а ``_sessions``
        # keyed by session_id. Ищем сессию через ``_find_session_by_client_id``.
        session = self._find_session_by_client_id(client_id)
        if session is None or not session.is_open():
            return
        ws = self._ws_by_session.get(session.session_id)
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
        log.info("quest: floor_lost client_id=%s (external)", client_id)

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
        # issue #2135: буфер wake-фразы принадлежит сессии. Оборвался WS
        # (disconnect/watchdog/GOODBYE) — недособранную фразу выбрасываем,
        # иначе она склеится с речью следующего оператора.
        self.bridge.reset_wake_audio()
        # Освободить voice floor, если эта сессия его держала
        # (watchdog/GOODBYE/disconnect → без явного voice_ptt_stop).
        # ADR-0051 §2.2: avatar_arbiter.release_voice() — теперь
        # единственная точка освобождения voice_floor.
        if self._avatar_arbiter.release_voice(session.session_id):
            # mirror в кэше: UI-подписчики должны сразу увидеть IDLE.
            self._voice_cache.update(FloorState.IDLE, None)
            log.info(
                "quest: voice floor released by disconnect session=%s",
                session.session_id,
            )
        # AV-19: освободить teleop_floor, если эта сессия его держала.
        # Это симметрично try_acquire_floor в _on_hello. В Phase 2
        # этот код пойдёт через avatar_supervisor service — здесь будет
        # release_floor service-call.
        was_held = self._avatar_arbiter.release_floor(session.session_id)
        if was_held:
            self._avatar_arbiter.reset_floor_held_rate_limit(session.session_id)
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
        # ADR-0051 §2.2 (issue #1999, C2): вместо локального tracker-а
        # — ``avatar_arbiter.try_acquire_floor()``. В Phase 2 тут будет
        # service-call к ``/avatar_arbiter/acquire_floor`` (avatar_arbiter
        # клиент уже сейчас подменяется ``LocalAvatarArbiterClient``,
        # заменяется на ROS 2 клиент без правок ws_server). Сейчас —
        # best-effort: если floor уже занят другой сессией — мы НЕ
        # отказываем в WELCOME (это убьёт UX при попытке Telegram-op
        # перехватить), но помечаем сессию «не держит» — teleop_twist
        # гейт не пройдёт и шлёт ERROR{FLOOR_HELD} rate-limited
        # (см. _on_json_cmd).
        acquire = self._avatar_arbiter.try_acquire_floor(session.session_id)
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
            # ADR-0051 §2.2: voice floor живёт в avatar_arbiter; здесь —
            # только snapshot из read-only кэша (mirror /avatar/state).
            voice_state_str = self._voice_cache.state.value
            voice_holder = self._voice_cache.holder
            await self._send_voice_state(
                ws,
                state=voice_state_str,
                ts_ms=int(time.time() * 1000),
                holder_id=(voice_holder.label() if voice_holder is not None else None),
            )

    async def _on_json_cmd(
        self,
        ws,
        session: ClientSession,
        payload_obj: dict[str, Any],
    ) -> None:
        """Dispatch a JSON_CMD through the module-level handler table.

        Рефакторинг voice-vr 10 (issue #2195): бывшая разросшаяся
        if/elif-цепочка (~200 строк) сведена к module-level
        ``_json_cmd_*`` функциям, зарегистрированным в
        :data:`JSON_CMD_HANDLERS` (см. конец файла). Здесь только
        терминальный dispatcher: lookup → handler, иначе явный отказ
        ERROR{UNKNOWN_COMMAND} вместо молчаливого drop.
        """
        cmd = payload_obj.get("cmd")
        handler = JSON_CMD_HANDLERS.get(cmd)
        if handler is None:
            log.warning(
                "quest: unknown JSON_CMD received: cmd=%r session_id=%s "
                "(see issue #2195)",
                cmd,
                session.session_id,
            )
            await self._send_error(
                ws,
                0,
                ErrorCode.UNKNOWN_COMMAND,
                f"unknown JSON_CMD: {cmd!r}",
            )
            return
        await handler(self, ws, session, payload_obj)

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
        """aiohttp WebSocket handler (thin orchestrator, ADR-0021 R1).

        Декомпозиция (issue #2201, voice-vr 16):
            * ``_handle_handshake``     — WS-upgrade + subprotocol + register
            * ``_spawn_session_tasks``  — heartbeat / watchdog / state_update
            * ``_handle_frame``         — dispatch по FrameType через FRAME_HANDLERS
              + локальный _dispatch для особых ftype (HELLO/GOODBYE/STATE_UPDATE
              / supervisor / unknown).

        Frame-pump перенесён в ``_handle_frame`` (вводит дополнительный await
        между receive-loop и finally). Тест
        ``test_voice_floor_disconnect_releases_floor`` подтверждает, что
        семантика сохранена: cancel/await порядок в finally остался как
        в исходнике (все cancel'ятся синхронно до ``await task``), что
        даёт ``_unregister_session`` шанс отработать сразу после close.
        """
        ws, session = await self._handle_handshake(request)
        heartbeat_task, watchdog_task, state_update_task = self._spawn_session_tasks(
            ws, session
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
                # Исключения из frame-loop (issue #2099/#2100): один кривой
                # КАДР (баг конкретного cmd-хендлера) НЕ должен убивать всю
                # WS-сессию.
                try:
                    should_close = await self._handle_frame(
                        ws, session, ftype, sid, payload
                    )
                except asyncio.CancelledError:
                    raise
                except Exception as e:  # noqa: BLE001
                    log.exception(
                        "quest: frame handler crashed (ftype=%s): %s", ftype, e
                    )
                    try:
                        await self._send_error(ws, 0, ErrorCode.INTERNAL, str(e))
                    except Exception:  # noqa: BLE001
                        log.debug("quest: _send_error after handler crash failed")
                    continue
                if should_close is False:
                    return ws
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001
            log.exception("quest: ws_handler crashed: %s", e)
            try:
                await self._send_error(ws, 0, ErrorCode.INTERNAL, str(e))
            except Exception:  # noqa: BLE001
                pass
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

    async def _handle_frame(
        self, ws, session: ClientSession, ftype: FrameType, sid: int, payload: bytes
    ) -> Optional[bool]:
        """Диспатчер одного фрейма. Возвращает ``False`` чтобы закрыть сокет.

        Использует таблицу ``FRAME_HANDLERS`` для ВСЕХ известных ftype
        (CC=1, dict lookup). Неизвестный ftype → ``_dispatch_unknown``.
        """
        handler = FRAME_HANDLERS.get(ftype)
        if handler is not None:
            if ftype == FrameType.VOICE_AUDIO:
                return await handler(self, ws, session, sid, payload)
            return await handler(self, ws, session, payload)
        return await _dispatch_unknown(self, ws, ftype)

    async def _handle_handshake(self, request):
        """WS-upgrade + subprotocol negotiation + register session.

        Возвращает пару ``(ws, session)`` готовую к frame-pump'у:
            * ``ws``      — aiohttp WebSocketResponse после ``ws.prepare()``
            * ``session`` — ClientSession c выставленным ``protocol_version``
              и clock-baseline через ``feed_heartbeat()``.
        """
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
        return ws, session

    def _spawn_session_tasks(
        self, ws, session: ClientSession
    ) -> tuple[asyncio.Task, asyncio.Task, asyncio.Task]:
        """Стартует 3 фоновых task'а для сессии. Возвращает кортеж для cancel'а.

        Фоновые task'и:
            * ``_heartbeat_loop``             — клиент-keepalive каждые 200 мс
            * ``_watchdog_loop``              — kill-switch при долгом бездействии
            * ``_state_update_keepalive_loop`` — STATE_UPDATE 1Hz только для v2
        """
        heartbeat_task = asyncio.create_task(self._heartbeat_loop(ws, session))
        watchdog_task = asyncio.create_task(self._watchdog_loop(ws, session))
        # STATE_UPDATE keep-alive 1 Hz (только для v2-сессий; для v1 — no-op).
        state_update_task = asyncio.create_task(
            self._state_update_keepalive_loop(ws, session)
        )
        return heartbeat_task, watchdog_task, state_update_task

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
    #
    # Декомпозиция (voice-vr 17, t_af606d97, ADR-0021 R1):
    #  * 4 pre-guard'а (protocol_version, auth, msgpack unpack, client_id
    #    normalization) вынесены в _prepare_supervisor_dispatch (CC≤15).
    #  * Per-frame логика — module-level handler-ы (_handle_set_mode /
    #    _handle_acquire_floor / _handle_release_floor / _handle_set_voice)
    #    с сигнатурой ``async (server, ws, session, data, client_id) -> None``.
    #  * Общий snapshot-отправитель вынесен в _send_supervisor_state_update
    #    (раньше была копипаста в SET_MODE success и FLOOR success).
    #  * Диспетчер табличный (SUPERVISOR_HANDLERS), default-ветка ловит
    #    «неизвестный supervisor frame» → ERROR{BAD_PAYLOAD}.
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
        # pre-guard: каждый guard либо отправляет ERROR и возвращает None,
        # либо возвращает (data, client_id) — «можно продолжать».
        prepared = await self._prepare_supervisor_dispatch(ws, session, ftype, payload)
        if prepared is None:
            return
        data, client_id = prepared
        # табличный dispatcher; default-ветка ловит FrameType'ы, которые
        # маршрутизируются сюда из _ws_handler, но handler'а не имеют
        # (например, будущие расширения SUPERVISOR_FRAME_TYPES без правки
        # таблицы). Защита «добавил новый ftype → забыл handler» →
        # явный ERROR клиенту, а не молчаливый drop.
        handler = SUPERVISOR_HANDLERS.get(ftype)
        if handler is None:
            log.error(
                "supervisor_api: no handler for ftype=%s (frame dropped)",
                ftype,
            )
            await self._send_error(
                ws,
                0,
                ErrorCode.BAD_PAYLOAD,
                f"unknown supervisor frame {ftype.name} (0x{ftype.value:02x})",
            )
            return
        await handler(self, ws, session, data, client_id)

    async def _prepare_supervisor_dispatch(
        self,
        ws,
        session: ClientSession,
        ftype: FrameType,
        payload: bytes,
    ) -> Optional[tuple[dict, str]]:
        """4 pre-guard'а для supervisor-фрейма: version / auth / msgpack / client_id.

        Возвращает ``(data, server_client_id)`` если все гварды прошли, иначе
        ``None`` (ERROR уже отправлен). Каждый guard — отдельная ветка,
        CC≤15 благодаря выносу из ``_handle_supervisor_command``.
        """
        if session.protocol_version != 2:
            await self._send_error(
                ws,
                0,
                ErrorCode.PROTOCOL_VERSION,
                f"{ftype.name} requires subprotocol v2 (negotiated: "
                f"{session.protocol_version}); update client (docs §11)",
            )
            return None
        if session.server_client_id is None:
            # Пре-аутентификация, теоретически не должно случиться (выше в
            # _ws_handler есть защита «session.state != authenticated»), но
            # defensive: ошибка аутентификации, не падать.
            await self._send_error(
                ws, 0, ErrorCode.AUTH_FAIL, "session not authenticated"
            )
            return None
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
            return None
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
        return (data, client_id)

    async def _send_supervisor_state_update(self, ws) -> None:
        """Отправить клиенту STATE_UPDATE с msgpack {state: <bridge snapshot>}.

        Вынесен из двух копи-паст (SET_MODE success + FLOOR success). Если
        bridge ещё не подключился (``snapshot is None``) — молча ничего не
        шлём, клиент дождётся следующего STATE_UPDATE (1 Hz keep-alive или
        следующая успешная supervisor-команда).
        """
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


# === AV-16: supervisor-frame handlers (module-level) =========================
#
# Каждый handler — async-функция с сигнатурой
#     async def _handle_<frame>(server, ws, session, data, client_id) -> None
# Почему не методы WSSServer: handler регистрируется в SUPERVISOR_HANDLERS
# на module-level (после определения класса), и обращение через unbound
# функцию WSSServer._method() потеряет self — мы бы передавали server
# в аргумент ``self`` handler-а, и сигнатура не совпала бы. Module-level
# async-функция явно принимает server первым аргументом — без сюрпризов.
#
# Handler'ы не делают pre-guard'ы (это работа _prepare_supervisor_dispatch);
# они работают с уже провалидированными (data, client_id) и сразу
# диспетчируют в bridge.


async def _handle_set_mode(
    server: "WSSServer",
    ws,
    session: ClientSession,
    data: dict,
    client_id: str,
) -> None:
    """SET_MODE (0x30): валидация mode → bridge.supervisor_set_mode → STATE_UPDATE.

    Контракт: applied=False → FSM не пропустила → MODE_CONFLICT;
    иначе → успех → STATE_UPDATE со свежим снапшотом.
    """
    mode = data.get("mode")
    if not isinstance(mode, str) or mode not in VALID_MODES_V2:
        await server._send_error(
            ws,
            0,
            ErrorCode.BAD_PAYLOAD,
            f"mode must be one of {list(VALID_MODES_V2)}; got {mode!r}",
        )
        return
    try:
        body = server.bridge.supervisor_set_mode(client_id, mode)
    except Exception as exc:  # noqa: BLE001
        # Мост не должен валить event-loop; если падает — это баг
        # реализации Bridge и его надо исправлять, но клиент получит
        # INTERNAL и сможет retry.
        log.exception("supervisor_set_mode bridge crashed: %s", exc)
        await server._send_error(
            ws,
            0,
            ErrorCode.INTERNAL,
            f"supervisor_set_mode: {exc}",
        )
        return
    if not body.get("applied"):
        await server._send_error(
            ws,
            0,
            ErrorCode.MODE_CONFLICT,
            str(body.get("reason", "refused")),
        )
        return
    # Успех: шлём клиенту свежий STATE_UPDATE с msgpack {state: ...}.
    await server._send_supervisor_state_update(ws)


async def _handle_acquire_floor(
    server: "WSSServer",
    ws,
    session: ClientSession,
    data: dict,
    client_id: str,
) -> None:
    """ACQUIRE_FLOOR (0x31): валидация floor → bridge.supervisor_acquire_floor.

    Контракт: granted/applied=False → FLOOR_HELD (с held_by если есть);
    иначе → успех → STATE_UPDATE со свежим снапшотом.
    """
    await _handle_floor_op(
        server,
        ws,
        session,
        data,
        client_id,
        bridge_call=server.bridge.supervisor_acquire_floor,
    )


async def _handle_release_floor(
    server: "WSSServer",
    ws,
    session: ClientSession,
    data: dict,
    client_id: str,
) -> None:
    """RELEASE_FLOOR (0x32): валидация floor → bridge.supervisor_release_floor.

    Контракт как у ACQUIRE_FLOOR (см. _handle_acquire_floor).
    """
    await _handle_floor_op(
        server,
        ws,
        session,
        data,
        client_id,
        bridge_call=server.bridge.supervisor_release_floor,
    )


async def _handle_floor_op(
    server: "WSSServer",
    ws,
    session: ClientSession,
    data: dict,
    client_id: str,
    *,
    bridge_call: Any,
) -> None:
    """Общая часть ACQUIRE/RELEASE_FLOOR: валидация + bridge_call + STATE_UPDATE.

    Раньше ACQUIRE/RELEASE шли двумя копи-паста if-блоками внутри
    ``_handle_supervisor_command`` (строки 2822-2876 develop @ 2026-09-08).
    Оба пути теперь здесь; выбор acquire vs release — через ``bridge_call``
    (частично применённый bridge-метод, передаётся из caller-а).
    """
    floor = data.get("floor")
    if not isinstance(floor, str) or floor not in VALID_FLOORS_V2:
        await server._send_error(
            ws,
            0,
            ErrorCode.BAD_PAYLOAD,
            f"floor must be one of {list(VALID_FLOORS_V2)}; got {floor!r}",
        )
        return
    try:
        body = bridge_call(client_id, floor)
    except Exception as exc:  # noqa: BLE001
        log.exception("supervisor_floor bridge crashed: %s", exc)
        await server._send_error(
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
        await server._send_error(
            ws,
            0,
            ErrorCode.FLOOR_HELD,
            err_message,
        )
        return
    # Успех: STATE_UPDATE со свежим snapshot (аналогично SET_MODE).
    await server._send_supervisor_state_update(ws)


# Каркас supervisor-фреймов: FrameType → async-handler.
# Используется ``_handle_supervisor_command`` после pre-guard-ов; default
# в dispatcher отправляет ERROR{BAD_PAYLOAD} «unknown supervisor frame»
# для FrameType-ов из SUPERVISOR_FRAME_TYPES, которые ещё не имеют
# зарегистрированного handler-а (защита от «добавил новый ftype в
# маршрутизацию — забыл handler»).
#
# Значения — module-level async-функции (НЕ bound-методы WSSServer),
# потому что dict инициализируется в module-scope, где instance
# WSSServer ещё не существует. Сигнатура handler-а — см. комментарий
# выше перед _handle_set_mode.
SUPERVISOR_HANDLERS: dict[FrameType, Any] = {
    FrameType.SET_MODE: _handle_set_mode,
    FrameType.ACQUIRE_FLOOR: _handle_acquire_floor,
    FrameType.RELEASE_FLOOR: _handle_release_floor,
}


# === JSON_CMD handler implementations ======================================


async def _json_cmd_ping(server, ws, session, payload):
    session.feed_ping()
    server.bridge.feed_client_alive()


async def _json_cmd_stream_list(server, ws, session, payload):
    await server._send(
        ws,
        FrameType.JSON_EVENT,
        0,
        {"type": "stream_list", "items": list(server.bridge.available_streams())},
    )


async def _json_cmd_stream_select(server, ws, session, payload):
    topic = payload.get("topic")
    spec = get_stream(topic) if isinstance(topic, str) else None
    if spec is None:
        await server._send_error(
            ws, 0, ErrorCode.TOPIC_UNKNOWN, f"topic '{topic}' not in registry"
        )
        return
    await server._send(
        ws,
        FrameType.JSON_EVENT,
        0,
        {
            "type": "stream_select_ack",
            "topic": topic,
            "stream_id": session.subscribed.get(topic),
            "kind": spec.kind.value,
        },
    )


async def _json_cmd_teleop_twist(server, ws, session, payload):
    try:
        linear = float(payload.get("linear", {}).get("x", 0.0))
        angular = float(payload.get("angular", {}).get("z", 0.0))
    except (TypeError, ValueError):
        await server._send_error(
            ws, 0, ErrorCode.BAD_PAYLOAD, "teleop_twist: bad linear/angular"
        )
        return
    ts_ms, seq = _json_cmd_ts_seq(payload)
    # AV-19 (issue #2190, voice-vr 05): сравниваем с server_client_id,
    # а не session_id — единый формат «quest:<uuid>» во всех точках
    # (gate/heartbeat/release/STATE_UPDATE). Это «внешнее имя» сессии,
    # которое видит avatar_supervisor.
    if server._require_teleop_floor and server._avatar_arbiter.floor_holder != (
        session.server_client_id
    ):
        if server._should_send_floor_held_error(session.session_id):
            await server._send_error(
                ws,
                0,
                ErrorCode.FLOOR_HELD,
                f"teleop_floor held by {server._avatar_arbiter.floor_holder!r}",
            )
        server.bridge.feed_client_alive()
        return
    server.bridge.publish_quest(linear, angular)
    server.bridge.feed_client_alive()
    try:
        # server_client_id гарантированно не None после AUTHENTICATED
        # (см. ClientSession.mark_authenticated); ``or ""`` — defensive.
        server.bridge.relay_teleop_heartbeat(
            session.server_client_id or "", ts_ms, seq
        )
    except Exception as exc:  # noqa: BLE001
        log.warning("quest: relay_teleop_heartbeat failed: %s", exc)


async def _json_cmd_teleop_heartbeat(server, ws, session, payload):
    ts_ms, seq = _json_cmd_ts_seq(payload)
    # AV-19 (issue #2190): сравниваем с server_client_id (как в gate twist).
    if server._require_teleop_floor and server._avatar_arbiter.floor_holder != (
        session.server_client_id
    ):
        # Тем не менее feed_client_alive — watchdog WSS-сессии
        # крутится по любой живости клиента.
        server.bridge.feed_client_alive()
        return
    try:
        server.bridge.relay_teleop_heartbeat(
            session.server_client_id or "", ts_ms, seq
        )
    except Exception as exc:  # noqa: BLE001
        log.warning("quest: relay_teleop_heartbeat failed: %s", exc)
    server.bridge.feed_client_alive()


def _json_cmd_ts_seq(payload: dict[str, Any]) -> tuple[int, int]:
    now_ms = int(time.time() * 1000)
    ts_raw = payload.get("ts_ms", now_ms)
    seq_raw = payload.get("seq", 0)
    try:
        ts_ms = int(ts_raw) if isinstance(ts_raw, (int, float)) else now_ms
        seq = int(seq_raw) if isinstance(seq_raw, (int, float)) else 0
    except (TypeError, ValueError):
        return now_ms, 0
    return ts_ms, seq


async def _json_cmd_stop_emergency(server, ws, session, payload):
    server.bridge.publish_emergency()
    server.bridge.emergency_stop()


async def _json_cmd_voice_ptt_start(server, ws, session, payload):
    client_id = payload.get("client_id")
    client_id = client_id if isinstance(client_id, str) else None
    result = server._avatar_arbiter.try_acquire_voice(session.session_id, client_id or "")
    if not result.granted:
        holder = result.busy_holder.label() if result.busy_holder is not None else "unknown"
        log.info("quest: voice floor DENIED session=%s busy=%s", session.session_id, holder)
        await server._send_voice_state(
            ws, "denied", int(time.time() * 1000), holder, f"busy: {holder}"
        )
        return
    server._voice_cache.update(
        FloorState.LISTENING, FloorHolder(session.session_id, client_id or "anon")
    )
    log.info(
        "quest: voice floor ACQUIRED session=%s client_id=%s",
        session.session_id,
        client_id or "anon",
    )
    if payload.get("mode") == "robot_voice":
        server.bridge.publish_voice_robot_start()
    else:
        server.bridge.publish_voice_barge_in()
    await server._send_voice_state(
        ws,
        "listening",
        int(time.time() * 1000),
        server._voice_cache.holder.label() if server._voice_cache.holder else None,
    )


async def _json_cmd_voice_ptt_stop(server, ws, session, payload):
    was_holder = server._avatar_arbiter.release_voice(session.session_id)
    if was_holder:
        server._voice_cache.update(FloorState.IDLE, None)
        log.info("quest: voice floor RELEASED session=%s", session.session_id)
    if payload.get("mode") == "robot_voice":
        server.bridge.publish_voice_robot_stop()
    else:
        server.bridge.publish_voice_stop()
    if was_holder:
        await server._send_voice_state(ws, "idle", int(time.time() * 1000))


async def _json_cmd_voice_mode(server, ws, session, payload):
    mode = payload.get("mode")
    server.bridge.set_voice_mode(mode)
    await server._send(
        ws,
        FrameType.JSON_EVENT,
        0,
        {"type": "voice_mode_ack", "mode": mode, "ts_ms": int(time.time() * 1000)},
    )


async def _json_cmd_voice_listen(server, ws, session, payload):
    active = payload.get("cmd") == "voice_listen_start"
    try:
        server.bridge.set_wake_stream_state(active)
    except Exception as exc:  # noqa: BLE001
        log.warning("quest: set_wake_stream_state failed: %s", exc)
    await server._send(
        ws,
        FrameType.JSON_EVENT,
        0,
        {"type": "voice_listen_ack", "active": active, "ts_ms": int(time.time() * 1000)},
    )


async def _json_cmd_supervisor(server, ws, session, payload):
    cmd = payload.get("cmd")
    # [voice-vr 09 / issue #2194]: канонические имена — ``avatar_set_mode``
    # / ``avatar_acquire_floor`` / ``avatar_release_floor`` (мета-API
    # ADR-0080 §1.2). Прежние ``supervisor_*`` остаются рабочими алиасами
    # (старые клиенты, тесты в test_ws_server_v2.py) — но каждое
    # использование логируем WARNING, чтобы легче отследить откат на
    # старые имена в полевых логах.
    if isinstance(cmd, str) and cmd.startswith("supervisor_"):
        log.warning(
            "quest: deprecated JSON_CMD alias used: %s "
            "(use avatar_* equivalent; see ADR-0080 §1.2 / issue #2194)",
            cmd,
        )
    if session.protocol_version != 2:
        await server._send_error(
            ws,
            0,
            ErrorCode.PROTOCOL_VERSION,
            f"{cmd} requires subprotocol v2 (negotiated: {session.protocol_version}); update client (docs §11)",
        )
        return
    if session.server_client_id is None:
        await server._send_error(ws, 0, ErrorCode.AUTH_FAIL, "session not authenticated")
        return
    client_id = session.server_client_id
    payload_client_id = payload.get("client_id")
    if payload_client_id is not None and payload_client_id != client_id:
        log.warning(
            "supervisor_api (JSON): client_id mismatch session=%s payload=%s (ignored; using server-side)",
            session.session_id,
            payload_client_id,
        )
    if cmd == "supervisor_get_state":
        await _json_cmd_supervisor_get_state(server, ws)
        return
    # [voice-vr 09 / issue #2194] Канонические имена avatar_* маппятся в
    # legacy supervisor_* для bridge (один и тот же API супервизора).
    # Payload в avatar_* использует ``kind``, в supervisor_* — ``floor``;
    # нормализуем в ``floor`` здесь, чтобы дальнейшая логика (включая
    # ``_json_cmd_supervisor_floor_op``, который сам читает
    # ``payload["cmd"]``) работала с одним именем.
    # avatar_get_state НЕ маппим и не регистрируем: клиент не шлёт ни
    # его, ни supervisor_get_state (grep по webxr_client/src пуст) —
    # команда без отправителя (issue #2194).
    if isinstance(cmd, str) and cmd.startswith("avatar_"):
        cmd = "supervisor_" + cmd[len("avatar_"):]
        payload = {**payload, "cmd": cmd}
        if "floor" not in payload and "kind" in payload:
            payload["floor"] = payload["kind"]
    if cmd == "supervisor_set_mode":
        if not await _json_cmd_supervisor_set_mode(server, ws, client_id, payload):
            return
    else:
        if not await _json_cmd_supervisor_floor_op(server, ws, client_id, payload):
            return
    await server._send(
        ws,
        FrameType.JSON_EVENT,
        0,
        {"type": "supervisor_state", "state": server.bridge.supervisor_state(), "ts_ms": int(time.time() * 1000)},
    )


async def _json_cmd_supervisor_get_state(server, ws) -> None:
    snapshot = server.bridge.supervisor_state()
    if isinstance(snapshot, (bytes, bytearray)):
        await server._send_error(ws, 0, ErrorCode.INTERNAL, "supervisor_state snapshot is msgpack bytes; use binary STATE_UPDATE frame instead")
        return
    await server._send(ws, FrameType.JSON_EVENT, 0, {"type": "supervisor_state", "state": snapshot, "ts_ms": int(time.time() * 1000)})


async def _json_cmd_supervisor_set_mode(server, ws, client_id, payload) -> bool:
    mode = payload.get("mode")
    if not isinstance(mode, str) or mode not in VALID_MODES_V2:
        await server._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, f"mode must be one of {list(VALID_MODES_V2)}; got {mode!r}")
        return False
    try:
        body = server.bridge.supervisor_set_mode(client_id, mode)
    except Exception as exc:  # noqa: BLE001
        log.exception("supervisor_set_mode bridge crashed: %s", exc)
        await server._send_error(ws, 0, ErrorCode.INTERNAL, f"supervisor_set_mode: {exc}")
        return False
    if not body.get("applied"):
        await server._send_error(ws, 0, ErrorCode.MODE_CONFLICT, str(body.get("reason", "refused")))
        return False
    return True


async def _json_cmd_supervisor_floor_op(server, ws, client_id, payload) -> bool:
    floor = payload.get("floor")
    if not isinstance(floor, str) or floor not in VALID_FLOORS_V2:
        await server._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, f"floor must be one of {list(VALID_FLOORS_V2)}; got {floor!r}")
        return False
    cmd = payload.get("cmd")
    try:
        method = server.bridge.supervisor_acquire_floor if cmd == "supervisor_acquire_floor" else server.bridge.supervisor_release_floor
        body = method(client_id, floor)
    except Exception as exc:  # noqa: BLE001
        log.exception("supervisor_floor bridge crashed: %s", exc)
        await server._send_error(ws, 0, ErrorCode.INTERNAL, f"supervisor floor: {exc}")
        return False
    if not bool(body.get("granted", body.get("applied"))):
        held_by = body.get("held_by")
        reason = body.get("reason", "refused")
        await server._send_error(ws, 0, ErrorCode.FLOOR_HELD, reason if held_by is None else f"{reason}; held_by={held_by}")
        return False
    return True


async def _json_cmd_list_voices(server, ws, session, payload):
    server._voice_rate_limit_check(ws, "list_voices", VOICE_LIST_MIN_INTERVAL_S)
    snap = server.bridge.list_voices_snapshot()
    await server._send(
        ws,
        FrameType.JSON_EVENT,
        0,
        {"type": "voice_list", "voices": snap["voices"], "active_provider": snap["active_provider"], "active_voice": snap["active_voice"], "ts_ms": int(time.time() * 1000)},
    )


async def _json_cmd_set_voice(server, ws, session, payload):
    mode = payload.get("mode")
    if mode is None:
        # Обратная совместимость (issue #2195): explicit ``mode`` — новое
        # поле контракта, но webxr_client/src/main.ts (sendStyleChange:305,
        # TTS picker apply:949) его пока не шлёт. До парной правки клиента
        # отсутствие ``mode`` не должно ронять picker — определяем намерение
        # по ЗНАЧЕНИЮ payload, как раньше (ADR-0080/AV-28 §P7, было в
        # if/elif-цепочке _on_json_cmd до voice-vr 10): preset из
        # VOICE_PRESET_IDS, наличие language, или preset без voice_id → это
        # style/language запрос (AV-28); иначе — TTS picker (AV-27).
        preset = payload.get("preset")
        language = payload.get("language")
        voice_id = payload.get("voice_id")
        style_without_voice = (
            isinstance(preset, str)
            and bool(preset)
            and not (isinstance(voice_id, str) and voice_id)
        )
        is_style_request = (
            (isinstance(preset, str) and preset in VOICE_PRESET_IDS)
            or isinstance(language, str)
            or style_without_voice
        )
        mode = "style" if is_style_request else "voice"
    if mode not in ("voice", "style"):
        await server._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "set_voice: mode must be 'voice' or 'style'")
        return
    if mode == "style":
        await _json_cmd_set_voice_style(server, ws, payload)
        return
    await _json_cmd_set_voice_provider(server, ws, payload)


async def _json_cmd_set_voice_style(server, ws, payload):
    preset = payload.get("preset")
    language = payload.get("language")
    if not isinstance(preset, str) and preset is not None:
        await server._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "set_voice: preset must be string")
        return
    if not isinstance(language, str) and language is not None:
        await server._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "set_voice: language must be string")
        return
    if not server._voice_rate_limit_check(ws, "set_voice_style", VOICE_STYLE_MIN_INTERVAL_S):
        await server._send(ws, FrameType.JSON_EVENT, 0, {"type": "voice_set_nack", "preset": preset, "language": language, "reason": "rate_limited", "ts_ms": int(time.time() * 1000)})
        return
    reason = _validate_voice_set_payload(preset, language)
    if reason:
        await server._send(ws, FrameType.JSON_EVENT, 0, {"type": "voice_set_nack", "preset": preset, "language": language, "reason": reason, "ts_ms": int(time.time() * 1000)})
        return
    if preset is not None:
        server.bridge.set_voice_preset(preset)
    if language is not None:
        server.bridge.set_voice_language(language)
    await server._send(ws, FrameType.JSON_EVENT, 0, {"type": "voice_set_ack", "preset": preset, "language": language, "ts_ms": int(time.time() * 1000)})


async def _json_cmd_set_voice_provider(server, ws, payload):
    voice_id = payload.get("voice_id")
    preset = payload.get("preset")
    if not server._voice_rate_limit_check(ws, "set_voice", VOICE_SET_MIN_INTERVAL_S):
        await server._send(ws, FrameType.JSON_EVENT, 0, {"type": "voice_set_nack", "voice_id": voice_id if isinstance(voice_id, str) else "", "reason": "rate_limited", "ts_ms": int(time.time() * 1000)})
        return
    if not isinstance(voice_id, str) or not voice_id:
        await server._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "set_voice: voice_id required")
        return
    if preset is not None and not isinstance(preset, str):
        await server._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, "set_voice: preset must be string")
        return
    ok, applied_voice, reason, available = server.bridge.set_voice(voice_id, preset)
    if not ok:
        err = {"type": "voice_set_nack", "voice_id": voice_id, "reason": reason or "unknown", "ts_ms": int(time.time() * 1000)}
        if available:
            err["available"] = available
        await server._send(ws, FrameType.JSON_EVENT, 0, err)
        return
    await server._send(ws, FrameType.JSON_EVENT, 0, {"type": "voice_set_ack", "voice_id": applied_voice or voice_id, "preset": preset or "standard", "ts_ms": int(time.time() * 1000)})


async def _json_cmd_voice_pipeline(server, ws, session, payload):
    if not server._voice_rate_limit_check(ws, "voice_pipeline", VOICE_STYLE_MIN_INTERVAL_S):
        await server._send(ws, FrameType.JSON_EVENT, 0, {"type": "voice_pipeline_nack", "preset": payload.get("preset"), "language": payload.get("language"), "reason": "rate_limited", "ts_ms": int(time.time() * 1000)})
        return
    llm_enabled = bool(payload.get("llm_enabled"))
    preset = payload.get("preset").strip().lower() if isinstance(payload.get("preset"), str) else ""
    language = payload.get("language").strip().lower() if isinstance(payload.get("language"), str) else VOICE_PIPELINE_DEFAULT_LANGUAGE
    reason = _validate_voice_pipeline_payload(llm_enabled, preset, language)
    if reason:
        await server._send(ws, FrameType.JSON_EVENT, 0, {"type": "voice_pipeline_nack", "preset": preset, "language": language, "reason": reason, "ts_ms": int(time.time() * 1000)})
        return
    server.bridge.publish_voice_pipeline(llm_enabled, preset, language)
    await server._send(ws, FrameType.JSON_EVENT, 0, {"type": "voice_pipeline_ack", "llm_enabled": llm_enabled, "preset": preset, "language": language, "ts_ms": int(time.time() * 1000)})


async def _json_cmd_preview_voice(server, ws, session, payload):
    if not server._voice_rate_limit_check(ws, "preview_voice", VOICE_PREVIEW_MIN_INTERVAL_S):
        return
    request_id, voice_id, text = payload.get("request_id"), payload.get("voice_id"), payload.get("text")
    for value, message in ((request_id, "request_id"), (voice_id, "voice_id"), (text, "text")):
        if not isinstance(value, str) or not value:
            await server._send_error(ws, 0, ErrorCode.BAD_PAYLOAD, f"preview_voice: {message} required")
            return
    if not server.start_preview_session(request_id, ws):
        await server._send(ws, FrameType.JSON_EVENT, 0, {"type": "preview_voice_error", "request_id": request_id, "reason": "too_many_concurrent_previews", "ts_ms": int(time.time() * 1000)})
        return
    server.bridge.publish_preview_voice(request_id, voice_id, text)


JSON_CMD_HANDLERS.update(
    {
        "ping": _json_cmd_ping,
        "stream_list": _json_cmd_stream_list,
        "stream_select": _json_cmd_stream_select,
        "teleop_twist": _json_cmd_teleop_twist,
        "teleop_heartbeat": _json_cmd_teleop_heartbeat,
        "stop_emergency": _json_cmd_stop_emergency,
        "voice_ptt_start": _json_cmd_voice_ptt_start,
        "voice_ptt_stop": _json_cmd_voice_ptt_stop,
        "voice_mode": _json_cmd_voice_mode,
        "voice_listen_start": _json_cmd_voice_listen,
        "voice_listen_stop": _json_cmd_voice_listen,
        "supervisor_set_mode": _json_cmd_supervisor,
        "supervisor_acquire_floor": _json_cmd_supervisor,
        "supervisor_release_floor": _json_cmd_supervisor,
        "supervisor_get_state": _json_cmd_supervisor,
        # [voice-vr 09 / issue #2194] канонические имена (ADR-0080 §1.2);
        # avatar_get_state НЕ регистрируем — senderless, удалён (closes #2194).
        "avatar_set_mode": _json_cmd_supervisor,
        "avatar_acquire_floor": _json_cmd_supervisor,
        "avatar_release_floor": _json_cmd_supervisor,
        "list_voices": _json_cmd_list_voices,
        "set_voice": _json_cmd_set_voice,
        "voice_pipeline": _json_cmd_voice_pipeline,
        "preview_voice": _json_cmd_preview_voice,
    }
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
