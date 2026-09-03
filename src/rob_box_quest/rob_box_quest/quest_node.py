"""ROS2-нода rob_box_quest — Zenoh-мост + teleop + dead-man + safety.

Источник истины: docs/architecture/meta-quest-api.md §5/§6/§9,
docs/adr/0027 §3.3 (dead-man, watchdog, emergency B),
docs/plans/2026-08-24-meta-quest-telepresence.md §1.3.

Что делает:
1. Поднимает aiohttp WSS server (Phase 1.2) внутри ROS executor.
2. QuestBridge имплементирует Protocol из ws_server.py:
   - publish_quest → TeleopController.consume + publish cmd_vel_quest
   - publish_emergency → publish cmd_vel_emergency (edge)
   - feed_client_alive → reset Watchdog
   - emergency_stop → TeleopController.emergency_stop + close WS
3. Периодический tick loop (30 Гц) — TeleopController.tick → publish.
4. Watchdog loop — если клиент молчит > 0.5 с → emergency_stop.
5. Подписка на /odom для robot_status aggregation (Phase 1.3: базовый,
   в Phase 1.6 — battery/wifi).

Запуск:
    ros2 run rob_box_quest quest_node
    # или
    python3 -m rob_box_quest.quest_node
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import threading
import time
from typing import Any, Optional

from audio_common_msgs.msg import AudioData
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String

from .core.safety import Watchdog
from .core.teleop import TeleopController
from .server.session import WATCHDOG_TIMEOUT_S as SESSION_WATCHDOG_TIMEOUT_S
from .server.ws_server import NoOpBridge, WSSServer, build_app
from .streams.alerts import Alert, AlertThresholds, evaluate_alerts
from .streams.battery import parse_battery_json, voltage_to_pct
from .streams.lidar import scan_to_payload
from .streams.occupancy import encode_map_2d, grid_to_png
from .streams.provider import CameraFrame, CameraProvider
from .streams.registry import STREAM_CATALOG
from .streams.status import StatusAggregator
from .streams.voice_state import normalize_voice_state
from .streams.wifi import read_wifi_rssi
from .protocol.topics import encode_voice_state

log = logging.getLogger(__name__)


# Throttle публикации cmd_vel_quest (meta-quest-api.md §9):
# "не чаще 30 Гц, на повторный seq сервер отбрасывает".
# У нас — fixed-rate 30 Гц.
PUBLISH_RATE_HZ: float = 30.0
PUBLISH_PERIOD_S: float = 1.0 / PUBLISH_RATE_HZ

# map_2d: минимальный интервал между PNG-кадрами карты. rtabmap
# перепубликует /rtabmap/map чаще, чем решётка реально меняется, а кодирование
# 958×744 в PNG на Pi стоит миллисекунды и сотни килобайт трафика. Поза робота
# едет отдельным лёгким кадром (см. Bridge.publish_map_pose), так что карта под
# ногами не отстаёт от движения даже при редком PNG.
MAP_PNG_MIN_PERIOD_S: float = 5.0

# map_2d: как часто перевысылать последний PNG, даже если карта не менялась.
# Кадры — не история: клиент, подключившийся между двумя обновлениями
# rtabmap, иначе остался бы вообще без карты (а rtabmap перепубликует её
# только когда решётка выросла — это могут быть минуты). Перевысылка берёт
# УЖЕ закодированный PNG из кэша, так что стоит она только трафика:
# ~120 KB на реальной карте робота, то есть ~12 KB/с — на фоне 800 KB/с
# одной потолочной камеры это шум.
MAP_PNG_RESEND_PERIOD_S: float = 4.0

# Wire-режим голоса (meta-quest-api.md §5) → voice_input_mode (ADR-0027 §3.4).
# Маппинг живёт здесь, потому что quest-сервер — точка перевода wire→ROS.
WIRE_TO_VOICE_INPUT_MODE: dict[str, str] = {
    "off": "respeaker",
    "passthrough": "quest_passthrough",
    "ttts_proxy": "quest_ttts",
    "stt_llm": "quest_stt",
    "llm_formalize": "quest_llm_formalize",
}

# Робот-голос (P7): EOU-детекция на лету. Пока оператор держит грип, PCM
# буферизуется, а по тишине (конец фразы) буфер уходит в STT — распознавание
# успевает ДО отпускания грипа (иначе гонка с voice_input_mode=respeaker).
VOICE_SAMPLE_RATE_HZ: int = 16000  # int16 PCM 16 kHz mono (webxr_client)
VOICE_BYTES_PER_MS: float = VOICE_SAMPLE_RATE_HZ * 2 / 1000.0  # 32 байта/мс
VOICE_SILENCE_THRESHOLD: int = 500  # пик int16 ниже → считаем тишиной
VOICE_SILENCE_TIMEOUT_MS: float = 300.0  # тишина дольше → конец фразы


def _chunk_is_silent(payload: bytes, threshold: int = VOICE_SILENCE_THRESHOLD) -> bool:
    """True если int16 LE PCM-чанк — тишина (пик |сэмпла| < threshold)."""
    if len(payload) < 2:
        return True
    for i in range(0, len(payload) - 1, 2):
        s = payload[i] | (payload[i + 1] << 8)  # int16 little-endian
        if s >= 0x8000:
            s -= 0x10000  # знаковый разряд
        if abs(s) >= threshold:
            return False
    return True


class _AlwaysActiveWSServer:
    """Заглушка для юнит-тестов: при отсутствии реального WSSServer считаем
    что «есть активная сессия» (1) — _publish_zero() будет лить нули как
    раньше, чтобы старая логика тестов осталась валидной."""

    def get_active_sessions(self) -> int:
        return 1

    # AV-16: broadcast_state_update — no-op в заглушке (Bridge без ws_server).
    def broadcast_state_update(self, payload: bytes) -> int:  # noqa: ARG002
        return 0


def _stop_msg() -> String:
    """String("STOP") — команда остановки воспроизведения/стрима."""
    m = String()
    m.data = "STOP"
    return m


def _string_msg(value: str) -> String:
    """std_msgs/String с заданным значением."""
    m = String()
    m.data = value
    return m


def _read_alert_thresholds(node) -> AlertThresholds:
    """Читает ROS-параметры в AlertThresholds. Выделено в функцию, чтобы
    тесты могли использовать ту же логику без rclpy-ноды."""
    return AlertThresholds(
        battery_low_pct=int(node.get_parameter("alert_battery_low_pct").value),
        battery_hysteresis_pct=int(
            node.get_parameter("alert_battery_hysteresis_pct").value
        ),
        wifi_weak_dbm=int(node.get_parameter("alert_wifi_weak_dbm").value),
        wifi_hysteresis_dbm=int(node.get_parameter("alert_wifi_hysteresis_dbm").value),
        stuck_timeout_s=float(node.get_parameter("alert_stuck_timeout_s").value),
        stuck_cmd_eps=float(node.get_parameter("alert_stuck_cmd_eps").value),
        hold_ms=int(node.get_parameter("alert_hold_ms").value),
    )


class OdomMotionTracker:
    """Следит за движением робота по ``/odom`` для ROBOT_STUCK детектора.

    Алгоритм:
      - берём pose (x,y) из последнего /odom;
      - сравниваем с прошлым — если дистанция > ``eps_m``, считаем что
        робот реально двигался (сбрасываем счётчик);
      - иначе накапливаем ``motion_s`` секунд (от monotonic).

    Считаем по 2D-плоскости (x,y) потому что твист — плоский; z и
    orientation используем только если 2D-плоскости нет (одометрия
    в стартовом положении).

    ``odom_motion_s`` сбрасывается в 0 при движении. Для
    ROBOT_STUCK важно «сколько секунд не двигался» — это и есть
    основное поле, передаваемое в ``evaluate_alerts``.
    """

    def __init__(self, eps_m: float = 0.01) -> None:
        self._eps_m2 = eps_m * eps_m
        self._last_x: Optional[float] = None
        self._last_y: Optional[float] = None
        self._last_motion_monotonic: Optional[float] = None

    def update(self, x: float, y: float, now_monotonic: float) -> float:
        """Обновить позицию, вернуть секунды с последнего видимого движения."""
        if self._last_x is None or self._last_y is None:
            self._last_x = float(x)
            self._last_y = float(y)
            self._last_motion_monotonic = now_monotonic
            return 0.0
        dx = float(x) - self._last_x
        dy = float(y) - self._last_y
        if (dx * dx + dy * dy) > self._eps_m2:
            self._last_x = float(x)
            self._last_y = float(y)
            self._last_motion_monotonic = now_monotonic
            return 0.0
        if self._last_motion_monotonic is None:
            self._last_motion_monotonic = now_monotonic
            return 0.0
        return float(now_monotonic - self._last_motion_monotonic)

    def seconds_since_last_motion(self, now_monotonic: float) -> Optional[float]:
        """Секунд с момента последнего видимого движения, ``None`` если
        ещё ни одного /odom не приходило. Используется в
        ``QuestNode._on_alert_timer`` для ROBOT_STUCK detector'а.
        """
        if self._last_motion_monotonic is None:
            return None
        return float(max(0.0, now_monotonic - self._last_motion_monotonic))


class QuestBridge:
    """Реализация Bridge Protocol из ws_server.py через rclpy publishers.

    Хранит TeleopController (один на Phase 1) и Watchdog.
    Publish из event-loop aiohttp безопасен (rclpy thread-safe).
    """

    def __init__(
        self,
        node: "QuestNode",
        cmd_vel_quest_pub,
        cmd_vel_emergency_pub,
        ws_server: Optional[WSSServer] = None,
        voice_in_pub=None,
        tts_control_pub=None,
        sound_stop_pub=None,
        stt_in_pub=None,
        set_voice_mode_pub=None,
        set_voice_preset_pub=None,
        set_voice_language_pub=None,
        set_voice_pub=None,
        preview_voice_pub=None,
        voices_cache_ttl_sec: float = 300.0,
        heartbeat_pub=None,  # AV-19: publisher in /teleop_heartbeat
        supervisor_acquire_client=None,
        supervisor_release_client=None,
        supervisor_set_mode_client=None,
        avatar_state_subscription=None,
    ) -> None:
        self._node = node
        self._pub_quest = cmd_vel_quest_pub
        self._pub_emergency = cmd_vel_emergency_pub
        # Рация (P3): голос оператора + STOP-каналы. None в unit-тестах моста.
        self._voice_in_pub = voice_in_pub
        self._tts_control_pub = tts_control_pub
        self._sound_stop_pub = sound_stop_pub
        # Робот-голос (P7): буфер PCM → /audio/quest_in (STT).
        self._stt_in_pub = stt_in_pub
        # voice_mode → супервизор (ADR-0028 S5): /avatar/set_voice_mode.
        self._set_voice_mode_pub = set_voice_mode_pub
        # AV-28 §P7 (issue #1920) — voice style preset / language → супервизор.
        # /avatar/set_voice_preset, /avatar/set_voice_language (см. meta-quest-api.md §P7).
        self._set_voice_preset_pub = set_voice_preset_pub
        self._set_voice_language_pub = set_voice_language_pub
        # AV-27 / issue #1919 — set_voice / preview_voice → супервизор.
        self._set_voice_pub = set_voice_pub
        self._preview_voice_pub = preview_voice_pub
        # AV-19: publisher в /teleop_heartbeat. None в unit-тестах —
        # тогда relay_teleop_heartbeat будет no-op (см. его комментарий).
        self._heartbeat_pub = heartbeat_pub
        # AV-16: supervisor service-clients (каждый — async call_service).
        # None в unit-тестах моста; реальные ROS-клиенты создаются на уровне
        # QuestNode (этот конструктор — DI). Sync-обёртки service calls
        # живут ниже (supervisor_acquire_floor / _release_floor / _set_mode).
        self._srv_acquire = supervisor_acquire_client
        self._srv_release = supervisor_release_client
        self._srv_set_mode = supervisor_set_mode_client
        self._state_sub = avatar_state_subscription
        # Локальный кеш последнего /avatar/state snapshot (msgpack bytes).
        # None до первого прихода callback-а; Bridge.supervisor_state() → None.
        self._avatar_state_cache: Optional[bytes] = None
        # Lock для cache update — не rclpy-thread-safe, ROS-callback-и
        # и WS-handler-ы (aiohttp loop) пишут/читают параллельно.
        self._avatar_state_lock = threading.Lock()
        # Текущий голосовой режим: "radio" (рация, default) | "robot_voice".
        self._voice_mode: str = "radio"
        self._voice_buffer: list[bytes] = []
        self._voice_silence_ms: float = 0.0
        # ws_server может быть None в юнит-тестах. В проде QuestNode всегда
        # передаёт реальный WSSServer — иначе _publish_zero() вернёт True
        # через заглушку и поведение будет как «есть активная сессия».
        self._ws_server = ws_server or _AlwaysActiveWSServer()
        self._teleop = TeleopController()
        self._watchdog = Watchdog(timeout_s=SESSION_WATCHDOG_TIMEOUT_S)
        # Edge-флаг: один emergency per link-loss до reset() (анти-спам).
        self._emergency_published: bool = False
        # Маппинг camera ui_name → device_id (для on_frame callback из CameraProvider).
        self._camera_id_to_ui: dict[str, str] = {}
        # map_2d: геометрия последней карты (res, w, h, origin_x, origin_y) —
        # нужна, чтобы лёгкий кадр «только поза» нёс те же размеры, что PNG.
        self._map_info: Optional[tuple[float, int, int, float, float]] = None
        # Последний закодированный PNG + отметки времени: когда кодировали
        # (дроссель на CPU) и когда в последний раз отправили (перевысылка
        # для поздно подключившихся клиентов).
        self._map_png: Optional[bytes] = None
        self._map_png_encoded_ts: float = 0.0
        self._map_png_sent_ts: float = 0.0
        # ── AV-27 voices-cache ────────────────────────────────────────────
        # Кэш последнего /voice/tts/voices (latched, TRANSIENT_LOCAL). tts_node
        # перепубликует на старт + каждый set_provider → мы не теряем свежести.
        # Тем не менее держим локальный expiry на случай переподключения WS без
        # рестарта quest_node (tts_node мог переподняться, а quest-WS не
        # переподписался). По дизайн-доку t_5b9d5d0c §128-150.
        self._voices_cache: list[dict[str, Any]] = []
        self._voices_cache_ts: float = 0.0
        self._voices_cache_ttl_sec: float = float(voices_cache_ttl_sec)
        self._voices_cache_lock = threading.Lock()
        # Кэш активного провайдера + голоса (/voice/tts/provider_state).
        self._active_provider: str = ""
        self._active_voice: str = ""
        # Сохраняем loop aiohttp для отправки STATE_UPDATE из ROS-callback-а.
        # None до тех пор, пока ``QuestNode._start_aiohttp()`` не сходится.
        self._aio_send_loop: Optional[asyncio.AbstractEventLoop] = None

    def set_aio_send_loop(self, loop: Optional[asyncio.AbstractEventLoop]) -> None:
        """Запомнить aiohttp-loop для потокобезопасной отправки STATE_UPDATE.

        Вызывается из :py:meth:`QuestNode._start_aiohttp` (раньше аналогичный
        hook уже был у ``WSSServer.set_send_loop`` — теперь и Bridge, потому
        что supervisor-* callbacks на Bridge идут ВНЕ aiohttp thread (ROS
        executor), а ``broadcast_state_update`` в WSSServer — это send_bytes,
        требующий async-loop.
        """
        self._aio_send_loop = loop

    # --- Bridge Protocol -------------------------------------------------

    def publish_quest(self, linear: float, angular: float) -> None:
        """Bridge.feed из ws_server на teleop_twist.

        Phase 1.3: просто публикуем cmd_vel_quest с текущими значениями.
        Контроллер (dead-man) живёт на стороне клиента: при отпускании
        grip клиент шлёт JSON_CMD{cmd:'teleop_twist', deadman:false} →
        мы публикуем нулевой Twist (это обеспечивается самим клиентом).
        Здесь мы публикуем те значения, которые присланы (clamping внутри
        TeleopController).
        """
        now = time.monotonic()
        twist = self._teleop.consume(linear, angular, deadman=True, now_monotonic=now)
        if twist is None:
            # consume вернул None (emergency) → публикуем нулевой Twist
            # чтобы twist_mux увидел timeout и emergency_stop выиграл.
            self._publish_zero()
            return
        msg = Twist()
        msg.linear.x = float(twist.linear_x)
        msg.angular.z = float(twist.angular_z)
        self._pub_quest.publish(msg)

    def publish_emergency(self) -> None:
        """Edge-triggered: ОДИН Twist в cmd_vel_emergency + ОДИН WARNING.

        twist_mux видит timeout 0.1 с на cmd_vel_emergency (priority 255)
        → effective cmd_vel = 0, держится до нового feed() или reset().
        Повторные вызовы до reset() — no-op (анти-спам в логах и по сети).
        """
        if self._emergency_published:
            return
        self._emergency_published = True
        self._node.get_logger().warning(
            "🛑 EMERGENCY STOP from Quest client — publishing cmd_vel_emergency"
        )
        self._pub_emergency.publish(Twist())

    def feed_client_alive(self) -> None:
        """Клиент прислал валидный фрейм → сброс watchdog + снять edge."""
        self._watchdog.feed(time.monotonic())
        self._emergency_published = False

    def emergency_stop(self) -> None:
        """Зафиксировать emergency lock (клиент прислал stop_emergency)."""
        self._teleop.emergency_stop()
        self._publish_zero()

    def publish_frame(self, ui_name: str, payload: bytes) -> None:
        """Bridge.publish_frame — пересылает payload в WS-подписчикам."""
        # Передаём в WSSServer, тот сам знает про сессии и стримы.
        self._ws_server.broadcast_frame(ui_name, payload)

    def available_streams(self) -> list[dict[str, Any]]:
        """Список стримов для JSON_CMD{cmd:stream_list}."""
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

    # --- Voice passthrough (рация, P3) ---------------------------------

    def publish_voice_barge_in(self) -> None:
        """PTT start: STOP в /voice/tts/control + /voice/sound/stop (barge-in)."""
        if self._tts_control_pub is None or self._sound_stop_pub is None:
            return
        self._tts_control_pub.publish(_stop_msg())
        self._sound_stop_pub.publish(_stop_msg())

    def publish_voice_audio(self, payload: bytes) -> None:
        """VOICE_AUDIO: radio → /avatar/voice_in; robot_voice → EOU-детекция.

        В robot_voice PCM НЕ стримим в динамик, а буферизуем. По тишине
        (конец фразы, пока грип ещё зажат) буфер уходит одним AudioData в
        /audio/quest_in (STT) — распознавание успевает до отпускания грипа.
        """
        if self._voice_mode == "robot_voice":
            if _chunk_is_silent(payload):
                if self._voice_buffer:
                    chunk_ms = len(payload) / VOICE_BYTES_PER_MS
                    self._voice_silence_ms += chunk_ms
                    if self._voice_silence_ms >= VOICE_SILENCE_TIMEOUT_MS:
                        self._flush_voice_buffer()
                # ведущая тишина — игнор
            else:
                self._voice_buffer.append(payload)
                self._voice_silence_ms = 0.0
            return
        if self._voice_in_pub is None:
            return
        msg = AudioData()
        msg.data = list(payload)
        self._voice_in_pub.publish(msg)

    def _flush_voice_buffer(self) -> None:
        """Накопленный PCM → один AudioData в /audio/quest_in (STT)."""
        if self._stt_in_pub is None or not self._voice_buffer:
            self._voice_buffer = []
            self._voice_silence_ms = 0.0
            return
        data = b"".join(self._voice_buffer)
        self._voice_buffer = []
        self._voice_silence_ms = 0.0
        msg = AudioData()
        msg.data = list(data)
        self._stt_in_pub.publish(msg)

    def publish_voice_stop(self) -> None:
        """PTT stop: STOP в /voice/sound/stop → sound_node закрывает стрим."""
        if self._sound_stop_pub is None:
            return
        self._sound_stop_pub.publish(_stop_msg())

    def publish_voice_robot_start(self) -> None:
        """PTT start (робот-голос): barge-in + перейти в режим буферизации."""
        self._voice_mode = "robot_voice"
        self._voice_buffer = []
        self._voice_silence_ms = 0.0
        self.publish_voice_barge_in()

    def publish_voice_robot_stop(self) -> None:
        """PTT stop (робот-голос): слить остаток фразы без завершающей тишины."""
        if self._voice_mode != "robot_voice":
            return
        self._voice_mode = "radio"
        self._flush_voice_buffer()

    def set_voice_mode(self, mode: str) -> None:
        """voice_mode cmd → запрос супервизору сменить режим голоса.

        Wire-режим (meta-quest-api.md §5) маппится в ``voice_input_mode``
        (ADR-0027 §3.4) и публикуется в /avatar/set_voice_mode. Собственно
        применяет параметр супервизор (ADR-0028 S5) — quest-сервер только
        ретранслирует запрос и НЕ трогает dialogue_node напрямую.
        """
        if self._set_voice_mode_pub is None:
            return
        param_mode = WIRE_TO_VOICE_INPUT_MODE.get(mode)
        if param_mode is None:
            self._node.get_logger().warning(f"quest: unknown voice_mode {mode!r}")
            return
        self._set_voice_mode_pub.publish(_string_msg(param_mode))

    # ── AV-28 §P7 (issue #1920): voice style preset + language ──────────────
    # Симметрично ``set_voice_mode``: ws_server вызывает → публикуем в
    # /avatar/set_voice_preset или /avatar/set_voice_language → супервизор
    # делает SetParameters(voice_preset=…) / SetParameters(voice_output_language=…)
    # на dialogue_node (ADR-0028 §S5, meta-quest-api.md §P7).
    # Сам quest_node НЕ трогает dialogue_node напрямую — единая точка записи
    # для всех voice-параметров супервизор.
    def set_voice_preset(self, preset: str) -> None:
        """AV-28 §P7: запрос супервизору сменить стиль речи.

        Публикует ``String`` с ``preset`` (один из VOICE_PRESET_IDS) в
        ``/avatar/set_voice_preset``. Whitelist — на ws_server, но если
        сюда дошёл неожиданный ID, пишем WARN и выходим.
        """
        if self._set_voice_preset_pub is None:
            self._node.get_logger().warning(
                "quest: set_voice_preset called but publisher not initialized"
            )
            return
        self._set_voice_preset_pub.publish(_string_msg(preset))

    def set_voice_language(self, language: str) -> None:
        """AV-28 §P7: запрос супервизору сменить язык вывода.

        Публикует ``String`` с ``language`` (один из VOICE_LANGUAGES: ru|en)
        в ``/avatar/set_voice_language``. Без рестарта dialogue_node —
        параметр подхватывается на следующей фразе.
        """
        if self._set_voice_language_pub is None:
            self._node.get_logger().warning(
                "quest: set_voice_language called but publisher not initialized"
            )
            return
        self._set_voice_language_pub.publish(_string_msg(language))

    # ── AV-27 TTS picker (issue #1919) ────────────────────────────────────

    def list_voices_snapshot(self) -> dict[str, Any]:
        """Sync-снимок voices-кэша + активный провайдер/голос.

        Возвращаемый словарь — это ровно та полезная нагрузка, которую ws_server
        форвардит клиенту в ``JSON_EVENT{type:"voice_list"}`` + добавляет ts_ms.
        Если tts_node ещё ни разу не прислал /voice/tts/voices (например, нода
        упала) — voices=[] и active_provider="" ; UI отрисует
        «провайдер не отдаёт список голосов» (acceptance #1919). Никаких
        хардкод-fallback'ов (см. design t_5b9d5d0c §52-87).
        """
        with self._voices_cache_lock:
            cache = list(self._voices_cache)
            cache_ts = self._voices_cache_ts
        fresh = (cache_ts > 0.0) and (
            (time.monotonic() - cache_ts) <= self._voices_cache_ttl_sec
        )
        if not fresh:
            cache = []  # просрочен — честно пусто (WS-клиент увидит voices=[])
        return {
            "voices": cache,
            "active_provider": self._active_provider,
            "active_voice": self._active_voice,
        }

    def set_voice(
        self, voice_id: str, preset: str | None
    ) -> tuple[bool, str | None, str | None, list[str] | None]:
        """Sync-валидация + публикация запроса на /avatar/set_voice.

        Валидация делается ЗДЕСЬ (а не откладывается на supervisor) чтобы
        ws_server мог сразу вернуть voice_set_nack с осмысленным reason
        (voice_unavailable / tts_unreachable / unknown_voice_id) — пользователь
        в UI Quest увидит ошибку мгновенно, а не через 1.5 с после таймаута.

        Returns:
            (ok, applied_voice_id, reason, available)
            * ok=True, applied_voice_id=voice_id (мы подтверждаем голос, зная
              что он есть у активного провайдера), reason=None, available=None —
              нормальный ack; фактическое применение подтвердится через
              /voice/tts/provider_state (republish tts_node на смену параметра).
            * ok=False, reason="tts_unreachable"|"voice_unavailable"|"missing_publisher"
              — nack; available заполняется списком id голосов активного
              провайдера когда reason="voice_unavailable" (UI-подсказка).
        """
        if self._set_voice_pub is None:
            return False, None, "missing_publisher", None
        provider = self._active_provider
        if not provider:
            return False, None, "tts_unreachable", None
        voices = _voices_for(provider)
        if not voices:
            return False, None, "tts_unreachable", None
        if voice_id not in voices:
            return False, None, "voice_unavailable", voices
        # Валидно → публикуем запрос супервизору. Формат: JSON-строка в
        # std_msgs/String (как /avatar/set_voice_mode и /avatar/set_voice).
        payload = {
            "voice_id": voice_id,
            "preset": preset,
            "provider": provider,
            "ts_ms": int(time.time() * 1000),
        }
        try:
            self._set_voice_pub.publish(
                _string_msg(json.dumps(payload, ensure_ascii=False))
            )
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f"quest: set_voice publish failed: {exc}")
            return False, None, "publish_failed", None
        return True, voice_id, None, None

    def publish_preview_voice(self, request_id: str, voice_id: str, text: str) -> None:
        """Опубликовать запрос на preview-синтез.

        Ответ придёт асинхронно через /avatar/preview_voice/{audio,result,error}.
        ws_server держит маппинг request_id → ws и форвардит preview_voice_audio
        (BINARY_FRAME + JSON_EVENT{type:"preview_voice_audio", ...}) + итоговые
        done/error. Это разделение делает мост «publish-only»: топик односторонний,
        supervisor пишет ответ в СВОЙ топик, ws_server слушает его и шлёт клиенту.
        """
        if self._preview_voice_pub is None:
            return
        provider = self._active_provider or ""
        payload = {
            "request_id": request_id,
            "voice_id": voice_id,
            "text": text,
            "provider": provider,
            "ts_ms": int(time.time() * 1000),
        }
        try:
            self._preview_voice_pub.publish(
                _string_msg(json.dumps(payload, ensure_ascii=False))
            )
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(
                f"quest: preview_voice publish failed: {exc}"
            )

    # ── AV-27 ROS hooks (tts_node → quest_node) ──────────────────────────

    def on_voices_message(self, msg: String) -> None:
        """Подписка на /voice/tts/voices (TRANSIENT_LOCAL, depth=1).

        Полученный JSON парсим и заменяем кэш. TTL — монотонный clock; устаревший
        кэш (если tts_node пропал) отдаётся как [] через list_voices_snapshot.
        """
        try:
            data = json.loads(msg.data) if msg.data else None
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        voices_raw = data.get("voices")
        if not isinstance(voices_raw, list):
            return
        # Приводим к dict-структуре. tts_node публикует список dict'ов с
        # полями voice_id, display_name, language, gender, presets, provider;
        # голоса «без метаданных» (fallback в registry) уже включают все
        # обязательные поля — см. tts_voice_registry.voice_info_for.
        normalized: list[dict[str, Any]] = []
        for entry in voices_raw:
            if isinstance(entry, dict) and "voice_id" in entry:
                normalized.append(entry)
        with self._voices_cache_lock:
            self._voices_cache = normalized
            self._voices_cache_ts = time.monotonic()
        # active_provider/active_voice могут прилететь и в /voice/tts/voices
        # payload — обновляем по согласованию (provider_state — основной
        # источник, этот — fallback).
        provider = data.get("provider")
        voice = data.get("voice") or data.get("default_voice")
        if isinstance(provider, str) and provider:
            self._active_provider = provider
        if isinstance(voice, str) and voice:
            self._active_voice = voice

    def on_provider_state_message(self, msg: String) -> None:
        """Подписка на /voice/tts/provider_state (volatile, depth=10).

        Это SoT для ``active_provider``/``active_voice`` на стороне Quest.
        Полезно при холодном старте quest_node (latched /voice/tts/voices
        уже пришёл, но провайдер мог переключиться с тех пор). Также
        триггерит invalidation voices-кэша если провайдер изменился — UI
        увидит новые голоса немедленно (см. design §128-150 invalidation).
        """
        try:
            data = json.loads(msg.data) if msg.data else None
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        provider = data.get("provider")
        voice = data.get("voice") or data.get("default_voice")
        if isinstance(provider, str) and provider and provider != self._active_provider:
            self._active_provider = provider
            # Смена провайдера → invalidate локальный TTL, чтобы следующий
            # list_voices заставил нас либо использовать свежий latched-publish
            # от tts_node (он уже прилетит сразу за provider_state), либо
            # честно показать [] пока не получим.
            with self._voices_cache_lock:
                self._voices_cache_ts = 0.0
        if isinstance(voice, str) and voice:
            self._active_voice = voice

    def on_camera_frame(self, frame: CameraFrame) -> None:
        """Hook из CameraProvider.capture-loop (capture-thread)."""
        # device_id → ui_name reverse-lookup.
        ui_name = self._camera_id_to_ui.get(frame.device_id)
        if ui_name is None:
            return
        self.publish_frame(ui_name, frame.data)

    def on_lidar_scan(self, msg: LaserScan) -> None:
        """Hook из ROS subscription /scan → encode → broadcast."""
        payload = scan_to_payload(
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            ranges=list(msg.ranges),
            intensities=list(msg.intensities),
        )
        self.publish_frame("lidar_2d", payload)

    def on_map(self, msg, pose: Optional[tuple[float, float, float]]) -> None:
        """Hook из ROS subscription /rtabmap/map → map_2d (0x1103).

        Кодирование решётки в PNG стоит несколько миллисекунд на 958×744,
        а rtabmap перепубликует карту чаще, чем она реально меняется,
        поэтому дросселируем: не чаще одного PNG в MAP_PNG_MIN_PERIOD_S.
        Между полными кадрами позу везёт publish_map_pose.
        """
        now = time.monotonic()
        if now - self._map_png_encoded_ts < MAP_PNG_MIN_PERIOD_S:
            return
        info = msg.info
        try:
            png = grid_to_png(msg.data, info.width, info.height)
        except Exception as e:  # noqa: BLE001
            # Нет cv2/numpy или битая решётка — карта просто не появится,
            # ронять ноду из-за декорации пола нельзя.
            logging.getLogger(__name__).warning("map_2d: grid encode failed: %s", e)
            return
        self._map_png_encoded_ts = now
        self._map_png = png
        self._map_info = (
            float(info.resolution),
            int(info.width),
            int(info.height),
            float(info.origin.position.x),
            float(info.origin.position.y),
        )
        self._publish_map(png=png, pose=pose)

    def publish_map_pose(self, pose: Optional[tuple[float, float, float]]) -> None:
        """Лёгкий map_2d-кадр: та же карта, новая поза робота.

        Раз в MAP_PNG_RESEND_PERIOD_S подмешивает в кадр закодированный
        ранее PNG. Без этого оператор, надевший очки между двумя
        обновлениями rtabmap, стоял бы на пустом полу: BINARY_FRAME —
        не latched-топик, историю новому подписчику никто не отдаёт.
        """
        if self._map_info is None or pose is None:
            return
        now = time.monotonic()
        resend = (
            self._map_png is not None
            and now - self._map_png_sent_ts >= MAP_PNG_RESEND_PERIOD_S
        )
        self._publish_map(png=self._map_png if resend else None, pose=pose)

    def _publish_map(
        self,
        *,
        png: Optional[bytes],
        pose: Optional[tuple[float, float, float]],
    ) -> None:
        assert self._map_info is not None
        resolution, width, height, origin_x, origin_y = self._map_info
        rx, ry, ryaw = pose if pose is not None else (None, None, None)
        if png is not None:
            self._map_png_sent_ts = time.monotonic()
        payload = encode_map_2d(
            resolution=resolution,
            width=width,
            height=height,
            origin_x=origin_x,
            origin_y=origin_y,
            robot_x=rx,
            robot_y=ry,
            robot_yaw=ryaw,
            ts_ms=int(time.time() * 1000),
            png=png,
        )
        self.publish_frame("map_2d", payload)

    # --- Periodic helpers (вызываются из QuestNode loop) -----------------

    def tick_publish(self, now_monotonic: float) -> None:
        """Периодическая публикация последнего Twist (30 Гц).

        Если клиент молчит — tick() вернёт None и мы публикуем нули.
        """
        twist = self._teleop.tick(now_monotonic)
        if twist is None:
            self._publish_zero()
            return
        msg = Twist()
        msg.linear.x = float(twist.linear_x)
        msg.angular.z = float(twist.angular_z)
        self._pub_quest.publish(msg)

    def current_mode(self, now_monotonic: float) -> str:
        """Режим для robot_status HUD: emergency > teleop_active > idle.

        ``TeleopController.tick`` — чистое чтение (не публикует), поэтому
        его можно спросить «есть ли свежая команда» из status-таймера.
        """
        if self._teleop.is_emergency:
            return "emergency"
        if self._ws_server.get_active_sessions() == 0:
            return "idle"
        return (
            "teleop_active" if self._teleop.tick(now_monotonic) is not None else "idle"
        )

    def last_cmd_twist(self) -> tuple[float, float]:
        """Последний non-zero (linear.x, angular.z) от teleop для ROBOT_STUCK.

        Если клиент DISARMED / emergency / ничего не прислал — (0, 0).
        caller'ы (QuestNode._on_alert_timer) сравнивают с ``stuck_cmd_eps``
        и поднимают ROBOT_STUCK только при наличии значимой команды.
        """
        if self._teleop.is_emergency:
            return 0.0, 0.0
        twist = self._teleop.last_twist
        return float(twist.linear_x), float(twist.angular_z)

    def watchdog_check(self, now_monotonic: float) -> bool:
        """True если watchdog trip → safe stop нужен."""
        return self._watchdog.tripped(now_monotonic)

    def watchdog_consume_trip(self, now_monotonic: float) -> bool:
        """Edge-triggered watchdog для timer 10 Гц — True только ОДИН раз
        на trip (не спамит emergency при каждом тике)."""
        return self._watchdog.consume_trip(now_monotonic)

    def reset(self) -> None:
        """Operator ack или новый HELLO — снимаем lock и edge-флаги."""
        self._teleop.reset()
        self._watchdog.reset()
        self._emergency_published = False

    # --- AV-19 (ADR-0028 §4.4 S10) ---------------------------------------

    def relay_teleop_heartbeat(self, client_id: str, ts_ms: int, seq: int) -> None:
        """Опубликовать TeleopHeartbeat в ``/teleop_heartbeat`` от ``client_id``.

        Контракт:
        - topic: ``/teleop_heartbeat`` (``std_msgs/String``, msgpack-encoded
          dict ``{client_id, ts_ms, seq}``).
        - Источник живости — клиент (ADR-0028 §4.4 «Не слать heartbeat на
          автомате»): мы только релеим, никогда не генерируем сами.
        - ts_ms — клиентское локальное время, seq — монотонная
          последовательность из TeleopFSM (используется для
          дедупликации на стороне супервизора и метрик).
        - ``self._heartbeat_pub`` может быть ``None`` в юнит-тестах —
          это сознательно, чтобы ws_server тестировался без rclpy.
        """
        if self._heartbeat_pub is None:
            return
        # ``_heartbeat_pub`` уже лениво создан в __init__ только при наличии
        # rclpy (см. _init_heartbeat_pub). Если телеоп-узел ещё не создал
        # pub (тест-сценарий), выходим тихо — relay не критичен.
        try:
            from std_msgs.msg import String as RosString  # type: ignore
            import json as _json

            payload = {"client_id": client_id, "ts_ms": int(ts_ms), "seq": int(seq)}
            msg = RosString()
            # JSON вместо msgpack — supervisor_client.py из rob_box_telegram
            # уже парсит оба (см. _on_state_msg), для единообразия Phase 1
            # шлём JSON (msgpack потребует AV-5 IDL).
            msg.data = _json.dumps(payload, ensure_ascii=False)
            self._heartbeat_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 — relay не должен ронять ноду
            self._node.get_logger().warning(
                f"quest: relay_teleop_heartbeat failed: {exc}"
            )

    def on_floor_lost(self, client_id: str) -> None:
        """Fail-safe при потере teleop_floor (ADR-0028 §4.4).

        Немедленно публикуем ``Twist(0,0)`` в ``cmd_vel_quest``,
        чтобы робот не продолжал ехать по инерции последнего фрейма.
        ``TeleopController.emergency_stop()`` здесь НЕ зовём — это
        другая семантика (полный lock + WS close); мы лишь
        «отрубаем подачу движения» и сбрасываем внутренний twist в
        ноль, чтобы tick() начал возвращать None.
        """
        self._teleop.emergency_stop()  # consume/tick → None до reset()
        self._publish_zero()
        # Логируем на уровне warning — это важный safety-event, Шифу
        # потом смотрит эти логи при разборе инцидентов «робот ехал
        # когда не должен был».
        self._node.get_logger().warning(
            f"🛑 FLOOR LOST for client_id={client_id} — publishing zero Twist (AV-19 fail-safe)"
        )

    # --- internal --------------------------------------------------------

    def _publish_zero(self) -> None:
        """Шлёт Twist(0,0) в cmd_vel_quest ТОЛЬКО пока есть активная WS-сессия.

        Если клиента нет — не публикуем ничего, чтобы:
        1) не блокировать twist_mux для других источников (joystick/nav2/teleop).
        2) не лить мусор в cmd_vel_quest когда Quest оффлайн.
        """
        if self._ws_server.get_active_sessions() == 0:
            return
        msg = Twist()
        # zero linear/angular — twist_mux timeout'нет и emergency_stop (255)
        # остаётся приоритетом; cmd_vel_quest публикует нули.
        self._pub_quest.publish(msg)

    # --- AV-16: supervisor API (bridge реализация) ------------------------

    def supervisor_acquire_floor(self, client_id: str, floor: str) -> dict:
        """Sync-обёртка: ``AcquireFloor`` сервис supervisor-а.

        Контракт см. ws_server.Bridge.supervisor_acquire_floor (Protocol).
        Реализация: ``asyncio.run_coroutine_threadsafe(call_service, loop)``
        на ROS-executor — критично, потому что ``client.call_async(req)
        .call_service`` — async и блокирует aiohttp event-loop.

        Если ROS-клиент недоступен (dev-env без rclpy, или supervisor ещё не
        задеплоен) → возвращаем ``applied=False/reason=service_unavailable``
        и логируем warning один раз.
        """
        if self._srv_acquire is None:
            return self._supervisor_unavailable("acquire_floor")
        return self._run_supervisor_service(
            "acquire_floor", self._srv_acquire, client_id=client_id, floor=floor
        )

    def supervisor_release_floor(self, client_id: str, floor: str) -> dict:
        if self._srv_release is None:
            return self._supervisor_unavailable("release_floor")
        return self._run_supervisor_service(
            "release_floor", self._srv_release, client_id=client_id, floor=floor
        )

    def supervisor_set_mode(self, client_id: str, mode: str) -> dict:
        """``SET_MODE`` (0x30) → сервис ``set_avatar_mode``.

        Целевой режим уходит на провод КАК ЕСТЬ. Маппинг «режим → FSM-
        событие» живёт только в супервизоре
        (``supervisor_node.MODE_TRANSITIONS``, AV-12): одно событие значит
        разные переходы из разных режимов, поэтому клиентская копия
        таблицы обязана разъехаться. Здесь она и была неверной —
        ``mixed`` жёстко маппился в ``quest_acquire_floor_teleop_only``,
        что верно только из ``telegram_active``, а неизвестный режим по
        умолчанию превращался в ``force_off``, то есть опечатка выключала
        аватар.
        """
        if self._srv_set_mode is None:
            return self._supervisor_unavailable("set_avatar_mode")
        return self._run_supervisor_service(
            "set_avatar_mode", self._srv_set_mode, mode=mode, client_id=client_id
        )

    @staticmethod
    def _supervisor_unavailable(name: str) -> dict:
        # «applied=False» + reason — клиент получит MODE_CONFLICT/FLOOR_HELD,
        # что совпадает с поведением supervisor в monitor-режиме. Если бы
        # возвращали INTERNAL, дев-сессии и монитор-развёртывания не смогли бы
        # тестировать WS-контракт без полного supervisor'а.
        return {
            "applied": False,
            "granted": False,
            "reason": f"supervisor_service_unavailable:{name}",
        }

    def _run_supervisor_service(
        self,
        service_name: str,
        client,
        *,
        client_id: Optional[str] = None,
        floor: Optional[str] = None,
        mode: Optional[str] = None,
        # Таймаут sync-вызова из WS-handler-а (acceptance: < 100 мс при
        # «зависшем» сервисе). 50 мс — запас над обычным ROS round-trip;
        # если supervisor отвечает дольше — degradation на INTERNAL, не
        # блокировать event-loop aiohttp.
        timeout_s: float = 0.05,
    ) -> dict:
        """Маршалит async call_service в ROS-executor и ждёт ответ sync.

        Pattern: ``asyncio.run_coroutine_threadsafe(call_async(req),
        ros_loop).result(timeout=...)``.
        """
        # Supervisor service contract (ADR-0028 §4.3 + типизированный IDL
        # rob_box_supervisor_msgs, AV-12 #1904): поля запроса — ровно
        # ``client_id`` + ``floor`` (AcquireFloor/ReleaseFloor) или
        # ``client_id`` + ``mode`` (SetAvatarMode). Имена жёсткие: у
        # сгенерированных rosidl-сообщений ``__slots__``, и setattr на
        # поле, которого в .srv нет, бросит AttributeError. Поэтому
        # никаких ad-hoc атрибутов вроде ``event`` здесь больше нет.
        ros_loop = getattr(self._node, "_ros_loop", None)
        if ros_loop is None:
            # rclpy.executors не разворачивает loop явно — попросим у самого Node.
            # Внутри rclpy-spin-callback-а ``asyncio.get_event_loop()`` бросит
            # ``RuntimeError`` (есть running-loop уже у rclpy). В этом случае
            # Sync-вызов через ``run_coroutine_threadsafe`` не пройдёт: spin_once
            # не обработает наш future, ws-handler ждать не может. Возвращаемся
            # к fallback — service_unavailable (см. монитор-режим supervisor-а).
            try:
                ros_loop = asyncio.get_event_loop()  # noqa: F841 — defensive
            except RuntimeError:
                return self._supervisor_unavailable(service_name)

        # Формируем request через ``client.cli_type.Request()`` — это
        # конкретный srv-тип (Trigger в Phase 1), атрибуты ставятся ad-hoc.
        request_cls = client.srv_type.Request
        request_obj = request_cls()
        if client_id is not None:
            setattr(request_obj, "client_id", client_id)
        if floor is not None:
            setattr(request_obj, "floor", floor)
        if mode is not None:
            setattr(request_obj, "mode", mode)

        async def _call() -> dict:
            fut = client.call_async(request_obj)
            result = await fut
            return _trigger_response_to_dict(result)

        try:
            future = asyncio.run_coroutine_threadsafe(_call(), ros_loop)
            return future.result(timeout=timeout_s)
        except (asyncio.TimeoutError, TimeoutError):
            self._node.get_logger().warning(
                f"supervisor_service:{service_name} timeout after {timeout_s * 1000:.0f} мс"
            )
            return {
                "applied": False,
                "granted": False,
                "reason": f"supervisor_service_timeout:{service_name}",
            }
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(
                f"supervisor_service:{service_name} failed: {exc}"
            )
            return {
                "applied": False,
                "granted": False,
                "reason": f"supervisor_service_failed:{service_name}:{exc}",
            }

    def supervisor_state(self) -> Optional[bytes]:
        """Текущий ``/avatar/state`` снапшот (msgpack bytes) или None."""
        with self._avatar_state_lock:
            return self._avatar_state_cache

    def on_supervisor_state(self, cb) -> None:
        # Реализация по контракту: callback подписки на изменения /avatar/state.
        # В QuestNode уже есть ``self._on_avatar_state_msg`` (ниже), который
        # обновляет cache И дёргает внешний cb, зарегистрированный через
        # ``on_supervisor_state``. Сейчас WSSServer нами не пользуется через
        # этот hook (вместо него делает broadcast через ``broadcast_state_update``
        # из QuestNode ROS-callback-а), но сигнатура нужна для Protocol.
        # Здесь — простая внутренняя защёлка: тесты могут её переопределить.
        self._external_state_cb = cb

    # --- /avatar/state ROS subscription callback ---------------------------

    def on_avatar_state(self, msg) -> None:
        """ROS subscription /avatar/state (transient_local, depth 1).

        Payload — ``std_msgs/String``, декодированный supervisor'ом как
        latin-1-mapped msgpack bytes (см. supervisor_node._publish_avatar_state).
        Декодируем **только** через ``rob_box_supervisor.core.state.unpack``
        (AV-14), сохраняем bytes в cache и пушим в WS через ws_server.
        """
        raw_text = getattr(msg, "data", None)
        if not isinstance(raw_text, str):
            return
        try:
            raw_bytes = raw_text.encode("latin-1")
        except UnicodeEncodeError:
            return
        # Валидация через единый decoder (запрет собственного парсера).
        try:
            from rob_box_supervisor.core.state import (
                unpack as _state_unpack,
            )  # noqa: WPS433

            _state_unpack(raw_bytes)
        except Exception as exc:  # noqa: BLE001
            # Schema-version mismatch или мусор — не падаем, лог + пропуск.
            self._node.get_logger().debug(f"avatar/state decode skipped: {exc}")
            return

        with self._avatar_state_lock:
            self._avatar_state_cache = raw_bytes

        # WS broadcast в v2-сессии (потокобезопасно через run_coroutine_threadsafe).
        if self._aio_send_loop is None:
            return
        loop = self._aio_send_loop
        try:
            asyncio.run_coroutine_threadsafe(
                self._dispatch_state_update(raw_bytes), loop
            )
        except RuntimeError:
            pass  # loop уже закрыт

    async def _dispatch_state_update(self, raw_bytes: bytes) -> None:
        """Async coroutine для ``broadcast_state_update``.

        Выполняется в aiohttp-loop thread; никакого rclpy внутри.
        """
        try:
            self._ws_server.broadcast_state_update(raw_bytes)
        except Exception as exc:  # noqa: BLE001
            log.debug("dispatch_state_update failed: %s", exc)


def _trigger_response_to_dict(response: Any) -> dict:
    """std_srvs/Trigger response (success + message) → dict для ws_server.

    ``response.message`` несёт JSON с полями ``applied/granted/reason`` —
    см. supervisor_node._fill_floor_response. Парсим без жёсткой зависимости
    от её содержимого: всё, что в response.success — флаг, остальное парсим.
    """
    import json as _json

    success = bool(getattr(response, "success", False))
    message_raw = getattr(response, "message", "")
    out: dict = {"success": success}
    if isinstance(message_raw, str) and message_raw:
        try:
            parsed = _json.loads(message_raw)
        except (TypeError, ValueError):
            parsed = {}
        if isinstance(parsed, dict):
            # НЕ пробрасываем False на «granted» если success=False но ключ
            # отсутствует — supervisor никогда не пишет «granted=True» если
            # сервис не сработал; фронт принимает ``applied/reason`` без
            # «granted» как «refused».
            for key, value in parsed.items():
                out[key] = value
    return out


class QuestNode(Node):
    """Главная ROS2-нода rob_box_quest.

    Параметры:
      ws_host (str, default "0.0.0.0")
      ws_port (int, default 8765) — внутри host-network,
                                    снаружи через Caddy (8443).
      pin     (str, optional) — если не задан, генерируется ACTIVE_PIN.
      log_pin (bool, default True) — логировать PIN в stdout (для docker logs).
    """

    def __init__(self) -> None:
        super().__init__("quest_node")
        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("ws_port", 8765)
        self.declare_parameter("log_pin", True)
        # Wave 3.A / R8 — live-телеметрия для HUD мостика.
        # Топик JSON-снапшота с батареей (std_msgs/String). Сегодня его никто
        # не публикует (ADR-0010 §4 — firmware сенсор-борда не готов), поэтому
        # имя вынесено в параметр: появится источник — хватит смены топика.
        self.declare_parameter("battery_json_topic", "/device/snapshot")
        # Границы пакета для перевода вольт → проценты. 0 = не переводить
        # (HUD покажет вольты). Угадывать химию/число банок нельзя.
        self.declare_parameter("battery_voltage_empty", 0.0)
        self.declare_parameter("battery_voltage_full", 0.0)
        # Wi-Fi RSSI читаем локально на Vision Pi — это тот же линк, по
        # которому идёт WSS до Quest. Пустое имя = первый интерфейс в таблице.
        self.declare_parameter("wifi_iface", "")
        # AV-19 (issue #1911, ADR-0028 §4.4): гейт teleop_floor.
        # Default=false чтобы не сломать текущий рабочий мостик; включается
        # # отдельным коммитом после e2e (карточка явно просит).
        self.declare_parameter("require_teleop_floor", False)
        # AV-26 / R7: robot_alert пороги (см. streams/alerts.py). Дефолты
        # дублируют значения из webxr_client/src/scene/status_hud.ts — клиент
        # и сервер не должны разъезжаться на «свечке» (acceptance: «в PR
        # цитата обеих сторон рядом»).
        self.declare_parameter("alert_battery_low_pct", 20)
        self.declare_parameter("alert_battery_hysteresis_pct", 5)
        self.declare_parameter("alert_wifi_weak_dbm", -75)
        self.declare_parameter("alert_wifi_hysteresis_dbm", 5)
        self.declare_parameter("alert_stuck_timeout_s", 3.0)
        self.declare_parameter("alert_stuck_cmd_eps", 0.05)
        self.declare_parameter("alert_hold_ms", 10_000)

        # AV-27 / issue #1919 — TTL локального voices-кэша. По дизайн-доку
        # t_5b9d5d0c §128-150 default 300 (5 минут); 0 = «никогда не
        # протухает, всегда отдавать последний latched-payload».
        self.declare_parameter("voices_cache_ttl_sec", 300)

        log_pin = bool(self.get_parameter("log_pin").value)
        self._battery_v_empty = float(self.get_parameter("battery_voltage_empty").value)
        self._battery_v_full = float(self.get_parameter("battery_voltage_full").value)
        self._wifi_iface = str(self.get_parameter("wifi_iface").value) or None
        self._require_teleop_floor = bool(
            self.get_parameter("require_teleop_floor").value
        )
        self._alert_thresholds = _read_alert_thresholds(self)

        # Publishers (см. twist_mux.yaml: priority 40 quest, 255 emergency).
        _RE = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub_quest = self.create_publisher(Twist, "cmd_vel_quest", _RE)
        self._pub_emergency = self.create_publisher(Twist, "cmd_vel_emergency", _RE)
        # Рация (voice passthrough): голос оператора → /avatar/voice_in (best-effort)
        # + STOP-каналы для barge-in (TTS) и закрытия стрима (sound).
        _VOICE_QOS = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Камера: подписываемся на JPEG-compressed image_transport-топик
        # (лёгкий поток ~300 KB/s вместо raw ~13.5 MB/s). OAK-D публикует его
        # RELIABLE KEEP_LAST(10) — RELIABLE-подписка обязательна (проверено
        # `ros2 topic info -v` на Vision Pi 27.08.2026).
        _CAMERA_QOS = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._voice_in_pub = self.create_publisher(
            AudioData, "/avatar/voice_in", _VOICE_QOS
        )
        self._tts_control_pub = self.create_publisher(String, "/voice/tts/control", _RE)
        self._sound_stop_pub = self.create_publisher(String, "/voice/sound/stop", _RE)
        # Робот-голос (P7): буфер операторского PCM → STT через /audio/quest_in.
        self._stt_in_pub = self.create_publisher(
            AudioData, "/audio/quest_in", _VOICE_QOS
        )
        # voice_mode → супервизор (ADR-0028 S5): /avatar/set_voice_mode.
        self._set_voice_mode_pub = self.create_publisher(
            String, "/avatar/set_voice_mode", _RE
        )
        # AV-28 §P7 (issue #1920) — voice style preset / language → супервизор.
        # Топики /avatar/set_voice_preset, /avatar/set_voice_language
        # (см. meta-quest-api.md §P7). Супервизор делает SetParameters на
        # dialogue_node (voice_preset / voice_output_language). Без рестарта
        # dialogue_node — параметр подхватывается на следующей фразе.
        self._set_voice_preset_pub = self.create_publisher(
            String, "/avatar/set_voice_preset", _RE
        )
        self._set_voice_language_pub = self.create_publisher(
            String, "/avatar/set_voice_language", _RE
        )
        # AV-27 / issue #1919 — TTS picker: set_voice / preview_voice →
        # супервизор (ADR-0028 S5/S12 — никаких прямых SetParameters из
        # quest_node на tts_node). Топики std_msgs/String (JSON payload),
        # симметрично /avatar/set_voice_mode.
        self._set_voice_pub = self.create_publisher(String, "/avatar/set_voice", _RE)
        self._preview_voice_pub = self.create_publisher(
            String, "/avatar/preview_voice", _RE
        )
        # Ответы preview_voice (String JSON):
        # /avatar/preview_voice/result — done/error с request_id;
        # /avatar/preview_voice/audio  — metaданные аудио (String JSON);
        # /avatar/preview_voice/error  — error напрямую.
        # payload для audio — JSON с audio_b64 (base64-encoded bytes);
        # BINARY-топик пока не используется (MVP — base64 внутри String).
        self._preview_result_sub = self.create_subscription(
            String, "/avatar/preview_voice/result", self._on_preview_result, 10
        )
        self._preview_audio_sub = self.create_subscription(
            String, "/avatar/preview_voice/audio", self._on_preview_audio, 10
        )
        self._preview_error_sub = self.create_subscription(
            String, "/avatar/preview_voice/error", self._on_preview_error, 10
        )
        # Подписка на /voice/tts/voices (TRANSIENT_LOCAL depth=1) — это
        # первый TRANSIENT_LOCAL publisher tts_node (см. design t_5b9d5d0c
        # §47-49). RELIABLE обязательно — TRANSIENT_LOCAL «latched» semantics
        # работают только при совпадении durability обеих сторон.
        _LATCHED_VOICES_QOS = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._voices_sub = self.create_subscription(
            String,
            "/voice/tts/voices",
            self._on_tts_voices,
            _LATCHED_VOICES_QOS,
        )
        # /voice/tts/provider_state — SoT для active_provider/active_voice.
        self._provider_state_sub = self.create_subscription(
            String,
            "/voice/tts/provider_state",
            self._on_tts_provider_state,
            10,
        )
        # AV-19 (issue #1911, ADR-0028 §4.4 S10): relay teleop_heartbeat.
        # Сюда ws_server релеит клиентский teleop_heartbeat / teleop_twist
        # от имени client_id (см. WSSServer._on_json_cmd).
        self._heartbeat_pub = self.create_publisher(String, "/teleop_heartbeat", _RE)

        # AV-16: supervisor service-clients (sync-вызовы из WS-handler через
        # run_coroutine_threadsafe). std_srvs/Trigger — Phase 1 IDL; Supervisor
        # принимает client_id/floor/event через getattr-атрибуты запроса
        # (см. supervisor_node._extract_*). Сервисы могут отсутствовать
        # в dev-env / на старте supervisor-а → QuestBridge получает None и
        # отвечает ``service_unavailable`` (см. _supervisor_unavailable).
        from std_srvs.srv import Trigger  # noqa: PLC0415 — локальный импорт

        try:
            self._srv_acquire = self.create_client(
                Trigger,
                "acquire_floor",
            )
        except Exception:  # pragma: no cover — rclpy без executor
            self._srv_acquire = None
        try:
            self._srv_release = self.create_client(
                Trigger,
                "release_floor",
            )
        except Exception:  # pragma: no cover
            self._srv_release = None
        try:
            self._srv_set_mode = self.create_client(
                Trigger,
                "set_avatar_mode",
            )
        except Exception:  # pragma: no cover
            self._srv_set_mode = None

        # /avatar/state подписка для STATE_UPDATE broadcast (транзент_локал,
        # depth 1 = «latched» по ADR-0028 §4.3 + supervisor_node:153-162).
        # QoS — те же параметры, что и у publisher'а супервизора; иначе
        # подписка не увидит late-joining snapshot.
        from rclpy.qos import (
            DurabilityPolicy as _DPolicy,
            ReliabilityPolicy as _RPolicy,
        )  # noqa: PLC0415

        _STATE_QOS = QoSProfile(
            depth=1,
            durability=_DPolicy.TRANSIENT_LOCAL,
            reliability=_RPolicy.RELIABLE,
        )
        try:
            self._avatar_state_sub = self.create_subscription(
                String,
                "/avatar/state",
                self._on_avatar_state_msg,
                _STATE_QOS,
            )
        except Exception:  # pragma: no cover
            self._avatar_state_sub = None
        # Подписки для стримов (Phase 1.4 v2: lidar + camera_rear-фолбэк через
        # ROS; остальные камеры — мимо ROS через CameraProvider).
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, _RE)
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan", self._on_scan, _RE
        )
        # camera_rear (0x1001): OAK-D color JPEG (image_transport compressed) →
        # форвардим bytes as-is в WS. Лёгкий путь: без cv2/numpy/перекодирования,
        # сеть грузится ~300 KB/s вместо raw ~13.5 MB/s.
        self._camera_rear_sub = self.create_subscription(
            CompressedImage,
            "/camera/camera/color/image_raw/compressed",
            self._on_camera_image,
            _CAMERA_QOS,
        )
        # camera_ceiling (0x1005): usb_cam публикует потолочную камеру в
        # /ceiling_camera/image_raw/compressed. Форвардим JPEG as-is — тот
        # же лёгкий путь, что у camera_rear.
        self._camera_ceiling_sub = self.create_subscription(
            CompressedImage,
            "/ceiling_camera/image_raw/compressed",
            self._on_ceiling_image,
            _CAMERA_QOS,
        )
        # map_2d (0x1103): SLAM-карта rtabmap. Публикуется TRANSIENT_LOCAL
        # (latched) — подписка обязана совпадать, иначе уже опубликованная
        # карта не придёт до следующего обновления, а оно может быть через
        # минуты.
        _MAP_QOS = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, "/rtabmap/map", self._on_map, _MAP_QOS
        )
        # voice_state (0x1202): /voice/dialogue/state (std_msgs/String) →
        # нормализованный msgpack-фрейм → WS-подписчикам. Подписка
        # RELIABLE — dialogue_node публикует так же (см.
        # docs/recon/voice-dialogue-state-payload.md §1.1).
        self._dialogue_state_sub = self.create_subscription(
            String,
            "/voice/dialogue/state",
            self._on_dialogue_state,
            _RE,
        )
        # Эх последнего state — для тестов моста без полного ROS-стека.
        self._last_voice_state: dict[str, Any] = {
            "state": "idle",
            "detail": None,
        }
        # Батарея (Wave 3.A): JSON-снапшот сенсор-борда + VESC-напряжение.
        # vesc_msgs есть не в каждом образе (пакет собирается на Main Pi),
        # поэтому подписка опциональная — без него остаётся JSON-путь.
        battery_topic = str(self.get_parameter("battery_json_topic").value)
        self._battery_json_sub = self.create_subscription(
            String, battery_topic, self._on_battery_json, _RE
        )
        self._vesc_state_sub = None
        try:
            from vesc_msgs.msg import VescStateStamped  # noqa: WPS433

            self._vesc_state_sub = self.create_subscription(
                VescStateStamped,
                "/sensors/motor_state/front_left",
                self._on_vesc_state,
                _RE,
            )
        except ImportError:
            self.get_logger().info(
                "vesc_msgs unavailable — battery voltage from JSON snapshot only"
            )
        self._latest_odom: Optional[Odometry] = None
        self._status = StatusAggregator()
        # AV-26: state для evaluate_alerts() — список Alert'ов, активных
        # на прошлом тике. Храним тут, не в Bridge, потому что Bridge живёт
        # без ROS и не должен зависеть от WS-сессии.
        self._active_alerts: list[Alert] = []
        self._odom_motion_tracker = OdomMotionTracker()

        # WS server (инициализируем первым чтобы передать в Bridge).
        from .server.ws_server import ACTIVE_PIN

        self.ws_server = WSSServer(
            bridge=NoOpBridge(),
            pin=ACTIVE_PIN,
            require_teleop_floor=self._require_teleop_floor,
        )
        self.bridge = QuestBridge(
            node=self,
            cmd_vel_quest_pub=self._pub_quest,
            cmd_vel_emergency_pub=self._pub_emergency,
            ws_server=self.ws_server,
            voice_in_pub=self._voice_in_pub,
            tts_control_pub=self._tts_control_pub,
            sound_stop_pub=self._sound_stop_pub,
            stt_in_pub=self._stt_in_pub,
            set_voice_mode_pub=self._set_voice_mode_pub,
            set_voice_preset_pub=self._set_voice_preset_pub,
            set_voice_language_pub=self._set_voice_language_pub,
            set_voice_pub=self._set_voice_pub,
            preview_voice_pub=self._preview_voice_pub,
            voices_cache_ttl_sec=float(
                self.get_parameter("voices_cache_ttl_sec").value
            ),
            heartbeat_pub=self._heartbeat_pub,
            supervisor_acquire_client=self._srv_acquire,
            supervisor_release_client=self._srv_release,
            supervisor_set_mode_client=self._srv_set_mode,
            avatar_state_subscription=self._avatar_state_sub,
        )
        # Replace NoOpBridge на реальный (после создания обоих).
        self.ws_server.bridge = self.bridge
        if log_pin:
            self.get_logger().warning(
                f"🔑 Quest PIN: {ACTIVE_PIN} "
                "(show this to operator — required to start a session)"
            )

        # CameraProvider — capture-loop в отдельных потоках (depthai/OpenCV).
        # Пока не доступны в dev-env, на роботе (Phase 1.6) добавятся.
        # camera_ceiling здесь больше нет: /dev/video0 держит контейнер
        # `ceiling-camera` (usb_cam) и в этот контейнер устройство не
        # прокинуто вовсе — capture-поток только писал в лог «cannot open
        # /dev/video0» и умирал. Потолочная камера теперь ROS-стрим, см.
        # `_on_ceiling_image` и streams/registry.py.
        cameras = [
            ("camera_oak_color", "oak:color", 15.0),
            ("camera_oak_depth", "oak:depth", 5.0),
        ]
        self._camera_provider = CameraProvider(cameras=cameras)
        for ui_name, source_id, _fps in cameras:
            self.bridge._camera_id_to_ui[source_id] = ui_name
        self._camera_provider.set_callback(self.bridge.on_camera_frame)

        # Запускаем aiohttp в отдельном thread (rclpy и aiohttp —
        # оба event-loop; запускать aiohttp в rclpy callback'е нельзя).
        self._aio_thread: Optional[threading.Thread] = None
        self._aio_loop: Optional[asyncio.AbstractEventLoop] = None
        self._stop_event = threading.Event()

        # Периодический tick 30 Гц (publish cmd_vel_quest пока свежо).
        self._tick_timer = self.create_timer(PUBLISH_PERIOD_S, self._on_tick_timer)
        # Watchdog check (раз в 100 мс).
        self._watchdog_timer = self.create_timer(0.1, self._on_watchdog_timer)
        # robot_status (1 Hz).
        self._status_timer = self.create_timer(1.0, self._on_status_timer)
        # AV-26 / R7: robot_alert evaluation — 1 Hz, чтобы не было мигающего
        # тоста на границе порога. Минимум, который нужен чтобы hold_ms=10 с
        # отрабатывал с точностью до 1 с (10..11 с).
        self._alert_timer = self.create_timer(1.0, self._on_alert_timer)
        # map_2d: поза робота на карте (5 Гц). Сама решётка приходит редко и
        # едет в PNG (см. Bridge.on_map), а вот пол под оператором обязан
        # ехать вместе с роботом — поэтому лёгкий кадр «только поза» шлём
        # чаще карты. Он ~130 байт, это дешевле, чем гонять PNG.
        self._map_pose_timer = self.create_timer(0.2, self._on_map_pose_timer)

        # tf map → base_link: единственный источник позы робота на карте.
        # /rtabmap/localization_pose для этого не годится — на роботе он
        # объявлен сразу двумя типами (PoseStamped и PoseWithCovarianceStamped),
        # подписка на такой топик неоднозначна.
        try:
            from tf2_ros import Buffer, TransformListener  # локальный импорт

            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        except Exception as e:  # noqa: BLE001  # pragma: no cover
            self.get_logger().warning(f"tf2 unavailable — map_2d без позы: {e}")
            self._tf_buffer = None
            self._tf_listener = None

        # Запуск aiohttp отложен до first timer callback (rclpy init
        # уже произошёл к этому моменту).
        self._aio_started = False
        self._camera_started = False

    # --- ROS callbacks ----------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg
        # Phase 1.4: подпитка StatusAggregator для robot_status payload.
        try:
            self._status.update_velocity(
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f"status update failed: {e}")
        # AV-26: обновить счётчик «сколько секунд робот не двигался»
        # для ROBOT_STUCK detector.
        try:
            self._odom_motion_tracker.update(
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                time.monotonic(),
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f"odom motion tracker update failed: {e}")

    def _on_scan(self, msg: LaserScan) -> None:
        """ROS subscription /scan → WS-подписчикам (Phase 1.4 v2)."""
        self.bridge.on_lidar_scan(msg)

    def _on_camera_image(self, msg: CompressedImage) -> None:
        """ROS /camera/camera/color/image_raw/compressed → WS (camera_rear).

        image_transport уже отдаёт JPEG-байты (format="bgr8; jpeg compressed"),
        поэтому перекодирование не нужно — форвардим msg.data как есть. Кадры
        шлются только сессиям, подписанным на camera_rear.
        """
        if not msg.data:
            return
        self.bridge.publish_frame("camera_rear", bytes(msg.data))

    def _on_ceiling_image(self, msg: CompressedImage) -> None:
        """ROS /ceiling_camera/image_raw/compressed → WS (camera_ceiling)."""
        if not msg.data:
            return
        self.bridge.publish_frame("camera_ceiling", bytes(msg.data))

    def _on_map(self, msg: OccupancyGrid) -> None:
        """ROS /rtabmap/map → map_2d (0x1103): PNG решётки + поза робота."""
        self.bridge.on_map(msg, self._map_pose())

    def _map_pose(self) -> Optional[tuple[float, float, float]]:
        """Поза робота на карте из tf ``map → base_link``: (x, y, yaw).

        ``None``, если tf ещё не собрался (карта тогда не показывается —
        класть её «куда-нибудь» хуже, чем не класть вовсе).
        """
        if self._tf_buffer is None:
            return None
        try:
            import rclpy.time

            tr = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            ).transform
        except Exception:  # noqa: BLE001 — TF ещё не готов / нет цепочки
            return None
        q = tr.rotation
        # Плоский робот: берём только yaw из кватерниона.
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (tr.translation.x, tr.translation.y, yaw)

    def _on_map_pose_timer(self) -> None:
        """5 Гц: лёгкий map_2d-кадр «только поза» (без PNG)."""
        self.bridge.publish_map_pose(self._map_pose())

    def _on_dialogue_state(self, msg: String) -> None:
        """ROS /voice/dialogue/state → msgpack voice_state (0x1202) → WS.

        Шлём каждый переход FSM (event-driven, drop-newest — см.
        meta-quest-api.md §4 frequency policy). Тело — простое:
        нормализация (таблица DialogueStateKind → bridge state),
        stamp server time, encode_voice_state, broadcast_frame.

        Никаких проверок «не повторять тот же state» — ROS уже сам
        не публикует дублей при неизменном состоянии, а если
        upstream пришлёт — безопасно переслать (клиент сам
        дедуплицирует по ts_ms+state).
        """
        normalized = normalize_voice_state(msg)
        self._last_voice_state = normalized
        payload = encode_voice_state(
            state=normalized["state"],
            ts_ms=int(time.time() * 1000),
            detail=normalized["detail"],
        )
        try:
            self.bridge.publish_frame("voice_state", payload)
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f"voice_state publish failed: {e}")

    def _on_tts_voices(self, msg: String) -> None:
        """ROS /voice/tts/voices (TRANSIENT_LOCAL, latched) → bridge кэш.

        Источник истины для voice_list payload. Приходит ОДИН раз на старте
        (latched от tts_node) + на каждое переключение provider.
        """
        self.bridge.on_voices_message(msg)

    def _on_tts_provider_state(self, msg: String) -> None:
        """ROS /voice/tts/provider_state → bridge.active_provider/active_voice.

        SoT для активного провайдера и голоса; инвалидирует voices-кэш
        если провайдер сменился (см. design t_5b9d5d0c §128-150).
        """
        self.bridge.on_provider_state_message(msg)

    def _on_preview_result(self, msg: String) -> None:
        """ROS /avatar/preview_voice/result (String JSON) → ws_server.deliver_preview_done.

        ``done`` — финальный preview_voice_done для клиента. Чистим
        pending и шлём JSON_EVENT. В MVP supervisor никогда не публикует
        done (только error) — но контракт есть, и через этот канал
        будущие карточки добавят success-путь.
        """
        try:
            data = json.loads(msg.data) if msg.data else None
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        request_id = data.get("request_id")
        if not isinstance(request_id, str):
            return
        # WS-серверный поток: ws_server уже зарегистрировал request_id в
        # start_preview_session → здесь просто достаём и шлём done.
        try:
            self.ws_server.deliver_preview_done(request_id)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"preview_result deliver failed: {exc}")

    def _on_preview_audio(self, msg: String) -> None:
        """ROS /avatar/preview_voice/audio (String JSON, audio_b64) → WS audio.

        payload: ``{request_id, format, content_type, audio_b64, seq, total}``.
        audio_b64 декодируется в raw bytes и отправляется отдельным
        BINARY_FRAME после JSON_EVENT-метаданных. В MVP не используется
        (supervisor отвечает только error) — канал готов для followup.
        """
        try:
            data = json.loads(msg.data) if msg.data else None
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        request_id = data.get("request_id")
        if not isinstance(request_id, str):
            return
        audio_format = data.get("format") or "mp3"
        content_type = data.get("content_type") or f"audio/{audio_format}"
        seq = int(data.get("seq") or 0)
        total = int(data.get("total") or 1)
        b64 = data.get("audio_b64") or ""
        try:
            import base64

            audio_bytes = base64.b64decode(b64) if b64 else b""
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"preview_audio decode failed: {exc}")
            audio_bytes = b""
        try:
            self.ws_server.deliver_preview_audio(
                request_id=request_id,
                audio_bytes=audio_bytes,
                audio_format=str(audio_format),
                content_type=str(content_type),
                seq=seq,
                total=total,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"preview_audio deliver failed: {exc}")

    def _on_preview_error(self, msg: String) -> None:
        """ROS /avatar/preview_voice/error → ws_server.deliver_preview_error."""
        try:
            data = json.loads(msg.data) if msg.data else None
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        request_id = data.get("request_id")
        reason = data.get("reason") or "preview_error"
        if not isinstance(request_id, str):
            return
        try:
            self.ws_server.deliver_preview_error(request_id, str(reason))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"preview_error deliver failed: {exc}")

    def _on_tick_timer(self) -> None:
        if not self._aio_started:
            self._start_aiohttp()
            self._aio_started = True
        if not self._camera_started:
            try:
                self._camera_provider.start()
                self._camera_started = True
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"camera provider start failed: {e}")
        self.bridge.tick_publish(time.monotonic())

    def _on_watchdog_timer(self) -> None:
        # Edge-triggered: один раз на trip → один WARNING + один emergency.
        # Повторные тики до feed()/reset() — no-op (анти-спам).
        if self.bridge.watchdog_consume_trip(time.monotonic()):
            self.get_logger().warning(
                "🛑 Watchdog tripped — emergency stop (Quest client silent)"
            )
            self.bridge.publish_emergency()
            self.bridge.emergency_stop()

    def _on_battery_json(self, msg: String) -> None:
        """JSON-снапшот сенсор-борда (std_msgs/String) → battery_pct / battery_v."""
        import json as _json

        try:
            data = _json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        pct, volts = parse_battery_json(data)
        if pct is None:
            pct = voltage_to_pct(volts, self._battery_v_empty, self._battery_v_full)
        self._status.update_battery(pct=pct, volts=volts)

    def _on_vesc_state(self, msg: Any) -> None:
        """VESC state → напряжение пакета (единственный живой источник)."""
        volts = getattr(getattr(msg, "state", None), "voltage_input", None)
        if volts is None:
            return
        pct = voltage_to_pct(float(volts), self._battery_v_empty, self._battery_v_full)
        self._status.update_battery(pct=pct, volts=float(volts))

    def _on_avatar_state_msg(self, msg: String) -> None:
        """ROS subscription /avatar/state → Bridge.on_avatar_state.

        Делегируем в Bridge, который:
        1) Валидирует msgpack-формат через rob_box_supervisor.core.state.unpack (AV-14).
        2) Обновляет cache для ``Bridge.supervisor_state()``.
        3) Broadcast STATE_UPDATE в v2-сессии (через run_coroutine_threadsafe).
        """
        self.bridge.on_avatar_state(msg)

    def _on_status_timer(self) -> None:
        """1 Hz robot_status broadcast."""
        self._status.update_wifi(read_wifi_rssi(iface=self._wifi_iface))
        self._status.set_mode(self.bridge.current_mode(time.monotonic()))
        try:
            payload = self._status.payload()
            self.bridge.publish_frame("robot_status", payload)
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f"status publish failed: {e}")

    def _on_alert_timer(self) -> None:
        """1 Hz robot_alert evaluation → broadcast только на изменении
        (acceptance: «60 одинаковых тиков → 1 событие»).

        Контракт (meta-quest-api.md §6 + AV-26 acceptance):
          - поднятие алёрта → ``JSON_EVENT{type:"robot_alert", active:true,
            level, code, args, ts_ms}``;
          - снятие → ``JSON_EVENT{type:"robot_alert", active:false,
            level:"info", code, args:{...}, ts_ms}`` (тот же код, чтобы
            клиент мог матчить без хранения prev state).

        ``ts_ms`` в payload — wall-clock из ROS-таймера; ``since_ms``
        у Alert'а (от evaluate_alerts) — внутреннее, для hold/hysteresis,
        в payload не идёт (приватная деталь реализации).
        """
        now_monotonic = time.monotonic()
        now_ms = int(now_monotonic * 1000)
        cmd_linear, cmd_angular = self.bridge.last_cmd_twist()
        odom_motion_value = self._odom_motion_tracker.seconds_since_last_motion(
            now_monotonic
        )

        new_alerts = evaluate_alerts(
            now_ms=now_ms,
            thresholds=self._alert_thresholds,
            battery_pct=(
                self._status.battery_pct
                if self._status.battery_pct is not None
                and self._status.battery_pct >= 0
                else None
            ),
            wifi_rssi=(
                self._status.wifi_rssi
                if self._status.wifi_rssi is not None and self._status.wifi_rssi != 0
                else None
            ),
            cmd_vel_linear=cmd_linear,
            cmd_vel_angular=cmd_angular,
            odom_motion_s=odom_motion_value,
            prev_alerts=self._active_alerts,
        )

        # Diff new_alerts vs self._active_alerts: для поднятия — broadcast
        # с active=True; для снятия — broadcast с active=False для тех, что
        # были в prev но не попали в new. Симметричный контракт упрощает
        # клиент (не нужен set-tracking).
        prev_codes = {a.code for a in self._active_alerts}
        new_codes = {a.code for a in new_alerts}
        # Поднятия.
        for alert in new_alerts:
            self._send_alert_event(alert, active=True)
        # Снятия.
        prev_by_code = {a.code: a for a in self._active_alerts}
        for code in prev_codes - new_codes:
            cleared = prev_by_code[code]
            self._send_alert_event(cleared, active=False)
        self._active_alerts = new_alerts

    def _send_alert_event(self, alert: Alert, *, active: bool) -> None:
        """Сформировать JSON_EVENT для robot_alert и разослать всем сессиям."""
        payload = {
            "type": "robot_alert",
            "code": alert.code,
            "active": bool(active),
            # При снятии шлём level="info" (мета-quest-api.md §6 — явный
            # формат снятия через level). Поднятия — warn/error.
            "level": alert.level if active else "info",
            "args": dict(alert.args),
            "ts_ms": int(time.time() * 1000),
        }
        try:
            self.ws_server.broadcast_json_event(payload)
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f"robot_alert broadcast failed: {e}")

    # --- aiohttp lifecycle ------------------------------------------------

    def _start_aiohttp(self) -> None:
        if self._aio_thread is not None:
            return

        from aiohttp import web as _aiohttp_web

        app = build_app(self.ws_server)

        def _runner() -> None:
            self._aio_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._aio_loop)
            # Потокобезопасная отправка BINARY_FRAME: ROS-поток шлёт кадры
            # через этот loop (иначе _schedule_send молча теряет их).
            self.ws_server.set_send_loop(self._aio_loop)
            # AV-16: тот же loop для STATE_UPDATE broadcast из
            # Bridge.on_avatar_state (см. supervisor-N-callback → ws).
            self.bridge.set_aio_send_loop(self._aio_loop)
            runner = _aiohttp_web.AppRunner(app)
            self._aio_loop.run_until_complete(runner.setup())
            # reuse_port=True — устойчивость к stale-процессам в host-network
            # (Vision Pi: при не-чистом redeploy предыдущий процесс может
            # удерживать сокет в TIME_WAIT; SO_REUSEPORT позволяет новому
            # bind'у пройти и сразу же отвечать на healthcheck. Без этого
            # рискуем повторить OSError [Errno 98] EADDRINUSE из
            # test-round-232 (см. issue #1650, t_4d530162).
            site = _aiohttp_web.TCPSite(
                runner,
                str(self.get_parameter("ws_host").value),
                int(self.get_parameter("ws_port").value),
                reuse_port=True,
            )
            self._aio_loop.run_until_complete(site.start())
            self.get_logger().info(
                f"🌐 Quest WSS server listening on "
                f"{self.get_parameter('ws_host').value}:"
                f"{self.get_parameter('ws_port').value}/quest"
            )
            try:
                self._aio_loop.run_forever()
            finally:
                self._aio_loop.run_until_complete(runner.cleanup())

        self._aio_thread = threading.Thread(
            target=_runner, name="quest-aiohttp", daemon=True
        )
        self._aio_thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        try:
            self._camera_provider.stop()
        except Exception as e:  # noqa: BLE001
            log.debug("camera provider stop: %s", e)
        if self._aio_loop is not None and self._aio_loop.is_running():
            self._aio_loop.call_soon_threadsafe(self._aio_loop.stop)
        if self._aio_thread is not None:
            self._aio_thread.join(timeout=2.0)


def main(args: Optional[list[str]] = None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = QuestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
