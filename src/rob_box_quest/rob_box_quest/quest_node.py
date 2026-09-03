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
import threading
import time
from typing import Any, Optional

from audio_common_msgs.msg import AudioData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
from .streams.battery import parse_battery_json, voltage_to_pct
from .streams.lidar import scan_to_payload
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
        set_voice_pub=None,
        preview_voice_pub=None,
        voices_cache_ttl_sec: float = 300.0,
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
        # AV-27 / issue #1919 — set_voice / preview_voice → супервизор.
        self._set_voice_pub = set_voice_pub
        self._preview_voice_pub = preview_voice_pub
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
        self._node.get_logger().warning("🛑 EMERGENCY STOP from Quest client — publishing cmd_vel_emergency")
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
            self._set_voice_pub.publish(_string_msg(json.dumps(payload, ensure_ascii=False)))
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
            self._preview_voice_pub.publish(_string_msg(json.dumps(payload, ensure_ascii=False)))
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f"quest: preview_voice publish failed: {exc}")

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
        return "teleop_active" if self._teleop.tick(now_monotonic) is not None else "idle"

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

        # AV-27 / issue #1919 — TTL локального voices-кэша. По дизайн-доку
        # t_5b9d5d0c §128-150 default 300 (5 минут); 0 = «никогда не
        # протухает, всегда отдавать последний latched-payload».
        self.declare_parameter("voices_cache_ttl_sec", 300)

        log_pin = bool(self.get_parameter("log_pin").value)
        self._battery_v_empty = float(self.get_parameter("battery_voltage_empty").value)
        self._battery_v_full = float(self.get_parameter("battery_voltage_full").value)
        self._wifi_iface = str(self.get_parameter("wifi_iface").value) or None

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
        self._voice_in_pub = self.create_publisher(AudioData, "/avatar/voice_in", _VOICE_QOS)
        self._tts_control_pub = self.create_publisher(String, "/voice/tts/control", _RE)
        self._sound_stop_pub = self.create_publisher(String, "/voice/sound/stop", _RE)
        # Робот-голос (P7): буфер операторского PCM → STT через /audio/quest_in.
        self._stt_in_pub = self.create_publisher(AudioData, "/audio/quest_in", _VOICE_QOS)
        # voice_mode → супервизор (ADR-0028 S5): /avatar/set_voice_mode.
        self._set_voice_mode_pub = self.create_publisher(String, "/avatar/set_voice_mode", _RE)
        # AV-27 / issue #1919 — TTS picker: set_voice / preview_voice →
        # супервизор (ADR-0028 S5/S12 — никаких прямых SetParameters из
        # quest_node на tts_node). Топики std_msgs/String (JSON payload),
        # симметрично /avatar/set_voice_mode.
        self._set_voice_pub = self.create_publisher(String, "/avatar/set_voice", _RE)
        self._preview_voice_pub = self.create_publisher(String, "/avatar/preview_voice", _RE)
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
        # Подписки для стримов (Phase 1.4 v2: lidar + camera_rear-фолбэк через
        # ROS; остальные камеры — мимо ROS через CameraProvider).
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, _RE)
        self._scan_sub = self.create_subscription(LaserScan, "/scan", self._on_scan, _RE)
        # camera_rear (0x1001): OAK-D color JPEG (image_transport compressed) →
        # форвардим bytes as-is в WS. Лёгкий путь: без cv2/numpy/перекодирования,
        # сеть грузится ~300 KB/s вместо raw ~13.5 MB/s.
        self._camera_rear_sub = self.create_subscription(
            CompressedImage,
            "/camera/camera/color/image_raw/compressed",
            self._on_camera_image,
            _CAMERA_QOS,
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
        self._battery_json_sub = self.create_subscription(String, battery_topic, self._on_battery_json, _RE)
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
            self.get_logger().info("vesc_msgs unavailable — battery voltage from JSON snapshot only")
        self._latest_odom: Optional[Odometry] = None
        self._status = StatusAggregator()

        # WS server (инициализируем первым чтобы передать в Bridge).
        from .server.ws_server import ACTIVE_PIN

        self.ws_server = WSSServer(bridge=NoOpBridge(), pin=ACTIVE_PIN)
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
            set_voice_pub=self._set_voice_pub,
            preview_voice_pub=self._preview_voice_pub,
            voices_cache_ttl_sec=float(self.get_parameter("voices_cache_ttl_sec").value),
        )
        # Replace NoOpBridge на реальный (после создания обоих).
        self.ws_server.bridge = self.bridge
        if log_pin:
            self.get_logger().warning(
                f"🔑 Quest PIN: {ACTIVE_PIN} " "(show this to operator — required to start a session)"
            )

        # CameraProvider — capture-loop в отдельных потоках (depthai/OpenCV).
        # Пока не доступны в dev-env, на роботе (Phase 1.6) добавятся.
        cameras = [
            ("camera_oak_color", "oak:color", 15.0),
            ("camera_oak_depth", "oak:depth", 5.0),
            ("camera_ceiling", "/dev/video0", 15.0),
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
            self.get_logger().warning("🛑 Watchdog tripped — emergency stop (Quest client silent)")
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

    def _on_status_timer(self) -> None:
        """1 Hz robot_status broadcast."""
        self._status.update_wifi(read_wifi_rssi(iface=self._wifi_iface))
        self._status.set_mode(self.bridge.current_mode(time.monotonic()))
        try:
            payload = self._status.payload()
            self.bridge.publish_frame("robot_status", payload)
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f"status publish failed: {e}")

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

        self._aio_thread = threading.Thread(target=_runner, name="quest-aiohttp", daemon=True)
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
