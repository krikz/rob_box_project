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
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .core.safety import Watchdog
from .core.teleop import TeleopController
from .server.session import WATCHDOG_TIMEOUT_S as SESSION_WATCHDOG_TIMEOUT_S
from .server.ws_server import NoOpBridge, WSSServer, build_app
from .streams.camera import image_to_payload
from .streams.lidar import scan_to_payload
from .streams.provider import CameraFrame, CameraProvider
from .streams.registry import STREAM_CATALOG
from .streams.status import StatusAggregator

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

        log_pin = bool(self.get_parameter("log_pin").value)

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
        # Камера: OAK-D (depthai_ros_driver v2) публикует sensor_msgs/Image
        # как BEST_EFFORT (SENSOR_DATA QoS) — RELIABLE-подписка не матчится.
        _CAMERA_QOS = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
        # Подписки для стримов (Phase 1.4 v2: lidar + camera_rear-фолбэк через
        # ROS; остальные камеры — мимо ROS через CameraProvider).
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, _RE)
        self._scan_sub = self.create_subscription(LaserScan, "/scan", self._on_scan, _RE)
        # camera_rear (0x1001): OAK-D color /camera/camera/color/image_raw → JPEG.
        # ROS-фолбэк к bypass-ROS CameraProvider (прямой depthai в quest-контейнер
        # невозможен: USB-устройство занято контейнером oak-d).
        self._camera_rear_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self._on_camera_image, _CAMERA_QOS
        )
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

    def _on_camera_image(self, msg: Image) -> None:
        """ROS /camera/camera/color/image_raw → JPEG → WS (camera_rear).

        Phase 1.4 baseline: cv_bridge → cv2.imencode('.jpg') → BINARY_FRAME.
        Ошибку кодирования (нет cv_bridge/cv2, неподдерживаемый encoding)
        логируем на debug и пропускаем кадр — нода не падает. Кадры шлются
        только сессиям, подписанным на camera_rear (broadcast_frame no-op
        если подписчиков нет).
        """
        try:
            payload = image_to_payload(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f"camera_rear encode failed: {e}")
            return
        self.bridge.publish_frame("camera_rear", payload)

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

    def _on_status_timer(self) -> None:
        """1 Hz robot_status broadcast."""
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
