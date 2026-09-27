#!/usr/bin/env python3
"""vision_hailo_node — AI HAT+ 26 TOPS (Hailo-8) inference node (ADR-0089).

Подписывается на ROS-топики с изображениями через шов «Взгляд» (gaze.py,
ADR-0104, issue #2531), прогоняет их через pre-compiled HEF (Hailo
Executable Format) модели на NPU, публикует результат как `VisionEvent`
массив на `/vision/hailo/events`.

Архитектура (ADR-0104):
  [Camera topic] --(Gaze source)--> [Frame rgb+letterbox-info]
                                     --> [HEF loader]
                                     --> [post-process with unproject]
                                     --> /vision/hailo/events
                                                       |
                                                       v
                                          [context_aggregator_node]
                                          -> PerceptionEvent.vision_events_json
                                          -> [mcp_server.py] -> LLM

Два режима работы (выбирается через launch-параметры):
  1. **real** (`hailo_enabled=True` + HEF file present): загружает HEF
     через `hailort` Python binding (требует hardware + driver + TAPPAS).
  2. **stub** (`hailo_enabled=False` или HEF missing): детерминированный
     цикл публикации с тестовыми `VisionEvent` сообщениями. Используется
     в CI (нет железа) и для smoke-теста на production до установки HEF.

Важно: stub **не претендует** на то, что он детектирует что-то реальное.
Он публикует сценарий "person detected at 1m" каждые `stub_period_sec`
секунд для тестирования downstream-pipeline.

Шов «Взгляд» (gaze.py) скрывает: выбор ROS-топика и типа сообщения,
JPEG/PNG decode, BGR→RGB. Нода НЕ подписывается на ROS-топики напрямую —
это ADR-0104 acceptance #1: единый шов источника кадра.

HEFLoader / Stub / Real / make_loader / normalize_event_dict /
filter_by_confidence — в отдельном модуле `vision_hailo_loader.py`
(без rclpy зависимости, тестируется в CI без colcon-build).

Touchpoints:
- ADR-0089 §3 (touchpoint #3) — этот файл.
- ADR-0104 (gaze.py, единый шов источника кадра).
- ADR-0110 (launch-файл vision_hailo.launch.py — выбор gaze_source).
- ROS msg: rob_box_perception_msgs/VisionEvent
- Aggregator: context_aggregator_node.py (подписка на /vision/hailo/events)

Hardware reference: https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html
Hailo model zoo:    https://github.com/hailo-ai/hailo_model_zoo
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node

from rob_box_perception.gaze import (
    GazeSourceUnavailable,
    make_source,
)
from rob_box_perception.utils.heartbeat import (
    FileHeartbeat,
    default_heartbeat_path,
)
from rob_box_perception.vision_hailo_loader import (
    HEFLoader,
    VISION_EVENT_FIELDS,
    filter_by_confidence,
    make_loader,
    normalize_event_dict,
)

try:
    from rob_box_perception_msgs.msg import VisionEvent as VisionEventMsg
except ImportError:
    # Fallback для случая когда пакет ещё не собран через colcon —
    # позволяет импортировать модуль без workspace build (для тестов).
    VisionEventMsg = None


# Сколько секунд ждать первый кадр от gaze source (ADR-0104, capability-
# honest). Если источник недоступен — нода либо fail-fast (когда
# hailo_enabled=True), либо деградирует с WARN и переходит в stub.
DEFAULT_FIRST_FRAME_TIMEOUT_SEC = 10.0

# Сколько секунд без нового кадра считать источник «мёртвым» в real-mode.
# Используется только для лога; healthcheck делается на уровне docker-compose
# (см. ADR-0104 acceptance #6 — healthcheck проверяет свежесть /vision/hailo/events).
DEFAULT_FRAME_STALE_SEC = 30.0

# Порог confidence ниже которого события НЕ публикуются.
# Этот параметр общий для всех event_type — HEF-specific tuning
# делается через `hailo_models.yaml` в Phase 2/3.
DEFAULT_CONFIDENCE_THRESHOLD = 0.5

# Допустимые имена gaze source (ADR-0104, см. gaze.make_source).
KNOWN_GAZE_SOURCES = ('oak_d', 'ceiling_camera', 'stub')


class VisionHailoNode(Node):
    """AI HAT+ inference node (ADR-0089 Phase 1 + ADR-0104).

    Параметры:
        hailo_enabled (bool, default False): включить реальный HEF loader.
            В CI/стенде без железа оставляем False — узел работает как
            stub и публикует детерминированные VisionEvent для
            проверки downstream-pipeline.
        hef_path (str, default ""): путь к .hef файлу. Пустой = stub.
        stub_period_sec (float, default 2.0): период stub-событий.
        confidence_threshold (float, default 0.5): фильтр confidence.
        gaze_source (str, default "oak_d"): какой адаптер «Взгляд»
            использовать (см. gaze.py — OakDSource, CeilingCameraSource,
            StubSource). ADR-0104 acceptance #1: единственный шов выбора
            источника кадра.
        output_topic (str, default "/vision/hailo/events"): куда слать.
        first_frame_timeout_sec (float, default 10.0): сколько ждать
            первый кадр от real-источника перед fail-fast (capability-honest).
        publish_when_no_input (bool, default True): публиковать stub-события
            даже когда нет входящих кадров (важно для smoke-теста CI).
            В real-режиме НЕ рекомендуется — это mode-mask против capability-
            honest (ADR-0104 acceptance #5); в проде оставлять False.
        heartbeat_path (str, default ""): путь к файлу-heartbeat живости
            (issue #2703, #2704). Пустая строка = авто
            (``/tmp/{NODE_NAME}_heartbeat``, см.
            ``utils.heartbeat.default_heartbeat_path``). Обновляется ПОСЛЕ
            каждого успешного ``infer()`` — НЕ по факту публикации события,
            иначе пустая сцена (0 детекций, infer успешен) снова читается
            healthcheck'ом как смерть ноды. Docker healthcheck
            (``healthcheck_frame.sh`` / ``healthcheck_face_frame.sh``)
            проверяет возраст этого файла вместо ``ros2 topic echo``.
    """

    #: Имя ROS-ноды. Переопределяется сабклассом ``vision_face_node``
    #: (ADR-0089 Phase 2, issue #2599) без дублирования __init__.
    NODE_NAME: str = 'vision_hailo'

    #: Дефолтный confidence threshold (переопределяется сабклассом).
    DEFAULT_CONFIDENCE_THRESHOLD: float = 0.5

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)

        # ============ Параметры ============
        self.declare_parameter('hailo_enabled', False)
        self.declare_parameter('hef_path', '')
        self.declare_parameter('stub_period_sec', 2.0)
        self.declare_parameter('confidence_threshold', self.DEFAULT_CONFIDENCE_THRESHOLD)
        self.declare_parameter('gaze_source', 'oak_d')
        self.declare_parameter('output_topic', '/vision/hailo/events')
        self.declare_parameter('first_frame_timeout_sec', DEFAULT_FIRST_FRAME_TIMEOUT_SEC)
        self.declare_parameter('publish_when_no_input', True)
        # NMS IoU. Базовый YOLOv8n-нода не меняет дефолт; сабкласс
        # vision_face_node прокидывает его в make_face_loader.
        self.declare_parameter('nms_iou_threshold', 0.45)
        # Heartbeat живости (issue #2703, #2704). Пустая строка = авто
        # (/tmp/{NODE_NAME}_heartbeat).
        self.declare_parameter('heartbeat_path', '')

        self.hailo_enabled = bool(self.get_parameter('hailo_enabled').value)
        hef_path_param = str(self.get_parameter('hef_path').value).strip()
        self.hef_path = hef_path_param or None
        self.stub_period_sec = float(self.get_parameter('stub_period_sec').value)
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.nms_iou_threshold = float(
            self.get_parameter('nms_iou_threshold').value
        )
        self.gaze_source_name = str(self.get_parameter('gaze_source').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.first_frame_timeout_sec = float(
            self.get_parameter('first_frame_timeout_sec').value
        )
        self.publish_when_no_input = bool(
            self.get_parameter('publish_when_no_input').value
        )
        heartbeat_path_param = str(
            self.get_parameter('heartbeat_path').value
        ).strip()
        self.heartbeat_path = heartbeat_path_param or default_heartbeat_path(
            self.NODE_NAME
        )
        # issue #2703/#2704: heartbeat живости, независимый от детекций.
        # Обновляется в _tick() ПОСЛЕ успешного infer() (см. beat() ниже).
        self._heartbeat = FileHeartbeat(self.heartbeat_path)

        if self.gaze_source_name not in KNOWN_GAZE_SOURCES:
            raise ValueError(
                f'gaze_source={self.gaze_source_name!r} не из списка '
                f'{KNOWN_GAZE_SOURCES!r}. '
                f'Проверьте vision_hailo.launch.py и hailo_models.yaml.'
            )

        # ============ Режим: один расчёт, не два ============
        # ADR-0104 (issue #2531 acceptance #7): _is_real_mode — единственное
        # определение реального режима. Лог и фактическое поведение должны
        # использовать одну и ту же переменную. Старый код считал mode
        # двумя разными способами и мог лгать ("mode=real" при _is_real_mode=False).
        self._is_real_mode = bool(self.hailo_enabled and self.hef_path)

        # ============ HEF loader ============
        # Phase 1.5: stub vs real через make_loader (не зависит от gaze).
        # _make_loader() — точка расширения: сабкласс vision_face_node
        # переопределяет её под свой RetinaFaceLoader (ADR-0089 Phase 2).
        self._loader = self._make_loader()

        # ============ Шов «Взгляд» (ADR-0104) ============
        # Это ЕДИНСТВЕННОЕ место, где нода знает про ROS-топики и cv2-decode.
        # Все адаптеры скрыты за gaze.make_source().
        #
        # ADR-0104 acceptance #5: если real-источник не отдал кадр за
        # first_frame_timeout_sec, нода либо fail-fast (hailo_enabled=true),
        # либо явно логирует degraded-mode и переходит в stub.
        try:
            self._gaze = make_source(
                name=self.gaze_source_name,
                node=self,
                timeout_sec=self.first_frame_timeout_sec,
            )
        except GazeSourceUnavailable as exc:
            if self._is_real_mode:
                # В real-режиме источник обязан быть живым (ADR-0018
                # capability-honest) — fail-fast с понятным сообщением.
                self.get_logger().error(
                    f'vision_hailo: {exc}. fail-fast (real-mode обязателен, '
                    f'gaze_source={self.gaze_source_name!r}).'
                )
                raise
            # В stub-режиме — degradation с WARN, нода продолжит работу.
            self.get_logger().warning(
                f'vision_hailo: {exc}. Источник недоступен, нода продолжит '
                f'работу в stub-режиме (hailo_enabled=False).'
            )
            self._gaze = make_source(
                name='stub',
                node=self,
                timeout_sec=0.0,
            )

        # ============ Состояние кадров ============
        # Кэш последнего кадра от gaze для infer() в _tick. Содержит уже
        # декодированный RGB + метаданные letterbox (когда применимо).
        self._latest_frame: Optional[Any] = None  # gaze.Frame
        self._has_received_frame = False
        self._last_frame_id: Optional[str] = None
        self._last_frame_stamp: Optional[float] = None

        # ============ Publishers ============
        if VisionEventMsg is not None:
            self._publisher = self.create_publisher(
                VisionEventMsg, self.output_topic, 10
            )
        else:
            # Сборка пакета не завершена — не падаем, просто логируем
            # warning. Узел всё равно полезен: можно дёргать public API
            # для тестов без rclpy-serialization.
            self.get_logger().warning(
                'VisionEvent msg не найден — публикация отключена до '
                'сборки rob_box_perception_msgs через colcon.'
            )
            self._publisher = None

        # ============ Таймер поллинга gaze source ============
        # Non-blocking poll (issue #2602): кадр складывается ROS-колбэком
        # источника в поле _latest, а _poll_gaze лишь читает его через
        # poll_latest(). Никакого rclpy.spin_once из колбэка таймера —
        # рекурсивный spin блокировал executor и не давал _tick публиковать.
        self._poll_timer = self.create_timer(0.05, self._poll_gaze)

        # ============ Таймер публикации (heartbeat) ============
        # Stub-mode использует self.stub_period_sec; real-mode использует
        # фактический поток кадров через _poll_gaze → _tick.
        timer_period = max(0.1, self.stub_period_sec / 4.0)
        self._tick_timer = self.create_timer(timer_period, self._tick)

        # ============ Degraded state (issue #2538 п.6) ============
        # Если HailoRT init падает (AttributeError / RuntimeError), нода
        # НЕ должна спамить ERROR каждые stub_period_sec/4 секунды.
        # Логируем один раз, дальше — degraded + rate-limited.
        self._degraded_logged: bool = False
        self._consecutive_failures: int = 0
        self._max_logged_failures: int = 3  # потом молчим до восстановления

        # ============ Один и тот же mode в логах и в коде ============
        # ADR-0104 acceptance #7: лог НЕ может утверждать mode=real,
        # если _is_real_mode=False. Один расчёт, одна строка.
        mode = 'real' if self._is_real_mode else 'stub'
        self.get_logger().info(
            f'Vision Hailo node started '
            f'(mode={mode}, '
            f'loader={type(self._loader).__name__}, '
            f'gaze_source={self.gaze_source_name!r} → '
            f'topic={self._gaze.topic!r}, '
            f'output_topic={self.output_topic}, '
            f'confidence_threshold={self.confidence_threshold}, '
            f'publish_when_no_input={self.publish_when_no_input}, '
            f'heartbeat_path={self.heartbeat_path!r})'
        )

    # ----------------------------------------------------------------
    # Extension point
    # ----------------------------------------------------------------

    def _make_loader(self) -> HEFLoader:
        """Создать HEF loader по параметрам ноды (точка расширения).

        Сабкласс ``vision_face_node`` переопределяет под ``make_face_loader``
        (ADR-0089 Phase 2, issue #2599).
        """
        return make_loader(
            hailo_enabled=self.hailo_enabled,
            hef_path=self.hef_path,
            stub_period_sec=self.stub_period_sec,
            nms_iou_threshold=self.nms_iou_threshold,
        )

    # ----------------------------------------------------------------
    # Lifecycle hooks
    # ----------------------------------------------------------------

    def _poll_gaze(self) -> None:
        """Poll gaze source — сохранить последний кадр для _tick.

        В stub-mode _poll_gaze просто no-op (StubSource сам не публикует
        события, его выдачу обрабатывает _tick через _loader.infer(image=None)).
        В real-mode (oak_d / ceiling_camera) — читаем последний RGB-кадр
        + метаданные letterbox для следующего infer().

        Non-blocking (issue #2602): кадр обновляется ROS-колбэком источника
        (``_on_msg`` в gaze.py, вызывается executor'ом ноды), а здесь мы
        только читаем поле через ``poll_latest()``. Старый вызов блокирующего
        ``frames()`` крутил ``rclpy.spin_once`` из колбэка таймера — это
        рекурсивный spin, который блокировал executor и не давал ``_tick``
        публиковать события.
        """
        if self.gaze_source_name == 'stub':
            # StubSource отдаёт один синтетический кадр; считаем, что
            # "кадр пришёл" — для downstream-агрегатора это сигнал, что
            # узел жив (но детектор не настоящий, см. ADR-0018).
            self._has_received_frame = True
            self._last_frame_id = 'stub_frame'
            return

        frame = self._gaze.poll_latest()
        if frame is None:
            return
        self._latest_frame = frame
        self._has_received_frame = True
        self._last_frame_id = frame.frame_id
        self._last_frame_stamp = frame.stamp

    def _tick(self) -> None:
        """Периодический тик: публикация VisionEvent от loader'а.

        В stub-режиме `_loader.infer(...)` сам решает, когда emit'ить
        (через `stub_period_sec`).
        В real-режиме (Phase 1.5) — `infer` вызывается на последнем
        декодированном кадре (если есть).

        Degraded-state policy (issue #2538 п.6): если `infer` падает
        каждый тик (init-failure или per-frame-failure), логируем
        первые `_max_logged_failures` ошибок и дальше — пропускаем
        молча. Сбрасываем счётчик после успешного `infer`.

        ADR-0104 (issue #2531 acceptance #5): в проде
        `publish_when_no_input=False` обязателен, иначе маскируем
        реальный источник кадра.
        """
        if self._publisher is None:
            return
        # В real-режиме ждём хотя бы один кадр (если не стоит
        # `publish_when_no_input=True` явно — для CI/smoke удобно).
        # ADR-0104 acceptance #5: в проде `publish_when_no_input=False`
        # обязателен, иначе маскируем реальный источник.
        if not self.publish_when_no_input and not self._has_received_frame:
            return

        frame_id = self._last_frame_id or 'unknown'

        # Real-mode: передаём последний RGB-кадр от gaze (или None если
        # ещё ни одного). Stub-mode: image=None → _loader.infer сам знает,
        # что делать (heartbeat по stub_period_sec).
        image_for_infer = (
            self._latest_frame.rgb
            if (self._is_real_mode and self._latest_frame is not None)
            else None
        )

        try:
            raw_events = self._loader.infer(
                frame_id=frame_id,
                image=image_for_infer,
            )
        except Exception as exc:  # noqa: BLE001
            # Real-mode failure (capability-honest, ADR-0018).
            # НЕ silent fallback на stub: нода остаётся в degraded,
            # но НЕ спамит ERROR каждую ~0.5 c (см. issue #2538 п.6).
            self._consecutive_failures += 1
            if not self._degraded_logged:
                # Первый фейл — самый информативный (traceback виден).
                self.get_logger().error(
                    f'HEF loader failed: {exc!r}. '
                    f'Кадр пропущен. Нода переходит в degraded state '
                    f'(лог повторится до {self._max_logged_failures} раз).'
                )
                self._degraded_logged = True
            elif self._consecutive_failures <= self._max_logged_failures:
                # Следующие 2 фейла — короткий лог (без traceback).
                self.get_logger().warning(
                    f'HEF loader всё ещё failing '
                    f'({self._consecutive_failures}/{self._max_logged_failures}). '
                    f'Кадр пропущен.'
                )
            # Дальше — тишина до первого успеха (см. ниже).
            return

        # Успех — сброс счётчика и degraded-флага.
        if self._consecutive_failures > 0 or self._degraded_logged:
            self.get_logger().info(
                f'HEF loader восстановился после '
                f'{self._consecutive_failures} подряд фейлов.'
            )
        self._consecutive_failures = 0
        self._degraded_logged = False

        # issue #2703/#2704: heartbeat — ПО ФАКТУ успешного infer(), а НЕ
        # по факту публикации события. Пустая сцена (raw_events == [] или
        # все ниже confidence_threshold) — это ЗДОРОВАЯ нода, а не мёртвая.
        # Если бы heartbeat стоял ниже (после filter_by_confidence + publish
        # loop), пустая комната снова красила бы контейнер unhealthy — ровно
        # регресс issue #2703.
        self._heartbeat.beat()

        filtered = filter_by_confidence(raw_events, self.confidence_threshold)
        for event_dict in filtered:
            self._publish_event(event_dict)

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------

    def _publish_event(self, event_dict: Dict[str, Any]) -> None:
        if self._publisher is None or VisionEventMsg is None:
            return
        msg = VisionEventMsg()
        msg.stamp = self.get_clock().now().to_msg()
        norm = normalize_event_dict(event_dict)
        for field in VISION_EVENT_FIELDS:
            setattr(msg, field, norm[field])
        self._publisher.publish(msg)

    def destroy_node(self) -> bool:
        """Остановить gaze source при destroy (важно для тестов)."""
        try:
            if self._gaze is not None:
                self._gaze.stop()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = VisionHailoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
