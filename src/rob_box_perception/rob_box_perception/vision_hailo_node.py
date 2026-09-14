#!/usr/bin/env python3
"""vision_hailo_node — AI HAT+ 26 TOPS (Hailo-8) inference node (ADR-0089).

Подписывается на ROS-топики с изображениями, прогоняет их через pre-compiled
HEF (Hailo Executable Format) модели на NPU, публикует результат как
`VisionEvent` массив на `/vision/hailo/events`.

Архитектура:
  [Camera topic] -> pre-process -> [HailoRT VDevice] -> post-process -> /vision/hailo/events
                                                                          |
                                                                          v
                                                          [context_aggregator_node]
                                                          -> PerceptionEvent.vision_events_json
                                                          -> [mcp_server.py] -> LLM

Два режима работы (выбирается через launch-параметры):
  1. **real** (`hailo_enabled=True` + HEF file present): загружает HEF через
     `hailort` Python binding (требует hardware + driver + TAPPAS).
  2. **stub** (`hailo_enabled=False` или HEF missing): детерминированный
     цикл публикации с тестовыми `VisionEvent` сообщениями. Используется
     в CI (нет железа) и для smoke-теста на production до установки HEF.

Важно: stub **не претендует** на то, что он детектирует что-то реальное.
Он публикует сценарий "person detected at 1m" каждые `stub_period_sec`
секунд для тестирования downstream-pipeline.

HEFLoader / Stub / Real / make_loader / normalize_event_dict /
filter_by_confidence — в отдельном модуле `vision_hailo_loader.py`
(без rclpy зависимости, тестируется в CI без colcon-build).

Touchpoints:
  - ADR-0089 §3 (touchpoint #3) — этот файл.
  - ROS msg: rob_box_perception_msgs/VisionEvent
  - Aggregator: context_aggregator_node.py (подписка на /vision/hailo/events)
  - Launch: launch/internal_dialogue.launch.py (gated hailo_enabled)

Hardware reference: https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html
Hailo model zoo:    https://github.com/hailo-ai/hailo_model_zoo
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node

from rob_box_perception.vision_hailo_loader import (
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


# Порог confidence ниже которого события НЕ публикуются.
# Этот параметр общий для всех event_type — HEF-specific tuning
# делается через `hailo_models.yaml` в Phase 2/3.
DEFAULT_CONFIDENCE_THRESHOLD = 0.5


class VisionHailoNode(Node):
    """AI HAT+ inference node (ADR-0089 Phase 1).

    Параметры:
        hailo_enabled (bool, default False): включить реальный HEF loader.
            В CI/стенде без железа оставляем False — узел работает как
            stub и публикует детерминированные VisionEvent для
            проверки downstream-pipeline.
        hef_path (str, default ""): путь к .hef файлу. Пустой = stub.
        stub_period_sec (float, default 2.0): период stub-событий.
        confidence_threshold (float, default 0.5): фильтр confidence.
        input_topic (str, default "/oak/rgb/image_raw/compressed"):
            откуда брать кадры (Phase 1 — OAK-D).
        output_topic (str, default "/vision/hailo/events"): куда слать.
        publish_when_no_input (bool, default True): публиковать stub-события
            даже когда нет входящих кадров (важно для smoke-теста).
    """

    def __init__(self) -> None:
        super().__init__('vision_hailo')

        # ============ Параметры ============
        self.declare_parameter('hailo_enabled', False)
        self.declare_parameter('hef_path', '')
        self.declare_parameter('stub_period_sec', 2.0)
        self.declare_parameter('confidence_threshold', DEFAULT_CONFIDENCE_THRESHOLD)
        self.declare_parameter('input_topic', '/oak/rgb/image_raw/compressed')
        self.declare_parameter('output_topic', '/vision/hailo/events')
        self.declare_parameter('publish_when_no_input', True)

        self.hailo_enabled = bool(self.get_parameter('hailo_enabled').value)
        hef_path_param = str(self.get_parameter('hef_path').value).strip()
        self.hef_path = hef_path_param or None
        self.stub_period_sec = float(self.get_parameter('stub_period_sec').value)
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.publish_when_no_input = bool(
            self.get_parameter('publish_when_no_input').value
        )

        # ============ HEF loader ============
        self._loader = make_loader(
            hailo_enabled=self.hailo_enabled,
            hef_path=self.hef_path,
            stub_period_sec=self.stub_period_sec,
        )

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

        # ============ Subscribers ============
        # Phase 1: одна подписка (OAK-D compressed). Phase 2 добавит
        # depth topic для distance fusion.
        self._has_received_frame = False
        self._last_frame_id: Optional[str] = None
        if VisionEventMsg is not None:
            from sensor_msgs.msg import CompressedImage  # type: ignore
            self.create_subscription(
                CompressedImage,
                self.input_topic,
                self._on_image,
                10,
            )
            self.get_logger().info(
                f'Subscribed to {self.input_topic} (CompressedImage)'
            )
        else:
            self.get_logger().warning(
                'Подписка на image топик отключена — нет VisionEvent.msg'
            )

        # ============ Таймер публикации (stub heartbeat) ============
        # В stub-режиме публикуем события периодически, чтобы downstream
        # (context_aggregator) видел активность. В real-режиме — публикация
        # управляется callback'ом `_on_image`, таймер нужен только для
        # `publish_when_no_input`.
        timer_period = max(0.1, self.stub_period_sec / 4.0)
        self._timer = self.create_timer(timer_period, self._tick)

        mode = 'real' if self.hailo_enabled and self.hef_path else 'stub'
        self.get_logger().info(
            f'Vision Hailo node started (mode={mode}, '
            f'loader={type(self._loader).__name__}, '
            f'output_topic={self.output_topic}, '
            f'confidence_threshold={self.confidence_threshold})'
        )

    # ----------------------------------------------------------------
    # Lifecycle hooks
    # ----------------------------------------------------------------

    def _on_image(self, msg: Any) -> None:
        """Callback входящего кадра. Phase 1 — только фиксируем факт.

        В stub-режиме payload не нужен — инференс выдаёт детерминированный
        результат по таймеру. В real-режиме здесь будет вызов
        `self._loader.infer(...)` с `numpy.frombuffer(msg.data)`.
        """
        self._has_received_frame = True
        self._last_frame_id = (
            msg.header.frame_id if hasattr(msg, 'header') else None
        )

    def _tick(self) -> None:
        """Периодический тик: публикация VisionEvent от loader'а.

        В stub-режиме `_loader.infer(...)` сам решает, когда emit'ить
        (через `stub_period_sec`). В real-режиме здесь стоит вызывать
        `infer` на последнем кадре (Phase 1.5).
        """
        if self._publisher is None:
            return
        # В real-режиме ждём хотя бы один кадр (если не стоит
        # `publish_when_no_input=True` явно — для тестов удобно).
        if not self.publish_when_no_input and not self._has_received_frame:
            return

        frame_id = self._last_frame_id or 'unknown'
        try:
            raw_events = self._loader.infer(frame_id=frame_id, image=None)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'HEF loader failed: {exc!r}. Falling back to no-events.'
            )
            return

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
        msg.source_camera = norm['source_camera']
        msg.event_type = norm['event_type']
        msg.class_name = norm['class_name']
        msg.class_id = norm['class_id']
        msg.confidence = norm['confidence']
        msg.bbox_cx = norm['bbox_cx']
        msg.bbox_cy = norm['bbox_cy']
        msg.bbox_w = norm['bbox_w']
        msg.bbox_h = norm['bbox_h']
        msg.distance_m = norm['distance_m']
        msg.embedding_id = norm['embedding_id']
        msg.display_name = norm['display_name']
        msg.attributes_json = norm['attributes_json']
        self._publisher.publish(msg)


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
