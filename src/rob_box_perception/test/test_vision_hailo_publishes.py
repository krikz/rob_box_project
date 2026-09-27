#!/usr/bin/env python3
"""e2e-регрессия issue #2602: нода vision_hailo реально публикует события.

Баг: ``vision_hailo_node._poll_gaze`` (колбэк таймера) вызывал блокирующий
``gaze.frames()``, который внутри делал ``rclpy.spin_once`` на той же ноде —
рекурсивный spin блокировал executor, поэтому таймер публикации ``_tick``
не срабатывал. Нода выглядела здоровой (publisher count=1, чистый лог,
низкий CPU), но не публиковала ни одного события.

Тест гоняет НАСТОЯЩУЮ ноду с настоящим rclpy:
  1. Поднимаем фейковый издатель ``sensor_msgs/Image`` на топик камеры
     ``/camera/camera/color/image_raw`` (фоновый поток, ~20 Гц).
  2. Конструируем ``VisionHailoNode`` (stub-loader, gaze_source='oak_d').
  3. Подписываемся на ``/vision/hailo/events``.
  4. Крутим executor до первого события (максимум ~3 × stub_period_sec).
  5. Assert: хотя бы одно сообщение получено.

На багнутом коде спиновая петля виснет в рекурсивном spin_once (поэтому
петля крутится в daemon-потоке, а главный поток ждёт с таймаутом и
фейлит, если событий нет). На пофикшенном коде первый stub-событие
приходит почти сразу (StubHEFLoader emit'ит на первом же infer).

Требует rclpy + rob_box_perception_msgs → запускается только в colcon
test CI (локально без ROS2 — skip через importorskip).
"""

from __future__ import annotations

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')


def _make_image_msg():
    """320x240 RGB-кадр (rgb8) — валидный sensor_msgs/Image."""
    import numpy as np  # type: ignore[import-not-found]

    from sensor_msgs.msg import Image  # type: ignore[import-not-found]

    img = Image()
    img.header.frame_id = 'camera_color_optical_frame'
    img.height = 240
    img.width = 320
    img.encoding = 'rgb8'
    img.step = 320 * 3
    img.data = np.full((240, 320, 3), 128, dtype=np.uint8).tobytes()
    return img


def test_vision_hailo_publishes_stub_events_when_source_alive():
    """Нода публикует в /vision/hailo/events, пока источник кадра жив.

    Используем дефолтные параметры ноды (gaze_source='oak_d',
    hailo_enabled=False → stub-loader). Фейковая камера заставляет
    ``OakDSource._on_msg`` наполнять ``_latest``, а фикс issue #2602
    должен позволить ``_tick`` доехать до публикации.
    """
    if not rclpy.ok():
        rclpy.init()

    from rob_box_perception_msgs.msg import VisionEvent  # type: ignore[import-not-found]
    from sensor_msgs.msg import Image  # type: ignore[import-not-found]

    from rob_box_perception.vision_hailo_node import VisionHailoNode

    # --- Фейковая камера ДО конструирования ноды: wait_for_first_frame
    # внутри __init__ ждёт первый кадр, и он должен успеть прийти.
    cam_node = rclpy.create_node('test_cam_pub')
    cam_pub = cam_node.create_publisher(
        Image, '/camera/camera/color/image_raw', 10
    )
    stop_cam = threading.Event()

    def _pump_camera() -> None:
        img = _make_image_msg()
        while not stop_cam.is_set():
            img.header.stamp = cam_node.get_clock().now().to_msg()
            cam_pub.publish(img)
            stop_cam.wait(0.05)

    cam_thread = threading.Thread(target=_pump_camera, daemon=True)
    cam_thread.start()

    node = VisionHailoNode()

    received: list = []
    sub_node = rclpy.create_node('test_event_sub')
    sub_node.create_subscription(
        VisionEvent, '/vision/hailo/events',
        lambda msg: received.append(msg), 10,
    )

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(cam_node)
    executor.add_node(sub_node)

    deadline = time.monotonic() + 3.0 * node.stub_period_sec

    # Спин в daemon-потоке: на багнутом коде spin_once может зависнуть
    # в рекурсивном spin'е, поэтому главный поток ждёт с таймаутом.
    def _spin() -> None:
        while time.monotonic() < deadline and not received:
            executor.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()
    spin_thread.join(timeout=3.0 * node.stub_period_sec + 2.0)

    stop_cam.set()
    cam_thread.join(timeout=1.0)

    try:
        assert received, (
            'issue #2602 regression: нода не опубликовала ни одного '
            f'события в /vision/hailo/events за ~{3.0 * node.stub_period_sec:.1f}s '
            '(executor блокирован рекурсивным spin_once?).'
        )
    finally:
        for n in (node, sub_node, cam_node):
            try:
                n.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        if rclpy.ok():
            rclpy.shutdown()
