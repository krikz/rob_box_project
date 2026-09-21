#!/usr/bin/env python3
"""vision_face.launch.py — декларативный запуск vision_face_node (ADR-0089 Phase 2).

Шим над общей фабрикой ``rob_box_perception.launch_factory.make_hailo_node_launch``
(ADR-0110 + issue #2658). Вся launch-конструкция
(``DeclareLaunchArgument`` × 9, ``OpaqueFunction`` pre-flight, ``Node``)
объявляется ровно в одном месте — в ``launch_factory.py``. Этот файл
только фиксирует отличия vision_face от vision_hailo:

- executable/name = ``vision_face`` (RetinaFace HEF),
- confidence_threshold по умолчанию 0.6 (retinaface score range),
- есть ``nms_iou_threshold`` (default 0.45, single-class),
- нет legacy ``input_topic`` (никогда не объявлялся для face),
- preflight WARN префикс ``vision_face``.

Capability-honest (ADR-0018): при hailo_enabled=true, но без /dev/hailo0
или HEF — нода деградирует в stub-режим с WARN (pre-flight не блокирует).

Touchpoints:
- ADR-0089 §3 touchpoint #3 (vision_face_node как Phase 2 дополнение).
- ADR-0110 §2.1 (vision_hailo.launch.py SSoT — теперь shared factory).
- Issue #2599 PR-A (face events в /vision/hailo/events).
- Issue #2658 (DRY-рефакторинг launch-файлов).
- docker/vision/config/hailo_models.yaml (SSoT параметров).
"""

from launch import LaunchDescription

from rob_box_perception.launch_factory import make_hailo_node_launch


def generate_launch_description() -> LaunchDescription:
    """Сформировать launch-описание vision_face_node."""
    return make_hailo_node_launch(
        executable='vision_face',
        confidence_threshold_default='0.6',
        include_nms_iou=True,
        include_input_topic=False,
        preflight_prefix='vision_face',
    )
