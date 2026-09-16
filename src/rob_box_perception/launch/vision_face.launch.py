#!/usr/bin/env python3
"""vision_face.launch.py — декларативный запуск vision_face_node (ADR-0089 Phase 2).

Зеркалит ``vision_hailo.launch.py`` (ADR-0110) под лицевую ноду
``vision_face``: отдельный процесс, отдельный HEF (retinaface), но тот же
шов «Взгляд» (gaze.py) и тот же выходной топик ``/vision/hailo/events``
(issue #2599 PR-A acceptance: face-события видны там же).

Отличие от vision_hailo.launch.py:
  - executable/name = ``vision_face``,
  - confidence_threshold по умолчанию 0.6 (retinaface),
  - добавлен nms_iou_threshold (default 0.45).

Capability-honest (ADR-0018): при hailo_enabled=true, но без /dev/hailo0
или HEF — нода деградирует в stub-режим с WARN (pre-flight не блокирует).

Touchpoints:
- ADR-0089 §3 touchpoint #3 (vision_face_node как Phase 2 дополнение).
- Issue #2599 PR-A.
- docker/vision/config/hailo_models.yaml (SSoT параметров).
"""

from __future__ import annotations

from typing import Any, Dict

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rob_box_perception.preflight import make_preflight_check


_DEFAULTS: Dict[str, Any] = {
    'hailo_enabled': 'false',
    'hef_path': '',
    'stub_period_sec': '2.0',
    'confidence_threshold': '0.6',
    'nms_iou_threshold': '0.45',
    'gaze_source': 'oak_d',
    'first_frame_timeout_sec': '10.0',
    'output_topic': '/vision/hailo/events',
    'publish_when_no_input': 'true',
}


def generate_launch_description() -> LaunchDescription:
    """Сформировать launch-описание vision_face_node."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'hailo_enabled',
            default_value=_DEFAULTS['hailo_enabled'],
            description='Включить real HEF inference (retinaface). '
                        'False = stub-режим.',
        ),
        DeclareLaunchArgument(
            'hef_path',
            default_value=_DEFAULTS['hef_path'],
            description='Путь к retinaface_mobilenet_v1.hef.',
        ),
        DeclareLaunchArgument(
            'stub_period_sec',
            default_value=_DEFAULTS['stub_period_sec'],
            description='Период stub-событий (секунды).',
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value=_DEFAULTS['confidence_threshold'],
            description='Порог face-score [0.0, 1.0].',
        ),
        DeclareLaunchArgument(
            'nms_iou_threshold',
            default_value=_DEFAULTS['nms_iou_threshold'],
            description='IoU порог NMS (single-class).',
        ),
        DeclareLaunchArgument(
            'gaze_source',
            default_value=_DEFAULTS['gaze_source'],
            description='Имя адаптера шва «Взгляд» (ADR-0104): '
                        'oak_d | ceiling_camera | stub.',
        ),
        DeclareLaunchArgument(
            'first_frame_timeout_sec',
            default_value=_DEFAULTS['first_frame_timeout_sec'],
            description='Сколько ждать первый кадр перед fail-fast.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value=_DEFAULTS['output_topic'],
            description='Публикация VisionEvent[] (consumed by context_aggregator).',
        ),
        DeclareLaunchArgument(
            'publish_when_no_input',
            default_value=_DEFAULTS['publish_when_no_input'],
            description='В stub-режиме публиковать события без кадров.',
        ),

        OpaqueFunction(function=make_preflight_check('vision_face')),

        Node(
            package='rob_box_perception',
            executable='vision_face',
            name='vision_face',
            output='screen',
            parameters=[{
                'hailo_enabled': LaunchConfiguration('hailo_enabled'),
                'hef_path': LaunchConfiguration('hef_path'),
                'stub_period_sec': LaunchConfiguration('stub_period_sec'),
                'confidence_threshold': LaunchConfiguration(
                    'confidence_threshold'
                ),
                'nms_iou_threshold': LaunchConfiguration('nms_iou_threshold'),
                'gaze_source': LaunchConfiguration('gaze_source'),
                'first_frame_timeout_sec': LaunchConfiguration(
                    'first_frame_timeout_sec'
                ),
                'output_topic': LaunchConfiguration('output_topic'),
                'publish_when_no_input': LaunchConfiguration(
                    'publish_when_no_input'
                ),
            }],
        ),
    ])
