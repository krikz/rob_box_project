#!/usr/bin/env python3
"""vision_hailo.launch.py — декларативный запуск vision_hailo_node (ADR-0110).

Этот launch — SSoT параметров ноды ``vision_hailo``. Заменяет прямой вызов
``ros2 run rob_box_perception vision_hailo --ros-args ...`` в bash-скрипте
``docker/vision/scripts/vision-hailo/start_vision_hailo.sh``.

Почему отдельный launch-файл, а не Node в ``internal_dialogue.launch.py``:

- ``internal_dialogue.launch.py`` исполняется на host-dev-машине (без
  железного /dev/hailo0 в большинстве случаев).
- ``internal_dialogue_docker.launch.py`` исполняется на **Main Pi**
  (10.1.1.22), где HAT-слот занят CAN HAT (MCP2515) для VESC. /dev/hailo0
  на Main Pi **не существует физически** (ADR-0089 §1.1).
- ``vision-hailo`` Docker-сервис живёт на **Vision Pi** (10.1.1.21)
  в ``docker/vision/docker-compose.yaml`` — отдельный контейнер, доступ
  к /dev/hailo0 через ``devices:`` bind-mount.

Выбор источника кадра (ADR-0104, issue #2531):

Параметр ``gaze_source`` (NEW, ssoT) — имя адаптера шва «Взгляд»
(``oak_d`` / ``ceiling_camera`` / ``stub``). Нода НЕ подписывается на
ROS-топики напрямую — это делает ``rob_box_perception.gaze``. Подробнее:

- ``oak_d`` → подписка на ``/camera/camera/color/image_raw`` (Image msg),
  путь, который публикует ``oakd_with_apriltag.launch.py`` с
  ``namespace='camera'`` + ``i_rs_compat: true``.
- ``ceiling_camera`` → подписка на ``/ceiling_camera/image_raw/compressed``
  (CompressedImage msg). Действующие потребители: ``quest_node.py:2151``,
  ``telegram_node.py:107``. Это второй независимый сценарий, который
  делает шов «Взгляд» настоящим, а не гипотетическим.
- ``stub`` → синтетический кадр без ROS (для CI/smoke).

Параметр ``input_topic`` оставлен для back-compat с уже выпущенным
документированным контрактом, но НЕ используется нодой. Источник правды —
``gaze_source``. Issue #2531 acceptance #2: устранено тройное дублирование
дефолта ``/oak/rgb/image_raw/compressed``.

Capability-honest gate (ADR-0018): если ``hailo_enabled=true``, но
``/dev/hailo0`` отсутствует или HEF не читается — нода запускается в
degraded stub-режиме с WARN-логом (см. ``vision_hailo_node.py:_is_real_mode``).
Этот launch **не блокирует** старт в таких случаях, а только логирует
pre-flight check (через ``OpaqueFunction``), чтобы оператор видел причину
degraded-режима без необходимости курить node-логи.

Совместимость:
- Сохранена совместимость с ``docker/vision/config/hailo_models.yaml`` (SSoT).
- Bash entrypoint ``start_vision_hailo.sh`` остаётся orchestration layer
  (YAML-парсинг, ENV override, smoke hailortcli scan), но финальный запуск
  ноды идёт через этот launch (см. ADR-0110 §2.2).

Touchpoints:
- ADR-0110 §3 touchpoint #1 — этот файл.
- ADR-0089 §3 touchpoint #6 — переформулирован ADR-0110 (vision_hailo.launch.py
  вместо internal_dialogue.launch.py).
- ADR-0104 — gaze_source parameter (single source of truth).
"""

from __future__ import annotations

from typing import Any, Dict

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rob_box_perception.preflight import make_preflight_check


# SSoT defaults. Совпадают с дефолтами в vision_hailo_node.py и с дефолтами
# в docker/vision/config/hailo_models.yaml. Поднимать только когда меняется
# поведение ноды или YAML.
#
# ADR-0104: ``gaze_source`` — ЕДИНСТВЕННЫЙ выбор источника кадра.
# ``input_topic`` — legacy, сохранён для back-compat но нодой не используется.
_DEFAULTS: Dict[str, Any] = {
    'hailo_enabled': 'false',
    'hef_path': '',
    'stub_period_sec': '2.0',
    'confidence_threshold': '0.5',
    'gaze_source': 'oak_d',
    # Legacy / back-compat: нода НЕ использует input_topic напрямую,
    # источник выбирается по gaze_source (см. gaze.py → OakDSource и т.д.).
    # Оставлено, чтобы старые скрипты / документация не сломались; нода
    # просто игнорирует.
    'input_topic': '/camera/camera/color/image_raw',
    'output_topic': '/vision/hailo/events',
    'publish_when_no_input': 'true',
    'first_frame_timeout_sec': '10.0',
}


def generate_launch_description() -> LaunchDescription:
    """Сформировать launch-описание vision_hailo_node."""
    return LaunchDescription([
        # ============ Launch arguments ============
        DeclareLaunchArgument(
            'hailo_enabled',
            default_value=_DEFAULTS['hailo_enabled'],
            description='Включить real HEF inference (требует /dev/hailo0 + HEF). '
                        'False = stub-режим (deterministic VisionEvent для smoke-теста).',
        ),
        DeclareLaunchArgument(
            'hef_path',
            default_value=_DEFAULTS['hef_path'],
            description='Путь к .hef файлу. Игнорируется если hailo_enabled=false.',
        ),
        DeclareLaunchArgument(
            'stub_period_sec',
            default_value=_DEFAULTS['stub_period_sec'],
            description='Период публикации stub-событий (секунды). В real-mode не используется.',
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value=_DEFAULTS['confidence_threshold'],
            description='Фильтр confidence [0.0, 1.0]. События ниже порога drop.',
        ),
        DeclareLaunchArgument(
            'gaze_source',
            default_value=_DEFAULTS['gaze_source'],
            description='Имя адаптера шва «Взгляд» (ADR-0104): '
                        'oak_d | ceiling_camera | stub. oak_d подписывается на '
                        '/camera/camera/color/image_raw (Image msg, согласовано '
                        'с oak_d_config.yaml i_rs_compat:true).',
        ),
        DeclareLaunchArgument(
            'first_frame_timeout_sec',
            default_value=_DEFAULTS['first_frame_timeout_sec'],
            description='Сколько секунд ждать первый кадр от real-источника '
                        'перед fail-fast в real-mode (ADR-0104, capability-honest).',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value=_DEFAULTS['input_topic'],
            description='LEGACY / back-compat. Нода НЕ использует — выбор '
                        'источника делается по gaze_source. Сохранён, чтобы '
                        'старые скрипты / документация не сломались.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value=_DEFAULTS['output_topic'],
            description='Публикация VisionEvent[] (consumed by context_aggregator).',
        ),
        DeclareLaunchArgument(
            'publish_when_no_input',
            default_value=_DEFAULTS['publish_when_no_input'],
            description='В stub-режиме публиковать события даже без входящих кадров '
                        '(true = smoke-test, false = production).',
        ),

        # ============ Pre-flight check (ADR-0018) ============
        OpaqueFunction(function=make_preflight_check('vision_hailo')),

        # ============ Node ============
        Node(
            package='rob_box_perception',
            executable='vision_hailo',
            name='vision_hailo',
            output='screen',
            parameters=[{
                'hailo_enabled': LaunchConfiguration('hailo_enabled'),
                'hef_path': LaunchConfiguration('hef_path'),
                'stub_period_sec': LaunchConfiguration('stub_period_sec'),
                'confidence_threshold': LaunchConfiguration(
                    'confidence_threshold'
                ),
                'gaze_source': LaunchConfiguration('gaze_source'),
                'first_frame_timeout_sec': LaunchConfiguration(
                    'first_frame_timeout_sec'
                ),
                # input_topic — legacy, нода игнорирует (ADR-0104).
                'output_topic': LaunchConfiguration('output_topic'),
                'publish_when_no_input': LaunchConfiguration(
                    'publish_when_no_input'
                ),
            }],
        ),
    ])
