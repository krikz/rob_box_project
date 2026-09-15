#!/usr/bin/env python3
"""vision_hailo.launch.py — декларативный запуск vision_hailo_node (ADR-0096).

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
  ноды идёт через этот launch (см. ADR-0096 §2.2).

Touchpoints:
- ADR-0096 §3 touchpoint #1 — этот файл.
- ADR-0089 §3 touchpoint #6 — переформулирован ADR-0096 (vision_hailo.launch.py
  вместо internal_dialogue.launch.py).
"""

from __future__ import annotations

import os
from typing import Any, Dict, List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# SSoT defaults. Совпадают с дефолтами в vision_hailo_node.py и с дефолтами
# в docker/vision/config/hailo_models.yaml. Поднимать только когда меняется
# поведение ноды или YAML.
_DEFAULTS: Dict[str, Any] = {
    'hailo_enabled': 'false',
    'hef_path': '',
    'stub_period_sec': '2.0',
    'confidence_threshold': '0.5',
    'input_topic': '/oak/rgb/image_raw/compressed',
    'output_topic': '/vision/hailo/events',
    'publish_when_no_input': 'true',
}


def _preflight_check(context, *args, **kwargs) -> List[Any]:
    """Pre-flight check для capability-honest mode (ADR-0018).

    Если ``hailo_enabled=true``, логируем доступность:
    - /dev/hailo0 (PCIe device file)
    - hef_path (если указан — readable ли файл)
    - numpy/cv2/hailo_platform (Python deps для real mode)

    Не блокирует запуск — это только информационный WARN (узел сам
    деградирует в stub-mode если что-то отсутствует, см.
    vision_hailo_node.py:_is_real_mode).
    """
    hailo_enabled = LaunchConfiguration('hailo_enabled').perform(context)
    hef_path = LaunchConfiguration('hef_path').perform(context)

    messages: List[str] = []

    if hailo_enabled.lower() == 'true':
        if not os.path.exists('/dev/hailo0'):
            messages.append(
                'vision_hailo.preflight: /dev/hailo0 отсутствует — '
                'нода деградирует в stub-режим (ADR-0018 capability-honest).'
            )
        if hef_path and not os.path.isfile(hef_path):
            messages.append(
                f'vision_hailo.preflight: hef_path={hef_path!r} '
                'не является файлом — нода деградирует в stub-режим.'
            )
        # Python deps. Не фатально если их нет — узел логирует подробнее.
        for dep in ('numpy', 'cv2', 'hailo_platform'):
            try:
                __import__(dep)
            except ImportError:
                messages.append(
                    f'vision_hailo.preflight: {dep} не установлен — '
                    'нода деградирует в stub-режим.'
                )

    if messages:
        # Launch system сам выведет эти сообщения в stdout. Оператор видит
        # причину degraded-режима ДО старта ноды, что упрощает триаж.
        for msg in messages:
            print(f'[WARN] {msg}')

    return []


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
            'input_topic',
            default_value=_DEFAULTS['input_topic'],
            description='Подписка на compressed image (Phase 1: OAK-D).',
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
        OpaqueFunction(function=_preflight_check),

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
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'publish_when_no_input': LaunchConfiguration(
                    'publish_when_no_input'
                ),
            }],
        ),
    ])
