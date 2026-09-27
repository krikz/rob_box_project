#!/usr/bin/env python3
"""vision_hailo.launch.py — декларативный запуск vision_hailo_node (ADR-0110).

Шим над общей фабрикой ``rob_box_perception.launch_factory.make_hailo_node_launch``
(ADR-0110 + issue #2658). Этот файл — SSoT для запуска ``vision_hailo``
из docker-сервиса ``vision-hailo`` на Vision Pi (10.1.1.21,
см. ADR-0089 §1.1, ADR-0110 §1.2). Вся launch-конструкция
(``DeclareLaunchArgument`` × 9, ``OpaqueFunction`` pre-flight, ``Node``)
объявляется ровно в одном месте — в ``launch_factory.py``.

Файл сохраняет историческую сигнатуру аргументов (``input_topic`` legacy)
для back-compat со старыми bash-скриптами и документацией, но источник
кадра выбирается по ``gaze_source`` (ADR-0104).

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
- ADR-0110 §2.1 — этот файл (SSoT launch-параметров).
- ADR-0089 §3 touchpoint #6 — переформулирован ADR-0110.
- ADR-0104 — ``gaze_source`` parameter (single source of truth).
- Issue #2658 — общая factory с vision_face.launch.py.
"""

from launch import LaunchDescription

from rob_box_perception.launch_factory import make_hailo_node_launch


def generate_launch_description() -> LaunchDescription:
    """Сформировать launch-описание vision_hailo_node."""
    return make_hailo_node_launch(
        executable='vision_hailo',
        confidence_threshold_default='0.5',
        include_nms_iou=False,
        include_input_topic=True,
        preflight_prefix='vision_hailo',
    )
