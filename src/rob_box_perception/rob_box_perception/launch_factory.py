"""Shared launch-фрабрика для vision_hailo / vision_face (ADR-0110 + issue #2658).

DRY-рефакторинг для двух near-identical launch-файлов
``launch/vision_hailo.launch.py`` и ``launch/vision_face.launch.py``.
Раньше оба файла повторяли одну и ту же ``LaunchDescription``-конструкцию
(``DeclareLaunchArgument`` × 9 + ``OpaqueFunction`` pre-flight + ``Node``)
и отличались только:

- именем ``executable`` / ``name`` ноды,
- дефолтом ``confidence_threshold`` (0.5 vs 0.6),
- наличием/отсутствием ``nms_iou_threshold`` аргумента,
- наличием/отсутствием legacy ``input_topic`` аргумента,
- префиксом pre-flight WARN-сообщений (``vision_hailo.preflight`` vs
  ``vision_face.preflight``).

Сейчас эта общая launch-конструкция существует ровно в одном месте —
здесь. ``grep -nE 'DeclareLaunchArgument' src/rob_box_perception/launch/``
должен теперь возвращать 0 вхождений (все аргументы объявляются через
фабрику). Сами launch-файлы превращаются в 10-20-строчные шимы,
которые вызывают ``make_hailo_node_launch(...)`` с нужными дефолтами.

Где живёт модуль:
  - Не в ``launch/``, потому что ``setup.py`` устанавливает
    ``launch/*.launch.py`` как ``data_files`` (через ``ament_index``),
    а не как Python-пакет. ``importlib.util``-хак через путь к файлу —
    уродлив и хрупок при сборке/тестах.
  - В ``rob_box_perception/`` (``find_packages`` подхватывает), и
    существующий паттерн ``from rob_box_perception import X`` уже
    работает между модулями этого пакета
    (см. ``vision_face_node.py``, ``vision_hailo_node.py``).

Touchpoints:
- ADR-0110 §2.1 (vision_hailo.launch.py SSoT).
- ADR-0089 §3 touchpoint #6 (vision_face как Phase 2 дополнение).
- Issue #2599 PR-A (face detection acceptance).
- Issue #2658 (этот рефакторинг).
"""

from __future__ import annotations

import os
from typing import Any, Dict, List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# SSoT defaults. Совпадают с дефолтами в vision_hailo_node.py,
# vision_face_node.py и docker/vision/config/hailo_models.yaml.
# Поднимать только когда меняется поведение ноды или YAML.
#
# ADR-0104: ``gaze_source`` — ЕДИНСТВЕННЫЙ выбор источника кадра.
# ``input_topic`` — legacy, сохранён для back-compat, но нодой не
# используется.
_DEFAULTS: Dict[str, Any] = {
    'hailo_enabled': 'false',
    'hef_path': '',
    'stub_period_sec': '2.0',
    'gaze_source': 'oak_d',
    'first_frame_timeout_sec': '10.0',
    # Legacy / back-compat (только для vision_hailo): нода НЕ использует
    # input_topic напрямую, источник выбирается по gaze_source
    # (см. gaze.py → OakDSource и т.д.).
    'input_topic': '/camera/camera/color/image_raw',
    'output_topic': '/vision/hailo/events',
    'publish_when_no_input': 'true',
    # issue #2703/#2704: пустая строка = нода сама вычисляет
    # /tmp/{executable}_heartbeat (см. utils.heartbeat.default_heartbeat_path).
    'heartbeat_path': '',
}


def _preflight_check(
    context, preflight_prefix: str, *args, **kwargs
) -> List[Any]:
    """Pre-flight check для capability-honest mode (ADR-0018).

    Если ``hailo_enabled=true``, логируем доступность:
    - /dev/hailo0 (PCIe device file),
    - hef_path (если указан — readable ли файл),
    - numpy/cv2/hailo_platform (Python deps для real mode).

    Не блокирует запуск — это только информационный WARN (узел сам
    деградирует в stub-mode если что-то отсутствует, см.
    ``vision_hailo_node.py:_is_real_mode``). Префикс в WARN-сообщениях
    выбирается через ``preflight_prefix`` (``vision_hailo`` /
    ``vision_face``), чтобы оператор сразу видел, какая нода
    деградирует.
    """
    hailo_enabled = LaunchConfiguration('hailo_enabled').perform(context)
    hef_path = LaunchConfiguration('hef_path').perform(context)

    messages: List[str] = []
    if hailo_enabled.lower() == 'true':
        if not os.path.exists('/dev/hailo0'):
            messages.append(
                f'{preflight_prefix}.preflight: /dev/hailo0 отсутствует — '
                'нода деградирует в stub-режим '
                '(ADR-0018 capability-honest).'
            )
        if hef_path and not os.path.isfile(hef_path):
            messages.append(
                f'{preflight_prefix}.preflight: hef_path={hef_path!r} '
                'не является файлом — нода деградирует в stub-режим.'
            )
        for dep in ('numpy', 'cv2', 'hailo_platform'):
            try:
                __import__(dep)
            except ImportError:
                messages.append(
                    f'{preflight_prefix}.preflight: {dep} не установлен — '
                    'нода деградирует в stub-режим.'
                )

    for msg in messages:
        print(f'[WARN] {msg}')
    return []


def _make_preflight_opaque(preflight_prefix: str) -> OpaqueFunction:
    """Закрытие ``preflight_prefix`` в ``OpaqueFunction`` через default-arg.

    ``OpaqueFunction(function=_preflight_check)`` передаёт ``function``
    как keyword. Чтобы пробросить ``preflight_prefix`` без глобалов
    и без ``functools.partial`` (который launch иногда плохо сериализует
    в Substitutions), используем замыкание с default-arg.
    """
    def _bound(context, *args, **kwargs):
        return _preflight_check(context, preflight_prefix, *args, **kwargs)
    return OpaqueFunction(function=_bound)


def make_hailo_node_launch(
    *,
    executable: str,
    confidence_threshold_default: str,
    preflight_prefix: str,
    include_nms_iou: bool = False,
    include_input_topic: bool = False,
) -> LaunchDescription:
    """Собрать LaunchDescription для vision_* ноды (ADR-0110, issue #2658).

    Единая точка объявления launch-аргументов + pre-flight gate +
    Node(parameters=[...]) для всех vision_hailo / vision_face launch-файлов.

    Параметры:
      executable: имя консольного скрипта (== имя ноды в ROS graph,
        используется и в Node(executable=...), и в Node(name=...)).
      confidence_threshold_default: строковый дефолт ('0.5' для hailo,
        '0.6' для face — у retinaface другой score range).
      preflight_prefix: префикс WARN-сообщений capability-honest gate
        ('vision_hailo' / 'vision_face').
      include_nms_iou: True для vision_face (retinaface одиночный класс,
        NMS нужен), False для vision_hailo (yolo, NMS — внутри HEF).
      include_input_topic: True для vision_hailo (legacy/back-compat
        аргумент, ADR-0104), False для vision_face (никогда не объявлялся).
    """
    args: List[Any] = [
        DeclareLaunchArgument(
            'hailo_enabled',
            default_value=_DEFAULTS['hailo_enabled'],
            description='Включить real HEF inference (требует /dev/hailo0 + HEF). '
                        'False = stub-режим.',
        ),
        DeclareLaunchArgument(
            'hef_path',
            default_value=_DEFAULTS['hef_path'],
            description='Путь к .hef файлу. Игнорируется если hailo_enabled=false.',
        ),
        DeclareLaunchArgument(
            'stub_period_sec',
            default_value=_DEFAULTS['stub_period_sec'],
            description='Период публикации stub-событий (секунды). '
                        'В real-mode не используется.',
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value=confidence_threshold_default,
            description='Фильтр confidence [0.0, 1.0]. События ниже порога drop.',
        ),
    ]
    if include_nms_iou:
        args.append(DeclareLaunchArgument(
            'nms_iou_threshold',
            default_value='0.45',
            description='IoU порог NMS (single-class).',
        ))
    args.extend([
        DeclareLaunchArgument(
            'gaze_source',
            default_value=_DEFAULTS['gaze_source'],
            description=(
                'Имя адаптера шва «Взгляд» (ADR-0104): '
                'oak_d | ceiling_camera | stub. oak_d подписывается на '
                '/camera/camera/color/image_raw (Image msg, согласовано '
                'с oak_d_config.yaml i_rs_compat:true).'
            ),
        ),
        DeclareLaunchArgument(
            'first_frame_timeout_sec',
            default_value=_DEFAULTS['first_frame_timeout_sec'],
            description=(
                'Сколько секунд ждать первый кадр от real-источника '
                'перед fail-fast в real-mode (ADR-0104, capability-honest).'
            ),
        ),
    ])
    if include_input_topic:
        args.append(DeclareLaunchArgument(
            'input_topic',
            default_value=_DEFAULTS['input_topic'],
            description=(
                'LEGACY / back-compat. Нода НЕ использует — выбор '
                'источника делается по gaze_source. Сохранён, чтобы '
                'старые скрипты / документация не сломались.'
            ),
        ))
    args.extend([
        DeclareLaunchArgument(
            'output_topic',
            default_value=_DEFAULTS['output_topic'],
            description='Публикация VisionEvent[] (consumed by context_aggregator).',
        ),
        DeclareLaunchArgument(
            'publish_when_no_input',
            default_value=_DEFAULTS['publish_when_no_input'],
            description='В stub-режиме публиковать события даже без входящих '
                        'кадров (true = smoke-test, false = production).',
        ),
        DeclareLaunchArgument(
            'heartbeat_path',
            default_value=_DEFAULTS['heartbeat_path'],
            description=(
                'Путь к файлу-heartbeat живости (issue #2703/#2704). Пустая '
                'строка = авто /tmp/{executable}_heartbeat. Обновляется '
                'после каждого успешного infer() — healthcheck-скрипт в '
                'контейнере проверяет его возраст вместо ros2 topic echo.'
            ),
        ),
    ])

    node_parameters: Dict[str, Any] = {
        'hailo_enabled': LaunchConfiguration('hailo_enabled'),
        'hef_path': LaunchConfiguration('hef_path'),
        'stub_period_sec': LaunchConfiguration('stub_period_sec'),
        'confidence_threshold': LaunchConfiguration('confidence_threshold'),
        'gaze_source': LaunchConfiguration('gaze_source'),
        'first_frame_timeout_sec': LaunchConfiguration(
            'first_frame_timeout_sec'
        ),
        'output_topic': LaunchConfiguration('output_topic'),
        'publish_when_no_input': LaunchConfiguration('publish_when_no_input'),
        'heartbeat_path': LaunchConfiguration('heartbeat_path'),
    }
    if include_nms_iou:
        node_parameters['nms_iou_threshold'] = LaunchConfiguration(
            'nms_iou_threshold'
        )

    return LaunchDescription([
        *args,
        # Pre-flight check (ADR-0018 capability-honest gate).
        _make_preflight_opaque(preflight_prefix),
        # Node.
        Node(
            package='rob_box_perception',
            executable=executable,
            name=executable,
            output='screen',
            parameters=[node_parameters],
        ),
    ])
