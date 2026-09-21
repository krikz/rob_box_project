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
- preflight WARN префикс ``vision_face``,
- ``extra_params=FACE_RECOGNITION_PARAMS`` — узнавание лица (ArcFace +
  FaceStore, ADR-0123, issue #2599 PR-B): без этого объявления
  ``ros2 launch`` отклонил бы ``arcface_enabled:=...`` и другие
  ``key:=value``, которые подставляет ``start_vision_face.sh`` из
  ENV/YAML (см. комментарий у ``FACE_RECOGNITION_PARAMS`` ниже).

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

#: Параметры узнавания лица (issue #2599 PR-B, ADR-0123).
#:
#: Дефолты здесь намеренно «выключено»: SSoT значений —
#: ``docker/vision/config/hailo_models.yaml`` (секция ``vision_face_node``),
#: откуда их достаёт ``start_vision_face.sh`` и передаёт в launch. Если
#: конфига нет (голый ``ros2 launch`` на столе), нода поднимается как
#: чистый детектор — узнавание не включается «само» и уж точно не пишет
#: на диск лица тех, кто не ожидал, что их запомнят.
FACE_RECOGNITION_PARAMS = {
    'arcface_enabled': 'false',
    'arcface_hef_path': '',
    # ADR-0123 §4: том, который переживает пересоздание контейнера.
    'face_store_root': '/data/faces',
    # ADR-0123 §2: режим приватности. Единственное место, где он
    # проверяется, — FaceStore; остальной код его не читает.
    'face_privacy_mode': 'workshop',
    # НЕ откалиброван: стартовое значение. Порог обязан калиброваться
    # таблицей sweep на реальных данных, как голосовой в #2348, а не
    # «на глаз» (ADR-0123 §6).
    'face_identify_threshold': '0.45',
    # Правило Встречи (ADR-0123 §3): мелькнувшее лицо встречей не станет.
    'min_track_sec': '2.0',
    'min_face_px': '48.0',
    # Бюджет NPU: ArcFace ~230-260 мс на лицо, тик ноды 0.5 с.
    'max_embeds_per_frame': '2',
    'max_embeddings': '20',
    'keep_encounters': '10',
    'max_strangers': '500',
}


def generate_launch_description() -> LaunchDescription:
    """Сформировать launch-описание vision_face_node."""
    return make_hailo_node_launch(
        executable='vision_face',
        confidence_threshold_default='0.6',
        include_nms_iou=True,
        include_input_topic=False,
        preflight_prefix='vision_face',
        extra_params=FACE_RECOGNITION_PARAMS,
    )
