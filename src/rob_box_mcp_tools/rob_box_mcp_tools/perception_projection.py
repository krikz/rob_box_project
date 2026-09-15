#!/usr/bin/env python3
"""perception_projection.py — что из ``PerceptionEvent`` видит Личность.

Зачем модуль (issue #2532):
    ``PerceptionEvent`` — сообщение из 27 полей. До LLM через
    ``get_perception_context`` доезжали 2 из них, а ещё два поля читались
    по несуществующим именам под ``hasattr``-гвардами и молча давали ноль:

      * ``msg.battery_percentage`` — в IDL поле зовётся ``battery_voltage``
        → ``update_battery()`` не вызывался ни разу за всё время жизни ноды,
        и ``get_battery_level`` всегда отвечал «Данные о батарее недоступны».
      * ``msg.timestamp`` — в IDL поле зовётся ``stamp`` (и это
        ``builtin_interfaces/Time``, а не float) → в контекст уходил ``0.0``.

    ``hasattr``-гвард превращает расхождение имён в тихий ноль: код не падает,
    в логах пусто, а данных нет. Поэтому здесь гвардов нет — поля читаются
    напрямую, и любой дрейф IDL ловится тестом паритета
    (``test/test_perception_projection.py``), а не глазами на проде.

Контракт:
    ``PROJECTED_FIELDS`` + ``EXCLUDED_FIELDS`` обязаны в объединении давать
    ПОЛНЫЙ набор полей ``PerceptionEvent.msg``. Это значит, что добавление
    нового поля в IDL заставляет автора явно решить, видит его Личность или
    нет, — забыть нельзя, тест покраснеет.

    Тот же приём, что у ``VISION_EVENT_FIELDS`` в
    ``rob_box_perception/vision_hailo_loader.py`` + ``test_vision_event_parity.py``.

Чистый Python: ни rclpy, ни ROS-типов. Тестируется без железа и без colcon.
"""

from __future__ import annotations

from typing import Any, Dict, Tuple


# ---------------------------------------------------------------------------
# Батарея: вольты → проценты
# ---------------------------------------------------------------------------
# ``PerceptionEvent.battery_voltage`` — напряжение бортовой шины в вольтах
# (context_aggregator_node берёт его из /dynamic_joint_states, ключ
# ``battery/voltage``). Границы — по docs/architecture/HARDWARE.md: 10S LiPo,
# 42 В заряжена (10 × 4.2), 30 В cutoff (10 × 3.0).
#
# Важно: агрегатор пишет 0.0, когда данных о батарее ещё не было
# (``current_sensors.get('battery', 0.0)``). Ноль вольт на 10S — не «разряжена»,
# а «нет данных», и трактуется именно так (см. ``voltage_to_percent``).
BATTERY_VOLTAGE_FULL: float = 42.0
BATTERY_VOLTAGE_EMPTY: float = 30.0


def voltage_to_percent(volts: float) -> float | None:
    """Вольты бортовой шины → проценты 0..100.

    Возвращает ``None``, если данных нет (``volts <= 0``) — это отличается от
    «разряжена в ноль». Значения вне диапазона зажимаются в 0..100.
    """
    if volts is None or volts <= 0.0:
        return None
    span = BATTERY_VOLTAGE_FULL - BATTERY_VOLTAGE_EMPTY
    pct = (float(volts) - BATTERY_VOLTAGE_EMPTY) / span * 100.0
    return max(0.0, min(100.0, pct))


# ---------------------------------------------------------------------------
# Контракт проекции
# ---------------------------------------------------------------------------

#: Поля ``PerceptionEvent.msg``, которые видит Личность.
PROJECTED_FIELDS: Tuple[str, ...] = (
    'stamp',
    'vision_context',
    'is_moving',
    'battery_voltage',
    'temperature',
    'apriltag_ids',
    'system_health_status',
    'health_issues',
    'current_time_human',
    'time_period',
    'internet_available',
    'failed_nodes',
    'missing_nodes',
    'mapping_mode',
    'memory_summary',
)

#: Поля, которые Личность НЕ видит, и почему. Причина обязательна: если поле
#: исключено «просто так», это не решение, а забывчивость.
EXCLUDED_FIELDS: Dict[str, str] = {
    'pose': 'есть отдельный тул get_current_pose — не дублируем в контексте',
    'velocity': 'сырой Twist бесполезен Личности; факт движения даёт is_moving',
    'time_context_json': (
        'полностью дублирует current_time_human + time_period, '
        'лишний объём в промпте'
    ),
    'active_nodes': (
        'длинный список имён нод — шум для Личности; что сломано, '
        'говорят failed_nodes/missing_nodes'
    ),
    'equipment_summary_json': "всегда '{}' — заглушка Stage 2, продюсера нет",
    'speech_summaries': 'мёртвое поле: ни один продюсер его не заполняет',
    'robot_response_summaries': 'мёртвое поле: ни один продюсер его не заполняет',
    'robot_thought_summaries': 'мёртвое поле: ни один продюсер его не заполняет',
    'vision_summaries': 'мёртвое поле; ADR-0089 резервирует его под Phase 3 scene-graph',
    'system_summaries': 'мёртвое поле: ни один продюсер его не заполняет',
    # --- приватность, ADR-0089 §2.2 -------------------------------------
    # Заглушка vision_hailo (StubHEFLoader) публикует ВЫДУМАННОЕ событие
    # "person, conf 0.92, 1 м" каждые stub_period_sec, и отличить его от
    # реальной детекции сейчас нечем: event_type == 'person' (не 'stub'),
    # а source_camera == 'unknown' (frame_id подставляется в _tick).
    # Пока честного маркера нет, эти два поля к LLM подключать НЕЛЬЗЯ —
    # иначе Личность начнёт рассказывать про несуществующего человека рядом.
    # Снять исключение можно только вместе с маркером: issue #2538 / #2531.
    'vision_event_count': (
        'приватность ADR-0089 §2.2: у стаб-событий нет честного маркера '
        '(см. #2538, #2531) — даже счётчик выдаёт выдуманные детекции'
    ),
    'vision_events_json': (
        'приватность ADR-0089 §2.2: стаб публикует event_type="person" и '
        'source_camera="unknown" — отличить выдуманное событие от реального '
        'нечем (см. #2538, #2531)'
    ),
}


def _stamp_to_unix(stamp: Any) -> float:
    """``builtin_interfaces/Time`` → unix-timestamp (float).

    Поле называется ``stamp``, а не ``timestamp``, и это структура
    ``{sec, nanosec}``, а не число, — именно на этом ломался прежний код.
    """
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def project_perception_event(msg: Any) -> Dict[str, Any]:
    """``PerceptionEvent`` → словарь, который увидит Личность.

    Без ``hasattr``: если IDL переименует поле, здесь будет громкий
    ``AttributeError``, а не тихий ноль в контексте.
    """
    battery_voltage = float(msg.battery_voltage)
    return {
        'timestamp': _stamp_to_unix(msg.stamp),
        'vision_context': msg.vision_context,
        'is_moving': bool(msg.is_moving),
        'battery_voltage': battery_voltage,
        'battery_percent': voltage_to_percent(battery_voltage),
        'temperature': float(msg.temperature),
        'apriltag_ids': list(msg.apriltag_ids),
        'system_health_status': msg.system_health_status,
        'health_issues': list(msg.health_issues),
        'current_time_human': msg.current_time_human,
        'time_period': msg.time_period,
        'internet_available': bool(msg.internet_available),
        'failed_nodes': list(msg.failed_nodes),
        'missing_nodes': list(msg.missing_nodes),
        'mapping_mode': msg.mapping_mode,
        'memory_summary': msg.memory_summary,
    }
