"""Паритет ``PerceptionEvent.msg`` со проекцией для Личности (issue #2532).

Что здесь проверяется:

 1. PARITY: ``PROJECTED_FIELDS`` + ``EXCLUDED_FIELDS`` покрывают все поля
    ``PerceptionEvent.msg``. Новое поле в IDL заставляет автора явно решить,
    видит его Личность или нет, — «забыть» нельзя, тест покраснеет.
    Тот же приём, что в ``test_vision_event_parity.py`` для VisionEvent.
 2. REGRESSION A: батарея доходит до контекста. Раньше читалось
    ``msg.battery_percentage`` (в IDL — ``battery_voltage``) под ``hasattr``
    → ``update_battery()`` не вызывался ни разу, тул всегда отвечал
    «Данные о батарее недоступны».
 3. REGRESSION B: ``timestamp`` не ноль. Раньше читалось ``msg.timestamp``
    (в IDL — ``stamp``, структура ``{sec, nanosec}``) → всегда 0.0.
 4. DRIFT: переименование поля в IDL даёт громкий ``AttributeError``,
    а не тихий ноль (гвардов ``hasattr`` в проекции быть не должно).
 5. PRIVACY: зрительные события не утекают в контекст Личности, пока у
    стаб-детекций нет честного маркера (ADR-0089 §2.2, #2538/#2531).

Запуск:
    cd src/rob_box_mcp_tools && python -m pytest test/test_perception_projection.py -v
"""

from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import Set

import pytest


# ---------- paths ----------------------------------------------------------

# Этот файл: src/rob_box_mcp_tools/test/test_perception_projection.py
#   parents[1] = src/rob_box_mcp_tools  -> содержит пакет rob_box_mcp_tools/
#   parents[2] = src                    -> содержит rob_box_perception_msgs/
_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

_PERCEPTION_EVENT_MSG = (
    _HERE.parents[2] / 'rob_box_perception_msgs' / 'msg' / 'PerceptionEvent.msg'
)

from rob_box_mcp_tools.perception_projection import (  # noqa: E402
    BATTERY_VOLTAGE_EMPTY,
    BATTERY_VOLTAGE_FULL,
    EXCLUDED_FIELDS,
    PROJECTED_FIELDS,
    project_perception_event,
    voltage_to_percent,
)


# ---------- helpers --------------------------------------------------------

_MSG_FIELD_RE = re.compile(
    r'^\s*(?P<type>[A-Za-z_][\w/]*(?:\[\d*\])?)\s+(?P<name>[a-z_][a-z0-9_]*)\s*$'
)


def _parse_msg_field_names(msg_path: Path) -> Set[str]:
    """Имена полей из .msg (текстовый IDL, без colcon build)."""
    names: Set[str] = set()
    for raw_line in msg_path.read_text(encoding='utf-8').splitlines():
        line = raw_line.split('#', 1)[0].rstrip()
        if not line.strip():
            continue
        m = _MSG_FIELD_RE.match(line)
        if m:
            names.add(m.group('name'))
    return names


class _Stamp:
    def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
        self.sec = sec
        self.nanosec = nanosec


class _FakePerceptionEvent:
    """Дублёр msg: те же имена полей, что в IDL, без ROS-зависимостей."""

    def __init__(self, **overrides) -> None:
        self.stamp = _Stamp(1_700_000_000, 500_000_000)
        self.vision_context = ''
        self.pose = object()
        self.velocity = object()
        self.is_moving = False
        self.battery_voltage = 38.4
        self.temperature = 41.5
        self.apriltag_ids = [7, 9]
        self.system_health_status = 'healthy'
        self.health_issues = []
        self.current_time_human = '2026-09-15 21:40:00'
        self.time_period = 'evening'
        self.time_context_json = '{"period": "evening"}'
        self.internet_available = True
        self.active_nodes = ['/a', '/b']
        self.failed_nodes = []
        self.missing_nodes = []
        self.equipment_summary_json = '{}'
        self.mapping_mode = 'localization'
        self.memory_summary = 'Денис спрашивал про погоду'
        self.speech_summaries = ''
        self.robot_response_summaries = ''
        self.robot_thought_summaries = ''
        self.vision_summaries = ''
        self.system_summaries = ''
        self.vision_event_count = 27
        self.vision_events_json = '[{"event_type": "person", "confidence": 0.92}]'
        for key, value in overrides.items():
            setattr(self, key, value)


# ---------- 1. parity ------------------------------------------------------


def test_projection_contract_covers_every_msg_field():
    """PROJECTED + EXCLUDED покрывают все поля PerceptionEvent.msg."""
    msg_fields = _parse_msg_field_names(_PERCEPTION_EVENT_MSG)
    covered = set(PROJECTED_FIELDS) | set(EXCLUDED_FIELDS)

    missing = msg_fields - covered
    extra = covered - msg_fields

    assert not missing and not extra, (
        'Контракт проекции разошёлся с PerceptionEvent.msg.\n'
        f'  Поля IDL без решения (добавь в PROJECTED_FIELDS либо в '
        f'EXCLUDED_FIELDS с причиной): {sorted(missing) or "нет"}\n'
        f'  Поля контракта, которых нет в IDL: {sorted(extra) or "нет"}\n'
        f'  Все поля .msg: {sorted(msg_fields)}'
    )


def test_projected_and_excluded_do_not_overlap():
    overlap = set(PROJECTED_FIELDS) & set(EXCLUDED_FIELDS)
    assert not overlap, (
        f'Поле не может быть одновременно видимым и скрытым: {sorted(overlap)}'
    )


def test_every_excluded_field_has_a_reason():
    """Исключение без причины — это забывчивость, а не решение."""
    empty = [name for name, reason in EXCLUDED_FIELDS.items() if not reason.strip()]
    assert not empty, f'Нет причины исключения для: {sorted(empty)}'


def test_field_counts_are_pinned():
    """Ловит тихое добавление/удаление поля с любой стороны."""
    msg_fields = _parse_msg_field_names(_PERCEPTION_EVENT_MSG)
    assert len(msg_fields) == 27, (
        f'PerceptionEvent.msg должен содержать 27 полей, сейчас {len(msg_fields)}. '
        f'Если IDL расширили — обнови PROJECTED_FIELDS/EXCLUDED_FIELDS и это число.'
    )
    assert len(PROJECTED_FIELDS) == 15, f'PROJECTED_FIELDS: {len(PROJECTED_FIELDS)}'
    assert len(EXCLUDED_FIELDS) == 12, f'EXCLUDED_FIELDS: {len(EXCLUDED_FIELDS)}'


# ---------- 2/3. регрессии на два живых бага -------------------------------


def test_battery_voltage_reaches_context():
    """REGRESSION A: раньше читалось msg.battery_percentage -> всегда 0.0."""
    ctx = project_perception_event(_FakePerceptionEvent(battery_voltage=38.4))
    assert ctx['battery_voltage'] == pytest.approx(38.4)
    assert ctx['battery_percent'] is not None
    assert ctx['battery_percent'] == pytest.approx(70.0)


def test_timestamp_comes_from_stamp_not_zero():
    """REGRESSION B: раньше читалось msg.timestamp -> всегда 0.0."""
    ctx = project_perception_event(
        _FakePerceptionEvent(stamp=_Stamp(1_700_000_000, 500_000_000))
    )
    assert ctx['timestamp'] == pytest.approx(1_700_000_000.5)
    assert ctx['timestamp'] != 0.0


def test_time_period_reaches_context():
    """Время суток — прямое требование сценария «вопрос по времени суток»."""
    ctx = project_perception_event(_FakePerceptionEvent(time_period='evening'))
    assert ctx['time_period'] == 'evening'
    assert ctx['current_time_human'] == '2026-09-15 21:40:00'


# ---------- 4. дрейф IDL ловится громко ------------------------------------


def test_renamed_idl_field_raises_instead_of_silent_zero():
    """Без hasattr: расхождение имён — громкая ошибка, а не тихий ноль."""
    msg = _FakePerceptionEvent()
    del msg.battery_voltage
    with pytest.raises(AttributeError):
        project_perception_event(msg)


# ---------- 5. приватность -------------------------------------------------


def test_vision_events_do_not_leak_to_personality():
    """ADR-0089 2.2: у стаб-детекций нет маркера — в контекст они не идут."""
    msg = _FakePerceptionEvent(
        vision_event_count=27,
        vision_events_json='[{"event_type": "person", "confidence": 0.92}]',
    )
    ctx = project_perception_event(msg)
    assert 'vision_events_json' not in ctx
    assert 'vision_event_count' not in ctx
    # и ни одно значение контекста не тащит выдуманную детекцию текстом
    assert all('person' not in str(v) for v in ctx.values())


def test_dead_summary_fields_are_excluded():
    for name in (
        'speech_summaries',
        'robot_response_summaries',
        'robot_thought_summaries',
        'vision_summaries',
        'system_summaries',
    ):
        assert name in EXCLUDED_FIELDS


# ---------- вольты -> проценты ---------------------------------------------


def test_zero_volts_means_no_data_not_empty_battery():
    """Агрегатор пишет 0.0 В когда данных не было — это не «разряжена»."""
    assert voltage_to_percent(0.0) is None
    assert voltage_to_percent(-1.0) is None


def test_voltage_to_percent_endpoints_and_clamp():
    assert voltage_to_percent(BATTERY_VOLTAGE_FULL) == pytest.approx(100.0)
    assert voltage_to_percent(BATTERY_VOLTAGE_EMPTY) == pytest.approx(0.0)
    assert voltage_to_percent(36.0) == pytest.approx(50.0)
    assert voltage_to_percent(45.0) == pytest.approx(100.0)
    assert voltage_to_percent(29.0) == pytest.approx(0.0)
