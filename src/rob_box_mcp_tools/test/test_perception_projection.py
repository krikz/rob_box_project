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
 5. VISION: зрительные события доходят до Личности в структурированном
    виде (label/confidence/distance_m, не сырым JSON), а счётчик считается
    от факта — от разобранного списка, а не от отдельного поля-счётчика.
    Приватность (ADR-0089 §2.2) к моменту проекции уже обеспечена
    продюсером (context_aggregator_node, PR #2583/#2532) — здесь
    stub-фильтрации нет по архитектурному решению, и это не баг, а граница.

Запуск:
    cd src/rob_box_mcp_tools && python -m pytest test/test_perception_projection.py -v
"""

from __future__ import annotations

import json
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
    assert len(PROJECTED_FIELDS) == 16, f'PROJECTED_FIELDS: {len(PROJECTED_FIELDS)}'
    assert len(EXCLUDED_FIELDS) == 11, f'EXCLUDED_FIELDS: {len(EXCLUDED_FIELDS)}'


# ---------- 2/3. регрессии на два живых бага -------------------------------


def test_battery_voltage_reaches_context():
    """REGRESSION A: раньше читалось msg.battery_percentage -> всегда 0.0."""
    ctx = project_perception_event(_FakePerceptionEvent(battery_voltage=38.4))
    assert ctx['battery_voltage'] == pytest.approx(38.4)
    assert ctx['battery_percent'] is not None
    # кривая Li-ion 10S (rob_box_core.battery): 3.84 В на банку между 3.79 (50 %) и 3.85 (60 %)
    assert ctx['battery_percent'] == pytest.approx(58.3333, abs=1e-3)


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


# ---------- 5. зрительные события -------------------------------------


def test_real_vision_event_reaches_context():
    """Реальная детекция доходит до Личности в структурированном виде."""
    msg = _FakePerceptionEvent(
        vision_event_count=1,
        vision_events_json=(
            '[{"event_type": "person", "class_name": "person", '
            '"confidence": 0.87, "distance_m": 1.4, '
            '"source_camera": "oak-d"}]'
        ),
    )
    ctx = project_perception_event(msg)
    assert ctx['vision_event_count'] == 1
    assert ctx['vision_events'] == [
        {
            'label': 'person',
            'kind': 'person',
            'name': None,
            'confidence': pytest.approx(0.87),
            'distance_m': pytest.approx(1.4),
            'age_s': None,
        }
    ]


def test_vision_event_age_is_relative_to_context_stamp():
    """Личность видит, насколько детекция старая.

    Живой случай 16.09.2026: человек ушёл, а робот продолжал «видеть» его
    лицо — в контексте не было ничего, что отличало бы старую детекцию от
    свежей. Возраст считается от stamp самого PerceptionEvent, а не от
    часов mcp_server — так он не зависит от рассинхрона хостов.
    """
    msg = _FakePerceptionEvent(
        vision_events_json=(
            '[{"event_type": "face", "confidence": 0.99, '
            '"stamp": {"sec": 1699999958, "nanosec": 0}}]'
        ),
    )
    ctx = project_perception_event(msg)
    assert ctx['vision_events'][0]['age_s'] == pytest.approx(42.5)


def test_stub_event_that_reaches_projection_is_not_filtered_here():
    """Архитектурное решение (#2532): фильтрация — в context_aggregator_node,
    НЕ в проекции. Если stub-событие всё же долетело сюда (например, продюсер
    сломан) — проекция его честно покажет, а не молча спрячет. Приватность
    гарантируется на входе в PerceptionEvent, а не на выходе из неё; см.
    комментарий у EXCLUDED_FIELDS в perception_projection.py.
    """
    msg = _FakePerceptionEvent(
        vision_event_count=1,
        vision_events_json=(
            '[{"event_type": "stub", "source_camera": "stub", '
            '"class_name": "person", "confidence": 0.92, "distance_m": 1.0}]'
        ),
    )
    ctx = project_perception_event(msg)
    # Намеренно НЕ фильтруется здесь — граница в другом модуле.
    assert ctx['vision_event_count'] == 1
    assert ctx['vision_events'][0]['label'] == 'person'


def test_mixed_real_and_junk_events_count_only_valid_dicts():
    """Смесь валидных и мусорных записей в JSON: считаются только dict'ы."""
    msg = _FakePerceptionEvent(
        vision_events_json=(
            '[{"class_name": "person", "confidence": 0.8, "distance_m": 2.0}, '
            '"not-a-dict", 42, '
            '{"class_name": "cup", "confidence": 0.6, "distance_m": -1.0}]'
        ),
    )
    ctx = project_perception_event(msg)
    assert ctx['vision_event_count'] == 2
    labels = [e['label'] for e in ctx['vision_events']]
    assert labels == ['person', 'cup']
    # distance_m == -1 в исходных данных значит "неизвестно" -> None
    assert ctx['vision_events'][1]['distance_m'] is None


def test_empty_vision_events_json_does_not_break_context():
    """Пустая строка/пустой массив -> пустой список, без исключений."""
    for empty_payload in ('', '[]'):
        msg = _FakePerceptionEvent(
            vision_event_count=0, vision_events_json=empty_payload,
        )
        ctx = project_perception_event(msg)
        assert ctx['vision_event_count'] == 0
        assert ctx['vision_events'] == []


def test_malformed_vision_events_json_does_not_raise():
    """Битый JSON -> пустой список, а не исключение (capability-honest)."""
    msg = _FakePerceptionEvent(vision_events_json='{not valid json')
    ctx = project_perception_event(msg)
    assert ctx['vision_event_count'] == 0
    assert ctx['vision_events'] == []


def test_vision_event_count_ignores_stale_counter_field():
    """vision_event_count в контексте считается от факта (длины разобранного
    списка), а не доверяется отдельному полю msg.vision_event_count — тот
    же приём защиты от рассинхронизации, что и у battery/timestamp выше.
    """
    msg = _FakePerceptionEvent(
        vision_event_count=99,  # намеренно врёт
        vision_events_json='[{"class_name": "person", "confidence": 0.9}]',
    )
    ctx = project_perception_event(msg)
    assert ctx['vision_event_count'] == 1


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


def test_voltage_to_percent_is_the_core_curve():
    """Кривая живёт в rob_box_core.battery (10S Li-ion); проекция только импортирует."""
    from rob_box_core import battery

    assert voltage_to_percent is battery.voltage_to_percent
    assert voltage_to_percent(42.0) == pytest.approx(100.0)
    assert voltage_to_percent(41.0) == pytest.approx(90.0)
    assert voltage_to_percent(36.0) == pytest.approx(15.0)
    assert voltage_to_percent(32.0) == pytest.approx(0.0)
    assert voltage_to_percent(45.0) == pytest.approx(100.0)
    assert voltage_to_percent(29.0) == pytest.approx(0.0)


# ---------- 6. что в кадре: issue #4352 --------------------------------------
# Живой PerceptionEvent 04.11.2026 (ros2 topic echo --once /perception/context_update на Main Pi):
# vision_context='' (у /perception/vision_context нет издателя), vision_event_count=134 - одно лицо
# «Дэнчик» и один силуэт, повторённые за окно памяти. Тул отдавал модели пустую строку.

def _live_event(event_type, sec, display_name='', embedding_id='', distance=1.02):
    return {
        'stamp': {'sec': sec, 'nanosec': 0}, 'source_camera': 'camera_color_optical_frame',
        'event_type': event_type, 'class_name': event_type, 'class_id': 0, 'confidence': 0.99,
        'distance_m': distance, 'embedding_id': embedding_id, 'display_name': display_name,
        'attributes_json': '',
    }


def _live_payload(now_sec=1_700_000_000):
    events = []
    for k in range(60):
        events.append(_live_event('face', now_sec - 59 + k, 'Литвин', '566331d3'))
        events.append(_live_event('person', now_sec - 59 + k, distance=1.04))
    return json.dumps(events, ensure_ascii=False)


def test_known_face_in_view_is_named_in_vision_context():
    ctx = project_perception_event(_FakePerceptionEvent(
        vision_events_json=_live_payload(), apriltag_ids=[21],
    ))
    assert ctx['vision_context'] == (
        'в кадре: лицо «Литвин» ~1.0 м; силуэт человека ~1.0 м; метки AprilTag: 21'
    )
    assert ctx['faces'] == [{'name': 'Литвин', 'distance_m': pytest.approx(1.02), 'age_s': 0.5}]


def test_repeated_detections_collapse_to_one_entry_per_object():
    ctx = project_perception_event(_FakePerceptionEvent(vision_events_json=_live_payload()))
    assert ctx['vision_event_count'] == 2
    assert [e['kind'] for e in ctx['vision_events']] == ['face', 'person']


def test_unknown_face_is_said_as_unknown():
    payload = json.dumps([_live_event('face', 1_700_000_000, embedding_id='abc')])
    ctx = project_perception_event(_FakePerceptionEvent(vision_events_json=payload, apriltag_ids=[]))
    assert ctx['vision_context'] == 'в кадре: незнакомое лицо ~1.0 м'
    assert ctx['faces'][0]['name'] is None


def test_no_detections_is_honest_no_data_not_empty_string():
    ctx = project_perception_event(_FakePerceptionEvent(vision_events_json='[]', apriltag_ids=[]))
    assert ctx['vision_context'] == 'камера: нет данных о лицах и людях'
    assert ctx['faces'] == []


def test_only_tag_in_view():
    ctx = project_perception_event(_FakePerceptionEvent(vision_events_json='[]', apriltag_ids=[21]))
    assert ctx['vision_context'] == 'в кадре: метки AprilTag: 21'


def test_person_who_left_is_not_in_view():
    payload = json.dumps([_live_event('face', 1_699_999_988, 'Литвин', '566331d3')], ensure_ascii=False)
    ctx = project_perception_event(_FakePerceptionEvent(vision_events_json=payload, apriltag_ids=[]))
    assert ctx['faces'] == []
    assert ctx['vision_context'] == 'сейчас в кадре никого; последним было: лицо «Литвин» ~1.0 м 12 с назад'


# ---------- 7. одно лицо - одна запись: issue #4388 ---------------------------
# Живой 04.11.2026 22:51 UTC: в кадре одно фото «Литвин», а get_perception_context отдал
# faces=[{name: None, 1.115 м, age 0.5}, {name: 'Литвин', 1.120 м, age 0.9}] и «незнакомое лицо ~1.1 м; ... лицо
# «Литвин» ~1.1 м» - робот сказал о двух лицах.

def _face_at(sec, nanosec, name='', embedding_id='', distance=1.12, cx=None):
    event = _live_event('face', sec, name, embedding_id, distance)
    event['stamp'] = {'sec': sec, 'nanosec': nanosec}
    if cx is not None:
        event.update(bbox_cx=cx, bbox_cy=0.5, bbox_w=0.2, bbox_h=0.25)
    return event


def test_unnamed_face_from_other_frame_at_same_place_merges_into_named():
    payload = json.dumps([
        _face_at(1_699_999_999, 600_000_000, 'Литвин', '566331d3', distance=1.1198),
        _face_at(1_700_000_000, 0, distance=1.1148),
    ], ensure_ascii=False)
    ctx = project_perception_event(_FakePerceptionEvent(vision_events_json=payload, apriltag_ids=[21]))
    assert ctx['faces'] == [{'name': 'Литвин', 'distance_m': pytest.approx(1.1148), 'age_s': 0.5}]
    assert ctx['vision_context'] == 'в кадре: лицо «Литвин» ~1.1 м; метки AprilTag: 21'


def test_unnamed_face_with_its_own_record_id_still_merges_by_place():
    """Кадр мог узнать фото как другую, безымянную запись галереи (104 безымянных лица) - место то же."""
    payload = json.dumps([
        _face_at(1_699_999_999, 600_000_000, 'Литвин', '566331d3', cx=0.50),
        _face_at(1_700_000_000, 0, embedding_id='46b612c4', cx=0.52),
    ], ensure_ascii=False)
    ctx = project_perception_event(_FakePerceptionEvent(vision_events_json=payload, apriltag_ids=[]))
    assert [f['name'] for f in ctx['faces']] == ['Литвин']


def test_two_faces_in_one_frame_stay_two():
    """Тот же stamp - один кадр, два лица: не схлопывать, даже на одной дистанции."""
    payload = json.dumps([
        _face_at(1_700_000_000, 0, 'Литвин', '566331d3'),
        _face_at(1_700_000_000, 0, distance=1.15),
    ], ensure_ascii=False)
    ctx = project_perception_event(_FakePerceptionEvent(vision_events_json=payload, apriltag_ids=[]))
    assert sorted(str(f['name']) for f in ctx['faces']) == ['None', 'Литвин']


def test_unnamed_face_elsewhere_in_frame_is_another_person():
    payload = json.dumps([
        _face_at(1_699_999_999, 600_000_000, 'Литвин', '566331d3', cx=0.2),
        _face_at(1_700_000_000, 0, cx=0.8),
    ], ensure_ascii=False)
    ctx = project_perception_event(_FakePerceptionEvent(vision_events_json=payload, apriltag_ids=[]))
    assert len(ctx['faces']) == 2
    assert ctx['vision_context'].startswith('в кадре: незнакомое лицо')
