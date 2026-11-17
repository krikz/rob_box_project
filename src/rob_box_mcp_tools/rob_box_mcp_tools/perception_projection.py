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

import json
from typing import Any, Dict, List, Tuple

from rob_box_core.battery import voltage_to_percent  # noqa: F401 (реэкспорт для потребителей модуля)


# ---------------------------------------------------------------------------
# Батарея: вольты → проценты
# ---------------------------------------------------------------------------
# ``PerceptionEvent.battery_voltage`` — напряжение бортовой шины в вольтах
# (context_aggregator_node берёт его из /dynamic_joint_states: joint ``battery``,
# интерфейс ``voltage``). Кривая «вольты -> проценты» и пороги (10S Li-ion,
# 32..42 В) живут в одном месте — ``rob_box_core.battery``; здесь только импорт.
#
# Важно: агрегатор пишет 0.0, когда данных о батарее ещё не было
# (``current_sensors.get('battery', 0.0)``). Ноль вольт на 10S — не «разряжена»,
# а «нет данных», и трактуется именно так (``voltage_to_percent`` -> ``None``).


# ---------------------------------------------------------------------------
# Контракт проекции
# ---------------------------------------------------------------------------

#: Поля ``PerceptionEvent.msg``, которые видит Личность.
PROJECTED_FIELDS: Tuple[str, ...] = (
    'stamp',
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
    # --- зрительные события, issue #2532 (снято после PR #2583) ---------
    # См. комментарий у EXCLUDED_FIELDS ниже про то, ПОЧЕМУ это теперь
    # безопасно и ГДЕ живёт сама фильтрация (не здесь).
    'vision_event_count',
    'vision_events_json',
)

#: Поля, которые Личность НЕ видит, и почему. Причина обязательна: если поле
#: исключено «просто так», это не решение, а забывчивость.
EXCLUDED_FIELDS: Dict[str, str] = {
    'vision_context': (
        'мёртвое поле (issue #4352): у /perception/vision_context нет издателя, агрегатор его '
        'больше не слушает; ключ vision_context в контексте строит код из vision_events_json '
        'и apriltag_ids (describe_view)'
    ),
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
}

# ---------------------------------------------------------------------------
# Приватность зрительных событий, ADR-0089 §2.2 (issue #2532)
# ---------------------------------------------------------------------------
# vision_event_count / vision_events_json раньше сидели в EXCLUDED_FIELDS:
# заглушка vision_hailo (StubHEFLoader) публиковала ВЫДУМАННОЕ событие
# "person, conf 0.92, 1 м", и отличить его от реальной детекции было нечем.
# PR #2583 завёл честный маркер (STUB_EVENT_TYPE='stub', STUB_SOURCE_CAMERA
# ='stub', предикат vision_hailo_loader.is_stub_event) — причина исключения
# отпала, и это тот самый повод, который отменяет исключение выше.
#
# Архитектурное решение (issue #2532): маркер отсекается НЕ здесь.
#
#   rob_box_mcp_tools (этот пакет) НЕ зависит от rob_box_perception —
#   проверено по package.xml/setup.py обоих пакетов: mcp_tools тянет только
#   rob_box_perception_msgs (сообщения), а is_stub_event живёт в коде
#   rob_box_perception.vision_hailo_loader. Импортировать его отсюда значило
#   бы тянуть необъявленную межпакетную зависимость — то, ради чего в этом
#   модуле и так уже нет ни одного ROS/rclpy импорта.
#
#   context_aggregator_node.py (rob_box_perception) — который и строит
#   PerceptionEvent — уже импортирует vision_hailo_loader (VISION_EVENT_
#   FIELDS), то есть зависимость там объявлена и легальна. Фильтрация
#   is_stub_event встроена в publish_event() ИМЕННО в момент сборки
#   PerceptionEvent: см. context_aggregator_node.py. Раз это единственный
#   продюсер PerceptionEvent, граница "выдумка не пересекает" проведена на
#   входе в сообщение, а не на выходе из него — сама структура на проводе
#   уже не может солгать, и proекции здесь не нужно повторно спрашивать
#   is_stub_event (второй копией того же условия, дублирующей источник
#   истины).
#
#   Отладочная ценность выдумки (event_type='stub' виден на топике) при
#   этом не теряется: сырой /vision/hailo/events как публиковал stub, так и
#   публикует — это отдельный топик, PerceptionEvent.vision_events_json
#   лишь агрегирует его для Личности, и именно на этой агрегации граница
#   и должна стоять.
#
# project_perception_event() ниже поэтому просто разбирает
# vision_events_json в структуру, удобную для промпта — без повторной
# приватность-логики.


#: Детекция моложе этого (секунды) - «в кадре сейчас». Лицо идёт ~2-5 Гц, пауза в 3 с - уже ушёл.
IN_VIEW_S = 3.0

#: Фраза, когда о кадре нечего сказать. Пустая строка была враньём молчанием (issue #4352).
NO_CAMERA_DATA = 'камера: нет данных о лицах и людях'


def _project_vision_events(events_json: str, now: float) -> List[Dict[str, Any]]:
    """``vision_events_json`` -> по одной записи на объект в кадре, свежие первыми.

    Сырой JSON-строкой в контекст — плохо (шум, читать боту нечем). Агрегатор держит всё окно
    памяти: на роботе 04.11.2026 это 134 события на одно лицо и один силуэт (issue #4352). Здесь
    они схлопываются по объекту: лицо - по ``embedding_id`` (или имени), силуэт человека и прочие
    классы - по классу. Остаётся самая свежая детекция объекта.

    К моменту, когда PerceptionEvent долетает сюда, stub-события уже
    отфильтрованы на стороне продюсера (context_aggregator_node, ADR-0089
    §2.2) — это инвариант границы сообщения, а не этого модуля. Здесь —
    только разбор формата, устойчивый к пустой строке и мусору в JSON.

    ``age_s`` — сколько секунд прошло от детекции до ``now`` (stamp самого
    PerceptionEvent). Без него Личность не отличала старую детекцию от
    свежей и «видела» ушедшего человека всё окно памяти агрегатора.
    """
    newest: Dict[Tuple[str, str], Dict[str, Any]] = {}
    where: Dict[Tuple[str, str], _Spot] = {}
    for item in _parse_events(events_json):
        entry = _vision_entry(item, now)
        key = (entry['kind'], str(item.get('embedding_id') or entry['name'] or ''))
        kept = newest.get(key)
        if kept is None or _fresher(entry, kept):
            newest[key] = entry
            where[key] = _spot(item, entry)
    return sorted(_one_entry_per_face(newest, where), key=_age_order)


#: Безымянное лицо ближе этого (м) к именованному в другом кадре окна - то же лицо (issue #4388), когда bbox нет.
SAME_FACE_DISTANCE_M = 0.25

#: ``(stamp, cx, cy, сторона bbox, distance_m)`` самой свежей детекции объекта; ``None`` - величины нет.
_Spot = Tuple[Any, Any, Any, Any, Any]


def _spot(item: Dict[str, Any], entry: Dict[str, Any]) -> _Spot:
    stamp = item.get('stamp')
    at = None
    if isinstance(stamp, dict):
        try:
            at = float(stamp['sec']) + float(stamp.get('nanosec', 0)) * 1e-9
        except (KeyError, TypeError, ValueError):
            at = None
    try:
        cx, cy = float(item['bbox_cx']), float(item['bbox_cy'])
        side = max(float(item['bbox_w']), float(item['bbox_h']))
    except (KeyError, TypeError, ValueError):
        cx = cy = side = None
    if side is not None and side <= 0:
        cx = cy = side = None
    return at, cx, cy, side, entry['distance_m']


def _same_place(a: _Spot, b: _Spot) -> bool:
    """Две детекции лица из РАЗНЫХ кадров на одном месте: центры bbox ближе стороны бокса, а без bbox - дистанции
    ближе :data:`SAME_FACE_DISTANCE_M`. Один кадр (тот же stamp) - два разных лица, не схлопываются."""
    if a[0] is None or b[0] is None or a[0] == b[0]:
        return False
    if a[1] is not None and b[1] is not None:
        return ((a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5 <= max(a[3], b[3])
    if a[4] is not None and b[4] is not None:
        return abs(a[4] - b[4]) <= SAME_FACE_DISTANCE_M
    return False


def _one_entry_per_face(
    newest: Dict[Tuple[str, str], Dict[str, Any]], where: Dict[Tuple[str, str], _Spot],
) -> List[Dict[str, Any]]:
    """Одно физическое лицо - одна запись (issue #4388, живой 04.11: «незнакомое лицо ~1.1 м» age 0.5 с и «Литвин»
    ~1.1 м age 0.9 с - одно фото; почему кадр остался без имени, по логу не установлено). Безымянное лицо из другого
    кадра на месте именованного вливается в именованное: имя остаётся, возраст и дистанция - самой свежей детекции."""
    named = [k for k, e in newest.items() if e['kind'] == 'face' and e['name']]
    out = dict(newest)
    for key, entry in newest.items():
        if entry['kind'] != 'face' or entry['name']:
            continue
        twin = next((k for k in named if _same_place(where[key], where[k])), None)
        if twin is None:
            continue
        del out[key]
        if _fresher(entry, out[twin]):
            out[twin] = dict(out[twin], age_s=entry['age_s'], distance_m=entry['distance_m'])
    return list(out.values())


def _parse_events(events_json: str) -> List[Dict[str, Any]]:
    if not events_json:
        return []
    try:
        raw = json.loads(events_json)
    except (TypeError, ValueError):
        return []
    if not isinstance(raw, list):
        return []
    return [item for item in raw if isinstance(item, dict)]


def _vision_entry(item: Dict[str, Any], now: float) -> Dict[str, Any]:
    name = item.get('display_name') or None
    kind = item.get('event_type') or item.get('class_name') or 'object'
    distance = item.get('distance_m')
    # -1 (и вообще < 0) — это "глубина неизвестна" (см. VisionEvent.msg),
    # а не "0 метров". Отдаём None, чтобы промпт не соврал про дистанцию.
    distance_m = (
        float(distance)
        if isinstance(distance, (int, float)) and distance >= 0
        else None
    )
    return {
        'label': str(name or item.get('class_name') or kind),
        'kind': str(kind),
        'name': str(name) if name else None,
        'confidence': float(item.get('confidence', 0.0)),
        'distance_m': distance_m,
        'age_s': _event_age(item.get('stamp'), now),
    }


def _fresher(entry: Dict[str, Any], kept: Dict[str, Any]) -> bool:
    """Свежее ли ``entry`` уже взятого. Без stamp - берётся позднее в списке."""
    if entry['age_s'] is None or kept['age_s'] is None:
        return True
    return entry['age_s'] <= kept['age_s']


def _age_order(entry: Dict[str, Any]) -> Tuple[bool, float]:
    age = entry['age_s']
    return (age is None, age if age is not None else 0.0)


def in_view(entry: Dict[str, Any]) -> bool:
    """Объект в кадре сейчас: детекция моложе ``IN_VIEW_S``."""
    return entry['age_s'] is not None and entry['age_s'] <= IN_VIEW_S


#: Классы детектора (COCO, порядок как в ``vision_hailo_loader.COCO_CLASS_NAMES``) по-русски.
#: Класса нет в таблице - остаётся английское слово как есть, без выдумок (issue #4352).
COCO_RU: Dict[str, str] = {
    'bicycle': 'велосипед', 'car': 'машина', 'motorcycle': 'мотоцикл', 'airplane': 'самолёт',
    'bus': 'автобус', 'train': 'поезд', 'truck': 'грузовик', 'boat': 'лодка',
    'traffic light': 'светофор', 'fire hydrant': 'гидрант', 'stop sign': 'знак стоп',
    'parking meter': 'паркомат', 'bench': 'скамейка', 'bird': 'птица', 'cat': 'кошка',
    'dog': 'собака', 'horse': 'лошадь', 'sheep': 'овца', 'cow': 'корова', 'elephant': 'слон',
    'bear': 'медведь', 'zebra': 'зебра', 'giraffe': 'жираф', 'backpack': 'рюкзак',
    'umbrella': 'зонт', 'handbag': 'сумка', 'tie': 'галстук', 'suitcase': 'чемодан',
    'frisbee': 'фрисби', 'skis': 'лыжи', 'snowboard': 'сноуборд', 'sports ball': 'мяч',
    'kite': 'воздушный змей', 'baseball bat': 'бейсбольная бита',
    'baseball glove': 'бейсбольная перчатка', 'skateboard': 'скейтборд',
    'surfboard': 'сёрфборд', 'tennis racket': 'теннисная ракетка', 'bottle': 'бутылка',
    'wine glass': 'бокал', 'cup': 'чашка', 'fork': 'вилка', 'knife': 'нож', 'spoon': 'ложка',
    'bowl': 'миска', 'banana': 'банан', 'apple': 'яблоко', 'sandwich': 'сэндвич',
    'orange': 'апельсин', 'broccoli': 'брокколи', 'carrot': 'морковь', 'hot dog': 'хот-дог',
    'pizza': 'пицца', 'donut': 'пончик', 'cake': 'торт', 'chair': 'стул', 'couch': 'диван',
    'potted plant': 'комнатное растение', 'bed': 'кровать', 'dining table': 'обеденный стол',
    'toilet': 'унитаз', 'tv': 'телевизор', 'laptop': 'ноутбук', 'mouse': 'компьютерная мышь',
    'remote': 'пульт', 'keyboard': 'клавиатура', 'cell phone': 'телефон',
    'microwave': 'микроволновка', 'oven': 'духовка', 'toaster': 'тостер', 'sink': 'раковина',
    'refrigerator': 'холодильник', 'book': 'книга', 'clock': 'часы', 'vase': 'ваза',
    'scissors': 'ножницы', 'teddy bear': 'плюшевый мишка', 'hair drier': 'фен',
    'toothbrush': 'зубная щётка',
}


def ru_class(class_name: str) -> str:
    """Русское имя класса детектора; незнакомый класс - как пришёл."""
    return COCO_RU.get(class_name, class_name)


def _say_entry(entry: Dict[str, Any]) -> str:
    if entry['kind'] == 'face':
        what = f'лицо «{entry["name"]}»' if entry['name'] else 'незнакомое лицо'
    elif entry['kind'] == 'person':
        what = 'силуэт человека'
    else:
        what = ru_class(entry['label'])
    if entry['distance_m'] is not None:
        what += f' ~{entry["distance_m"]:.1f} м'
    return what


def describe_view(vision_events: List[Dict[str, Any]], apriltag_ids: List[int]) -> str:
    """Что видит камера - фразой, которую строит код, а не модель (ADR-0148, issue #4352)."""
    parts = [_say_entry(e) for e in vision_events if in_view(e)]
    if apriltag_ids:
        parts.append('метки AprilTag: ' + ', '.join(str(i) for i in apriltag_ids))
    if parts:
        return 'в кадре: ' + '; '.join(parts)
    if vision_events:
        last = vision_events[0]
        ago = f' {last["age_s"]:.0f} с назад' if last['age_s'] is not None else ''
        return f'сейчас в кадре никого; последним было: {_say_entry(last)}{ago}'
    return NO_CAMERA_DATA


def _faces(vision_events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Лица в кадре сейчас: имя (None - не узнано), дистанция, возраст детекции."""
    return [
        {'name': e['name'], 'distance_m': e['distance_m'], 'age_s': e['age_s']}
        for e in vision_events if e['kind'] == 'face' and in_view(e)
    ]


def _event_age(stamp: Any, now: float) -> Any:
    """Возраст детекции в секундах (с точностью 0.1) или None без stamp."""
    if not isinstance(stamp, dict):
        return None
    try:
        at = float(stamp['sec']) + float(stamp.get('nanosec', 0)) * 1e-9
    except (KeyError, TypeError, ValueError):
        return None
    return round(max(now - at, 0.0), 1)


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
    # vision_event_count здесь НЕ читается из msg.vision_event_count
    # напрямую: он пересчитывается из фактической длины разобранного
    # vision_events_json. Это тот же приём, что уже применён к battery/
    # timestamp выше по файлу — не доверять отдельному полю-счётчику,
    # который в принципе может разойтись с данными, а считать от источника
    # истины. Так vision_event_count и vision_events всегда согласованы.
    timestamp = _stamp_to_unix(msg.stamp)
    vision_events = _project_vision_events(msg.vision_events_json, timestamp)
    apriltag_ids = list(msg.apriltag_ids)
    return {
        'timestamp': timestamp,
        'vision_context': describe_view(vision_events, apriltag_ids),
        'faces': _faces(vision_events),
        'is_moving': bool(msg.is_moving),
        'battery_voltage': battery_voltage,
        'battery_percent': voltage_to_percent(battery_voltage),
        'temperature': float(msg.temperature),
        'apriltag_ids': apriltag_ids,
        'system_health_status': msg.system_health_status,
        'health_issues': list(msg.health_issues),
        'current_time_human': msg.current_time_human,
        'time_period': msg.time_period,
        'internet_available': bool(msg.internet_available),
        'failed_nodes': list(msg.failed_nodes),
        'missing_nodes': list(msg.missing_nodes),
        'mapping_mode': msg.mapping_mode,
        'memory_summary': msg.memory_summary,
        'vision_event_count': len(vision_events),
        'vision_events': vision_events,
    }
