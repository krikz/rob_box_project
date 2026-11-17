"""IDL-паритет Observation.msg ↔ observation_geometry.OBSERVATION_FIELDS (ADR-0138).

По образцу test_vision_event_parity.py: ``.msg`` парсится как текст (без
colcon), поле-в-поле сверяется с Python-стороной, которой нода заполняет
сообщение (``vision_hailo_node._publish_observation``). Плюс инварианты
контракта ADR-0130 §5.1, которые видны прямо в IDL:

  * п.1 — в Observation нет имени;
  * header (stamp наблюдения + frame_id датчика) — первое поле;
  * честный статус глубины (position_valid + position_status) есть.

Если сгенерированный msg-класс доступен (colcon build) — сверка и с ним,
иначе skip с причиной.

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_observation_parity.py -q --no-cov
"""

from __future__ import annotations

import importlib
import re
import sys
from pathlib import Path
from typing import List, Tuple

import pytest

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

_MSGS_ROOT = _HERE.parents[3] / 'rob_box_perception_msgs'
_OBSERVATION_MSG = _MSGS_ROOT / 'msg' / 'Observation.msg'
_CMAKELISTS = _MSGS_ROOT / 'CMakeLists.txt'

geo = importlib.import_module('rob_box_perception.observation_geometry')

_MSG_FIELD_RE = re.compile(
    r'^\s*(?P<type>[A-Za-z_][A-Za-z0-9_/\[\]]*)\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*$'
)


def _parse_fields(path: Path) -> List[Tuple[str, str]]:
    fields = []
    for raw in path.read_text(encoding='utf-8').splitlines():
        line = raw.split('#', 1)[0].rstrip()
        if not line.strip():
            continue
        m = _MSG_FIELD_RE.match(line)
        assert m, f'Не распарсилась строка IDL: {raw!r}'
        fields.append((m.group('type'), m.group('name')))
    return fields


def test_observation_fields_match_idl():
    idl = [name for _t, name in _parse_fields(_OBSERVATION_MSG)]
    data = [name for name in idl if name != 'header']
    assert data == list(geo.OBSERVATION_FIELDS), (
        f'Observation.msg и OBSERVATION_FIELDS разошлись.\n'
        f'  IDL (без header): {data}\n'
        f'  Python:           {list(geo.OBSERVATION_FIELDS)}'
    )


def test_header_is_first_and_std_msgs():
    fields = _parse_fields(_OBSERVATION_MSG)
    assert fields[0] == ('std_msgs/Header', 'header')


def test_idl_types_of_contract_fields():
    types = {name: t for t, name in _parse_fields(_OBSERVATION_MSG)}
    assert types['position'] == 'geometry_msgs/Point'
    assert types['position_valid'] == 'bool'
    assert types['position_status'] == 'string'
    assert types['distance_m'] == 'float32'
    assert types['position_stddev_m'] == 'float32'
    assert types['candidate_id'] == 'string'
    assert types['image_width'] == 'uint32'


def test_idl_carries_no_name():
    """ADR-0130 §5.1 п.1: Наблюдение не содержит имени."""
    names = {name for _t, name in _parse_fields(_OBSERVATION_MSG)}
    forbidden = {'display_name', 'name', 'person_name', 'acquaintance_id',
                 'track_id', 'name_state'}
    assert not names & forbidden, sorted(names & forbidden)


def test_observation_msg_is_generated():
    text = _CMAKELISTS.read_text(encoding='utf-8')
    assert '"msg/Observation.msg"' in text


def test_status_vocabulary_documented_in_idl():
    """Каждый STATUS_* из кода перечислен в комментарии IDL (один словарь)."""
    text = _OBSERVATION_MSG.read_text(encoding='utf-8')
    statuses = [
        getattr(geo, n) for n in dir(geo) if n.startswith('STATUS_')
    ]
    assert statuses
    missing = [s for s in statuses if s not in text]
    assert not missing, f'Статусы не описаны в Observation.msg: {missing}'


def test_fields_match_generated_msg_class():
    try:
        from rob_box_perception_msgs.msg import Observation  # type: ignore
        generated = set(Observation.get_fields_and_field_types())
    except Exception as exc:  # noqa: BLE001
        pytest.skip(
            f'rob_box_perception_msgs.msg.Observation не импортируется ({exc!r}); '
            'нужен colcon build. IDL-парсер выше уже покрыл контракт.'
        )
    assert generated - {'header'} == set(geo.OBSERVATION_FIELDS)
