"""Parity-test VisionEvent: VISION_EVENT_FIELDS ↔ VisionEvent.msg (ADR-0089 §3).

Зачем: контракт из 13 полей `VisionEvent` раньше жил в 4 местах
  1. `VisionEvent.msg` — IDL
  2. `VISION_EVENT_FIELDS` + `normalize_event_dict` в `vision_hailo_loader.py`
  3. `_publish_event` в `vision_hailo_node.py` — dict→msg
  4. `on_hailo_vision_event` в `context_aggregator_node.py` — msg→dict

Добавить поле = править 3-4 места, и ничто не гарантирует согласованность.
После рефактора #3 и #4 ходят по `VISION_EVENT_FIELDS` циклом, но
контракт всё ещё может разъехаться с `.msg`. Этот тест — единственный
источник правды: парсит `.msg` напрямую и сверяет с константой.

Контракт:
    set(VISION_EVENT_FIELDS) == set(полей VisionEvent.msg) − {'stamp'}

Запуск:
    pytest src/rob_box_perception/test/unit/test_vision_event_parity.py -v
"""

from __future__ import annotations

import importlib
import re
import sys
from pathlib import Path

import pytest


# ---------- import target under test ------------------------------------

def _import_module():
    """Импортировать vision_hailo_loader в обход pytest rootdir.

    rootdir = <worktree>/ (где лежит pytest.ini), но пакет лежит в src/.
    Поэтому PYTHONPATH должен указывать на src/rob_box_perception,
    а не на worktree root.
    """
    here = Path(__file__).resolve()
    pkg_root = str(here.parents[2])  # <worktree>/src/rob_box_perception
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)
    return importlib.import_module('rob_box_perception.vision_hailo_loader')


loader_mod = _import_module()


def _find_msg_file() -> Path:
    """Найти VisionEvent.msg от rob_box_perception_msgs.

    Идём вверх от текущего файла:
      here = <repo>/src/rob_box_perception/test/unit/test_vision_event_parity.py
      parents[0] = .../test/unit
      parents[1] = .../test
      parents[2] = .../rob_box_perception
      parents[3] = .../src        <-- здесь лежит rob_box_perception_msgs/msg/
    Это самая стабильная стратегия — не зависит от worktree-имени.
    """
    here = Path(__file__).resolve()
    src_dir = here.parents[3]
    msg_path = src_dir / 'rob_box_perception_msgs' / 'msg' / 'VisionEvent.msg'
    if not msg_path.is_file():
        pytest.skip(
            f'VisionEvent.msg не найден ({msg_path}) — parity-test пропущен'
        )
    return msg_path


_MSG_FIELD_RE = re.compile(r'^\s*(?:int\d+|float\d+|bool|uint\d+|string|\w+)\s+(\w+)\s*$')


def _parse_msg_fields(msg_path: Path):
    """Вернуть список имён полей из .msg-файла.

    ROS .msg формат:
        <type> <field_name>  # optional comment

    builtin_interfaces/Time stamp — это сложный тип (составное поле),
    но его имя в слот-контракте просто `stamp`. Парсим только имя поля,
    тип нас не интересует (нас интересует только совпадение имён).
    Пустые строки и строки-комментарии игнорируются.
    """
    fields = []
    for raw in msg_path.read_text(encoding='utf-8').splitlines():
        line = raw.split('#', 1)[0].rstrip()
        if not line.strip():
            continue
        match = _MSG_FIELD_RE.match(line)
        if match:
            fields.append(match.group(1))
    return fields


# ---------- tests ---------------------------------------------------------


def test_vision_event_fields_count_is_13():
    """VISION_EVENT_FIELDS содержит ровно 13 нестамповых полей.

    Защита от случайного удаления или дублирования при редактировании.
    """
    assert len(loader_mod.VISION_EVENT_FIELDS) == 13, (
        f'VISION_EVENT_FIELDS должен содержать 13 полей (без stamp), '
        f'но их {len(loader_mod.VISION_EVENT_FIELDS)}: '
        f'{list(loader_mod.VISION_EVENT_FIELDS)}'
    )


def test_vision_event_fields_match_msg_fields():
    """Главный parity-test: имена в .msg == имена в константе − {'stamp'}.

    Это и есть контракт. Если кто-то добавит поле в .msg, но забыл
    обновить VISION_EVENT_FIELDS (или наоборот) — тест ловит это
    немедленно, без запуска полного rclpy/ros2 пайплайна.
    """
    msg_path = _find_msg_file()
    msg_fields = set(_parse_msg_fields(msg_path))
    code_fields = set(loader_mod.VISION_EVENT_FIELDS)

    # stamp в .msg присутствует, в VISION_EVENT_FIELDS — нет
    # (его проставляет ROS-нода отдельно). Это и есть единственная разница.
    msg_no_stamp = msg_fields - {'stamp'}

    missing_in_code = msg_no_stamp - code_fields
    extra_in_code = code_fields - msg_no_stamp

    assert not missing_in_code, (
        f'Поля в VisionEvent.msg, которых нет в VISION_EVENT_FIELDS: '
        f'{sorted(missing_in_code)}. Добавь в vision_hailo_loader.VISION_EVENT_FIELDS.'
    )
    assert not extra_in_code, (
        f'Поля в VISION_EVENT_FIELDS, которых нет в VisionEvent.msg: '
        f'{sorted(extra_in_code)}. Убери из vision_hailo_loader.VISION_EVENT_FIELDS.'
    )
    assert msg_no_stamp == code_fields


def test_vision_event_fields_no_stamp():
    """VISION_EVENT_FIELDS НЕ должен содержать stamp.

    stamp проставляет ROS-нода отдельно (msg.stamp = clock.now().to_msg()).
    Включение его в список сломает и _publish_event, и on_hailo_vision_event
    — нормализатор выдаст KeyError.
    """
    assert 'stamp' not in loader_mod.VISION_EVENT_FIELDS


def test_normalize_event_dict_covers_all_vision_event_fields():
    """normalize_event_dict возвращает dict, где есть каждое поле из VISION_EVENT_FIELDS.

    Если в константу добавят поле, но забудут добавить ключ в
    normalize_event_dict — _publish_event упадёт на `norm[field]` с KeyError.
    """
    norm = loader_mod.normalize_event_dict({})
    missing = set(loader_mod.VISION_EVENT_FIELDS) - set(norm.keys())
    assert not missing, (
        f'normalize_event_dict не возвращает ключи: {sorted(missing)}. '
        f'Добавь дефолт в vision_hailo_loader.normalize_event_dict.'
    )
