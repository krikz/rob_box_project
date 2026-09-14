"""Unit-test: parity between IDL `VisionEvent.msg` and `VISION_EVENT_FIELDS`.

Зачем:
  VisionEvent.msg (IDL) и `VISION_EVENT_FIELDS` в `vision_hailo_loader`
  описывают один и тот же контракт с двух сторон:
    - `.msg` — это IDL, из которого `colcon build` генерит Python/С++ msg-классы.
    - `VISION_EVENT_FIELDS` — Python-side canonical tuple, который используется
      в `vision_hailo_node._publish_event` (setattr loop) и
      в `context_aggregator_node.on_hailo_vision_event` (getattr loop).
      Любые расхождения = silent drift: либо IDL расширили и забыли Python,
      либо Python расширили и забыли IDL.

Что проверяем:
  1. PRIMARY: парсим `VisionEvent.msg` напрямую (без colcon build) —
     поле-в-поле сравниваем с `VISION_EVENT_FIELDS` (за вычетом `stamp`,
     которое задаётся нодой отдельно, вне tuple). Это работает в любой
     среде (dev / CI / Docker), потому что `.msg` — это просто текстовый
     файл в репо.
  2. SECONDARY: ровно 13 полей в tuple и ровно 14 полей в `.msg` —
     ловит silent add/remove.
  3. OPTIONAL: если доступен сгенерированный msg-класс
     `rob_box_perception_msgs.msg.VisionEvent` (на машине с colcon build),
     сравниваем ещё и с ним через `get_fields_and_field_types()` — это
     проверяет, что IDL реально скомпилировался без рассинхрона.
     Если класс не импортируется — skip с явной причиной (не fail).
     Этот test не должен ломать прогон в чистом unit-env.

Запуск:
    pytest src/rob_box_perception/test/unit/test_vision_event_parity.py -v
    pytest src/rob_box_perception/test/unit/ -v
"""

from __future__ import annotations

import importlib
import re
import sys
from pathlib import Path
from typing import Set

import pytest


# ---------- paths ----------------------------------------------------------

# Этот файл лежит в: src/rob_box_perception/test/unit/test_vision_event_parity.py
# Пакет:             src/rob_box_perception/                                  ← parents[2]
#   └─ rob_box_perception/   ← вот этот каталог добавляем в sys.path, чтобы
#       импорт `rob_box_perception.vision_hailo_loader` зарезолвился.
# IDL:               src/rob_box_perception_msgs/                             ← parents[3]/rob_box_perception_msgs
#   └─ msg/VisionEvent.msg
# Тот же приём, что и в test_vision_hailo_node.py / test_vision_events_aggregator.py.
_HERE = Path(__file__).resolve()
# `parents[2]` = .../src/rob_box_perception  → содержит пакет rob_box_perception/.
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

# `parents[3]` = .../src  → содержит соседний пакет rob_box_perception_msgs/.
_PERCEPTION_MSGS_ROOT = _HERE.parents[3] / 'rob_box_perception_msgs'
_VISION_EVENT_MSG = _PERCEPTION_MSGS_ROOT / 'msg' / 'VisionEvent.msg'


# ---------- import target under test --------------------------------------

loader_mod = importlib.import_module('rob_box_perception.vision_hailo_loader')
VISION_EVENT_FIELDS = loader_mod.VISION_EVENT_FIELDS


# ---------- helpers --------------------------------------------------------


# Регексп для одной строки поля `.msg`. Формат:
#   <type> <field_name>
# Возможны модификаторы массива `[]` (мы их НЕ используем в VisionEvent, но
# учитываем на будущее, чтобы регексп не сломался, если кто-то добавит).
#
# Примеры строк, которые мы должны распарсить:
#   builtin_interfaces/Time stamp
#   string source_camera
#   int32 class_id
#   float32 confidence
#   float32 bbox_cx
#
# А эти строки МЫ должны пропустить:
#   `# comment`
#   пустые строки
_MSG_FIELD_RE = re.compile(
    r'^\s*(?P<type>[A-Za-z_][A-Za-z0-9_/\[\] ]*?)\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*$'
)


def _parse_msg_field_names(msg_path: Path) -> Set[str]:
    """Извлечь имена полей из `.msg`-файла.

    Не зависит от colcon build / ament / rclpy — читает файл как текст и
    парсит по регекспу. Это и есть primary check, который работает всегда.
    """
    if not msg_path.exists():
        # Если IDL исчез — это критическая проблема для репо; но мы
        # даём внятный failure, а не молчаливый skip.
        raise FileNotFoundError(
            f'VisionEvent.msg не найден: {msg_path}. '
            f'IDL — единственный source of truth для контракта.'
        )

    field_names: Set[str] = set()
    for raw_line in msg_path.read_text(encoding='utf-8').splitlines():
        # Убираем trailing-комментарии (в `.msg` нет `#` комментариев
        # в ROS 2 IDL, но на всякий случай).
        line = raw_line.split('#', 1)[0].rstrip()
        if not line.strip():
            continue
        m = _MSG_FIELD_RE.match(line)
        if m:
            field_names.add(m.group('name'))
    return field_names


# ---------- primary check -------------------------------------------------


def test_vision_event_field_names_match_msg_idl():
    """VISION_EVENT_FIELDS == поля VisionEvent.msg без 'stamp'.

    Это primary check: парсим IDL напрямую. Должен работать в любой среде,
    даже без colcon build и без ROS вообще.
    """
    msg_field_names = _parse_msg_field_names(_VISION_EVENT_MSG)
    msg_data_field_names = msg_field_names - {'stamp'}

    missing = msg_data_field_names - set(VISION_EVENT_FIELDS)
    extra = set(VISION_EVENT_FIELDS) - msg_data_field_names

    assert not missing and not extra, (
        f'VISION_EVENT_FIELDS (в vision_hailo_loader.py) рассинхронизирован '
        f'с VisionEvent.msg.\n'
        f'  Отсутствуют в VISION_EVENT_FIELDS (есть в .msg): {sorted(missing) or "—"}'
        f'\n'
        f'  Лишние в VISION_EVENT_FIELDS (нет в .msg): {sorted(extra) or "—"}'
        f'\n'
        f'Полный список полей .msg: {sorted(msg_field_names)}'
        f'\n'
        f'Полный список VISION_EVENT_FIELDS: {list(VISION_EVENT_FIELDS)}'
        f'\n'
        f'Контракт: VisionEvent.msg и VISION_EVENT_FIELDS должны быть '
        f'идентичны за вычетом stamp (его ставит нода отдельно).'
    )


def test_vision_event_field_count_is_13():
    """VISION_EVENT_FIELDS содержит ровно 13 полей.

    Ловит silent add/remove: если кто-то добавит поле в tuple, забыв
    обновить `.msg` (или наоборот), этот тест заорёт громко.
    """
    assert len(VISION_EVENT_FIELDS) == 13, (
        f'VISION_EVENT_FIELDS должен содержать ровно 13 полей (по числу '
        f'полей данных в VisionEvent.msg, без stamp). Сейчас: '
        f'{len(VISION_EVENT_FIELDS)}.\n'
        f'Поля: {list(VISION_EVENT_FIELDS)}'
    )


def test_vision_event_msg_has_14_fields_including_stamp():
    """VisionEvent.msg содержит 14 полей: stamp + 13 data.

    Сейчас ровно 14 (`stamp` + 13 data-полей, описанных в
    `VISION_EVENT_FIELDS`). Если IDL расширят — обновлять ОБЕ стороны.
    """
    msg_field_names = _parse_msg_field_names(_VISION_EVENT_MSG)
    assert len(msg_field_names) == 14, (
        f'VisionEvent.msg должен содержать 14 полей (stamp + 13 data). '
        f'Сейчас: {len(msg_field_names)}.\n'
        f'Поля: {sorted(msg_field_names)}'
    )
    assert 'stamp' in msg_field_names, (
        f'VisionEvent.msg обязан содержать поле stamp (builtin_interfaces/Time).'
    )


def test_stamp_is_excluded_from_vision_event_fields():
    """`stamp` НЕ должен входить в VISION_EVENT_FIELDS.

    `stamp` ставится нодой отдельно через `set_msg_fields` / `rclpy.time`,
    и его НЕТ в normalized-event-dict. Если кто-то случайно добавит
    `stamp` в tuple — сломается `_publish_event` (не сможет найти
    `norm['stamp']`) и `on_hailo_vision_event` (getattr на msg.stamp
    вне loop).
    """
    assert 'stamp' not in VISION_EVENT_FIELDS, (
        f'stamp не должен входить в VISION_EVENT_FIELDS — он ставится '
        f'отдельно. Сейчас: {list(VISION_EVENT_FIELDS)}'
    )


# ---------- secondary check: against generated msg class ------------------


def _try_import_generated_msg_class():
    """Попробовать импортировать сгенерированный ROS msg-класс.

    Возвращает класс или None. Не падает, если не получилось — это
    нормально для чистого unit-env без colcon build.
    """
    try:
        from rob_box_perception_msgs.msg import VisionEvent  # type: ignore
        return VisionEvent
    except Exception:
        return None


def test_vision_event_fields_match_generated_msg_class():
    """Если msg-класс сгенерирован (colcon build) — проверить через него.

    На dev-машине / в Docker с собранным workspace мы получаем самую
    строгую проверку: `get_fields_and_field_types()` даёт ровно те же
    ключи, что и IDL, минуя любые ошибки ручного парсинга. Это
    дополнительная страховка к IDL-парсеру выше.

    Если класс не импортируется (чистый unit-env) — skip с понятной
    причиной.
    """
    VisionEvent = _try_import_generated_msg_class()
    if VisionEvent is None:
        pytest.skip(
            'rob_box_perception_msgs.msg.VisionEvent не импортируется '
            '(вероятно, не выполнен colcon build). '
            'Primary check через IDL-парсер уже покрыл контракт.'
        )

    # ROS-generated msg-классы экспонируют имена полей через
    # get_fields_and_field_types() — это каноничный API.
    try:
        generated_fields = set(VisionEvent.get_fields_and_field_types().keys())
    except Exception as exc:
        pytest.skip(
            f'VisionEvent.get_fields_and_field_types() не сработал: {exc}. '
            f'Primary check через IDL-парсер уже покрыл контракт.'
        )

    generated_data_fields = generated_fields - {'stamp'}

    missing = generated_data_fields - set(VISION_EVENT_FIELDS)
    extra = set(VISION_EVENT_FIELDS) - generated_data_fields

    assert not missing and not extra, (
        f'VISION_EVENT_FIELDS рассинхронизирован со сгенерированным '
        f'VisionEvent msg-классом.\n'
        f'  Отсутствуют в VISION_EVENT_FIELDS (есть в сгенерированном '
        f'классе): {sorted(missing) or "—"}\n'
        f'  Лишние в VISION_EVENT_FIELDS (нет в сгенерированном классе): '
        f'{sorted(extra) or "—"}\n'
        f'  Сгенерированные поля (полный список): {sorted(generated_fields)}\n'
        f'  VISION_EVENT_FIELDS: {list(VISION_EVENT_FIELDS)}\n'
        f'Контракт: IDL ↔ generated msg ↔ VISION_EVENT_FIELDS должны '
        f'быть идентичны за вычетом stamp.'
    )
