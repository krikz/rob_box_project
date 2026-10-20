"""Заглушки ROS-сообщений, собранные из настоящих ``.msg`` файлов.

Тесты без ROS-рантайма (Windows, CI-unit) не могут импортировать
``rob_box_dialog_msgs.msg``: rosidl там не собирается. Рукописная заглушка
расходилась бы с определением: поле переименовали в ``.msg``, тест остался
зелёным. Здесь класс строится ИЗ ``.msg``-файла, поэтому набор полей, их
значения по умолчанию и константы всегда совпадают с настоящим типом.

Только для тестов и инструментов; в рантайме робота не используется.
"""

from __future__ import annotations

import re
import sys
import types
from pathlib import Path
from typing import Any, Callable, Dict, Optional

_SCALAR_DEFAULTS: Dict[str, Any] = {
    "bool": False,
    "float32": 0.0,
    "float64": 0.0,
    "string": "",
}
_INT_TYPES = ("int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64")
_LINE = re.compile(r"^(?P<type>[\w/]+(?:\[\])?)\s+(?P<name>\w+)(?:\s*=\s*(?P<value>.+))?$")


def _parse_constant(type_name: str, raw: str) -> Any:
    raw = raw.strip()
    if type_name == "string":
        return raw.strip("\"'")
    if type_name == "bool":
        return raw.lower() in ("true", "1")
    if type_name.startswith("float"):
        return float(raw)
    return int(raw)


def _default_for(type_name: str, registry: Dict[str, type]) -> Callable[[], Any]:
    if type_name.endswith("[]"):
        return list
    if type_name == "builtin_interfaces/Time":
        return lambda: types.SimpleNamespace(sec=0, nanosec=0)
    if type_name in _INT_TYPES:
        return int
    if type_name in _SCALAR_DEFAULTS:
        value = _SCALAR_DEFAULTS[type_name]
        return lambda: value
    short = type_name.split("/")[-1]
    return lambda: registry[short]()


def _class_from_msg(name: str, text: str, registry: Dict[str, type]) -> type:
    constants: Dict[str, Any] = {}
    fields: Dict[str, Callable[[], Any]] = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or line == "---":
            continue
        match = _LINE.match(line)
        if match is None:
            raise ValueError(f"{name}: не разобрана строка {line!r}")
        type_name, field, value = match["type"], match["name"], match["value"]
        if value is not None:
            constants[field] = _parse_constant(type_name, value)
        else:
            fields[field] = _default_for(type_name, registry)

    def __init__(self: Any, **kwargs: Any) -> None:
        for key, factory in fields.items():
            setattr(self, key, factory())
        for key, value in kwargs.items():
            if key not in fields:
                raise AttributeError(f"{name} не имеет поля {key!r}")
            setattr(self, key, value)

    def __repr__(self: Any) -> str:
        shown = ", ".join(f"{k}={getattr(self, k)!r}" for k in fields)
        return f"{name}({shown})"

    return type(name, (), {**constants, "__init__": __init__, "__repr__": __repr__,
                           "_FIELDS": tuple(fields)})


def build_msg_module(msg_dir: Path, package: str, module_name: Optional[str] = None) -> types.ModuleType:
    """Собрать модуль ``<package>.msg`` из каталога с ``*.msg``.

    Вложенные типы (``SceneFrame`` в ``SessionState``) разрешаются по имени:
    сообщения создаются в порядке зависимостей.
    """
    texts = {p.stem: p.read_text(encoding="utf-8") for p in sorted(Path(msg_dir).glob("*.msg"))}
    registry: Dict[str, type] = {}
    pending = dict(texts)
    while pending:
        progressed = False
        for name, text in list(pending.items()):
            deps = {
                m["type"].split("/")[-1].replace("[]", "")
                for m in (_LINE.match(ln.split("#", 1)[0].strip()) for ln in text.splitlines())
                if m and m["value"] is None and m["type"].split("/")[-1].replace("[]", "") in texts
            }
            if deps - set(registry) - {name}:
                continue
            registry[name] = _class_from_msg(name, text, registry)
            del pending[name]
            progressed = True
        if not progressed:
            raise ValueError(f"циклическая зависимость сообщений: {sorted(pending)}")
    module = types.ModuleType(module_name or f"{package}.msg")
    for name, cls in registry.items():
        setattr(module, name, cls)
    return module


def install_dialog_msgs_stub() -> types.ModuleType:
    """Положить ``rob_box_dialog_msgs.msg`` (из ``.msg`` репозитория) в ``sys.modules``.

    Идемпотентно: если модуль уже есть (настоящий или заглушка), он
    возвращается как есть.
    """
    existing = sys.modules.get("rob_box_dialog_msgs.msg")
    if existing is not None:
        return existing
    msg_dir = Path(__file__).resolve().parents[2] / "rob_box_dialog_msgs" / "msg"
    module = build_msg_module(msg_dir, "rob_box_dialog_msgs")
    package = types.ModuleType("rob_box_dialog_msgs")
    package.msg = module
    sys.modules["rob_box_dialog_msgs"] = package
    sys.modules["rob_box_dialog_msgs.msg"] = module
    return module
