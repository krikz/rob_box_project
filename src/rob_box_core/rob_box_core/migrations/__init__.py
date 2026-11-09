"""Миграции SQLite памяти: один нумерованный поток ``NNN_*.sql`` - данные пакета ядра (ADR-0167 §2.4, шаг Ш1а).

Что это. Схема БД инструментов (:mod:`rob_box_core.tools_db`: факты, FAQ, карты и точки, архив музыки) и маркеры
переноса памяти собеседников. Файлы лежат здесь же, рядом с этим модулем, и ставятся вместе с пакетом
(``package_data`` в ``setup.py`` для colcon робота, ``package-data`` в ``packaging/dialog-core/pyproject.toml``).
Раньше поток лежал в корне чекаута (``migrations/``), и каждый потребитель искал его сам: вверх от своего файла,
через монтирование ``/migrations`` в контейнер - ядро, поставленное пакетом, его не находило.

Как читать. :func:`stream` - весь поток по порядку версий, :func:`script` - один файл по имени, :func:`apply` -
применить к соединению шаги новее его ``PRAGMA user_version``. Ресурсов в пакете нет - ошибка: молча пустой поток
оставил бы БД без таблиц.

Поток один на несколько БД (в нём таблицы разных владельцев) - так было и в корне чекаута; делить его - отдельная
работа, не перенос.
"""

from __future__ import annotations

import contextlib
import sqlite3
from importlib import resources
from typing import ContextManager, Iterable, NamedTuple, Optional, Tuple


class Migration(NamedTuple):
    """Один шаг потока: версия из префикса имени, имя файла, текст SQL."""

    version: int
    name: str
    sql: str


def _sql_files():
    files = sorted((f for f in resources.files(__name__).iterdir() if f.name.endswith(".sql")), key=lambda f: f.name)
    if not files:
        raise FileNotFoundError(f"в пакете {__name__} нет *.sql: ядро поставлено без данных (package_data)")
    return files


def stream() -> Tuple[Migration, ...]:
    """Весь поток по возрастанию имени; файл без числового префикса - не шаг потока (пропуск, как раньше)."""
    out = []
    for f in _sql_files():
        try:
            version = int(f.name.split("_", 1)[0])
        except ValueError:
            continue
        out.append(Migration(version, f.name, f.read_text(encoding="utf-8")))
    return tuple(out)


def script(name: str) -> str:
    """Текст одного файла потока по имени (``003_waypoints.sql``); нет такого - ``FileNotFoundError``."""
    for f in _sql_files():
        if f.name == name:
            return f.read_text(encoding="utf-8")
    raise FileNotFoundError(f"нет миграции {name} в пакете {__name__}")


def apply(conn: sqlite3.Connection, steps: Iterable[Migration], lock: Optional[ContextManager] = None) -> None:
    """Применить шаги новее ``PRAGMA user_version`` БД, каждый в своей транзакции, и поднять версию.

    ``lock`` - замок владельца соединения (берётся на чтение версии и на каждый шаг). Ошибка шага не глотается.
    """
    guard = lock if lock is not None else contextlib.nullcontext()
    with guard:
        current = conn.execute("PRAGMA user_version").fetchone()[0]
    for step in steps:
        if step.version <= current:
            continue
        with guard, conn:
            conn.executescript(step.sql)
            conn.execute(f"PRAGMA user_version = {step.version}")


__all__ = ["Migration", "apply", "script", "stream"]
