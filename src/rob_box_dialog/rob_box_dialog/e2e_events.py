"""Текст ходов и речи — только в e2e-БД вне ``/data`` (ADR-0150 §7.4, решение В8; PR-8).

В проде текст турнов на диск не пишется (директива Шифу 02.09): журнал ходов живёт в памяти, на диск — агрегаты без
текста (``turn_log.TurnAggregate``). Для приёмки (§14) харнесс даёт путь ``e2e_db_path`` — тогда события хода
пишутся в SQLite, таблица :data:`TABLE`; её читают ``scripts/dialog/turn_metrics.py --db`` и ``honesty_audit.py --db``.
Переключателем служит сам путь: отдельного bool ``e2e_mode`` здесь нет (— режим e2e у робота один, владелец —
``rob_box_core.e2e_mode``; журнал ходов не пишет в ``/data`` и режимом БД не является).

:func:`open_events` — единственная дверь: с пустым путём записи нет (``None``), путь внутри боевой ``/data`` (общая с
живыми людьми, ADR-0128) — громкий отказ ``ValueError``.
"""

from __future__ import annotations

import json
import posixpath
import sqlite3
import threading
import time
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Mapping, Optional

#: Таблица событий диалога в e2e-БД.
TABLE = "dialog_events"
#: Боевой каталог данных робота: e2e-БД внутри него запрещена.
ROBOT_DATA_DIR = PurePosixPath("/data")
#: Виды событий: реплика человека, реплика отдана владельцу речи, началось звучание, отзвучало (озвученная часть),
#: исход действия, команда модели (принята/отвергнута), агрегат хода.
KINDS = ("user", "speech_requested", "speech_started", "speech_spoken", "action", "command", "turn")

_SCHEMA = f"""
CREATE TABLE IF NOT EXISTS {TABLE} (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts REAL NOT NULL,
    session_id TEXT NOT NULL,
    epoch INTEGER NOT NULL,
    turn_id TEXT NOT NULL,
    agent TEXT NOT NULL,
    kind TEXT NOT NULL,
    text TEXT NOT NULL DEFAULT '',
    data_json TEXT NOT NULL DEFAULT '{{}}'
)
"""


def inside_robot_data(path: str) -> bool:
    """Путь лежит в боевой ``/data`` (или это она сама). Сравнение по POSIX-частям после нормализации ``..``:
    ``/database`` — не ``/data``, ``/tmp/../data/x`` — ``/data``."""
    parts = PurePosixPath(posixpath.normpath(str(path).replace("\\", "/"))).parts
    return parts[:len(ROBOT_DATA_DIR.parts)] == ROBOT_DATA_DIR.parts


def _plain(value: Any) -> Any:
    return dict(value) if isinstance(value, Mapping) else str(value)


class DialogEvents:
    """Писатель e2e-БД одного процесса: одно соединение, запись под замком (колбэки ROS из разных потоков)."""

    def __init__(self, path: str, session_id: str, clock: Callable[[], float] = time.time) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._session_id = session_id
        self._clock = clock
        self._lock = threading.Lock()
        self._db = sqlite3.connect(path, check_same_thread=False)
        with self._lock:
            self._db.execute(_SCHEMA)
            self._db.commit()

    def write(self, kind: str, *, turn_id: str, epoch: int, agent: str, text: str = "",
              data: Optional[Mapping[str, Any]] = None) -> None:
        """Записать событие хода. Чужой ``kind`` — ``ValueError`` (метрики читают только известные виды)."""
        if kind not in KINDS:
            raise ValueError(f"dialog_events: kind must be one of {KINDS}, got {kind!r}")
        row = (self._clock(), self._session_id, int(epoch), turn_id, agent, kind, text or "",
               json.dumps(dict(data or {}), ensure_ascii=False, default=_plain))
        with self._lock:
            self._db.execute(f"INSERT INTO {TABLE} (ts, session_id, epoch, turn_id, agent, kind, text, data_json) "
                             "VALUES (?, ?, ?, ?, ?, ?, ?, ?)", row)
            self._db.commit()

    def close(self) -> None:
        with self._lock:
            self._db.close()


def open_events(db_path: str, session_id: str,
                clock: Callable[[], float] = time.time) -> Optional[DialogEvents]:
    """Писатель e2e-БД или ``None`` (прод: путь пуст — текст на диск не попадает).
    Путь в боевой ``/data`` — ``ValueError``: харнесс обязан дать отдельный каталог."""
    path = (db_path or "").strip()
    if not path:
        return None
    if inside_robot_data(path):
        raise ValueError(f"e2e_db_path={path!r} внутри {ROBOT_DATA_DIR}: боевая /data общая с живыми людьми, "
                         "e2e-БД — только вне неё")
    return DialogEvents(path, session_id, clock)


__all__ = ["DialogEvents", "KINDS", "ROBOT_DATA_DIR", "TABLE", "inside_robot_data", "open_events"]
