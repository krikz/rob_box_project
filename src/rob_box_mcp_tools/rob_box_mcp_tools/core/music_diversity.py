"""music_diversity.py — память о сыгранном и выбор со штрафом за недавнее.

Issue #3224 (карточка (а) umbrella #3223, ADR-0146). Живой лог 30.09: из
10 club-треков прогрессия ``VI-III-VII-i`` выпала 5 раз, один и тот же
сид сыгран дважды подряд. Выбор был ``rng.choice`` по сиду без учёта того,
что уже играло, а ``_repeat_warning`` помнит только один прошлый вызов.

Модуль чистый (без ROS/Renardo):

* :class:`MusicHistory` — таблица ``music_history`` в SQLite (тот же
  ``VOICE_MEMORY_DB_PATH``, что у ``RtttlLibrary``; схема создаётся кодом
  ``CREATE TABLE IF NOT EXISTS``). Переживает перезапуск.
* :func:`weighted_pick` — взвешенный выбор: штраф за появление значения в
  недавней истории затухает экспоненциально с давностью.

Недоступная БД НЕ роняет музыку и НЕ молчит: пишется WARNING, история
пуста, выбор идёт как без памяти.
"""

from __future__ import annotations

import logging
import os
import sqlite3
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, TypeVar

__all__ = [
    "DEFAULT_DECAY",
    "DEFAULT_FLOOR",
    "HISTORY_FIELDS",
    "MusicHistory",
    "weighted_pick",
]

_LOG = logging.getLogger(__name__)

T = TypeVar("T")

#: Как быстро забывается сыгранное: штраф за значение на i-м месте от
#: свежего = ``decay ** i`` (свежий повтор — полный штраф 1.0).
DEFAULT_DECAY = 0.7

#: Нижняя граница веса: выбор возможен всегда, даже если все опции недавно играли.
DEFAULT_FLOOR = 0.03

#: Поля записи истории (кроме ``id``/``ts``), все опциональны.
HISTORY_FIELDS = (
    "set_id", "style", "melody_name", "fragment_offset", "hook_fingerprint",
    "progression", "template", "kick", "hats", "lead", "bass", "pad",
    "root", "bpm", "scale",
    "clap", "lpf", "balance",  # issue #3226: пулы клэпа/lpf/баланса слоёв
    "sample",  # issue #3254: слой сэмплов DJ_Dave (core/club_samples)
)

#: Колонки, добавленные после первой версии схемы (#3226): старые БД
#: дополняются ``ALTER TABLE ADD COLUMN`` (``CREATE TABLE IF NOT EXISTS``
#: существующую таблицу не меняет).
_ADDED_COLUMNS = (("clap", "TEXT"), ("lpf", "TEXT"), ("balance", "TEXT"), ("sample", "TEXT"))

_SCHEMA = """
CREATE TABLE IF NOT EXISTS music_history (
    id               INTEGER PRIMARY KEY AUTOINCREMENT,
    ts               REAL    NOT NULL,
    set_id           TEXT,
    style            TEXT,
    melody_name      TEXT,
    fragment_offset  INTEGER,
    hook_fingerprint TEXT,
    progression      TEXT,
    template         TEXT,
    kick             TEXT,
    hats             TEXT,
    lead             TEXT,
    bass             TEXT,
    pad              TEXT,
    root             TEXT,
    bpm              REAL,
    scale            TEXT,
    clap             TEXT,
    lpf              TEXT,
    balance          TEXT,
    sample           TEXT
);
CREATE INDEX IF NOT EXISTS idx_music_history_ts ON music_history(ts);
"""


def weighted_pick(
    options: Sequence[T],
    recent_values: Sequence[Any],
    rng: Any,
    *,
    decay: float = DEFAULT_DECAY,
    floor: float = DEFAULT_FLOOR,
) -> T:
    """Выбрать опцию, штрафуя недавно игравшие.

    ``recent_values`` упорядочен от СВЕЖЕГО к старому. Вес опции =
    ``max(floor, 1 - sum(decay ** i))`` по всем позициям ``i``, где
    опция встречалась. Пустая история — равные веса (равномерный выбор).
    Детерминирована при заданном ``rng`` (тратит ровно один ``rng.random()``).

    Raises:
        ValueError: пустые ``options``, ``floor`` не в (0, 1] или ``decay`` не в [0, 1].
    """
    if not options:
        raise ValueError("weighted_pick: пустой список опций")
    if not 0.0 < floor <= 1.0:
        raise ValueError(f"weighted_pick: floor={floor!r} вне (0, 1]")
    if not 0.0 <= decay <= 1.0:
        raise ValueError(f"weighted_pick: decay={decay!r} вне [0, 1]")
    weights = [_option_weight(option, recent_values, decay, floor) for option in options]
    point = rng.random() * sum(weights)
    acc = 0.0
    for option, weight in zip(options, weights):
        acc += weight
        if point < acc:
            return option
    return options[-1]


def _option_weight(option: Any, recent_values: Sequence[Any], decay: float, floor: float) -> float:
    penalty = sum(decay ** i for i, value in enumerate(recent_values) if value == option)
    return max(floor, 1.0 - penalty)


class MusicHistory:
    """Персистентная история сыгранного (таблица ``music_history``).

    ``db_path``: ``None`` → ``$VOICE_MEMORY_DB_PATH`` или ``/data/voice_memory.db``;
    ``":memory:"`` — для тестов. Если БД не открылась — :attr:`available`
    ``False``, ``record`` возвращает ``False``, ``recent`` — ``[]``; причина
    в WARNING (это не молчаливая деградация).
    """

    def __init__(self, db_path: Optional[str] = None) -> None:
        self._db_path = db_path or os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        self._lock = threading.Lock()
        self._conn: Optional[sqlite3.Connection] = None
        try:
            self._conn = self._open(self._db_path)
        except (sqlite3.Error, OSError) as exc:
            _LOG.warning(
                "music_history: БД %s недоступна (%s) — история сыгранного отключена, "
                "выбор каркаса идёт без учёта прошлого", self._db_path, exc,
            )

    @staticmethod
    def _open(db_path: str) -> sqlite3.Connection:
        if db_path != ":memory:":
            os.makedirs(os.path.dirname(db_path) or ".", exist_ok=True)
        conn = sqlite3.connect(db_path, check_same_thread=False)
        conn.row_factory = sqlite3.Row
        try:
            if db_path != ":memory:":
                conn.execute("PRAGMA journal_mode=WAL")
            conn.executescript(_SCHEMA)
            MusicHistory._migrate(conn)
            conn.commit()
        except sqlite3.Error:
            conn.close()
            raise
        return conn

    @staticmethod
    def _migrate(conn: sqlite3.Connection) -> None:
        """Дописать колонки, которых нет в таблице, созданной старой версией."""
        have = {row[1] for row in conn.execute("PRAGMA table_info(music_history)")}
        for name, sql_type in _ADDED_COLUMNS:
            if name not in have:
                conn.execute(f"ALTER TABLE music_history ADD COLUMN {name} {sql_type}")

    @property
    def available(self) -> bool:
        """``True``, если БД открыта и история реально копится."""
        return self._conn is not None

    def announce(self, logger: Any) -> None:
        """Одна строка в лог узла: история подключена или её нет (не молчаливая деградация)."""
        if self._conn is not None:
            logger.info(f"music_history: история сыгранного подключена ({self._db_path})")
        else:
            logger.warning("music_history: БД недоступна — club выбирает без памяти")

    def record(self, **fields: Any) -> bool:
        """Дописать запись о сыгранном. ``ts`` по умолчанию — сейчас.

        Неизвестное поле — ``TypeError`` (опечатка не должна тихо теряться).
        Returns:
            ``True`` — записано; ``False`` — истории нет или запись не удалась (WARNING).
        """
        ts = fields.pop("ts", None)
        unknown = sorted(set(fields) - set(HISTORY_FIELDS))
        if unknown:
            raise TypeError(f"music_history.record: неизвестные поля {unknown}")
        if self._conn is None:
            return False
        columns = ["ts"] + [k for k in HISTORY_FIELDS if k in fields]
        values = [time.time() if ts is None else float(ts)] + [fields[k] for k in columns[1:]]
        sql = f"INSERT INTO music_history ({', '.join(columns)}) VALUES ({', '.join('?' * len(columns))})"
        try:
            with self._lock:
                self._conn.execute(sql, values)
                self._conn.commit()
            return True
        except sqlite3.Error as exc:
            _LOG.warning("music_history: запись не удалась (%s) — этот трек в историю не попал", exc)
            return False

    def recent(self, limit: int = 20, within_sec: Optional[float] = None) -> List[Dict[str, Any]]:
        """Последние записи, СВЕЖИЕ ПЕРВЫМИ (порядок, нужный :func:`weighted_pick`).

        ``within_sec`` — только записи не старше указанного числа секунд.
        Ошибка чтения — WARNING и ``[]``.
        """
        if self._conn is None or limit <= 0:
            return []
        sql, args = "SELECT * FROM music_history", []
        if within_sec is not None:
            sql += " WHERE ts >= ?"
            args.append(time.time() - float(within_sec))
        sql += " ORDER BY ts DESC, id DESC LIMIT ?"
        args.append(int(limit))
        try:
            with self._lock:
                rows = self._conn.execute(sql, args).fetchall()
        except sqlite3.Error as exc:
            _LOG.warning("music_history: чтение не удалось (%s) — выбор без учёта прошлого", exc)
            return []
        return [dict(row) for row in rows]

    def close(self) -> None:
        """Закрыть соединение (тесты; в проде живёт с процессом)."""
        with self._lock:
            if self._conn is not None:
                self._conn.close()
                self._conn = None
