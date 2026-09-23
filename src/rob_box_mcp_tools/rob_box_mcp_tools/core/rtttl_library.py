"""SQLite-хранилище RTTTL-мелодий: архив в репо → миграция в БД на диске.

Мелодии лежат в ``data/rtttl_melodies.jsonl.gz`` (сжато, ~0.7 MB в репо).
Мигратор (``RtttlLibrary.__init__``) при первом запуске читает архив и
разворачивает его в таблицу ``rtttl_melodies`` (SQLite, на диске) одной
транзакцией. Дальше поиск идёт SQL-запросами — в ОЗУ держится только
результат запроса, а не все 10460 записей (на Raspberry Pi ОЗУ мало).

Формат строки JSONL в архиве:
    {"name": slug, "title": "...", "artist": "...", "source": "...",
     "tags": [...], "rtttl": "Name:d=4,o=5,b=...:ноты"}
"""

from __future__ import annotations

import gzip
import json
import os
import re
import sqlite3
import threading
from datetime import datetime, timezone
from importlib.resources import as_file, files
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, TextIO, Union

__all__ = ["RtttlLibrary"]

#: Имя архива внутри пакета ``rob_box_mcp_tools/data/``.
_ARCHIVE_NAME = "rtttl_melodies.jsonl.gz"

#: Размер батча INSERT при импорте архива — держит пик ОЗУ в узде.
_BATCH = 400

_SCHEMA = """
CREATE TABLE IF NOT EXISTS rtttl_melodies (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    name        TEXT    NOT NULL,
    title       TEXT    NOT NULL DEFAULT '',
    artist      TEXT    NOT NULL DEFAULT '',
    source      TEXT    NOT NULL DEFAULT '',
    tags        TEXT    NOT NULL DEFAULT '[]',
    rtttl       TEXT    NOT NULL UNIQUE,
    rtttl_name  TEXT    NOT NULL DEFAULT '',
    created_at  TEXT    NOT NULL,
    updated_at  TEXT    NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_rtttl_melodies_name  ON rtttl_melodies(name);
CREATE INDEX IF NOT EXISTS idx_rtttl_melodies_title ON rtttl_melodies(title);
CREATE INDEX IF NOT EXISTS idx_rtttl_melodies_artist ON rtttl_melodies(artist);
"""

#: Русские/жаргонные названия → канонический англ. запрос (архив англоязычный).
_ALIASES = {
    "гимн ссср": "soviet anthem",
    "гимн россии": "soviet anthem",
    "советский гимн": "soviet anthem",
    "гимн": "soviet anthem",
    "ссср": "soviet anthem",
    "ussr": "soviet anthem",
    "имперский марш": "imperial march",
    "дарт вейдер": "imperial march",
    "в пещере горного короля": "mountain king",
    "григ": "mountain king",
    "тетрис": "tetris",
    "коробейники": "tetris",
    "марио": "mario",
    "супер марио": "mario",
    "нокиа": "nokia",
    "к элизе": "fur elise",
    "ода к радости": "ode to joy",
    "с днём рождения": "happy birthday",
    "с днем рождения": "happy birthday",
    "джингл белс": "jingle bells",
    "звёздные войны": "star wars",
    "звездные войны": "star wars",
}

_ALIAS_SORTED = sorted(_ALIASES.items(), key=lambda kv: -len(kv[0]))

#: Известные мусорные записи архива: под правдоподобным именем/тегами лежит
#: вырожденная запись (короткий мотив, зациклённый N раз, «визжащая» октава).
#: issue #2840: ``get('russian anthem')`` находил ``russiann`` — 10-нотный
#: цикл ``2e,d,c,2d,c,d,2e,g,e,1d`` × 2 в o=7 — вместо настоящей темы
#: Александрова (``national_2``). ``national`` — тот же мусор под другим
#: слагом. Список не запрещает записи (прямой ``get('russiann')`` по имени
#: их всё ещё честно найдёт), а только понижает их в ранжировании
#: неточных/токенных совпадений — см. :func:`_rank_score`.
_GARBAGE_NAMES = frozenset({"russiann", "national"})

#: Насколько сильно давит денилист на ранжирование — должен перевешивать
#: разницу в токен-скоре (:func:`_score` даёт максимум ~4 очка за токен),
#: иначе «мусор» с более точным текстовым совпадением всё равно победит.
_GARBAGE_PENALTY = 60.0

#: Вес токен-скора относительно качества мелодии в комбинированном ранге —
#: текстовое совпадение остаётся главным критерием, качество решает только
#: при близких/равных текстовых скорах (или топит явный мусор).
_MATCH_WEIGHT = 10.0

#: Буква тона (без диеза/бемоля/октавы) → полутон в пределах октавы.
_NOTE_PITCH_RE = re.compile(
    r"^(?P<dur>\d*)(?P<pitch>[a-gp])(?P<mod>[#_]?)(?P<dot1>\.?)"
    r"(?P<octave>\d?)(?P<dot2>\.?)$",
    re.IGNORECASE,
)


def _parse_notes_for_quality(rtttl: str) -> tuple:
    """RTTTL-строка → (список (буква, октава) без пауз, октава по умолчанию).

    Упрощённый разбор для эвристики качества: не обязан быть побитово точным
    (диезы/точки игнорируются) — важны только буква тона и октава, чтобы
    оценить разнообразие высот и диапазон.
    """
    parts = (rtttl or "").split(":")
    if len(parts) < 3:
        return [], 5
    defaults = parts[1]
    notes_part = ":".join(parts[2:])
    m = re.search(r"o\s*=\s*(\d)", defaults, re.IGNORECASE)
    default_octave = int(m.group(1)) if m else 5
    notes: List[tuple] = []
    for tok in notes_part.split(","):
        tok = tok.strip()
        if not tok:
            continue
        match = _NOTE_PITCH_RE.match(tok)
        if not match:
            continue
        pitch = match.group("pitch").lower()
        if pitch == "p":
            continue
        octv_str = match.group("octave")
        octv = int(octv_str) if octv_str else default_octave
        notes.append((pitch, octv))
    return notes, default_octave


def _shortest_repeating_period(seq: List[tuple]) -> Optional[int]:
    """Наименьший период ``p < len(seq)``, для которого ``seq`` = мотив × N.

    ``None``, если запись не является целым числом повторов одного мотива.
    """
    n = len(seq)
    for p in range(1, n // 2 + 1):
        if n % p:
            continue
        if all(seq[i] == seq[i % p] for i in range(n)):
            return p
    return None


def _row_rtttl(row: sqlite3.Row) -> str:
    """``rtttl`` строки-кандидата, безопасно (не все SELECT его выбирают)."""
    return row["rtttl"] if "rtttl" in row.keys() else ""


def _melody_quality(rtttl: str) -> float:
    """Эвристическое качество мелодии: длина, разнообразие, повтор, октава.

    Выше — лучше. Используется, чтобы среди кандидатов с близким текстовым
    совпадением («russian anthem» матчит и мусорный ``russiann``, и
    настоящую тему по токену «anthem») предпочесть содержательную запись, а
    не короткий зацикленный мотив в визгливой октаве.
    """
    try:
        notes, default_octave = _parse_notes_for_quality(rtttl)
    except Exception:
        return 0.0
    n = len(notes)
    if n == 0:
        return -10.0
    score = 0.0
    # Длина: больше нот — содержательнее тема, но с убывающей отдачей.
    score += min(n, 40) * 0.15
    # Разнообразие высот: сколько разных ступеней и (ступень, октава) пар.
    distinct_letters = len({p for p, _ in notes})
    score += distinct_letters * 1.0
    distinct_pairs = len({(p, o) for p, o in notes})
    score += (distinct_pairs / n) * 5.0
    # Штраф за N-кратный повтор одного короткого мотива (визитная карточка
    # мусорных записей вроде RussianN — 10 нот × 2).
    period = _shortest_repeating_period(notes)
    if period is not None and period < n:
        repeats = n // period
        score -= (repeats - 1) * 3.0
    # Разумность октавы по умолчанию: o=7+ или o<=2 — почти всегда брак
    # записи (див «визжащий» регистр), а не осмысленный выбор.
    if default_octave >= 7 or default_octave <= 2:
        score -= abs(default_octave - 5) * 2.0
    return score


#: Английские стоп-слова, встречающиеся почти в каждом названии («of», «the»,
#: «and»). Без их отсева LIKE-кандидаты раздуваются до всего архива, LIMIT-окно
#: обрезает реальные совпадения — «National Anthem Of Soviet» находил «American
#: National Anthem» (токен «of» матчил всё, и нужная строка не влезала в окно).
_STOPWORDS = frozenset({
    "of", "the", "a", "an", "and", "in", "on", "for", "to", "with",
    "from", "at", "by", "it", "is", "are", "this", "that", "de", "la",
})


def _normalize(query: str) -> str:
    """Нижний регистр + замена русских/жаргонных имён на канонический англ."""
    q = (query or "").strip().lower()
    for key, value in _ALIAS_SORTED:
        if key in q:
            q = q.replace(key, value)
    return q


def _tokens(query: str) -> List[str]:
    """Разбить запрос на значимые токены (кириллица отбрасывается после алиасов)."""
    return [
        t for t in re.split(r"[^a-z0-9]+", query) if t and t not in _STOPWORDS
    ]


def _default_archive() -> Union[Path, Any]:
    """Bundled ресурс (importlib.resources) → fallback на дерево исходников."""
    try:
        res = files("rob_box_mcp_tools.data").joinpath(_ARCHIVE_NAME)
        if res.is_file():
            return res
    except (ModuleNotFoundError, TypeError):
        pass
    return Path(__file__).resolve().parent.parent / "data" / _ARCHIVE_NAME


def _iter_archive_rows(archive: Union[Path, Any]) -> Iterator[Dict[str, Any]]:
    """Построчно отдавать записи из gzip-архива (не копируя всё в память)."""
    if isinstance(archive, Path):
        with gzip.open(archive, "rt", encoding="utf-8") as fh:
            yield from _iter_json_lines(fh)
    else:
        with as_file(archive) as p:
            with gzip.open(p, "rt", encoding="utf-8") as fh:
                yield from _iter_json_lines(fh)


def _iter_json_lines(fh: TextIO) -> Iterator[Dict[str, Any]]:
    for line in fh:
        line = line.strip()
        if line:
            yield json.loads(line)


class RtttlLibrary:
    """Мигратор архива RTTTL-мелодий + поиск через SQLite (без in-memory).

    При первом ``__init__`` читает ``data/rtttl_melodies.jsonl.gz`` и
    разворачивает его в таблицу ``rtttl_melodies`` той же БД, что и
    TrackLibrary (``VOICE_MEMORY_DB_PATH`` → ``/data/voice_memory.db``).
    Импорт идемпотентен: если таблица уже заполнена — архив не читается.
    """

    def __init__(
        self,
        db_path: Optional[str] = None,
        archive_path: Optional[str] = None,
    ) -> None:
        self._db_path = db_path or os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        self._archive: Union[Path, Any] = (
            Path(archive_path) if archive_path else _default_archive()
        )
        self._lock = threading.Lock()

        os.makedirs(os.path.dirname(self._db_path) or ".", exist_ok=True)
        self._conn = sqlite3.connect(self._db_path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        with self._lock:
            self._conn.execute("PRAGMA journal_mode=WAL")
            self._conn.executescript(_SCHEMA)
            self._conn.commit()
            self._migrate_from_archive()

    # ------------------------------------------------------------------
    # Миграция: архив → SQLite (одна транзакция, идемпотентно)
    # ------------------------------------------------------------------

    def _migrate_from_archive(self) -> None:
        self._ensure_rtttl_name_column()
        count = self._conn.execute("SELECT COUNT(*) FROM rtttl_melodies").fetchone()[0]
        if count == 0:
            now = datetime.now(timezone.utc).isoformat()
            insert = (
                "INSERT OR IGNORE INTO rtttl_melodies "
                "(name, title, artist, source, tags, rtttl, created_at, updated_at) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?)"
            )
            with self._conn:  # одна транзакция — частичный импорт невозможен
                batch: List[tuple] = []
                for rec in _iter_archive_rows(self._archive):
                    batch.append((
                        rec.get("name", ""),
                        rec.get("title", ""),
                        rec.get("artist", ""),
                        rec.get("source", ""),
                        json.dumps(rec.get("tags") or [], ensure_ascii=False),
                        rec.get("rtttl", ""),
                        now,
                        now,
                    ))
                    if len(batch) >= _BATCH:
                        self._conn.executemany(insert, batch)
                        batch = []
                if batch:
                    self._conn.executemany(insert, batch)
        # Для уже существующих и свежесозданных БД: добить rtttl_name и
        # восстановить потерянные имена (title='Unknown' → artist).
        self._repair_missing_names()

    def _ensure_rtttl_name_column(self) -> None:
        """Добавить колонку ``rtttl_name`` в уже существующую БД (ALTER)."""
        cols = {row[1] for row in self._conn.execute("PRAGMA table_info(rtttl_melodies)")}
        if "rtttl_name" not in cols:
            self._conn.execute(
                "ALTER TABLE rtttl_melodies ADD COLUMN rtttl_name TEXT NOT NULL DEFAULT ''"
            )
            self._conn.commit()

    def _repair_missing_names(self) -> None:
        """Восстановить имя из двух источников, которые миграция потеряла.

        Архив местами хранит ``name='unknown_NNN'``, ``title='Unknown'``, а
        реальное имя — в ``artist`` («Batman V1.0»). Дополнительно в самом
        формате RTTTL есть поле имени (префикс строки до первого ':'): его
        кладём в ``rtttl_name``, чтобы поиск матчил и по нему.
        """
        with self._conn:
            self._conn.execute(
                "UPDATE rtttl_melodies "
                "SET rtttl_name = substr(rtttl, 1, instr(rtttl, ':') - 1) "
                "WHERE rtttl_name = '' AND instr(rtttl, ':') > 0"
            )
            self._conn.execute(
                "UPDATE rtttl_melodies SET title = artist "
                "WHERE (title IS NULL OR trim(title) = '' OR lower(trim(title)) = 'unknown') "
                "AND artist IS NOT NULL AND trim(artist) != '' "
                "AND lower(trim(artist)) != 'unknown'"
            )

    # ------------------------------------------------------------------
    # Поиск (SQL, на диске)
    # ------------------------------------------------------------------

    @staticmethod
    def _to_dict(row: sqlite3.Row, include_rtttl: bool = False) -> Dict[str, Any]:
        d = dict(row)
        d["tags"] = json.loads(d.get("tags") or "[]")
        for key in ("id", "created_at", "updated_at"):
            d.pop(key, None)
        if not include_rtttl:
            d.pop("rtttl", None)
        return d

    def total(self) -> int:
        with self._lock:
            return self._conn.execute("SELECT COUNT(*) FROM rtttl_melodies").fetchone()[0]

    @staticmethod
    def _score(row: sqlite3.Row, tokens: List[str]) -> int:
        """Скоринг: сколько токенов запроса попало в поля (name/rtttl_name весомее)."""
        name_l = (row["name"] or "").lower()
        title_l = (row["title"] or "").lower()
        artist_l = (row["artist"] or "").lower()
        tags_l = (row["tags"] or "").lower()
        rtttl_name_l = (row["rtttl_name"] or "").lower()
        score = 0
        for token in tokens:
            if token == name_l or token == rtttl_name_l:
                score += 4
            elif token in title_l:
                # title несёт полное имя («Happy Birthday To You») — ценнее,
                # чем подстрока slug'а. Иначе «happy birthday» выигрывает
                # трек Ashanti «Happy» (token == name) у правильного.
                score += 3
            elif token in name_l or token in rtttl_name_l:
                score += 2
            elif token in artist_l or token in tags_l:
                score += 1
        return score

    @classmethod
    def _rank_score(cls, row: sqlite3.Row, tokens: List[str]) -> float:
        """Комбинированный ранг: текстовый скор (главный) + качество - штраф.

        Текст решает основную часть ранга (``_MATCH_WEIGHT`` на очко), но
        качество мелодии сглаживает выбор между близкими совпадениями, а
        известный мусор (``_GARBAGE_NAMES``) получает штраф, перевешивающий
        обычный разрыв в текстовом скоре — так «russian anthem» вместо
        зацикленного 10-нотного ``russiann`` находит полноценную тему
        ``national_2``, даже если та матчит меньше токенов буквально.

        Возвращает ``-inf``, если текстового совпадения вообще нет (скор 0)
        — такую строку выбирать нельзя, иначе честный "не найдено" подменится
        случайной записью, где качество просто оказалось выше нуля.
        """
        match = cls._score(row, tokens)
        if match <= 0:
            return float("-inf")
        quality = _melody_quality(_row_rtttl(row))
        name_l = (row["name"] or "").strip().lower()
        penalty = _GARBAGE_PENALTY if name_l in _GARBAGE_NAMES else 0.0
        return match * _MATCH_WEIGHT + quality - penalty

    def _candidates(self, tokens: List[str], cap: int) -> List[sqlite3.Row]:
        """Строки, где хотя бы один токен встречается в полях (метаданные)."""
        clauses = []
        params: List[str] = []
        for token in tokens:
            like = f"%{token}%"
            clauses.append(
                "(lower(name) LIKE ? OR lower(title) LIKE ? "
                "OR lower(artist) LIKE ? OR lower(tags) LIKE ? "
                "OR lower(rtttl_name) LIKE ?)"
            )
            params += [like, like, like, like, like]
        sql = (
            "SELECT id, name, title, artist, source, tags, rtttl_name, rtttl "
            "FROM rtttl_melodies WHERE " + " OR ".join(clauses) + " LIMIT ?"
        )
        return self._conn.execute(sql, params + [cap]).fetchall()

    def get(self, name: str) -> Optional[Dict[str, Any]]:
        """Найти одну мелодию (точное имя → лучший по токенам запроса).

        Точное совпадение имени обычно возвращается сразу (прямая
        адресация: ``get('russiann')`` честно находит ``russiann``). Но
        если точное имя само в денилисте (``_GARBAGE_NAMES``), оно не
        побеждает автоматически — участвует в ранжировании наравне с
        токен-кандидатами и уступает более качественной записи, если такая
        нашлась (иначе остаётся честным fallback'ом, когда лучшего нет).
        """
        q = _normalize(name)
        tokens = _tokens(q)
        if not tokens:
            return None
        with self._lock:
            exact = self._conn.execute(
                "SELECT * FROM rtttl_melodies WHERE lower(name) = ? LIMIT 1", (q,)
            ).fetchone()
            exact_name_l = ""
            if exact is not None:
                exact_name_l = (exact["name"] or "").strip().lower()
            if exact is not None and exact_name_l not in _GARBAGE_NAMES:
                return self._to_dict(exact, include_rtttl=True)
            rows = self._candidates(tokens, cap=2000)
        best: Optional[sqlite3.Row] = None
        best_rank = float("-inf")
        if exact is not None:
            best = exact
            best_rank = self._rank_score(exact, tokens)
        for row in rows:
            rank = self._rank_score(row, tokens)
            if rank > best_rank:
                best_rank = rank
                best = row
        if best is None:
            return None
        with self._lock:
            full = self._conn.execute(
                "SELECT * FROM rtttl_melodies WHERE id = ?", (best["id"],)
            ).fetchone()
        if full is None:
            return None
        return self._to_dict(full, include_rtttl=True)

    def search(self, query: str, limit: int = 20) -> List[Dict[str, Any]]:
        """Поиск по токенам запроса (SQL кандидаты → скоринг в Python), top-N.

        Сортировка: текстовый скор (главный ключ, как раньше) → title
        (алфавит, прежний тай-брейк) → качество мелодии (последний
        тай-брейк, для истинных ничьих по скору и названию). Денилист бьёт
        по позиции внутри той же текстовой группы через штраф в скоре.
        """
        q = _normalize(query)
        tokens = _tokens(q)
        if not tokens:
            return []
        limit = max(1, min(50, int(limit)))
        with self._lock:
            rows = self._candidates(tokens, cap=limit * 10)
        scored = []
        for row in rows:
            match = self._score(row, tokens)
            if match <= 0:
                continue
            name_l = (row["name"] or "").strip().lower()
            penalty = _GARBAGE_PENALTY if name_l in _GARBAGE_NAMES else 0.0
            effective = match - penalty / _MATCH_WEIGHT
            quality = _melody_quality(_row_rtttl(row))
            scored.append((effective, row, quality))

        def _sort_key(item: tuple) -> tuple:
            effective, row, quality = item
            return (-effective, (row["title"] or "").lower(), -quality)

        scored.sort(key=_sort_key)
        return [self._to_dict(row) for _eff, row, _quality in scored[:limit]]
