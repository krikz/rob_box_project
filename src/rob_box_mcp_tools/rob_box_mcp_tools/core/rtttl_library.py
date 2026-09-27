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
import math
import os
import re
import sqlite3
import threading
from datetime import datetime, timezone
from importlib.resources import as_file, files
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, TextIO, Union

__all__ = ["RtttlLibrary", "covers_tokens", "match_info"]

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
    # Архив хранит русский гимн под существительным «Russia» («National
    # Anthem Of Russia»), не под прилагательным «Russian» — без этой
    # замены токен «russian» не матчит запись вовсе, и «russian anthem»
    # решает голое совпадение «anthem» по полусотне чужих гимнов (#2840).
    "russian": "russia",
}

_ALIAS_SORTED = sorted(_ALIASES.items(), key=lambda kv: -len(kv[0]))

#: Известные мусорные записи архива: под правдоподобным именем/тегами лежит
#: вырожденная запись (короткий мотив, зациклённый N раз, «визжащая» октава).
#: issue #2840: ``get('russian anthem')`` находил ``russiann`` — 10-нотный
#: цикл ``2e,d,c,2d,c,d,2e,g,e,1d`` × 2 в o=7 — вместо настоящей темы
#: Александрова (``national_2``). ``national`` — тот же мусор под другим
#: слагом. Список не запрещает записи (прямой ``get('russiann')`` по имени
#: их всё ещё честно найдёт), а только понижает их в ранжировании
#: неточных/токенных совпадений внутри своего же bucket'а текстового скора
#: — см. :meth:`RtttlLibrary._best_in_bucket`. Денилист НЕ может перевесить
#: более высокий текстовый скор другой записи (issue #2840, живой прогон
#: 23.09: «russian anthem» после первой версии фикса находил случайную
#: «Irish National Anthem» — качество перебивало смысл названия).
_GARBAGE_NAMES = frozenset({"russiann", "national"})

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
    """Нижний регистр + замена русских/жаргонных имён на канонический англ.

    Замена — по границам СЛОВ (``\\b``), не голой подстрокой: латинские
    алиасы вроде «russian» — обычные английские слова, и подстрочная
    замена ломает прямую адресацию похожих записей архива (issue #2840,
    живой прогон 23.09: ``get('russiann')`` после подстрочной замены
    «russian»→«russia» превращался в ``get('russian')`` — искал ЧУЖУЮ
    запись вместо честного прямого попадания по имени).
    """
    q = (query or "").strip().lower()
    for key, value in _ALIAS_SORTED:
        pattern = r"\b" + re.escape(key) + r"\b"
        if re.search(pattern, q):
            q = re.sub(pattern, value, q)
    return q


def _tokens(query: str) -> List[str]:
    """Разбить запрос на значимые токены (кириллица отбрасывается после алиасов)."""
    return [
        t for t in re.split(r"[^a-z0-9]+", query) if t and t not in _STOPWORDS
    ]


#: Поля, которые опознают запись (НЕ ``tags`` — жанровые метки, а не
#: опознавание; общее слово вроде «anthem»/«movie» в тегах даёт ложное
#: покрытие — от этого уже страдал скоринг, см. :meth:`RtttlLibrary._score`).
_IDENTITY_FIELDS: tuple = ("name", "title", "artist", "rtttl_name")


def _identity_haystack(record: Dict[str, Any]) -> str:
    return " ".join(str(record.get(f) or "").lower() for f in _IDENTITY_FIELDS)


def covers_tokens(record: Dict[str, Any], query: str) -> bool:
    """Все значимые токены *query* нашлись в опознавательных полях *record*?

    Низкоуровневый строгий (не взвешенный) критерий — каждый токен ЛИБО
    есть подстрокой в ``name``/``title``/``artist``/``rtttl_name``, ЛИБО
    нет. :func:`match_info` строит на нём взвешенную (IDF) версию для
    тулов — сам этот критерий тулами больше НЕ используется как вердикт
    found/not-found (issue #2964, товарищ Шифу: «жёсткий порог … тоже не
    делай», см. историю отката #2882→#2896 — единой эвристики отказа не
    нашлось, «лишние слова» отличают и «другую песню», и просто длинное
    название). Полезен как отдельная проверка и как строительный блок.
    """
    tokens = _tokens(_normalize(query))
    if not tokens:
        return False
    haystack = _identity_haystack(record)
    return all(token in haystack for token in tokens)


#: Кэш IDF-веса токена — процесс живёт долго (нода), а вес токена не
#: меняется, пока архив не переимпортирован. Кэш живёт на самом
#: инстансе :class:`RtttlLibrary` (см. :meth:`RtttlLibrary.token_weights`),
#: поэтому разные инстансы (например, в тестах — маленькие синтетические
#: архивы с разными БД) не путают чужие веса.
class _TokenWeights:
    """IDF-подобный вес токена по корпусу архива — БЕЗ списка стоп-слов.

    issue #2964 (правка товарища Шифу): раньше рассматривался хардкод-
    список общих слов («theme», «main», «soundtrack», «song» — сотни
    записей архива буквально называются «Theme»). Вместо списка — частота
    самого токена В ЭТОМ ЖЕ корпусе: токен, который встречается в сотнях
    записей, получает вес около нуля САМ, без переписывания под «Theme» —
    то же правило одинаково понижает любое другое частое слово, которое
    сегодня в архиве не бросилось в глаза.
    """

    def __init__(self, library: "RtttlLibrary") -> None:
        self._library = library
        self._cache: Dict[str, float] = {}

    def __call__(self, token: str) -> float:
        cached = self._cache.get(token)
        if cached is not None:
            return cached
        total = self._library.total()
        df = self._library.document_frequency(token)
        weight = math.log((total + 1) / max(df, 1))
        self._cache[token] = weight
        return weight


def _is_informative_title(weights: "_TokenWeights", title: str) -> bool:
    """Хотя бы один токен title достаточно редок в корпусе (не «Theme»)?

    Порог — свойство корпуса (токен встречается меньше чем в ~1% архива),
    а не список запрещённых слов: любое слово, ставшее таким же частым в
    архиве, как «theme», получает тот же исход автоматически.
    """
    tokens = _tokens(title.lower())
    if not tokens:
        return False
    threshold = math.log(100.0)  # df/total <= ~1%
    return any(weights(t) >= threshold for t in tokens)


def display_title(library: "RtttlLibrary", record: Dict[str, Any]) -> str:
    """Название записи для показа модели — с исполнителем, если title общий.

    issue #2964: сотни записей архива честно называются ``title='Theme'``,
    настоящее название — в ``artist`` (``theme_178``: title «Theme»,
    artist «Terminatorv v2.0» — правильная тема Терминатора, НЕ ошибка
    записи). ``lookup_melody('terminator theme')`` не должен отвечать
    голым «Нашёл «Theme»» — юзер не поймёт, что это Терминатор. Критерий
    информативности title — тот же корпусный вес :class:`_TokenWeights`,
    без разбора «title == 'Theme'»: любой другой такой же частый title
    в архиве получит ту же добавку artist автоматически.
    """
    title = str(record.get("title") or "").strip()
    artist = str(record.get("artist") or "").strip()
    if not title:
        return artist or str(record.get("rtttl_name") or record.get("name") or "")
    try:
        weights = library.token_weights()
        informative = _is_informative_title(weights, title)
    except Exception:  # noqa: BLE001 — вызывающая сторона мокает библиотеку
        # в юнит-тестах (Mock() без token_weights()); без реального корпуса
        # честнее не выдумывать доп. текст, чем упасть на служебном вызове.
        informative = True
    if artist and not informative:
        return f"{title} — {artist}"
    return title


def match_info(library: "RtttlLibrary", record: Dict[str, Any], query: str) -> Dict[str, Any]:
    """Прозрачная (не вердиктная) сверка *record* с токенами *query*.

    issue #2964: ``get()``/``search()`` осознанно всегда возвращают ЛУЧШЕГО
    по тексту кандидата, даже при слабом совпадении (issue #2896/#2877 —
    единой эвристики отказа не нашлось: «лишние слова» отличают и «другую
    песню» от нужной, и просто длинное название той же песни). Раньше
    единственной защитой было message «сверь title сама» — LLM (minimax)
    это игнорировал и играл что дали (``lookup_melody('Gin and Juice')`` →
    «Everybody's Changing», ``lookup_melody('Nuthin But A G Thang')`` →
    «If I Can Poppin Them Thangs», ``compose_music(name='stranger
    things')`` реально сыграл «Strangers In The Night»).

    Эта функция НЕ выносит вердикт found/not-found (жёсткого порога нет —
    история отката #2882→#2896: «stranger things» ломало «super mario»).
    Она отдаёт ЧЕСТНУЮ структурированную сверку — решение, это та же песня
    или нет, остаётся у модели (промпт скилла composer):

    - ``matched``/``unmatched`` — какие значимые токены запроса нашлись в
      опознавательных полях записи (:func:`covers_tokens`, потокенно), а
      какие нет.
    - ``coverage`` — доля ВЗВЕШЕННОГО (IDF, :class:`_TokenWeights`) веса
      запроса, которая покрылась. Частый токен вроде «theme» весит около
      нуля и почти не двигает coverage сам по себе — вес считается по
      корпусу архива, а не по списку слов.
    """
    tokens = _tokens(_normalize(query))
    if not tokens:
        return {"matched": [], "unmatched": [], "coverage": 0.0}
    try:
        weights = library.token_weights()
    except Exception:  # noqa: BLE001 — вызывающая сторона мокает библиотеку
        # в юнит-тестах (Mock() без token_weights()) — без реального корпуса
        # честнее считать каждый токен равнозначным, чем упасть.
        weights = None
    haystack = _identity_haystack(record)
    matched: List[str] = []
    unmatched: List[str] = []
    matched_weight = 0.0
    total_weight = 0.0
    for token in tokens:
        try:
            w = max(float(weights(token)), 0.0) if weights is not None else 1.0
        except Exception:  # noqa: BLE001 — тот же мокнутый вызов
            w = 1.0
        total_weight += w
        if token in haystack:
            matched.append(token)
            matched_weight += w
        else:
            unmatched.append(token)
    coverage = (matched_weight / total_weight) if total_weight > 0 else 0.0
    return {"matched": matched, "unmatched": unmatched, "coverage": round(coverage, 3)}


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
        # issue #2964 — IDF-вес токена по корпусу архива, см. _TokenWeights.
        self._token_weights = _TokenWeights(self)

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

    def document_frequency(self, token: str) -> int:
        """В скольких записях архива *token* встречается подстрокой.

        Основа IDF-веса (issue #2964, :class:`_TokenWeights`) — без
        отдельного индекса: колонки уже покрыты индексами по
        name/title/artist (см. ``_SCHEMA``), а сам счёт по 10К строк на
        ``LIKE`` укладывается в единицы миллисекунд.
        """
        like = f"%{token}%"
        with self._lock:
            return self._conn.execute(
                "SELECT COUNT(*) FROM rtttl_melodies WHERE "
                "lower(name) LIKE ? OR lower(title) LIKE ? "
                "OR lower(artist) LIKE ? OR lower(rtttl_name) LIKE ?",
                (like, like, like, like),
            ).fetchone()[0]

    def token_weights(self) -> "_TokenWeights":
        """Взвешиватель токенов (issue #2964) — с кэшем на этом инстансе."""
        return self._token_weights

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

    @staticmethod
    def _is_garbage(row: sqlite3.Row) -> bool:
        return (row["name"] or "").strip().lower() in _GARBAGE_NAMES

    @classmethod
    def _best_in_bucket(
        cls, rows: List[sqlite3.Row], tokens: List[str]
    ) -> Optional[sqlite3.Row]:
        """Лучшая строка: сперва по ТЕКСТОВОМУ смыслу, потом по качеству.

        🔴 FIX (live 23.09, issue #2840 регрессия): первая версия смешивала
        текстовый скор и качество мелодии в одну непрерывную формулу
        (``score * WEIGHT + quality``). Небольшой перевес качества у
        случайной «Irish National Anthem» (raw=3, только токен «anthem»)
        перебивал точный текстовый скор «National Anthem Of Russia»
        (тоже raw=3) чисто по количеству нот — семантика («russia» в
        запросе) при этом ни при чём не участвовала.

        Правильный порядок: сначала находим МАКСИМАЛЬНЫЙ текстовый скор
        среди кандидатов (bucket) — это решает совпадение по смыслу.
        Качество и денилист работают только ВНУТРИ этого bucket, где
        текстовое совпадение уже одинаково хорошее: денилист выкидывает
        мусор, если в bucket'е есть не-мусорная запись, а качество
        выбирает лучшую среди оставшихся.
        """
        scored = [(cls._score(row, tokens), row) for row in rows]
        scored = [(s, row) for s, row in scored if s > 0]
        if not scored:
            return None
        max_score = max(s for s, _row in scored)
        bucket = [row for s, row in scored if s == max_score]
        non_garbage = [row for row in bucket if not cls._is_garbage(row)]
        pool = non_garbage or bucket
        return max(pool, key=lambda row: _melody_quality(_row_rtttl(row)))

    def _candidates(self, tokens: List[str], cap: int) -> List[sqlite3.Row]:
        """Строки-кандидаты для скоринга: полные совпадения + capped-пул частичных.

        🔴 FIX (issue #2941): раньше был один SQL-запрос — «хотя бы один
        токен встречается в поле» (``OR`` по токенам) с ``LIMIT cap``
        ПРИМЕНЁННЫМ В SQL, ДО скоринга в Python. На частых словах («dre»,
        «anthem», «hot» — по полсотне-сотне совпадений каждое) запись,
        реально совпадающая по ВСЕМ токенам запроса (``stilldre_2`` для
        «still dre», ``national_2`` для «russia anthem»), могла просто не
        попасть в первые ``cap`` строк, которые SQLite отдаёт без
        ``ORDER BY`` (порядок физического хранения/id) — скоринг такую
        запись никогда не видел, хотя :meth:`get` (``cap=2000``, почти
        весь архив) её находил.

        Фикс: запись, где встретились ВСЕ значимые токены запроса (AND по
        токенам), гарантированно входит в выборку — без лимита, потому что
        полное совпадение уже само по себе сильный фильтр (пересечение, не
        объединение). Частичные совпадения (``OR`` хотя бы по одному
        токену) остаются как раньше, но только как дополняющий пул —
        capped, для случаев, когда полного совпадения нет вовсе, и для
        участия в ранжировании тай-брейков. Лимит на итоговое число
        результатов применяется в :meth:`search` уже ПОСЛЕ скоринга — эта
        функция отдаёт весь пул кандидатов, а не финальный топ-N.
        """
        if not tokens:
            return []

        def _clause_params() -> tuple:
            clauses: List[str] = []
            params: List[str] = []
            for token in tokens:
                like = f"%{token}%"
                clauses.append(
                    "(lower(name) LIKE ? OR lower(title) LIKE ? "
                    "OR lower(artist) LIKE ? OR lower(tags) LIKE ? "
                    "OR lower(rtttl_name) LIKE ?)"
                )
                params += [like, like, like, like, like]
            return clauses, params

        clauses, params = _clause_params()
        select = (
            "SELECT id, name, title, artist, source, tags, rtttl_name, rtttl "
            "FROM rtttl_melodies WHERE "
        )

        full_rows: List[sqlite3.Row] = []
        if len(tokens) > 1:
            # AND по всем токенам — без LIMIT: гарантированное включение
            # полных совпадений в выборку для скоринга.
            and_sql = select + " AND ".join(clauses)
            full_rows = self._conn.execute(and_sql, params).fetchall()

        or_sql = select + " OR ".join(clauses) + " LIMIT ?"
        or_rows = self._conn.execute(or_sql, params + [cap]).fetchall()

        if not full_rows:
            return or_rows
        seen = {row["id"] for row in full_rows}
        merged = list(full_rows) + [row for row in or_rows if row["id"] not in seen]
        return merged

    def get(self, name: str) -> Optional[Dict[str, Any]]:
        """Найти одну мелодию (точное имя → лучший по токенам запроса).

        Точное совпадение имени обычно возвращается сразу (прямая
        адресация: ``get('russiann')`` честно находит ``russiann``). Но
        если точное имя само в денилисте (``_GARBAGE_NAMES``), оно не
        побеждает автоматически — участвует в ранжировании наравне с
        токен-кандидатами и уступает более качественной записи, если такая
        нашлась (иначе остаётся честным fallback'ом, когда лучшего нет).

        🔴 issue #2896 (регрессия #2882→#2877): раньше здесь был ещё один
        фильтр — ``_is_weak_match``, честно отклонявший кандидата, если у
        title было «слишком много лишних слов» относительно токенов
        запроса. На практике это правило било по СИЛЬНЫМ совпадениям:
        ``get('super mario')`` не находил «Super Mario Brothers 1»
        (лишних слов в title больше, чем токенов в запросе), ``get('star
        wars')`` не находил «Star Wars - Imperial March 1» ровно по той
        же причине. Единой эвристики, которая отличает «лишние слова —
        другая песня» (issue #2877, «stranger things» → «Strangers In
        The Night») от «лишние слова — просто длинное название» (issue
        #2896, «super mario» → «Super Mario Brothers 1»), не нашлось —
        каждый фикс под один случай ломал другой.

        Поэтому ``get()`` больше не решает это молчаливым отказом: он
        всегда возвращает лучшего по тексту кандидата (как до #2882), а
        ответственность «эта ли песня имелась в виду» переходит на
        вызывающую сторону — ``ComposeMusicTool``/``LookupMelodyTool``
        отдают модели ``title`` найденной записи и короткий список
        альтернатив (:meth:`search`), а промпт скилла composer учит
        модель сверять их с тем, что просил юзер, вместо того чтобы
        молча доверять первому результату.
        """
        q = _normalize(name)
        tokens = _tokens(q)
        if not tokens:
            return None
        with self._lock:
            exact = self._conn.execute(
                "SELECT * FROM rtttl_melodies WHERE lower(name) = ? LIMIT 1", (q,)
            ).fetchone()
            if exact is not None and not self._is_garbage(exact):
                return self._to_dict(exact, include_rtttl=True)
            rows = self._candidates(tokens, cap=2000)
        pool = list(rows)
        if exact is not None and not any(row["id"] == exact["id"] for row in pool):
            pool.append(exact)
        best = self._best_in_bucket(pool, tokens)
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

        Сортировка: текстовый скор (главный ключ — решает семантику
        совпадения) → денилист (внутри одного скора мусор тонет под
        не-мусором) → качество мелодии → title (алфавит, последний
        тай-брейк, для истинных ничьих по скору, денилисту и качеству).
        Качество и денилист НЕ могут перевесить разницу в текстовом скоре
        — см. :meth:`_best_in_bucket`.

        🔴 FIX (issue #2941): тай-брейк «качество → title» — тот же
        порядок, что использует :meth:`_best_in_bucket` (через который
        работает :meth:`get`). Раньше здесь было «title → качество»: для
        одинакового скора несколько записей архива с разными title
        (``«Super Mario Brothers 1»`` vs ``«Supermario Brothers»`` —
        пробел сортируется раньше буквы) сортировались алфавитно ДО учёта
        качества, и ``search('super mario')`` ставил первым не тот трек,
        что находил ``get('super mario')`` — ``get``/``search`` расходились
        при равном скоре, но разных title. ``LIMIT`` на итоговый размер
        результата применяется ЗДЕСЬ, уже после скоринга и сортировки —
        не в SQL (см. :meth:`_candidates`).
        """
        q = _normalize(query)
        tokens = _tokens(q)
        if not tokens:
            return []
        limit = max(1, min(50, int(limit)))
        # cap для OR-пула частичных совпадений — тот же порог, что у
        # get() (см. её вызов _candidates(..., cap=2000)): меньший cap,
        # завязанный на limit (``limit * 10``), давал search()/get()
        # расходиться на многословных запросах, где не все токены
        # находят буквальную подстроку ни в одной записи («bros» не
        # substring «Brothers») — тогда решает не AND-гарантия (см.
        # _candidates), а сам OR-пул, и его размер обязан совпадать с
        # тем, что видит get() (issue #2941, «search/get согласованы»).
        with self._lock:
            rows = self._candidates(tokens, cap=max(limit * 10, 2000))
        scored = []
        for row in rows:
            match = self._score(row, tokens)
            if match <= 0:
                continue
            quality = _melody_quality(_row_rtttl(row))
            scored.append((match, self._is_garbage(row), row, quality))

        def _sort_key(item: tuple) -> tuple:
            match, garbage, row, quality = item
            return (-match, garbage, -quality, (row["title"] or "").lower())

        scored.sort(key=_sort_key)
        return [self._to_dict(row) for _m, _g, row, _q in scored[:limit]]
