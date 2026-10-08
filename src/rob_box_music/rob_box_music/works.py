"""Реестр произведений: одна запись ``Work`` на пьесу, у каждого поля — источник (ADR-0155 K-2).

Что здесь живёт: модель (``Work``/``Fact``/``WorkSource``), нормализация названий, чистка идентичности записей
RTTTL-библиотеки (категория вместо исполнителя → тип произведения; «Theme» + название в ``artist`` → ``title``),
связи «фраза темы → строки поиска архива» (``ThemeLink``: семена из ``data/theme_link_seeds.json``, проверенные
ответы LLM и ручные связи — таблица ``theme_links``; из семян поиск ``rtttl_library`` берёт :func:`alias_pairs` и
:func:`ru_phrase_by_query`, разбор слов темы — :func:`word_links`), группы версий (канон — порядок ``canon``),
гейт «обогащать поле или нет» (:func:`gate`, §3.3 ADR) и отчёт; связи произведений с партитурами пака
(:func:`match_scores`, K-3: источники ``pdmx:<id>`` — только предложения, В6; построение и отчёт —
``scripts/music/works_pdmx.py``). Сеть здесь не вызывается: ни один сетевой источник гейт сегодня не проходит.

Реестр — таблицы ``works``/``work_sources``/``work_facts`` в той же SQLite, что RTTTL-библиотека (ADR-0155 В3):
``python -m rob_box_music.works --db voice_memory.db`` перестраивает их (идемпотентно), ``--holes genre`` печатает
``work_id`` рабочего множества без значения поля, ``--report`` — покрытие полей и вердикты гейта.
"""

from __future__ import annotations

import argparse
import difflib
import functools
import gzip
import hashlib
import json
import re
import sqlite3
import sys
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .knowledge import (CATEGORY_ARTISTS, DEFAULT_HOOKS, EMPTY_TITLES, LICENSE_STOP_LIST, ROOTS,
                        TAG_WORK_TYPE, THEMES)
from .material import ScoreMaterial, license_usable
from .rtttl import consensus_order

FACT_SOURCES = ("rtttl", "pdmx", "derived", "manual", "wikidata", "mb")
LINK_LEVELS = ("exact_artist", "exact", "fuzzy", "manual")
FIELDS = ("work_type", "genre", "year")
#: Гейт §3.3 п.2 (ADR-0155 В5): доля рабочего множества без значения поля, с которой обогащение вообще рассматривается.
GATE_SHARE = 0.20
#: TTL записи журнала запросов §3.4 (сеть в K-2 не вызывается; константа — для K-3/K-4).
LOOKUP_TTL_DAYS = 180
#: Версия правил чистки идентичности: меняется — факты перестраиваются.
RULES_VERSION = "k2-1"
#: Рабочее множество §3.3: теги каталога RTTTL (плюс строки ``THEMES`` и ``DEFAULT_HOOKS``).
WORKING_TAGS = ("tv", "movie", "game", "classical", "anthem", "christmas", "folk")
#: Кто читает поле ``Work`` на develop (§3.3 п.1; файл:функция). Пусто — потребителя нет, поле не обогащается.
FIELD_CONSUMERS: Mapping[str, Tuple[str, ...]] = {
    "work_type": (), "genre": (), "year": (),
    "aliases": ("core/rtttl_library.py:_alias_normalize", "core/rtttl_library.py:ru_alias_for"),
}

_NORM_RE = re.compile(r"[0-9a-zа-я]")
_VERSION_TAIL_RE = re.compile(r"\s+v\d+(?:\.\d+)*$", re.I)
_WORD_RE = re.compile(r"[0-9a-zа-я]+")
_CYRILLIC_RE = re.compile(r"[а-яё]")


def norm(text: Any) -> str:
    """Название для сравнения: нижний регистр, ё → е, только буквы и цифры без пробелов."""
    return "".join(_NORM_RE.findall(str(text or "").lower().replace("ё", "е")))


def _words(text: Any) -> frozenset:
    return frozenset(_WORD_RE.findall(str(text or "").lower().replace("ё", "е")))


@dataclass(frozen=True)
class Fact:
    """Значение поля произведения и его происхождение; поле без источника — не ``Fact``, а ``None``."""

    value: str
    source: str
    source_id: str = ""
    fetched_at: str = ""
    rules_version: str = RULES_VERSION
    verified: bool = False

    def __post_init__(self) -> None:
        if not self.value.strip():
            raise ValueError("Fact.value пуст: поле без значения — None, не пустая строка")
        if self.source not in FACT_SOURCES:
            raise ValueError(f"Fact.source {self.source!r} не из {FACT_SOURCES}")


@dataclass(frozen=True)
class WorkSource:
    """Откуда берётся материал произведения: запись RTTTL, партитура PDMX (K-3) или своя партитура."""

    kind: str
    material_id: str
    link_level: str = "exact"
    link_score: float = 1.0
    confirmed: bool = False

    def __post_init__(self) -> None:
        if self.kind not in ("rtttl", "pdmx", "local"):
            raise ValueError(f"WorkSource.kind {self.kind!r}: rtttl | pdmx | local")
        if not self.material_id.startswith(self.kind + ":"):
            raise ValueError(f"material_id {self.material_id!r} должен начинаться с {self.kind}:")
        if self.link_level not in LINK_LEVELS:
            raise ValueError(f"link_level {self.link_level!r} не из {LINK_LEVELS}")
        if not 0.0 <= self.link_score <= 1.0:
            raise ValueError("link_score вне [0, 1]")
        # ADR-0155 В6: сопоставление по названию — только предложение; подтверждает человек (manual).
        if self.confirmed and self.kind != "rtttl" and self.link_level != "manual":
            raise ValueError(f"связь {self.link_level} с {self.kind} подтверждается только вручную (link_level=manual)")


@dataclass(frozen=True)
class Work:
    work_id: str
    title: str
    sources: Tuple[WorkSource, ...]
    artist: str = ""
    composer: str = ""
    aliases: Tuple[str, ...] = ()
    genre: Optional[Fact] = None
    year: Optional[Fact] = None
    work_type: Optional[Fact] = None
    #: Имя из стоп-списка живых правообладателей (:data:`knowledge.LICENSE_STOP_LIST`, ADR-0155 В1).
    stop_listed: bool = False

    def __post_init__(self) -> None:
        if not re.fullmatch(r"[0-9a-f]{8}", self.work_id):
            raise ValueError(f"work_id {self.work_id!r}: sha8")
        if not self.title.strip():
            raise ValueError("Work.title пуст")
        if not self.sources:
            raise ValueError("у Work должен быть хотя бы один источник")

    def fact(self, field: str) -> Optional[Fact]:
        if field not in FIELDS:
            raise ValueError(f"поле {field!r} не из {FIELDS}")
        return getattr(self, field)


@dataclass(frozen=True)
class Identity:
    """Название, исполнитель и тип записи после чистки (``clean_identity``)."""

    title: str
    artist: str
    work_type: str
    named: bool = True  # False — у записи нет названия, ``title`` взят из её имени (slug)


# ── Семена связей: «фраза → строки поиска», данные, а не код (#3493) ────────────────────────────────────────────
#: Файл семян (``ThemeLink`` с ``source="seed"``): перенос прежних ручных таблиц RU_ALIASES и THEME_CONCEPTS.
SEEDS_FILE = Path(__file__).resolve().parent / "data" / "theme_link_seeds.json"
#: Ключ семени с этим знаком на конце — начало слова («хогвар*» — «Хогвардсу», «Хогвартсе»), без — целые слова.
PREFIX = "*"


@functools.lru_cache(maxsize=None)
def theme_seeds() -> Tuple["ThemeLink", ...]:
    """Семена в порядке файла. Ключ-начало — одно слово (любая из строк подходит), ключ из целых слов — одна строка
    (фраза запроса заменяется ею); иначе ``ValueError`` — файл чинят, а не обходят в коде."""
    out = []
    for item in json.loads(SEEDS_FILE.read_text(encoding="utf-8"))["seeds"]:
        phrase = str(item["phrase"]).lower()  # ё как в файле: замена в запросе идёт по точному написанию
        queries = tuple(str(q) for q in item["queries"])
        prefix = phrase.endswith(PREFIX)
        if prefix and " " in phrase or not prefix and len(queries) != 1:
            raise ValueError(f"семя {phrase!r}: начало — одно слово, целые слова — одна строка поиска")
        out.append(ThemeLink(phrase, "found", "seed", "concept" if prefix else "work", queries))
    return tuple(out)


def word_links(word: str) -> Tuple[str, ...]:
    """Строки поиска первого семени-начала, с которого начинается слово темы: «интерстеллара» → ``("space",)``."""
    return next((s.queries for s in theme_seeds() if s.phrase.endswith(PREFIX) and word.startswith(s.phrase[:-1])), ())


def concept_queries(word: str) -> Tuple[str, ...]:
    """Строки поиска всех семян-начал слова (строка таблицы тем по понятию, ``theme.match_row``)."""
    return tuple(q for s in theme_seeds() if s.phrase.endswith(PREFIX) and word.startswith(s.phrase[:-1])
                 for q in s.queries)


def _phrase_seeds() -> List[Tuple[str, str]]:
    return [(s.phrase, s.queries[0]) for s in theme_seeds() if not s.phrase.endswith(PREFIX)]


def alias_pairs() -> List[Tuple[str, str]]:
    """``(фраза, строка поиска)`` семян из целых слов, длинные фразы первыми: «гимн ссср» раньше «гимн»."""
    return sorted(_phrase_seeds(), key=lambda kv: -len(kv[0]))


def ru_phrase_by_query() -> Dict[str, str]:
    """Строка поиска → ПЕРВАЯ (по порядку семян) русская фраза на неё (озвучка названия, #3178)."""
    out: Dict[str, str] = {}
    for phrase, query in _phrase_seeds():
        if _CYRILLIC_RE.search(phrase):
            out.setdefault(query, phrase)
    return out


def aliases_of(title: str, artist: str) -> Tuple[str, ...]:
    """Русские фразы, чья строка поиска целиком состоит из слов названия/исполнителя (как слова поиска)."""
    have = _words(title) | _words(artist)
    return tuple(p for p, q in _phrase_seeds() if _CYRILLIC_RE.search(p) and _words(q) <= have)


# ── Чистка идентичности записи RTTTL ─────────────────────────────────────────────────────────────────────────

def _tag_type(tags: Iterable[str]) -> str:
    return next((TAG_WORK_TYPE[t] for t in tags if t in TAG_WORK_TYPE), "")


def clean_identity(rec: Mapping[str, Any]) -> Identity:
    """Категория вместо исполнителя («Films And Tv», «Computer Games») → ``work_type``, артиста нет; название
    «Theme» с настоящим названием в ``artist`` («20th Century Fox») → ``title``; «Batman Theme» в ``artist`` — это
    название, не человек. Тип без категории — из тегов каталога."""
    tags = [str(t) for t in rec.get("tags") or []]
    title = _VERSION_TAIL_RE.sub("", str(rec.get("title") or "").strip())
    artist = str(rec.get("artist") or "").strip()
    category = artist.lower() in CATEGORY_ARTISTS
    wtype = CATEGORY_ARTISTS.get(artist.lower(), "")
    named_theme = artist.lower().endswith(" theme") and not category
    if norm(title) in {norm(t) for t in EMPTY_TITLES}:
        title = artist if artist and not category else ""
        artist = ""
    elif category or named_theme:
        artist = ""
    return Identity(title or str(rec.get("name") or ""), artist, wtype or _tag_type(tags), bool(title))


def work_key(rec: Mapping[str, Any]) -> str:
    """Ключ группы версий: нормализованные название и исполнитель; запись без названия — сама по себе."""
    ident = clean_identity(rec)
    if not ident.named:
        return "name|" + str(rec.get("name") or "")
    return norm(ident.title) + "|" + norm(ident.artist)


def work_id_of(key: str) -> str:
    return hashlib.sha1(key.encode("utf-8")).hexdigest()[:8]


def is_stop_listed(*names: str) -> bool:
    flat = norm(" ".join(names))
    return any(s in flat for s in LICENSE_STOP_LIST)


# ── Сборка реестра ───────────────────────────────────────────────────────────────────────────────────────────

def _source_of(rec: Mapping[str, Any]) -> WorkSource:
    return WorkSource("rtttl", "rtttl:" + str(rec.get("name")), "exact", 1.0, True)


def _build_one(key: str, group: List[Mapping[str, Any]]) -> Work:
    ident = clean_identity(group[0])
    name = str(group[0].get("name"))
    wtype = Fact(ident.work_type, "rtttl", name) if ident.work_type else None
    tags = [str(t) for t in group[0].get("tags") or []]
    genre = next((Fact(t, "rtttl", name) for t in tags if t in WORKING_TAGS), None)
    return Work(work_id_of(key), ident.title, tuple(_source_of(r) for r in group), ident.artist, "",
                aliases_of(ident.title, ident.artist), genre, None, wtype, is_stop_listed(ident.title, ident.artist))


def build_works(records: Iterable[Mapping[str, Any]]) -> List[Work]:
    """Записи RTTTL → ``Work``. Версии одной пьесы (одинаковые чистые название+исполнитель) — один ``Work``, источники
    в порядке :func:`rtttl.consensus_order` (одна реализация с поиском темы: версии с общим контуром начала выше
    одиночных; канон — первый источник). Порядок ``Work`` — по первой записи группы."""
    groups: Dict[str, List[Mapping[str, Any]]] = {}
    for rec in records:
        groups.setdefault(work_key(rec), []).append(rec)
    out = []
    for key, group in groups.items():
        if len(group) > 1:
            order = consensus_order([(1.0, r) for r in group], limit=len(group))
            group = sorted(group, key=lambda r: order.index(r["name"]) if r["name"] in order else len(order))
        out.append(_build_one(key, group))
    return out


def working_ids(records: Iterable[Mapping[str, Any]], names: Iterable[str] = ()) -> frozenset:
    """``work_id`` рабочего множества §3.3: теги :data:`WORKING_TAGS`, хуки ``THEMES`` и ``DEFAULT_HOOKS``.
    Журнала запросов ``music_history`` здесь нет — в K-2 он не учитывается."""
    hooks = set(names) | set(DEFAULT_HOOKS) | {h for row in THEMES.values() for h in row.hooks}
    return frozenset(work_id_of(work_key(r)) for r in records
                     if str(r.get("name")) in hooks or set(WORKING_TAGS) & set(r.get("tags") or []))


# ── Гейт и отчёт ─────────────────────────────────────────────────────────────────────────────────────────────

def holes(works: Iterable[Work], field: str, working: Optional[frozenset] = None) -> List[str]:
    """``work_id`` без значения поля ``field`` (в рабочем множестве, если задано)."""
    return [w.work_id for w in works if w.fact(field) is None and (working is None or w.work_id in working)]


@dataclass(frozen=True)
class Gate:
    field: str
    working: int
    holes: int
    share: float
    consumers: Tuple[str, ...]
    verdict: str


def gate(works: Sequence[Work], field: str, working: frozenset) -> Gate:
    """§3.3: потребитель есть, дыра ≥ :data:`GATE_SHARE`, точность источника доказана пилотом (код этого не знает:
    даже открытый гейт отвечает ``candidate`` — пилот ≤ 30 записей на проверку Шифу)."""
    pool = [w for w in works if w.work_id in working]
    miss = len(holes(works, field, working))
    share = miss / len(pool) if pool else 0.0
    consumers = FIELD_CONSUMERS.get(field, ())
    if not consumers:
        verdict = "closed: нет потребителя"
    elif share < GATE_SHARE:
        verdict = f"closed: дыра {share:.0%} < {GATE_SHARE:.0%}"
    else:
        verdict = "candidate: нужен пилот ≤ 30 записей (п.3)"
    return Gate(field, len(pool), miss, round(share, 4), consumers, verdict)


def report(records: Sequence[Mapping[str, Any]], works: Sequence[Work]) -> str:
    working = working_ids(records)
    multi = [w for w in works if len(w.sources) > 1]
    cleaned = sum(1 for r in records
                  if (clean_identity(r).title, clean_identity(r).artist) != (r.get("title") or "", r.get("artist") or ""))
    lines = [f"записей RTTTL {len(records)} → произведений {len(works)} (версий >1 у {len(multi)}); "
             f"рабочее множество {len(working)}; идентичность изменена у {cleaned}; "
             f"с RU-алиасом {sum(1 for w in works if w.aliases)}; в стоп-списке {sum(w.stop_listed for w in works)}",
             "поле      заполнено   рабочее: дыр / из    дыра   вердикт гейта"]
    for field in FIELDS:
        g = gate(works, field, working)
        have = sum(w.fact(field) is not None for w in works)
        lines.append(f"{field:<9} {have:>9}   {g.holes:>14} / {g.working:<5} {g.share:>5.0%}  {g.verdict}")
    lines.append("запросов в сеть: 0 (журнал lookups — K-3; сетевой провайдер — K-4, только после гейта)")
    return "\n".join(lines)


# ── SQLite: реестр в той же БД, что RTTTL-библиотека ─────────────────────────────────────────────────────────

_REGISTRY_SCHEMA = """
CREATE TABLE IF NOT EXISTS works (work_id TEXT PRIMARY KEY, title TEXT NOT NULL, artist TEXT NOT NULL DEFAULT '',
    composer TEXT NOT NULL DEFAULT '', aliases TEXT NOT NULL DEFAULT '[]', stop_listed INTEGER NOT NULL DEFAULT 0);
CREATE TABLE IF NOT EXISTS work_sources (work_id TEXT NOT NULL, material_id TEXT NOT NULL, kind TEXT NOT NULL,
    link_level TEXT NOT NULL, link_score REAL NOT NULL, confirmed INTEGER NOT NULL, rank INTEGER NOT NULL,
    license TEXT, rating REAL, keysig TEXT, key TEXT, composer TEXT, PRIMARY KEY (work_id, material_id));
CREATE TABLE IF NOT EXISTS work_facts (work_id TEXT NOT NULL, field TEXT NOT NULL, value TEXT NOT NULL,
    source TEXT NOT NULL, source_id TEXT NOT NULL, fetched_at TEXT NOT NULL, rules_version TEXT NOT NULL,
    verified INTEGER NOT NULL, PRIMARY KEY (work_id, field));
"""


def write_registry(conn: sqlite3.Connection, works: Iterable[Work]) -> int:
    """Перестроить реестр одной транзакцией (идемпотентно; ручные факты K-3+ сюда не пишутся — таблицы пересоздаются
    из источников). Возвращает число произведений."""
    works = list(works)
    with conn:
        conn.executescript(_REGISTRY_SCHEMA)
        _ensure_source_columns(conn)
        for table in ("works", "work_sources", "work_facts"):
            conn.execute(f"DELETE FROM {table}")
        for w in works:
            conn.execute("INSERT INTO works VALUES (?,?,?,?,?,?)", (w.work_id, w.title, w.artist, w.composer,
                         json.dumps(list(w.aliases), ensure_ascii=False), int(w.stop_listed)))
            conn.executemany("INSERT INTO work_sources (work_id, material_id, kind, link_level, link_score, confirmed, "
                             "rank) VALUES (?,?,?,?,?,?,?)",
                             [(w.work_id, s.material_id, s.kind, s.link_level, s.link_score, int(s.confirmed), i)
                              for i, s in enumerate(w.sources)])
            conn.executemany("INSERT INTO work_facts VALUES (?,?,?,?,?,?,?,?)",
                             [(w.work_id, f, x.value, x.source, x.source_id, x.fetched_at, x.rules_version,
                               int(x.verified)) for f in FIELDS for x in [w.fact(f)] if x])
    link_score_sources(conn)  # перестройка стирает work_sources целиком — партитуры из score_index подвязываем заново
    return len(works)


# ── Индекс партитур (ADR-0154 §3.2): ``score_index`` в той же SQLite, партитура — источник ``pdmx:<id>`` произведения ──

_SCORE_INDEX_SCHEMA = """
CREATE TABLE IF NOT EXISTS score_index (material_id TEXT PRIMARY KEY, title TEXT NOT NULL, composer TEXT NOT NULL,
    genres TEXT NOT NULL, rating REAL, n_ratings INTEGER, n_views INTEGER, complexity INTEGER, meter TEXT NOT NULL,
    bpm INTEGER, key TEXT NOT NULL, keysig TEXT, bars INTEGER NOT NULL, license TEXT NOT NULL,
    phrase_count INTEGER NOT NULL, hook_phrase_bar INTEGER, file TEXT NOT NULL);
"""
_SCORE_COLUMNS = ("material_id", "title", "composer", "genres", "rating", "n_ratings", "n_views", "complexity",
                  "meter", "bpm", "key", "keysig", "bars", "license", "phrase_count", "hook_phrase_bar", "file")


@dataclass(frozen=True)
class ScoreIndexRow:
    """Строка ``score_index``: то, чем ищут и ранжируют партитуру, не открывая её JSON (``file`` — имя в каталоге)."""

    material_id: str
    title: str
    composer: str
    meter: str
    key: str
    bars: int
    license: str
    phrase_count: int
    file: str
    genres: str = ""
    rating: Optional[float] = None
    n_ratings: Optional[int] = None
    n_views: Optional[int] = None
    complexity: Optional[int] = None
    bpm: Optional[int] = None
    keysig: Optional[str] = None  # ключевые знаки партитуры ("2 sharps"), не вывод анализатора; ``key`` — вывод
    hook_phrase_bar: Optional[int] = None  # такт самой повторяемой фразы — кандидат в хук (Н9)


def score_index_row(m: ScoreMaterial, *, bars: int, file: str, genres: str = "", n_ratings: Optional[int] = None,
                    keysig: Optional[str] = None) -> ScoreIndexRow:
    """Строка индекса из материала; хук-фраза — самая повторяемая, при равенстве — самая ранняя."""
    best = max(m.phrases, key=lambda p: (p.repeats, -p.bar), default=None)
    return ScoreIndexRow(m.material_id, m.title, m.composer, f"{m.meter[0]}/{m.meter[1]}",
                         f"{ROOTS[m.key.root]} {m.key.mode}", bars, m.license, len(m.phrases), file, genres,
                         m.stats.rating, n_ratings, m.stats.n_views, m.stats.complexity, m.bpm, keysig,
                         best.bar if best else None)


def write_score_index(conn: sqlite3.Connection, rows: Iterable[ScoreIndexRow]) -> Tuple[int, List[Tuple[str, str]]]:
    """Записать строки в ``score_index`` (upsert по ``material_id``). Строка без пригодной лицензии
    (:func:`material.license_usable`: пусто, ``unknown``, ``…conflict``) в индекс не попадает (ADR-0154 M6) —
    возвращается ``(записано, [(material_id, причина), ...])``."""
    rejected: List[Tuple[str, str]] = []
    good = []
    for r in rows:
        if license_usable(r.license):
            good.append(r)
        else:
            rejected.append((r.material_id, f"лицензия {r.license!r} не годится для индекса"))
    with conn:
        conn.executescript(_SCORE_INDEX_SCHEMA)
        conn.executemany(f"INSERT OR REPLACE INTO score_index ({','.join(_SCORE_COLUMNS)}) "
                         f"VALUES ({','.join('?' * len(_SCORE_COLUMNS))})",
                         [tuple(getattr(r, c) for c in _SCORE_COLUMNS) for r in good])
    link_score_sources(conn)
    return len(good), rejected


def link_score_sources(conn: sqlite3.Connection, scores: str = "main") -> int:
    """Подвязать партитуры ``score_index`` (схема ``scores``: ``main`` или присоединённый ATTACH-ем индекс пака
    ``/opt/rob_box/scores/score_index.db``) к произведениям реестра как источники ``pdmx:<id>`` — **предложением**
    (ADR-0155 В6, правила — :func:`match_scores`); вместе со связью пишутся лицензия, рейтинг, знаки, лад и
    композитор партитуры. Возвращает число связей; без ``score_index`` или ``works`` — 0."""
    tables = {r[0] for r in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    score_tables = {r[0] for r in conn.execute(f"SELECT name FROM {scores}.sqlite_master WHERE type='table'")}
    if not {"works", "work_sources"} <= tables or "score_index" not in score_tables:
        return 0
    works_rows = conn.execute("SELECT work_id, title, artist, composer FROM works").fetchall()
    cols = ("material_id", "title", "composer", "license", "rating", "keysig", "key")
    rows = [dict(zip(cols, r)) for r in conn.execute(f"SELECT {','.join(cols)} FROM {scores}.score_index")]
    links, _rejected = match_scores(works_rows, rows)
    with conn:
        _ensure_source_columns(conn)
        conn.execute("DELETE FROM work_sources WHERE kind='pdmx'")
        ranks = dict(conn.execute("SELECT work_id, MAX(rank) FROM work_sources GROUP BY work_id"))
        out = []
        for link in links:
            ranks[link.work_id] = ranks.get(link.work_id, -1) + 1
            s = link.source
            out.append((link.work_id, s.material_id, s.kind, s.link_level, s.link_score, int(s.confirmed),
                        ranks[link.work_id], link.license, link.rating, link.keysig, link.key, link.composer))
        conn.executemany(f"INSERT OR REPLACE INTO work_sources ({','.join(_SOURCE_COLUMNS)}) "
                         f"VALUES ({','.join('?' * len(_SOURCE_COLUMNS))})", out)
    return len(out)


# ── Сопоставление «произведение ↔ партитура» (ADR-0155 K-3): правила замера §2.2, только предложения (В6) ────────
#: Названия, которые произведение не опознают (замер K-1: 28 «Unknown» RTTTL ↔ 28 «Unknown» PDMX).
GENERIC_TITLES = frozenset(norm(t) for t in (*EMPTY_TITLES, "untitled song", "test", "song", "melody", "intro",
                                             "main theme", "theme song", "title"))
#: Короче этого (нормализованных букв) название точным совпадением не связывается («Up», «Go»).
MIN_TITLE = 3
#: Слова названия, не опознающие пьесу: без них сравниваются наборы слов на нечётком уровне.
TITLE_STOP = frozenset({"the", "a", "an", "of", "in", "from", "and", "theme", "themes", "song", "music", "for", "to",
                        "by", "op", "no", "version", "ver", "remix", "main", "title", "tune", "intro", "ost",
                        "soundtrack", "piano", "solo", "easy", "arr", "arrangement"})
#: Слова «исполнителя», не называющие человека: совпадение по ним — не ``exact_artist``.
PEOPLE_STOP = frozenset({"the", "and", "misc", "traditional", "trad", "anon", "anonymous", "composer", "unknown",
                         "arr", "arranged", "by", "music", "band", "theme", "tunes", "games", "computer"})
#: Нечётко: сходство difflib слов названия не ниже, вложение наборов слов — меньший ≥ 2 слов и длиннее не более
#: чем на ``CONTAIN_SLACK`` (замер K-1: «Not yet» ⊂ «I'm Not A Girl Not Yet A Woman»).
FUZZY_MIN = 0.88
CONTAIN_SLACK = 2
#: Слово, встречающееся в названиях чаще, — не кандидат-ключ нечёткого поиска (иначе «love» тянет тысячи строк).
RARE_DF = 3000
#: Нечётких предложений на произведение (лучшие по сходству и рейтингу); точные не режутся.
FUZZY_PER_WORK = 5
#: Вес связи по уровню; нечёткая — половина сходства (точность по замеру K-1: 0.75 / 0.35 / 0.17).
LINK_SCORES: Mapping[str, float] = {"exact_artist": 0.9, "exact": 0.6}
_SOURCE_COLUMNS = ("work_id", "material_id", "kind", "link_level", "link_score", "confirmed", "rank", "license",
                   "rating", "keysig", "key", "composer")


@dataclass(frozen=True)
class ScoreLink:
    """Предложенная связь произведения с партитурой и то, что о партитуре нужно отчёту и выбору (лицензия, рейтинг,
    знаки, лад, композитор); ``source`` проходит валидатор :class:`WorkSource` (В6: ``confirmed`` только ``manual``)."""

    work_id: str
    source: WorkSource
    license: str
    rating: Optional[float] = None
    keysig: Optional[str] = None
    key: Optional[str] = None
    composer: str = ""


def _ensure_source_columns(conn: sqlite3.Connection) -> None:
    """Колонки партитуры в ``work_sources`` реестра, созданного до K-3 (ALTER TABLE, данные не трогаются)."""
    have = {r[1] for r in conn.execute("PRAGMA table_info(work_sources)")}
    for col, kind in (("license", "TEXT"), ("rating", "REAL"), ("keysig", "TEXT"), ("key", "TEXT"),
                      ("composer", "TEXT")):
        if col not in have:
            conn.execute(f"ALTER TABLE work_sources ADD COLUMN {col} {kind}")


def _title_tokens(text: Any) -> Tuple[str, ...]:
    return tuple(t for t in _WORD_RE.findall(str(text or "").lower().replace("ё", "е")) if t not in TITLE_STOP)


def _people(text: Any) -> frozenset:
    return frozenset(t for t in _words(text) if len(t) >= 3 and t not in PEOPLE_STOP)


def _score_titles(title: str, people: frozenset) -> Dict[str, bool]:
    """Написания названия партитуры для точного совпадения → «взято до « - »»: целиком и до скобок — ``False``;
    до « - автор» — ``True`` (годится только с совпавшим автором, :func:`_exact_links`). Начало до « - » из одних
    имён автора партитуры («Mozart - Concerto K. 191», «J.S. Bach - Air») — это автор, не название."""
    head = re.split(r"[(\[]", title)[0]
    out = {norm(title): False, norm(head): False}
    for part in {title.split(" - ")[0], head.split(" - ")[0]}:
        names = _people(part)
        if " - " in title and norm(part) not in out and not (names and names <= people):
            out[norm(part)] = True
    return {v: dash for v, dash in out.items() if len(v) >= MIN_TITLE and v not in GENERIC_TITLES}


def _named_after_person(title: str, artist: str, composer: str) -> bool:
    """Запись RTTTL названа именем автора («Mozart» — Mozart): это не название пьесы, точной пары у неё нет."""
    words = _people(title)
    return bool(words) and words <= (_people(artist) | _people(composer))


class _Scores:
    """Партитуры с пригодной лицензией: индексы по написаниям названия и по редким словам (кандидаты нечёткого)."""

    def __init__(self, rows: Sequence[Mapping[str, Any]]) -> None:
        self.rows = sorted(rows, key=lambda r: (-float(r.get("rating") or 0.0), str(r["material_id"])))
        self.by_title: Dict[str, List[Tuple[int, bool]]] = {}
        self.by_token: Dict[str, List[int]] = {}
        self.tokens: List[Tuple[str, ...]] = []
        self.people: List[frozenset] = []
        for i, r in enumerate(self.rows):
            title = str(r.get("title") or "")
            self.people.append(_people(r.get("composer")) | _people(" ".join(title.split(" - ")[1:])))
            for v, dash in _score_titles(title, self.people[i]).items():
                self.by_title.setdefault(v, []).append((i, dash))
            toks = _title_tokens(title.split(" - ")[0])
            self.tokens.append(toks)
            for t in set(toks):
                self.by_token.setdefault(t, []).append(i)

    def candidates(self, toks: Sequence[str]) -> List[int]:
        rare = sorted((t for t in set(toks) if 0 < len(self.by_token.get(t, ())) <= RARE_DF),
                      key=lambda t: len(self.by_token[t]))[:2]
        return sorted({i for t in rare for i in self.by_token[t]})

    def fuzzy(self, toks: Tuple[str, ...], i: int) -> float:
        other = self.tokens[i]
        if len(set(other)) < 2:  # одно слово («Death», «Force») — не опознаёт пьесу без точного совпадения
            return 0.0
        small, big = sorted((set(toks), set(other)), key=len)
        if small <= big and len(small) >= 2 and len(big) - len(small) <= CONTAIN_SLACK:
            return 0.95
        sm = difflib.SequenceMatcher(None, " ".join(toks), " ".join(other))
        if sm.real_quick_ratio() < FUZZY_MIN or sm.quick_ratio() < FUZZY_MIN:
            return 0.0
        return sm.ratio()


def _link(work_id: str, row: Mapping[str, Any], level: str, score: float) -> ScoreLink:
    return ScoreLink(work_id, WorkSource("pdmx", str(row["material_id"]), level, round(score, 3)),
                     str(row.get("license") or ""), row.get("rating"), row.get("keysig"), row.get("key"),
                     str(row.get("composer") or ""))


def _exact_links(works: Sequence[Sequence[str]], idx: _Scores) -> Dict[str, List[Tuple[str, int]]]:
    """``{work_id: [(уровень, строка)]}`` по точному названию; партитура с ``exact_artist`` к одному произведению
    не предлагается одноимённым чужим («Dreams» Corrs ↔ Cranberries — главный источник ложных по замеру)."""
    found: Dict[str, List[Tuple[str, int]]] = {}
    owner: Dict[int, str] = {}
    for wid, title, artist, composer in works:
        key = norm(title)
        if len(key) < MIN_TITLE or key in GENERIC_TITLES or _named_after_person(title, artist, composer):
            continue
        authors = _people(artist) | _people(composer)
        for i, dash in idx.by_title.get(key, ()):
            level = "exact_artist" if authors & idx.people[i] else "exact"
            if dash and level == "exact":  # «Название - Кто-то»: без совпавшего автора хвост мог быть названием
                continue
            found.setdefault(wid, []).append((level, i))
            if level == "exact_artist":
                owner[i] = wid
    return {wid: [(lv, i) for lv, i in hits if lv == "exact_artist" or owner.get(i, wid) == wid]
            for wid, hits in found.items()}


def _fuzzy_links(work: Sequence[str], idx: _Scores) -> List[ScoreLink]:
    work_id, title, artist, composer = work
    toks = _title_tokens(title)
    if len(set(toks)) < 2 or norm(title) in GENERIC_TITLES or _named_after_person(title, artist, composer):
        return []
    scored = sorted(((s, i) for i in idx.candidates(toks) for s in [idx.fuzzy(toks, i)] if s >= FUZZY_MIN),
                    key=lambda si: (-si[0], si[1]))[:FUZZY_PER_WORK]
    return [_link(work_id, idx.rows[i], "fuzzy", s / 2) for s, i in scored]


def match_scores(works: Iterable[Sequence[str]], rows: Iterable[Mapping[str, Any]]
                 ) -> Tuple[List[ScoreLink], List[Tuple[str, str]]]:
    """Произведения ``(work_id, title, artist, composer)`` ↔ строки ``score_index`` → ``(связи, отказы)``.

    Уровни (ADR-0155 §3.1, замер §2.2): ``exact_artist`` — нормализованное название совпало (целиком, до скобок или
    до « - автор») и композитор/автор партитуры пересёкся с исполнителем/композитором произведения; ``exact`` —
    только название (целиком или до скобок); ``fuzzy`` — вложение наборов слов (≥ 2 слов) или сходство
    ≥ :data:`FUZZY_MIN` (только у произведений без точной пары, ≤ :data:`FUZZY_PER_WORK`). Запись, названная именем
    своего автора («Mozart» — Mozart), не связывается. Ни одна связь не подтверждена (В6). Партитура без
    пригодной лицензии (:func:`material.license_usable`: пусто, ``unknown``, ``…conflict``) не связывается — отказ
    с причиной (M6)."""
    works = list(works)
    good, rejected = [], []
    for r in rows:
        if license_usable(r.get("license")):
            good.append(r)
        else:
            rejected.append((str(r["material_id"]), f"лицензия {r.get('license')!r} не годится (ADR-0154 M6)"))
    idx = _Scores(good)
    exact = _exact_links(works, idx)
    links: List[ScoreLink] = []
    for work in works:
        wid = work[0]
        hits = sorted(set(exact.get(wid, ())), key=lambda h: (h[0] != "exact_artist", h[1]))
        links += [_link(wid, idx.rows[i], lv, LINK_SCORES[lv]) for lv, i in hits] or _fuzzy_links(work, idx)
    return links, rejected


def score_links_report(conn: sqlite3.Connection) -> str:
    """Сводка связей партитур реестра: произведения и связи по уровням, подтверждённые, лицензии, стоп-список."""
    total = conn.execute("SELECT COUNT(*) FROM works").fetchone()[0]
    linked = conn.execute("SELECT COUNT(DISTINCT work_id) FROM work_sources WHERE kind='pdmx'").fetchone()[0]
    lines = [f"произведений {total}; с партитурой (любой уровень) {linked}",
             "уровень       произведений  связей  партитур  подтверждено"]
    for level, works_n, links_n, scores_n, conf in conn.execute(
            "SELECT link_level, COUNT(DISTINCT work_id), COUNT(*), COUNT(DISTINCT material_id), SUM(confirmed) "
            "FROM work_sources WHERE kind='pdmx' GROUP BY link_level ORDER BY link_level"):
        lines.append(f"{level:<13} {works_n:>12} {links_n:>7} {scores_n:>9} {conf:>13}")
    lic = conn.execute("SELECT license, COUNT(DISTINCT material_id) FROM work_sources WHERE kind='pdmx' "
                       "GROUP BY license ORDER BY 2 DESC").fetchall()
    lines.append("лицензии связанных партитур: " + (", ".join(f"{k} {n}" for k, n in lic) or "—"))
    bad = sum(n for k, n in lic if not license_usable(k))
    lines.append(f"непригодных лицензий (license_conflict, unknown, пусто) среди связанных: {bad} (M6 ADR-0154: 0)")
    stop = sum(1 for c, t in conn.execute("SELECT DISTINCT s.composer, w.title FROM work_sources s JOIN works w "
                                          "USING (work_id) WHERE s.kind='pdmx'") if is_stop_listed(c or "", t))
    lines.append(f"связей со стоп-списком живых правообладателей (knowledge.LICENSE_STOP_LIST): {stop}")
    return "\n".join(lines)


# ── Связи «фраза темы → строки поиска архива» (#3493): одно хранилище исключений и журнал непонятого ─────────────
#: Откуда связь: ``seed`` — перенос старой ручной таблицы, ``llm`` — предложение LLM, проверенное каталогом
#: («llm_suggested+catalog_verified»), ``manual`` — Шифу (``--add-theme-link``), ``miss`` — непонятое без вердикта.
THEME_LINK_SOURCES = ("seed", "llm", "manual", "miss")
#: ``found`` — есть проверенные строки поиска; ``not_found`` — LLM предложила, каталог не подтвердил ничего;
#: ``not_theme`` — часть не про музыку (стиль, повод, оценка); ``missed`` — прямой поиск промахнулся, вердикта нет
#: (LLM недоступна/опоздала) — копится для отчёта и не мешает спросить LLM в следующий раз.
THEME_LINK_STATUSES = ("found", "not_found", "not_theme", "missed")

_THEME_LINKS_SCHEMA = """
CREATE TABLE IF NOT EXISTS theme_links (phrase TEXT PRIMARY KEY, status TEXT NOT NULL, kind TEXT NOT NULL,
    queries TEXT NOT NULL, names TEXT NOT NULL, rejected TEXT NOT NULL, source TEXT NOT NULL,
    rules_version TEXT NOT NULL, created_at TEXT NOT NULL, last_hit TEXT NOT NULL, hits INTEGER NOT NULL);
"""


@dataclass(frozen=True)
class ThemeLink:
    """Запись хранилища: фраза темы (:func:`theme_phrase`) → проверенные строки поиска и вердикт."""

    phrase: str
    status: str
    source: str
    kind: str = ""
    queries: Tuple[str, ...] = ()
    names: Tuple[str, ...] = ()
    rejected: Tuple[str, ...] = ()
    rules_version: str = RULES_VERSION
    created_at: str = ""
    hits: int = 0

    def __post_init__(self) -> None:
        if not self.phrase:
            raise ValueError("ThemeLink.phrase пуст")
        if self.status not in THEME_LINK_STATUSES or self.source not in THEME_LINK_SOURCES:
            raise ValueError(f"ThemeLink: status {self.status!r} / source {self.source!r} вне перечня")
        if self.status == "found" and not self.queries:
            raise ValueError("found без проверенных строк поиска")


def theme_phrase(text: Any) -> str:
    """Ключ фразы темы: слова в нижнем регистре, ё → е, через пробел: «Танец утят!» → «танец утят»."""
    return " ".join(_WORD_RE.findall(str(text or "").lower().replace("ё", "е")))


def _link_fresh(link: ThemeLink, now: datetime) -> bool:
    """Вердикт действует: ручные и семена — всегда; LLM — пока не сменились правила и не прошёл TTL §3.4."""
    if link.status == "missed":
        return False
    if link.source in ("seed", "manual"):
        return True
    if link.rules_version != RULES_VERSION:
        return False
    return now - datetime.fromisoformat(link.created_at) <= timedelta(days=LOOKUP_TTL_DAYS)


def get_theme_link(conn: sqlite3.Connection, text: str, now: datetime) -> Optional[ThemeLink]:
    """Действующая связь фразы (счётчик срабатываний +1) или ``None``."""
    conn.executescript(_THEME_LINKS_SCHEMA)
    row = conn.execute("SELECT phrase, status, source, kind, queries, names, rejected, rules_version, created_at, hits "
                       "FROM theme_links WHERE phrase=?", (theme_phrase(text),)).fetchone()
    if row is None:
        return None
    link = ThemeLink(row[0], row[1], row[2], row[3], *(tuple(json.loads(v)) for v in row[4:7]), row[7], row[8],
                     row[9])
    if not _link_fresh(link, now):
        return None
    with conn:
        conn.execute("UPDATE theme_links SET hits=hits+1, last_hit=? WHERE phrase=?", (now.isoformat(), link.phrase))
    return link


def put_theme_link(conn: sqlite3.Connection, link: ThemeLink, now: datetime) -> None:
    """Записать вердикт (заменяет прежний). Непонятое (``missed``) не затирает действующий вердикт и копит счётчик."""
    conn.executescript(_THEME_LINKS_SCHEMA)
    stamp = now.isoformat()
    with conn:
        if link.status == "missed":
            conn.execute("INSERT INTO theme_links VALUES (?,?,?,?,?,?,?,?,?,?,1) ON CONFLICT(phrase) DO UPDATE SET "
                         "hits=hits+1, last_hit=excluded.last_hit", (link.phrase, "missed", "", "[]", "[]", "[]",
                                                                     "miss", RULES_VERSION, stamp, stamp))
            return
        conn.execute("INSERT OR REPLACE INTO theme_links VALUES (?,?,?,?,?,?,?,?,?,?,?)",
                     (link.phrase, link.status, link.kind, json.dumps(list(link.queries), ensure_ascii=False),
                      json.dumps(list(link.names), ensure_ascii=False),
                      json.dumps(list(link.rejected), ensure_ascii=False), link.source, link.rules_version, stamp,
                      stamp, link.hits))


def theme_links_report(conn: sqlite3.Connection, days: int, now: datetime) -> str:
    """Что копится: непонятые фразы (``missed``/``not_found``) по числу повторов и новые связи за ``days`` дней."""
    conn.executescript(_THEME_LINKS_SCHEMA)
    since = (now - timedelta(days=days)).isoformat()
    lines = [f"непонятые фразы тем (за {days} дн., по повторам):"]
    lines += [f"  {hits:>4}  {status:<9} «{phrase}»" for phrase, status, hits in conn.execute(
        "SELECT phrase, status, hits FROM theme_links WHERE status IN ('missed','not_found') AND last_hit>=? "
        "ORDER BY hits DESC, phrase LIMIT 50", (since,))] or ["  —"]
    lines.append(f"новые связи (за {days} дн.):")
    lines += [f"  {source:<6} {status:<9} «{phrase}» → {', '.join(json.loads(q)) or '—'}"
              for phrase, status, source, q in conn.execute(
                  "SELECT phrase, status, source, queries FROM theme_links WHERE status NOT IN ('missed') AND "
                  "created_at>=? ORDER BY created_at DESC LIMIT 100", (since,))] or ["  —"]
    return "\n".join(lines)


def read_db_records(conn: sqlite3.Connection) -> List[Dict[str, Any]]:
    conn.row_factory = sqlite3.Row
    rows = conn.execute("SELECT name, title, artist, tags, rtttl FROM rtttl_melodies ORDER BY id").fetchall()
    return [{**dict(r), "tags": json.loads(r["tags"] or "[]")} for r in rows]


def read_archive_records(path: str) -> List[Dict[str, Any]]:
    with gzip.open(path, "rt", encoding="utf-8") as fh:
        return [json.loads(line) for line in fh if line.strip()]


def _default_archive() -> str:
    from importlib.resources import files  # пакет данных живёт в rob_box_mcp_tools (по имени, не импортом)
    return str(files("rob_box_mcp_tools.data") / "rtttl_melodies.jsonl.gz")


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--db", help="SQLite с таблицей rtttl_melodies; реестр пишется в неё же")
    ap.add_argument("--archive", help="jsonl.gz архив RTTTL (по умолчанию — из пакета rob_box_mcp_tools)")
    ap.add_argument("--holes", choices=FIELDS, help="напечатать work_id рабочего множества без поля")
    ap.add_argument("--report", action="store_true", help="покрытие полей и вердикты гейта")
    ap.add_argument("--theme-report", type=int, metavar="DAYS", help="непонятые фразы тем и новые связи (нужен --db)")
    ap.add_argument("--add-theme-link", nargs="+", metavar=("PHRASE", "QUERY"),
                    help="ручная связь Шифу: фраза темы и строки поиска архива; без строк — «не тема» (нужен --db)")
    args = ap.parse_args(argv)
    conn = sqlite3.connect(args.db) if args.db else None
    if conn and (args.theme_report is not None or args.add_theme_link):
        now = datetime.now(timezone.utc).replace(tzinfo=None)
        if args.add_theme_link:
            phrase, *queries = args.add_theme_link
            put_theme_link(conn, ThemeLink(theme_phrase(phrase), "found" if queries else "not_theme", "manual",
                                           "work" if queries else "not_theme", tuple(queries)), now)
        print(theme_links_report(conn, args.theme_report or 30, now))
        return 0
    records = read_db_records(conn) if conn else read_archive_records(args.archive or _default_archive())
    works = build_works(records)
    if conn:
        print(f"реестр записан: {write_registry(conn, works)} произведений", file=sys.stderr)
    if args.holes:
        print("\n".join(holes(works, args.holes, working_ids(records))))
    if args.report or not args.holes:
        print(report(records, works))
    return 0


__all__ = ["alias_pairs", "aliases_of", "build_works", "clean_identity", "concept_queries", "Fact", "FIELDS", "Gate",
           "gate", "GATE_SHARE", "get_theme_link", "holes", "Identity", "LINK_LEVELS", "link_score_sources",
           "LOOKUP_TTL_DAYS", "match_scores", "norm", "put_theme_link", "report", "ru_phrase_by_query",
           "RULES_VERSION", "score_index_row", "score_links_report", "ScoreIndexRow", "ScoreLink", "SEEDS_FILE",
           "THEME_LINK_SOURCES", "THEME_LINK_STATUSES", "theme_links_report", "theme_phrase", "theme_seeds",
           "ThemeLink", "word_links", "Work", "work_id_of", "work_key", "working_ids", "WorkSource", "write_registry",
           "write_score_index"]

if __name__ == "__main__":
    sys.exit(main())
