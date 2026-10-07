"""Реестр произведений: одна запись ``Work`` на пьесу, у каждого поля — источник (ADR-0155 K-2).

Что здесь живёт: модель (``Work``/``Fact``/``WorkSource``), нормализация названий, чистка идентичности записей
RTTTL-библиотеки (категория вместо исполнителя → тип произведения; «Theme» + название в ``artist`` → ``title``),
русские алиасы (одна таблица :data:`knowledge.RU_ALIASES`; из неё поиск ``rtttl_library`` берёт :func:`alias_pairs` и
:func:`ru_phrase_by_query`), группы версий (канон — порядок ``canon``), гейт «обогащать поле или нет»
(:func:`gate`, §3.3 ADR) и отчёт. Сеть здесь не вызывается: ни один сетевой источник гейт сегодня не проходит.

Реестр — таблицы ``works``/``work_sources``/``work_facts`` в той же SQLite, что RTTTL-библиотека (ADR-0155 В3):
``python -m rob_box_music.works --db voice_memory.db`` перестраивает их (идемпотентно), ``--holes genre`` печатает
``work_id`` рабочего множества без значения поля, ``--report`` — покрытие полей и вердикты гейта.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import re
import sqlite3
import sys
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .knowledge import (CATEGORY_ARTISTS, DEFAULT_HOOKS, EMPTY_TITLES, LICENSE_STOP_LIST, ROOTS, RU_ALIASES,
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


# ── Алиасы: одна таблица, два читателя (поиск библиотеки) ────────────────────────────────────────────────────

def alias_pairs() -> List[Tuple[str, str]]:
    """``(фраза, канонический запрос)``, длинные фразы первыми: «гимн ссср» раньше «гимн»."""
    return sorted(RU_ALIASES.items(), key=lambda kv: -len(kv[0]))


def ru_phrase_by_query() -> Dict[str, str]:
    """Канонический запрос → ПЕРВАЯ (по порядку таблицы) русская фраза на него (озвучка названия, #3178)."""
    out: Dict[str, str] = {}
    for phrase, query in RU_ALIASES.items():
        if _CYRILLIC_RE.search(phrase):
            out.setdefault(query, phrase)
    return out


def aliases_of(title: str, artist: str) -> Tuple[str, ...]:
    """Русские фразы, чей канонический запрос целиком состоит из слов названия/исполнителя (как слова поиска)."""
    have = _words(title) | _words(artist)
    return tuple(p for p, q in RU_ALIASES.items() if _CYRILLIC_RE.search(p) and _words(q) <= have)


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
    PRIMARY KEY (work_id, material_id));
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
        for table in ("works", "work_sources", "work_facts"):
            conn.execute(f"DELETE FROM {table}")
        for w in works:
            conn.execute("INSERT INTO works VALUES (?,?,?,?,?,?)", (w.work_id, w.title, w.artist, w.composer,
                         json.dumps(list(w.aliases), ensure_ascii=False), int(w.stop_listed)))
            conn.executemany("INSERT INTO work_sources VALUES (?,?,?,?,?,?,?)",
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


def link_score_sources(conn: sqlite3.Connection) -> int:
    """Подвязать партитуры ``score_index`` к произведениям реестра как источники ``pdmx:<id>`` — **предложением**
    (ADR-0155 В6): ``exact_artist`` — название совпало и композитор пересёкся с автором/исполнителем, ``exact`` —
    только название; ``confirmed`` не ставится (подтверждает человек, ``manual``). Возвращает число связей;
    без ``score_index`` или ``works`` — 0."""
    have = {r[0] for r in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    if not {"score_index", "works", "work_sources"} <= have:
        return 0
    by_title: Dict[str, List[Tuple[str, str]]] = {}
    for wid, title, artist, composer in conn.execute("SELECT work_id, title, artist, composer FROM works"):
        by_title.setdefault(norm(title), []).append((wid, artist + " " + composer))
    out = []
    with conn:
        conn.execute("DELETE FROM work_sources WHERE kind='pdmx'")
        ranks = dict(conn.execute("SELECT work_id, MAX(rank) FROM work_sources GROUP BY work_id"))
        for mid, title, composer in conn.execute("SELECT material_id, title, composer FROM score_index ORDER BY "
                                                 "COALESCE(rating, 0) DESC, material_id").fetchall():
            for wid, authors in by_title.get(norm(title), []):
                same = bool(_words(composer) & _words(authors))
                ranks[wid] = ranks.get(wid, -1) + 1
                out.append((wid, mid, "pdmx", "exact_artist" if same else "exact", 0.9 if same else 0.6, 0, ranks[wid]))
        conn.executemany("INSERT OR REPLACE INTO work_sources VALUES (?,?,?,?,?,?,?)", out)
    return len(out)


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


__all__ = ["alias_pairs", "aliases_of", "build_works", "clean_identity", "Fact", "FIELDS", "Gate", "gate",
           "GATE_SHARE", "get_theme_link", "holes", "Identity", "LINK_LEVELS", "link_score_sources", "LOOKUP_TTL_DAYS",
           "norm", "put_theme_link", "report", "ru_phrase_by_query", "RULES_VERSION", "score_index_row",
           "ScoreIndexRow", "THEME_LINK_SOURCES", "THEME_LINK_STATUSES", "theme_links_report", "theme_phrase",
           "ThemeLink", "Work", "work_id_of", "work_key", "working_ids", "WorkSource", "write_registry",
           "write_score_index"]

if __name__ == "__main__":
    sys.exit(main())
