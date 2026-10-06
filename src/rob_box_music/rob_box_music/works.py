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
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .knowledge import (CATEGORY_ARTISTS, DEFAULT_HOOKS, EMPTY_TITLES, LICENSE_STOP_LIST, RU_ALIASES, TAG_WORK_TYPE,
                        THEMES)
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
    return len(works)


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
    args = ap.parse_args(argv)
    conn = sqlite3.connect(args.db) if args.db else None
    records = read_db_records(conn) if conn else read_archive_records(args.archive or _default_archive())
    works = build_works(records)
    if conn:
        print(f"реестр записан: {write_registry(conn, works)} произведений", file=sys.stderr)
    if args.holes:
        print("\n".join(holes(works, args.holes, working_ids(records))))
    if args.report or not args.holes:
        print(report(records, works))
    return 0


__all__ = ["Fact", "FIELDS", "GATE_SHARE", "Gate", "Identity", "LINK_LEVELS", "LOOKUP_TTL_DAYS", "RULES_VERSION", "Work",
           "WorkSource", "alias_pairs", "aliases_of", "build_works", "clean_identity", "gate", "holes", "norm",
           "report", "ru_phrase_by_query", "work_id_of", "work_key", "working_ids", "write_registry"]

if __name__ == "__main__":
    sys.exit(main())
