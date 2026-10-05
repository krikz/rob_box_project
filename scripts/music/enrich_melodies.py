#!/usr/bin/env python3
"""Сборщик фактуры мелодий: MusicBrainz + Wikidata, LLM только предлагает и проверяет (issue #3428, ADR-0148).

Библиотека RTTTL (10461 запись) не знает, чья это песня, какого она года и жанра. Скрипт добывает факты из
открытых баз и пишет артефакт ``melody_facts.jsonl.gz``: одна строка = один принятый факт
``{name, field, value, source, source_id, source_url, fetched_at, confidence, verified, license, evidence}``.
Аранжировщик артефакт НЕ читает (отдельный PR после просмотра Шифу).

Этапы на запись (решение в коде, LLM говорит):

1. прямой запрос по полям записи: MusicBrainz ``recording`` + поиск Wikidata (``wbsearchentities``);
2. артист пуст/категория («Films And Tv», «Theme») или прямой запрос не дал годного кандидата → LLM-нормализация
   (исправленное название, артист/композитор, 2-3 запроса); с ``--llm none`` этап пропускается;
3. кандидаты баз собираются в «документы» (сырой ответ базы + разбор);
4. поля предлагаются: LLM-проверка выдачи (``--llm minimax|stub``) либо детерминированный извлекатель (``none``).
   Каждое предложение ``{field, value, source_id, quote}`` проходит ОДНО решающее место — ``accept_field``:
   цитата обязана буквально быть в сыром ответе базы, а кандидат — пройти правила (MB: score 100 + пересечение
   артиста + не remix/cover/live/karaoke/instrumental-запись + название совпало; Wikidata: тип «музыкальное
   произведение» + композитор/исполнитель; год ТОЛЬКО Wikidata P577; жанр — тег MB / P136, маппится в ключ стиля).
   Уверенность LLM для решений не используется.

«Сырой ответ базы»: для MusicBrainz — компактный JSON записи ровно как отдала база; для Wikidata — детерминированная
проекция (метки вместо Q-идентификаторов, даты P577 как в базе ``+1980-06-17T00:00:00Z``), потому что сырой
``wbgetentities`` на порядок больше и без меток нечитаем. Цитата проверяется по этому тексту.

Резюмируемость: состояние в ``<out>/enrich_state.sqlite`` (кэш HTTP-ответов, в том числе пустых — отрицательный
кэш, и статус каждой записи); повторный запуск пропускает записи со статусом ``done``. Артефакт и лог
(``enrich.log``) пишутся после каждого чанка. Между запросами к MusicBrainz ≥ 1.5 с, 503/429/таймаут — ретраи с паузой.

Локальная отладка без LLM (только базы)::

    python scripts/music/enrich_melodies.py --limit 10 --seed 20261008 --llm none --out /tmp/enrich

Живой прогон с LLM — на роботе, в контейнере ``voice-assistant`` (там ``MINIMAX_API_KEY``). ``scripts/`` в образ не
копируется (``docker/vision/voice_assistant/Dockerfile`` кладёт только пакеты и ``src/*/scripts``), поэтому скрипт
доставляют ``docker cp``; библиотека RTTTL лежит в пакете ``rob_box_mcp_tools`` (``/ws/src/rob_box_mcp_tools``)::

    docker cp scripts/music/enrich_melodies.py voice-assistant:/tmp/enrich_melodies.py
    docker exec voice-assistant python3 /tmp/enrich_melodies.py --limit 10 --seed 20261008 --llm minimax --out /tmp/enrich
    docker cp voice-assistant:/tmp/enrich ./enrich_out

Скрипт сам находит пакеты репо по пути файла (``<repo>/src``); в контейнере они уже в ``PYTHONPATH`` через overlay
``/ws/install``. ``--lib`` — путь к ``rtttl_melodies.jsonl.gz``, если автопоиск не сработал.
"""

from __future__ import annotations

import argparse
import asyncio
import gzip
import json
import logging
import random
import re
import sqlite3
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _pkg in ("rob_box_music", "rob_box_mcp_tools", "rob_box_harness", "rob_box_llm"):
    _p = str(_REPO_ROOT / "src" / _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

_LOG = logging.getLogger("enrich")
USER_AGENT = "rob_box_enrich/0.1 (https://github.com/krikz/rob_box_project)"
MB_URL = "https://musicbrainz.org/ws/2/recording?fmt=json&limit=5&query="
WD_API = "https://www.wikidata.org/w/api.php?format=json&"
MB_GAP_S = 1.6
WD_GAP_S = 1.0
RETRY_PAUSES_S = (5.0, 15.0)
ARTIFACT = "melody_facts.jsonl.gz"
DEFAULT_LIB = "src/rob_box_mcp_tools/rob_box_mcp_tools/data/rtttl_melodies.jsonl.gz"

# ---------------------------------------------------------------------------------------------- данные (знание)
BAD_ARTISTS = frozenset({"", "films and tv", "film theme", "theme", "computer games", "tv", "movie", "movies", "game",
                         "games", "anthem", "christmas", "classical", "unknown", "various", "various artists",
                         "soundtrack", "bollywood", "traditional"})
BAD_TITLES = frozenset({"", "theme", "unknown", "untitled"})
#: Запись-вариант, а не оригинал (ложный приём «Tetris Reset (Mellow Sonic remix)» в спайке).
VARIANT_RE = re.compile(r"\b(remix|re-?mix|cover|live|karaoke|instrumental|tribute|acoustic|demo|bootleg|mashup|"
                        r"medley|ringtone|piano version|8-?bit|vs\.?)\b", re.I)
STOP_TOKENS = frozenset({"the", "feat", "and", "theme", "soundtrack", "film", "ft", "featuring", "with"})
MUSIC_TYPES = ("song", "single", "soundtrack", "score", "theme", "musical composition", "musical work",
               "video game music", "signature tune", "march", "carol", "hymn", "anthem", "folk song")
#: Тип произведения Wikidata (подстрока метки instance-of) → work_type перечня промпта.
WORK_TYPE_MAP: Mapping[str, str] = {
    "folk song": "folk", "carol": "folk", "single": "song", "song": "song", "film score": "film_theme",
    "soundtrack": "film_theme", "video game music": "game_theme", "march": "classical", "anthem": "folk",
}
WORK_TYPES = frozenset(set(WORK_TYPE_MAP.values()) | {"tv_theme", "ringtone", "unknown"})
#: Жанр (подстрока тега/P136) → ключ стиля аранжировщика. ДЛЯ ADR-0153: в ``knowledge.STYLES`` после S0 есть только
#: ``club``; остальные ключи — планируемые стили, помечаются ``verified=false`` и выбор аранжировщика не меняют.
GENRE_STYLE_MAP: Mapping[str, str] = {
    "house": "club", "techno": "club", "trance": "club", "dance": "club", "electronic": "club", "edm": "club",
    "disco": "club", "euro": "club", "synth": "club",
    "hip hop": "hiphop", "hip-hop": "hiphop", "rap": "hiphop", "r&b": "rnb", "rhythm and blues": "rnb",
    "rock": "rock", "metal": "rock", "punk": "rock", "pop": "pop", "jazz": "jazz", "blues": "jazz",
    "classical": "classical", "folk": "folk", "carol": "folk", "christmas": "folk", "soundtrack": "soundtrack",
    "score": "soundtrack", "video game": "game",
}
LICENSE = {"mb": "CC0-1.0", "mb_tags": "CC-BY-NC-SA-3.0", "wd": "CC0-1.0", "mapping": "project"}
FIELDS = ("canonical_title", "artist", "composer", "work_type", "genre", "year")

PROMPT_NORMALIZE = """Ты нормализатор метаданных для библиотеки RTTTL-рингтонов (10461 запись, названия из пользовательских архивов,
поля title/artist часто перепутаны, опечатки, вместо артиста бывают категории "Films And Tv", "Theme", "Computer Games").
Тебе дают одну запись. Используй ТОЛЬКО поля записи и общие знания. Не выдумывай: если не опознаёшь - work_type "unknown",
artist_or_composer "unknown", confidence <= 0.2. Не используй поиск и базы.

INPUT (JSON): {"name","title","artist","tags"}

OUTPUT (только JSON, без текста):
{"canonical_title": str, "artist_or_composer": str, "work_type": "song|film_theme|game_theme|tv_theme|folk|classical|ringtone|unknown",
 "queries": [str, str, str],   // первая 'recording:"..." AND artist:"..."' для MusicBrainz, остальные - простой текст для Wikidata
 "confidence": 0..1, "why": str}"""

PROMPT_VERIFY = """Ты проверяешь выдачу открытых баз (MusicBrainz, Wikidata) для одной мелодии. Дана запись библиотеки и
кандидаты: у каждого source_id и raw - сырой ответ базы. Выбери кандидата, который описывает ИМЕННО ЭТО произведение
(не ремикс, не кавер, не live, не караоке), и извлеки поля. Для КАЖДОГО поля дай quote - ТОЧНУЮ подстроку raw
этого кандидата, на которой поле основано (копируй символ в символ). Нет основания в raw - поле не давай.
Год - только из Wikidata publication_dates. Жанр - только значение из tags/genre кандидата.

OUTPUT (только JSON): {"fields": [{"field": "canonical_title|artist|composer|work_type|genre|year",
 "value": str|int, "source_id": str, "quote": str}]}"""


def toks(text: str) -> frozenset:
    return frozenset(t for t in re.findall(r"[a-z0-9]+", (text or "").lower()) if len(t) > 2 and t not in STOP_TOKENS)


def norm(text: str) -> str:
    return "".join(re.findall(r"[a-z0-9]", (text or "").lower()))


def is_variant(*texts: str) -> bool:
    return any(VARIANT_RE.search(t or "") for t in texts)


def title_match(candidate: str, expected: Iterable[str]) -> bool:
    c = norm(candidate)
    for e in map(norm, expected):
        if c and e and (c == e or (min(len(c), len(e)) >= 4 and (c in e or e in c)
                                   and min(len(c), len(e)) / max(len(c), len(e)) >= 0.6)):
            return True
    return False


def style_keys() -> frozenset:
    try:
        from rob_box_music.knowledge import STYLES
        return frozenset(STYLES)
    except ImportError:  # скрипт без пакетов репо — единственный стиль S0
        return frozenset({"club"})


def genre_to_style(genre: str) -> Optional[str]:
    low = genre.lower()
    for key, style in GENRE_STYLE_MAP.items():
        if key in low:
            return style
    return None


# ------------------------------------------------------------------------------------------------ HTTP + состояние
class BudgetExceeded(RuntimeError):
    """Исчерпан ``--max-requests``; прогон останавливается, состояние сохранено."""


class State:
    """sqlite: кэш ответов (в том числе пустых) и статус записей."""

    def __init__(self, path: Path) -> None:
        self.db = sqlite3.connect(str(path))
        self.db.execute("CREATE TABLE IF NOT EXISTS http(url TEXT PRIMARY KEY, body TEXT, fetched_at TEXT)")
        self.db.execute("CREATE TABLE IF NOT EXISTS rec(name TEXT PRIMARY KEY, status TEXT, reason TEXT, "
                        "facts TEXT, updated TEXT)")
        self.db.commit()

    def cached(self, url: str) -> Optional[Tuple[str, str]]:
        row = self.db.execute("SELECT body, fetched_at FROM http WHERE url=?", (url,)).fetchone()
        return (row[0], row[1]) if row else None

    def store(self, url: str, body: str, fetched_at: str) -> None:
        self.db.execute("INSERT OR REPLACE INTO http VALUES(?,?,?)", (url, body, fetched_at))
        self.db.commit()

    def set_rec(self, name: str, status: str, reason: str, facts: Sequence[Mapping[str, Any]]) -> None:
        self.db.execute("INSERT OR REPLACE INTO rec VALUES(?,?,?,?,?)",
                        (name, status, reason, json.dumps(list(facts), ensure_ascii=False), now_iso()))
        self.db.commit()

    def done_names(self) -> frozenset:
        return frozenset(r[0] for r in self.db.execute("SELECT name FROM rec WHERE status != 'error'"))

    def records(self) -> List[Tuple[str, str, str, List[dict]]]:
        return [(n, s, r, json.loads(f)) for n, s, r, f in
                self.db.execute("SELECT name, status, reason, facts FROM rec ORDER BY name")]


def now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def urllib_get(url: str) -> Tuple[int, str]:
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT, "Accept": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            return resp.status, resp.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        return exc.code, ""
    except (urllib.error.URLError, TimeoutError, OSError):
        return 0, ""


class Fetcher:
    """Вежливый клиент: пауза на хост, ретраи 503/429/таймаута, кэш, бюджет запросов."""

    def __init__(self, state: State, *, get: Callable[[str], Tuple[int, str]] = urllib_get,
                 sleep: Callable[[float], None] = time.sleep, clock: Callable[[], float] = time.monotonic,
                 max_requests: int = 0) -> None:
        self.state, self._get, self._sleep, self._clock = state, get, sleep, clock
        self.max_requests, self.requests, self._last = max_requests, 0, {}

    def json(self, url: str) -> Tuple[Optional[dict], str]:
        """(ответ, fetched_at); ответ None — база недоступна (не кэшируется, запись получит статус error)."""
        hit = self.state.cached(url)
        if hit:
            return json.loads(hit[0]), hit[1]
        body = self._fetch(url)
        if body is None:
            return None, ""
        stamp = now_iso()
        self.state.store(url, body, stamp)
        return json.loads(body), stamp

    def _fetch(self, url: str) -> Optional[str]:
        for attempt in range(len(RETRY_PAUSES_S) + 1):
            self._pace(url)
            status, body = self._get(url)
            if status == 200:
                try:
                    json.loads(body)
                    return body
                except ValueError:
                    status = 0
            _LOG.warning("HTTP %s (попытка %d) %s", status, attempt + 1, url[:120])
            if status not in (0, 429, 502, 503, 504) or attempt >= len(RETRY_PAUSES_S):
                return None
            self._sleep(RETRY_PAUSES_S[attempt])
        return None

    def _pace(self, url: str) -> None:
        if self.max_requests and self.requests >= self.max_requests:
            raise BudgetExceeded(f"max_requests={self.max_requests}")
        host = urllib.parse.urlparse(url).netloc
        gap = MB_GAP_S if "musicbrainz" in host else WD_GAP_S
        wait = self._last.get(host, -1e9) + gap - self._clock()
        if wait > 0:
            self._sleep(wait)
        self._last[host] = self._clock()
        self.requests += 1


# -------------------------------------------------------------------------------------------- кандидаты и документы
@dataclass
class Doc:
    """Кандидат базы. ``raw`` — текст, по которому проверяется цитата; поля разобраны кодом."""

    source: str                    # mb | wd
    source_id: str
    url: str
    raw: str
    title: str
    people: List[str] = field(default_factory=list)      # MB: artist-credit; WD: исполнители (P175)
    composers: List[str] = field(default_factory=list)   # WD P86
    instance_of: List[str] = field(default_factory=list)
    genres: List[str] = field(default_factory=list)
    dates: List[str] = field(default_factory=list)       # WD P577 как в базе
    score: int = 0
    fetched_at: str = ""
    extra: str = ""                                      # disambiguation и т.п. для фильтра вариантов


def compact(obj: Any) -> str:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))


def mb_docs(resp: Mapping[str, Any], fetched_at: str) -> List[Doc]:
    docs = []
    for item in resp.get("recordings", []) or []:
        names = [c.get("name", "") for c in item.get("artist-credit", []) if isinstance(c, dict)]
        docs.append(Doc("mb", item["id"], "https://musicbrainz.org/recording/" + item["id"], compact(item),
                        item.get("title", ""), people=names, score=int(item.get("score", 0)),
                        genres=[t["name"] for t in item.get("tags", []) if "name" in t],
                        fetched_at=fetched_at, extra=item.get("disambiguation", "")))
    return docs


def wd_ids(resp: Mapping[str, Any], limit: int = 3) -> List[str]:
    return [c["id"] for c in (resp.get("search") or [])[:limit] if "id" in c]


def claim_ids(entity: Mapping[str, Any], prop: str) -> List[str]:
    out = []
    for c in entity.get("claims", {}).get(prop, []):
        v = c.get("mainsnak", {}).get("datavalue", {})
        if v.get("type") == "wikibase-entityid":
            out.append(v["value"]["id"])
    return out


def claim_dates(entity: Mapping[str, Any]) -> List[str]:
    out = []
    for c in entity.get("claims", {}).get("P577", []):
        v = c.get("mainsnak", {}).get("datavalue", {})
        if v.get("type") == "time":
            out.append(v["value"]["time"])
    return out


def wd_doc(qid: str, entity: Mapping[str, Any], labels: Mapping[str, str], fetched_at: str) -> Doc:
    def lab(prop: str) -> List[str]:
        return [labels.get(i, i) for i in claim_ids(entity, prop)]
    label = entity.get("labels", {}).get("en", {}).get("value", "")
    digest = {"qid": qid, "label": label, "description": entity.get("descriptions", {}).get("en", {}).get("value", ""),
              "instance_of": lab("P31"), "composer": lab("P86"), "performer": lab("P175"), "genre": lab("P136"),
              "publication_dates": claim_dates(entity)}
    return Doc("wd", qid, "https://www.wikidata.org/wiki/" + qid, compact(digest), label, people=digest["performer"],
               composers=digest["composer"], instance_of=digest["instance_of"], genres=digest["genre"],
               dates=digest["publication_dates"], fetched_at=fetched_at)


# ----------------------------------------------------------------------------------------------- решение (код)
@dataclass
class Anchor:
    """Что мы знаем о записи до баз: названия и артист для сверки; ``via`` — откуда (library|llm)."""

    titles: List[str]
    artist: str
    via: str

    @property
    def usable(self) -> bool:
        return bool(toks(self.artist)) and any(norm(t) for t in self.titles)


def library_anchor(rec: Mapping[str, Any]) -> Anchor:
    title, artist = rec.get("title", ""), rec.get("artist", "")
    bad_title = title.strip().lower() in BAD_TITLES
    bad_artist = artist.strip().lower() in BAD_ARTISTS or artist.strip().lower().endswith(" theme")
    return Anchor([] if bad_title else [title], "" if bad_artist else artist, "library")


def eligible(doc: Doc, anchor: Anchor) -> Tuple[bool, str]:
    """Годен ли кандидат как «то самое произведение» (правила приёма из спайка + фильтр вариантов)."""
    if not anchor.usable:
        return False, "no_anchor"
    if is_variant(doc.title, doc.extra):
        return False, "variant"
    if not title_match(doc.title, anchor.titles):
        return False, "title_mismatch"
    names = " ".join(doc.people + doc.composers)
    if not toks(anchor.artist) & toks(names):
        return False, "artist_mismatch"
    if doc.source == "mb":
        return (True, "") if doc.score == 100 else (False, f"score_{doc.score}")
    kinds = " ".join(doc.instance_of).lower()
    return (True, "") if any(k in kinds for k in MUSIC_TYPES) else (False, "not_a_musical_work")


def accept_field(prop: Mapping[str, Any], docs: Mapping[str, Doc], anchor: Anchor) -> Tuple[bool, str]:
    """Единственное место, где предложение (LLM или извлекателя) становится фактом."""
    doc = docs.get(str(prop.get("source_id", "")))
    fld, value, quote = prop.get("field"), prop.get("value"), str(prop.get("quote", ""))
    if doc is None or fld not in FIELDS or value in (None, ""):
        return False, "unknown_candidate_or_field"
    if not quote or quote not in doc.raw:
        return False, "quote_not_in_raw"
    ok, why = eligible(doc, anchor)
    if not ok:
        return False, why
    return _field_rule(str(fld), value, quote, doc)


def _field_rule(fld: str, value: Any, quote: str, doc: Doc) -> Tuple[bool, str]:
    sval = str(value)
    if fld == "year":
        if doc.source != "wd":
            return False, "year_only_wikidata_p577"
        ok = isinstance(value, int) and re.fullmatch(r"[+-]\d{4}-\d\d-\d\d.*", quote) and \
            any(quote == d and int(d[1:5]) == value for d in doc.dates)
        return (True, "") if ok else (False, "year_not_in_p577")
    if fld == "work_type":
        # значение — ключ перечня, в цитате метка типа произведения: поддержка = метка в цитате + маппинг даёт значение
        ok = any(WORK_TYPE_MAP[k] == sval and k in i.lower() and i in quote for k in WORK_TYPE_MAP
                 for i in doc.instance_of)
        return (True, "") if ok else (False, "work_type_not_supported")
    if sval.lower() not in quote.lower():
        return False, "value_not_in_quote"
    pools = {"canonical_title": [doc.title], "artist": doc.people, "composer": doc.composers,
             "genre": doc.genres}
    if fld in pools and not any(sval.lower() == p.lower() for p in pools[fld]):
        return False, f"{fld}_not_in_doc"
    return True, ""


def quote_of(doc: Doc, fld: str, value: Any) -> str:
    """Цитата для детерминированного извлекателя: точный фрагмент ``doc.raw``."""
    if fld == "year":
        return value if isinstance(value, str) else ""
    if fld in ("canonical_title", "artist", "genre") and doc.source == "mb":
        return '"' + ("title" if fld == "canonical_title" else "name") + '":' + json.dumps(str(value), ensure_ascii=False)
    key = {"canonical_title": "label", "artist": "performer", "composer": "composer", "genre": "genre",
           "work_type": "instance_of"}.get(fld, fld)
    m = re.search(r'"%s":(\[[^\]]*\]|"[^"]*")' % key, doc.raw)
    return m.group(0) if m else ""


def rule_proposals(docs: Sequence[Doc], anchor: Anchor) -> List[dict]:
    """``--llm none``: годные кандидаты → предложения по всем полям (тот же ``accept_field``)."""
    props: List[dict] = []
    for doc in docs:
        if not eligible(doc, anchor)[0]:
            continue
        props.append({"field": "canonical_title", "value": doc.title, "source_id": doc.source_id})
        props += [{"field": "artist", "value": n, "source_id": doc.source_id} for n in doc.people[:1]]
        props += [{"field": "composer", "value": n, "source_id": doc.source_id} for n in doc.composers[:1]]
        props += [{"field": "genre", "value": g, "source_id": doc.source_id} for g in doc.genres[:2]]
        for kind, wtype in WORK_TYPE_MAP.items():
            if any(kind in i.lower() for i in doc.instance_of):
                props.append({"field": "work_type", "value": wtype, "source_id": doc.source_id})
                break
        if doc.dates:
            props.append({"field": "year", "value": int(min(doc.dates)[1:5]), "source_id": doc.source_id,
                          "quote": min(doc.dates)})
    for p in props:
        p.setdefault("quote", quote_of(next(d for d in docs if d.source_id == p["source_id"]), p["field"], p["value"]))
    return props


def fact_row(name: str, prop: Mapping[str, Any], doc: Doc, anchor: Anchor) -> dict:
    tag = prop["field"] == "genre" and doc.source == "mb"
    conf = 0.9 if anchor.via == "library" else 0.7
    return {"name": name, "field": prop["field"], "value": prop["value"], "source": "musicbrainz" if doc.source == "mb"
            else "wikidata", "source_id": doc.source_id, "source_url": doc.url, "fetched_at": doc.fetched_at,
            "confidence": conf if prop["field"] != "year" else min(conf, 0.8), "verified": True,
            "license": LICENSE["mb_tags" if tag else doc.source], "evidence": prop["quote"]}


def style_rows(rows: Sequence[dict]) -> List[dict]:
    """Жанр → ключ стиля (данные для ADR-0153, аранжировщик не читает); verified только если ключ есть в STYLES."""
    known = style_keys()
    out: List[dict] = []
    seen = set()
    for r in rows:
        if r["field"] != "genre":
            continue
        key = genre_to_style(str(r["value"]))
        if key and key not in seen:
            seen.add(key)
            out.append(dict(r, field="style_hint", value=key, source="mapping", license=LICENSE["mapping"],
                            verified=key in known, confidence=min(r["confidence"], 0.6)))
    return out


# --------------------------------------------------------------------------------------------------------- LLM
class Llm:
    """Подключаемый провайдер: ``complete(system, user) -> dict`` (JSON-ответ)."""

    def complete(self, system: str, user: str) -> dict:  # pragma: no cover - интерфейс
        raise NotImplementedError


class MinimaxLlm(Llm):
    """Тот же провайдер и клиент, что у ризонера робота (``engine.reasoner.minimax_provider``, ``MINIMAX_API_KEY``)."""

    def __init__(self, timeout_s: float = 60.0) -> None:
        from rob_box_mcp_tools.engine.reasoner import minimax_provider
        self._client = minimax_provider(timeout_s)()
        self._timeout = timeout_s

    def complete(self, system: str, user: str) -> dict:
        from rob_box_llm.provider import LLMMessage, LLMSettings

        async def run() -> Any:
            return await asyncio.wait_for(self._client.complete(
                [LLMMessage("system", system), LLMMessage("user", user)],
                settings=LLMSettings(max_tokens=2048, temperature=0.0)), timeout=self._timeout)
        return parse_json_reply(getattr(asyncio.run(run()), "content", "") or "")


def parse_json_reply(text: str) -> dict:
    text = re.sub(r"<think>.*?</think>", "", text, flags=re.S).strip()
    text = re.sub(r"^```(?:json)?|```$", "", text.strip(), flags=re.M).strip()
    try:
        data = json.loads(text)
    except ValueError:
        data = json.loads(text[text.index("{"):text.rindex("}") + 1])
    if not isinstance(data, dict):
        raise ValueError("ответ LLM не объект")
    return data


class StubLlm(Llm):
    """Без сети: «нормализация» возвращает поля записи как есть, проверка выдачи ничего не предлагает (отладка проводки)."""

    def complete(self, system: str, user: str) -> dict:
        if system == PROMPT_NORMALIZE:
            rec = json.loads(user)
            return {"canonical_title": rec.get("title", ""), "artist_or_composer": rec.get("artist") or "unknown",
                    "queries": [], "confidence": 0.0}
        return {"fields": []}


def build_llm(kind: str) -> Optional[Llm]:
    if kind == "none":
        return None
    if kind == "minimax":
        return MinimaxLlm()
    return StubLlm()


# ----------------------------------------------------------------------------------------------- конвейер записи
@dataclass
class Outcome:
    status: str                  # done | error
    reason: str
    facts: List[dict] = field(default_factory=list)
    rejects: List[str] = field(default_factory=list)


def is_bad_for_direct(anchor: Anchor) -> bool:
    return not anchor.usable


def mb_query(title: str, artist: str) -> str:
    q = 'recording:"%s"' % title.replace('"', "")
    return q + (' AND artist:"%s"' % artist.replace('"', "") if artist else "")


def gather(fetch: Fetcher, mb_queries: Sequence[str], wd_texts: Sequence[str]) -> Optional[List[Doc]]:
    docs: List[Doc] = []
    for q in mb_queries:
        resp, stamp = fetch.json(MB_URL + urllib.parse.quote(q))
        if resp is None:
            return None
        docs += mb_docs(resp, stamp)
    ids: List[str] = []
    for text in wd_texts:
        resp, _ = fetch.json(WD_API + "action=wbsearchentities&language=en&limit=3&search=" + urllib.parse.quote(text))
        if resp is None:
            return None
        ids += [i for i in wd_ids(resp) if i not in ids]
    wd = wd_documents(fetch, ids[:3]) if ids else []
    return None if wd is None else docs + wd


def wd_documents(fetch: Fetcher, ids: Sequence[str]) -> Optional[List[Doc]]:
    resp, stamp = fetch.json(WD_API + "action=wbgetentities&props=claims|labels|descriptions&languages=en&ids="
                             + "|".join(ids))
    if resp is None:
        return None
    ents = resp.get("entities", {})
    need = sorted({i for e in ents.values() for p in ("P31", "P86", "P175", "P136") for i in claim_ids(e, p)})
    labels: Dict[str, str] = {}
    if need:
        lresp, _ = fetch.json(WD_API + "action=wbgetentities&props=labels&languages=en&ids=" + "|".join(need[:50]))
        if lresp is None:
            return None
        labels = {k: v.get("labels", {}).get("en", {}).get("value", k) for k, v in lresp.get("entities", {}).items()}
    return [wd_doc(q, ents[q], labels, stamp) for q in ids if q in ents]


def llm_anchor(llm: Llm, rec: Mapping[str, Any]) -> Tuple[Optional[Anchor], List[str], List[str], str]:
    """Этап 2. Возвращает (якорь, MB-запросы, WD-тексты, причина отказа)."""
    user = compact({k: rec.get(k, "") for k in ("name", "title", "artist", "tags")})
    data = llm.complete(PROMPT_NORMALIZE, user)
    artist, canon = str(data.get("artist_or_composer", "")), str(data.get("canonical_title", ""))
    if artist.lower() in ("", "unknown") and canon.lower() in ("", "unknown"):
        return None, [], [], "llm_unknown"
    queries = [str(q) for q in data.get("queries", []) if isinstance(q, str)][:3]
    mbq = [q for q in queries if q.startswith("recording:")] or [mb_query(canon, artist if artist != "unknown" else "")]
    wdq = [q for q in queries if not q.startswith("recording:")] or [canon]
    return Anchor([canon, rec.get("title", "")], "" if artist.lower() == "unknown" else artist, "llm"), mbq[:1], wdq[:2], ""


def verify_with_llm(llm: Llm, rec: Mapping[str, Any], docs: Sequence[Doc]) -> List[dict]:
    cands = [{"source_id": d.source_id, "source": d.source, "raw": d.raw} for d in docs]
    data = llm.complete(PROMPT_VERIFY, compact({"record": {k: rec.get(k, "") for k in ("title", "artist", "tags")},
                                                "candidates": cands}))
    return [p for p in data.get("fields", []) if isinstance(p, dict)]


def decide(name: str, props: Iterable[Mapping[str, Any]], docs: Sequence[Doc], anchor: Anchor) -> Outcome:
    by_id = {d.source_id: d for d in docs}
    rows: List[dict] = []
    rejects: List[str] = []
    seen = set()
    for p in props:
        ok, why = accept_field(p, by_id, anchor)
        key = (p.get("field"), str(p.get("value")).lower())
        if ok and key not in seen:
            seen.add(key)
            rows.append(fact_row(name, p, by_id[str(p["source_id"])], anchor))
        elif not ok:
            rejects.append(f"{p.get('field')}:{why}")
    rows += style_rows(rows)
    if not rows:
        return Outcome("done", "no_match", [], rejects)
    return Outcome("done", "accepted", rows, rejects)


def enrich_record(rec: Mapping[str, Any], fetch: Fetcher, llm: Optional[Llm]) -> Outcome:
    name = str(rec["name"])
    anchor = library_anchor(rec)
    docs: List[Doc] = []
    if anchor.usable:
        got = gather(fetch, [mb_query(anchor.titles[0], anchor.artist)], [anchor.titles[0]])
        if got is None:
            return Outcome("error", "http_unavailable")
        docs = got
    if not any(eligible(d, anchor)[0] for d in docs) and llm is not None:
        anchor2, mbq, wdq, why = llm_anchor(llm, rec)
        if anchor2 is None:
            return Outcome("done", why)
        got = gather(fetch, mbq, wdq)
        if got is None:
            return Outcome("error", "http_unavailable")
        anchor, docs = anchor2, docs + [d for d in got if d.source_id not in {x.source_id for x in docs}]
    if not anchor.usable:
        return Outcome("done", "no_anchor")
    props = verify_with_llm(llm, rec, docs) if llm is not None else rule_proposals(docs, anchor)
    return decide(name, props, docs, anchor)


# ----------------------------------------------------------------------------------------------------- прогон
def load_library(path: Path) -> List[dict]:
    with gzip.open(path, "rt", encoding="utf8") as fh:
        return [json.loads(line) for line in fh if line.strip()]


def pick(lib: Sequence[dict], limit: int, seed: int, names: Sequence[str]) -> List[dict]:
    if names:
        wanted = set(names)
        return [r for r in lib if r["name"] in wanted]
    return random.Random(seed).sample(list(lib), min(limit, len(lib))) if limit else list(lib)


def write_artifact(state: State, out: Path) -> int:
    rows = [f for _, _, _, facts in state.records() for f in facts]
    with gzip.open(out / ARTIFACT, "wt", encoding="utf8") as fh:
        for row in rows:
            fh.write(json.dumps(row, ensure_ascii=False) + "\n")
    return len(rows)


def run(records: Sequence[dict], out: Path, *, fetch_factory: Callable[[State], Fetcher], llm: Optional[Llm],
        chunk: int = 50) -> Dict[str, Any]:
    out.mkdir(parents=True, exist_ok=True)
    state = State(out / "enrich_state.sqlite")
    fetch = fetch_factory(state)
    skip = state.done_names()
    todo = [r for r in records if r["name"] not in skip]
    _LOG.info("записей %d, уже сделано %d, в работе %d", len(records), len(records) - len(todo), len(todo))
    try:
        for i in range(0, len(todo), chunk):
            for rec in todo[i:i + chunk]:
                outcome = safe_enrich(rec, fetch, llm)
                state.set_rec(rec["name"], outcome.status, outcome.reason + (
                    " | " + "; ".join(outcome.rejects) if outcome.rejects else ""), outcome.facts)
                _LOG.info("%s -> %s %s facts=%d", rec["name"], outcome.status, outcome.reason, len(outcome.facts))
            _LOG.info("чанк %d: артефакт %d фактов, запросов %d", i // chunk, write_artifact(state, out),
                      fetch.requests)
    except BudgetExceeded as exc:
        _LOG.warning("бюджет запросов исчерпан (%s); состояние сохранено, повторный запуск продолжит", exc)
    write_artifact(state, out)
    return summarize(state, fetch.requests)


def safe_enrich(rec: Mapping[str, Any], fetch: Fetcher, llm: Optional[Llm]) -> Outcome:
    try:
        return enrich_record(rec, fetch, llm)
    except BudgetExceeded:
        raise
    except Exception as exc:  # noqa: BLE001 - одна плохая запись не валит прогон; повторится при --resume
        _LOG.exception("запись %s", rec.get("name"))
        return Outcome("error", f"{type(exc).__name__}: {exc}")


def summarize(state: State, requests: int) -> Dict[str, Any]:
    recs = state.records()
    fields: Dict[str, int] = {}
    for _, _, _, facts in recs:
        for f in facts:
            fields[f["field"]] = fields.get(f["field"], 0) + 1
    status: Dict[str, int] = {}
    for _, s, reason, _ in recs:
        key = f"{s}:{reason.split(' | ')[0]}"
        status[key] = status.get(key, 0) + 1
    return {"records": len(recs), "requests": requests, "by_status": status, "facts_by_field": fields}


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--lib", default=str(_REPO_ROOT / DEFAULT_LIB))
    ap.add_argument("--out", required=True)
    ap.add_argument("--limit", type=int, default=10, help="0 = вся библиотека")
    ap.add_argument("--seed", type=int, default=20261008)
    ap.add_argument("--names", default="", help="имена записей через запятую (вместо выборки)")
    ap.add_argument("--llm", choices=("none", "minimax", "stub"), default="none")
    ap.add_argument("--chunk", type=int, default=50)
    ap.add_argument("--max-requests", type=int, default=0, help="потолок HTTP-запросов за запуск, 0 = без потолка")
    return ap.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                        handlers=[logging.FileHandler(out / "enrich.log", encoding="utf8"), logging.StreamHandler()])
    records = pick(load_library(Path(args.lib)), args.limit, args.seed, [n for n in args.names.split(",") if n])
    summary = run(records, out, fetch_factory=lambda st: Fetcher(st, max_requests=args.max_requests),
                  llm=build_llm(args.llm), chunk=args.chunk)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
