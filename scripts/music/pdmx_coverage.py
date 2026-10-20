#!/usr/bin/env python3
"""Замер для ADR-0155: что из знания о мелодии закрывают метаданные PDMX и насколько PDMX покрывает RTTTL-библиотеку.

Офлайн, stdlib, без сети. Вход — ``PDMX.csv`` (Zenodo 10.5281/zenodo.15571083), опционально ``metadata.tar.gz``
(JSON MuseScore на партитуру) и библиотека RTTTL ``rtttl_melodies.jsonl.gz``. Выход — Markdown-отчёт с сырыми числами
(``--out``) и JSONL сопоставлений (``--matches``) для ручного просмотра.

Что считает:

1. колонки ``PDMX.csv`` и доля заполненности каждой (значение не ``NA``/пусто) — по всему CSV и по подмножеству
   ``subset:no_license_conflict``; распределения жанров, лицензий, рейтинга;
2. ключи JSON ``metadata.tar.gz`` (выборка ``--meta-sample`` файлов) и их заполненность — там ли темп/тональность;
3. покрытие RTTTL партитурами: по нормализованному названию точно (``exact``), точно + совпал артист/композитор
   (``exact_artist``), нечётко (``fuzzy``: один рядок токенов содержит другой или difflib ≥ ``FUZZY_MIN`` при общем
   редком токене) — в целом, по тегам (tv/movie/game/classical/anthem/christmas/folk) и по списку тем Шифу;
4. сколько партитур PDMX находит каждая тема Шифу напрямую (по названию/композитору), всего и в
   ``no_license_conflict``.

Пример (katana)::

    python3 scripts/music/pdmx_coverage.py --csv ~/pdmx/PDMX.csv --metadata ~/pdmx/metadata.tar.gz \\
        --lib rtttl_melodies.jsonl.gz --out pdmx_coverage.md --matches pdmx_matches.jsonl
"""

from __future__ import annotations

import argparse
import csv
import difflib
import gzip
import io
import json
import random
import re
import sys
import tarfile
import time
import unicodedata
from collections import Counter, defaultdict
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

csv.field_size_limit(min(sys.maxsize, 2**31 - 1))

NA = frozenset({"", "NA", "nan", "None", "[]", "{}"})
NLC = "subset:no_license_conflict"
TAGS = ("tv", "movie", "game", "classical", "anthem", "christmas", "folk")
#: Темы из реальных запросов товарища Шифу (бриф 06.10): ключевые слова в названии записи RTTTL и в PDMX.
SHIFU_THEMES: Mapping[str, Tuple[str, ...]] = {
    "Марио": ("mario",), "Тетрис": ("tetris", "korobeiniki"), "Контра": ("contra",), "Зельда": ("zelda",),
    "Аладдин": ("aladdin",), "Интерстеллар": ("interstellar",), "Терминатор": ("terminator",),
    "Попкорн": ("popcorn",), "Моцарт 40": ("mozart", "symphony 40", "k 550", "k550", "symphony no 40"),
    "Чайковский": ("tchaikovsky", "chaikovsky"), "Вивальди": ("vivaldi",), "Бах": ("bach",),
}
#: Целословный поиск по PDMX (первый прогон: подстрока «contra» ловила «contralto», «mozart» — любого Моцарта).
SHIFU_EXACT: Tuple[str, ...] = ("super mario", "korobeiniki", "tetris", "contra", "zelda", "aladdin", "interstellar",
                                "terminator", "popcorn", "symphony 40", "nutcracker", "four seasons", "toccata")
#: Слова, которые не опознают произведение: без них сравниваются названия.
STOP = frozenset({"the", "a", "an", "of", "in", "from", "and", "theme", "themes", "song", "music", "for", "to", "by",
                  "op", "no", "version", "ver", "remix", "main", "title", "tune", "intro", "ost", "soundtrack"})
FUZZY_MIN = 0.88
#: Названия, которые не опознают произведение (28 записей «Unknown» ↔ 28 партитур «Unknown» в первом прогоне).
GENERIC_TITLES = frozenset({"unknown", "untitled", "theme", "untitled song", "test", "song", "melody", "intro"})
#: Нечётко: меньший набор токенов ≥ 2 и вложен в больший, который длиннее не более чем на столько токенов
#: (первый прогон: «Not yet» ⊂ «I'm Not A Girl Not Yet A Woman», гимны вбирали любые два слова).
CONTAIN_SLACK = 2
MAX_CANDIDATES = 4000
RARE_DF = 3000


# ------------------------------------------------------------------------------------------------- нормализация
def norm(text: Optional[str]) -> str:
    """Нижний регистр, без диакритики и пунктуации, без хвоста « - композитор» и номера версии («Walk Of Life 3»)."""
    if not text:
        return ""
    text = unicodedata.normalize("NFKD", text)
    text = "".join(ch for ch in text if not unicodedata.combining(ch)).lower()
    text = text.split(" - ")[0]
    text = re.sub(r"[^a-z0-9а-яё]+", " ", text).strip()
    text = re.sub(r"\s\d{1,2}$", "", text)
    return text


def tokens(text: str) -> Tuple[str, ...]:
    return tuple(t for t in norm(text).split() if t not in STOP)


def is_na(value: Optional[str]) -> bool:
    return value is None or value.strip() in NA


# --------------------------------------------------------------------------------------------------------- CSV
def read_csv(path: str) -> Tuple[List[str], List[Dict[str, str]]]:
    with open(path, encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        rows = list(reader)
        return list(reader.fieldnames or []), rows


def fill_rates(columns: Sequence[str], rows: Sequence[Mapping[str, str]]) -> Dict[str, float]:
    n = len(rows) or 1
    return {c: sum(1 for r in rows if not is_na(r.get(c))) / n for c in columns}


def top_values(rows: Sequence[Mapping[str, str]], column: str, k: int = 20, split: Optional[str] = None) -> List[Tuple[str, int]]:
    counter: Counter = Counter()
    for r in rows:
        v = r.get(column)
        if is_na(v):
            continue
        counter.update(v.split(split) if split else [v])
    return counter.most_common(k)


# ---------------------------------------------------------------------------------------------------- metadata
def _flatten(obj: Any, prefix: str = "", out: Optional[Dict[str, bool]] = None, depth: int = 0) -> Dict[str, bool]:
    """Ключи JSON с признаком «заполнено» (не None/''/[]/{}); вложенность до 3 уровней, списки — по первому элементу."""
    out = {} if out is None else out
    if isinstance(obj, dict) and depth < 3:
        for k, v in obj.items():
            key = f"{prefix}.{k}" if prefix else k
            out[key] = v not in (None, "", [], {}) and v != "NA"
            _flatten(v, key, out, depth + 1)
    elif isinstance(obj, list) and obj and depth < 3:
        _flatten(obj[0], prefix + "[]", out, depth + 1)
    return out


def metadata_keys(path: str, sample: int) -> Tuple[int, Dict[str, int]]:
    filled: Counter = Counter()
    seen = 0
    with tarfile.open(path, "r:gz") as tar:
        for member in tar:
            if not member.isfile() or not member.name.endswith(".json"):
                continue
            fh = tar.extractfile(member)
            if fh is None:
                continue
            try:
                data = json.load(io.TextIOWrapper(fh, encoding="utf-8"))
            except (ValueError, UnicodeDecodeError):
                continue
            seen += 1
            for key, ok in _flatten(data).items():
                if ok:
                    filled[key] += 1
            if seen >= sample:
                break
    return seen, dict(filled)


# ----------------------------------------------------------------------------------------------------- RTTTL
def read_library(path: str) -> List[Dict[str, Any]]:
    with gzip.open(path, "rt", encoding="utf-8") as fh:
        return [json.loads(line) for line in fh if line.strip()]


class PdmxIndex:
    """Индексы партитур по нормализованному названию и по токенам (для блокировки нечёткого поиска)."""

    def __init__(self, rows: Sequence[Mapping[str, str]]) -> None:
        self.rows = rows
        self.by_title: Dict[str, List[int]] = defaultdict(list)
        self.by_token: Dict[str, List[int]] = defaultdict(list)
        self.title_tokens: List[Tuple[str, ...]] = []
        self.people: List[Set[str]] = []
        for i, r in enumerate(rows):
            names = {norm(r.get("song_name")), norm(r.get("title"))} - {""}
            toks: Set[str] = set()
            for nm in names:
                self.by_title[nm].append(i)
                toks.update(t for t in nm.split() if t not in STOP)
            self.title_tokens.append(tuple(sorted(toks)))
            for t in toks:
                self.by_token[t].append(i)
            self.people.append(set(tokens(r.get("artist_name", ""))) | set(tokens(r.get("composer_name", ""))))
        self.df = {t: len(v) for t, v in self.by_token.items()}

    def candidates(self, toks: Sequence[str]) -> List[int]:
        rare = sorted((t for t in toks if t in self.df and self.df[t] <= RARE_DF), key=lambda t: self.df[t])[:2]
        out: List[int] = []
        for t in rare:
            out.extend(self.by_token[t])
            if len(out) >= MAX_CANDIDATES:
                break
        return out[:MAX_CANDIDATES]


def match_record(rec: Mapping[str, Any], index: PdmxIndex) -> Dict[str, Any]:
    """Уровень сопоставления записи RTTTL с PDMX: ``exact_artist`` > ``exact`` > ``fuzzy`` > ``none``."""
    title = norm(rec.get("title") or rec.get("name"))
    toks = tuple(t for t in title.split() if t not in STOP)
    artist = set(tokens(rec.get("artist") or ""))
    result: Dict[str, Any] = {"name": rec.get("name"), "title": rec.get("title"), "artist": rec.get("artist"),
                              "level": "none", "pdmx": None, "score": 0.0}
    if not toks or title in GENERIC_TITLES:
        result["level"] = "generic"
        return result
    exact = index.by_title.get(title, [])
    if exact:
        with_artist = [i for i in exact if artist and artist & index.people[i]]
        pick = with_artist[0] if with_artist else exact[0]
        result.update(level="exact_artist" if with_artist else "exact", pdmx=pick, score=1.0,
                      n_exact=len(exact))
        return result
    best: Tuple[float, Optional[int]] = (0.0, None)
    tset = set(toks)
    for i in index.candidates(toks):
        other = set(index.title_tokens[i])
        if not other:
            continue
        small, big = (tset, other) if len(tset) <= len(other) else (other, tset)
        if small <= big and len(small) >= 2 and len(big) - len(small) <= CONTAIN_SLACK:
            score = 0.95
        else:
            sm = difflib.SequenceMatcher(None, " ".join(toks), " ".join(index.title_tokens[i]))
            if sm.real_quick_ratio() < FUZZY_MIN or sm.quick_ratio() < FUZZY_MIN:
                continue
            score = sm.ratio()
        if score > best[0]:
            best = (score, i)
    if best[1] is not None and best[0] >= FUZZY_MIN:
        result.update(level="fuzzy", pdmx=best[1], score=round(best[0], 3))
    return result


def coverage(lib: Sequence[Mapping[str, Any]], index: PdmxIndex, nlc_mask: Sequence[bool]) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    matches = [match_record(rec, index) for rec in lib]
    for m, rec in zip(matches, lib):
        m["tags"] = [t for t in (rec.get("tags") or []) if not t.startswith("picaxe:")]
        if m["pdmx"] is not None:
            r = index.rows[m["pdmx"]]
            m["pdmx_title"] = r.get("title")
            m["pdmx_composer"] = r.get("composer_name")
            m["pdmx_genres"] = r.get("genres")
            m["pdmx_rating"] = r.get("rating")
            m["pdmx_nlc"] = bool(nlc_mask[m["pdmx"]])

    def summary(sub: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
        sub = list(sub)
        levels = Counter(m["level"] for m in sub)
        return {"n": len(sub), **{lv: levels.get(lv, 0) for lv in ("exact_artist", "exact", "fuzzy", "generic", "none")},
                "any": sum(levels[lv] for lv in ("exact_artist", "exact", "fuzzy")),
                "any_nlc": sum(1 for m in sub if m["pdmx"] is not None and m.get("pdmx_nlc"))}

    stats: Dict[str, Any] = {"all": summary(matches)}
    for tag in TAGS:
        stats[f"tag:{tag}"] = summary(m for m in matches if tag in m["tags"])
    stats["with_artist"] = summary(m for m in matches if (m.get("artist") or "").strip())
    stats["no_artist"] = summary(m for m in matches if not (m.get("artist") or "").strip())
    for theme, keys in SHIFU_THEMES.items():
        stats[f"theme:{theme}"] = summary(
            m for m in matches if any(k in norm(f"{m.get('title')} {m.get('artist')}") for k in keys))
    return matches, stats


def shifu_exact_rows(rows: Sequence[Mapping[str, str]], limit: int = 8) -> Dict[str, Tuple[int, int, List[str]]]:
    """Целое слово/фраза в названии+композиторе: всего, в NLC, первые строки с лицензией и флагом конфликта."""
    hay = [norm(f"{r.get('title')} {r.get('song_name')} {r.get('composer_name')}") for r in rows]
    out: Dict[str, Tuple[int, int, List[str]]] = {}
    for key in SHIFU_EXACT:
        pat = re.compile(rf"\b{re.escape(key)}\b")
        hits = [i for i, h in enumerate(hay) if pat.search(h)]
        nlc = [i for i in hits if rows[i].get(NLC) == "True"]
        sample = [f"{rows[i].get('title', '')[:60]} — {rows[i].get('composer_name', '')[:25]} "
                  f"[{rows[i].get('license')}, conflict={rows[i].get('license_conflict')}, r={rows[i].get('rating')}/"
                  f"{rows[i].get('n_ratings')}]" for i in hits[:limit]]
        out[key] = (len(hits), len(nlc), sample)
    return out


def shifu_in_pdmx(rows: Sequence[Mapping[str, str]], nlc_mask: Sequence[bool]) -> Dict[str, Tuple[int, int, List[str]]]:
    hay = [norm(f"{r.get('song_name')} {r.get('title')} {r.get('composer_name')} {r.get('artist_name')}") for r in rows]
    out: Dict[str, Tuple[int, int, List[str]]] = {}
    for theme, keys in SHIFU_THEMES.items():
        hits = [i for i, h in enumerate(hay) if any(k in h for k in keys)]
        nlc = [i for i in hits if nlc_mask[i]]
        sample = [f"{rows[i].get('title')} [{rows[i].get('license')}; r={rows[i].get('rating')}]" for i in nlc[:3]]
        out[theme] = (len(hits), len(nlc), sample)
    return out


# ----------------------------------------------------------------------------------------------------- отчёт
def pct(x: float) -> str:
    return f"{100 * x:.1f} %"


def write_report(out: str, args: argparse.Namespace, columns: Sequence[str], rows: Sequence[Mapping[str, str]],
                 nlc_rows: Sequence[Mapping[str, str]], meta: Optional[Tuple[int, Dict[str, int]]],
                 cov: Optional[Dict[str, Any]], matches: Sequence[Dict[str, Any]], shifu: Dict[str, Tuple[int, int, List[str]]], elapsed: float) -> None:
    lines: List[str] = []
    w = lines.append
    w("# Замер PDMX для ADR-0155 — сырой вывод `scripts/music/pdmx_coverage.py`\n")
    w(f"Сгенерировано {time.strftime('%Y-%m-%d %H:%M UTC', time.gmtime())}; CSV `{args.csv}`; время {elapsed:.0f} с.\n")
    w(f"Строк в PDMX.csv: **{len(rows)}**; в `{NLC}`: **{len(nlc_rows)}** ({pct(len(nlc_rows) / max(1, len(rows)))}).\n")
    w("## 1. Колонки PDMX.csv и заполненность (не NA/пусто)\n")
    w("| колонка | все | no_license_conflict |\n|---|---|---|")
    fa, fn = fill_rates(columns, rows), fill_rates(columns, nlc_rows)
    for c in columns:
        w(f"| `{c}` | {pct(fa[c])} | {pct(fn[c])} |")
    w("")
    for col, split in (("genres", ","), ("license", None), ("tags", ","), ("composer_name", None)):
        w(f"### Топ `{col}` в no_license_conflict\n")
        w("| значение | записей |\n|---|---|")
        for v, n in top_values(nlc_rows, col, 20, split):
            w(f"| {v.strip()} | {n} |")
        w("")
    rated = [r for r in nlc_rows if not is_na(r.get("n_ratings")) and float(r["n_ratings"]) >= 3
             and not is_na(r.get("rating")) and float(r["rating"]) >= 4.0]
    w(f"В no_license_conflict с рейтингом ≥ 4 и ≥ 3 оценок (порог В2 ADR-0154): **{len(rated)}**.\n")
    if meta:
        seen, filled = meta
        w(f"## 2. Ключи metadata.tar.gz (выборка {seen} JSON)\n")
        w("| ключ | заполнено |\n|---|---|")
        shown = [(k, n) for k, n in filled.items() if not k.startswith("score.") and "_links" not in k
                 and not k.startswith("data.paywall")]
        for k, n in sorted(shown, key=lambda kv: (-kv[1], kv[0]))[:110]:
            w(f"| `{k}` | {pct(n / max(1, seen))} |")
        w("")
    if cov:
        w("## 3. Покрытие RTTTL-библиотеки партитурами PDMX\n")
        w("Уровни: `exact_artist` — нормализованное название совпало и артист/композитор пересёкся; `exact` — только "
          "название; `fuzzy` — вложение токенов или difflib ≥ %.2f; `generic` — название без значимых слов («Theme»); "
          "`any_nlc` — найденная партитура в no_license_conflict.\n" % FUZZY_MIN)
        w("| срез | n | exact_artist | exact | fuzzy | generic | none | any | any_nlc | any % |\n|---|---|---|---|---|---|---|---|---|---|")
        for key, s in cov.items():
            w(f"| {key} | {s['n']} | {s['exact_artist']} | {s['exact']} | {s['fuzzy']} | {s['generic']} | {s['none']} "
              f"| {s['any']} | {s['any_nlc']} | {pct(s['any'] / max(1, s['n']))} |")
        w("")
    if matches:
        w("## 3a. Выборка сопоставлений для ручной проверки точности (сид 20261006, по 20 на уровень)\n")
        w("| уровень | запись RTTTL | → партитура PDMX | жанры | NLC |\n|---|---|---|---|---|")
        rng = random.Random(20261006)
        for lv in ("exact_artist", "exact", "fuzzy"):
            sub = [m for m in matches if m["level"] == lv]
            for m in rng.sample(sub, min(20, len(sub))):
                w(f"| {lv} | `{m['name']}` {m.get('title')} / {m.get('artist')} | {m.get('pdmx_title')} / "
                  f"{m.get('pdmx_composer')} | {m.get('pdmx_genres')} | {m.get('pdmx_nlc')} |")
        w("")
    w("## 4. Темы Шифу напрямую в PDMX (подстрока в названии/композиторе/исполнителе)\n")
    w("| тема | партитур всего | в no_license_conflict | примеры (NLC) |\n|---|---|---|---|")
    for theme, (n_all, n_nlc, sample) in shifu.items():
        w(f"| {theme} | {n_all} | {n_nlc} | {'; '.join(sample)} |")
    w("")
    w("## 4a. Целое слово в названии/композиторе: лицензии и конфликт (кто реально лежит в no_license_conflict)\n")
    w("| фраза | всего | NLC | первые строки |\n|---|---|---|---|")
    for key, (n_all, n_nlc, sample) in shifu_exact_rows(rows).items():
        w(f"| {key} | {n_all} | {n_nlc} | {'<br>'.join(sample)} |")
    w("")
    w("## 5. Лицензии PDMX\n")
    w(f"`license` по всем строкам: {top_values(rows, 'license', 5)}; `license_conflict`: {top_values(rows, 'license_conflict', 3)}; "
      f"в NLC `is_original`: {top_values(nlc_rows, 'is_original', 3)}; NLC ∩ `subset:deduplicated`: "
      f"{sum(1 for r in nlc_rows if r.get('subset:deduplicated') == 'True')}; NLC ∩ `subset:rated_deduplicated`: "
      f"{sum(1 for r in nlc_rows if r.get('subset:rated_deduplicated') == 'True')}.\n")
    with open(out, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--metadata", help="metadata.tar.gz (опционально)")
    ap.add_argument("--meta-sample", type=int, default=20000)
    ap.add_argument("--lib", help="rtttl_melodies.jsonl.gz (опционально)")
    ap.add_argument("--out", default="pdmx_coverage.md")
    ap.add_argument("--matches", default="pdmx_matches.jsonl")
    args = ap.parse_args(argv)
    t0 = time.time()
    columns, rows = read_csv(args.csv)
    nlc_mask = [r.get(NLC) == "True" for r in rows]
    nlc_rows = [r for r, ok in zip(rows, nlc_mask) if ok]
    print(f"csv: {len(rows)} rows, nlc {len(nlc_rows)}, {time.time() - t0:.0f}s", file=sys.stderr)
    meta = metadata_keys(args.metadata, args.meta_sample) if args.metadata else None
    if meta:
        print(f"metadata: {meta[0]} files, {time.time() - t0:.0f}s", file=sys.stderr)
    cov, matches = None, []
    if args.lib:
        lib = read_library(args.lib)
        index = PdmxIndex(rows)
        matches, cov = coverage(lib, index, nlc_mask)
        with open(args.matches, "w", encoding="utf-8") as fh:
            for m in matches:
                fh.write(json.dumps(m, ensure_ascii=False) + "\n")
        print(f"coverage: {cov['all']}, {time.time() - t0:.0f}s", file=sys.stderr)
    shifu = shifu_in_pdmx(rows, nlc_mask)
    write_report(args.out, args, columns, rows, nlc_rows, meta, cov, matches, shifu, time.time() - t0)
    print(f"report: {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
