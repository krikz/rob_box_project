#!/usr/bin/env python3
"""Теги свода решений товарища Шифу (``docs/shifu-decisions.md``): проверка и оглавление.

Без зависимостей. Словарь тегов — в самом файле между ``<!-- tags:begin -->`` и ``<!-- tags:end -->``,
строками ``- `#тег` — пояснение``. Запись — якорь ``<a id="…"></a>``, заголовок ``### ДД.ММ.ГГГГ — …`` и строка
``Теги: #тег …`` (1–4 тега из словаря) первой непустой строкой под заголовком.

    python -X utf8 scripts/docs/shifu_decisions_tags.py              # проверка + сводка «тег → число записей»
    python -X utf8 scripts/docs/shifu_decisions_tags.py --write-toc  # пересобрать оглавление по тегам

Проверка падает (код 1), если у записи нет якоря или тегов, тег не из словаря, тегов больше четырёх, якорь повторяется
или оглавление между ``<!-- toc:begin -->`` и ``<!-- toc:end -->`` расходится с тегами записей.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

DEFAULT_PATH = Path(__file__).resolve().parents[2] / "docs" / "shifu-decisions.md"
MAX_TAGS = 4
TAGS_BEGIN, TAGS_END = "<!-- tags:begin -->", "<!-- tags:end -->"
TOC_BEGIN, TOC_END = "<!-- toc:begin -->", "<!-- toc:end -->"

_DICT_LINE = re.compile(r"^- `(#[^`\s]+)` — \S")
_ANCHOR = re.compile(r'^<a id="([A-Za-z0-9_-]+)"></a>$')
_HEADING = re.compile(r"^### (\d{2}\.\d{2}\.\d{4}) — (\S.*)$")
_TAGS_LINE = re.compile(r"^Теги:((?: #[^\s#]+)+)\s*$")


@dataclass(frozen=True)
class Entry:
    anchor: str
    date: str
    title: str
    tags: Tuple[str, ...]
    line: int


def _between(lines: Sequence[str], begin: str, end: str) -> Tuple[int, int]:
    """Индексы строк-маркеров ``begin`` и ``end``; ``ValueError``, если маркеров нет."""
    b = lines.index(begin)
    e = lines.index(end, b + 1)
    return b, e


def parse_dictionary(lines: Sequence[str]) -> List[str]:
    b, e = _between(lines, TAGS_BEGIN, TAGS_END)
    tags = []
    for ln in lines[b + 1 : e]:
        m = _DICT_LINE.match(ln)
        if m:
            tags.append(m.group(1))
    return tags


def parse_entries(lines: Sequence[str]) -> Tuple[List[Entry], List[str]]:
    """Записи файла и ошибки разбора (запись без якоря, без строки тегов)."""
    entries: List[Entry] = []
    errors: List[str] = []
    for i, ln in enumerate(lines):
        m = _HEADING.match(ln)
        if not m:
            continue
        where = f"строка {i + 1} «{ln[4:]}»"
        prev = lines[i - 1] if i > 0 else ""
        am = _ANCHOR.match(prev)
        if not am:
            errors.append(f'{where}: нет якоря <a id="…"></a> строкой выше')
        tags_line = next((x for x in lines[i + 1 :] if x.strip()), "")
        tm = _TAGS_LINE.match(tags_line)
        if not tm:
            errors.append(f"{where}: первая строка под заголовком должна быть «Теги: #тег …»")
        tags = tuple(tm.group(1).split()) if tm else ()
        entries.append(Entry(am.group(1) if am else "", m.group(1), m.group(2), tags, i + 1))
    return entries, errors


def check(lines: Sequence[str]) -> List[str]:
    try:
        vocab = parse_dictionary(lines)
    except ValueError:
        return [f"нет словаря тегов между {TAGS_BEGIN} и {TAGS_END}"]
    if not vocab:
        return ["словарь тегов пуст"]
    entries, errors = parse_entries(lines)
    if not entries:
        errors.append("нет ни одной записи «### ДД.ММ.ГГГГ — …»")
    seen: Dict[str, int] = {}
    for e in entries:
        where = f"строка {e.line} «{e.date} — {e.title}»"
        if e.anchor:
            if e.anchor in seen:
                errors.append(f"{where}: якорь {e.anchor} уже есть на строке {seen[e.anchor]}")
            seen[e.anchor] = e.line
        unknown = [t for t in e.tags if t not in vocab]
        if unknown:
            errors.append(f"{where}: теги не из словаря: {' '.join(unknown)}")
        if len(e.tags) > MAX_TAGS:
            errors.append(f"{where}: тегов {len(e.tags)}, можно не больше {MAX_TAGS}")
        if len(set(e.tags)) != len(e.tags):
            errors.append(f"{where}: тег повторяется")
    try:
        b, end = _between(lines, TOC_BEGIN, TOC_END)
    except ValueError:
        errors.append(f"нет оглавления между {TOC_BEGIN} и {TOC_END}")
    else:
        if not errors and list(lines[b + 1 : end]) != render_toc(vocab, entries):
            errors.append("оглавление устарело: запусти с --write-toc")
    return errors


def summary(vocab: Sequence[str], entries: Sequence[Entry]) -> Dict[str, int]:
    return {t: sum(1 for e in entries if t in e.tags) for t in vocab}


def render_toc(vocab: Sequence[str], entries: Sequence[Entry]) -> List[str]:
    out: List[str] = []
    for tag in vocab:
        tagged = [e for e in entries if tag in e.tags]
        if not tagged:
            out.append(f"- **{tag}** (0): —")
            continue
        links = ", ".join(f"[{e.date} — {e.title}](#{e.anchor})" for e in tagged)
        out.append(f"- **{tag}** ({len(tagged)}): {links}")
    return out


def write_toc(path: Path) -> None:
    lines = path.read_text(encoding="utf-8").splitlines()
    vocab = parse_dictionary(lines)
    entries, _ = parse_entries(lines)
    b, e = _between(lines, TOC_BEGIN, TOC_END)
    new = list(lines[: b + 1]) + render_toc(vocab, entries) + list(lines[e:])
    path.write_text("\n".join(new) + "\n", encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("path", nargs="?", default=str(DEFAULT_PATH))
    ap.add_argument("--write-toc", action="store_true", help="пересобрать оглавление по тегам")
    args = ap.parse_args(argv)
    path = Path(args.path)
    if args.write_toc:
        write_toc(path)
    lines = path.read_text(encoding="utf-8").splitlines()
    errors = check(lines)
    for err in errors:
        print(f"FAIL: {err}")
    if errors:
        return 1
    vocab = parse_dictionary(lines)
    entries, _ = parse_entries(lines)
    print(f"OK: записей {len(entries)}, тегов в словаре {len(vocab)}")
    for tag, n in summary(vocab, entries).items():
        print(f"  {tag}: {n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
