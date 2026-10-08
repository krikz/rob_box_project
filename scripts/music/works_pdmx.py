#!/usr/bin/env python3
"""ADR-0155 K-3: связать произведения реестра (``works``) с партитурами пака (``score_index``) и показать, что вышло.

Офлайн, без сети. Вход — RTTTL-библиотека (``--db`` с таблицей ``rtttl_melodies`` или архив ``--archive``, по
умолчанию — архив пакета) и индекс партитур пака (``--scores``: ``/opt/rob_box/scores/score_index.db`` с робота,
открывается только на чтение). Выход — реестр в SQLite ``--db`` (таблицы ``works``/``work_sources``/``work_facts``,
ADR-0155 В3; источники ``pdmx:<id>`` — предложения, ``confirmed=0``, В6) и текстовый отчёт: связи по уровням,
лицензии, стоп-список и темы Шифу → партитуры (прямой поиск по индексу, как ищет робот, и через произведения).

Темы разрешаются ``engine.search.part_query`` — той же функцией, что у сета на роботе (#3512); произведения по
теме ищутся тем же ``score_library.ScoreIndex``, что партитуры, — второго разбора темы нет.

В git — только этот скрипт и отчёт с числами (ADR-0155 В1 «(в)»): ни реестр, ни индекс, ни партитуры.

Пример::

    python scripts/music/works_pdmx.py --db registry.sqlite --scores score_index.db
"""

from __future__ import annotations

import argparse
import sqlite3
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from rob_box_music import knowledge as kn
from rob_box_music import works
from rob_box_music.material import license_usable

#: Темы из запросов товарища Шифу (бриф 06.10, ADR-0155 §2.3, поручение K-3).
SHIFU_THEMES: Tuple[str, ...] = ("Григ", "Моцарт", "Бах", "Чайковский", "Марио", "Тетрис", "Гарри Поттер",
                                 "Интерстеллар", "кино", "Щелкунчик", "Зельда", "Звёздные войны")
#: Сколько id партитур показать на тему (лучшие по рейтингу).
TOP_IDS = 3
_ALL = 10 ** 6


def _score_rows(conn: sqlite3.Connection) -> List[Dict[str, Any]]:
    cols = ("material_id", "title", "composer", "genres", "rating", "n_ratings", "license")
    return [dict(zip(cols, r)) for r in conn.execute(f"SELECT {','.join(cols)} FROM scores.score_index")]


def _work_rows(conn: sqlite3.Connection) -> List[Dict[str, Any]]:
    """Произведения как строки индекса поиска: название, автор (исполнитель + композитор), жанры PDMX по метке
    каталога (``knowledge.SCORE_GENRES``: тег ``movie`` → ``soundtrack``)."""
    genre = dict(conn.execute("SELECT work_id, value FROM work_facts WHERE field='genre'"))
    return [{"material_id": wid, "title": title, "composer": f"{artist} {composer}".strip(),
             "genres": "-".join(kn.SCORE_GENRES.get(genre.get(wid, ""), ())), "rating": 0.0}
            for wid, title, artist, composer in conn.execute("SELECT work_id, title, artist, composer FROM works")]


def _licenses(rows: Sequence[Mapping[str, Any]]) -> str:
    return ", ".join(f"{k} {n}" for k, n in Counter(str(r["license"]) for r in rows).most_common()) or "—"


def theme_report(conn: sqlite3.Connection, themes: Sequence[str]) -> str:
    """Тема → партитуры прямым поиском по индексу пака и через произведения реестра (связи ``work_sources``)."""
    from rob_box_mcp_tools.engine.score_library import ScoreIndex
    from rob_box_mcp_tools.engine.search import ThemeQuery, part_query

    scores = _score_rows(conn)
    by_id = {r["material_id"]: r for r in scores}
    score_index, work_index = ScoreIndex(scores), ScoreIndex(_work_rows(conn))
    links: Dict[str, List[Tuple[str, str]]] = {}
    for wid, mid, level in conn.execute("SELECT work_id, material_id, link_level FROM work_sources WHERE kind='pdmx'"):
        links.setdefault(wid, []).append((mid, level))
    lines = []
    for theme in themes:
        query = ThemeQuery(theme, (part_query(None, theme),))
        direct = score_index.search(query, _ALL)
        wids = work_index.search(query, _ALL)
        via = {mid: level for wid in wids for mid, level in links.get(wid, ())}
        levels = ", ".join(f"{k} {n}" for k, n in sorted(Counter(via.values()).items())) or "—"
        top = sorted(via, key=lambda m: -float(by_id[m]["rating"] or 0.0))[:TOP_IDS]
        best = "; ".join(f"{m} [{via[m]}, {by_id[m]['license']}]" for m in top) or "—"
        lines += [f"«{theme}» строки {list(query.parts[0].strings())}",
                  f"  прямой поиск партитур: {len(direct)}; лицензии: {_licenses([by_id[m] for m in direct])}",
                  f"  произведений реестра {len(wids)}, из них с партитурой {sum(1 for x in wids if x in links)}; "
                  f"партитур через произведения {len(via)} ({levels}); лицензии: {_licenses([by_id[m] for m in via])}"
                  f"; в прямом поиске из них {len(set(via) & set(direct))}",
                  f"  лучшие через произведения: {best}"]
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--db", required=True, help="SQLite реестра (с rtttl_melodies — записи берутся из неё)")
    ap.add_argument("--archive", help="jsonl.gz архив RTTTL, если в --db нет rtttl_melodies (умолчание — из пакета)")
    ap.add_argument("--scores", required=True, help="score_index.db пака партитур (только чтение)")
    ap.add_argument("--themes", nargs="*", default=list(SHIFU_THEMES), help="темы отчёта")
    args = ap.parse_args(argv)
    conn = sqlite3.connect(f"file:{Path(args.db).as_posix()}", uri=True)  # uri — для ATTACH индекса ``mode=ro``
    has_lib = conn.execute("SELECT 1 FROM sqlite_master WHERE name='rtttl_melodies'").fetchone()
    records = works.read_db_records(conn) if has_lib else works.read_archive_records(
        args.archive or works._default_archive())
    conn.row_factory = None
    built = works.build_works(records)
    works.write_registry(conn, built)
    conn.execute("ATTACH DATABASE ? AS scores", (f"file:{Path(args.scores).as_posix()}?mode=ro",))
    started = time.perf_counter()
    n = works.link_score_sources(conn, "scores")
    took = time.perf_counter() - started
    total = conn.execute("SELECT COUNT(*) FROM scores.score_index").fetchone()[0]
    bad = sum(1 for (lic,) in conn.execute("SELECT license FROM scores.score_index") if not license_usable(lic))
    print(f"записей RTTTL {len(records)} → произведений {len(built)}; партитур в индексе {total}, "
          f"с непригодной лицензией (не связываются) {bad}; связей {n} за {took:.1f} с")
    print(works.score_links_report(conn))
    print(theme_report(conn, args.themes))
    return 0


if __name__ == "__main__":
    sys.exit(main())
