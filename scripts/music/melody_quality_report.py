#!/usr/bin/env python
"""Аудит архива RTTTL общим детектором качества мелодии (issue #2959).

Товарищ Шифу попросил после разбора гимна России в живой сессии 24.09.2026
(RTTTL ``national_2`` записан с октавной ошибкой в кульминации фразы и с
ритмом в половинных длительностях): не чинить руками одну запись, а
построить ОБЩИЙ детектор такого класса ошибок и прогнать по всему архиву
(``core.rtttl_compose.detect_contour_breaks``/``detect_half_time_risk``, см.
их докстринги — там же обоснование порогов и разбор ложного срабатывания,
пойманного на том же гимне при первой версии детектора).

Детектор смотрит на КАЖДУЮ запись одинаково — нет списков имён/тегов,
которые бы решали результат: единственный вход — сама мелодия (MIDI +
ритм), разобранная ``core.rtttl.parse_rtttl``.

НЕ ПАДАЕТ и не гейтит CI — это отчёт для PR (ADR-0018/AGENTS.md: сырой
вывод, не «на глаз»). Печатает:

  1. сколько записей архива детектор пометил (разрывы контура — исправлено
     автоматически / только помечено; риск half-time — только помечено) и
     почему;
  2. по 5-10 примеров каждой категории (имя, title, что нашли).

Запуск (из корня репозитория)::

    python scripts/music/melody_quality_report.py
    python scripts/music/melody_quality_report.py --engines   # + сравнение v1/v2 (ADR-0149 A15)

``--engines`` (ADR-0149 PR-11, приёмка A15 «не хуже v1»): по каждой записи архива — поиск по её title
(v1: ``lookup_melody`` + правило ``named_play.melody_hit``; v2: ``engine.classic.find_record``), построение
(v1: ``melody_to_compose_params``; v2: песня ``arrange.song`` + ``render``) и совпадение нот мелодии v2 с
темой v1 (``lead_midi``). Печатает счётчики и примеры расхождений.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _pkg in ("rob_box_mcp_tools", "rob_box_music", "rob_box_voice"):
    if str(_REPO_ROOT / "src" / _pkg) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / "src" / _pkg))

from rob_box_mcp_tools.core.rtttl_compose import (  # noqa: E402
    detect_contour_breaks,
    detect_half_time_risk,
    rtttl_to_melody,
)
from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary  # noqa: E402

_EXAMPLES_PER_CATEGORY = 10


def main() -> None:
    lib = RtttlLibrary(db_path=str(_REPO_ROOT / "_melody_quality_report.db"))
    total = lib.total()
    print(f"Архив: {total} записей ({RtttlLibrary.__module__})")

    fixed_examples: List[str] = []
    flagged_only_examples: List[str] = []
    half_time_examples: List[str] = []
    n_fixed = 0
    n_flagged_only = 0
    n_half_time = 0
    n_errors = 0

    # Прямой обход БД, а не lib.search()/get() — нужны ВСЕ записи, а не
    # top-N по текстовому скору.
    cursor = lib._conn.execute(  # noqa: SLF001 -- отчёт, не библиотечный код
        "SELECT name, title, rtttl FROM rtttl_melodies"
    )
    for row in cursor:
        name, title, rtttl = row["name"], row["title"], row["rtttl"]
        try:
            melody = rtttl_to_melody(rtttl)
        except Exception:  # noqa: BLE001 -- отчёт не падает на кривой строке
            n_errors += 1
            continue

        breaks = detect_contour_breaks(melody)
        fixed = [b for b in breaks if b.auto_fixable]
        flagged = [b for b in breaks if not b.auto_fixable]
        risk = detect_half_time_risk(melody)

        if fixed:
            n_fixed += 1
            if len(fixed_examples) < _EXAMPLES_PER_CATEGORY:
                detail = ", ".join(
                    f"idx={b.index} {b.note_before}->{b.note_at} "
                    f"({b.interval:+d} п/т, фикс {b.octave_shift:+d})"
                    for b in fixed
                )
                fixed_examples.append(f"{name!r} ({title}): {detail}")
        if flagged:
            n_flagged_only += 1
            if len(flagged_only_examples) < _EXAMPLES_PER_CATEGORY:
                detail = ", ".join(
                    f"idx={b.index} {b.note_before}->{b.note_at} "
                    f"({b.interval:+d} п/т)"
                    for b in flagged
                )
                flagged_only_examples.append(f"{name!r} ({title}): {detail}")
        if risk:
            n_half_time += 1
            if len(half_time_examples) < _EXAMPLES_PER_CATEGORY:
                half_time_examples.append(f"{name!r} ({title}): {risk}")

    print()
    print(f"Разрывы контура — исправлено автоматически: {n_fixed} записей")
    for line in fixed_examples:
        print(f"  - {line}")
    print()
    print(
        f"Разрывы контура — только помечено (правка неоднозначна): "
        f"{n_flagged_only} записей"
    )
    for line in flagged_only_examples:
        print(f"  - {line}")
    print()
    print(
        f"Риск half-time (плотный ритм без долгих нот на быстром темпе): "
        f"{n_half_time} записей"
    )
    for line in half_time_examples:
        print(f"  - {line}")
    print()
    if n_errors:
        print(f"Не разобрано (кривой RTTTL, не детектор): {n_errors} записей")


def _v1_search(lib: RtttlLibrary, query: str) -> object:
    from rob_box_mcp_tools.core.rtttl_library import match_info
    from rob_box_voice.core.named_play import melody_hit

    rec = lib.get(query)  # lookup_melody: _resolve_melody_with_candidate(name) → get
    if rec is None:
        return None
    hit, _reason = melody_hit({"name": rec.get("name"), "match": match_info(lib, rec, query)})
    return hit.key if hit else None


def _v2_lead(rec: dict) -> List[int]:
    from rob_box_mcp_tools.engine.classic import song_material
    from rob_box_music.arrange.song import song_track
    from rob_box_music.render.renardo import render

    track = song_track(song_material(rec, rec["name"]), seed=0)
    render(track, "A")
    verse = track.form.sections[0].bars * 4
    return [p.midi for p in sorted(track.parts["lead"].pitches, key=lambda p: p.beat) if p.beat < verse]


def compare_engines(lib: RtttlLibrary) -> None:
    """ADR-0149 A15: поиск, построение и ноты мелодии v2 против v1 по всему архиву."""
    from rob_box_mcp_tools.core.rtttl_compose import melody_to_compose_params
    from rob_box_mcp_tools.engine.classic import find_record

    stats = {k: 0 for k in ("rows", "v1_found", "v2_found", "search_diff", "v1_built", "v2_built", "lead_diff")}
    v2_fail: List[str] = []
    for row in lib._conn.execute("SELECT name, title, rtttl FROM rtttl_melodies").fetchall():  # noqa: SLF001
        rec = {"name": row["name"], "title": row["title"], "rtttl": row["rtttl"]}
        stats["rows"] += 1
        query = str(row["title"] or row["name"])
        v1_key = _v1_search(lib, query)
        v2_rec, _reason = find_record(lib, query)
        v2_key = v2_rec.get("name") if v2_rec else None
        stats["v1_found"] += v1_key is not None
        stats["v2_found"] += v2_key is not None
        stats["search_diff"] += v1_key != v2_key
        try:
            v1 = [m for m, _d in melody_to_compose_params(rtttl_to_melody(rec["rtttl"]))["harmony"].lead if m]
        except Exception:  # noqa: BLE001 -- v1 тоже не сыграл бы
            continue
        stats["v1_built"] += 1
        try:
            lead = _v2_lead(rec)
        except Exception as exc:  # noqa: BLE001 -- отчёт, не падает
            if len(v2_fail) < _EXAMPLES_PER_CATEGORY:
                v2_fail.append(f"{rec['name']!r}: {type(exc).__name__}: {exc}")
            continue
        stats["v2_built"] += 1
        stats["lead_diff"] += lead != v1
    print("Сравнение движков (ADR-0149 A15):")
    for key, value in stats.items():
        print(f"  {key}: {value}")
    for line in v2_fail:
        print(f"  - v2 не построил {line}")


if __name__ == "__main__":
    main()
    if "--engines" in sys.argv[1:]:
        compare_engines(RtttlLibrary(db_path=str(_REPO_ROOT / "_melody_quality_report.db")))
