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
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List

_REPO_ROOT = Path(__file__).resolve().parents[2]
_MCP_PKG = _REPO_ROOT / "src" / "rob_box_mcp_tools"
if str(_MCP_PKG) not in sys.path:
    sys.path.insert(0, str(_MCP_PKG))

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


if __name__ == "__main__":
    main()
