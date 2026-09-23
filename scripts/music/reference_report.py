#!/usr/bin/env python
"""Информационный отчёт по тональности и инвариантам аранжировщика.

ADR-0132 §7 (PR-8): точная таблица ``_KEY_REFERENCE`` и xfail-тесты
``test_known_key_misses`` (issue #2873) перестали быть гейтом CI — подгонка
весов ``detect_key`` под конкретные темы однажды уже сменила тональность у
24% архива, улучшая ровно те темы, ради которых её правили. Гейт CI теперь
— инварианты аранжировки (``test/test_arrangement_invariants.py``), а не
совпадение с конкретной тональностью конкретной песни.

Этот скрипт остаётся ГЛАЗАМИ на то же самое: печатает точность
``detect_key`` (``auto`` — текущие веса тонального центра #2873, ``profile``
— чистый Крумханзл без позиционной опоры) на:

  1. эталонных темах архива (ручная разметка по нотам записи, см.
     :data:`REFERENCE_KEYS`/:data:`KNOWN_MISSES`);
  2. детерминированной seed-выборке ~200 тем реального архива
     (:class:`rob_box_mcp_tools.core.rtttl_library.RtttlLibrary`), где
     эталона тональности нет — печатается согласие ``auto``/``profile``;

а следом — те же инварианты аранжировки, что гейтит
``test_arrangement_invariants.py``, на обеих выборках. Логика подсчёта
инвариантов сознательно ПРОДУБЛИРОВАНА (не импортируется из теста): скрипт
обязан работать самостоятельно (``python scripts/music/reference_report.py``
из корня репозитория) без зависимости от pytest-раскладки тестового пакета.
При правке порогов/комбинации синтов — правь ОБА файла (см. эту заметку
и заметку в ``test_arrangement_invariants.py``).

НЕ ПАДАЕТ и не гейтит PR. Обязателен в описании PR, который трогает
авто-константы аранжировщика (ADR-0132 §7, CONTRIBUTING.md) — приложить
полный вывод по ВСЕЙ выборке (raw, ADR-0018/AGENTS.md), не выборочные строки.

Запуск (из корня репозитория)::

    python scripts/music/reference_report.py
"""

from __future__ import annotations

import gzip
import json
import random
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Tuple

_REPO_ROOT = Path(__file__).resolve().parents[2]
_MCP_PKG = _REPO_ROOT / "src" / "rob_box_mcp_tools"
if str(_MCP_PKG) not in sys.path:
    sys.path.insert(0, str(_MCP_PKG))

from rob_box_mcp_tools.core.arranger import render, spec_from_flat  # noqa: E402
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando  # noqa: E402
from rob_box_mcp_tools.core.rtttl_compose import (  # noqa: E402
    detect_key_ranked,
    melody_to_compose_params,
    rtttl_to_melody,
)
from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary  # noqa: E402
from rob_box_mcp_tools.core.score_sheet import describe  # noqa: E402

#: Ручная разметка «тоника, лад» по нотам записи (было ``_KEY_REFERENCE``).
#: Гармонический минор засчитывается как минор — его выбирает отдельное
#: правило (вводный тон), здесь проверяется тональный центр.
REFERENCE_KEYS: Dict[str, Tuple[str, str]] = {
    "hallofth_2": ("B", "minor"), "mountainking": ("B", "minor"),
    "hallofth": ("A", "minor"), "hallofth_3": ("A", "minor"),
    "inthehal": ("C", "minor"), "national_2": ("C", "major"),
    "furelise": ("A", "minor"), "odetojoy": ("F", "major"),
    "odetojoy_2": ("D", "major"), "5thsymph": ("A#", "minor"),
    "imperial": ("D", "minor"), "tetris": ("A", "minor"),
    "tetris_2": ("A#", "minor"), "supermar_4": ("C", "major"),
    "jesujoyo": ("G", "major"), "eineklei": ("G", "major"),
    "badineri": ("B", "minor"), "mozart40": ("G", "minor"),
    "4seasons_5": ("F", "minor"), "ladonnae": ("B", "major"),
    "indianaj_4": ("C", "major"), "raidersm": ("C#", "major"),
    "james_bond": ("C", "minor"), "missionimpossi": ("C", "minor"),
    "pinkpant": ("E", "minor"), "rondoala": ("A", "minor"),
    "stilldre": ("A", "minor"), "nextepis": ("D", "minor"),
    "terminat": ("G#", "minor"), "jinglebe_2": ("F", "major"),
    "thefirst": ("C", "major"), "whitechr": ("D", "major"),
    "rudolpht_2": ("C", "major"), "rudolpht_4": ("C", "major"),
}

#: Известные промахи (было ``_KEY_KNOWN_MISSES``, раньше ``xfail(strict=True)``).
#: Разбор — см. историю ADR-0132/#2873: не решаемо единой эвристикой без
#: регрессии где-то ещё.
KNOWN_MISSES: Dict[str, Tuple[str, str]] = {
    "toccata": ("D", "minor"),
    "jinglebe_5": ("G", "major"),
}

#: Сколько тем архива брать в seed-выборку и какой сид (детерминизм отчёта).
#: Тот же сид/размер, что в ``test_arrangement_invariants.py`` — иначе
#: цифры двух отчётов не сравнить.
SAMPLE_SIZE = 200
SAMPLE_SEED = 20260924

#: Сочетание ручек «по умолчанию» — то же самое, что использует
#: ``test_arrangement_invariants.py`` для гейта (см. заметку в шапке файла).
DEFAULT_FLAT = dict(
    form="arc", lead_synth="pluck", bass_synth="bass", pad_synth="strings",
    repeat=False,
)

_ARCHIVE = _MCP_PKG / "rob_box_mcp_tools" / "data" / "rtttl_melodies.jsonl.gz"


def _archive_names() -> List[str]:
    names: List[str] = []
    seen = set()
    with gzip.open(_ARCHIVE, "rt", encoding="utf-8") as fh:
        for line in fh:
            name = json.loads(line)["name"]
            if name not in seen:
                seen.add(name)
                names.append(name)
    return names


def _family(scale: str) -> str:
    return "major" if scale == "major" else "minor"


def _detect(rtttl: str, method: str) -> Tuple[str, str]:
    melody = rtttl_to_melody(rtttl)
    midi = [note for note, _dur in melody.notes]
    dur = [d for _note, d in melody.notes]
    root, scale, _score = detect_key_ranked(midi, dur, method=method)[0]
    return root, _family(scale)


def _accuracy(library: RtttlLibrary, table: Dict[str, Tuple[str, str]],
              method: str) -> Tuple[int, int, List[str]]:
    hits = 0
    misses: List[str] = []
    for name, expected in sorted(table.items()):
        entry = library.get(name)
        if entry is None:
            misses.append(f"{name} (нет в архиве)")
            continue
        got = _detect(entry["rtttl"], method)
        if got == expected:
            hits += 1
        else:
            misses.append(f"{name}: ожидали {expected}, {method} {got}")
    return hits, len(table), misses


def _print_key_accuracy(library: RtttlLibrary) -> None:
    print("=== Точность detect_key на эталонных темах ===")
    for label, table in (("reference", REFERENCE_KEYS), ("known misses", KNOWN_MISSES)):
        for method in ("auto", "profile"):
            hits, total, misses = _accuracy(library, table, method)
            print(f"{label:>13} / {method:>7}: {hits}/{total} ({hits / total:.0%})")
            if method == "auto":
                for line in misses:
                    print(f"    промах: {line}")


def _sample_names(seed: int, size: int) -> List[str]:
    names = sorted(_archive_names())
    rnd = random.Random(seed)
    return rnd.sample(names, min(size, len(names)))


def _key_agreement(library: RtttlLibrary, names: List[str]) -> Tuple[int, int]:
    """Сколько тем сходятся auto/profile (без эталона — просто согласие)."""
    agree = total = 0
    for name in names:
        entry = library.get(name)
        if entry is None:
            continue
        total += 1
        agree += _detect(entry["rtttl"], "auto") == _detect(entry["rtttl"], "profile")
    return agree, total


def _print_sample_key_report(library: RtttlLibrary, names: List[str]) -> None:
    print(f"\n=== Seed-выборка архива (n={len(names)}, seed={SAMPLE_SEED}) ===")
    agree, total = _key_agreement(library, names)
    rate = f"{agree / total:.0%}" if total else "n/a"
    print(f"detect_key auto==profile: {agree}/{total} ({rate})")
    print("(эталона тональности на случайной выборке нет — это согласие "
          "методов, не точность; точность — только на REFERENCE_KEYS выше)")


# ---------------------------------------------------------------------------
# Инварианты аранжировки (та же схема, что test_arrangement_invariants.py)
# ---------------------------------------------------------------------------


def _render_default(rtttl: str):
    params = melody_to_compose_params(rtttl_to_melody(rtttl))
    harmony = params["harmony"]
    spec = spec_from_flat(
        harmony=harmony, bpm=float(params["bpm"]), root=str(params["root"]),
        scale=str(params["scale"]), lead_midi=str(params["lead_midi"]),
        lead_dur=str(params["lead_dur"]), **DEFAULT_FLAT,
    )
    return harmony, spec, render(spec)


def _bass_fit_ratio(harmony) -> float:
    from rob_box_mcp_tools.core.arranger import SCALE_INTERVALS, VALID_ROOTS

    root = VALID_ROOTS.index(harmony.root) if harmony.root in VALID_ROOTS else 0
    scale = {(root + i) % 12 for i in SCALE_INTERVALS.get(harmony.scale, SCALE_INTERVALS["minor"])}

    def chord_at(beat: float):
        for chord in harmony.chords:
            if chord.start <= beat < chord.start + chord.beats:
                return chord
        return harmony.chords[-1]

    cursor = total = fit = 0.0
    for note, dur in harmony.bass:
        if note is not None:
            total += dur
            chord = chord_at(cursor)
            if note % 12 in scale or note % 12 in chord.pitch_classes:
                fit += dur
        cursor += dur
    return fit / total if total else 1.0


def _lead_fraction_in_range(harmony, lo: int = 55, hi: int = 88) -> float:
    total = sum(d for n, d in harmony.lead if n is not None)
    inside = sum(d for n, d in harmony.lead if n is not None and lo <= n <= hi)
    return inside / total if total else 1.0


def _theme_invariants(library: RtttlLibrary, name: str) -> Dict[str, object]:
    entry = library.get(name)
    if entry is None:
        return {"name": name, "parse_error": "нет в архиве"}
    try:
        harmony, spec, code = _render_default(entry["rtttl"])
    except ValueError as exc:
        return {"name": name, "parse_error": str(exc)}
    sheet = describe(spec=spec, code=code, harmony=harmony)
    checks = sheet["checks"]
    sanitized = sanitize_renando(code, 0.7)
    return {
        "name": name,
        "bass_fit": _bass_fit_ratio(harmony),
        "lead_fit": _lead_fraction_in_range(harmony),
        "pad_over_bass": checks.get("pad_over_bass"),
        "pad_under_body": checks.get("pad_under_theme_body"),
        "players": len(checks.get("slots", [])),
        "sanitizer_ok": not (
            sanitized.security_error or sanitized.quality_errors or sanitized.slot_error
        ),
    }


def _print_invariant_summary(library: RtttlLibrary, names: List[str]) -> None:
    print("\n=== Инварианты аранжировки (та же выборка) ===")
    rows = [_theme_invariants(library, name) for name in names]
    parse_errors = [r for r in rows if "parse_error" in r]
    ok = [r for r in rows if "parse_error" not in r]
    print(f"тем: {len(names)}, не разобрано: {len(parse_errors)}, проверено: {len(ok)}")
    if not ok:
        return
    bass_fail = sum(1 for r in ok if r["bass_fit"] < 0.90)
    lead_fail = sum(1 for r in ok if r["lead_fit"] < 0.90)
    pad_fail = sum(1 for r in ok if r["pad_over_bass"] is not None and r["pad_over_bass"] <= 0)
    players_fail = sum(1 for r in ok if r["players"] > 6)
    sanitizer_fail = sum(1 for r in ok if not r["sanitizer_ok"])
    worst_pad_under = max(
        (r["pad_under_body"] for r in ok if r["pad_under_body"] is not None), default=None
    )
    print(f"бас в ладу/аккорде <90% по длительности: {bass_fail}/{len(ok)}")
    print(f"лид вне [55, 88] >10% по длительности: {lead_fail}/{len(ok)}")
    print(f"пэд не выше баса (min(pad) <= max(bass)): {pad_fail}/{len(ok)}")
    print(f"плееров больше 6: {players_fail}/{len(ok)}")
    print(f"не проходит renardo_sanitizer: {sanitizer_fail}/{len(ok)}")
    print(f"худшее (макс.) pad_under_theme_body: {worst_pad_under}")


def main() -> None:
    import shutil

    tmp = tempfile.mkdtemp(prefix="reference_report_")
    try:
        library = RtttlLibrary(db_path=str(Path(tmp) / "lib.db"))
        _print_key_accuracy(library)
        sample = _sample_names(SAMPLE_SEED, SAMPLE_SIZE)
        _print_sample_key_report(library, sample)
        reference_names = list(REFERENCE_KEYS) + list(KNOWN_MISSES)
        _print_invariant_summary(library, reference_names)
        _print_invariant_summary(library, sample)
    finally:
        # sqlite соединение не закрыто явно (RtttlLibrary его не отдаёт) —
        # на Windows файл ещё занят процессом, поэтому удаление best-effort.
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
