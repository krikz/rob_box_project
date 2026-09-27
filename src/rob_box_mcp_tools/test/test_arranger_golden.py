"""Golden-снимок Renardo-кода аранжировщика на эталонах архива (ADR-0132, PR-1).

ЧТО ЭТО. Страховка РЕФАКТОРИНГА, а не цель тюнинга. Снимок снят ДО
первых правок ADR-0132 (партитура, ``detect_key_ranked``, записи
``decisions``) текущим путём ``compose_music(name=...)``:
``melody_to_compose_params`` → ``spec_from_flat(harmony=...)`` → ``render``.
Тест требует байт-в-байт того же кода: PR-1..PR-3 ADR-0132 обязаны делать
музыку ПРОЗРАЧНОЙ, не меняя её.

ЧЕГО ЭТО НЕ ЗНАЧИТ. Снимок не утверждает, что музыка «правильная» — он
фиксирует сегодняшнее поведение со всеми его ошибками. Подгонять код под
снимок или снимок под «звучит лучше на теме X» нельзя (ADR-0132 §8: жалоба
на песню X чинится ручкой или пресетом). Перезаписывать снимок можно только
при СОЗНАТЕЛЬНОЙ смене дефолта по ADR, с отчётом по всей выборке в PR:

    python src/rob_box_mcp_tools/test/test_arranger_golden.py --regen

Вход заморожен вместе с выходом: в фикстуре лежит сама RTTTL-строка по
ключу архива, а не запрос к ``RtttlLibrary.get()`` — ранжирование поиска
меняется (#2896/#2900) и не должно ронять этот тест.
"""

from __future__ import annotations

import gzip
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

import pytest

_FIXTURE = Path(__file__).parent / "fixtures" / "arranger_golden.json"

#: Ключи архива ``data/rtttl_melodies.jsonl.gz`` (поле ``name``). Темы, на
#: которых за сентябрь 2026 правили эвристики (#2839/#2840/#2873/#2876/
#: #2896), плюс типовые жанры библиотеки: марши, классика, чиптюн, кино.
GOLDEN_KEYS = (
    "national_2", "hallofth_2", "hallofth", "stilldre_2", "nextepis_3",
    "terminat", "supermar_4", "tetris", "furelise", "odetojoy",
    "imperial", "starwars", "starwars_imp", "pinkpant", "jamesbon_2",
    "mission", "zelda", "axelf", "jinglebe_6", "happybir_5",
    "rudolpht", "rudolpht_2", "mariobro", "nokiatun", "simpsons",
    "indianaj", "xfiles", "entertai", "pacman", "popcorn",
    "macgyver", "knightri", "ghostbus", "adamsfam", "greensle",
    "carmen", "toccata", "soviethy", "mozart40", "auldlang",
    "smokeont", "finalcou", "takeonme", "godfather",
)

#: Сочетания синтов/форм. Подобраны так, чтобы задеть разные ветки:
#: синты со сдвигом высоты (SYNTH_SEMITONE_SHIFT: blip −12, moogbass +24,
#: subbass +19), выключенный второй голос и октавы, жанровые каркасы
#: ударных, повтор/конечная форма, свинг.
COMBOS: Dict[str, Dict[str, Any]] = {
    "arc_blip": {
        "drum_style": "auto",
        "flat": dict(
            form="arc", lead_synth="blip", bass_synth="moogbass",
            pad_synth="strings", repeat=False,
        ),
    },
    "verse_pluck_march": {
        "drum_style": "march",
        "flat": dict(
            form="verse_chorus", lead_synth="pluck", bass_synth="bass",
            pad_synth="warmpad", counter_synth="off", theme_octaves=False,
            repeat=True, drums_sample=2, hats_sample=1,
        ),
    },
    "ambient_brass_halftime": {
        "drum_style": "halftime",
        "flat": dict(
            form="ambient", lead_synth="imperialbrass", bass_synth="subbass",
            pad_synth="ambi", counter_synth="pianovel", swing=0.1,
            repeat=False,
        ),
    },
}


def render_case(rtttl: str, combo: str) -> str:
    """Тот же путь, что ``ComposeMusicTool.execute`` при ``name=``."""
    from rob_box_mcp_tools.core.arranger import render, spec_from_flat
    from rob_box_mcp_tools.core.rtttl_compose import (
        melody_to_compose_params,
        rtttl_to_melody,
    )

    cfg = COMBOS[combo]
    params = melody_to_compose_params(
        rtttl_to_melody(rtttl), drum_style=cfg["drum_style"]
    )
    spec = spec_from_flat(
        harmony=params["harmony"],
        bpm=float(params["bpm"]),
        root=str(params["root"]),
        scale=str(params["scale"]),
        lead_midi=str(params["lead_midi"]),
        lead_dur=str(params["lead_dur"]),
        **cfg["flat"],
    )
    return render(spec)


def _load_fixture() -> List[Dict[str, str]]:
    return json.loads(_FIXTURE.read_text(encoding="utf-8"))["cases"]


def _archive_rows() -> Dict[str, Dict[str, Any]]:
    archive = (
        Path(__file__).resolve().parents[1]
        / "rob_box_mcp_tools" / "data" / "rtttl_melodies.jsonl.gz"
    )
    rows: Dict[str, Dict[str, Any]] = {}
    with gzip.open(archive, "rt", encoding="utf-8") as fh:
        for line in fh:
            row = json.loads(line)
            rows.setdefault(row["name"], row)
    return rows


def regenerate() -> None:
    """Перезаписать снимок. Только при сознательной смене дефолта (ADR-0132)."""
    rows = _archive_rows()
    cases = []
    for key in GOLDEN_KEYS:
        row = rows[key]
        for combo in COMBOS:
            cases.append({
                "key": key,
                "title": row.get("title", ""),
                "combo": combo,
                "rtttl": row["rtttl"],
                "code": render_case(row["rtttl"], combo),
            })
    payload = {
        "_doc": (
            "ADR-0132 golden: страховка рефакторинга, НЕ цель тюнинга. "
            "Перезапись — только при сознательной смене дефолта по ADR."
        ),
        "cases": cases,
    }
    _FIXTURE.write_text(
        json.dumps(payload, ensure_ascii=False, indent=0) + "\n",
        encoding="utf-8",
    )
    print(f"wrote {len(cases)} cases -> {_FIXTURE}")


_CASES = _load_fixture() if _FIXTURE.exists() else []


def test_golden_fixture_covers_all_keys_and_combos():
    """Снимок полный: каждая эталонная тема × каждое сочетание."""
    got = {(c["key"], c["combo"]) for c in _CASES}
    want = {(k, c) for k in GOLDEN_KEYS for c in COMBOS}
    assert got == want
    assert len(GOLDEN_KEYS) >= 40


@pytest.mark.parametrize(
    "case", _CASES, ids=[f"{c['key']}-{c['combo']}" for c in _CASES]
)
def test_render_is_byte_identical_to_golden(case):
    """Код аранжировщика совпадает со снимком байт-в-байт."""
    assert render_case(case["rtttl"], case["combo"]) == case["code"]


if __name__ == "__main__":
    if "--regen" in sys.argv:
        regenerate()
