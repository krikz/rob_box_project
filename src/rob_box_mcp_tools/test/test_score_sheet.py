"""Партитура аранжировщика (ADR-0132, PR-1): структура, решения, проверки.

Партитура описывает то, что УЖЕ построено, и ничего не меняет — это
стережёт ``test_arranger_golden``. Здесь проверяется, что описание верно
отражает построенное: аккордов ровно по числу тактов, диапазоны партий —
те же, что в слоях спецификации (в звучащей высоте), решения и проверки
присутствуют, текст укладывается в ~1 КБ.
"""

from __future__ import annotations

import gzip
import json
import random
from pathlib import Path

import pytest

from rob_box_mcp_tools.core.arranger import (
    SYNTH_SEMITONE_SHIFT,
    render,
    spec_from_flat,
)
from rob_box_mcp_tools.core.rtttl_compose import (
    detect_key,
    detect_key_ranked,
    melody_to_compose_params,
    rtttl_to_melody,
)
from rob_box_mcp_tools.core.score_sheet import (
    analyze_melody,
    chord_name,
    chords_by_bar,
    describe,
    note_name,
)

_ARCHIVE = (
    Path(__file__).resolve().parents[1]
    / "rob_box_mcp_tools" / "data" / "rtttl_melodies.jsonl.gz"
)

_THEMES = ("national_2", "hallofth_2", "stilldre_2", "pinkpant", "terminat", "supermar_4")

#: Мягкий предел текста партитуры в символах (ADR-0132: «≤ ~1 КБ»).
_TEXT_LIMIT = 1100


@pytest.fixture(scope="module")
def archive():
    rows = {}
    with gzip.open(_ARCHIVE, "rt", encoding="utf-8") as fh:
        for line in fh:
            row = json.loads(line)
            rows.setdefault(row["name"], row)
    return rows


def _build(rtttl: str, **flat):
    params = melody_to_compose_params(rtttl_to_melody(rtttl))
    kw = dict(
        form="arc", lead_synth="blip", bass_synth="moogbass",
        pad_synth="strings", repeat=False,
    )
    kw.update(flat)
    spec = spec_from_flat(
        harmony=params["harmony"],
        bpm=float(params["bpm"]),
        root=str(params["root"]),
        scale=str(params["scale"]),
        lead_midi=str(params["lead_midi"]),
        lead_dur=str(params["lead_dur"]),
        **kw,
    )
    return params, spec, render(spec)


def _sheet(archive, key, **flat):
    params, spec, code = _build(archive[key]["rtttl"], **flat)
    sheet = describe(
        spec=spec, code=code, harmony=params["harmony"],
        prep_decisions=params["decisions"], title=archive[key]["title"],
    )
    return params, spec, code, sheet


def test_note_and_chord_names():
    assert note_name(60) == "C4"
    assert note_name(69) == "A4"
    assert note_name(None) == "—"
    assert chord_name((9, 0, 4)) == "Am"
    assert chord_name((0, 4, 7)) == "C"


@pytest.mark.parametrize("key", _THEMES)
def test_chords_listed_per_bar(archive, key):
    params, _spec, _code, sheet = _sheet(archive, key)
    harmony = params["harmony"]
    assert len(sheet["chords"]) == harmony.bars
    assert chords_by_bar(harmony) == sheet["chords"]
    assert all(sheet["chords"])


@pytest.mark.parametrize("key", _THEMES)
def test_part_ranges_are_sounding_ranges_of_spec_layers(archive, key):
    """Диапазон партии = ноты слоя минус собственный сдвиг синта."""
    _params, spec, _code, sheet = _sheet(archive, key)
    for layer in spec.layers:
        if not layer.synth:
            continue
        shift = SYNTH_SEMITONE_SHIFT.get(layer.synth, 0)
        flat = []
        for note in layer.midi:
            if isinstance(note, tuple):
                flat.extend(note)
            elif note is not None:
                flat.append(note)
        part = sheet["parts"][layer.role]
        assert part["synth"] == layer.synth
        assert part["synth_shift"] == shift
        assert (part["lo"], part["hi"]) == (min(flat) - shift, max(flat) - shift)


@pytest.mark.parametrize("key", _THEMES)
def test_decisions_and_checks_present(archive, key):
    _params, _spec, _code, sheet = _sheet(archive, key)
    for knob in (
        "tempo_fold", "lead_octave", "lead_outliers", "density", "key",
        "chords", "bass_style", "bass_approach", "pad_style", "drums",
        "counter", "theme_octaves", "form",
    ):
        assert knob in sheet["decisions"], knob
    checks = sheet["checks"]
    for name in ("slots", "pad_over_bass", "bass_out_of_key", "lead_in_range"):
        assert name in checks, name
    assert 1 <= len(checks["slots"]) <= 6
    assert "Решения по умолчанию:" in sheet["text"]
    assert "Проверки:" in sheet["text"]
    assert len(sheet["text"]) <= _TEXT_LIMIT, len(sheet["text"])


def test_key_alternatives_come_from_ranked_detection(archive):
    params, _spec, _code, sheet = _sheet(archive, "hallofth_2")
    ranked = params["decisions"]["key_ranked"]
    key = sheet["key"]
    assert (key["root"], key["scale"]) == (ranked[0][0], ranked[0][1])
    assert [(a["root"], a["scale"]) for a in key["alternatives"]] == [
        (r, s) for r, s, _sc in ranked[1:]
    ]
    assert key["gap"] == params["decisions"]["key_gap"]


def test_sparse_theme_explains_why_counter_and_octaves_are_off(archive):
    """Pink Panther — редкая тема: второй голос и удвоение выключены."""
    params, _spec, _code, sheet = _sheet(archive, "pinkpant")
    assert params["harmony"].dense is False
    assert sheet["decisions"]["counter"] == "auto→off (редкая тема)"
    assert sheet["decisions"]["theme_octaves"] == "auto→off (редкая тема)"


def test_explicitly_disabled_counter_is_reported(archive):
    _p, _s, _c, sheet = _sheet(
        archive, "national_2", counter_synth="off", theme_octaves=False
    )
    assert sheet["decisions"]["counter"] == "auto→off (counter_synth выключен)"
    assert sheet["decisions"]["theme_octaves"] == "auto→off (theme_octaves=False)"
    assert "counter" not in sheet["parts"]


def test_external_warnings_and_key_mismatch_are_reported(archive):
    params, spec, code = _build(archive["national_2"]["rtttl"])
    # Рассинхрон spec и гармонизации (с PR-2 compose_music его не создаёт,
    # но партитура-страховка обязана о нём сказать).
    spec.root, spec.scale = "D", "minor"
    sheet = describe(
        spec=spec, code=code, harmony=params["harmony"],
        prep_decisions=params["decisions"], warnings=["санитайзер: X"],
    )
    assert sheet["warnings"][0] == "санитайзер: X"
    assert any("расходятся" in w for w in sheet["warnings"])
    assert "санитайзер: X" in sheet["text"]


def test_composed_track_without_theme_still_described():
    """Сочинённый трек (без name=): партитура есть, но без темы/аккордов."""
    spec = spec_from_flat(
        bpm=100, root="A", scale="minor", form="nonexistent",
        drums="X...o...", lead_synth="pluck", lead_notes="0, 2, 4",
        pad_synth="warmpad", pad_notes="0, 2, 4",
    )
    sheet = describe(spec=spec, code=render(spec))
    assert sheet["theme"] is None
    assert sheet["chords"] == []
    assert sheet["key"]["source"] == "задана вызовом"
    assert sheet["parts"]["lead"]["degrees"] == [0, 2, 4]
    assert any("неизвестна" in w for w in sheet["warnings"])
    assert "bass" in sheet["parts"]  # _autofill_bass виден в партитуре


def test_analyze_melody_block(archive):
    params = melody_to_compose_params(rtttl_to_melody(archive["national_2"]["rtttl"]))
    analysis = analyze_melody(params)
    assert (analysis["root"], analysis["scale"]) == (params["root"], params["scale"])
    assert analysis["bars"] == params["harmony"].bars
    assert analysis["lo"] <= analysis["hi"]
    assert analysis["key_alternatives"]
    assert analysis["text"].startswith("Анализ:")


def test_detect_key_is_top_of_ranked_on_archive_sample(archive):
    """``detect_key`` = первый кандидат ``detect_key_ranked`` (сид-выборка)."""
    keys = sorted(archive)
    sample = random.Random(132).sample(keys, 200)
    for key in sample:
        melody = rtttl_to_melody(archive[key]["rtttl"])
        midi = [m for m, _d in melody.notes]
        durs = [d for _m, d in melody.notes]
        ranked = detect_key_ranked(midi, durs)
        assert len(ranked) == 24 or midi.count(None) == len(midi)
        assert detect_key(midi, durs) == (ranked[0].root, ranked[0].scale)
        scores = [c.score for c in ranked]
        assert scores == sorted(scores, reverse=True)


def test_detect_key_ranked_without_notes():
    assert detect_key_ranked([None, None]) == [("C", "major", 0.0)]
    assert detect_key([None]) == ("C", "major")
