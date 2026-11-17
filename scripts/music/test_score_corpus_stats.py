"""Тесты ``research/score_corpus_stats.py`` (ADR-0154 PR-6): библиотека JSON → строки для перевыучивания таблиц §3.4.

``python -m pytest scripts/music/test_score_corpus_stats.py -v -o pythonpath=src/rob_box_music``. Материал — синтетический.
"""

from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "research"))

import score_bass_tones as sbt  # noqa: E402
import score_corpus_stats as scs  # noqa: E402
import score_markov_harmony as smh  # noqa: E402

from rob_box_music import material as mt  # noqa: E402
from rob_box_music.model import Key, PitchEvent  # noqa: E402


def synth(mid="pdmx:a", mode="major", degrees=(0, 5, 3, 4), bass_midis=(36, 45, 41, 43)):
    """Такт на аккорд (4/4): C F G / Am; бас — по такту, тон задан по построению."""
    scale = (0, 2, 4, 5, 7, 9, 11) if mode == "major" else (0, 2, 3, 5, 7, 8, 10)
    chords = []
    for i, d in enumerate(degrees):
        root = scale[d] % 12
        third = (scale[(d + 2) % 7] - scale[d]) % 12
        chords.append(mt.ChordSpan(i * 4.0, 4.0, root, "maj" if third == 4 else "min", d))
    melody = tuple(PitchEvent(72, i * 1.0, 1.0, 2) for i in range(4 * len(degrees)))
    bass = tuple(PitchEvent(m, i * 4.0, 4.0, 2) for i, m in enumerate(bass_midis))
    return mt.ScoreMaterial(mid, "t", "c", "synthetic", "PD", (4, 4), 100, Key(0, mode), melody, tuple(chords), bass)


def write(lib: Path, m: mt.ScoreMaterial) -> None:
    (lib / (m.material_id.replace(":", "_") + ".json")).write_text(mt.to_json(m), encoding="utf-8")


def test_bass_on_root_third_fifth_and_other_are_counted_against_the_chord_of_that_beat():
    # такты: C (бас C), F (бас A = терция), G (бас D = квинта), C-ступень 0 (бас F# = чужая); подхода нет
    m = synth(degrees=(0, 3, 4, 0), bass_midis=(36, 45, 50, 42))
    out = scs.bass_relations(m)
    assert out["bass_rel"] == {"root": 1, "third": 1, "fifth": 1, "other": 1}
    assert out["bass_approach"] == 0 and out["bars"] == 4


def test_semitone_approach_to_the_first_beat_of_the_next_bar_is_counted():
    m = synth(degrees=(0, 3), bass_midis=(36, 38))  # C -> D: тон, не полутон
    m2 = synth(degrees=(0, 0, 3), bass_midis=(36, 40, 41))  # E -> F полутоном на первой доле следующего такта
    assert scs.bass_relations(m)["bass_approach"] == 0
    assert scs.bass_relations(m2)["bass_approach"] == 1


def test_library_to_rows_to_tables_end_to_end(tmp_path):
    for i, mode in enumerate(("major", "major", "minor")):
        write(tmp_path, synth(f"pdmx:{i}", mode))
    write(tmp_path, replace(synth("pdmx:dorian"), key=Key(0, "dorian")))  # лады вне §3.4 в корпус таблиц не входят
    rows = scs.collect(tmp_path, jobs=1)
    assert sorted(r["mode"] for r in rows) == ["major", "major", "minor"]
    assert all(r["degree_seq"] == [0, 5, 3, 4] for r in rows if r["mode"] == "major")
    table = smh.corpus_table(rows, "major")
    assert all(abs(sum(row) - 1.0) < 1e-3 for row in table["next"])
    assert table["next"][4][0] == max(table["next"][4])  # V → I — самая частая у этого корпуса
    bass = sbt.mode_table(rows, "major")
    assert abs(sum(bass[k] for k in sbt.RELATIONS) - 1.0) < 1e-3 and bass["root"] == 1.0
    json.dumps(rows)  # строки сериализуются в stats.json
