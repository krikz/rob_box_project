"""Тесты импортёра партитур ``score_import.py`` (ADR-0154 PR-2) на синтетических MusicXML, написанных здесь же.

``python -m pytest scripts/music/test_score_import.py -v`` — нужны ``music21`` (иначе тесты пропускаются) и
``PYTHONPATH`` на ``src/rob_box_music``. Партитуры PDMX и чужие аранжировки сюда не кладутся.
"""

from __future__ import annotations

import sqlite3
import sys
from pathlib import Path

import pytest

pytest.importorskip("music21")
sys.path.insert(0, str(Path(__file__).resolve().parent))

from music21 import key, meter, note, stream, tempo  # noqa: E402

import score_import as si  # noqa: E402
from rob_box_music import material as mt  # noqa: E402
from rob_box_music import works  # noqa: E402

C_MAJOR_TUNE = ["C5", "D5", "E5", "G5", "E5", "D5", "C5", "G4"]


def _bars(pitches, per_bar, beats_per_note, ts):
    """Мелодия: ``per_bar`` нот в такте, по кругу ``pitches``, ``bars`` определяется длиной ``pitches``."""
    part = stream.Part()
    first = True
    for b in range(0, len(pitches), per_bar):
        m = stream.Measure(number=b // per_bar + 1)
        if first:
            m.append(key.KeySignature(0))
            m.append(meter.TimeSignature(ts))
            m.append(tempo.MetronomeMark(number=100))
            first = False
        for p in pitches[b:b + per_bar]:
            m.append(note.Note(p, quarterLength=beats_per_note))
        part.append(m)
    return part


def _bass(roots, bar_len, ts, with_signature=True):
    part = stream.Part()
    for i, r in enumerate(roots):
        m = stream.Measure(number=i + 1)
        if i == 0 and with_signature:
            m.append(meter.TimeSignature(ts))
        m.append(note.Note(r, quarterLength=bar_len))
        part.append(m)
    return part


def _write(tmp_path, name, parts):
    s = stream.Score()
    for p in parts:
        s.insert(0, p)
    path = tmp_path / name
    s.write("musicxml", fp=str(path))
    return path


def c_major_44(tmp_path):
    """8 тактов 4/4, до мажор: четыре четверти мелодии в такте над целыми нотами баса C2 C2 F2 G2 ..."""
    tune = (C_MAJOR_TUNE * 4)[:32]
    return _write(tmp_path, "c_major_44.musicxml",
                  [_bars(tune, 4, 1.0, "4/4"), _bass(["C2", "C2", "F2", "G2", "C2", "C2", "G2", "C2"], 4.0, "4/4")])


def a_minor_34_with_pickup(tmp_path):
    """Затакт в одну четверть и 8 тактов 3/4, ля минор."""
    melody = stream.Part()
    pick = stream.Measure(number=0)
    pick.append(meter.TimeSignature("3/4"))
    pick.append(note.Note("E4", quarterLength=1.0))
    pick.padAsAnacrusis()
    melody.append(pick)
    tune = ["A4", "C5", "E5", "A4", "B4", "E5", "A4", "C5", "A4"] * 3
    for i in range(8):
        m = stream.Measure(number=i + 1)
        for p in tune[i * 3:i * 3 + 3]:
            m.append(note.Note(p, quarterLength=1.0))
        melody.append(m)
    bass = stream.Part()
    pick_b = stream.Measure(number=0)
    pick_b.append(meter.TimeSignature("3/4"))
    pick_b.append(note.Rest(quarterLength=1.0))
    pick_b.padAsAnacrusis()
    bass.append(pick_b)
    for i in range(8):
        m = stream.Measure(number=i + 1)
        m.append(note.Note("A2" if i % 2 == 0 else "E2", quarterLength=3.0))
        bass.append(m)
    return _write(tmp_path, "a_minor_34.musicxml", [melody, bass])


def mixed_meters(tmp_path):
    """2 такта 3/4, затем 8 тактов 4/4: берётся самый длинный участок — 4/4."""
    part = stream.Part()
    for i in range(10):
        m = stream.Measure(number=i + 1)
        if i == 0:
            m.append(meter.TimeSignature("3/4"))
        if i == 2:
            m.append(meter.TimeSignature("4/4"))
        n = 3 if i < 2 else 4
        for p in (C_MAJOR_TUNE * 2)[:n]:
            m.append(note.Note(p, quarterLength=1.0))
        part.append(m)
    return _write(tmp_path, "mixed.musicxml", [part])


def too_short(tmp_path):
    return _write(tmp_path, "short.musicxml", [_bars(C_MAJOR_TUNE[:3] * 1 + ["C5"], 4, 1.0, "4/4")])


META = {"material_id": "local:deadbeef", "license": "PD", "composer": "test", "source": "synthetic"}


def _build(path, **over):
    from music21 import converter
    return si.build_material(converter.parse(str(path)), {**META, "title": path.stem, **over})


# ── материал из партитуры ──────────────────────────────────────────────────────────────────────────────────────

def test_c_major_44_gives_valid_material_with_authors_chords_bass_and_key(tmp_path):
    material, info = _build(c_major_44(tmp_path))
    mt.validate_material(material)
    assert (material.meter, material.bpm, material.key.mode) == ((4, 4), 100, "major")
    assert material.key.root == 0 and len(material.melody) == 32 and info["bars_used"] == 8
    assert material.melody[0].accent == 3 and material.melody[1].accent == 2  # первая доля такта, другая доля
    degrees = [c.degree for c in material.chords]
    assert degrees[0] == 0 and 3 in degrees and 4 in degrees  # I, IV, V из басов C F G
    assert {e.midi for e in material.bass} >= {36, 41, 43}  # C2 F2 G2 — басовый голос, не мелодия
    assert material.phrases and material.phrases[0].relation == "new" and material.stats.key_fit == 1.0


def test_pickup_bar_is_dropped_and_bars_start_on_the_first_full_bar(tmp_path):
    material, info = _build(a_minor_34_with_pickup(tmp_path))
    assert material.meter == (3, 4) and info["bars_used"] == 8 and info["bars_total"] == 9
    assert material.melody[0].beat == 0.0 and material.melody[0].midi == 69  # A4, не затакт E4
    assert material.key.mode == "minor" and material.key.root == 9


def test_mixed_meters_keep_the_longest_run(tmp_path):
    material, info = _build(mixed_meters(tmp_path))
    assert material.meter == (4, 4) and info["bars_used"] == 8 and info["bars_total"] == 10


def test_too_short_score_is_refused_with_a_reason(tmp_path):
    with pytest.raises(si.Refusal) as exc:
        _build(too_short(tmp_path))
    assert exc.value.code == "too_short" and "< 4" in exc.value.reason


def test_unpitched_percussion_part_is_skipped_not_a_parse_error(tmp_path):
    """Барабанная партия PDMX (``Unpitched``) раньше роняла файл в ``parse_error`` — теперь это просто не мелодия."""
    drums = stream.Part()
    for i in range(8):
        m = stream.Measure(number=i + 1)
        for _ in range(4):
            m.append(note.Unpitched(quarterLength=1.0))
        drums.append(m)
    tune = (C_MAJOR_TUNE * 4)[:32]
    path = _write(tmp_path, "with_drums.musicxml", [_bars(tune, 4, 1.0, "4/4"), drums])
    material, _info = _build(path)
    assert len(material.melody) == 32


def test_material_survives_json_round_trip(tmp_path):
    material, _info = _build(c_major_44(tmp_path))
    assert mt.from_json(mt.to_json(material)) == material


def test_chord_degrees_come_from_the_score_not_a_template(tmp_path):
    material, _ = _build(c_major_44(tmp_path))
    roots = [(c.beat, c.root_pc) for c in material.chords]
    assert roots[0] == (0.0, 0) and (8.0, 5) in roots and (12.0, 7) in roots  # F на 3-м такте, G на 4-м


# ── пакет, лицензии, индекс ────────────────────────────────────────────────────────────────────────────────────

def test_local_file_without_license_is_refused_not_accepted_as_is(tmp_path):
    f = c_major_44(tmp_path)
    res = si.process_file((str(f), None, "", ""))
    assert res["status"] == "refused" and res["code"] == "license"
    ok = si.process_file((str(f), None, "PD", "synthetic"))
    assert ok["status"] == "ok" and ok["material_id"].startswith("local:")


def test_pdmx_row_with_license_conflict_is_refused_before_parsing(tmp_path):
    f = c_major_44(tmp_path)
    csv_rows = {"c_major_44": {"license": "cc-zero", "license_conflict": True, "rating": 4.5}}
    res = si.process_file((str(f), csv_rows, "", ""))
    assert (res["status"], res["code"]) == ("refused", "license_conflict")


def test_file_missing_from_pdmx_csv_is_refused(tmp_path):
    res = si.process_file((str(c_major_44(tmp_path)), {"other": {"license": "cc-zero", "license_conflict": False}}, "", ""))
    assert res["code"] == "not_in_csv"


def test_garbage_file_is_a_parse_error_in_the_report_not_a_crash(tmp_path):
    bad = tmp_path / "bad.musicxml"
    bad.write_text("это не MusicXML", encoding="utf-8")
    res = si.process_file((str(bad), None, "PD", ""))
    assert res["status"] == "refused" and res["code"] == "parse_error"


def test_pdmx_import_goes_to_json_library_and_score_index_with_license_rating_and_keysig(tmp_path):
    f = c_major_44(tmp_path)
    csv_rows = {"c_major_44": {"license": "publicdomain", "license_conflict": False, "rating": 4.8, "n_ratings": 12,
                               "n_views": 300, "complexity": 1, "genres": "classical", "title": "Тестовая тема",
                               "composer": "Некто"}}
    results = si.run_batch([f], csv_rows, "", "", 1)
    assert results[0]["status"] == "ok" and results[0]["material_id"] == "pdmx:c_major_44"
    db = tmp_path / "voice_memory.db"
    out = si.store(results, tmp_path / "lib", str(db))
    assert out["index_written"] == 1 and not out["index_rejected"]
    assert mt.from_json((tmp_path / "lib" / "pdmx_c_major_44.json").read_text(encoding="utf-8")).title == "Тестовая тема"
    row = sqlite3.connect(db).execute("SELECT license, rating, n_ratings, keysig, meter, key, genres, file "
                                      "FROM score_index WHERE material_id='pdmx:c_major_44'").fetchone()
    assert row == ("publicdomain", 4.8, 12, "C major", "4/4", "C major", "classical", "pdmx_c_major_44.json")


def test_report_counts_reasons_and_speed(tmp_path):
    files = [c_major_44(tmp_path), too_short(tmp_path), mixed_meters(tmp_path)]
    results = si.run_batch(files, None, "PD", "synthetic", 1)
    text = si.report(results, 1.5, 1)
    assert "файлов 3; принято 2 (66.7%" in text and "too_short ×1" in text and "файл/с" in text


def test_score_source_is_linked_to_the_work_as_a_proposal_only(tmp_path):
    db = sqlite3.connect(tmp_path / "m.db")
    works.write_registry(db, works.build_works([{"name": "ode", "title": "Тестовая тема", "artist": "", "tags": [],
                                                 "rtttl": "t:d=4,o=5,b=120:c,d,e,f,g,a,b,c6,d6"}]))
    f = c_major_44(tmp_path)
    results = si.run_batch([f], {"c_major_44": {"license": "publicdomain", "license_conflict": False,
                                                "title": "Тестовая тема", "composer": "Некто"}}, "", "", 1)
    si.store(results, None, str(tmp_path / "m.db"))
    rows = sqlite3.connect(tmp_path / "m.db").execute(
        "SELECT material_id, kind, link_level, confirmed FROM work_sources ORDER BY rank").fetchall()
    assert rows == [("rtttl:ode", "rtttl", "exact", 1), ("pdmx:c_major_44", "pdmx", "exact", 0)]
