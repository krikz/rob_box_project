"""Тесты батча PDMX ``pdmx_batch.py`` (ADR-0154 PR-6) на синтетическом CSV и синтетических партитурах.

``python -m pytest scripts/music/test_pdmx_batch.py -v`` — нужны ``music21`` (иначе пропуск) и ``pytest -o pythonpath=src/rob_box_music``.
"""

from __future__ import annotations

import csv
import json
import sqlite3
import sys
from pathlib import Path

import pytest

pytest.importorskip("music21")
sys.path.insert(0, str(Path(__file__).resolve().parent))

import pdmx_batch as pb  # noqa: E402
from test_score_import import c_major_44, too_short  # noqa: E402

COLUMNS = ["mxl", "title", "song_name", "composer_name", "artist_name", "license", "license_conflict", "rating",
           "n_ratings", "n_views", "complexity", "genres", "subset:no_license_conflict", "subset:deduplicated"]


def _row(stem, title, composer="Некто", lic="cc-zero", conflict="False", rating="4.5", n="10", views="100",
         nlc="True", dedup="True"):
    return {"mxl": f"./mxl/0/0/{stem}.mxl", "title": title, "song_name": title, "composer_name": composer,
            "artist_name": "", "license": lic, "license_conflict": conflict, "rating": rating, "n_ratings": n,
            "n_views": views, "complexity": "1", "genres": "classical", "subset:no_license_conflict": nlc,
            "subset:deduplicated": dedup}


def _write_csv(path, rows):
    with open(path, "w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)


def test_select_filters_license_stop_list_and_orders_by_bayes_rating(tmp_path):
    _write_csv(tmp_path / "p.csv", [
        _row("a", "Один голос оценки", rating="5.0", n="1"),   # 5.0×1 не обгоняет 4.6×50
        _row("b", "Проверенная", rating="4.6", n="50"),
        _row("c", "Конфликт", conflict="True", nlc="False"),
        _row("d", "Без лицензии", lic=""),
        _row("e", "Тема Hans Zimmer", composer="Hans Zimmer"),  # стоп-лист живых правообладателей
        _row("f", "Копия", dedup="False"),
    ])
    rows, stats = pb.select_rows(str(tmp_path / "p.csv"))
    assert [r["stem"] for r in rows] == ["b", "a", "f"]
    assert [r["tier"] for r in rows] == [1, 1, 2]
    assert stats["drop_license_conflict"] == 1 and stats["drop_license_unusable"] == 1 and stats["drop_stop_list"] == 1


def test_select_tier2_drops_copies_of_works_already_in_tier1(tmp_path):
    _write_csv(tmp_path / "p.csv", [_row("a", "Ода"), _row("b", "Ода", dedup="False"),
                                    _row("c", "Другая", dedup="False"), _row("d", "Другая", dedup="False", rating="3.0")])
    rows, _ = pb.select_rows(str(tmp_path / "p.csv"))
    assert [r["stem"] for r in rows] == ["a", "c"]  # b — копия a; из двух «Другая» лучшая по рангу
    rows_all, _ = pb.select_rows(str(tmp_path / "p.csv"), tier2_dedup=False)
    assert len(rows_all) == 4


def _selection(tmp_path, stems):
    sel = tmp_path / "sel.jsonl"
    rows = [{"stem": s, "mxl": f"mxl/0/0/{s}.mxl", "license": "cc-zero", "license_conflict": False, "rating": 4.0,
             "n_ratings": 5, "n_views": 10, "complexity": 1, "genres": "", "title": s, "composer": "Некто"}
            for s in stems]
    sel.write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in rows), encoding="utf-8")
    return sel


def _files(tmp_path):
    d = tmp_path / "mxl" / "0" / "0"
    d.mkdir(parents=True)
    (d / "good.mxl").write_bytes(c_major_44(tmp_path).read_bytes())
    (d / "short.mxl").write_bytes(too_short(tmp_path).read_bytes())
    (d / "bad.mxl").write_text("не музыка", encoding="utf-8")


def test_run_writes_manifest_json_and_index_and_resumes(tmp_path):
    _files(tmp_path)
    sel = _selection(tmp_path, ["good", "short", "bad", "gone"])
    args = ["run", "--sel", str(sel), "--root", str(tmp_path), "--lib", str(tmp_path / "lib"),
            "--manifest", str(tmp_path / "m.jsonl"), "--jobs", "1"]
    assert pb.main(args) == 0
    manifest = {m["stem"]: m for m in pb.read_jsonl(tmp_path / "m.jsonl")}
    assert manifest["good"]["status"] == "ok" and manifest["good"]["material_id"] == "pdmx:good"
    assert [manifest[s]["code"] for s in ("short", "bad", "gone")] == ["too_short", "parse_error", "missing_file"]
    assert (tmp_path / "lib" / "pdmx_good.json").is_file()
    assert pb.main(args) == 0  # повтор ничего не делает: всё в манифесте
    assert len(list(pb.read_jsonl(tmp_path / "m.jsonl"))) == 4
    assert pb.main(["index", "--manifest", str(tmp_path / "m.jsonl"), "--lib", str(tmp_path / "lib")]) == 0
    row = sqlite3.connect(tmp_path / "lib" / pb.INDEX_FILE).execute(
        "SELECT material_id, license, rating FROM score_index").fetchall()
    assert row == [("pdmx:good", "cc-zero", 4.0)]


def test_torn_last_manifest_line_is_redone_not_fatal(tmp_path):
    (tmp_path / "m.jsonl").write_text('{"stem": "a", "status": "refused", "code": "x", "reason": ""}\n{"stem": "b", "st',
                                      encoding="utf-8")
    assert [m["stem"] for m in pb.read_jsonl(tmp_path / "m.jsonl")] == ["a"]


def test_report_counts_reasons(tmp_path):
    lines = pb.report_lines([
        {"stem": "a", "status": "ok", "seconds": 0.2, "bytes": 1000},
        {"stem": "b", "status": "refused", "seconds": 0.1, "code": "too_short", "reason": "1 т."}])
    text = "\n".join(lines)
    assert "принято 1 (50.0%" in text and "too_short ×1" in text
