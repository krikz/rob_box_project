"""Импорт первых записей кэша (ADR-0158 §2.10): ``scripts/music/import_gen_cache.py``.

Синтетические mp3/JSON в ``tmp_path`` (формы ответов — как у реального KIE);
отдельный тест на настоящих файлах пропускается, если каталога проб нет.
"""

from __future__ import annotations

import importlib.util
import json
import sqlite3
import sys
from pathlib import Path

import pytest

from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary

_REPO = Path(__file__).resolve().parents[3]
_SCRIPT = _REPO / "scripts" / "music" / "import_gen_cache.py"
if not _SCRIPT.exists():
    pytest.skip(f"скрипт импорта не найден: {_SCRIPT}", allow_module_level=True)
_spec = importlib.util.spec_from_file_location("import_gen_cache", _SCRIPT)
imp = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = imp  # dataclass ищет модуль по имени
_spec.loader.exec_module(imp)


def _kie_record(task_id: str, op: str, inp: dict, durations, credits=12.0, complete_ms=1791885265064) -> dict:
    items = [{"id": f"{task_id}-{i}", "audio_url": f"https://x/{i}.mp3", "duration": d, "title": inp.get("title", "")}
             for i, d in enumerate(durations)]
    return {"code": 200, "data": {
        "taskId": task_id, "model": op, "state": "success", "creditsConsumed": credits,
        "completeTime": complete_ms, "resultJson": json.dumps({"code": 200, "data": items}),
        "paramJson": json.dumps({"input": json.dumps(inp), "model": op}),
    }}


@pytest.fixture
def src(tmp_path):
    d = tmp_path / "suno"
    d.mkdir()
    (d / "valkyries_ours.wav").write_bytes(b"RIFFours")
    for v in (0, 1):
        (d / f"valkyries_cover_{v}.mp3").write_bytes(b"cover%d" % v)
        (d / f"robbox_song_{v}.mp3").write_bytes(b"song%d" % v)
        (d / f"suno_{v}.mp3").write_bytes(b"inst%d" % v)
        (d / f"valkyries_la_{v}.mp3").write_bytes(b"REJECTED%d" % v)  # отвергнуто Шифу — не импортировать
    (d / "lyrics.txt").write_text("Включаюсь в семь, гудит мотор", encoding="utf-8")
    (d / "add_instr_record.json").write_text(json.dumps(_kie_record(
        "7e10941d9e526f029d6f5c5ea90da618", "ai-music-api/add-instrumental",
        {"title": "Я робот (РОББОКС)", "model": "V6", "tags": "90s grunge rock", "audio_weight": 0.8},
        [74.8, 75.16])), encoding="utf-8")
    (d / "record.json").write_text(json.dumps(_kie_record(
        "f7cff936b1124273f2837d0c3402d03c", "ai-music-api/generate",
        {"title": "Robbox Grunge Test", "model": "V6_MINI", "style": "90s grunge", "duration": 60},
        [60.0, 60.0], complete_ms=1791884445760)), encoding="utf-8")
    return d


def _run(src, lib):
    return [r for e in imp.FIRST_ENTRIES for r in imp.import_entry(e, src, lib)]


def test_imports_six_tracks_and_three_ledger_rows(src, tmp_path):
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    rows = _run(src, lib)
    assert [r["status"] for r in rows] == ["imported"] * 6
    assert lib.count == 6
    conn = sqlite3.connect(str(tmp_path / "lib" / "index.db"))
    assert conn.execute("SELECT COUNT(*), SUM(credits) FROM gen_ledger").fetchone() == (3, 36.0)
    ids = {r[0] for r in conn.execute("SELECT task_id FROM gen_ledger")}
    assert {"7e10941d9e526f029d6f5c5ea90da618", "f7cff936b1124273f2837d0c3402d03c"} <= ids


def test_second_run_changes_nothing(src, tmp_path):
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    _run(src, lib)
    snap = sqlite3.connect(str(tmp_path / "lib" / "index.db")).execute(
        "SELECT id, task_id, variant FROM generated_tracks ORDER BY id").fetchall()
    rows = _run(src, GeneratedMusicLibrary(str(tmp_path / "lib")))
    assert [r["status"] for r in rows] == ["exists"] * 6
    assert lib.count == 6
    assert sqlite3.connect(str(tmp_path / "lib" / "index.db")).execute(
        "SELECT id, task_id, variant FROM generated_tracks ORDER BY id").fetchall() == snap
    assert sqlite3.connect(str(tmp_path / "lib" / "index.db")).execute("SELECT COUNT(*) FROM gen_ledger").fetchone()[0] == 3


def test_rejected_valkyries_la_never_imported(src, tmp_path):
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    _run(src, lib)
    for t in lib.list_tracks(limit=100)["tracks"]:
        assert not (Path(lib.root_dir) / t["id"] / "track.mp3").read_bytes().startswith(b"REJECTED")
    assert all("valkyries_la" not in e.prefix for e in imp.FIRST_ENTRIES)


def test_cover_metadata_from_request_params(src, tmp_path):
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    _run(src, lib)
    cover = [t for t in lib.list_tracks(limit=100)["tracks"] if t["kind"] == "cover"]
    assert len(cover) == 2
    c = cover[0]
    assert c["task_id"] is None and c["meta_source"] == "request_params"
    assert (c["work_id"], c["style_key"], c["model"], c["credits"]) == ("Ride Of The Valkyries", "orchestral", "V6", 12.0)
    assert c["source_audio_sha"] == imp.sha256_file(src / "valkyries_ours.wav")
    assert c["is_instrumental"] is True and c["pool_key"] == "cover|orchestral|inst"
    assert c["request_key"] and cover[1]["request_key"] == c["request_key"]


def test_song_metadata_from_api_record(src, tmp_path):
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    _run(src, lib)
    songs = sorted((t for t in lib.list_tracks(limit=100)["tracks"] if t["kind"] == "robot_song"),
                   key=lambda t: t["variant"])
    assert [s["variant"] for s in songs] == [0, 1]
    s = songs[0]
    assert s["task_id"] == "7e10941d9e526f029d6f5c5ea90da618" and s["meta_source"] == "api_record"
    assert (s["tts_provider"], s["voice"], s["style_key"], s["model"]) == ("minimax", "Russian_ReliableMan", "grunge", "V6")
    assert s["duration_ms"] == 74800 and songs[1]["duration_ms"] == 75160
    assert s["lyrics"].startswith("Включаюсь") and s["title"] == "Я робот (РОББОКС)"
    assert s["created_at"].startswith("2026-")  # время из completeTime записи


def test_missing_file_is_reported_not_hidden(src, tmp_path):
    (src / "suno_1.mp3").unlink()
    rows = _run(src, GeneratedMusicLibrary(str(tmp_path / "lib")))
    assert [r["status"] for r in rows].count("missing") == 1


def test_cli_dry_run_touches_nothing(src, tmp_path, capsys):
    lib_dir = tmp_path / "never"
    assert imp.main(["--src", str(src), "--library", str(lib_dir), "--dry-run"]) == 0
    assert not lib_dir.exists()
    assert "valkyries_cover_0.mp3" in capsys.readouterr().out


_REAL = next(
    (p / "artifacts_live_check_2026-10-13" / "suno" for p in _REPO.parents
     if (p / "artifacts_live_check_2026-10-13" / "suno").is_dir()),
    Path("/nonexistent"),
)


@pytest.mark.skipif(not (_REAL / "add_instr_record.json").exists(), reason="каталог проб 13.10 есть только у координатора")
def test_real_probe_files_import_cleanly_and_idempotently(tmp_path):
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    first = _run(_REAL, lib)
    assert [r["status"] for r in first] == ["imported"] * 6, first
    again = _run(_REAL, GeneratedMusicLibrary(str(tmp_path / "lib")))
    assert [r["status"] for r in again] == ["exists"] * 6
    assert lib.count == 6
