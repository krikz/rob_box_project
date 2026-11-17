# -*- coding: utf-8 -*-
"""#4512: e2e-БД ходов ``dialog_events_<run>.db`` забирается из контейнера при любом исходе акта.

БД лежала в /tmp контейнера ``voice-assistant``, деплой пересоздавал контейнер -- не сохранилась ни одна.
Теперь trap EXIT
харнесса зовёт ``collect_e2e_dialog_db``; tar БД остаётся на билд-машине (репо публичное, в БД реплики людей),
в артефакт идёт только сводка без текстов.

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_4512_turn_db_artifact.py -v -o addopts=""
"""

from __future__ import annotations

import json
import re
import sqlite3
import subprocess
import sys
import tarfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
E2E = REPO_ROOT / "scripts" / "ci" / "e2e"
HARNESS = (E2E / "e2e_voice_test.sh").read_text(encoding="utf-8")
LIB = (E2E / "e2e_voice_lib.sh").read_text(encoding="utf-8")
WORKFLOW = (REPO_ROOT / ".github" / "workflows" / "L-E2E Voice Test.yml").read_text(encoding="utf-8")
SUMMARY = E2E / "dialog_db_summary.py"


def test_trap_exit_collects_dialog_db_after_deactivate():
    """Копирование -- в trap EXIT (срабатывает и при FAIL/обрыве), строго после deactivate (БД закрыта)."""
    trap = next(ln for ln in HARNESS.splitlines() if ln.startswith("trap ") and "EXIT" in ln and "collect_e2e" in ln)
    assert trap.index("deactivate_e2e_dialog_db") < trap.index('collect_e2e_dialog_db "$OUT_DIR"')


def test_collect_function_reads_wal_and_never_live_data():
    body = LIB[LIB.index("collect_e2e_dialog_db() {") : LIB.index("restore_robot_tts_default() {")]
    assert "docker exec voice-assistant" in body
    assert "-wal" in body and "-shm" in body  # WAL: без них свежие строки теряются
    assert "/data | /data/*" in body  # боевая /data не копируется
    assert "E2E_ACT_STARTED" in body and "return 0" in body  # trap не падает и не красит акт


def test_workflow_pushes_helper_and_does_not_upload_db_or_tar():
    assert "scripts/ci/e2e/dialog_db_summary.py ros2@10.1.1.249:/tmp/dialog_db_summary.py" in WORKFLOW
    assert "/tmp/dialog_db_summary.py && echo HARNESS_PUSHED" in WORKFLOW
    # репо публичное: тексты реплик не в артефакт -- в upload-паттернах нет ни *.tar, ни *.db
    patterns = re.findall(r"/tmp/e2e_artifacts_\$\{\{ github\.run_id \}\}/\S+", WORKFLOW)
    assert patterns and not [p for p in patterns if p.endswith((".tar", ".db", "/*", "/**"))]


def test_summary_has_no_texts(tmp_path):
    db = tmp_path / "dialog_events_1.db"
    con = sqlite3.connect(db)
    con.execute(
        "CREATE TABLE dialog_events (id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL, session_id TEXT, epoch INTEGER,"
        " turn_id TEXT, agent TEXT, kind TEXT, text TEXT, data_json TEXT)"
    )
    rows = [
        ("s", 1, "t1", "user", "user", "СЕКРЕТНАЯ РЕПЛИКА", "{}"),
        ("s", 1, "t1", "tars", "turn", "", json.dumps({"stage": "done", "reason": "answer"})),
        ("s", 1, "t2", "tars", "turn", "", json.dumps({"stage": "failed"})),
    ]
    con.executemany(
        "INSERT INTO dialog_events (ts, session_id, epoch, turn_id, agent, kind, text, data_json)"
        " VALUES (0, ?, ?, ?, ?, ?, ?, ?)",
        rows,
    )
    con.commit()
    con.close()
    tar = tmp_path / "x.tar"
    with tarfile.open(tar, "w") as tf:
        tf.add(db, arcname=db.name)
    out = tmp_path / "out.json"
    proc = subprocess.run([sys.executable, str(SUMMARY), str(tar), db.name, str(out)], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    raw = out.read_text(encoding="utf-8")
    assert "СЕКРЕТНАЯ" not in raw
    report = json.loads(raw)
    assert report["events"] == {"user": 1, "turn": 2}
    assert report["turns"] == 2
    assert report["turn_outcomes"] == {"done/answer": 1, "failed/-": 1}


def test_summary_survives_broken_tar(tmp_path):
    bad = tmp_path / "bad.tar"
    bad.write_bytes(b"not a tar")
    out = tmp_path / "out.json"
    proc = subprocess.run([sys.executable, str(SUMMARY), str(bad), "x.db", str(out)], capture_output=True, text=True)
    assert proc.returncode == 0
    assert "error" in json.loads(out.read_text(encoding="utf-8"))
