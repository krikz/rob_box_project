"""#4512 (шаг 1 PR #4603): офлайн-вердикты детекторов сбоя хода в артефакте акта и сводке марафона, без текстов.

``scripts/ci/e2e/turn_failures_summary.py`` гонит лог ``voice-assistant`` акта через
``scripts/dialog/turn_failures.py``;
в ``turn_failures.json`` идут счётчики и коды причин -- репо публичное, текстов реплик там быть не должно.

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_4512_marathon_turn_failures.py -v --no-cov
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SUMMARY = REPO_ROOT / "scripts" / "ci" / "e2e" / "turn_failures_summary.py"
WORKFLOW = (REPO_ROOT / ".github" / "workflows" / "L-E2E Voice Test.yml").read_text(encoding="utf-8")
MARATHON = (REPO_ROOT / "scripts" / "e2e" / "run_night_marathon.sh").read_text(encoding="utf-8")

# BASH_EXE -- явный bash (на Windows в PATH бывает WSL-bash, не понимающий пути D:/).
BASH = os.environ.get("BASH_EXE") or shutil.which("bash") or "bash"
SECRET = "СЕКРЕТНАЯ РЕПЛИКА ЧЕЛОВЕКА"
_STT = "[stt_node-5] [WARN] [%d.000000000] [stt_node]: [stt_attempt] y:empty(1ms) -> rejected"
_DIALOG = "[dialog_node-3] [INFO] [%d.500000000] [dialog_node]: "


def deaf_lines(base=1793743400):
    return [_STT % (base + 10 * i) for i in range(3)]


def silent_turn_lines(base=1793743500, turn_id="0123456789ab"):
    """Ход открылся, закончился (agent=personality, stage=done) -- и ни одной реплики в очереди."""
    return [
        _DIALOG % base + f"🧭 [turn-v2] user said {SECRET} turn={turn_id}",
        _DIALOG % (base + 1)
        + f"📊 [turn] turn_id={turn_id} epoch=3 agent=personality stage=done status=finished latency_ms=1",
    ]


def run(tmp_path, lines):
    log = tmp_path / "voice.log"
    log.write_text("\n".join(lines) + "\n", encoding="utf-8")
    out = tmp_path / "turn_failures.json"
    proc = subprocess.run(
        [sys.executable, str(SUMMARY), str(log), str(out)], capture_output=True, text=True, encoding="utf-8"
    )
    assert proc.returncode == 0, proc.stderr
    return json.loads(out.read_text(encoding="utf-8")), proc.stdout.strip(), out.read_text(encoding="utf-8")


def test_deaf_series_is_counted_with_reason_code(tmp_path):
    report, line, _ = run(tmp_path, deaf_lines())
    assert report["verdicts"] == {"deaf": 1, "no_speech": 0}
    assert report["reasons"] == {"deaf/stt_rejected=3": 1}
    assert line == "E2E_SUMMARY turn_failures=deaf:1"


def test_silent_turn_is_counted_and_report_has_no_texts(tmp_path):
    report, line, raw = run(tmp_path, silent_turn_lines() + deaf_lines())
    assert report["verdicts"]["no_speech"] == 1 and report["verdicts"]["deaf"] == 1
    assert line == "E2E_SUMMARY turn_failures=deaf:1,no_speech:1"
    assert SECRET not in raw and SECRET not in line
    assert "ts=" not in raw  # строки Verdict.line() в отчёт не идут -- только счётчики и коды


def test_clean_log_is_zero_and_empty_log_is_unknown(tmp_path):
    ok_turn = silent_turn_lines()[:1] + [
        _DIALOG % 1793743501
        + "📊 [turn] turn_id=0123456789ab epoch=3 agent=telegram stage=done status=finished latency_ms=1"
    ]
    report, line, _ = run(tmp_path, ok_turn)
    assert report["verdicts"] == {"deaf": 0, "no_speech": 0} and line == "E2E_SUMMARY turn_failures=0"
    # «0 сбоев» при пустом логе (шаг сбора записал "Failed") было бы враньём: событий нет -- unknown
    report, line, _ = run(tmp_path, ["Failed"])
    assert report["events"] == 0 and line == "E2E_SUMMARY turn_failures=unknown"


def test_missing_log_does_not_fail_the_act(tmp_path):
    out = tmp_path / "x.json"
    proc = subprocess.run(
        [sys.executable, str(SUMMARY), str(tmp_path / "нет.log"), str(out)],
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    assert proc.returncode == 0
    assert "error" in json.loads(out.read_text(encoding="utf-8"))
    assert proc.stdout.strip() == "E2E_SUMMARY turn_failures=unknown"


def test_marathon_reads_value_from_act_log(tmp_path):
    markers = REPO_ROOT / "scripts" / "e2e" / "marathon_markers.sh"
    log = tmp_path / "act.log"
    log.write_text(
        "build\tTurn-failure verdicts\t2026-11-09T10:00:00.1Z E2E_SUMMARY turn_failures=deaf:2,no_speech:1\n",
        encoding="utf-8",
    )
    p = subprocess.run(
        [BASH, "-c", '. "%s"; marathon_summary_value "%s" turn_failures' % (markers.as_posix(), log.as_posix())],
        capture_output=True,
        timeout=30,
    )
    assert p.stdout.decode().strip() == "deaf:2,no_speech:1", p


def test_workflow_and_marathon_are_wired():
    assert "scripts/ci/e2e/turn_failures_summary.py" in WORKFLOW
    assert 'cp -f /tmp/e2e_turn_failures.json "$ART_DIR/turn_failures.json"' in WORKFLOW
    assert 'marathon_summary_value "$act_log" turn_failures' in MARATHON and "turn_failures.tsv" in MARATHON
