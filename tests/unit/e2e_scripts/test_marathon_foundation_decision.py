"""Провал акта 1-2 без вердикта — не «красный по фиче» (issue #5076).

Попытка марафона 22.11: акт 1, GHA run 44973642247, ``conclusion=failure``.
Харнесс напечатал ``E2E_FATAL`` и вышел раньше вердикта (``E2E verdict
missing``), маркера ``E2E_FEATURE_FAIL`` в логе не было. Раннер всё равно
записал «акт 1 красный по фиче — продолжаю марафон»: ветка продолжения была
безусловным else и не смотрела в лог.

Правило (``marathon_foundation_decision``): продолжать марафон можно только
по настоящему маркеру ``E2E_FEATURE_FAIL``. ``E2E_FATAL``, ``E2E_INFRA_FAIL``,
``E2E_NO_REACTION``, ``E2E_STAND_FAIL`` и провал без единого маркера вердикта
останавливают марафон. Эхо исходника шага workflow (``set -x``, ANSI после
метки времени) маркером не считается.

Гоняется настоящая функция из ``scripts/e2e/marathon_markers.sh``.

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_marathon_foundation_decision.py -v --no-cov
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
MARKERS = REPO_ROOT / "scripts" / "e2e" / "marathon_markers.sh"
RUNNER = REPO_ROOT / "scripts" / "e2e" / "run_night_marathon.sh"

needs_bash = pytest.mark.skipif(shutil.which("bash") is None, reason="нужен bash")
BASH = os.environ.get("BASH_EXE") or shutil.which("bash") or "bash"

TS = "2026-11-22T01:55:12.8858151Z"


def decide(lines):
    """Решение настоящей функции по фикстуре лога: 'stop' | 'continue'."""
    log = Path(pytest.importorskip("tempfile").mkdtemp()) / "act.log"
    log.write_text("\n".join(lines) + "\n", encoding="utf-8")
    script = "\n".join(
        [
            "set -u",
            f'. "{MARKERS}"',
            f'marathon_foundation_decision "{log}"',
        ]
    )
    p = subprocess.run(
        [BASH, "-s"], input=script.encode("utf-8"), capture_output=True, timeout=60
    )
    return (
        p.stdout.decode("utf-8", "replace").strip(),
        p.stderr.decode("utf-8", "replace"),
        p.returncode,
    )


@needs_bash
class TestFoundationDecision:
    def test_feature_marker_continues(self):
        """Настоящий E2E_FEATURE_FAIL — находка, марафон идёт дальше."""
        out, err, rc = decide([f"{TS} E2E_FEATURE_FAIL", f"{TS} E2E_VERDICT FAIL"])
        assert rc == 0, (out, err)
        assert out == "continue", (out, err)

    def test_feature_marker_with_prefix_continues(self):
        """Маркер за ``>>>`` (сводка харнесса) — тоже настоящий."""
        out, err, rc = decide([f"{TS} >>> E2E_FEATURE_FAIL gate1"])
        assert rc == 0 and out == "continue", (out, err)

    def test_fatal_stops(self):
        """E2E_FATAL (run 44973642247) — фундаментальный провал, стоп."""
        out, err, rc = decide(
            [
                f"{TS} E2E_FATAL: часть ? серии запущена режимом first, а серия не с нуля",
                "2026-11-22T01:55:12.8905356Z ❌ E2E verdict missing (harness crash?)",
            ]
        )
        assert rc == 0, (out, err)
        assert out == "stop", (out, err)

    def test_infra_fail_stops(self):
        out, err, rc = decide([f"{TS} E2E_INFRA_FAIL synth"])
        assert rc == 0 and out == "stop", (out, err)

    def test_no_reaction_stops(self):
        out, err, rc = decide([f"{TS} E2E_NO_REACTION"])
        assert rc == 0 and out == "stop", (out, err)

    def test_stand_fail_stops(self):
        out, err, rc = decide([f"{TS} E2E_STAND_FAIL"])
        assert rc == 0 and out == "stop", (out, err)

    def test_no_marker_at_all_stops(self):
        """Провал без единого маркера вердикта — не фича, стоп."""
        out, err, rc = decide(
            [
                f"{TS} ##[group]Run e2e",
                "2026-11-22T01:55:12.8905356Z ❌ E2E verdict missing (harness crash?)",
            ]
        )
        assert rc == 0 and out == "stop", (out, err)

    def test_empty_log_stops(self):
        out, err, rc = decide([])
        assert rc == 0 and out == "stop", (out, err)

    def test_workflow_trace_echo_is_not_a_marker(self):
        """Эхо исходника (set -x, ANSI после метки времени) не открывает «продолжать».

        Именно так ``E2E_FEATURE_FAIL`` присутствовал в логе run 44973642247:
        только внутри строки трассировки шага workflow.
        """
        out, err, rc = decide(
            [
                f"{TS} \x1b[36;1mgrep -q E2E_FEATURE_FAIL act.log\x1b[0m",
                f"{TS} E2E_FATAL: серия не с нуля",
            ]
        )
        assert rc == 0 and out == "stop", (out, err)

    def test_fatal_does_not_override_a_real_feature_marker(self):
        """Настоящий маркер фичи решает, даже если рядом есть E2E_FATAL."""
        out, err, rc = decide([f"{TS} E2E_FATAL: x", f"{TS} E2E_FEATURE_FAIL"])
        assert rc == 0 and out == "continue", (out, err)


def test_runner_asks_the_decision_instead_of_an_unconditional_else():
    """Развилка раннера зовёт функцию и не продолжает марафон безусловно."""
    text = RUNNER.read_text(encoding="utf-8")
    assert 'marathon_foundation_decision "$act_log"' in text
    assert "останавливаю марафон" in text
    # Старая безусловная ветка печатала «продолжаю» вне всякой проверки маркера.
    assert text.count("продолжаю марафон") == 1
    assert '[ "$(marathon_foundation_decision "$act_log")" = "continue" ]' in text
