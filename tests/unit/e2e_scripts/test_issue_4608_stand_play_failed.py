"""Issue #4608 -- провал воспроизведения фразы (PulseAudio connection refused) -- сбой стенда, а не «робот не принял».

Исполняются настоящие ``play_eq_wav`` / ``step_stand_play_failed`` / ``mark_fail_kind`` / ``write_summary`` из
``e2e_voice_test.sh`` и ``stand_audio_preflight`` из ``e2e_voice_lib.sh``; paplay/pactl -- заглушки в PATH.

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_4608_stand_play_failed.py -v --no-cov
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / "scripts" / "ci" / "e2e"
E2E_SCRIPT = SCRIPTS_DIR / "e2e_voice_test.sh"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"
MARKERS = REPO_ROOT / "scripts" / "e2e" / "marathon_markers.sh"
SCRIPT_LINES = E2E_SCRIPT.read_text(encoding="utf-8").replace("\r\n", "\n").split("\n")

needs_bash = pytest.mark.skipif(
    shutil.which("bash") is None or shutil.which("python3") is None, reason="нужны bash и python3"
)

# BASH_EXE -- явный bash (на Windows в PATH бывает WSL-bash, где заглушки из tmp не исполняются).
BASH = os.environ.get("BASH_EXE") or shutil.which("bash") or "bash"

CONNECT_FAILED = "pa_context_connect() failed: Connection refused"


def extract_function(name):
    start = next(i for i, ln in enumerate(SCRIPT_LINES) if ln.startswith(name + "() {"))
    heredoc = None
    for i in range(start + 1, len(SCRIPT_LINES)):
        line = SCRIPT_LINES[i]
        if heredoc is not None:
            if line.strip() == heredoc:
                heredoc = None
            continue
        found = re.search(r"<<-?'([A-Za-z_][A-Za-z0-9_]*)'", line)
        if found:
            heredoc = found.group(1)
        elif line == "}":
            return "\n".join(SCRIPT_LINES[start : i + 1])
    raise AssertionError(name)


def lib_function(name):
    lines = LIB.read_text(encoding="utf-8").replace("\r\n", "\n").split("\n")
    start = next(i for i, ln in enumerate(lines) if ln.startswith(name + "() {"))
    end = next(i for i in range(start + 1, len(lines)) if lines[i] == "}")
    return "\n".join(lines[start : end + 1])


def run(tmp_path, body, paplay="exit 0", pactl="exit 0"):
    bindir = tmp_path / "bin"
    bindir.mkdir(exist_ok=True)
    counter = tmp_path / "play_count"
    (bindir / "paplay").write_text(
        '#!/bin/bash\nn=$(( $(cat "%s" 2>/dev/null || echo 0) + 1 ))\necho "$n" > "%s"\n%s\n'
        % (counter.as_posix(), counter.as_posix(), paplay),
        encoding="utf-8",
    )
    (bindir / "pactl").write_text("#!/bin/bash\n%s\n" % pactl, encoding="utf-8")
    # python3 -- тот же интерпретатор, что гонит pytest (в Git Bash python3 бывает заглушкой Microsoft Store).
    shim = "#!/bin/bash" + chr(10) + 'exec "%s" "$@"' % Path(sys.executable).as_posix() + chr(10)
    (bindir / "python3").write_text(shim, encoding="utf-8")
    for name in ("paplay", "pactl", "python3"):
        (bindir / name).chmod(0o755)
    out = tmp_path / "out"
    out.mkdir(exist_ok=True)
    script = "\n".join(
        [
            "set -u",
            'BIN="%s"; command -v cygpath >/dev/null 2>&1 && BIN="$(cygpath -u "$BIN")"; export PATH="$BIN:$PATH"'
            % bindir.as_posix(),
            'OUT_DIR="%s"' % out.as_posix(),
            "E2E_PLAY_RETRY_PAUSE=0; E2E_PREFLIGHT_PAUSE=0; E2E_FAIL_KIND=''",
            'log() { echo ">>> $*"; }',
            extract_function("ensure_outdir"),
            extract_function("emit_step"),
            extract_function("mark_fail_kind"),
            extract_function("play_eq_wav"),
            extract_function("step_stand_play_failed"),
            lib_function("stand_audio_preflight"),
            body,
            'echo "PLAY_COUNT=$(cat "%s" 2>/dev/null || echo 0)"' % counter.as_posix(),
            'echo "FAIL_KIND=${E2E_FAIL_KIND}"',
        ]
    )
    p = subprocess.run([BASH, "-s"], input=script.encode("utf-8"), capture_output=True, timeout=60)
    return p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace"), out


@needs_bash
class TestPlayEqWav:
    def test_success_plays_once(self, tmp_path):
        out, err, _ = run(tmp_path, 'play_eq_wav x.wav; echo "RC=$? FAILED=$E2E_PLAY_FAILED"')
        assert "RC=0 FAILED=0" in out, (out, err)
        assert "PLAY_COUNT=1" in out and "PLAY_DONE" in out

    def test_connection_refused_retries_twice_then_fails_with_reason(self, tmp_path):
        out, err, _ = run(
            tmp_path,
            'play_eq_wav x.wav; echo "RC=$? FAILED=$E2E_PLAY_FAILED R=$E2E_PLAY_FAIL_REASON"',
            paplay='echo "%s" >&2; exit 1' % CONNECT_FAILED,
        )
        assert "RC=1 FAILED=1 R=pulse_connect_failed" in out, (out, err)
        assert "PLAY_COUNT=3" in out, "1 попытка + 2 повтора"
        assert "попытка 3/3" in out

    def test_connect_failed_in_output_fails_even_with_rc_zero(self, tmp_path):
        out, err, _ = run(tmp_path, 'play_eq_wav x.wav; echo "RC=$?"', paplay='echo "%s"; exit 0' % CONNECT_FAILED)
        assert "RC=1" in out, (out, err)

    def test_retry_recovers_when_pulse_comes_back(self, tmp_path):
        paplay = 'if [ "$n" -lt 2 ]; then echo "%s" >&2; exit 1; fi; exit 0' % CONNECT_FAILED
        out, err, _ = run(tmp_path, 'play_eq_wav x.wav; echo "RC=$? FAILED=$E2E_PLAY_FAILED"', paplay=paplay)
        assert "RC=0 FAILED=0" in out, (out, err)
        assert "PLAY_COUNT=2" in out

    def test_plain_nonzero_rc_reason(self, tmp_path):
        out, err, _ = run(tmp_path, 'play_eq_wav x.wav; echo "R=$E2E_PLAY_FAIL_REASON"', paplay="exit 3")
        assert "R=paplay_rc=3" in out, (out, err)


@needs_bash
class TestStandVerdict:
    def test_step_gets_stand_play_failed_not_no_accept(self, tmp_path):
        out, err, outdir = run(
            tmp_path, "play_eq_wav x.wav || step_stand_play_failed a1", paplay='echo "%s" >&2; exit 1' % CONNECT_FAILED
        )
        assert "E2E_STEP a1 FAIL stand_play_failed (pulse_connect_failed)" in out, (out, err)
        assert "no_accept" not in out
        assert "FAIL_KIND=stand" in out
        rec = json.loads((outdir / "steps.jsonl").read_text(encoding="utf-8").strip())
        assert rec["status"] == "FAIL" and rec["detail"].startswith("stand_play_failed")

    def test_stand_does_not_outrank_real_robot_failures(self, tmp_path):
        out, err, _ = run(tmp_path, "E2E_FAIL_KIND=feature; mark_fail_kind stand")
        assert "FAIL_KIND=feature" in out, (out, err)
        out, err, _ = run(tmp_path, "mark_fail_kind no_reaction; mark_fail_kind stand")
        assert "FAIL_KIND=stand" in out, (out, err)
        out, err, _ = run(tmp_path, "mark_fail_kind stand; mark_fail_kind no_reaction")
        assert "FAIL_KIND=stand" in out, "no_reaction не перебивает stand"


@needs_bash
class TestPreflight:
    def test_pulse_up(self, tmp_path):
        out, err, _ = run(tmp_path, 'stand_audio_preflight; echo "RC=$?"', pactl='echo "Server Name: pulse"')
        assert "RC=0" in out and "PREFLIGHT pulse: OK" in out, (out, err)

    def test_pulse_down_blocks_act(self, tmp_path):
        out, err, _ = run(tmp_path, 'stand_audio_preflight; echo "RC=$?"', pactl='echo "%s"; exit 1' % CONNECT_FAILED)
        assert "RC=1" in out and "акт не стартует" in out, (out, err)
        assert out.count("сервер не отвечает") == 3

    def test_pactl_missing_is_not_a_block(self, tmp_path):
        out, err, _ = run(tmp_path, 'PATH=/nonexistent; stand_audio_preflight; echo "RC=$?"')
        assert "RC=0" in out, (out, err)


@needs_bash
def test_workflow_and_main_flow_wire_the_stand_verdict():
    text = "\n".join(SCRIPT_LINES)
    assert "stand_audio_preflight" in text and "E2E_STAND_FAIL" in text
    assert text.count("|| { step_stand_play_failed") >= 2, "основной и backlog-PLAY обязаны проверять результат"
    marathon = (REPO_ROOT / "scripts" / "e2e" / "run_night_marathon.sh").read_text(encoding="utf-8")
    assert "E2E_STAND_FAIL" in marathon and "stand_play_failed" in marathon


@needs_bash
def test_marathon_reads_stand_count_from_summary_line(tmp_path):
    log = tmp_path / "act.log"
    log.write_text(
        "2026-11-09T10:00:00.1Z >>> E2E_SUMMARY verdict=FAIL steps=3/16 ok fail=2 skip=0 "
        "stand_play_failed=11 recovered=0 provider=silero\n",
        encoding="utf-8",
    )
    p = subprocess.run(
        [BASH, "-c", '. "%s"; marathon_summary_value "%s" stand_play_failed' % (MARKERS.as_posix(), log.as_posix())],
        capture_output=True,
        timeout=30,
    )
    assert p.stdout.decode().strip() == "11", p
