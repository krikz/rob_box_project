"""Issue #3857 -- НАСТОЯЩИЙ ``run_step`` на фейковом роботе v2: ответ за секунды, а не 40 с; перебивание.

Исполняются настоящие ``run_step``/``check_cycle`` из ``e2e_voice_test.sh`` и функции ожидания из
``e2e_voice_lib.sh``; робот -- ``fake_robot_ssh.sh`` с логом v2 (``{NOW-x}`` -- «x секунд назад»).

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_3857_run_step.py -v
"""

from __future__ import annotations

import re
import shutil
import subprocess
import time
from pathlib import Path

import pytest

from tests.unit.e2e_scripts.test_issue_3857_event_waits import (accept, llm, speech, turn_done, _l)

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
E2E_SCRIPT = SCRIPTS_DIR / "e2e_voice_test.sh"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"
FAKE = Path(__file__).resolve().parent / "fake_robot_ssh.sh"
SCRIPT_LINES = E2E_SCRIPT.read_text(encoding="utf-8").replace("\r\n", "\n").split("\n")

needs_bash = pytest.mark.skipif(shutil.which("bash") is None or shutil.which("python3") is None,
                                reason="нужны bash и python3")

SPEC = '{"after":"speech_started","n":1,"delay_sec":2.0}'
TTS_FINISHED = _l("tts_node", "{NOW-1}", "📢 Публикую TTS finished event: speech_id=aaaa1111..., success=True, duration=3.1s")
# Ответ целиком: принято 30 с назад, речь началась 3 с назад, ход закончен, речь закончилась 1 с назад.
LOG = (accept("{NOW-30}") + llm("{NOW-29}") + speech("started", "aaaa1111", "{NOW-3}") + turn_done("{NOW-2}")
       + speech("finished", "aaaa1111", "{NOW-1}", "spoken_chars=40") + TTS_FINISHED)


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
            return "\n".join(SCRIPT_LINES[start:i + 1])
    raise AssertionError(name)


def _run(tmp_path, steps_body, log_text=LOG):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    counter = tmp_path / "play_count"
    (bindir / "paplay").write_text(
        '#!/bin/bash\nn=$(( $(cat "%s" 2>/dev/null || echo 0) + 1 ))\necho "$n" > "%s"\nexit 0\n'
        % (counter.as_posix(), counter.as_posix()), encoding="utf-8")
    for name in ("paplay", "ffmpeg", "pactl", "ffprobe"):
        p = bindir / name
        if name != "paplay":
            p.write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")
        p.chmod(0o755)
    out = tmp_path / "out"
    out.mkdir()
    for label in ("a", "b"):
        (out / ("cmd_%s_eq.wav" % label)).write_bytes(b"RIFF....WAVEfmt ")
    log = tmp_path / "robot.log"
    log.write_text(log_text, encoding="utf-8")
    script = "\n".join([
        "set -u",
        'export PATH="%s:$PATH"' % bindir.as_posix(),
        'export FAKE_ROBOT_STATE="%s/state" FAKE_ROBOT_LOG="%s"' % (tmp_path.as_posix(), log.as_posix()),
        'OUT_DIR="%s"' % out.as_posix(),
        'ROBOT_SSH="bash %s"' % FAKE.as_posix(),
        'SCRIPT_DIR_E2E="%s"' % SCRIPTS_DIR.as_posix(),
        "E2E_MAX_ATTEMPTS=1; E2E_REACTION_WINDOW=40; E2E_RETRY_PAUSE=0; E2E_EVENT_POLL=1; E2E_BARGE_POLL=1",
        "E2E_DIALOG_ENGINE=v2; E2E_SILENCE_WAIT_MAX=30",
        'log() { echo ">>> $*"; }',
        LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
        extract_function("ensure_outdir"),
        extract_function("emit_step"),
        extract_function("mark_fail_kind"),
        extract_function("vad_reject_reason"),
        extract_function("emit_step_fail_or_vad"),
        extract_function("check_cycle"),
        extract_function("run_step"),
        "synth_command() { : ; }",
        steps_body,
        'echo "PLAY_COUNT=$(cat "%s" 2>/dev/null || echo 0)"' % counter.as_posix(),
    ])
    t0 = time.time()
    p = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=170)
    return p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace"), time.time() - t0


@needs_bash
class TestRunStepEventWaits:
    def test_step_finishes_in_seconds_and_logs_reply_and_silence(self, tmp_path):
        out, err, took = _run(tmp_path, 'run_step "Робот привет" anton a cycle; echo "RC=$?"')
        assert "RC=0" in out, (out, err)
        assert took < 15, "шаг шёл %.1f с (раньше >= 40+35 с)" % took
        assert re.search(r"STEP a: reply_after=\d+s, silence_after=\d+s \(wait=event\)", out), out
        assert "ПОЛНЫЙ ЦИКЛ" in out

    def test_barge_in_pair_plays_over_speech_and_logs_after_started(self, tmp_path):
        body = "\n".join([
            "E2E_STEP_NEXT_BARGE_IN='%s'; E2E_STEP_BARGE_IN=''" % SPEC,
            'run_step "Робот считай" anton a cycle; echo "RC_A=$?"',
            "E2E_STEP_NEXT_BARGE_IN=''; E2E_STEP_BARGE_IN='%s'" % SPEC,
            'run_step "Робот стоп" anton b cycle; echo "RC_B=$?"',
        ])
        out, err, took = _run(tmp_path, body)
        assert "RC_A=0" in out and "RC_B=0" in out, (out, err)
        assert "PLAY_COUNT=2" in out
        assert "якорь перебивания следующего шага наступил" in out  # шаг A не ждал конца ответа
        m = re.search(r"STEP b: barge_in after_started=([0-9.]+)s \(speech_id=aaaa1111", out)
        assert m, out
        assert float(m.group(1)) >= 2.0  # не раньше delay_sec от начала речи
        assert "робот молчит" not in out.split("STEP b:")[1].split("PLAY attempt")[0], "перебивание не должно ждать тишины"
        assert took < 25

    def test_missing_anchor_is_skip_not_pass(self, tmp_path):
        log = accept("{NOW-30}") + turn_done("{NOW-2}")  # речи не было вовсе
        body = ("E2E_REACTION_WINDOW=3; E2E_LAST_PLAY_SINCE=2026-01-01T00:00:00Z; "
                "E2E_STEP_BARGE_IN='%s'; run_step \"Робот стоп\" anton b cycle; echo \"RC_B=$?\"" % SPEC)
        out, err, took = _run(tmp_path, body, log)
        assert "RC_B=4" in out, (out, err)
        assert "SKIP barge_in_anchor_missing" in out
        assert "PLAY_COUNT=0" in out, "перебивать нечего -- реплику играть нельзя"
