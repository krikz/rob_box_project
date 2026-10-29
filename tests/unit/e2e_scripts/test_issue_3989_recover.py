"""Issues #3989, #4027, #3710 п.2 (эпик #3683) -- харнесс марафона честно восстанавливает шаг и чистит состояние акта.

#3989. Марафон 23.10 (night_20261023_210648): n301, n704, n803, n1004 -- STT потерял вейк, фраза ушла в бэклог
(``[backlog] accumulated reason=no_wake``), а харнесс трижды ждал потолок и писал ``cycle_unconfirmed_after_accept``.
Теперь судья ``e2e_events.py`` сразу отдаёт ``no_wake`` (или ``silenced`` -- ход проглотила тишина, ``stage=silenced``),
``run_step`` делает ОДНО восстановление (``Робот!`` / ``Робот, говори`` + фраза шага), пишет ``E2E_RECOVER`` и вердикт
``OK recovered_<вид>`` -- не чистый OK; сводка считает такие шаги (``recovered=N``), раннер марафона -- колонкой.

#4027. Наследующий акт больше не пропускает проверку музыки (утечка 7 -> 8: музыка n712 играла весь акт 8), на старте
каждого акта читается тишина (``/dialog/session``); ``restore_robot_tts_default`` возвращает и скорость, и тон.

#3710 п.2. В конце акта e2e-БД дикторов копируется (только чтение, один ssh) в артефакты акта.

Робот -- ``fake_robot_ssh.sh`` (лог v2 с метками ``{NOW-x}``, файловые плеер/сессия/параметры в ``ros_stubs``).

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_3989_recover.py -v -o addopts=""
"""

from __future__ import annotations

import json
import re
import shutil
import sqlite3
import subprocess
import sys
import tarfile
import time
from pathlib import Path

import pytest

from tests.unit.e2e_scripts.test_issue_2846_registration_verdict import extract_scenario_loop
from tests.unit.e2e_scripts.test_issue_3857_event_waits import _feed, _l, accept, llm, speech, turn_done
from tests.unit.e2e_scripts.test_issue_3857_run_step import extract_function

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
E2E_SCRIPT = SCRIPTS_DIR / "e2e_voice_test.sh"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"
EVENTS = SCRIPTS_DIR / "e2e_events.py"
FAKE = Path(__file__).resolve().parent / "fake_robot_ssh.sh"
NIGHT = REPO_ROOT / ".github" / "e2e" / "scenarios" / "night"
MARATHON = REPO_ROOT / "scripts" / "e2e"

sys.path.insert(0, str(SCRIPTS_DIR))
import e2e_events  # noqa: E402

needs_bash = pytest.mark.skipif(shutil.which("bash") is None or shutil.which("python3") is None,
                                reason="нужны bash и python3")


# -- строки лога v2 (формат dialog_node / turn_log.TurnAggregate.log_line / turn.py) ----------------------

def no_wake(ts="{NOW-28}", entries=1):
    return _l("dialog_node", ts, "[backlog] accumulated reason=no_wake entries=%d" % entries)


def heard(text, ts="{NOW-30}"):
    return _l("stt_node", ts, "✅ ПРИНЯТО (respeaker): %s" % text)


def turn_stage(stage, status="finished", ts="{NOW-20}", llm_calls=0):
    return _l("dialog_node", ts, "📊 [turn] turn_id=t9 epoch=3 agent=personality stage=%s status=%s latency_ms=12 "
                                 "llm_calls=%d llm_provider=- tokens=0" % (stage, status, llm_calls))


def tier1(kind, ts="{NOW-21}"):
    return _l("dialog_node", ts, "🧭 [turn-v2] tier1 kind=%s args={} turn=t9" % kind)


# Лог n301 марафона 23.10 (va_marathon.log:3062-3064): фраза принята без вейка и ушла в бэклог, хода нет.
N301 = heard("вот мы тут сейчас немного пошумим ты просто слушай и не влезай") + no_wake()
SILENCED_TURN = heard("робот посчитай до двадцати") + turn_stage("silenced")
REPLY = llm("{NOW-9}") + speech("started", "cccc3333", "{NOW-6}") + turn_done("{NOW-5}") + \
    speech("finished", "cccc3333", "{NOW-3}", "spoken_chars=30")


# -- e2e_events: судья ------------------------------------------------------------------------------------

class TestVerdicts:
    def test_no_wake_is_final_after_settle(self):
        st = e2e_events.state(_feed(N301), "v2", False, waited=e2e_events.NO_WAKE_SETTLE_S)
        assert st["verdict"] == "no_wake" and st["reason"] == "no_wake"

    def test_no_wake_waits_settle_for_a_split_phrase(self):
        assert e2e_events.state(_feed(N301), "v2", False, waited=1)["verdict"] == "wait"

    def test_no_wake_then_turn_is_not_no_wake(self):
        """STT порезал фразу: второй кусок начал ход -- судья ждёт ход, а не восстанавливает."""
        log = N301 + heard("робот", "{NOW-12}") + REPLY
        assert e2e_events.state(_feed(log), "v2", False, waited=30)["verdict"] == "done"
        log = N301 + tier1("volume", "{NOW-12}")
        assert e2e_events.state(_feed(log), "v2", False, waited=30)["verdict"] == "wait"

    def test_backlog_before_accept_is_not_this_phrase(self):
        log = no_wake("{NOW-40}") + accept("{NOW-30}")
        assert e2e_events.state(_feed(log), "v2", False, waited=30)["verdict"] == "wait"

    def test_silenced_turn_is_final_at_once(self):
        st = e2e_events.state(_feed(SILENCED_TURN), "v2", False)
        assert st["verdict"] == "silenced" and st["reason"].startswith("silenced")

    def test_silenced_is_not_silent_ok(self):
        """silent_ok разрешает ход без речи, но проглоченный тишиной ход -- не ответ шага."""
        assert e2e_events.state(_feed(SILENCED_TURN), "v2", True)["verdict"] == "silenced"

    def test_step_that_engages_silence_is_not_recovery(self):
        """n805 «всё, хватит, замолчи»: шаг САМ включил тишину (tier1 kind=silence) -- это его проверка."""
        log = heard("робот всё хватит замолчи") + tier1("silence") + turn_stage("done")
        assert e2e_events.state(_feed(log), "v2", True)["verdict"] == "silent"

    def test_rest_of_own_silence_command_is_not_a_leak(self):
        """«хватит, замолчи, посчитай»: остаток фразы после своего «замолчи» ход проглотил уже в тишине
        (turn._tier1_then_rest -> hold_silence, stage=silenced) -- ход без речи по просьбе шага, а не утечка."""
        log = heard("робот хватит замолчи и посчитай") + tier1("stop_all") + tier1("silence") + turn_stage("silenced")
        assert e2e_events.state(_feed(log), "v2", True)["verdict"] == "silent"
        assert e2e_events.state(_feed(log), "v2", False)["verdict"] == "nospeech"

    def test_v1_judge_unchanged(self):
        assert e2e_events.state(_feed(N301), "v1", False, waited=30)["verdict"] == "wait"
        assert e2e_events.state(_feed(SILENCED_TURN), "v1", False)["verdict"] == "wait"

    def test_cli_state_reports_no_wake(self):
        out = subprocess.run([sys.executable, str(EVENTS), "state", "--engine", "v2", "--waited", "9"],
                             input=_feed(N301), capture_output=True, text=True, encoding="utf-8").stdout
        assert "verdict=no_wake" in out.splitlines()


class TestRecoverPolicy:
    def test_default_allows_both(self):
        assert e2e_events.recover_kinds({}, {"text": "Робот, который час?"}) == ("no_wake", "silenced")

    def test_act_field_and_step_override(self):
        act = {"recover": ["silenced"]}
        assert e2e_events.recover_kinds(act, {"text": "Робот, который час?"}) == ("silenced",)
        assert e2e_events.recover_kinds(act, {"text": "Робот, x", "recover": True}) == ("no_wake", "silenced")
        assert e2e_events.recover_kinds({"recover": False}, {"text": "Робот, x"}) == ()
        assert e2e_events.recover_kinds({}, {"text": "Робот, x", "recover": []}) == ()

    def test_barge_in_step_is_never_recovered(self):
        assert e2e_events.recover_kinds({}, {"text": "Робот, стоп", "barge_in": {"after": "speech_started"}}) == ()

    def test_step_that_is_the_recovery_phrase_is_not_recovered_by_it(self):
        """n805b «Робот, говори.»: снять тишину за шаг харнесс не имеет права."""
        assert e2e_events.recover_kinds({}, {"text": "Робот, говори."}) == ("no_wake",)

    def test_bad_field_is_an_error_not_silence(self):
        with pytest.raises(ValueError):
            e2e_events.recover_kinds({}, {"text": "x", "recover": ["nowake"]})
        with pytest.raises(ValueError):
            e2e_events.recover_kinds({"recover": "yes"}, {"text": "x"})

    def test_cli_policy_on_act8(self):
        out = subprocess.run([sys.executable, str(EVENTS), "policy", "--scenario",
                              str(NIGHT / "night_marathon_act8_barge_in_v1.json")],
                             capture_output=True, text=True, encoding="utf-8")
        assert out.returncode == 0, out.stderr
        rows = dict(line.split("\t") for line in out.stdout.splitlines())
        assert rows["n801_wake_open"] == "no_wake,silenced"
        assert rows["n803_barge_in_new_topic"] == "-"
        assert rows["n805b_unsilence"] == "no_wake"

    def test_cli_policy_bad_field_exits_2(self, tmp_path):
        scen = tmp_path / "s.json"
        scen.write_text(json.dumps({"steps": [{"label": "a", "text": "x", "recover": 1}]}), encoding="utf-8")
        out = subprocess.run([sys.executable, str(EVENTS), "policy", "--scenario", str(scen)],
                             capture_output=True, text=True, encoding="utf-8")
        assert out.returncode == 2 and "recover" in out.stderr

    def test_cli_recovery_phrase(self):
        out = subprocess.run([sys.executable, str(EVENTS), "recovery", "--kind", "silenced"],
                             capture_output=True, text=True, encoding="utf-8").stdout
        assert "phrase=Робот, говори." in out.splitlines() and "replay=1" in out.splitlines()


class TestSessionSilence:
    NOW = 1_790_000_000.0

    def test_live_silence_frame(self):
        scenes = [{"kind": "DJ", "set_by": "user", "ttl_until": 0}, {"kind": "Silence", "set_by": "user",
                                                                     "ttl_until": self.NOW + 300}]
        assert e2e_events.silence_frame(scenes, self.NOW)["set_by"] == "user"

    def test_expired_frame_is_not_silence(self):
        """Владелец снимает истёкшие кадры лениво (Session.expire), latched-снимок остаётся прежним."""
        assert e2e_events.silence_frame([{"kind": "Silence", "ttl_until": self.NOW - 1}], self.NOW) is None

    def test_frame_without_ttl_is_silence(self):
        assert e2e_events.silence_frame([{"kind": "Silence", "ttl_until": 0}], self.NOW) is not None

    def test_session_report(self):
        rep = {"snapshot": True, "now": self.NOW, "scenes": [{"kind": "Silence", "ttl_until": self.NOW + 60}],
               "after": []}
        want = {"fetch": 1, "silenced": 1, "after": 0, "detail": "Silence set_by=? ttl_left=60s"}
        assert e2e_events.session_report(rep) == want
        assert e2e_events.session_report({"snapshot": False, "now": self.NOW, "scenes": None})["silenced"] == 0
        assert e2e_events.session_report({})["fetch"] == 0


class TestKnowledgeDrift:
    """Харнесс живёт на katana без пакетов робота: литералы сверяются с исходниками владельцев."""

    def test_silenced_stage_and_scene(self):
        kn = (REPO_ROOT / "src/rob_box_dialog/rob_box_dialog/knowledge.py").read_text(encoding="utf-8")
        vs = (REPO_ROOT / "src/rob_box_core/rob_box_core/voice_state.py").read_text(encoding="utf-8")
        assert 'SILENCED_TURN_STAGE = "%s"' % e2e_events.SILENCED in kn
        assert 'SILENCE_SCENE = "%s"' % e2e_events.SILENCE_SCENE in vs
        tier1_kinds = re.search(r"TIER1_KINDS: Mapping\[str, bool\] = MappingProxyType\(\{(.*?)\}\)", kn, re.S).group(1)
        assert '"%s":' % e2e_events.SILENCE_TIER1 in tier1_kinds

    def test_recovery_phrases_mean_what_robot_knows(self):
        kn = (REPO_ROOT / "src/rob_box_dialog/rob_box_dialog/knowledge.py").read_text(encoding="utf-8")
        unsilence = re.search(r"UNSILENCE_WORDS: Tuple\[str, \.\.\.\] = \(([^)]*)\)", kn).group(1)
        words = e2e_events._norm(e2e_events.RECOVERY["silenced"].phrase).split()
        assert words[0] == "робот" and any(w.startswith(s.strip(' "')) for w in words[1:] for s in unsilence.split(","))
        assert e2e_events._norm(e2e_events.RECOVERY["no_wake"].phrase) == "робот"  # голое «робот» -- подхват (В12)

    def test_harness_greps_the_new_markers(self):
        lib = LIB.read_text(encoding="utf-8")
        grep = re.search(r"_E2E_EVENT_GREP='([^']+)'", lib).group(1)
        for line in (no_wake(), tier1("silence"), turn_stage("silenced")):
            assert re.search(grep, line), line


# -- bash: run_step с восстановлением на фейковом роботе ---------------------------------------------------

def _run_step(tmp_path, log_text, plays, body, env="", replace=False):
    """``plays[i]`` -- куски лога, которые робот «скажет» после (i+2)-го PLAY (первый PLAY -- фраза шага).
    ``replace`` -- кусок ЗАМЕНЯЕТ лог (окно нового PLAY начинается с чистого листа: фейк не режет ``--since``)."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    log = tmp_path / "robot.log"
    log.write_text(log_text, encoding="utf-8")
    for i, chunk in enumerate(plays, 2):
        (tmp_path / ("play_%d.log" % i)).write_text(chunk, encoding="utf-8")
    counter = tmp_path / "play_count"
    (bindir / "paplay").write_text(
        '#!/bin/bash\nn=$(( $(cat "{c}" 2>/dev/null || echo 0) + 1 ))\necho "$n" > "{c}"\necho "$1" >> "{t}/played"\n'
        '[ -f "{t}/play_$n.log" ] && cat "{t}/play_$n.log" {op} "{log}"\nexit 0\n'.format(
            c=counter.as_posix(), t=tmp_path.as_posix(), log=log.as_posix(), op=">" if replace else ">>"),
        encoding="utf-8")
    for name in ("ffmpeg", "pactl", "ffprobe"):
        (bindir / name).write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")
    for name in ("paplay", "ffmpeg", "pactl", "ffprobe"):
        (bindir / name).chmod(0o755)
    out = tmp_path / "out"
    out.mkdir()
    (out / "cmd_a_eq.wav").write_bytes(b"RIFF....WAVEfmt ")
    script = "\n".join([
        "set -u",
        'export PATH="%s:$PATH"' % bindir.as_posix(),
        'export FAKE_ROBOT_STATE="%s/state" FAKE_ROBOT_LOG="%s"' % (tmp_path.as_posix(), log.as_posix()),
        'OUT_DIR="%s"' % out.as_posix(),
        'ROBOT_SSH="bash %s"' % FAKE.as_posix(),
        'SCRIPT_DIR_E2E="%s"' % SCRIPTS_DIR.as_posix(),
        "E2E_MAX_ATTEMPTS=1; E2E_REACTION_WINDOW=40; E2E_RETRY_PAUSE=0; E2E_EVENT_POLL=1; E2E_BARGE_POLL=1",
        "E2E_DIALOG_ENGINE=v2; E2E_SILENCE_WAIT_MAX=30; E2E_FAIL_KIND=''",
        'log() { echo ">>> $*"; }',
        LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
        *(extract_function(name) for name in (
            "ensure_outdir", "emit_step", "mark_fail_kind", "vad_reject_reason", "emit_step_fail_or_vad",
            "check_cycle", "eq_command_wav", "play_eq_wav", "phrase_eq_wav", "step_recover", "step_wake_replay",
            "check_patterns", "check_music_compare", "_stt_step_executed", "step_stt_replay", "run_step")),
        # синтез: фраза восстановления -- файл с её текстом (по нему видно, ЧТО сыграно)
        'synth_command() { printf "%s" "$1" > "$3"; }',
        env,
        body,
        'echo "PLAY_COUNT=$(cat "%s" 2>/dev/null || echo 0)"' % counter.as_posix(),
        'echo "RECOVERED=${E2E_STEP_RECOVERED:-}"',
    ])
    t0 = time.time()
    p = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=170)
    played = (tmp_path / "played").read_text(encoding="utf-8").split() if (tmp_path / "played").exists() else []
    return p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace"), time.time() - t0, played


STEP = 'run_step "Робот, мы тут немного пошумим" anton a cycle; echo "RC=$?"'
BOTH = "E2E_STEP_RECOVER_KINDS=no_wake,silenced; E2E_STEP_RECOVERED=''"


@needs_bash
class TestRunStepRecovery:
    def test_no_wake_says_bare_wake_once_and_marks_step(self, tmp_path):
        pickup = heard("робот", "{NOW-12}") + REPLY
        out, err, took, played = _run_step(tmp_path, N301, [pickup], STEP, BOTH)
        assert "RC=0" in out, (out, err)
        assert "E2E_RECOVER no_wake a " in out
        assert "RECOVERED=no_wake" in out and "PLAY_COUNT=2" in out
        assert played[0].endswith("cmd_a_eq.wav") and played[1].endswith("rec_no_wake_a_eq.wav")
        assert (tmp_path / "out" / "rec_no_wake_a.wav").read_text(encoding="utf-8") == "Робот!"
        assert took < 40, "восстановление шло %.1f с -- раньше три потолка по 40 с" % took

    def test_silenced_unsilences_then_replays_the_step(self, tmp_path):
        unsilence = heard("робот говори", "{NOW-14}") + tier1("unsilence", "{NOW-13}") + turn_stage("done", ts="{NOW-12}")
        replay = heard("робот мы тут немного пошумим", "{NOW-11}") + REPLY
        out, err, took, played = _run_step(tmp_path, SILENCED_TURN, [unsilence, replay], STEP, BOTH)
        assert "RC=0" in out, (out, err)
        assert "E2E_RECOVER silenced a " in out and "RECOVERED=silenced" in out
        assert "PLAY_COUNT=3" in out
        assert played[1].endswith("rec_silenced_a_eq.wav") and played[2].endswith("cmd_a_eq.wav")
        assert (tmp_path / "out" / "rec_silenced_a.wav").read_text(encoding="utf-8") == "Робот, говори."

    def test_unsilence_that_fails_is_fail_not_replay(self, tmp_path):
        still = heard("робот говори", "{NOW-14}") + turn_stage("silenced", ts="{NOW-12}")
        out, err, _, _ = _run_step(tmp_path, SILENCED_TURN, [still], STEP, BOTH)
        assert "RC=1" in out and "PLAY_COUNT=2" in out, (out, err)
        assert "E2E_STEP a FAIL turn_silenced after_recover_silenced" in out

    def test_not_allowed_is_honest_fail_in_seconds(self, tmp_path):
        out, err, took, _ = _run_step(tmp_path, N301, [], STEP, "E2E_STEP_RECOVER_KINDS=''; E2E_STEP_RECOVERED=''")
        assert "RC=1" in out and "PLAY_COUNT=1" in out, (out, err)
        assert "E2E_STEP a FAIL turn_no_wake (no_wake)" in out and "E2E_RECOVER" not in out
        assert took < 25, took

    def test_recovery_that_did_not_help_is_fail_not_second_recovery(self, tmp_path):
        out, err, _, _ = _run_step(tmp_path, N301, [heard("робот", "{NOW-12}")], STEP, BOTH)
        assert "RC=1" in out and "PLAY_COUNT=2" in out, (out, err)
        assert out.count("E2E_RECOVER ") == 1
        assert "E2E_STEP a FAIL turn_no_wake after_recover_no_wake" in out

    def test_only_one_recovery_per_step_across_attempts(self, tmp_path):
        """E2E_MAX_ATTEMPTS=2: «Робот!» не услышан (no_accept) -> попытка 2 снова теряет вейк -> второго «Робот!»
        нет (ревью #4034: прежний тест не ловил удаление этой проверки -- run_step звал step_recover один раз)."""
        env = BOTH + "; E2E_MAX_ATTEMPTS=2; E2E_ACCEPT_WINDOW=3"
        out, err, _, played = _run_step(tmp_path, N301, ["", N301], STEP, env, replace=True)
        assert "RC=1" in out and "PLAY_COUNT=3" in out, (out, err)
        assert out.count("E2E_RECOVER ") == 1, out
        assert "восстановление шага уже было (no_wake), второго нет" in out
        assert [p.rsplit("/", 1)[-1] for p in played] == ["cmd_a_eq.wav", "rec_no_wake_a_eq.wav", "cmd_a_eq.wav"]
        assert "E2E_STEP a FAIL turn_no_wake after_recover_no_wake" in out

    def test_only_one_recovery_per_step_across_retry_acceptance(self, tmp_path):
        """Повтор шага (retry_acceptance) зовёт run_step с той же меткой: шаг уже восстановлен -- второго нет."""
        pickup = heard("робот", "{NOW-12}") + REPLY
        body = STEP.replace('echo "RC=$?"', 'echo "RC1=$?"') + "\n" + STEP.replace('echo "RC=$?"', 'echo "RC2=$?"')
        out, err, _, played = _run_step(tmp_path, N301, [pickup, N301], body, BOTH, replace=True)
        assert "RC1=0" in out and "RC2=1" in out and "PLAY_COUNT=3" in out, (out, err)
        assert out.count("E2E_RECOVER ") == 1, out
        assert "E2E_STEP a FAIL turn_no_wake after_recover_no_wake" in out


# -- bash: итог шага в scenario-цикле и сводка -------------------------------------------------------------

@needs_bash
def test_scenario_loop_marks_recovered_step_and_summary_counts_it(tmp_path):
    scen = tmp_path / "scenario.json"
    scen.write_text(json.dumps({"steps": [
        {"label": "n301_wake_open", "text": "Робот, мы тут пошумим", "expect": "cycle"},
        {"label": "n302_plain", "text": "Робот, привет", "expect": "cycle"},
    ]}, ensure_ascii=False), encoding="utf-8")
    out_dir = tmp_path / "out"
    out_dir.mkdir()
    script = "\n".join([
        "set -u",
        'OUT_DIR="%s"' % out_dir.as_posix(),
        'SCRIPT_DIR_E2E="%s"' % SCRIPTS_DIR.as_posix(),
        'ROBOT_SSH="true"; PASS=1; WAKE_GATE_SKIPPED_STEPS=0; E2E_RETRY_PAUSE=0; E2E_FAIL_KIND=""',
        'WAKE_GATE_PREFLIGHT_FILE="$OUT_DIR/wake_gate_preflight.json"',
        'log() { echo ">>> $*"; }',
        LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
        (SCRIPTS_DIR / "e2e_voice_wake_gate.sh").read_text(encoding="utf-8").replace("\r\n", "\n"),
        extract_function("ensure_outdir"),
        extract_function("emit_step"),
        extract_function("mark_fail_kind"),
        extract_function("check_music_compare"),  # #3971: сверка вида результата
        extract_function("write_summary"),
        # run_step: первый шаг восстановлен (как сделал бы step_recover), второй -- чистый
        'run_step() { case "$3" in n301*) E2E_STEP_RECOVERED=no_wake ;; esac; '
        'echo "KINDS[$3]=${E2E_STEP_RECOVER_KINDS:-}"; return 0; }',
        "parse_transcript() { :; }",
        'load_recover_policy "%s"' % scen.as_posix(),
        'parse_scenario_to_tsv "%s" "$OUT_DIR/scenario_parsed.txt"' % scen.as_posix(),
        'LAST_STEP_SPEECH_FILE="$OUT_DIR/.last_step_speech.txt"; : > "$LAST_STEP_SPEECH_FILE"',
        extract_scenario_loop(),
        "E2E_ACT_RECOVERED=music",
        "write_summary",
    ])
    p = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=120)
    out, err = p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace")
    assert "E2E_STEP n301_wake_open OK recovered_no_wake" in out, (out, err)
    assert "E2E_STEP n302_plain OK" in out.splitlines()
    assert "KINDS[n301_wake_open]=no_wake,silenced" in out
    assert re.search(r"E2E_SUMMARY verdict=PASS steps=2/2 ok fail=0 skip=0 recovered=1 ", out), out
    assert "E2E_SUMMARY act_recover=music" in out
    summary = json.loads((out_dir / "summary.json").read_text(encoding="utf-8"))
    assert summary["steps"]["recovered"] == 1 and summary["steps"]["recovered_labels"] == ["n301_wake_open"]
    assert summary["act_recover"] == ["music"]


# -- bash: старт акта -- тишина и музыка (#3989 п.5, #4027) -----------------------------------------------

SESSION_SILENCED = {"agent": "personality", "scenes": [{"kind": "Silence", "set_by": "user", "ttl_until": 0}]}


def _act_start(tmp_path, body, *, session=None, music=None, env=None):
    state = tmp_path / "state"
    state.mkdir()
    if session is not None:
        (state / "session_state.json").write_text(json.dumps(session), encoding="utf-8")
    if music is not None:
        (state / "music_state.json").write_text(music, encoding="utf-8")
    script = "\n".join([
        "set -u",
        'export FAKE_ROBOT_STATE="%s"' % state.as_posix(),
        *("export %s='%s'" % kv for kv in (env or {}).items()),
        'ROBOT_SSH="bash %s"' % FAKE.as_posix(),
        'SCRIPT_DIR_E2E="%s"' % SCRIPTS_DIR.as_posix(),
        "E2E_MUSIC_IDLE_WAIT=3; E2E_UNSILENCE_WAIT=3; E2E_DIALOG_ENGINE=v2; E2E_ACT_RECOVERED=''",
        'log() { echo ">>> $*"; }',
        LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
        extract_function("robot_ros"),
        body,
        'echo "ACT_RECOVERED=${E2E_ACT_RECOVERED}"',
    ])
    p = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=120)
    return p.returncode, p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace"), state


def _utterances(state):
    f = state / "utterances"
    return [json.loads(ln) for ln in f.read_text(encoding="utf-8").splitlines()] if f.exists() else []


@needs_bash
class TestActStartSilence:
    def test_silence_left_by_previous_act_is_lifted_and_marked(self, tmp_path):
        rc, out, err, state = _act_start(tmp_path, "ensure_not_silenced_for_act", session=SESSION_SILENCED)
        assert rc == 0, (out, err)
        # ревью #4034: один вид маркера -- тот же вид «silenced», что у шага (E2E_RECOVER silenced <шаг>)
        assert "E2E_RECOVER silenced at_act_start (было: Silence set_by=user" in out and "ACT_RECOVERED=silenced" in out
        assert "E2E_RECOVER silence " not in out
        assert [u["text"] for u in _utterances(state)] == ["Робот, говори."]
        assert _utterances(state)[0]["source"] == "respeaker"

    def test_no_silence_touches_nothing(self, tmp_path):
        rc, out, err, state = _act_start(tmp_path, "ensure_not_silenced_for_act",
                                         session={"agent": "personality", "scenes": []})
        assert rc == 0 and "E2E_PRECONDITION silence=off" in out and "ACT_RECOVERED=\n" in out + "\n", (out, err)
        assert _utterances(state) == []

    def test_silence_that_will_not_lift_is_fatal(self, tmp_path):
        rc, out, err, _ = _act_start(tmp_path, "ensure_not_silenced_for_act", session=SESSION_SILENCED,
                                     env={"FAKE_UNSILENCE_STUCK": "1"})
        assert rc == 2 and "E2E_FATAL: перед актом робот в тишине" in err, (rc, out, err)

    def test_never_published_session_is_no_silence(self, tmp_path):
        """Тишина -- изменение сессии, а каждое изменение уходит latched: нет снимка -- тишину не ставили."""
        rc, out, err, state = _act_start(tmp_path, "ensure_not_silenced_for_act")
        assert rc == 0 and "E2E_PRECONDITION silence=off" in out and "снимка нет" in out, (out, err)
        assert _utterances(state) == []

    def test_unreadable_session_is_loud_not_fatal(self, tmp_path):
        rc, out, err, _ = _act_start(tmp_path, 'robot_ros() { echo "Traceback: ImportError"; }; '
                                               "ensure_not_silenced_for_act", session=SESSION_SILENCED)
        assert rc == 0 and "E2E_PRECONDITION silence=unknown" in out, (out, err)

    def test_v1_engine_has_no_scene(self, tmp_path):
        rc, out, err, state = _act_start(tmp_path, "E2E_DIALOG_ENGINE=v1; ensure_not_silenced_for_act",
                                         session=SESSION_SILENCED)
        assert rc == 0 and "silence" not in out.replace("ACT_RECOVERED", ""), (out, err)
        assert _utterances(state) == []


PLAYING = json.dumps({"state": "playing", "track_id": "set1:3:abc", "form_ends_at": None, "stops_at": None,
                      "dj": {"enabled": True}, "finished_track_id": None, "ts": time.time()})


@needs_bash
class TestActStartMusic:
    def test_inherited_act_stops_leaked_music_and_marks_it(self, tmp_path):
        """Утечка 7 -> 8: n712 не остановил музыку, акт 8 (inherits_previous_act) шёл под неё."""
        rc, out, err, state = _act_start(tmp_path, 'ensure_music_stopped_for_act yes', music=PLAYING)
        assert rc == 0, (out, err)
        assert "E2E_RECOVER music at_act_start" in out and "ACT_RECOVERED=music" in out
        assert "E2E_PRECONDITION music=skipped" not in out
        calls = [json.loads(ln)["tool"] for ln in (state / "mcp_calls").read_text(encoding="utf-8").splitlines()]
        assert calls == ["dj_set", "stop_music"]

    def test_fresh_act_keeps_precondition_marker(self, tmp_path):
        rc, out, err, _ = _act_start(tmp_path, 'ensure_music_stopped_for_act no', music=PLAYING)
        assert rc == 0 and "E2E_PRECONDITION music=stopped" in out and "E2E_RECOVER" not in out, (out, err)


# -- bash: скорость и тон TTS (#4027) ---------------------------------------------------------------------

def _prosody(body_lines, tmp_path):
    """Старт акта -> ``remember_node_params /tts_node`` (как главный поток), дальше ``body_lines``."""
    return "\n".join([
        'E2E_RUN_BEFORE=""; E2E_NODE_PARAM_ORIGINALS_FILE="%s/originals.tsv"' % tmp_path.as_posix(),
        ': > "$E2E_NODE_PARAM_ORIGINALS_FILE"',
        "remember_node_params /tts_node $E2E_TTS_PROSODY_PARAMS",
        *body_lines,
    ])


@needs_bash
def test_restore_returns_speed_and_pitch_to_values_before_act(tmp_path):
    """#4027 одним механизмом (ревью #4034): значения до акта -- в файл node_params, возврат -- restore_node_params."""
    body = _prosody([
        # акт 4: n409/n410 «медленнее/тараторь», n411 «басом»
        "robot_ros \"ros2 param set /tts_node yandex_speed 1.2 --no-daemon\" >/dev/null",
        "robot_ros \"ros2 param set /tts_node pitch_shift 0.8 --no-daemon\" >/dev/null",
        "restore_robot_tts_default",
        ': > "$FAKE_ROBOT_STATE/ssh_calls"',
        "restore_node_params",
        'cp "$FAKE_ROBOT_STATE/ssh_calls" "$FAKE_ROBOT_STATE/restore_calls"',
        'for p in yandex_speed minimax_speed pitch_shift; do echo "$p=$(_ros2_param_value "$(_ros2_param_get_raw /tts_node $p)")"; done',
    ], tmp_path)
    rc, out, err, state = _act_start(tmp_path, body)
    assert rc == 0, (out, err)
    # чтение, одна пачка записи изменённых, проверка чтением -- три ssh, а не по паре на параметр
    assert len((state / "restore_calls").read_text(encoding="utf-8").splitlines()) == 3
    assert "yandex_speed=1.0" in out and "pitch_shift=1.0" in out and "minimax_speed=1.0" in out
    assert "node_params: /tts_node yandex_speed восстановлен к исходному 1.0" in out, out
    assert "node_params: /tts_node pitch_shift восстановлен к исходному 1.0" in out
    assert "/tts_node minimax_speed за акт не менялся (1.0)" in out  # не менялся -- не пишем
    originals = (tmp_path / "originals.tsv").read_text(encoding="utf-8").splitlines()
    assert originals == ["/tts_node\x1f%s\x1f1.0" % p for p in ("minimax_speed", "yandex_speed", "pitch_shift")]


@needs_bash
def test_restore_without_changes_does_not_set(tmp_path):
    body = _prosody([': > "$FAKE_ROBOT_STATE/ssh_calls"', "restore_node_params"], tmp_path)
    rc, out, err, state = _act_start(tmp_path, body)
    assert rc == 0 and out.count("за акт не менялся") == 3, (out, err)
    # одно чтение трёх параметров (один ssh), ни одной записи -- как у прежнего отдельного механизма
    calls = (state / "ssh_calls").read_text(encoding="utf-8").splitlines()
    assert len(calls) == 1 and not any("param set" in c or "param\\ set" in c for c in calls), calls


@needs_bash
def test_remember_reads_all_params_in_one_ssh(tmp_path):
    body = "\n".join([': > "$FAKE_ROBOT_STATE/ssh_calls"', _prosody([], tmp_path)])
    rc, out, err, state = _act_start(tmp_path, body)
    assert rc == 0 and "/tts_node до акта: minimax_speed=1.0, yandex_speed=1.0, pitch_shift=1.0" in out, (out, err)
    assert len((state / "ssh_calls").read_text(encoding="utf-8").splitlines()) == 1


def test_prosody_has_one_mechanism():
    """Ревью #4034: второй механизм возврата скорости/тона (_tts_prosody_read, snapshot_tts_prosody,
    _restore_tts_prosody) удалён; restore_robot_tts_default скорость/тон не трогает."""
    lib = LIB.read_text(encoding="utf-8")
    for gone in ("_tts_prosody_read", "_tts_prosody_value", "snapshot_tts_prosody", "_restore_tts_prosody",
                 "E2E_TTS_PROSODY_FILE"):
        assert gone not in lib, gone
    restore_voice = re.search(r"^restore_robot_tts_default\(\) \{(.*?)^\}", lib, re.S | re.M).group(1)
    assert "param set" not in restore_voice


# -- bash: e2e-БД дикторов в артефакты (#3710 п.2) --------------------------------------------------------

CAST = ["Саша", "Борис", "Гриша", "Валера"]


def _speaker_db(tmp_path):
    """e2e-БД как у speaker_id_node (speaker_embeddings._CREATE_SQL + эпитет): каст, клон Бориса, безымянный профиль
    и человек мастерской, которого speaker_id_node тоже услышал."""
    data = tmp_path / "data"
    data.mkdir()
    db = data / "speakers.e2e.db"
    con = sqlite3.connect(str(db))
    con.execute("create table speakers (speaker_id text, name text, created_at real, epithet text, tags text)")
    con.execute("create table embeddings (id integer primary key, speaker_id text, embedding blob, created_at real)")
    con.executemany("insert into speakers values (?, ?, 1.0, ?, ?)", [
        ("429eae2c", "Саша", "ночной инженер", "паяльник"),
        ("dc417cef", "Борис", None, None),
        ("ab12cd34", "борис", None, None),  # клон роли: регистр не важен
        ("ee0000ff", "", None, None),  # безымянный профиль (speaker_embeddings.UNNAMED)
        ("f00dbabe", "Иван Петрович", "сосед по цеху", "шахматы"),
    ])
    con.execute("insert into embeddings (speaker_id, embedding, created_at) values ('429eae2c', x'0102', 1.0)")
    con.commit()
    con.close()
    return db


@needs_bash
@pytest.mark.parametrize("with_cast", [True, False], ids=["cast", "no_cast_field"])
def test_speaker_db_copied_read_only_and_public_json_has_no_outsider_names(tmp_path, with_cast):
    """Ревью #4034: json уходит в артефакты публичного репо -- в нём число строк и имена ролей каста, не больше."""
    db = _speaker_db(tmp_path)
    before = db.read_bytes()
    scen = tmp_path / "scenario.json"
    scen.write_text(json.dumps({"cast": CAST, "steps": []} if with_cast else {"steps": []}, ensure_ascii=False),
                    encoding="utf-8")
    art = tmp_path / "art"
    art.mkdir()
    body = "\n".join([
        'E2E_ACT_STARTED=1; E2E_SPEAKER_DB="%s"' % db.as_posix(),
        ': > "$FAKE_ROBOT_STATE/ssh_calls"',
        'collect_e2e_speaker_db "%s" "%s"' % (art.as_posix(), scen.as_posix()),
    ])
    rc, out, err, state = _act_start(tmp_path, body)
    assert rc == 0, (out, err)
    assert len((state / "ssh_calls").read_text(encoding="utf-8").splitlines()) == 1
    with tarfile.open(str(art / "speakers_e2e_db.tar")) as tf:
        assert tf.getnames() == ["speakers.e2e.db"]
    raw = (art / "speakers_e2e_db.json").read_text(encoding="utf-8")
    summary = json.loads(raw)
    assert summary["tables"] == {"embeddings": {"count": 1}, "speakers": {"count": 5}}
    if with_cast:
        assert summary["speakers"] == {"cast": {"Саша": 1, "Борис": 2}, "unnamed": 1, "masked": 1}
    else:  # нет поля cast -- скрыты все имена
        assert summary["speakers"] == {"cast": {}, "unnamed": 1, "masked": 4}
        assert "Саша" not in raw
    for private in ("Иван", "Петрович", "сосед", "шахмат", "паяльник", "ночной инженер", "429eae2c", "f00dbabe"):
        assert private not in raw, private
        assert private not in out, private
    assert "E2E_SPEAKER_DB copied" in out
    assert db.read_bytes() == before


@needs_bash
def test_speaker_db_ssh_that_hangs_does_not_hold_the_trap(tmp_path):
    """Ревью #4034: копия БД идёт в trap EXIT -- зависший ssh не держит выход акта (timeout, E2E_SPEAKER_DB_TIMEOUT)."""
    slow = tmp_path / "slow_ssh.sh"
    slow.write_text("#!/bin/bash\nexec sleep 30\n", encoding="utf-8")
    art = tmp_path / "art"
    art.mkdir()
    body = "\n".join([
        'E2E_ACT_STARTED=1; E2E_SPEAKER_DB_TIMEOUT=2; ROBOT_SSH="bash %s"' % slow.as_posix(),
        'collect_e2e_speaker_db "%s" ""' % art.as_posix(),
    ])
    t0 = time.time()
    rc, out, err, _ = _act_start(tmp_path, body)
    took = time.time() - t0
    assert rc == 0 and "ssh не ответил за 2 с" in out and "E2E_SPEAKER_DB none" in out, (out, err)
    assert took < 20, took
    assert not (art / "speakers_e2e_db.tar").exists()


def test_trap_collects_db_with_timeout_and_scenario_cast():
    lib = LIB.read_text(encoding="utf-8")
    body = re.search(r"^collect_e2e_speaker_db\(\) \{(.*?)^\}", lib, re.S | re.M).group(1)
    assert 'timeout "$wait_s" ${ROBOT_SSH}' in body and 'timeout "$wait_s" python3' in body
    assert '"rows"' not in body and "SELECT * FROM" not in body  # строки таблиц не выносятся


@needs_bash
def test_speaker_db_refuses_non_e2e_path(tmp_path):
    rc, out, err, state = _act_start(tmp_path, 'E2E_ACT_STARTED=1; E2E_SPEAKER_DB=/data/speakers.db; '
                                               ': > "$FAKE_ROBOT_STATE/ssh_calls"; collect_e2e_speaker_db "%s"'
                                     % tmp_path.as_posix())
    assert rc == 0 and "боевую не копирую" in out, (out, err)
    assert (state / "ssh_calls").read_text(encoding="utf-8") == ""


# -- главный поток и раннер марафона ----------------------------------------------------------------------

def test_main_flow_wires_policy_act_state_and_db_copy():
    src = E2E_SCRIPT.read_text(encoding="utf-8").replace("\r\n", "\n")
    assert 'ensure_music_stopped_for_act "$_music_inherits"' in src
    assert src.index('ensure_music_stopped_for_act "$_music_inherits"') < src.index("    ensure_not_silenced_for_act\n")
    trap = next(line for line in src.splitlines() if line.startswith("trap 'restore_robot_tts_default"))
    assert 'collect_e2e_speaker_db "$OUT_DIR" "${SCENARIO_FILE:-}"' in trap
    # скорость/тон -- после голоса: set_voice не перебьёт возвращённые значения
    assert trap.index("restore_robot_tts_default") < trap.index("restore_node_params")
    assert src.index("remember_node_params /tts_node $E2E_TTS_PROSODY_PARAMS") < src.index(trap)
    assert src.index('load_recover_policy "$SCENARIO_FILE"') < src.index("start_recording\n")
    assert src.index('load_skip_rules "$SCENARIO_FILE"') < src.index("start_recording\n")


def test_recover_marker_has_one_form():
    """Ревью #4034: «E2E_RECOVER silenced <шаг>» и «E2E_RECOVER silence at_act_start» -- один вид, одна функция."""
    assert e2e_events.recover_marker("silenced", "n805c", "phrase=x") == "E2E_RECOVER silenced n805c phrase=x"
    assert e2e_events.recover_marker("silenced", "at_act_start", "(было: Silence)") == \
        "E2E_RECOVER silenced at_act_start (было: Silence)"
    assert e2e_events.recover_marker("music", "at_act_start") == "E2E_RECOVER music at_act_start"
    for kind, where in (("silence", "at_act_start"), ("music", "n712"), ("no_wake", "at_act_start"), ("silenced", "")):
        with pytest.raises(ValueError):
            e2e_events.recover_marker(kind, where)
    assert set(e2e_events.ACT_RECOVERY) - {"music"} <= set(e2e_events.RECOVERY)  # тишина -- тем же видом


def test_cli_mark_refuses_foreign_kind():
    ok = subprocess.run([sys.executable, str(EVENTS), "mark", "--kind", "no_wake", "--where", "n301",
                         "--detail", "phrase=«Робот!» replay=0"], capture_output=True, text=True, encoding="utf-8")
    assert ok.returncode == 0 and ok.stdout == "E2E_RECOVER no_wake n301 phrase=«Робот!» replay=0\n"
    bad = subprocess.run([sys.executable, str(EVENTS), "mark", "--kind", "silence", "--where", "at_act_start"],
                         capture_output=True, text=True, encoding="utf-8")
    assert bad.returncode == 2 and bad.stdout == "" and "silence" in bad.stderr


def test_harness_prints_recover_markers_only_through_recover_mark():
    """Все места маркера -- через recover_mark; литеральные виды в вызовах -- из словаря e2e_events."""
    lib = LIB.read_text(encoding="utf-8")
    src = lib + E2E_SCRIPT.read_text(encoding="utf-8")
    assert not re.search(r'echo "E2E_RECOVER', src), "маркер мимо recover_mark"
    calls = re.findall(r"^\s*recover_mark (\S+) (\S+)", src, re.M)
    assert {c for c in calls} >= {("music", "at_act_start"), ("silenced", "at_act_start"), ('"$kind"', '"$label"')}
    for kind, where in calls:
        if not kind.startswith('"$'):
            e2e_events.recover_marker(kind, where)  # ValueError -- чужой вид


@needs_bash
def test_recover_mark_with_foreign_kind_is_loud_and_not_counted(tmp_path):
    rc, out, err, _ = _act_start(tmp_path, "recover_mark silence at_act_start '(x)'")
    assert rc == 0 and not any(line.startswith("E2E_RECOVER") for line in out.splitlines()), (out, err)
    assert "❌ маркер восстановления не записан" in out and "ACT_RECOVERED=" in out.splitlines()


@needs_bash
def test_marathon_summary_value_reads_harness_summary(tmp_path):
    log = tmp_path / "act8.log"
    log.write_text(
        "2026-10-23T22:55:47.8157749Z >>> E2E_SUMMARY verdict=FAIL steps=12/14 ok fail=2 skip=0 recovered=2 "
        "provider=minimax\n"
        "2026-10-23T22:55:47.8158257Z >>> E2E_SUMMARY act_recover=music,silenced\n"
        "2026-10-23T22:55:48.0000000Z \x1b[36;1m echo \"E2E_SUMMARY recovered=99\"\n", encoding="utf-8")
    lib, path = (MARATHON / "marathon_markers.sh").as_posix(), log.as_posix()
    script = ('. "{lib}"; marathon_summary_value "{log}" recovered; echo; marathon_summary_value "{log}" act_recover; '
              'echo; marathon_summary_value "{log}" nothing').format(lib=lib, log=path)
    out = subprocess.run(["bash", "-c", script], capture_output=True, text=True, encoding="utf-8").stdout
    assert out.splitlines() == ["2", "music,silenced", "-"]


def test_marathon_summary_tsv_has_recovered_column():
    src = (MARATHON / "run_night_marathon.sh").read_text(encoding="utf-8")
    assert "printf 'act\\ttitle\\tstability\\tsteps\\trun_id\\tconclusion\\turl\\trecovered\\tact_recover\\n'" in src
    assert 'marathon_summary_value "$act_log" recovered' in src
    # лог акта забирается ДО строки сводки: иначе колонку не из чего заполнить
    assert src.index('gh run view "$run_id" --repo "$REPO_SLUG" --log') < \
        src.index('"$a_n" "$a_title" "$a_stab" "$a_steps" "$run_id" "$conclusion" "$url"')
