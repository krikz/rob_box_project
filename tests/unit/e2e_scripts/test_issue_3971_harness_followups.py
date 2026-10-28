"""Хвосты ревью PR #4046 (#3971, эпик #3683): харнесс марафона сам ждёт выступление и сам пропускает шаг без предусловия.

n412 (сказка, 23.10): выступление шло 94-162 с, а харнесс ждал ответа 3 × 40 с -- ``cycle_unconfirmed_after_accept``
за 4 с до конца сказки. Теперь судья ``e2e_events.py`` видит старт выступления (``выступление старт kind=… 'pending'``)
и его конец (``[perform] …: конец`` / ``PlanEvent block_end block=perform-…``): пока оно идёт, ответ не готов, потолок --
``PERFORM_CEILING_S`` (240 с) от старта по часам робота; не кончилось -- ``perform_ceiling``, FAIL без повтора фразы.

Акт 7 (R21, #4077): поле шага ``skip_when: {"reason": "NO_SONG", "log": <regex>}``. Строка ``[cover] баланс KIE: 0`` (или
провал заказа песни) в логе акта -- шаги n707-n709 не играются (``SKIP NO_SONG``). Механизм (``RULE``, шаги ``KIE_QUOTA``
ниже) проверяется на собственных сценариях; правила акта 7 -- на файле сценария.

Робот -- ``fake_robot_ssh.sh`` (лог с метками ``{NOW-x}``).

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_3971_harness_followups.py -v -o addopts=""
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
import time
from pathlib import Path

import pytest

from tests.unit.e2e_scripts.test_issue_2846_registration_verdict import extract_scenario_loop
from tests.unit.e2e_scripts.test_issue_3857_event_waits import _feed, _l, _run, accept, needs_bash, speech, turn_done
from tests.unit.e2e_scripts.test_issue_3857_run_step import extract_function
from tests.unit.e2e_scripts.test_issue_3989_recover import BOTH, STEP, _act_start, _run_step

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"
EVENTS = SCRIPTS_DIR / "e2e_events.py"
FAKE = Path(__file__).resolve().parent / "fake_robot_ssh.sh"
NIGHT = REPO_ROOT / ".github" / "e2e" / "scenarios" / "night"
ACT7 = NIGHT / "night_marathon_act7_neuro_composer_v1.json"

sys.path.insert(0, str(SCRIPTS_DIR))
import e2e_events  # noqa: E402

# -- строки робота (марафон 23.10, va_marathon.log:5281/5462/5464; turn.py, plan/perform.py, plan_events) -----------


def tale_start(ts, status="'status': 'pending', 'block_id': 'perform-ee2404b9048f'"):
    return _l("dialog_node", ts, "🧭 [turn-v2] выступление старт kind=tale backing=calm segments=7 → {%s} "
                                 "turn=ee2404b9048f" % status)


def tale_end(ts, reason="done"):
    return _l("dialog_node", ts, "🎤 [perform] perform-: конец (%s) сегментов 7, символов 917, synced=True, "
                                 "подложка stopped" % reason)


def block_end(ts, reason="done"):
    return _l("dialog_node", ts, "🗓️ [plan] PlanEvent block_end block=perform-ee24 seg=7 synced=true spoken_chars=917 "
                                 "text_len=0 offset_ms=- after=-/- eta_ms=0 reason=%s" % reason)


def kie(balance, ts="{NOW-50}"):
    return _l("mcp_server", ts, "🎼 [cover] баланс KIE: %s" % balance)


def ago(seconds):
    """Абсолютная метка «seconds назад»: метки ``{NOW-x}`` фейк пересчитывает при каждом чтении (они не стареют),
    а потолок выступления меряется от его старта -- ему нужно время, которое идёт."""
    return "%.3f" % (time.time() - seconds)


# Сказка в середине: ход кончился (stage=done), куплет 1 прочитан, куплет 2 ещё не начат -- открытых реплик нет.
TALE_BETWEEN_SEGMENTS = accept("{NOW-70}") + tale_start("{NOW-60}") + speech("started", "s1s1s1s1", "{NOW-58}") + \
    turn_done("{NOW-57}") + speech("finished", "s1s1s1s1", "{NOW-40}", "spoken_chars=120")


class TestPerformWait:
    def test_turn_done_between_segments_is_not_the_reply(self):
        st = e2e_events.state(_feed(TALE_BETWEEN_SEGMENTS), "v2", False)
        assert st["verdict"] == "wait" and st["performing"] == 1 and st["open"] == 0
        assert st["perform_for"] == "60" and st["perform_ceiling"] == "240"

    @pytest.mark.parametrize("end", [tale_end("{NOW-5}"), block_end("{NOW-5}"), tale_end("{NOW-5}", "stopped")],
                             ids=["perform_end", "plan_block_end", "stopped"])
    def test_performance_end_releases_the_reply(self, end):
        st = e2e_events.state(_feed(TALE_BETWEEN_SEGMENTS + end), "v2", False)
        assert st["verdict"] == "done" and st["performing"] == 0

    def test_performance_over_ceiling_is_final(self):
        log = accept("{NOW-260}") + tale_start("{NOW-250}") + turn_done("{NOW-245}")
        st = e2e_events.state(_feed(log), "v2", False)
        assert st["verdict"] == "perform_ceiling" and st["reason"] == "perform_unfinished:kind=tale,for=250s"
        assert e2e_events.state(_feed(log), "v2", False, perform_ceiling=300.0)["verdict"] == "wait"

    def test_lines_merged_into_running_performance_do_not_open_a_new_one(self):
        """``'status': 'done'`` -- куплеты легли в идущее выступление (марафон 23.10 22:50:28): своего конца нет."""
        log = accept("{NOW-30}") + tale_start("{NOW-20}", "'status': 'done'") + speech("started", "s2s2s2s2", "{NOW-19}") \
            + turn_done("{NOW-18}") + speech("finished", "s2s2s2s2", "{NOW-10}")
        assert e2e_events.state(_feed(log), "v2", False)["verdict"] == "done"

    def test_previous_step_performance_is_not_this_step(self):
        """Старт до ПРИНЯТО этой фразы -- выступление прошлого шага, конец этого хода его не ждёт."""
        log = tale_start("{NOW-90}") + accept("{NOW-30}") + speech("started", "s3s3s3s3", "{NOW-25}") + \
            turn_done("{NOW-20}") + speech("finished", "s3s3s3s3", "{NOW-10}")
        assert e2e_events.state(_feed(log), "v2", False)["verdict"] == "done"

    def test_cli_state_reports_performance_and_takes_ceiling(self):
        args = [sys.executable, str(EVENTS), "state", "--engine", "v2", "--perform-ceiling", "50"]
        out = subprocess.run(args, input=_feed(TALE_BETWEEN_SEGMENTS), capture_output=True, text=True,
                             encoding="utf-8").stdout.splitlines()
        assert "verdict=perform_ceiling" in out and "performing=1" in out and "perform_ceiling=50" in out
        default = subprocess.run(args[:-2] + ["--perform-ceiling", "0"], input=_feed(TALE_BETWEEN_SEGMENTS),
                                 capture_output=True, text=True, encoding="utf-8").stdout.splitlines()
        assert "verdict=wait" in default and "perform_ceiling=240" in default

    def test_harness_greps_performance_lines(self):
        grep = re.search(r"_E2E_EVENT_GREP='([^']+)'", LIB.read_text(encoding="utf-8")).group(1)
        for line in (tale_start("1"), tale_end("1"), block_end("1")):
            assert re.search(grep, line), line

    def test_marker_sources_exist_in_robot_code(self):
        turn = (REPO_ROOT / "src/rob_box_dialog/rob_box_dialog/turn.py").read_text(encoding="utf-8")
        perform = (REPO_ROOT / "src/rob_box_dialog/rob_box_dialog/plan/perform.py").read_text(encoding="utf-8")
        assert "выступление {'правка' if cmd.edit_pending else 'старт'} kind={cmd.perform.kind}" in turn
        assert 'self.log(f"конец ({reason}) сегментов' in perform


@needs_bash
class TestWaitReplyDoneForPerformance:
    def test_reaction_window_does_not_cut_a_running_tale(self, tmp_path):
        """Потолок ответа 2 с не обрывает сказку: ждём её до 240 с от старта; здесь старт 235 с назад -- ~5 с."""
        log = accept("{NOW-250}") + tale_start(ago(235)) + turn_done("{NOW-230}")
        rc, out, err, took = _run(tmp_path, log, 'E2E_REACTION_WINDOW=2; wait_reply_done 2026-01-01T00:00:00Z; '
                                                 'echo "RC=$? V=$E2E_REPLY_VERDICT R=$E2E_REPLY_REASON"')
        assert "RC=2 V=perform_ceiling R=perform_unfinished:kind=tale" in out, (out, err)
        assert 3.0 <= took < 30, took

    def test_finished_tale_is_the_reply(self, tmp_path):
        rc, out, err, took = _run(tmp_path, TALE_BETWEEN_SEGMENTS + tale_end("{NOW-4}") + block_end("{NOW-4}"),
                                  'E2E_REACTION_WINDOW=40; wait_reply_done 2026-01-01T00:00:00Z; echo "RC=$?"')
        assert "RC=0" in out and took < 10, (out, err)

    def test_perform_ceiling_is_configurable(self, tmp_path):
        log = accept("{NOW-70}") + tale_start(ago(58)) + turn_done("{NOW-50}")
        rc, out, err, took = _run(tmp_path, log,
                                  'E2E_REACTION_WINDOW=2; E2E_PERFORM_WAIT_MAX=61; wait_reply_done 2026-01-01T00:00:00Z; '
                                  'echo "RC=$? V=$E2E_REPLY_VERDICT"')
        assert "RC=2 V=perform_ceiling" in out and took < 30, (out, err)


@needs_bash
def test_run_step_does_not_replay_a_performance_that_overran(tmp_path):
    """E2E_MAX_ATTEMPTS=3: выступление не кончилось за потолок -- FAIL сразу, фразу шага второй раз не играем."""
    log = accept("{NOW-250}") + tale_start(ago(237)) + turn_done("{NOW-240}")
    out, err, took, played = _run_step(tmp_path, log, [], STEP, BOTH + "; E2E_MAX_ATTEMPTS=3")
    assert "RC=1" in out and "PLAY_COUNT=1" in out, (out, err)
    assert "E2E_STEP a FAIL turn_perform_ceiling (perform_unfinished:kind=tale" in out
    assert "выступление не кончилось за потолок" in out
    assert "cycle_unconfirmed_after_accept" not in out
    assert took < 40, took


# -- предусловие шага: skip_when (R21) ------------------------------------------------------------------------------


RULE = {"reason": "KIE_QUOTA", "log": r"\[cover\] баланс KIE: 0(\.0+)?(\s|$)"}


class TestSkipRule:
    def test_rule_and_verdict(self):
        rule = e2e_events.skip_rule({"label": "n710", "skip_when": RULE})
        assert rule.reason == "KIE_QUOTA"
        hit = e2e_events.skip_verdict(rule, _feed(accept() + kie("0.0")))
        assert hit["fetch"] == 1 and hit["skip"] == "KIE_QUOTA" and "баланс KIE: 0.0" in hit["evidence"]
        for funded in ("1712", "0.5", "неизвестен"):
            assert e2e_events.skip_verdict(rule, _feed(kie(funded)))["skip"] == "-", funded
        assert e2e_events.skip_verdict(rule, "Traceback\n" + kie("0.0")) == {"fetch": 0, "skip": "-", "evidence": "-"}

    def test_step_without_field_has_no_rule(self):
        assert e2e_events.skip_rule({"label": "n711"}) is None

    @pytest.mark.parametrize("raw", [
        "KIE_QUOTA", {"reason": "KIE_QUOTA"}, {"log": "x"}, {"reason": "kie quota", "log": "x"},
        {"reason": "KIE_QUOTA", "log": ""}, {"reason": "KIE_QUOTA", "log": "(unclosed"},
    ])
    def test_bad_field_is_an_error_not_a_silent_play(self, raw):
        with pytest.raises(ValueError):
            e2e_events.skip_rule({"label": "n710", "skip_when": raw})

    def test_act7_rules_come_from_the_generator(self):
        scenario = json.loads(ACT7.read_text(encoding="utf-8"))
        rules = dict(e2e_events.skip_rules(scenario))
        assert sorted(rules) == ["n707_gen_track_info", "n708_gen_play", "n709_stop_generated"]
        assert {r.reason for r in rules.values()} == {"NO_SONG"}
        assert "_skip_when" not in ACT7.read_text(encoding="utf-8")

    def test_every_generated_act_parses(self):
        for path in sorted(NIGHT.glob("night_marathon_act*_v1.json")):
            if path.name.endswith("_acceptance_v1.json"):
                continue
            scenario = json.loads(path.read_text(encoding="utf-8"))
            e2e_events.skip_rules(scenario)
            e2e_events.recover_policy(scenario)

    def test_cli_skips_and_skip(self, tmp_path):
        out = subprocess.run([sys.executable, str(EVENTS), "skips", "--scenario", str(ACT7)],
                             capture_output=True, text=True, encoding="utf-8")
        assert out.returncode == 0 and "n708_gen_play\tNO_SONG" in out.stdout.splitlines()
        zero = subprocess.run([sys.executable, str(EVENTS), "skip", "--scenario", str(ACT7), "--label", "n708_gen_play"],
                              input=_feed(kie("0.0")), capture_output=True, text=True, encoding="utf-8").stdout
        assert "skip=NO_SONG" in zero.splitlines() and "fetch=1" in zero.splitlines()
        failed = subprocess.run([sys.executable, str(EVENTS), "skip", "--scenario", str(ACT7), "--label", "n709_stop_generated"],
                                input=_feed(_l("voice_assistant", "{NOW-50}", "🎤 [song] не вышло: задача t1 без треков")),
                                capture_output=True, text=True, encoding="utf-8").stdout
        assert "skip=NO_SONG" in failed.splitlines(), "провал заказа песни тоже гасит шаги про неё"
        made = subprocess.run([sys.executable, str(EVENTS), "skip", "--scenario", str(ACT7), "--label", "n708_gen_play"],
                              input=_feed(kie("1712")), capture_output=True, text=True, encoding="utf-8").stdout
        assert "skip=-" in made.splitlines(), "баланс есть, песни не провалились — играть"
        free = subprocess.run([sys.executable, str(EVENTS), "skip", "--scenario", str(ACT7), "--label", "n711_renardo_not_broken"],
                              input=_feed(kie("0.0")), capture_output=True, text=True, encoding="utf-8").stdout
        assert "skip=-" in free.splitlines(), "у шага нет правила -- играть"
        bad = tmp_path / "bad.json"
        bad.write_text(json.dumps({"steps": [{"label": "a", "skip_when": {"reason": "X"}}]}), encoding="utf-8")
        err = subprocess.run([sys.executable, str(EVENTS), "skips", "--scenario", str(bad)],
                             capture_output=True, text=True, encoding="utf-8")
        assert err.returncode == 2 and "skip_when" in err.stderr


def _precondition(tmp_path, log_text, label="n708_gen_play", extra=""):
    log = tmp_path / "robot.log"
    log.write_text(log_text, encoding="utf-8")
    body = "\n".join([
        'export FAKE_ROBOT_LOG="%s"' % log.as_posix(),
        'OUT_DIR="%s"; E2E_RUN_BEFORE=2026-01-01T00:00:00Z' % tmp_path.as_posix(),
        'emit_step() { echo "E2E_STEP $1"; }',
        extra,
        'load_skip_rules "%s"' % ACT7.as_posix(),
        'step_precondition_skip %s "%s"; echo "RC=$?"' % (label, ACT7.as_posix()),
    ])
    return _act_start(tmp_path, body)


@needs_bash
class TestStepPrecondition:
    def test_kie_quota_skips_the_step(self, tmp_path):
        rc, out, err, _ = _precondition(tmp_path, kie("0.0"))
        assert "RC=0" in out and "E2E_STEP n708_gen_play SKIP NO_SONG" in out, (out, err)
        assert "баланс KIE: 0.0" in out and "НЕ играется" in out

    def test_funded_balance_plays_the_step(self, tmp_path):
        rc, out, err, _ = _precondition(tmp_path, kie("1712"))
        assert "RC=1" in out and "E2E_STEP" not in out and "не нарушено" in out, (out, err)

    def test_step_without_rule_does_not_read_the_log(self, tmp_path):
        rc, out, err, state = _precondition(tmp_path, kie("0.0"), label="n711_renardo_not_broken",
                                            extra=': > "$FAKE_ROBOT_STATE/ssh_calls"')
        assert "RC=1" in out, (out, err)
        assert (state / "ssh_calls").read_text(encoding="utf-8") == ""

    def test_unreadable_log_is_loud_and_plays(self, tmp_path):
        rc, out, err, _ = _precondition(tmp_path, kie("0.0"), extra='ROBOT_SSH="false"')
        assert "RC=1" in out and "предусловие NO_SONG не проверено" in out, (out, err)

    def test_bad_field_is_fatal_before_first_step(self, tmp_path):
        bad = tmp_path / "bad.json"
        bad.write_text(json.dumps({"steps": [{"label": "a", "skip_when": {"reason": "X"}}]}), encoding="utf-8")
        rc, out, err, _ = _act_start(tmp_path, 'OUT_DIR="%s"; load_skip_rules "%s"' % (tmp_path.as_posix(), bad.as_posix()))
        assert rc == 2 and "E2E_FATAL: поле skip_when сценария не разобрано" in err, (rc, out, err)


@needs_bash
def test_scenario_loop_does_not_play_skipped_step_and_summary_counts_it(tmp_path):
    """Цикл сценария: шаг с нарушенным предусловием не играется (run_step не зовётся), сводка -- skip=1."""
    scen = tmp_path / "scenario.json"
    scen.write_text(json.dumps({"steps": [
        {"label": "n709_stop_generated", "text": "Робот, останови.", "expect": "cycle", "skip_when": RULE},
        {"label": "n711_renardo", "text": "Робот, сыграй бит.", "expect": "cycle", "skip_when": dict(RULE, log="нет такой")},
    ]}, ensure_ascii=False), encoding="utf-8")
    out_dir = tmp_path / "out"
    out_dir.mkdir()
    log = tmp_path / "robot.log"
    log.write_text(kie("0.0"), encoding="utf-8")
    script = "\n".join([
        "set -u",
        'OUT_DIR="%s"; SCENARIO_FILE="%s"' % (out_dir.as_posix(), scen.as_posix()),
        'export FAKE_ROBOT_STATE="%s/state" FAKE_ROBOT_LOG="%s"' % (tmp_path.as_posix(), log.as_posix()),
        'ROBOT_SSH="bash %s"' % FAKE.as_posix(),
        'SCRIPT_DIR_E2E="%s"' % SCRIPTS_DIR.as_posix(),
        'PASS=1; WAKE_GATE_SKIPPED_STEPS=0; E2E_RETRY_PAUSE=0; E2E_FAIL_KIND=""; E2E_RUN_BEFORE=2026-01-01T00:00:00Z',
        'WAKE_GATE_PREFLIGHT_FILE="$OUT_DIR/wake_gate_preflight.json"',
        'log() { echo ">>> $*"; }',
        LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
        (SCRIPTS_DIR / "e2e_voice_wake_gate.sh").read_text(encoding="utf-8").replace("\r\n", "\n"),
        extract_function("ensure_outdir"),
        extract_function("emit_step"),
        extract_function("mark_fail_kind"),
        extract_function("check_music_compare"),  # #3971: сверка вида результата
        extract_function("write_summary"),
        'run_step() { echo "PLAYED[$3]"; return 0; }',
        "parse_transcript() { :; }",
        'load_recover_policy "$SCENARIO_FILE"',
        'load_skip_rules "$SCENARIO_FILE"',
        'parse_scenario_to_tsv "$SCENARIO_FILE" "$OUT_DIR/scenario_parsed.txt"',
        'LAST_STEP_SPEECH_FILE="$OUT_DIR/.last_step_speech.txt"; : > "$LAST_STEP_SPEECH_FILE"',
        "E2E_ACT_RECOVERED=''",
        extract_scenario_loop(),
        "write_summary",
    ])
    t0 = time.time()
    p = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=120)
    out, err = p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace")
    assert "E2E_STEP n709_stop_generated SKIP KIE_QUOTA" in out, (out, err)
    assert "PLAYED[n709_stop_generated]" not in out and "PLAYED[n711_renardo]" in out
    assert "E2E_STEP n711_renardo OK" in out.splitlines()
    assert re.search(r"E2E_SUMMARY verdict=PASS steps=1/2 ok fail=0 skip=1 ", out), out
    assert time.time() - t0 < 100
