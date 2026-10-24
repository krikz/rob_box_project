"""Issue #3857 (эпик #3683) -- харнесс ждёт СОБЫТИЙ робота, а не таймеров; шаг может намеренно перебивать.

Было: после play -- слепой ``sleep E2E_REACTION_WINDOW`` (40 с), перед шагом -- 20 с окно лога + 15 с
«тишины» по grep строк синтеза (на шаг 60-80 с при ответе за 5-10 с). Стало: опрос ``docker logs`` каждые 2 с
и разбор маркеров модулем ``e2e_events.py``:

* v2: ``📊 [turn] ... stage=done`` хода + ``🗣️ SpeechEvent`` реплик закрыты (finished/cancelled/rejected по
  ``speech_id``); «робот ещё говорит» -- ``started`` без пары; тихо -- +2 с после последнего события;
* v1: ``🔊 TTS: speech_id=X`` / ``Публикую TTS finished event: speech_id=X``;
* потолок -- прежний ``E2E_REACTION_WINDOW``; ``E2E_EVENT_WAITS=0`` возвращает слепой sleep;
* ``barge_in`` шага: ``{"after": "speech_started|llm_request|segment_started", "n": N, "delay_sec": X}``;
* #3860: ``strip_llm_request_dump`` понимает launch-префикс ``[dialog_node-3]``.

Время в фейк-логе задано меткой ``{NOW-3.5}`` (``fake_robot_ssh.sh``: «3.5 с назад» по часам робота).

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_3857_event_waits.py -v
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"
FAKE = Path(__file__).resolve().parent / "fake_robot_ssh.sh"
SCENARIOS = REPO_ROOT / ".github" / "e2e" / "scenarios"

sys.path.insert(0, str(SCRIPTS_DIR))
import e2e_events  # noqa: E402
import e2e_tool_match  # noqa: E402

needs_bash = pytest.mark.skipif(shutil.which("bash") is None or shutil.which("python3") is None,
                                reason="нужны bash и python3")


# -- строки лога (формат speech_owner._emit / turn_log.TurnAggregate.log_line) ------------------------

def _l(node, ts, msg):
    return "[%s-9] [INFO] [%s] [%s]: %s\n" % (node, ts, node, msg)


def accept(ts="{NOW-30}"):
    return _l("stt_node", ts, "✅ ПРИНЯТО (respeaker): робот привет")


def llm(ts="{NOW-29}"):
    return _l("dialog_node", ts, "🔎 LLM REQUEST START provider=deepseek mode=tool_choice")


def turn_done(ts="{NOW-20}"):
    return _l("dialog_node", ts, "📊 [turn] turn_id=t1 epoch=2 agent=personality stage=done status=ok "
                                 "latency_ms=1834 llm_calls=1 llm_provider=deepseek tokens=5120")


def speech(kind, sid, ts, extra=""):
    return _l("tts_node", ts, "🗣️ SpeechEvent %s speech_id=%s agent=personality epoch=2 text_len=40 %s" % (kind, sid, extra))


def v1_open(sid, ts):
    return _l("tts_node", ts, "🔊 TTS: speech_id=%s, dialogue_id=None, text='привет'" % sid)


def v1_close(sid, ts):
    return _l("tts_node", ts, "📢 Публикую TTS finished event: speech_id=%s..., success=True, duration=3.1s" % sid)


DONE_V2 = accept() + llm() + speech("started", "aaaa1111", "{NOW-25}") + turn_done() + \
    speech("finished", "aaaa1111", "{NOW-10}", "spoken_chars=40")
SPEAKING_V2 = accept() + llm() + speech("started", "aaaa1111", "{NOW-5}") + turn_done("{NOW-4}")


def _stamp(text, now=1_790_000_100.0):
    """Подставить {NOW-x} -> ROS-время как у фейка (для чисто питоновых тестов)."""
    return re.sub(r"\{NOW-([0-9.]+)\}", lambda m: "%.2f" % (now - float(m.group(1))), text)


def _feed(log_text, now=1_790_000_100.0):
    return "%.2f\n%s" % (now, _stamp(log_text, now))


# -- e2e_events: чистый разбор ------------------------------------------------------------------------

class TestEventsParsing:
    def test_v2_reply_done_when_turn_done_and_speech_closed(self):
        st = e2e_events.state(_feed(DONE_V2), "v2", False)
        assert st["verdict"] == "done" and st["open"] == 0 and st["ends"] == 1
        assert float(st["quiet_for"]) == pytest.approx(10.0, abs=0.1)

    def test_v2_still_speaking_is_wait(self):
        st = e2e_events.state(_feed(SPEAKING_V2), "v2", False)
        assert st["verdict"] == "wait" and st["open"] == 1

    def test_v2_turn_not_done_is_wait_even_if_speech_finished(self):
        log = accept() + speech("started", "aaaa1111", "{NOW-25}") + speech("finished", "aaaa1111", "{NOW-10}")
        assert e2e_events.state(_feed(log), "v2", False)["verdict"] == "wait"

    def test_v2_queued_without_start_counts_as_speaking(self):
        log = accept() + turn_done() + speech("queued", "bbbb2222", "{NOW-3}")
        assert e2e_events.state(_feed(log), "v2", False)["verdict"] == "wait"

    def test_v2_stale_open_speech_is_not_speaking(self):
        log = accept("{NOW-300}") + speech("started", "aaaa1111", "{NOW-200}") + turn_done("{NOW-190}")
        # протухшая реплика не держит «робот говорит» (иначе шаг ждал бы потолок на каждом шаге)...
        assert e2e_events.quiet(_feed(log), "v2")["open"] == 0
        assert e2e_events.state(_feed(log), "v2", False)["open"] == 0
        # ...но и ответом на фразу не считается: reply_wait честно дойдёт до потолка
        assert e2e_events.state(_feed(log), "v2", False)["verdict"] == "wait"

    def test_v2_turn_without_speech_is_nospeech_or_silent(self):
        log = accept() + turn_done()
        assert e2e_events.state(_feed(log), "v2", False)["verdict"] == "nospeech"
        assert e2e_events.state(_feed(log), "v2", True)["verdict"] == "silent"

    def test_v2_cancelled_foreign_speech_is_not_a_reply(self):
        # остановленная реплика прошлой фразы (started был ДО окна) -- не ответ на эту фразу
        log = accept() + speech("cancelled", "oldold01", "{NOW-25}") + turn_done()
        assert e2e_events.state(_feed(log), "v2", False)["verdict"] == "nospeech"

    def test_v2_progress_does_not_close(self):
        log = accept() + speech("started", "aaaa1111", "{NOW-8}") + speech("progress", "aaaa1111", "{NOW-2}", "spoken_chars=5") + turn_done()
        assert e2e_events.state(_feed(log), "v2", False)["verdict"] == "wait"

    def test_v1_pairs_by_speech_id(self):
        log = accept() + v1_open("5e1c2a9b", "{NOW-20}") + v1_open("7f3d1c2e", "{NOW-15}") + v1_close("5e1c2a9b", "{NOW-10}")
        assert e2e_events.state(_feed(log), "v1", False)["verdict"] == "wait"
        log += v1_close("7f3d1c2e", "{NOW-5}")
        st = e2e_events.state(_feed(log), "v1", False)
        assert st["verdict"] == "done" and st["ends"] == 2

    def test_fetch_failure_is_reported(self):
        assert e2e_events.state("garbage line\n", "v2", False)["fetch"] == 0

    def test_quiet_counts_only_live_open(self):
        assert e2e_events.quiet(_feed(SPEAKING_V2), "v2")["open"] == 1
        assert e2e_events.quiet(_feed(DONE_V2), "v2")["open"] == 0

    def test_anchor_speech_started_n(self):
        log = accept() + speech("started", "aaaa1111", "{NOW-9}") + speech("finished", "aaaa1111", "{NOW-8}") + \
            speech("started", "bbbb2222", "{NOW-6}")
        one = e2e_events.anchor(_feed(log), "v2", "speech_started", 1)
        two = e2e_events.anchor(_feed(log), "v2", "segment_started", 2)
        three = e2e_events.anchor(_feed(log), "v2", "speech_started", 3)
        assert one["found"] == 1 and one["speech_id"] == "aaaa1111" and float(one["ts"]) == pytest.approx(1_790_000_091.0)
        assert two["found"] == 1 and two["speech_id"] == "bbbb2222"
        assert three["found"] == 0

    def test_anchor_ignores_events_before_accept(self):
        log = speech("started", "old00001", "{NOW-40}") + accept() + llm()
        assert e2e_events.anchor(_feed(log), "v2", "speech_started", 1)["found"] == 0
        assert e2e_events.anchor(_feed(log), "v2", "llm_request", 1)["found"] == 1

    def test_anchor_llm_request_n(self):
        log = accept() + llm("{NOW-9}") + llm("{NOW-5}")
        assert e2e_events.anchor(_feed(log), "v2", "llm_request", 2)["found"] == 1
        assert e2e_events.anchor(_feed(log), "v2", "llm_request", 3)["found"] == 0


# -- #3860: дамп LLM-запроса с launch-префиксом ------------------------------------------------------------

DUMP_LINE = ("[dialog_node-3]   [17] user: '<system_context>\\n  <speaker>{\"name\":\"Борис\"}</speaker>\\n  "
             "<recent_actions>[{\"tool\":\"register_speaker\",\"status\":\"done\"}]</recent_actions>'")


class TestDumpStrip3860:
    def test_python_strips_launch_prefixed_dump(self):
        logs = DUMP_LINE + "\n[dialog_node-3]   tools(3): a, b, c\nнастоящий лог\n"
        out = e2e_tool_match.strip_llm_request_dump(logs)
        assert "register_speaker" not in out and "tools(3)" not in out and "настоящий лог" in out
        assert not e2e_tool_match.tool_invoked(out, "register_speaker")
        assert e2e_tool_match.tool_invoked(logs, "register_speaker")  # без чистки -- ложный вызов (баг #3860)

    def test_python_keeps_real_invocation_line(self):
        logs = DUMP_LINE + "\nЗапрос выполнения: register_speaker с параметрами {}\n"
        assert e2e_tool_match.tool_invoked(e2e_tool_match.strip_llm_request_dump(logs), "register_speaker")

    def test_python_truncated_dump_keeps_tail(self):
        logs = "🔎 LLM REQUEST START provider=x\n  [0] system: 'x'\nхвост\n"
        assert "хвост" in e2e_tool_match.strip_llm_request_dump(logs)

    @needs_bash
    def test_bash_strips_launch_prefixed_dump(self):
        script = 'source "%s"; printf "%%s\\n" "$1" "реальный лог" | strip_llm_request_dump' % LIB.as_posix()
        out = subprocess.run(["bash", "-c", script, "x", DUMP_LINE], capture_output=True, text=True, encoding="utf-8").stdout
        assert "register_speaker" not in out and "реальный лог" in out


# -- bash: ожидание событий на фейковом роботе --------------------------------------------------------------

def _run(tmp_path, log_text, body, env=None, timeout=60):
    log = tmp_path / "robot.log"
    log.write_text(log_text, encoding="utf-8")
    state = tmp_path / "state"
    script = "\n".join([
        "set -u",
        'export FAKE_ROBOT_STATE="%s" FAKE_ROBOT_LOG="%s"' % (state.as_posix(), log.as_posix()),
        'ROBOT_SSH="bash %s"' % FAKE.as_posix(),
        'SCRIPT_DIR_E2E="%s"' % SCRIPTS_DIR.as_posix(),
        'log() { echo ">>> $*"; }',
        "E2E_DIALOG_ENGINE=v2; E2E_EVENT_POLL=1; E2E_BARGE_POLL=1",
        LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
        body,
    ])
    t0 = time.time()
    p = subprocess.run(["bash", "-c", script], capture_output=True, text=True, encoding="utf-8", timeout=timeout,
                       env=dict(__import__("os").environ, **(env or {})))
    return p.returncode, p.stdout, p.stderr, time.time() - t0


@needs_bash
class TestWaitReplyDone:
    def test_done_reply_does_not_wait_the_window(self, tmp_path):
        rc, out, err, took = _run(tmp_path, DONE_V2,
                                  'E2E_REACTION_WINDOW=40; wait_reply_done 2026-01-01T00:00:00Z; echo "RC=$? AFTER=$E2E_REPLY_AFTER"')
        assert "RC=0" in out, (out, err)
        assert took < 10, "ждали %.1f с вместо ~2 (окно 40 с)" % took

    def test_still_speaking_waits_up_to_ceiling(self, tmp_path):
        rc, out, err, took = _run(tmp_path, SPEAKING_V2,
                                  'E2E_REACTION_WINDOW=3; wait_reply_done 2026-01-01T00:00:00Z; echo "RC=$? AFTER=$E2E_REPLY_AFTER"')
        assert "RC=1" in out, (out, err)
        assert 2.0 <= took < 15  # потолок меряется целыми секундами (date +%s)

    def test_turn_without_speech_waits_grace_then_flags_no_speech(self, tmp_path):
        rc, out, err, took = _run(tmp_path, accept() + turn_done(),
                                  'E2E_REACTION_WINDOW=30; E2E_NO_SPEECH_GRACE=2; wait_reply_done 2026-01-01T00:00:00Z; '
                                  'echo "RC=$? NOSPEECH=$E2E_REPLY_NO_SPEECH"')
        assert "RC=2 NOSPEECH=1" in out, (out, err)  # #3954: v2 -- ход без речи после льготы это FAIL, не «цикл ждём»
        assert took < 12

    def test_silent_ok_step_finishes_without_grace(self, tmp_path):
        rc, out, err, took = _run(tmp_path, accept() + turn_done(),
                                  'E2E_REACTION_WINDOW=30; E2E_STEP_SILENT_OK=1; wait_reply_done 2026-01-01T00:00:00Z; '
                                  'echo "RC=$? NOSPEECH=$E2E_REPLY_NO_SPEECH"')
        assert "RC=0 NOSPEECH=1" in out, (out, err)
        assert took < 8

    def test_fixed_sleep_fallback(self, tmp_path):
        rc, out, err, took = _run(tmp_path, DONE_V2,
                                  'E2E_REACTION_WINDOW=1; E2E_EVENT_WAITS=0; step_wait_after_play x; echo "MODE=$E2E_WAIT_MODE"')
        assert "MODE=fixed" in out, (out, err)


@needs_bash
class TestWaitRobotQuiet:
    def test_quiet_when_speech_closed(self, tmp_path):
        rc, out, err, took = _run(tmp_path, DONE_V2, 'E2E_SILENCE_WAIT_MAX=30; wait_robot_quiet; echo "RC=$? S=$E2E_STEP_SILENCE_SEC"')
        assert "RC=0" in out, (out, err)
        assert took < 8, "тишина определялась %.1f с (раньше >=35)" % took

    def test_not_quiet_while_started_without_finished(self, tmp_path):
        rc, out, err, took = _run(tmp_path, SPEAKING_V2, 'E2E_SILENCE_WAIT_MAX=3; wait_robot_quiet; echo "RC=$?"')
        assert "RC=1" in out, (out, err)

    def test_quiet_when_nothing_in_log(self, tmp_path):
        rc, out, err, took = _run(tmp_path, "[x] [INFO] [1.0] [n]: ничего\n", 'wait_robot_quiet; echo "RC=$?"')
        assert "RC=0" in out, (out, err)


@needs_bash
class TestBargeIn:
    SPEC = '{"after":"speech_started","n":1,"delay_sec":2.0}'

    def test_anchor_found_and_delay_is_held_from_robot_clock(self, tmp_path):
        log = accept() + speech("started", "aaaa1111", "{NOW-1}")
        rc, out, err, took = _run(
            tmp_path, log,
            "E2E_REACTION_WINDOW=5; wait_barge_anchor x '%s'; echo \"RC=$? ID=$E2E_ANCHOR_ID\"; "
            "echo \"AFTER=$(barge_hold '%s')\"" % (self.SPEC, self.SPEC))
        assert "RC=0 ID=aaaa1111" in out, (out, err)
        m = re.search(r"AFTER=([0-9.]+)", out)
        assert m and 1.9 <= float(m.group(1)) <= 3.0, out  # якорь 1 с назад, delay 2 с -> играем на ~2.0 с
        assert 0.5 <= took < 6

    def test_anchor_missing_returns_1(self, tmp_path):
        rc, out, err, took = _run(tmp_path, accept() + llm(),
                                  "E2E_REACTION_WINDOW=2; wait_barge_anchor x '%s'; echo \"RC=$?\"" % self.SPEC)
        assert "RC=1" in out, (out, err)

    def test_unknown_anchor_kind_is_scenario_error(self, tmp_path):
        rc, out, err, took = _run(tmp_path, accept(),
                                  "wait_barge_anchor x '{\"after\":\"banana\",\"n\":1,\"delay_sec\":0}'; echo \"RC=$?\"")
        assert "RC=2" in out, (out, err)

    def test_previous_step_stops_at_anchor_not_at_end_of_reply(self, tmp_path):
        rc, out, err, took = _run(
            tmp_path, SPEAKING_V2,
            "E2E_REACTION_WINDOW=10; E2E_STEP_NEXT_BARGE_IN='%s'; step_wait_after_play x; "
            'echo "MODE=$E2E_WAIT_MODE PENDING=$E2E_PENDING_ANCHOR_SPEC"' % self.SPEC)
        assert "MODE=anchor" in out, (out, err)
        assert took < 6


# -- разбор сценария --------------------------------------------------------------------------------------

@needs_bash
class TestScenarioTsv:
    def test_barge_fields_reach_the_row(self, tmp_path):
        sc = tmp_path / "sc.json"
        sc.write_text(json.dumps({"steps": [
            {"label": "a", "text": "Робот, считай"},
            {"label": "b", "text": "Робот, стоп", "barge_in": {"after": "llm_request", "n": 2, "delay_sec": 1.5}},
            {"label": "c", "text": "Робот, тихо", "silent_ok": True},
        ]}, ensure_ascii=False), encoding="utf-8")
        script = ('source "%s"; parse_scenario_to_tsv "%s" "%s/o.tsv"; '
                  'while IFS=$\'\\x1f\' eval "$E2E_SCENARIO_ROW_READ"; do '
                  'echo "$label|$barge_in_json|$next_barge_in_json|$silent_ok"; done < "%s/o.tsv"'
                  % (LIB.as_posix(), sc.as_posix(), tmp_path.as_posix(), tmp_path.as_posix()))
        p = subprocess.run(["bash", "-c", script], capture_output=True, text=True, encoding="utf-8")
        rows = p.stdout.strip().splitlines()
        spec = '{"after":"llm_request","n":2,"delay_sec":1.5}'
        assert rows == ["a||%s|0" % spec, "b|%s||0" % spec, "c|||1"], (rows, p.stderr)


class TestScenariosUseBargeIn:
    """Перебивания в сценариях переведены на barge_in (перечень -- в PR #3857)."""

    FILES = [
        "night/night_marathon_act8_barge_in_v1.json",
        "1280_barge_in_abort_old_topic.json",
        "968_merge_song_komar_i_enot.json",
        "dialogue_gap_probe_v1.json",
    ]

    def test_every_barge_in_is_valid_and_has_a_step_to_interrupt(self):
        total = 0
        for rel in self.FILES:
            steps = json.loads((SCENARIOS / rel).read_text(encoding="utf-8"))["steps"]
            for i, s in enumerate(steps):
                b = s.get("barge_in")
                if b is None:
                    continue
                total += 1
                assert i > 0, "%s: barge_in на первом шаге -- перебивать некого" % rel
                assert b["after"] in ("speech_started", "llm_request", "segment_started"), (rel, s["label"])
                assert int(b.get("n", 1)) >= 1 and float(b.get("delay_sec", 0)) >= 0
        assert total == 7, "ожидали 7 шагов-перебиваний (act8 x4, 1280, 968, gp12), нашли %d" % total
