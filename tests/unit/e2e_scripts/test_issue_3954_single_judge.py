"""Issue #3954 (эпик #3683) -- v2: ОДИН судья «ответ готов» (``e2e_events.py``), ``stage=failed`` -- конец хода.

Было: ``wait_reply_done`` ждал потолок ``E2E_REACTION_WINDOW`` (40 с) при ``stage=failed`` (концом хода считался
только ``stage=done``), потом ``check_cycle`` (наследие v1, требует строку ``TTS finished``) возвращал rc=3, и шаг
крутил ещё 2 попытки по 40 с + ``E2E_RETRY_PAUSE`` 10 с: ~2.5 мин тишины до FAIL ``cycle_unconfirmed_after_accept``.

Фикстуры -- реальные строки лога робота (n209, акт 2, run 39962806336) и реконструкция n109 (акт 1, run 39937320548:
в логе харнесса строк робота нет, сохранён только ``reply_after=22s``; реконструкция -- обычный ответ v2 БЕЗ строки
``TTS finished``, которой в v2 нет). Время в фикстурах пересчитано в ``{NOW-x}`` фейк-робота.

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_3954_single_judge.py -v
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

from tests.unit.e2e_scripts.test_issue_3857_run_step import _run, needs_bash

HERE = Path(__file__).resolve().parent
SCRIPTS_DIR = HERE.parents[2] / ".github" / "workflows" / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))
import e2e_events  # noqa: E402

TS = re.compile(r"\[(\d{10}\.\d+)\]")


def fixture(name, tail=1.0):
    """Строки фикстуры с ROS-временем, пересчитанным в ``{NOW-x}`` (конец последней строки = ``tail`` с назад)."""
    text = (HERE / "fixtures_3954" / name).read_text(encoding="utf-8")
    end = max(float(m.group(1)) for m in TS.finditer(text))
    return TS.sub(lambda m: "[{NOW-%.3f}]" % (end + tail - float(m.group(1))), text)


def stamped(name, now=1_790_000_100.0):
    return "%.2f\n%s" % (now, re.sub(r"\{NOW-([0-9.]+)\}", lambda m: "%.3f" % (now - float(m.group(1))), fixture(name)))


class TestFailedTurnIsTheEndOfTheTurn:
    def test_n209_failed_turn_is_failed_verdict_with_reason(self):
        st = e2e_events.state(stamped("n209_failed_turn.log"), "v2", False)
        assert st["verdict"] == "failed"
        assert st["reason"] == "failed:status=failed" and st["llm_calls"] == "3"

    def test_failed_stays_failed_even_for_silent_ok_step(self):
        assert e2e_events.state(stamped("n209_failed_turn.log"), "v2", True)["verdict"] == "failed"

    def test_n109_reply_without_tts_finished_is_done(self):
        assert "TTS finished" not in fixture("n109_reply_without_tts_finished.log")
        st = e2e_events.state(stamped("n109_reply_without_tts_finished.log"), "v2", False)
        assert st["verdict"] == "done"

    def test_failed_while_speech_open_waits_for_the_speech(self):
        text = stamped("n209_failed_turn.log") + (
            "[tts_node-7] [INFO] [1790000099.00] [tts_node]: 🗣️ SpeechEvent started speech_id=feedbeef "
            "agent=personality epoch=3 text_len=9\n")
        assert e2e_events.state(text, "v2", False)["verdict"] == "wait"

    def test_cancelled_ends_turn_but_not_for_the_barging_step(self):
        log = ("[stt_node-5] [INFO] [1789999990.00] [stt_node]: ✅ ПРИНЯТО (respeaker): робот стоп\n"
               "[dialog_node-3] [INFO] [1789999995.00] [dialog_node]: 📊 [turn] turn_id=old epoch=1 agent=personality "
               "stage=cancelled status=cancelled latency_ms=1 llm_calls=1 llm_provider=deepseek tokens=1\n")
        feed = "1790000000.00\n" + log
        assert e2e_events.state(feed, "v2", False)["verdict"] == "cancelled"
        assert e2e_events.state(feed, "v2", False, barge=True)["verdict"] == "wait"

    def test_no_accept_after_window_but_not_before(self):
        feed = "1790000000.00\n"
        assert e2e_events.state(feed, "v2", False, waited=5, accept_window=15)["verdict"] == "wait"
        assert e2e_events.state(feed, "v2", False, waited=15, accept_window=15)["verdict"] == "no_accept"
        assert e2e_events.state(feed, "v1", False, waited=99, accept_window=0)["verdict"] == "wait"


def _reply_after(out):
    """Наибольшее ``reply_after=Ns`` шага из лога харнесса (секунды по его же часам)."""
    return max(int(n) for n in re.findall(r"reply_after=(\d+)s", out))


@needs_bash
class TestRunStepSingleJudge:
    STEP = 'E2E_FAIL_KIND=""; E2E_MAX_ATTEMPTS=3; E2E_RETRY_PAUSE=10; run_step "Робот привет" anton a cycle; echo "RC=$?"'

    def test_failed_turn_is_fail_at_once_without_retries(self, tmp_path):
        out, err, took = _run(tmp_path, self.STEP, fixture("n209_failed_turn.log"))
        assert "RC=1" in out and "FAIL turn_failed (failed:status=failed)" in out, (out, err)
        assert _reply_after(out) < 35, out  # раньше: потолок 41 с на попытку
        assert took < 100, "ждали %.1f с (раньше 3x41 + 2x10 = 143 с)" % took
        assert out.count("PLAY attempt") == 1 and "ПРИНЯТО уже было" not in out and "cycle_unconfirmed" not in out

    def test_reply_without_tts_finished_is_ok_at_once(self, tmp_path):
        out, err, took = _run(tmp_path, self.STEP, fixture("n109_reply_without_tts_finished.log"))
        assert "RC=0" in out and "ПОЛНЫЙ ЦИКЛ" in out, (out, err)
        assert _reply_after(out) < 35 and out.count("PLAY attempt") == 1

    def test_no_accept_replays_after_accept_window_not_reaction_window(self, tmp_path):
        body = ('E2E_FAIL_KIND=""; E2E_MAX_ATTEMPTS=2; E2E_RETRY_PAUSE=0; E2E_ACCEPT_WINDOW=3; '
                'run_step "Робот привет" anton a cycle; echo "RC=$?"')
        out, err, took = _run(tmp_path, body, "")
        assert "RC=1" in out and "FAIL no_accept" in out, (out, err)
        assert out.count("PLAY attempt") == 2
        assert _reply_after(out) < 35, "ждали потолок вместо окна ПРИНЯТО: %s" % out
