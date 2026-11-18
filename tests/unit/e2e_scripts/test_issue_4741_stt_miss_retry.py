# -*- coding: utf-8 -*-
"""#4741 / #4738: харнесс повторяет шаг, если STT принял непустую чужую фразу.

n1009: «какой чай» -> «какой человек». n709: стоп -> «Thank you.». Это флаки STT, не баг диалога:
словаря подмен нет, шаг без stt_keys. Решение в коде (``stt_miss_retry``, ADR-0148): гипотеза не пустая
и не та фраза -- повтор до ``STT_GARBAGE_MAX_ATTEMPTS`` (3, считая первую); исчерпание -- честный FAIL
со счётчиком ``stt_attempt`` в вердикте; таймаут / падение / совпавшая фраза / отказ тула -- повтора нет.

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_4741_stt_miss_retry.py -v -o addopts=""
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from tests.unit.e2e_scripts.test_issue_3857_event_waits import _l
from tests.unit.e2e_scripts.test_issue_3989_recover import REPLY, SCRIPTS_DIR, _run_step, heard, needs_bash

ROOT = Path(__file__).resolve().parents[3]
for _dir in (ROOT / "scripts" / "e2e", SCRIPTS_DIR):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import e2e_events as eev  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

EVENTS = SCRIPTS_DIR / "e2e_events.py"

N1009 = "Робот, поищи в памяти, какой чай я пью."
N709 = "Робот, останови, Борис уже уснул под неё."
TEA_OK = "робот поищи в памяти какой чай я пью"
TEA_MISS = "робот поищи в памяти какой человек я пью"
STOP_OK = "робот останови борис уже уснул под неё"
THANK_YOU = "Thank you."

STEP = 'run_step "%s" anton a cycle; echo "RC=$?"' % N1009
STOP_STEP = 'run_step "%s" anton a cycle; echo "RC=$?"' % N709


def _names(played):
    return [p.rsplit("/", 1)[-1] for p in played]


def test_n1009_and_n709_have_no_stt_keys_and_no_substitution_dictionary():
    """Шифу: в баге смысла нет, словарь «человек→чай» / «Thank you→стой» запрещён."""
    steps = {s["label"]: s for act in gen.ACTS for s in act["steps"]}
    assert eev.STT_KEYS_FIELD not in steps["n1009_memory_context_after_reset"]
    assert eev.STT_KEYS_FIELD not in steps["n709_stop_generated"]
    assert eev.stt_miss_retry(N1009, TEA_MISS, 1)["retry"] == 1
    assert eev.stt_miss_retry(N709, THANK_YOU, 1)["retry"] == 1
    assert not eev.heard_matches_command(N1009, TEA_MISS)
    assert not eev.heard_matches_command(N709, THANK_YOU)


def test_miss_retries_until_the_budget_then_is_exhausted():
    first = eev.stt_miss_retry(N1009, TEA_MISS, 1)
    assert first == {"retry": 1, "reason": "retry", "attempt": 1, "max": 3}
    second = eev.stt_miss_retry(N1009, TEA_MISS, 2)
    assert second == {"retry": 1, "reason": "retry", "attempt": 2, "max": 3}
    third = eev.stt_miss_retry(N1009, TEA_MISS, 3)
    assert third["retry"] == 0 and third["reason"] == "attempts_exhausted"
    assert third["max"] == eev.STT_GARBAGE_MAX_ATTEMPTS == 3


def test_matching_phrase_and_empty_hypothesis_are_not_retried():
    assert eev.heard_matches_command(N1009, TEA_OK)
    assert eev.stt_miss_retry(N1009, TEA_OK, 1)["reason"] == "command_heard"
    assert eev.stt_miss_retry(N1009, TEA_OK, 1)["retry"] == 0
    assert eev.stt_miss_retry(N1009, "", 1)["reason"] == "nothing_heard"
    assert eev.stt_miss_retry(N1009, "", 1)["retry"] == 0
    # голое «Робот!» после no_wake -- подхват бэклога, не промах фразы шага
    assert eev.stt_miss_retry(N1009, "Робот!", 1)["reason"] == "bare_wake"
    assert eev.stt_miss_retry(N1009, "Робот!", 1)["retry"] == 0


def test_timeout_crash_and_barge_in_are_not_retried():
    """Таймаут (пустое или совпавшее ПРИНЯТО), падение робота, перебивание -- не промах STT."""
    empty_timeout = eev.stt_miss_retry(N1009, "", 1, timeout=True)
    assert empty_timeout["retry"] == 0 and empty_timeout["reason"] == "timeout"
    matched_timeout = eev.stt_miss_retry(N1009, TEA_OK, 1, timeout=True)
    assert matched_timeout["retry"] == 0 and matched_timeout["reason"] == "timeout"
    # цикл не закрылся, но гипотеза чужая -- это #4738, повтор нужен
    thank_timeout = eev.stt_miss_retry(N709, THANK_YOU, 1, timeout=True)
    assert thank_timeout["retry"] == 1 and thank_timeout["reason"] == "retry"
    assert eev.stt_miss_retry(N709, THANK_YOU, 1, crash=True)["reason"] == "crash"
    assert eev.stt_miss_retry(N709, THANK_YOU, 1, crash=True)["retry"] == 0
    assert eev.stt_miss_retry(N709, THANK_YOU, 1, barge_in=True)["reason"] == "barge_in"
    assert eev.stt_miss_retry(N709, THANK_YOU, 1, barge_in=True)["retry"] == 0


def test_matching_heard_is_not_retried_when_tts_or_tool_is_missing():
    """Совпавшая фраза -- робота судят как есть: нет TTS/тула не маскируем повтором STT."""
    got = eev.stt_miss_retry(N709, STOP_OK, 1)
    assert got["retry"] == 0 and got["reason"] == "command_heard"


def test_cli_stt_miss_reports_the_same_decision():
    out = subprocess.run(
        [sys.executable, str(EVENTS), "stt-miss", "--text", N1009, "--heard", TEA_MISS, "--attempt", "1"],
        capture_output=True, text=True, encoding="utf-8", check=True,
    ).stdout.splitlines()
    assert out == ["retry=1", "reason=retry", "attempt=1", "max=%d" % eev.STT_GARBAGE_MAX_ATTEMPTS]
    timeout = subprocess.run(
        [sys.executable, str(EVENTS), "stt-miss", "--text", N1009, "--heard", TEA_OK,
         "--attempt", "1", "--timeout", "1"],
        capture_output=True, text=True, encoding="utf-8", check=True,
    ).stdout.splitlines()
    assert timeout[0] == "retry=0" and timeout[1] == "reason=timeout"


@needs_bash
class TestRunStepSttMiss:
    def test_first_miss_then_success_is_pass_not_first_try_fail(self, tmp_path):
        """#4741: «какой человек», затем правильная фраза -- один повтор, RC=0, не PASS первой ошибки."""
        first = heard(TEA_MISS, "{NOW-30}") + REPLY
        again = heard(TEA_OK, "{NOW-11}") + REPLY
        out, err, _, played = _run_step(tmp_path, first, [again], STEP, replace=True)
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=2" in out
        assert out.count("E2E_RECOVER stt_replay a ") == 1
        assert "RECOVERED=stt_replay" in out
        assert "FAIL stt_miss" not in out
        assert "какой человек" in out
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_misses_until_the_budget_are_an_honest_fail_with_the_counter(self, tmp_path):
        """Промах на всех попытках -- честный FAIL, PLAY ровно по потолку, в вердикте stt_attempt."""
        bad = heard(TEA_MISS, "{NOW-30}") + REPLY
        again = heard(TEA_MISS, "{NOW-11}") + REPLY
        out, err, _, played = _run_step(
            tmp_path, bad, [again, again], STEP, replace=True)
        assert "RC=1" in out, (out, err)
        assert "PLAY_COUNT=%d" % eev.STT_GARBAGE_MAX_ATTEMPTS in out
        assert out.count("E2E_RECOVER stt_replay a ") == eev.STT_GARBAGE_MAX_ATTEMPTS - 1
        assert "E2E_STEP a FAIL stt_miss stt_attempt=%d/%d" % (
            eev.STT_GARBAGE_MAX_ATTEMPTS, eev.STT_GARBAGE_MAX_ATTEMPTS) in out
        assert "честный FAIL" in out
        assert _names(played) == ["cmd_a_eq.wav"] * eev.STT_GARBAGE_MAX_ATTEMPTS

    def test_second_miss_then_the_phrase_recovers_with_the_counter(self, tmp_path):
        """Второй промах ещё пробует: третья попытка услышана -- не FAIL, счётчик в логе повторов."""
        bad = heard(TEA_MISS, "{NOW-30}") + REPLY
        again = heard(TEA_MISS, "{NOW-11}") + REPLY
        ok = heard(TEA_OK, "{NOW-8}") + REPLY
        out, err, _, played = _run_step(tmp_path, bad, [again, ok], STEP, replace=True)
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=3" in out
        assert out.count("E2E_RECOVER stt_replay a ") == 2
        assert "stt_attempt=2/3" in out and "stt_attempt=3/3" in out
        assert "FAIL stt_miss" not in out
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_thank_you_instead_of_stop_replays_once_even_if_cycle_did_not_finish(self, tmp_path):
        """#4738: ПРИНЯТО «Thank you.», цикл не закрылся -- это промах STT, не таймаут без гипотезы."""
        first = heard(THANK_YOU, "{NOW-30}") + _l("dialog_node", "{NOW-29}", "🔎 LLM REQUEST START provider=x")
        again = heard(STOP_OK, "{NOW-11}") + REPLY
        env = "E2E_REACTION_WINDOW=2; E2E_ACCEPT_WINDOW=2"
        out, err, _, played = _run_step(tmp_path, first, [again], STOP_STEP, env, replace=True)
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=2" in out
        assert "Thank you" in out
        assert "E2E_RECOVER stt_replay a " in out
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_timeout_with_matching_heard_is_not_retried(self, tmp_path):
        """Совпавшее ПРИНЯТО, ход не закрылся: таймаут, PLAY один, повтора STT нет."""
        stuck = heard(TEA_OK, "{NOW-30}") + _l("dialog_node", "{NOW-29}", "🔎 LLM REQUEST START provider=x")
        env = "E2E_REACTION_WINDOW=2; E2E_ACCEPT_WINDOW=2; E2E_MAX_ATTEMPTS=1"
        out, err, _, played = _run_step(tmp_path, stuck, [], STEP, env)
        assert "PLAY_COUNT=1" in out, (out, err)
        assert "E2E_RECOVER stt_replay" not in out
        assert "повтора по промаху STT нет (timeout" in out or "повтора по промаху STT нет (command_heard" in out
        assert _names(played) == ["cmd_a_eq.wav"]
