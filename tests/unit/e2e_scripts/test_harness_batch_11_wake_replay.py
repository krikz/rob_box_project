"""Пачка харнесса 11 (решение Шифу 28.10, эпик #3683) -- голос Гриши и хрупкий вейк: n210 краснел в каждом марафоне.

Марафон 28.10 (night_20261028_092532, акт 2, n210_grisha_no_name, 09:46): голос MiniMax «Gentle_Senior» с акцентом -- STT
услышал «ребят я не маша…» вместо «Робот, я мимо шла…», вейк не сработал, фраза ушла в бэклог. Харнесс сказал «Робот!»
тем же голосом (штатное восстановление ``no_wake``, #3989) -- STT потерял и его, итог ``FAIL turn_no_wake
after_recover_no_wake``. Шифу: «пусть Гриша при отвале повторит робот ну и добавь в его фразу с роботом робот в начале и
робот в конце». Решения кодом (ADR-0148), а не правилом в тексте:

* генератор (``gen_night_marathon.WAKE_FRAGILE_VOICES``) сам собирает фразу голоса с акцентом -- вейк в начале и в конце;
* тот же шаг получает ``recover: [..., "wake_replay"]``; харнесс (``e2e_events.WAKE_REPLAY``, ``step_wake_replay``) при
  промахе вейка, если «Робот!» не помог, повторяет ФРАЗУ ШАГА один раз -- ретрай именно на промах вейка (хода по фразе
  не было), а неверный ответ робота по-прежнему ретраит только ``retry_acceptance``.

Run:
    python -m pytest tests/unit/e2e_scripts/test_harness_batch_11_wake_replay.py -v -o addopts=""
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from tests.unit.e2e_scripts.test_issue_3989_recover import (
    EVENTS,
    NIGHT,
    REPLY,
    SCRIPTS_DIR,
    SILENCED_TURN,
    STEP,
    N301,
    _run_step,
    heard,
    needs_bash,
    no_wake,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "e2e"))
sys.path.insert(0, str(SCRIPTS_DIR))

import e2e_events as eev  # noqa: E402
import gen_night_marathon as gen  # noqa: E402
import phrase_budget as pb  # noqa: E402

ALL_STEPS = {s["label"]: s for act in gen.ACTS for s in act["steps"]}
ACT_RECOVER = {act["n"]: act["recover"] for act in gen.ACTS}
#: Виды восстановления шага с хрупким вейком: штатные и повтор фразы шага.
FRAGILE_KINDS = ("no_wake", "silenced", "wake_replay")


# -- генератор: вейк с обоих концов фразы ------------------------------------------------------------------


class TestWakeAtBothEnds:
    @pytest.mark.parametrize(
        "text, want",
        [
            (
                "Робот, я мимо шла. Имя своё я тебе называть не буду.",
                "Робот, я мимо шла. Имя своё я тебе называть не буду, робот.",
            ),
            ("Робот, который час, я спрашиваю!", "Робот, который час, я спрашиваю, робот!"),
            ("Робот, который час?", "Робот, который час, робот?"),
            ("Робот, привет", "Робот, привет, робот."),
            ("Робот, привет...", "Робот, привет, робот..."),
        ],
    )
    def test_fragile_voice_gets_the_wake_at_the_end(self, text, want):
        assert gen.wake_at_both_ends(text, gen.GRISHA) == want

    def test_other_voices_are_untouched(self):
        for voice in (gen.SASHA, gen.BORIS, gen.VALERA, "arina"):
            assert gen.wake_at_both_ends("Робот, который час?", voice) == "Робот, который час?"

    def test_background_phrase_without_wake_is_untouched(self):
        """n304: фон для бэклога -- без вейка и должен остаться без вейка."""
        text = "Молодые люди, спать собираетесь? Третий час ночи."
        assert gen.wake_at_both_ends(text, gen.GRISHA) == text

    def test_idempotent(self):
        once = gen.wake_at_both_ends("Робот, который час, я спрашиваю!", gen.GRISHA)
        assert gen.wake_at_both_ends(once, gen.GRISHA) == once

    def test_the_voice_is_not_changed(self):
        assert ALL_STEPS["n210_grisha_no_name"]["voice"] == gen.GRISHA == "zahar"


class TestGeneratedScenario:
    def test_every_wake_phrase_of_a_fragile_voice_has_wake_at_both_ends(self):
        steps = [
            s for s in ALL_STEPS.values() if s["voice"] in gen.WAKE_FRAGILE_VOICES and gen.has_wake_start(s["text"])
        ]
        assert {s["label"] for s in steps} >= {"n210_grisha_no_name", "n507_wake_under_music"}
        for s in steps:
            assert s["text"].startswith("Робот"), s["label"]
            assert gen._WAKE_END_RE.search(s["text"]), (s["label"], s["text"])

    def test_n210_phrase_is_wake_wrapped_and_keeps_its_meaning(self):
        text = ALL_STEPS["n210_grisha_no_name"]["text"]
        assert text == (
            "Робот, я мимо шла. Имя своё я тебе называть не буду, я старой закалки и не доверяю железкам, " "робот."
        )

    def test_wrapped_phrase_fits_the_vad_window(self):
        for label in ("n210_grisha_no_name", "n507_wake_under_music"):
            s = ALL_STEPS[label]
            found = pb.verdict(label, s["voice"], s["text"], gen.PHRASE_STORE)
            assert found.level != pb.TOO_LONG, found.describe()

    def test_fragile_voice_wake_steps_may_replay_their_phrase(self):
        for label in ("n210_grisha_no_name", "n507_wake_under_music"):
            s = ALL_STEPS[label]
            assert s["recover"] == list(FRAGILE_KINDS), label
            assert eev.recover_kinds({}, s) == FRAGILE_KINDS, label

    def test_background_steps_of_the_same_voice_have_no_replay(self):
        """n304 (Гриша фоном, без вейка) вейка не ждёт -- ему нечего повторять."""
        assert "recover" not in ALL_STEPS["n304_bg_grisha_unknown"]

    def test_no_other_step_asks_for_the_replay(self):
        asking = {label for label, s in ALL_STEPS.items() if "wake_replay" in (s.get("recover") or [])}
        fragile = {
            label
            for label, s in ALL_STEPS.items()
            if s["voice"] in gen.WAKE_FRAGILE_VOICES and gen.has_wake_start(s["text"])
        }
        assert asking == fragile | {"n101_wake_cold_start"}  # #4481: холодный старт, один повтор фразы

    def test_act_that_forbids_the_wake_recovery_has_no_fragile_wake_step(self):
        """Акт 1 проверяет САМ вейк (recover=[silenced]): шаг с recover на уровне шага перекрыл бы запрет акта."""
        for act in gen.ACTS:
            if act["recover"] is not None and "no_wake" not in act["recover"]:
                # #4481: n101 -- повтор фразы (wake_replay) без no_wake, запрет акта на «Робот!» цел
                own = [s for s in act["steps"] if "recover" in s]
                assert all("no_wake" not in s["recover"] for s in own), act["n"]
                assert {s["label"] for s in own} <= {"n101_wake_cold_start"}, act["n"]

    def test_json_on_disk_carries_the_field(self):
        act2 = json.loads((NIGHT / "night_marathon_act2_acquaintance_v1.json").read_text(encoding="utf-8"))
        n210 = next(s for s in act2["steps"] if s["label"] == "n210_grisha_no_name")
        assert n210["recover"] == list(FRAGILE_KINDS)
        assert n210["text"].endswith(", робот.")

    def test_policy_cli_reports_the_replay_for_n210(self):
        out = subprocess.run(
            [
                sys.executable,
                str(EVENTS),
                "policy",
                "--scenario",
                str(NIGHT / "night_marathon_act2_acquaintance_v1.json"),
            ],
            capture_output=True,
            text=True,
            encoding="utf-8",
        )
        assert out.returncode == 0, out.stderr
        rows = dict(line.split("\t") for line in out.stdout.splitlines())
        assert rows["n210_grisha_no_name"] == "no_wake,silenced,wake_replay"
        assert rows["n211_who_do_you_know"] == "no_wake,silenced"


# -- e2e_events: вид по выбору шага -------------------------------------------------------------------------


class TestWakeReplayKind:
    def test_true_does_not_include_the_opt_in_kind(self):
        assert eev.recover_kinds({}, {"text": "Робот, x"}) == ("no_wake", "silenced")
        assert eev.recover_kinds({}, {"text": "Робот, x", "recover": True}) == ("no_wake", "silenced")
        assert eev.recover_kinds({"recover": True}, {"text": "Робот, x"}) == ("no_wake", "silenced")

    def test_step_can_ask_for_it_alone_or_with_the_rest(self):
        assert eev.recover_kinds({}, {"text": "Робот, x", "recover": ["wake_replay"]}) == ("wake_replay",)
        assert eev.recover_kinds({}, {"text": "Робот, x", "recover": ["wake_replay", "no_wake"]}) == (
            "no_wake",
            "wake_replay",
        )

    def test_barge_in_step_never_replays(self):
        step = {"text": "Робот, стоп", "recover": ["wake_replay"], "barge_in": {"after": "speech_started"}}
        assert eev.recover_kinds({}, step) == ()

    def test_own_phrase_filter_does_not_touch_the_replay(self):
        """«Робот!» как фраза шага не лечится «Робот!» -- а повтор фразы шага не имеет своей фразы."""
        assert eev.recover_kinds({}, {"text": "Робот!", "recover": ["no_wake", "wake_replay"]}) == ("wake_replay",)

    def test_typo_is_an_error_not_silence(self):
        with pytest.raises(ValueError):
            eev.recover_kinds({}, {"text": "x", "recover": ["wake_replay2"]})

    def test_marker_accepts_the_kind_for_a_step_but_not_at_act_start(self):
        assert eev.recover_marker("wake_replay", "n210_grisha_no_name", "replay=1") == (
            "E2E_RECOVER wake_replay n210_grisha_no_name replay=1"
        )
        with pytest.raises(ValueError):
            eev.recover_marker("wake_replay", eev.AT_ACT_START)

    def test_the_replay_is_not_a_table_recovery_phrase(self):
        """Своей фразы у повтора нет: таблица RECOVERY (что сказать роботу) его не знает."""
        assert "wake_replay" not in eev.RECOVERY
        assert eev.RECOVERABLE == ("no_wake", "silenced")


# -- bash: run_step с повтором фразы на промах вейка ----------------------------------------------------------

KINDS = "E2E_STEP_RECOVER_KINDS=no_wake,silenced,wake_replay; E2E_STEP_RECOVERED=''; E2E_STEP_WAKE_REPLAYED=''; E2E_STEP_WAKE_ATTEMPT=''"
STEP_PHRASE_HEARD = heard("робот мы тут немного пошумим робот", "{NOW-11}")


def _names(played):
    return [p.rsplit("/", 1)[-1] for p in played]


@needs_bash
class TestRunStepWakeReplay:
    def test_wake_missed_bare_wake_failed_phrase_is_replayed_once_and_step_passes(self, tmp_path):
        """28.10, n210: фраза без вейка -> «Робот!» не помог -> фраза шага ещё раз -> ход есть. Шаг -- recovered."""
        out, err, _, played = _run_step(
            tmp_path, N301, [heard("робот", "{NOW-12}"), STEP_PHRASE_HEARD + REPLY], STEP, KINDS
        )
        assert "RC=0" in out, (out, err)
        assert out.count("E2E_RECOVER no_wake a ") == 1 and out.count("E2E_RECOVER wake_replay a ") == 1, out
        assert "RECOVERED=wake_replay" in out and "PLAY_COUNT=3" in out
        assert _names(played) == ["cmd_a_eq.wav", "rec_no_wake_a_eq.wav", "cmd_a_eq.wav"]
        assert "вейк потерян -- фраза шага ещё раз (wake_attempt=2/3)" in out

    def test_wake_missed_everywhere_is_an_honest_fail_after_three_attempts(self, tmp_path):
        """06.11: «Робот!» не помог, фраза шага -- 2-я и 3-я попытки, все без вейка: FAIL с wake_attempt=3/3, без 4-й."""
        miss = heard("ребят мы пошумим", "{NOW-11}") + no_wake("{NOW-10}")
        out, err, _, played = _run_step(tmp_path, N301, [heard("робот", "{NOW-12}"), miss, miss], STEP, KINDS)
        assert "RC=1" in out and "PLAY_COUNT=4" in out, (out, err)
        assert out.count("E2E_RECOVER wake_replay a ") == 2
        assert "E2E_STEP a FAIL turn_no_wake after_recover_wake_replay wake_attempt=3/3" in out
        assert "повтора фразы нет (attempts_exhausted, wake_attempt=3/3)" in out
        assert _names(played) == ["cmd_a_eq.wav", "rec_no_wake_a_eq.wav", "cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_third_attempt_passes(self, tmp_path):
        miss = heard("ребят мы пошумим", "{NOW-11}") + no_wake("{NOW-10}")
        out, err, _, played = _run_step(
            tmp_path, N301, [heard("робот", "{NOW-12}"), miss, STEP_PHRASE_HEARD + REPLY], STEP, KINDS
        )
        assert "RC=0" in out and "PLAY_COUNT=4" in out, (out, err)
        assert "WAKE_ATTEMPT=3" in out and "RECOVERED=wake_replay" in out
        assert _names(played) == ["cmd_a_eq.wav", "rec_no_wake_a_eq.wav", "cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_step_that_asks_only_for_the_replay_skips_the_bare_wake(self, tmp_path):
        out, err, _, played = _run_step(
            tmp_path,
            N301,
            [STEP_PHRASE_HEARD + REPLY],
            STEP,
            "E2E_STEP_RECOVER_KINDS=wake_replay; E2E_STEP_RECOVERED=''; E2E_STEP_WAKE_REPLAYED=''; E2E_STEP_WAKE_ATTEMPT=''",
        )
        assert "RC=0" in out and "PLAY_COUNT=2" in out, (out, err)
        assert "E2E_RECOVER no_wake" not in out and out.count("E2E_RECOVER wake_replay a ") == 1
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_bare_wake_that_works_means_no_replay(self, tmp_path):
        """«Робот!» подхватил бэклог -- ход есть, фразу шага повторять незачем (второй ход -- дубль)."""
        out, err, _, played = _run_step(tmp_path, N301, [heard("робот", "{NOW-12}") + REPLY], STEP, KINDS)
        assert "RC=0" in out and "PLAY_COUNT=2" in out, (out, err)
        assert "RECOVERED=no_wake" in out and "E2E_RECOVER wake_replay" not in out
        assert _names(played) == ["cmd_a_eq.wav", "rec_no_wake_a_eq.wav"]

    def test_step_that_forbids_recover_is_still_replayed_on_no_wake(self, tmp_path):
        """06.11: глухота STT -- не ответ робота; ``recover: silenced`` (акт 1) повтор фразы не запрещает."""
        out, err, _, played = _run_step(
            tmp_path,
            N301,
            [STEP_PHRASE_HEARD + REPLY],
            STEP,
            "E2E_STEP_RECOVER_KINDS=silenced; E2E_STEP_RECOVERED=''; E2E_STEP_WAKE_REPLAYED=''; E2E_STEP_WAKE_ATTEMPT=''",
        )
        assert "RC=0" in out and "PLAY_COUNT=2" in out, (out, err)
        assert "E2E_RECOVER no_wake" not in out and out.count("E2E_RECOVER wake_replay a ") == 1
        assert "WAKE_ATTEMPT=2" in out
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_wrong_answer_is_not_a_wake_miss_and_is_not_replayed(self, tmp_path):
        """Ход был (робот услышал вейк и ответил, пусть неверно) -- судья отдаёт done, повтор фразы не заказывается:
        неверный ответ ретраит только retry_acceptance."""
        log = heard("робот мы тут немного пошумим", "{NOW-12}") + REPLY
        out, err, _, _ = _run_step(tmp_path, log, [], STEP, KINDS)
        assert "RC=0" in out and "PLAY_COUNT=1" in out, (out, err)
        assert "E2E_RECOVER" not in out and "RECOVERED=\n" in out + "\n"

    def test_silenced_turn_is_not_replayed_by_the_wake_replay(self, tmp_path):
        """Тишина -- другой вид (silenced): её снимает «Робот, говори.», а не повтор фразы шага без снятия тишины."""
        out, err, _, _ = _run_step(
            tmp_path,
            SILENCED_TURN,
            [],
            STEP,
            "E2E_STEP_RECOVER_KINDS=wake_replay; E2E_STEP_RECOVERED=''; " "E2E_STEP_WAKE_REPLAYED=''",
        )
        assert "RC=1" in out and "PLAY_COUNT=1" in out, (out, err)
        assert "E2E_RECOVER wake_replay" not in out

    def test_attempt_budget_is_shared_across_retry_acceptance(self, tmp_path):
        """Повтор шага (retry_acceptance) зовёт run_step с той же меткой: попытки вейка общие, 3 на шаг, не 3 на вызов."""
        chunks = [
            heard("робот", "{NOW-12}") + no_wake("{NOW-11}"),  # PLAY 2: «Робот!» не помог
            STEP_PHRASE_HEARD + REPLY,  # PLAY 3: попытка 2 -- ход есть
            N301,  # PLAY 4: вторая проверка шага снова без вейка (попытка 3)
            N301,  # PLAY 5 не должен случиться
        ]
        body = STEP.replace('echo "RC=$?"', 'echo "RC1=$?"') + "\n" + STEP.replace('echo "RC=$?"', 'echo "RC2=$?"')
        out, err, _, played = _run_step(tmp_path, N301, chunks, body, KINDS, replace=True)
        assert "RC1=0" in out and "RC2=1" in out and "PLAY_COUNT=5" in out, (out, err)
        assert out.count("E2E_RECOVER wake_replay ") == 2, out
        assert "wake_attempt=3/3" in out
        assert _names(played)[-1] == "cmd_a_eq.wav"


# -- e2e_events.wake_retry: решает код (ADR-0148), исключения -----------------------------------------------


class TestWakeRetryDecision:
    PHRASE = "Робот, который час?"

    def test_the_cap_is_one_constant_of_three(self):
        assert eev.WAKE_MAX_ATTEMPTS == 3

    @pytest.mark.parametrize("attempt", [1, 2])
    def test_no_wake_is_replayed_until_the_cap(self, attempt):
        d = eev.wake_retry("cycle", "no_wake", attempt, self.PHRASE)
        assert d == {"retry": 1, "reason": "retry", "attempt": attempt, "max": 3}

    def test_third_attempt_is_the_last(self):
        d = eev.wake_retry("cycle", "no_wake", 3, self.PHRASE)
        assert d["retry"] == 0 and d["reason"] == "attempts_exhausted"

    @pytest.mark.parametrize("verdict", ["done", "silenced", "failed", "perform_ceiling", "ok"])
    def test_a_turn_that_happened_is_never_replayed(self, verdict):
        d = eev.wake_retry("cycle", verdict, 1, self.PHRASE)
        assert d["retry"] == 0 and d["reason"] == "turn_happened"

    def test_step_that_tests_the_wake_itself_is_exempt(self):
        """expect: backlog -- «фраза без вейка уходит в бэклог»: повтор вейка сломал бы проверку."""
        d = eev.wake_retry("backlog", "no_wake", 1, self.PHRASE)
        assert d["retry"] == 0 and d["reason"] == "step_tests_wake"
        assert eev.WAKE_RETRY_EXEMPT_EXPECT == ("backlog",)

    def test_barge_in_step_is_exempt(self):
        assert eev.wake_retry("cycle", "no_wake", 1, self.PHRASE, barge_in=True)["reason"] == "barge_in"

    def test_phrase_without_wake_is_not_replayed(self):
        assert eev.wake_retry("cycle", "no_wake", 1, "Спать пора")["reason"] == "no_wake_word_in_phrase"

    def test_wake_word_must_be_a_whole_word(self):
        assert eev.has_wake("робби, привет") and eev.has_wake("Привет, Роббокс!")
        assert not eev.has_wake("Роботизация цехов")

    def test_every_backlog_step_of_the_marathon_is_exempt_and_every_wake_cycle_step_is_not(self):
        for path in sorted(NIGHT.glob("night_marathon_act*_v1.json")):
            scenario = json.loads(path.read_text(encoding="utf-8"))
            for step in scenario.get("steps", []):
                d = eev.wake_retry(step.get("expect", "cycle"), "no_wake", 1, step.get("text", ""),
                                   bool(step.get("barge_in")))
                if step.get("expect") == "backlog":
                    assert d["retry"] == 0, (path.name, step["label"])
                elif eev.has_wake(step.get("text", "")) and not step.get("barge_in"):
                    assert d["retry"] == 1, (path.name, step["label"])

    def test_cli_answers_like_the_function(self):
        out = subprocess.run(
            [sys.executable, str(EVENTS), "wake-retry", "--expect", "cycle", "--verdict", "no_wake",
             "--attempt", "2", "--text", self.PHRASE, "--barge-in", "0"],
            capture_output=True, text=True, encoding="utf-8",
        )
        assert out.returncode == 0, out.stderr
        assert out.stdout.split() == ["retry=1", "reason=retry", "attempt=2", "max=3"]
