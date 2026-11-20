# -*- coding: utf-8 -*-
"""#4951 (19.11, Telegram 21:36 UTC, решение Шифу): STT не донёс команду шага -- последняя попытка топиком.

Решение товарища Шифу 19.11: «в случае ошибки стт и не приятия сценария делать попытку через топик
отправить оригинальную команду». Когда честные повторы харнесса кончились (no_accept после всех попыток,
мусор STT до потолка, тишина после повтора), харнесс публикует ОРИГИНАЛЬНУЮ фразу шага в
/voice/stt/utterance (source=respeaker, provider=e2e_harness) -- один раз за шаг, только при живом
подписчике. Публикация в пустоту инъекцией не считается. Судья окна ищет в логе «✅ ПРИНЯТО», а пишет его
только stt_node, поэтому строку приёма харнесс пишет сам («✅ ПРИНЯТО (e2e_harness): <фраза>»).

Итог шага после инъекции -- не чистый PASS: ``OK recovered_stt_inject`` (маркер ``E2E_RECOVER stt_inject``).
Ход не состоялся -- ``FAIL no_accept_after_inject``: «робот не реагирует» теперь доказано. Принято, но цикл
не подтверждён -- честный ``cycle_unconfirmed_after_accept``; ошибка LLM/TTS -- ``llm_error``; строка приёма
не записалась -- ``FAIL injection_unverified``. Решение (когда ``inject``, когда ``attempts_exhausted``,
что считать инъекцией) -- в ``scripts/ci/e2e/e2e_events.py`` (ADR-0148), bash только исполняет.

Робот -- ``fake_robot_ssh.sh``: шаблон ``inject_reaction`` в $FAKE_ROBOT_STATE разворачивается
``ros_stubs/rclpy`` при публикации с provider=e2e_harness и отдаётся в ``docker logs`` после строки приёма.

Run:
    /home/ros2/rob_box_project/.venv-core/bin/pytest -v -o addopts='' \\
        tests/unit/e2e_scripts/test_issue_4951_stt_inject_fallback.py
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

from tests.unit.e2e_scripts.test_issue_2846_registration_verdict import extract_scenario_loop
from tests.unit.e2e_scripts.test_issue_3857_event_waits import speech, turn_done
from tests.unit.e2e_scripts.test_issue_3857_run_step import extract_function
from tests.unit.e2e_scripts.test_issue_3989_recover import (
    SCRIPTS_DIR, SILENCED_TURN, _run_step, heard, needs_bash, no_wake,
)
from tests.unit.e2e_scripts.test_issue_4694_stt_garbage_retry import GARBAGE, GARBAGE_AGAIN, KINDS, _names, _scenario

ROOT = Path(__file__).resolve().parents[3]
for _dir in (ROOT / "scripts" / "e2e", SCRIPTS_DIR):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import e2e_events as eev  # noqa: E402

EVENTS = SCRIPTS_DIR / "e2e_events.py"
E2E_SCRIPT = SCRIPTS_DIR / "e2e_voice_test.sh"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"

STEP = 'run_step "Робот, говори." anton a cycle; echo "RC=$?"'
STEP_WAKE = 'run_step "Робот, мы тут немного пошумим" anton a cycle; echo "RC=$?"'
# step_stt_inject и robot_ros живут в основном скрипте, stt_inject_command и _emit_container_log -- в LIB.
EXTRA = ("robot_ros", "step_stt_inject")

# Ход робота после инъекции: команда харнесса принята, LLM/TTS ответили (строки -- как у настоящих нод).
REACTION = (
    speech("started", "aaaa1111", "{NOW-2}")
    + turn_done(ts="{NOW-2}")
    + speech("finished", "aaaa1111", "{NOW-2}", "spoken_chars=12")
)
SESSION = {"agent": "personality", "scenes": []}


def _state(tmp_path: Path, reaction: bool = True) -> Path:
    """Состояние фейк-робота до прогона: снимок сессии (быстрый старт рабочего процесса инъекции)
    и, если ход после инъекции должен состояться, шаблон ``inject_reaction``."""
    state = tmp_path / "state"
    state.mkdir(exist_ok=True)
    (state / "session_state.json").write_text(json.dumps(SESSION), encoding="utf-8")
    if reaction:
        (state / "inject_reaction").write_text(REACTION, encoding="utf-8")
    return state


def _journal(path: Path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _cli(*args, stdin=None):
    return subprocess.run(
        [sys.executable, str(EVENTS), *args],
        input=stdin,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )


# -- решение: потолок повторов отдаёт ход топику (#4951) ----------------------------------------------------

class TestSttMissCeiling:
    def test_the_ceiling_asks_for_injection_when_the_fallback_is_on(self):
        got = eev.stt_miss_retry("Робот, говори.", "робот колории", 3, fallback=True)
        assert got == {"retry": 0, "reason": "inject", "attempt": 3, "max": eev.STT_GARBAGE_MAX_ATTEMPTS}

    def test_the_fallback_is_off_by_default(self):
        got = eev.stt_miss_retry("Робот, говори.", "робот колории", 3)
        assert got["retry"] == 0 and got["reason"] == "attempts_exhausted"

    def test_second_injection_in_the_same_step_does_not_happen(self):
        got = eev.stt_miss_retry("Робот, говори.", "робот колории", 3, fallback=True, injected=True)
        assert got["retry"] == 0 and got["reason"] == "attempts_exhausted"

    def test_below_the_ceiling_the_phrase_replay_wins_over_injection(self):
        got = eev.stt_miss_retry("Робот, говори.", "робот колории", 2, fallback=True)
        assert got == {"retry": 1, "reason": "retry", "attempt": 2, "max": eev.STT_GARBAGE_MAX_ATTEMPTS}

    def test_other_misses_are_not_the_injection_class(self):
        """Падение, перебивание, пустое ПРИНЯТО, голый вейк и узнанная команда -- не «STT не донёс»."""
        miss = dict(text="Робот, говори.", heard="робот колории", attempt=3, fallback=True)
        assert eev.stt_miss_retry(**miss, crash=True)["reason"] == "crash"
        assert eev.stt_miss_retry(**miss, barge_in=True)["reason"] == "barge_in"
        assert eev.stt_miss_retry("Робот, говори.", "", 3, timeout=True, fallback=True)["reason"] == "timeout"
        assert eev.stt_miss_retry("Робот, говори.", "", 3, fallback=True)["reason"] == "nothing_heard"
        assert eev.stt_miss_retry("Робот, говори.", "робот", 3, fallback=True)["reason"] == "bare_wake"
        assert eev.stt_miss_retry("Робот, говори.", "робот говори", 3, fallback=True)["reason"] == "command_heard"
        assert eev.stt_miss_retry("Робот, говори.", "робот говори", 3, timeout=True,
                                  fallback=True)["reason"] == "timeout"


class TestSttGarbageCeiling:
    def test_the_garbage_ceiling_asks_for_injection_too(self):
        got = eev.stt_garbage_retry("Робот, говори.", "робот колории", 3, missing="говори", fallback=True)
        assert got == {"retry": 0, "reason": "inject", "attempt": 3, "max": eev.STT_GARBAGE_MAX_ATTEMPTS}

    def test_garbage_honours_the_same_off_and_once_rules(self):
        args = ("Робот, говори.", "робот колории", 3)
        assert eev.stt_garbage_retry(*args, missing="говори")["reason"] == "attempts_exhausted"
        assert eev.stt_garbage_retry(*args, missing="говори", fallback=True,
                                     injected=True)["reason"] == "attempts_exhausted"
        assert eev.stt_garbage_retry("Робот, говори.", "робот колории", 2, missing="говори",
                                     fallback=True)["retry"] == 1

    def test_garbage_that_is_not_the_stt_miss_class_is_not_injected(self):
        args = ("Робот, говори.", "робот колории", 3)
        assert eev.stt_garbage_retry(*args, missing="говори", allowed=False,
                                     fallback=True)["reason"] == "not_allowed"
        assert eev.stt_garbage_retry("Робот, говори.", "", 3, missing="говори",
                                     fallback=True)["reason"] == "nothing_heard"
        assert eev.stt_garbage_retry("Робот, говори.", "робот говори", 3, missing="-",
                                     fallback=True)["reason"] == "command_heard"
        assert eev.stt_garbage_retry("Робот, говори басом.", "говори", 3, missing="-",
                                     fallback=True)["reason"] == "keys_present"


class TestInjectReport:
    def test_publication_into_the_void_is_not_an_injection(self):
        assert eev.inject_report({"injected": 1, "subscribers": 0}) == {"injected": 0, "subscribers": 0}

    def test_a_live_subscriber_makes_the_injection(self):
        assert eev.inject_report({"injected": 1, "subscribers": 1}) == {"injected": 1, "subscribers": 1}

    def test_a_broken_report_is_not_an_injection(self):
        assert eev.inject_report(None) == {"injected": 0, "subscribers": 0}
        assert eev.inject_report("nope") == {"injected": 0, "subscribers": 0}
        assert eev.inject_report({"subscribers": 1}) == {"injected": 0, "subscribers": 0}
        assert eev.inject_report({"injected": 1}) == {"injected": 0, "subscribers": 0}
        assert eev.inject_report({"injected": 1, "subscribers": "много"}) == {"injected": 0, "subscribers": 0}

    def test_subscribers_are_counted_as_a_number(self):
        assert eev.inject_report({"injected": 1, "subscribers": "2"}) == {"injected": 1, "subscribers": 2}

    def test_an_honest_zero_stays_zero(self):
        assert eev.inject_report({"injected": 0, "subscribers": 3}) == {"injected": 0, "subscribers": 3}


class TestInjectCli:
    def test_stt_miss_on_the_ceiling_reports_inject(self):
        p = _cli("stt-miss", "--text", "Робот, говори.", "--heard", "робот колории",
                 "--attempt", "3", "--fallback", "1")
        assert p.returncode == 0, p.stderr
        assert p.stdout.splitlines() == ["retry=0", "reason=inject", "attempt=3",
                                         "max=%d" % eev.STT_GARBAGE_MAX_ATTEMPTS]

    def test_stt_miss_keeps_the_honest_exhaustion_without_the_flag_and_after_injection(self):
        base = ["stt-miss", "--text", "Робот, говори.", "--heard", "робот колории", "--attempt", "3"]
        off = _cli(*base)
        assert off.stdout.splitlines()[1] == "reason=attempts_exhausted"
        once = _cli(*base, "--fallback", "1", "--injected", "1")
        assert once.stdout.splitlines()[1] == "reason=attempts_exhausted"

    def test_stt_retry_on_the_ceiling_reports_inject(self):
        p = _cli("stt-retry", "--text", "Робот, говори.", "--heard", "робот колории", "--attempt", "3",
                 "--missing", "говори", "--allowed", "1", "--fallback", "1")
        assert p.returncode == 0, p.stderr
        assert p.stdout.splitlines() == ["retry=0", "reason=inject", "attempt=3",
                                         "max=%d" % eev.STT_GARBAGE_MAX_ATTEMPTS]

    def test_inject_requires_a_live_subscriber(self):
        void = _cli("inject", stdin='{"injected": 1, "subscribers": 0}')
        assert void.stdout.splitlines() == ["injected=0", "subscribers=0"]
        live = _cli("inject", stdin='{"injected": 1, "subscribers": 2}')
        assert live.stdout.splitlines() == ["injected=1", "subscribers=2"]


# -- маркер и проводка --------------------------------------------------------------------------------------

def test_stt_inject_is_a_step_recovery_kind_and_never_an_act_phrase():
    """Маркер ``E2E_RECOVER stt_inject`` -- вид восстановления ШАГА: сценарий может его назвать, у акта его нет."""
    assert eev.STT_INJECT == "stt_inject"
    assert eev.STT_INJECT in eev.STEP_RECOVERY_KINDS
    assert eev.STT_INJECT not in eev.RECOVERY and eev.STT_INJECT not in eev.OPT_IN_RECOVERY
    assert eev.recover_kinds({}, {"text": "Робот, говори.", "recover": ["stt_inject"]}) == ("stt_inject",)
    assert eev.recover_marker("stt_inject", "a", "text=«Робот, говори.»") == \
        "E2E_RECOVER stt_inject a text=«Робот, говори.»"


def test_the_injection_is_wired_once_per_step_and_judged_by_the_topic():
    """ADR-0148: решение -- в e2e_events.py; bash только передаёт --fallback/--injected и исполняет ответ."""
    main_src = E2E_SCRIPT.read_text(encoding="utf-8")
    lib_src = LIB.read_text(encoding="utf-8")
    # публикация одна: provider=e2e_harness, честная строка приёма, судья отчёта
    assert lib_src.count("stt_inject_command() {") == 1
    assert lib_src.count('msg.source, msg.provider = Utterance.SOURCE_RESPEAKER, "e2e_harness"') == 1
    assert lib_src.count('_emit_container_log "✅ ПРИНЯТО (e2e_harness): ${text}"') == 1
    assert lib_src.count('python3 "$(_events_py)" inject') == 1
    # входы: финал шага + потолки обоих повторов + no_wake после wake_replay (#4951, акт 2 n210)
    assert main_src.count('step_stt_inject "$label"') == 4
    assert main_src.count('if [ "$reason" = "inject" ] && step_stt_inject "$label" "${text:-}"; then') == 2
    assert main_src.count('if step_stt_inject "$label" "${text:-}"; then') == 1
    assert main_src.count('--fallback "${E2E_STT_INJECT_FALLBACK:-1}" --injected "${E2E_STEP_STT_INJECTED:-0}"') == 2
    assert main_src.count("step_stt_inject() {") == 1
    # один заход за шаг и маркер: инъекция не маскируется под чистый PASS
    assert main_src.count("E2E_STEP_STT_INJECT_TRIED=1") == 1
    assert main_src.count("E2E_STEP_STT_INJECTED=1") == 1
    assert main_src.count('E2E_STEP_RECOVERED="stt_inject"') == 1
    assert main_src.count('E2E_STEP_STT_INJECTED=""') == 1
    assert main_src.count('E2E_STEP_STT_INJECT_TRIED=""') == 1
    assert main_src.count("E2E_STT_INJECT_FALLBACK") == 5


# -- bash: run_step делает инъекцию на потолке (фейк-робот) -------------------------------------------------

@needs_bash
class TestRunStepInjection:
    def test_no_accept_then_the_topic_delivers_the_command_and_the_turn_is_judged(self, tmp_path):
        """Акцепта нет все попытки -- последняя попытка топиком; ход робота состоялся, итог recovered_stt_inject."""
        state = _state(tmp_path)
        out, err, _, played = _run_step(tmp_path, "", [], STEP, "E2E_ACCEPT_WINDOW=3", extra=EXTRA)
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=1" in out
        assert "RECOVERED=stt_inject" in out
        assert out.count("E2E_RECOVER stt_inject a ") == 1 and "text=«Робот, говори.»" in out
        assert "ход после инъекции состоялся" in out
        assert "E2E_STEP a FAIL" not in out
        assert _names(played) == ["cmd_a_eq.wav"]
        assert _journal(state / "utterances") == [
            {"text": "Робот, говори.", "source": "respeaker", "provider": "e2e_harness"}
        ]
        assert "✅ ПРИНЯТО (e2e_harness): Робот, говори." in (state / "pid1_stdout").read_text(encoding="utf-8")

    def test_garbage_ceiling_tries_the_topic_once_and_the_turn_is_judged(self, tmp_path):
        """Мусор STT до потолка -- раньше честный FAIL #4694; теперь последняя попытка топиком, маркер stt_inject."""
        state = _state(tmp_path)
        out, err, _, played = _run_step(
            tmp_path,
            GARBAGE,
            [GARBAGE_AGAIN, GARBAGE_AGAIN],
            STEP,
            _scenario(tmp_path) + KINDS,
            replace=True,
            extra=EXTRA,
        )
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=%d" % eev.STT_GARBAGE_MAX_ATTEMPTS in out
        assert out.count("E2E_RECOVER stt_replay a ") == eev.STT_GARBAGE_MAX_ATTEMPTS - 1
        assert out.count("E2E_RECOVER stt_inject a ") == 1
        assert "RECOVERED=stt_inject" in out
        assert "E2E_STEP a FAIL" not in out
        assert "✅ ПОЛНЫЙ ЦИКЛ" in out
        assert _names(played) == ["cmd_a_eq.wav"] * eev.STT_GARBAGE_MAX_ATTEMPTS
        assert _journal(state / "utterances") == [
            {"text": "Робот, говори.", "source": "respeaker", "provider": "e2e_harness"}
        ]

    def test_accepted_but_no_turn_after_injection_is_an_honest_fail(self, tmp_path):
        """Команда в топике, ПРИНЯТО есть, хода нет -- cycle_unconfirmed, не no_accept (#4951)."""
        state = _state(tmp_path, reaction=False)
        out, err, _, _ = _run_step(
            tmp_path, "", [], STEP, "E2E_ACCEPT_WINDOW=3; E2E_REACTION_WINDOW=5", extra=EXTRA,
        )
        assert "RC=1" in out, (out, err)
        assert "ПРИНЯТО после инъекции есть, но цикл LLM/TTS не подтверждён за потолок" in out
        assert "E2E_STEP a FAIL cycle_unconfirmed_after_accept after_recover_stt_inject" in out
        assert "E2E_RECOVER stt_inject a " in out
        assert len(_journal(state / "utterances")) == 1

    def test_fallback_off_keeps_the_previous_honest_fail_and_the_topic_is_dark(self, tmp_path):
        state = _state(tmp_path)
        out, err, _, _ = _run_step(
            tmp_path, "", [], STEP, "E2E_ACCEPT_WINDOW=3; E2E_STT_INJECT_FALLBACK=0", extra=EXTRA,
        )
        assert "RC=1" in out, (out, err)
        assert "инъекция команды в топик выключена (E2E_STT_INJECT_FALLBACK=0)" in out
        assert "E2E_STEP a FAIL no_accept" in out
        assert not (state / "utterances").exists()

    def test_the_topic_is_tried_once_per_step(self, tmp_path):
        """Инъекция в шаге уже была -- второй публикации нет, шаг судится прежним честным FAIL."""
        state = _state(tmp_path)
        out, err, _, _ = _run_step(
            tmp_path, "", [], STEP, "E2E_ACCEPT_WINDOW=3; E2E_STEP_STT_INJECT_TRIED=1", extra=EXTRA,
        )
        assert "RC=1" in out, (out, err)
        assert "инъекция в этом шаге уже была -- второго захода нет (#4951)" in out
        assert "E2E_STEP a FAIL no_accept" in out
        assert not (state / "utterances").exists()

    def test_no_subscriber_means_no_injection_and_the_previous_honest_fail(self, tmp_path):
        """Подписчика нет -- публиковать некуда: шаг судится как раньше (честный FAIL no_accept)."""
        state = _state(tmp_path)
        out, err, _, _ = _run_step(
            tmp_path, "", [], STEP, "E2E_ACCEPT_WINDOW=3; stt_inject_command() { return 1; }", extra=EXTRA,
        )
        assert "RC=1" in out, (out, err)
        assert "инъекции не было -- шаг судится как раньше (честный FAIL)" in out
        assert "E2E_STEP a FAIL no_accept" in out
        assert not (state / "utterances").exists()

    def test_publication_without_the_accept_line_is_its_own_fail(self, tmp_path):
        """Реплика ушла, а строки приёма в логе нет -- судье нечего судить: injection_unverified, не no_accept."""
        state = _state(tmp_path)
        out, err, _, _ = _run_step(
            tmp_path, "", [], STEP, "E2E_ACCEPT_WINDOW=3; _emit_container_log() { return 1; }", extra=EXTRA,
        )
        assert "RC=1" in out, (out, err)
        assert "E2E_STEP a FAIL injection_unverified" in out
        assert "не записана в лог контейнера" in out
        assert "E2E_STEP a FAIL no_accept" not in out
        assert len(_journal(state / "utterances")) == 1
        assert not (state / "pid1_stdout").exists()


# -- bash: no_wake после wake_replay -- последняя попытка топиком (#4951, акт 2 n210) ---------------------

def _no_wake_log():
    """Фраза шага принята без вейка и ушла в бэклог -- тот же формат, что no_wake() в #3989."""
    return heard("ребят мы пошумим", "{NOW-11}") + no_wake("{NOW-10}")


_WAKE_KINDS = (
    "E2E_STEP_RECOVER_KINDS=no_wake,silenced,wake_replay; E2E_STEP_RECOVERED=''; "
    "E2E_STEP_WAKE_REPLAYED=''; E2E_STEP_WAKE_ATTEMPT=''"
)


@needs_bash
class TestNoWakeThenInject:
    """Акт 2 n210 (run 44789967142): фраза без вейка, wake_replay исчерпан, инъекции в логе не было.

    ``step_wake_replay`` возвращает 0, и case 4) писал ``FAIL turn_no_wake`` до вызова ``step_stt_inject``.
    Теперь перед FAIL -- один вызов. silenced не трогаем (#3989).
    """

    def test_wake_exhausted_then_the_topic_delivers_the_phrase_and_the_step_is_recovered(self, tmp_path):
        state = _state(tmp_path)
        miss = _no_wake_log()
        out, err, _, played = _run_step(
            tmp_path, miss, [heard("робот", "{NOW-12}"), miss, miss], STEP_WAKE, _WAKE_KINDS, extra=EXTRA,
        )
        assert "RC=0" in out, (out, err)
        assert "E2E_STEP a OK recovered_stt_inject after_recover_wake_replay wake_attempt=3/3" in out
        assert "RECOVERED=stt_inject" in out
        assert out.count("E2E_RECOVER stt_inject a ") == 1 and "text=«Робот, мы тут немного пошумим»" in out
        assert "E2E_STEP a FAIL" not in out
        assert "ход после инъекции есть" in out
        assert _names(played) == ["cmd_a_eq.wav", "rec_no_wake_a_eq.wav", "cmd_a_eq.wav", "cmd_a_eq.wav"]
        assert _journal(state / "utterances") == [
            {"text": "Робот, мы тут немного пошумим", "source": "respeaker", "provider": "e2e_harness"}
        ]

    def test_inject_that_does_not_land_keeps_the_honest_fail_with_inject_failed(self, tmp_path):
        state = _state(tmp_path)
        miss = _no_wake_log()
        out, err, _, played = _run_step(
            tmp_path, miss, [heard("робот", "{NOW-12}"), miss, miss], STEP_WAKE,
            _WAKE_KINDS + "; stt_inject_command() { return 1; }", extra=EXTRA,
        )
        assert "RC=1" in out, (out, err)
        assert "E2E_STEP a FAIL turn_no_wake after_recover_wake_replay wake_attempt=3/3 inject_failed" in out
        assert "E2E_STEP a OK" not in out
        assert "инъекции не было -- шаг судится как раньше" in out
        assert not (state / "utterances").exists()
        assert _names(played) == ["cmd_a_eq.wav", "rec_no_wake_a_eq.wav", "cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_fallback_off_does_not_publish_and_marks_inject_failed(self, tmp_path):
        state = _state(tmp_path)
        miss = _no_wake_log()
        out, err, _, _ = _run_step(
            tmp_path, miss, [heard("робот", "{NOW-12}"), miss, miss], STEP_WAKE,
            _WAKE_KINDS + "; E2E_STT_INJECT_FALLBACK=0", extra=EXTRA,
        )
        assert "RC=1" in out, (out, err)
        assert "инъекция команды в топик выключена (E2E_STT_INJECT_FALLBACK=0)" in out
        assert "E2E_STEP a FAIL turn_no_wake after_recover_wake_replay wake_attempt=3/3 inject_failed" in out
        assert not (state / "utterances").exists()

    def test_already_injected_step_is_not_injected_again(self, tmp_path):
        state = _state(tmp_path)
        miss = _no_wake_log()
        out, err, _, _ = _run_step(
            tmp_path, miss, [heard("робот", "{NOW-12}"), miss, miss], STEP_WAKE,
            _WAKE_KINDS + "; E2E_STEP_STT_INJECTED=1", extra=EXTRA,
        )
        assert "RC=1" in out, (out, err)
        assert "инъекция в этом шаге уже была -- второго захода нет (#4951)" in out
        assert "E2E_STEP a FAIL turn_no_wake after_recover_wake_replay wake_attempt=3/3 inject_failed" in out
        assert not (state / "utterances").exists()

    def test_silenced_turn_is_not_injected(self, tmp_path):
        """#3989: тишина -- не класс инъекции. case 4) для silenced не зовёт step_stt_inject."""
        state = _state(tmp_path)
        out, err, _, played = _run_step(
            tmp_path, SILENCED_TURN, [], STEP_WAKE,
            "E2E_STEP_RECOVER_KINDS=wake_replay; E2E_STEP_RECOVERED=''; E2E_STEP_WAKE_REPLAYED=''",
            extra=EXTRA,
        )
        assert "RC=1" in out, (out, err)
        assert "E2E_STEP a FAIL turn_silenced" in out
        assert "inject_failed" not in out
        assert "инъекц" not in out
        assert "/voice/stt/utterance" not in out
        assert not (state / "utterances").exists()
        assert _names(played) == ["cmd_a_eq.wav"]


# -- bash: итог шага в scenario-цикле -- recovered, не чистый PASS -------------------------------------------

@needs_bash
def test_scenario_loop_marks_the_injected_step_recovered_and_summary_counts_it(tmp_path):
    scen = tmp_path / "scenario.json"
    scen.write_text(
        json.dumps({"steps": [{"label": "n1701_stt_miss", "text": "Робот, говори.", "expect": "cycle"}]},
                   ensure_ascii=False),
        encoding="utf-8",
    )
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
        extract_function("check_music_compare"),
        extract_function("write_summary"),
        # run_step: шаг прошёл инъекцией (как сделал бы step_stt_inject)
        'run_step() { case "$3" in n1701*) E2E_STEP_RECOVERED=stt_inject ;; esac; return 0; }',
        "parse_transcript() { :; }",
        'load_recover_policy "%s"' % scen.as_posix(),
        'parse_scenario_to_tsv "%s" "$OUT_DIR/scenario_parsed.txt"' % scen.as_posix(),
        'LAST_STEP_SPEECH_FILE="$OUT_DIR/.last_step_speech.txt"; : > "$LAST_STEP_SPEECH_FILE"',
        extract_scenario_loop(),
        "E2E_ACT_RECOVERED=''",
        "write_summary",
    ])
    p = subprocess.run(["bash", "-s"], input=script.encode("utf-8"), capture_output=True, timeout=120)
    out, err = p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace")
    assert "E2E_STEP n1701_stt_miss OK recovered_stt_inject" in out, (out, err)
    assert re.search(r"E2E_SUMMARY verdict=PASS steps=1/1 ok fail=0 skip=0 stand_play_failed=0 recovered=1 ", out), out
    summary = json.loads((out_dir / "summary.json").read_text(encoding="utf-8"))
    assert summary["steps"]["recovered"] == 1
    assert summary["steps"]["recovered_labels"] == ["n1701_stt_miss"]
