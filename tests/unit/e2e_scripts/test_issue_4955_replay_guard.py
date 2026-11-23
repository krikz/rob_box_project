# -*- coding: utf-8 -*-
"""#4955: харнесс не повторяет уже исполненный ход и не роняет no_wake после повтора мимо восстановления.

Марафон #4950 (n812 акт 8, n903 акт 9): робот в первом ходе исполнил команду (все паттерны шага в окне),
STT исказил фразу, а ветка #4741/#4738 (шаг без stt_keys) проверки исхода не имела -- повторила фразу,
повтор потерял вейк, и ``run_step`` написал жёсткий ``FAIL turn_no_wake after_recover_stt_replay``: повтор
не получал ``step_recover``/``step_wake_replay``, которые даются первой попытке.

Акт 10, n1002: «не знаком» с пробелом не совпал со стемом «незнаком».

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_4955_replay_guard.py -v -o addopts=""
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from tests.unit.e2e_scripts.test_e2e_keyword_scope import keyword_hit
from tests.unit.e2e_scripts.test_issue_3857_event_waits import _l
from tests.unit.e2e_scripts.test_issue_3989_recover import REPLY, SCRIPTS_DIR, _run_step, heard, needs_bash, no_wake

ROOT = Path(__file__).resolve().parents[3]
for _dir in (ROOT / "scripts" / "e2e", SCRIPTS_DIR):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import gen_night_marathon as gen  # noqa: E402

N812 = "Робот, теперь и музыку выключай."
N812_MISS = "роботы бери музыку выключай"
N812_NO_WAKE = "роботы пере музыку выключай"
PATTERN = "✅ Инструмент stop_music выполнен успешно"

STEP = 'run_step "%s" anton a cycle; echo "RC=$?"; echo "WINDOW=${STEP_BEFORE:-}"' % N812
KINDS = "E2E_STEP_RECOVER_KINDS=no_wake,silenced; E2E_STEP_RECOVERED=''; E2E_STEP_STT_REPLAYED=''"

ROBOT_SAID = (
    "Нет, не знаю: имя я говорю только тем, кто сам представился. "
    "Сейчас в кадре одно лицо, и оно мне не знакомо."
)


def _names(played):
    return [p.rsplit("/", 1)[-1] for p in played]


def _scenario(tmp_path, *, patterns):
    """Минимальный сценарий шага без stt_keys: ``_stt_step_executed`` читает music_compare отсюда,
    паттерны -- из ``patterns_json`` (как scenario-цикл харнесса)."""
    path = tmp_path / "scenario.json"
    path.write_text(json.dumps({
        "steps": [{"label": "a", "text": N812, "patterns": patterns}],
    }, ensure_ascii=False), encoding="utf-8")
    pats = json.dumps(patterns, ensure_ascii=False)
    return 'SCENARIO_FILE="%s"; STEP_BEFORE=initial; patterns_json=%s;' % (path.as_posix(), "'" + pats + "'")


@needs_bash
class TestReplayGuard:
    def test_executed_step_without_stt_keys_is_not_replayed(self, tmp_path):
        """A: паттерны шага уже в окне первой попытки, STT исказил фразу -- второго PLAY нет,
        ``E2E_RECOVER stt_replay`` нет, RC=0, окно шага остаётся с первого исполнения."""
        first = heard(N812_MISS, "{NOW-30}") + _l("mcp_server", "{NOW-25}", PATTERN) + REPLY
        out, err, _, played = _run_step(
            tmp_path, first, [], STEP, _scenario(tmp_path, patterns=[PATTERN]) + KINDS)
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=1" in out
        assert "E2E_RECOVER stt_replay" not in out
        assert "E2E_RECOVER" not in out
        assert "команда исполнена, повтора нет" in out
        assert "WINDOW=initial" in out, "окно шага не сдвигается: судим первое исполнение"
        assert _names(played) == ["cmd_a_eq.wav"]

    def test_miss_without_patterns_is_still_replayed(self, tmp_path):
        """Контроль A: паттернов в первом окне нет -- исход не определён, повтор как у #4741."""
        first = heard(N812_MISS, "{NOW-30}") + REPLY
        again = heard("робот теперь и музыку выключай", "{NOW-11}") + REPLY
        out, err, _, played = _run_step(
            tmp_path, first, [again], STEP, _scenario(tmp_path, patterns=[PATTERN]) + KINDS, replace=True)
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=2" in out
        assert out.count("E2E_RECOVER stt_replay a ") == 1
        assert "команда исполнена, повтора нет" not in out
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_no_wake_after_replay_gets_the_same_wake_chance_as_the_first_try(self, tmp_path):
        """B: паттернов в первом окне нет -- повтор случился и потерял вейк (ПРИНЯТО без вейка + backlog
        no_wake). Третья фраза (wake replay) подхвачена и REPLY -- RC=0, PLAY_COUNT=3, в логе и
        ``E2E_RECOVER stt_replay``, и ``E2E_RECOVER wake_replay``, итог ``RECOVERED=wake_replay``.

        Строка ``[backlog] accumulated`` живёт в куске лога робота (фейк не печатает docker logs в stdout
        харнесса). Что судья увидел no_wake -- доказывает сам повтор вейка: ``step_wake_replay`` стартует
        только при ``E2E_REPLY_VERDICT=no_wake``. ``step_recover`` после повтора фразу «Робот!» не играет
        (восстановление уже было) -- это нормально, PLAY только фразы шага."""
        first = heard(N812_MISS, "{NOW-30}") + REPLY
        replay_lost = heard(N812_NO_WAKE, "{NOW-11}") + no_wake("{NOW-10}")
        picked_up = heard("робот теперь и музыку выключай", "{NOW-6}") + REPLY
        assert "[backlog] accumulated reason=no_wake entries=1" in replay_lost
        out, err, _, played = _run_step(
            tmp_path, first, [replay_lost, picked_up], STEP, KINDS, replace=True)
        assert "RC=0" in out, (out, err)
        assert "PLAY_COUNT=3" in out
        assert "восстановление шага уже было" in out
        assert out.count("E2E_RECOVER stt_replay a ") == 1, out
        assert out.count("E2E_RECOVER wake_replay a ") == 1, out
        assert "RECOVERED=wake_replay" in out
        assert "FAIL" not in out
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_no_wake_after_replay_without_pickup_is_an_honest_fail(self, tmp_path):
        """B-негатив: повтор теряет вейк, и подхват не помогает -- честный FAIL (RC=1), не PASS.

        ``wake_retry`` разрешает попытки 1 и 2, поэтому после stt-повтора фраза шага звучит ещё дважды
        (wake_attempt 2/3 и 3/3) и только потом FAIL ``after_recover_wake_replay``. После no_wake харнесс
        пробует инъекцию, она не ложится, и метка ``inject_failed`` — честная часть FAIL. PLAY_COUNT=4:
        первая попытка + stt_replay + два wake replay. ``after_recover_stt_replay`` здесь быть не должно:
        повтор получил тот же шанс, что первая попытка."""
        first = heard(N812_MISS, "{NOW-30}") + REPLY
        lost = heard(N812_NO_WAKE, "{NOW-11}") + no_wake("{NOW-10}")
        out, err, _, played = _run_step(
            tmp_path, first, [lost, lost, lost], STEP, KINDS, replace=True)
        assert "RC=1" in out, (out, err)
        assert (
            "E2E_STEP a FAIL turn_no_wake after_recover_wake_replay "
            "wake_attempt=3/3 stt_attempt=2/3 inject_failed (no_wake)"
        ) in out, out
        assert "after_recover_stt_replay" not in out
        assert "OK recovered" not in out
        assert out.count("E2E_RECOVER stt_replay a ") == 1, out
        assert out.count("E2E_RECOVER wake_replay a ") == 2, out
        assert "PLAY_COUNT=4" in out
        assert _names(played) == ["cmd_a_eq.wav"] * 4


def test_names_unknown_answer_matches_the_spaced_form_and_act10_json_has_it():
    """C: «не знакомо» (пробел) и оборот «имя я говорю только» -- стемы n1002, и они в сгенерированном JSON."""
    said = "🔊 TTS: text='%s'" % ROBOT_SAID
    assert keyword_hit(said, gen.NAMES_UNKNOWN_ANSWER), gen.NAMES_UNKNOWN_ANSWER
    stems_5131 = ("сам назвал", "сами назвал", "назвался", "назвались")
    for stem in ("не знаком", "не знакома", "не знакомо", "имя я говорю только", *stems_5131):
        assert stem in gen.NAMES_UNKNOWN_ANSWER.split("|"), stem
    # голое «не знаю» по-прежнему не стем: оно засчитывало бы «не знаю, о ком ты».
    # «представил» не стем: слово «представился» есть в самом вопросе n1002 (#4105, #5131).
    assert "не знаю" not in gen.NAMES_UNKNOWN_ANSWER.split("|")
    assert not any("представил" in stem for stem in gen.NAMES_UNKNOWN_ANSWER.split("|"))
    assert not keyword_hit("🔊 TTS: text='Не знаю, о ком ты.'", gen.NAMES_UNKNOWN_ANSWER)
    # #5131 (run 45054525357): «только тех, кто сам назвался» -- честный ответ, стем его ловит.
    named = "🔊 TTS: text='Нет, только тех, кто сам назвался. Остальных я узнаю по голосу.'"
    assert keyword_hit(named, gen.NAMES_UNKNOWN_ANSWER)

    act10 = ROOT / ".github" / "e2e" / "scenarios" / "night" / "night_marathon_act10_finale_memory_v1.json"
    steps = {s["label"]: s for s in json.loads(act10.read_text(encoding="utf-8"))["steps"]}
    keywords = steps["n1002_unknown_still_unknown"]["acceptance"]["expected_keywords"]
    assert keywords == [gen.NAMES_UNKNOWN_ANSWER]
    joined = "|".join(keywords)
    for stem in ("не знаком", "не знакома", "не знакомо", "имя я говорю только", *stems_5131):
        assert stem in joined.split("|"), stem
    question = steps["n1002_unknown_still_unknown"]["text"].lower()
    for stem in joined.split("|"):
        assert stem.strip() not in question, stem
