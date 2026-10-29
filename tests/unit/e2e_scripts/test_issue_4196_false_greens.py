# -*- coding: utf-8 -*-
"""#4196 (эпик #3683, приёмка #3710): харнесс марафона-3 28.10 -- ложно зелёные шаги и каскады без предусловия.

Сверка марафона-3 по логу робота (образ dev-513a70c): 3 ложно зелёных (n307, n507, n605), каскады n312/n511 (музыку не
включил/выключил предыдущий шаг), искажённый STT вход n411/n506; разбор логов (#4020) добавил n709 (run 40758476879,
20:00:04): Tier-1 «останови» снял деку, остаток фразы ушёл модели, и она сказала «сделаю потише… дослушивает шёпотом» без тула.
Мерило Шифу: 0 ложно зелёных. Судят те же функции, что у живого харнесса: ``e2e_tool_match.step_verdict`` (acceptance),
``patterns_missing`` (patterns), ``e2e_events.skip_verdict`` / ``stt_verdict`` (предусловие, сверка ПРИНЯТО).

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_4196_false_greens.py -v -o addopts=""
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict

import pytest

from tests.unit.e2e_scripts.test_issue_3857_event_waits import _l
from tests.unit.e2e_scripts.test_issue_3989_recover import (
    REPLY,
    SCRIPTS_DIR,
    _run_step,
    heard,
    needs_bash,
)

ROOT = Path(__file__).resolve().parents[3]
for _dir in (ROOT / "scripts" / "e2e", SCRIPTS_DIR):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import e2e_events as eev  # noqa: E402
import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

NIGHT = ROOT / ".github" / "e2e" / "scenarios" / "night"
EVENTS = SCRIPTS_DIR / "e2e_events.py"
DONE = (
    "📊 [turn] turn_id=t1 epoch=1 agent=main stage=done status=finished latency_ms=900 llm_calls=2 llm_provider=x "
    "tokens=10"
)
T0 = 1792793000.0


def _tts(text: str) -> str:
    return (
        "[tts_node-4] [INFO] [1792795079.69] [tts_node]: 🔊 TTS: speech_id=764009bb, dialogue_id=None, "
        "batch=None None/None, voice=default, lang=default, text='%s'" % text
    )


def _steps() -> Dict[str, Dict[str, Any]]:
    return {s["label"]: s for act in gen.ACTS for s in act["steps"]}


def _verdict(label: str, *lines: str) -> Dict[str, Any]:
    step = _steps()[label]
    log = "\n".join(lines)
    missing = etm.patterns_missing(log, step["patterns"])
    acc = etm.step_verdict(step.get("acceptance", {}), log, label=label)
    return {"missing": missing, "acc": acc, "pass": not missing and acc["pass"]}


def _mcp(ts: float, msg: str) -> str:
    return "[mcp_server-9] [INFO] [%.3f] [mcp_server]: %s" % (ts, msg)


def started(ts: float, track: str = "req77:01") -> str:
    return _mcp(ts, "🎵 [music v2] started track_id=%s clock_beat=9792.069 start_beat=9792.0 phase_in_form=0.0 "
                    "late_beats=0.069 bpm=90 deck=A form_beats=192.0 players_aligned=True latency_s=0.5" % track)


def stopped(ts: float, track: str = "req77:01", reason: str = "user_stop") -> str:
    return _mcp(ts, "🎵 [music v2] stop track_id=%s reason=%s" % (track, reason))


def finished(ts: float, track: str = "req77:01") -> str:
    return _mcp(ts, "🎵 [music v2] finished track_id=%s" % track)


def edited(ts: float, track: str = "req77:01") -> str:
    return _mcp(ts, "🎚️ [music v2] edit applied track_id=%s at_beat=9856 bpm=100" % track)


def _skip(label: str, *lines: str) -> Dict[str, object]:
    rule = eev.skip_rule(_steps()[label])
    assert rule is not None, label
    return eev.skip_verdict(rule, "%.3f\n%s" % (T0 + 500, "\n".join(lines)))


# ── п.1: n307 -- число людей ───────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("said", [
    "Четверо: Саша, Борис, Гриша и Валера.", "В комнате было четыре человека.", "Я насчитал 4 голоса.",
    "По голосам — четверых, включая тебя.",
])
def test_n307_the_right_count_is_green(said):
    assert _verdict("n307_count_voices", _tts(said), DONE)["pass"], said


@pytest.mark.parametrize("said", [
    "Было пять человек.", "Насчитал шестерых... нет, шестеро.", "Семеро, если считать меня.", "Здесь было 5 человек.",
    "Тут 7 голосов.",
])
def test_n307_a_wrong_count_is_red(said):
    got = _verdict("n307_count_voices", _tts(said), DONE)
    assert not got["pass"] and got["acc"]["forbidden_said"], said


@pytest.mark.parametrize("said", ["Было много народу.", "Опять тут шумно, не сосчитать.", "Несколько человек."])
def test_n307_no_count_is_red_and_the_word_opyat_is_not_a_count(said):
    got = _verdict("n307_count_voices", _tts(said), DONE)
    assert not got["pass"] and got["acc"]["missing_keywords"] == [gen.COUNT_FOUR] and not got["acc"]["forbidden_said"], said


def test_n307_wrong_count_beats_the_right_one_named_next_to_it():
    got = _verdict("n307_count_voices", _tts("Четверо говорили, а всего пять человек."), DONE)
    assert not got["pass"], got


def test_n307_the_four_voices_of_act3_are_the_cast():
    act3 = next(a for a in gen.ACTS if a["n"] == 3)["steps"]
    voices = {s["voice"] for s in act3}
    assert voices == {gen.SASHA, gen.BORIS, gen.GRISHA, gen.VALERA}, "число в ключе = число голосов акта"
    assert gen.COUNT_FOUR.startswith("четыр")


# ── n306: «кого узнал, а кого нет» (марафон-4, 21:55:10) ───────────────────────────────────────────────────


@pytest.mark.parametrize("said", [
    "Борис — узнал. Саша тоже. А третий голос не узнал.",  # марафон-4, попытка 2
    "Бориса узнал, а вот голос женщины не узнала.", "Борис знакомый, остальных не узнаю.",
    "Борис это Борис, а кто ещё говорил — не знаю, кто.", "Борис есть, один голос незнакомый.",
    "Борис, а остальные неопознанные.",
])
def test_n306_any_wording_of_the_unrecognized_is_green(said):
    assert _verdict("n306_who_was_talking", _tts(said), DONE)["pass"], said


@pytest.mark.parametrize("said", [
    "Всех узнал: Борис, Саша и остальные.",  # в бэклоге был незнакомый -- честный красный
    "Борис говорил, больше никого не было.", "Узнал Бориса и Сашу.",
    "Не знаю, о чём ты.",  # нет Бориса
])
def test_n306_claiming_to_know_everyone_stays_red(said):
    assert not _verdict("n306_who_was_talking", _tts(said), DONE)["pass"], said


def test_n306_key_is_the_one_constant():
    assert _steps()["n306_who_was_talking"]["acceptance"]["expected_keywords"] == ["Борис", gen.UNKNOWN_VOICE_ANSWER]
    assert "незнаком" in gen.UNKNOWN_VOICE_ANSWER and "неопознан" in gen.UNKNOWN_VOICE_ANSWER
    assert "не опознал" in gen.UNKNOWN_VOICE_ANSWER and "не опознаю" in gen.UNKNOWN_VOICE_ANSWER


# ── n1002: «тех, кто не представился, по именам знаешь?» (марафон-4, 22:55:38) ─────────────────────────────


@pytest.mark.parametrize("said", [
    "Нет, Саша, не знаю: если человек не представился, я его никак не называю. Безымянных я просто не зову по имени.",
    "Безымянных по именам не зову.", "Тех, кто не представился, я по имени не называю.", "По именам не знаю, они без имён.",
])
def test_n1002_honest_answers_about_the_nameless_are_green(said):
    assert _verdict("n1002_unknown_still_unknown", _tts(said), DONE)["pass"], said


@pytest.mark.parametrize("said", [
    "Тех, кто не представился, я знаю по именам: Гриша и Валера.",  # выдумал имена -- красный
    "Не знаю, о ком ты.", "Тех, кто не представился, ты по именам знаешь?",  # эхо вопроса
])
def test_n1002_invented_names_and_the_echo_of_the_question_stay_red(said):
    assert not _verdict("n1002_unknown_still_unknown", _tts(said), DONE)["pass"], said


# ── п.2/4: предусловие «музыка играет» -- n507, n511, n312 ──────────────────────────────────────────────────


def test_the_precondition_is_one_mechanism_with_two_cascade_reasons():
    steps = _steps()
    assert steps["n312_stop_music"]["skip_when"] == {"reason": "NO_MUSIC_N310", "log": gen.MUSIC_DECK_EVENT, "deck": "idle"}
    for label in ("n507_wake_under_music", "n511_stop_music"):
        assert steps[label]["skip_when"] == {"reason": "NO_MUSIC_N506", "log": gen.MUSIC_DECK_EVENT, "deck": "idle"}, label
    assert eev.SKIP_DECKS == (eev.DECK_IDLE, eev.DECK_PLAYING), "`ended` поглощён `idle`; `playing` -- обратное (n512)"
    assert {r.deck for r in gen.SKIP_WHEN.values() if r.deck} == {eev.DECK_IDLE, eev.DECK_PLAYING}


@pytest.mark.parametrize("label", ["n507_wake_under_music", "n511_stop_music", "n312_stop_music"])
@pytest.mark.parametrize("lines, why", [
    ((), "музыка не заигрывала вовсе"),
    ((started(T0 + 10), stopped(T0 + 50)), "дека снята стопом (n506 в марафоне-3)"),
    ((started(T0 + 10), finished(T0 + 50)), "трек доиграл сам"),
    ((started(T0 + 10), edited(T0 + 20), stopped(T0 + 30), edited(T0 + 31)), "правка deck не оживляет"),
])
def test_step_is_skipped_when_the_deck_is_empty(label, lines, why):
    got = _skip(label, *lines)
    assert got["fetch"] == 1 and got["skip"] == _steps()[label]["skip_when"]["reason"], (why, got)
    assert got["evidence"] != "-"


@pytest.mark.parametrize("label", ["n507_wake_under_music", "n511_stop_music", "n312_stop_music"])
@pytest.mark.parametrize("lines, why", [
    ((started(T0 + 10),), "трек играет"),
    ((started(T0 + 10), edited(T0 + 20)), "правка играющего трек не снимает"),
    ((started(T0 + 5), stopped(T0 + 6), started(T0 + 10, "req77:02")), "после стопа заиграл новый"),
])
def test_step_is_played_while_music_plays(label, lines, why):
    assert _skip(label, *lines) == {"fetch": 1, "skip": "-", "evidence": "-"}, why


def test_unreadable_log_does_not_skip_the_step_it_plays_loudly():
    rule = eev.skip_rule(_steps()["n507_wake_under_music"])
    assert eev.skip_verdict(rule, "Traceback\n" + stopped(T0)) == {"fetch": 0, "skip": "-", "evidence": "-"}


def test_n311_guards_the_music_between_n310_and_n312():
    """Стоп посреди n311 иначе выглядел бы для n312 как «каскад от n310» -- и SKIP спрятал бы дефект."""
    assert "stop_music" in _steps()["n311_who_asked"]["acceptance"]["must_not_call"]


def test_cli_skip_reads_the_deck_rule_from_the_scenario_file(tmp_path):
    scenario = json.loads((NIGHT / "night_marathon_act5_renardo_live_v1.json").read_text(encoding="utf-8"))
    copy = tmp_path / "e2e_scenario.json"
    copy.write_text(json.dumps(scenario, ensure_ascii=False), encoding="utf-8")

    def run(*lines: str):
        return subprocess.run(
            [sys.executable, str(EVENTS), "skip", "--scenario", str(copy), "--label", "n507_wake_under_music"],
            input="%.3f\n%s" % (T0 + 500, "\n".join(lines)), capture_output=True, text=True, encoding="utf-8", check=True,
        ).stdout.splitlines()

    assert "skip=NO_MUSIC_N506" in run(started(T0), stopped(T0 + 9))
    assert "skip=-" in run(started(T0))


# ── п.3: n605 -- сет объявлен ──────────────────────────────────────────────────────────────────────────────

SET_TOOL = _mcp(T0 + 3, "✅ Инструмент dj_set выполнен успешно")
PROGRESS = _tts("Секунду, музыка запускается.")
SET_OF_THREE_SYNTH = _mcp(T0 + 2, "🎛️ [dj_set] set04217 длина сета: 3 (названо), энергия=[0.4, 0.7, 0.9]")
SET_OF_THREE_GEN = _mcp(T0 + 2, "🎛️ [gen set] set18211 ['cache:a', 'cache:b', 'cache:c']: ['rock', 'rock', 'rock'] 3 трека, кэш 3, "
                                "запас 0, мост=нет gen=ok → True")
DJ_LINE = _mcp(T0 + 80, "🗣️ [dj line] set04217:02:B трек 2/3 phase=peak source=llm hook='Тетрис' text='Дальше Тетрис!'")
SPEECH = "🗣️ SpeechEvent started speech_id=s1 agent=main epoch=1 text_len=20"


def test_n605_any_speech_is_no_longer_enough():
    got = _verdict("n605_dj_set_plan", SPEECH, PROGRESS, SET_TOOL, DONE)
    assert not got["pass"] and got["missing"] == [gen.DJ_SET_ANNOUNCED], got


@pytest.mark.parametrize("plan", [SET_OF_THREE_SYNTH, SET_OF_THREE_GEN, DJ_LINE])
def test_n605_speech_tool_and_an_announced_set_is_green(plan):
    got = _verdict("n605_dj_set_plan", SPEECH, SET_TOOL, plan, DONE)
    assert got["pass"], got


@pytest.mark.parametrize("plan", [
    _mcp(T0 + 2, "🎛️ [dj_set] set04217 длина сета: 8 (по умолчанию), энергия=[0.4]"),
    _mcp(T0 + 2, "🎛️ [dj_set] set04217 длина сета: 3 (по умолчанию), энергия=[0.4]"),  # три -- не слова человека
    _mcp(T0 + 2, "🎛️ [gen set] set18211 ['cache:a']: ['rock'] 13 трека, кэш 3 → True"),
    _mcp(T0 + 2, "🗣️ [dj line] set04217:02:B не озвучена"),
])
def test_n605_a_set_of_another_length_or_a_silent_line_is_not_an_announcement(plan):
    got = _verdict("n605_dj_set_plan", SPEECH, SET_TOOL, plan, DONE)
    assert not got["pass"], got


def test_n605_still_needs_the_tool_and_the_speech():
    assert not _verdict("n605_dj_set_plan", SET_OF_THREE_SYNTH, DONE)["pass"], "ни речи, ни тула"
    assert not _verdict("n605_dj_set_plan", SPEECH, SET_OF_THREE_SYNTH, DONE)["pass"], "тул не выполнен успешно"


# ── п.5: ПРИНЯТО против ключевых слов шага -- n411, n506 ────────────────────────────────────────────────────


def test_stt_verdict_finds_the_keys_ignoring_case_and_yo():
    step = {"label": "x", "stt_keys": ["говори", "бас"]}
    assert eev.stt_verdict(step, "Робот, ГОВОРИ басом, как диктор") == {"keys": 2, "stt_ok": 1, "missing": "-"}


def test_stt_verdict_reports_the_lost_key_n411_28_10():
    step = _steps()["n411_pitch_bass"]
    assert eev.stt_verdict(step, "робот гарри басом как диктор в старом кино") == {"keys": 2, "stt_ok": 0, "missing": "говори"}


def test_stt_verdict_n506_lost_tempo():
    step = _steps()["n506_faster"]
    assert eev.stt_verdict(step, "робот и ускорь тренд а то мы тут засыпаем оба")["missing"] == "темп"
    assert eev.stt_verdict(step, "робот и ускорь темп а то мы тут засыпаем оба")["stt_ok"] == 1


def test_stt_verdict_executed_command_is_not_replayed_n506_29_10():
    """n506 марафона-6: «темп» -> «там», но «ускорь» уже исполнено (134 -> 145 BPM): второй раз команда упирается в потолок."""
    step = _steps()["n506_faster"]
    heard_text = "робот и ускорь там потом мы тут засыпаем оба"
    assert eev.stt_verdict(step, heard_text) == {"keys": 2, "stt_ok": 0, "missing": "темп"}
    assert eev.stt_verdict(step, heard_text, executed=True) == {"keys": 2, "stt_ok": 1, "missing": "темп", "kept": "executed"}
    # слова на месте -- «исполнено» ничего не меняет: kept только там, где повтор был бы нужен
    assert eev.stt_verdict(step, "робот и ускорь темп", executed=True) == {"keys": 2, "stt_ok": 1, "missing": "-"}


def test_stt_verdict_judges_nothing_without_keys_or_without_a_heard_phrase():
    assert eev.stt_verdict({"label": "x"}, "что угодно")["stt_ok"] == 1
    assert eev.stt_verdict({"label": "x", "stt_keys": ["темп"]}, "")["stt_ok"] == 1


def test_stt_alternatives_are_or_and_all_keys_are_required():
    step = {"label": "x", "stt_keys": ["ускор|быстрее", "темп"]}
    assert eev.stt_verdict(step, "сделай быстрее темп")["stt_ok"] == 1
    assert eev.stt_verdict(step, "сделай быстрее")["missing"] == "темп"


@pytest.mark.parametrize("raw", [[], "темп", [""], [1], ["|"]])
def test_bad_stt_keys_are_an_error_not_a_silent_skip(raw):
    with pytest.raises(ValueError):
        eev.stt_keys({"label": "x", "stt_keys": raw})


def test_replay_without_keys_is_a_schema_error_before_the_first_step():
    scenario = {"steps": [{"label": "x", "text": "Робот, привет", "recover": ["stt_replay"]}]}
    with pytest.raises(ValueError, match="stt_keys"):
        eev.recover_policy(scenario)


def test_the_generator_gives_exactly_n411_and_n506_the_replay_with_all_standard_kinds():
    steps = _steps()
    asking = {label for label, s in steps.items() if "stt_replay" in (s.get("recover") or [])}
    assert asking == {"n411_pitch_bass", "n506_faster"}
    for label in asking:
        assert eev.stt_keys(steps[label]), label
        assert eev.recover_kinds({}, steps[label]) == ("no_wake", "silenced", "stt_replay"), "штатные виды не отняты"
    assert {label for label, s in steps.items() if eev.STT_KEYS_FIELD in s} == asking


def test_stt_replay_composes_with_the_fragile_voice_replay():
    s = gen.step("x", "Робот, ускорь темп", voice=gen.GRISHA, stt_keys=["темп"])
    assert s["recover"] == ["no_wake", "silenced", "wake_replay", "stt_replay"]
    assert eev.recover_kinds({}, s) == ("no_wake", "silenced", "wake_replay", "stt_replay")


def test_marker_accepts_the_kind_for_a_step_but_not_at_act_start():
    assert eev.recover_marker("stt_replay", "n411_pitch_bass", "heard=x") == "E2E_RECOVER stt_replay n411_pitch_bass heard=x"
    with pytest.raises(ValueError):
        eev.recover_marker("stt_replay", eev.AT_ACT_START)


def test_cli_stt_reads_the_keys_from_the_scenario_file_not_from_its_name(tmp_path):
    copy = tmp_path / "e2e_scenario.json"
    copy.write_text((NIGHT / "night_marathon_act4_voice_prosody_v1.json").read_text(encoding="utf-8"), encoding="utf-8")

    def run(heard_text: str):
        return subprocess.run(
            [sys.executable, str(EVENTS), "stt", "--scenario", str(copy), "--label", "n411_pitch_bass", "--heard", heard_text],
            capture_output=True, text=True, encoding="utf-8", check=True,
        ).stdout.splitlines()

    assert run("робот гарри басом") == ["keys=2", "stt_ok=0", "missing=говори"]
    assert run("робот говори басом") == ["keys=2", "stt_ok=1", "missing=-"]


def test_cli_stt_executed_flag_keeps_the_first_execution(tmp_path):
    copy = tmp_path / "e2e_scenario.json"
    copy.write_text((NIGHT / "night_marathon_act5_renardo_live_v1.json").read_text(encoding="utf-8"), encoding="utf-8")
    base = [sys.executable, str(EVENTS), "stt", "--scenario", str(copy), "--label", "n506_faster",
            "--heard", "робот и ускорь там потом"]
    plain = subprocess.run(base, capture_output=True, text=True, encoding="utf-8", check=True).stdout.splitlines()
    kept = subprocess.run([*base, "--executed", "1"], capture_output=True, text=True, encoding="utf-8",
                          check=True).stdout.splitlines()
    assert plain == ["keys=2", "stt_ok=0", "missing=темп"]
    assert kept == ["keys=2", "stt_ok=1", "missing=темп", "kept=executed"]


def test_json_on_disk_carries_the_fields():
    disk = {}
    for path in NIGHT.glob("night_marathon_act*_v1.json"):
        if not path.name.endswith("_acceptance_v1.json"):
            for s in json.loads(path.read_text(encoding="utf-8"))["steps"]:
                disk[s["label"]] = s
    steps = _steps()
    for label in ("n307_count_voices", "n311_who_asked", "n312_stop_music", "n411_pitch_bass", "n506_faster",
                  "n507_wake_under_music", "n511_stop_music", "n605_dj_set_plan", "n709_stop_generated"):
        assert disk[label] == steps[label], label
    assert disk["n411_pitch_bass"]["stt_keys"] == ["говори", "бас"]


# ── n709: ложь о состоянии музыки без тула ──────────────────────────────────────────────────────────────────


def _n709(said: str) -> Dict[str, Any]:
    return _verdict(
        "n709_stop_generated",
        _mcp(T0, "📥 Запрос выполнения: stop_music с параметрами {}"),
        stopped(T0 + 0.5, "gen_ab12_1"),
        _mcp(T0 + 1, "✅ Инструмент stop_music выполнен успешно"),
        _tts(said),
        DONE,
    )


@pytest.mark.parametrize("said", [
    "Раз Борис уснул — сделаю потише. Пусть дослушивает шёпотом.",  # run 40758476879, 20:00:04
    "Хорошо, убавлю громкость.", "Сейчас включу что-нибудь тихое.", "Поставлю колыбельную.", "Сыграю потише.",
])
def test_n709_a_claim_of_another_music_action_after_the_stop_is_red(said):
    got = _n709(said)
    assert not got["pass"] and got["acc"]["forbidden_said"], said


@pytest.mark.parametrize("said", ["Выключил музыку.", "Хорошо, пусть Борис спит."])
def test_n709_the_outcome_phrase_or_a_harmless_remark_is_green(said):
    assert _n709(said)["pass"], said


CODE_CLOSED = ("📊 [turn] turn_id=t1 epoch=1 agent=main stage=done status=done latency_ms=3822 llm_calls=2 "
               "llm_provider=x tokens=10")


def _n709_closed_by_code(said: str) -> Dict[str, Any]:
    return _verdict(
        "n709_stop_generated",
        _mcp(T0, "📥 Запрос выполнения: stop_music с параметрами {}"),
        stopped(T0 + 0.5, "gen_ab12_1"),
        _mcp(T0 + 1, "✅ Инструмент stop_music выполнен успешно"),
        _tts(said),
        CODE_CLOSED,
    )


def test_n709_turn_closed_by_the_code_phrase_after_a_rejected_promise_is_green_29_10():
    """Марафон-6, run 40814536797, 03:58:43: `music_promise` отбит дважды, «Музыку я уже выключил.», `status=done`."""
    got = _n709_closed_by_code("Музыку я уже выключил.")
    assert got["pass"], got


@pytest.mark.parametrize("said", [
    "Данные получил, а ответ у меня не сложился. Спроси ещё раз?",  # заглушка каталога query.fallback_generic
    "Не вышло.",
    "Раз Борис уснул — сделаю потише.",  # ложь о другом действии с музыкой: проверка фразы не ослаблена
    "Хорошо, убавлю громкость.",
])
def test_n709_status_done_does_not_excuse_a_stub_or_a_false_music_claim(said):
    got = _n709_closed_by_code(said)
    assert not got["pass"] and got["acc"]["forbidden_said"], said


def test_n709_is_the_only_step_that_allows_status_done():
    allowing = {label for label, s in _steps().items()
                if gen.TURN_FALLBACK not in s["acceptance"]["must_not_call"]}
    assert allowing == {"n709_stop_generated"}
    assert gen.TURN_CLOSED_BY_CODE in _steps()["n709_stop_generated"]["patterns"]
    assert gen.TURN_DONE not in _steps()["n709_stop_generated"]["patterns"]
    assert etm.patterns_missing(CODE_CLOSED, [gen.TURN_CLOSED_BY_CODE]) == []
    assert etm.patterns_missing(DONE, [gen.TURN_CLOSED_BY_CODE]) == []
    assert etm.patterns_missing(CODE_CLOSED.replace("status=done", "status=failed"), [gen.TURN_CLOSED_BY_CODE])


def test_n709_without_the_stop_it_is_still_red_whatever_is_said():
    got = _verdict("n709_stop_generated", _tts("Выключил музыку."), DONE)
    assert not got["pass"]


def test_every_stop_step_forbids_the_false_action_claims():
    stops = [label for label, s in _steps().items() if s.get("music_compare") == "stopped"]
    assert {"n312_stop_music", "n511_stop_music", "n709_stop_generated"} <= set(stops)
    for label in stops:
        assert gen.MUSIC_FALSE_ACTION in _steps()[label]["acceptance"]["must_not_say"][0], label


# ── bash: run_step с повтором фразы на искажённый вход ──────────────────────────────────────────────────────

STEP = 'run_step "Робот, говори басом, как диктор" anton a cycle; echo "RC=$?"; echo "STEP_BEFORE=${STEP_BEFORE:-}"'
KINDS = "E2E_STEP_RECOVER_KINDS=no_wake,silenced,stt_replay; E2E_STEP_RECOVERED=''; E2E_STEP_STT_REPLAYED=''"


def _scenario(tmp_path: Path) -> str:
    path = tmp_path / "scenario.json"
    path.write_text(json.dumps({"steps": [{"label": "a", "text": "Робот, говори басом", "stt_keys": ["говори", "бас"],
                                           "recover": ["no_wake", "silenced", "stt_replay"]}]}, ensure_ascii=False),
                    encoding="utf-8")
    return 'SCENARIO_FILE="%s"; STEP_BEFORE=initial;' % path.as_posix()


PATTERNS_N506 = """; patterns_json='["✅ Инструмент edit_music выполнен успешно"]'"""


def _names(played):
    return [p.rsplit("/", 1)[-1] for p in played]


@needs_bash
class TestRunStepSttReplay:
    def test_distorted_key_word_plays_the_phrase_once_more_and_moves_the_step_window(self, tmp_path):
        """28.10 n411: «говори» -> «гарри». Ход был, но судить его по такому входу нельзя: фраза ещё раз, окно с повтора."""
        first = heard("робот гарри басом как диктор", "{NOW-30}") + REPLY
        again = heard("робот говори басом как диктор", "{NOW-11}") + REPLY
        out, err, _, played = _run_step(tmp_path, first, [again], STEP, _scenario(tmp_path) + KINDS, replace=True)
        assert "RC=0" in out and "PLAY_COUNT=2" in out, (out, err)
        assert out.count("E2E_RECOVER stt_replay a ") == 1 and "missing=говори" in out, out
        assert "RECOVERED=stt_replay" in out
        assert "STEP_BEFORE=initial" not in out, "окно шага должно начинаться с повтора"
        assert _names(played) == ["cmd_a_eq.wav", "cmd_a_eq.wav"]

    def test_phrase_heard_right_is_not_replayed(self, tmp_path):
        ok = heard("робот говори басом как диктор", "{NOW-30}") + REPLY
        out, err, _, played = _run_step(tmp_path, ok, [], STEP, _scenario(tmp_path) + KINDS)
        assert "RC=0" in out and "PLAY_COUNT=1" in out, (out, err)
        assert "E2E_RECOVER" not in out and "STEP_BEFORE=initial" in out
        assert _names(played) == ["cmd_a_eq.wav"]

    def test_replay_that_is_distorted_again_is_not_repeated_a_third_time(self, tmp_path):
        bad = heard("робот гарри басом", "{NOW-30}") + REPLY
        out, err, _, _ = _run_step(tmp_path, bad, [bad], STEP, _scenario(tmp_path) + KINDS, replace=True)
        assert "PLAY_COUNT=2" in out, (out, err)
        assert out.count("E2E_RECOVER stt_replay a ") == 1

    def test_command_already_executed_is_not_replayed_n506_29_10(self, tmp_path):
        """Марафон-6: «темп» -> «там», но `edit_music` уже исполнен (134 -> 145 BPM): повтор упёрся бы в потолок. Повтора
        нет, окно шага остаётся с первой попытки (паттерны судят первое исполнение)."""
        first = (heard("робот и ускорь там потом мы тут засыпаем оба", "{NOW-30}")
                 + _l("mcp_server", "{NOW-25}", "✅ Инструмент edit_music выполнен успешно") + REPLY)
        out, err, _, played = _run_step(tmp_path, first, [], STEP, _scenario(tmp_path) + KINDS + PATTERNS_N506)
        assert "RC=0" in out and "PLAY_COUNT=1" in out, (out, err)
        assert "E2E_RECOVER" not in out and "RECOVERED=stt_replay" not in out, out
        assert "STEP_BEFORE=initial" in out, "окно шага не сдвигается: судим первое исполнение"
        assert "команда исполнена, повтора нет" in out, out
        assert _names(played) == ["cmd_a_eq.wav"]

    def test_command_not_executed_is_still_replayed_when_the_step_has_patterns(self, tmp_path):
        """Исхода шага в окне нет (n411: «гарри» -- pitch_shift не менялся): повтор, как раньше."""
        first = heard("робот гарри басом как диктор", "{NOW-30}") + REPLY
        again = heard("робот говори басом как диктор", "{NOW-11}") + REPLY
        out, err, _, played = _run_step(tmp_path, first, [again], STEP, _scenario(tmp_path) + KINDS + PATTERNS_N506,
                                        replace=True)
        assert "PLAY_COUNT=2" in out and out.count("E2E_RECOVER stt_replay a ") == 1, (out, err)
        assert "повтора нет" not in out

    def test_step_without_the_kind_is_never_replayed(self, tmp_path):
        bad = heard("робот гарри басом", "{NOW-30}") + REPLY
        out, err, _, _ = _run_step(
            tmp_path, bad, [], STEP,
            _scenario(tmp_path) + "E2E_STEP_RECOVER_KINDS=no_wake,silenced; E2E_STEP_RECOVERED=''; E2E_STEP_STT_REPLAYED=''")
        assert "PLAY_COUNT=1" in out and "E2E_RECOVER" not in out, (out, err)


# ── М5 (29.10): n512 -- каскад от n511 и вопрос словами; n306 -- «не опознал» ──────────────────────────────


@pytest.mark.parametrize("lines, why", [
    ((started(T0 + 10),), "n511 не снял музыку: трек всё ещё играет"),
    ((started(T0 + 10), edited(T0 + 20)), "правка играющего трек не снимает"),
])
def test_n512_is_skipped_while_music_still_plays(lines, why):
    got = _skip("n512_silence_after_stop", *lines)
    assert got["skip"] == "NO_STOP_N511" and got["evidence"] != "-", (why, got)


@pytest.mark.parametrize("lines, why", [
    ((), "музыка не заигрывала"),
    ((started(T0 + 10), stopped(T0 + 50)), "n511 стоп исполнил"),
    ((started(T0 + 10), finished(T0 + 50)), "трек доиграл сам"),
])
def test_n512_is_played_when_the_deck_is_empty(lines, why):
    assert _skip("n512_silence_after_stop", *lines) == {"fetch": 1, "skip": "-", "evidence": "-"}, why


def test_n512_asks_in_words_and_does_not_accept_the_command():
    step = _steps()["n512_silence_after_stop"]
    assert "играет" in step["text"].lower() and "?" in step["text"]
    assert step["acceptance"]["expected_keywords"] == [gen.SILENCE_STATE_ANSWER]
    assert gen.SILENCE_COMMAND_ACK in step["acceptance"]["must_not_say"]
    assert not _verdict("n512_silence_after_stop", _tts("Выключил музыку. Молчу, Саша."), DONE)["pass"]
    assert _verdict("n512_silence_after_stop", _tts("Нет, Саша, сейчас ничего не играет."), DONE)["pass"]


@pytest.mark.parametrize("said, green", [
    ("На фоне говорили трое. Саша - его я узнал, Борис - тоже узнал. А третий голос я не опознал.", True),
    ("Борис - узнал. Один голос остался неопознан.", True),
    ("Борис - знакомый. Третья не опознала голос.", True),
    ("Борис - узнал, и всех остальных я тоже опознал.", False),
])
def test_n306_not_identified_forms_are_green_and_claims_to_know_everyone_are_red(said, green):
    assert _verdict("n306_who_was_talking", _tts(said), DONE)["pass"] is green, said
