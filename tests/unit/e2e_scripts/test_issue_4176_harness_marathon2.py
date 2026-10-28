# -*- coding: utf-8 -*-
"""#4176 (эпик #3683): харнесс марафона-2 28.10 -- несправедливые красные и слабые места.

Сверка по разбору ``docker logs voice-assistant`` (образ dev-00f61b1):
  * n908 (17:13:27) -- факт Саши пришёл из снимка ``<speaker_memory>``, ``memory_search`` не вызван: механизм «учёл память»
    сработал, а харнесс требовал вызов тула;
  * n1010 (17:23:15) -- «всего два факта», назван один: решение Шифу 28.10 (#4186) -- главное, что ответ не заглушка
    «подготовил, но сказать не могу»; одного факта Саши в речи достаточно;
  * n312 (16:05:12) -- кавер на 92 с кончился сам (16:05:11), «выключай» получило «Сейчас ничего не играет…»;
  * n707 (16:52:15) -- «этот трек» стоял до n708 (play), песня не играет, фраза неоднозначна.

Судят те же функции, что у живого харнесса: ``e2e_tool_match.step_verdict`` (acceptance), ``patterns_missing`` (patterns),
``e2e_events.skip_verdict`` (предусловие).

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_4176_harness_marathon2.py -v -o addopts=""
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List

import pytest

ROOT = Path(__file__).resolve().parents[3]
for _dir in (ROOT / "scripts" / "e2e", ROOT / ".github" / "workflows" / "scripts"):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import e2e_events as eev  # noqa: E402
import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

NIGHT = ROOT / ".github" / "e2e" / "scenarios" / "night"
EVENTS = ROOT / ".github" / "workflows" / "scripts" / "e2e_events.py"

DONE = (
    "📊 [turn] turn_id=t1 epoch=1 agent=main stage=done status=finished latency_ms=900 llm_calls=2 llm_provider=x "
    "tokens=10"
)
MEMORY_DONE = "[mcp_server]: ✅ Инструмент memory_search выполнен успешно"
#: Ответ n908 28.10 17:13:27 (факт из снимка, тула нет) + рецепт.
BORSCH_FROM_SNAPSHOT = (
    "Нашёл, Саша. Борщ сделаем без лука — ты его не ешь даже жареный. Свёкла, капуста, картошка, говядина на бульон."
)
BORSCH_NO_MEMORY = "Нашёл рецепт. Свёкла, капуста, картошка, морковь, лук и говядина на бульон. Варить два часа."
#: Ответ n1010 28.10 17:23:15.
TWO_FACTS_ONE_NAMED = "В памяти у меня про тебя всего два факта. Первый — пьёшь только зелёный чай и без сахара."


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


# ── п.1: n908 -- память из снимка ────────────────────────────────────────────────────────────────────────────


def test_n908_memory_from_snapshot_without_the_tool_is_green():
    """28.10 17:13:27: тула нет, факт в ответе -- механизм сработал."""
    got = _verdict("n908_search_borsch_memory", _tts(BORSCH_FROM_SNAPSHOT), DONE)
    assert got["pass"], got


def test_n908_memory_search_without_the_fact_in_speech_is_green():
    got = _verdict("n908_search_borsch_memory", MEMORY_DONE, _tts(BORSCH_NO_MEMORY), DONE)
    assert got["pass"], got


def test_n908_recipe_without_any_trace_of_memory_is_red():
    """Ни тула, ни оборота факта: «лук» -- ингредиент борща, а не память."""
    got = _verdict("n908_search_borsch_memory", _tts(BORSCH_NO_MEMORY), DONE)
    assert not got["pass"] and got["missing"] == [gen.MEMORY_USED], got


def test_n908_memory_trace_without_a_recipe_is_red():
    said = "Помню, ты пьёшь зелёный чай без сахара, Саша."
    got = _verdict("n908_search_borsch_memory", MEMORY_DONE, _tts(said), DONE)
    assert not got["pass"] and got["acc"]["missing_keywords"] == [gen.RECIPE_ANSWER], got


def test_n908_catalog_stub_is_red_even_with_the_tool():
    stub = gen.ROBOT_PHRASES["query.fallback_generic"][0]
    got = _verdict("n908_search_borsch_memory", MEMORY_DONE, _tts(stub), DONE)
    assert not got["pass"] and got["acc"]["forbidden_said"]


def test_n908_has_no_hard_tool_requirement_but_keeps_the_recipe_and_the_retry():
    step = _steps()["n908_search_borsch_memory"]
    assert step["acceptance"]["expected_tool_calls"] == []
    assert step["patterns"] == [gen.MEMORY_USED]
    assert step["acceptance"]["expected_keywords"] == [gen.RECIPE_ANSWER]
    assert step["retry_acceptance"] == 1


@pytest.mark.parametrize("said", [
    "Ты не ешь лук, Саша.", "Не ем лук, поэтому без него.", "Борщ без лука, как ты любишь.",
    "Пьёшь зелёный чай, значит к борщу чай.", "Зеленый чай без сахара, помню.", "Не любишь лук, помню.",
])
def test_memory_fact_spoken_catches_the_turns_of_the_fact(said):
    assert not etm.patterns_missing(_tts(said), [gen.MEMORY_USED]), said


@pytest.mark.parametrize("said", [
    "Положи лук, свёклу и зелень.", "Нужна зелёная фасоль.", "Варим борщ два часа.",
])
def test_memory_fact_spoken_is_not_fooled_by_recipe_words(said):
    assert etm.patterns_missing(_tts(said), [gen.MEMORY_USED]), said


def test_memory_fact_does_not_match_a_non_tts_line():
    """Дамп запроса LLM со снимком памяти («не ест лук») -- не речь робота."""
    dump = "[dialogue_node-1] LLM REQUEST: <speaker_memory>Пьёт только зелёный чай без сахара; не ест лук</speaker_memory>"
    assert etm.patterns_missing(dump, [gen.MEMORY_USED])


# ── п.2: n1010 -- ответ не заглушка + хотя бы один факт ──────────────────────────────────────────────────────


def test_n1010_one_fact_named_is_green_when_there_is_no_stub():
    """Решение Шифу 28.10: «два факта» + один названный (17:23:15) -- факт из памяти есть, заглушки нет."""
    got = _verdict("n1010_five_facts", _tts(TWO_FACTS_ONE_NAMED), DONE)
    assert got["pass"], got


@pytest.mark.parametrize("said", ["Ты не ешь лук, Саша.", "Ты пьёшь зеленый чай.", "Не ешь лук и пьёшь зелёный чай."])
def test_n1010_any_single_fact_is_enough(said):
    assert _verdict("n1010_five_facts", _tts(said), DONE)["pass"], said


@pytest.mark.parametrize("said", [
    "Я многое о тебе помню, Саша.", "Ты Саша, инженер и механик.",
    "Подготовил ответ, но сказать не могу.",
])
def test_n1010_no_fact_is_red(said):
    got = _verdict("n1010_five_facts", _tts(said), DONE)
    assert not got["pass"] and got["acc"]["missing_keywords"] == [gen.SASHA_ANY_FACT], said


def test_n1010_catalog_stub_is_red_even_with_a_fact():
    stub = gen.ROBOT_PHRASES["query.fallback_generic"][0]
    got = _verdict("n1010_five_facts", _tts(stub + " Ты не ешь лук."), DONE)
    assert not got["pass"] and got["acc"]["forbidden_said"]


def test_n1010_does_not_require_the_tool_and_takes_one_fact():
    step = _steps()["n1010_five_facts"]
    assert step["patterns"] == []
    assert step["acceptance"]["expected_tool_calls"] == []
    assert step["acceptance"]["expected_keywords"] == [gen.SASHA_ANY_FACT]
    assert gen.SASHA_ANY_FACT == "|".join(gen.SASHA_FACTS)
    assert "#4176" in step["acceptance"]["_comment"]


# ── п.3: n312 -- гонка с концом трека ────────────────────────────────────────────────────────────────────────


def _mcp(ts: float, msg: str) -> str:
    return "[mcp_server-9] [INFO] [%.3f] [mcp_server]: %s" % (ts, msg)


def started(ts: float, track: str = "gen_ab12_1") -> str:
    return _mcp(ts, "🎵 [music v2] started track_id=%s clock_beat=9792.069 start_beat=9792.0 phase_in_form=0.0 "
                    "late_beats=0.069 bpm=90 deck=A form_beats=192.0 players_aligned=True latency_s=0.5" % track)


def finished(ts: float, track: str = "gen_ab12_1") -> str:
    return _mcp(ts, "🎵 [music v2] finished track_id=%s" % track)


def stopped(ts: float, track: str = "gen_ab12_1", reason: str = "user_stop") -> str:
    return _mcp(ts, "🎵 [music v2] stop track_id=%s reason=%s" % (track, reason))


def edited(ts: float, track: str = "gen_ab12_1") -> str:
    return _mcp(ts, "🎚️ [music v2] edit applied track_id=%s at_beat=9856 bpm=100" % track)


T0 = 1792793000.0


def _skip(*lines: str, label: str = "n312_stop_music") -> Dict[str, object]:
    rule = eev.skip_rule(_steps()[label])
    assert rule is not None
    return eev.skip_verdict(rule, "%.3f\n%s" % (T0 + 500, "\n".join(lines)))


def test_n312_is_skipped_when_the_track_played_out_by_itself():
    got = _skip(started(T0 + 10), finished(T0 + 102))
    assert got["fetch"] == 1 and got["skip"] == "TRACK_ENDED" and "finished track_id=gen_ab12_1" in got["evidence"], got


def test_n312_is_skipped_when_an_earlier_track_was_stopped_and_the_last_one_ended():
    got = _skip(started(T0), stopped(T0 + 5), started(T0 + 20, "gen_2"), edited(T0 + 30, "gen_2"), finished(T0 + 100, "gen_2"))
    assert got["skip"] == "TRACK_ENDED", got


@pytest.mark.parametrize("lines, why", [
    ((started(T0 + 10),), "трек играет -- стоп проверяется"),
    ((started(T0 + 10), edited(T0 + 20)), "правка играющего трек не снимает"),
    ((started(T0 + 10), finished(T0 + 50), started(T0 + 60, "gen_2")), "после конца заиграл новый -- играет"),
    ((started(T0 + 10), stopped(T0 + 50)), "дека снята чужим стопом -- красный честный, не «доиграл сам»"),
    ((started(T0 + 10), finished(T0 + 50), stopped(T0 + 51, "None", "user_stop")), "последнее событие -- стоп"),
    ((), "музыка не заигрывала вовсе -- n310 красный сам, n312 не прячем"),
    ((finished(T0 + 5, "gen_old"), started(T0 + 10)), "конец ЧУЖОГО трека до старта текущего"),
])
def test_n312_is_played_when_the_deck_did_not_end_by_itself(lines, why):
    got = _skip(*lines)
    assert got == {"fetch": 1, "skip": "-", "evidence": "-"}, why


def test_n312_unreadable_log_does_not_skip():
    rule = eev.skip_rule(_steps()["n312_stop_music"])
    assert eev.skip_verdict(rule, "Traceback\n" + finished(T0)) == {"fetch": 0, "skip": "-", "evidence": "-"}


def test_n312_field_comes_from_one_table_and_the_judge_parses_it():
    step = _steps()["n312_stop_music"]
    assert step["skip_when"] == {"reason": "TRACK_ENDED", "log": gen.MUSIC_FINISHED, "deck": "ended"}
    assert gen.SKIP_WHEN["TRACK_ENDED"].deck == eev.DECK_ENDED
    assert eev.skip_rule(step).deck == "ended"
    # n312 по-прежнему action_step «выключил»: result=stopped, music_compare, стоп человека в логе шага
    assert step["music_compare"] == "stopped"
    assert gen.tool_done("stop_music") in step["patterns"]
    assert "Сейчас ничего не играет" in step["acceptance"]["must_not_say"][0]


@pytest.mark.parametrize("raw", [
    {"reason": "TRACK_ENDED", "log": "x", "deck": "playing"},
    {"reason": "TRACK_ENDED", "log": "x", "deck": 1},
])
def test_unknown_deck_rule_is_an_error_not_a_silent_play(raw):
    with pytest.raises(ValueError):
        eev.skip_rule({"label": "n312", "skip_when": raw})


def test_cli_skip_reads_the_rule_from_the_scenario_file_not_from_its_name(tmp_path):
    """Сценарий доставляется как ``/tmp/e2e_scenario.json``: правило берётся из поля шага, а не из имени файла."""
    scenario = json.loads((NIGHT / "night_marathon_act3_backlog_diarization_v1.json").read_text(encoding="utf-8"))
    copy = tmp_path / "e2e_scenario.json"
    copy.write_text(json.dumps(scenario, ensure_ascii=False), encoding="utf-8")

    def run(*lines: str) -> List[str]:
        return subprocess.run(
            [sys.executable, str(EVENTS), "skip", "--scenario", str(copy), "--label", "n312_stop_music"],
            input="%.3f\n%s" % (T0 + 500, "\n".join(lines)), capture_output=True, text=True, encoding="utf-8", check=True,
        ).stdout.splitlines()

    assert "skip=TRACK_ENDED" in run(started(T0), finished(T0 + 92))
    assert "skip=-" in run(started(T0))


# ── п.3б: n309 -- музыка перед n312 длится ~2 минуты и не тратит KIE ─────────────────────────────────────────


def _voice_imports():
    try:
        from rob_box_music.source import SYNTH, engine_route, source_ask
        from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
        from rob_box_voice.core.set_length_words import heard_set_length, heard_set_seconds
    except ImportError:
        pytest.skip("rob_box_voice/rob_box_music вне PYTHONPATH (в CI есть)")
    return SYNTH, engine_route, source_ask, MediaIntent, parse_media_command, heard_set_length, heard_set_seconds


def test_n309_is_a_free_two_minute_synth_set_read_by_the_code():
    """Деньги: «на синтезаторе» называет движок -> единственный кандидат синтез, генерации KIE нет. Длину решает код."""
    synth, engine_route, source_ask, intent, parse, set_length, set_seconds = _voice_imports()
    text = _steps()["n309_bg_command_valera"]["text"]
    assert "диджей-сет" in text and "две минуты" in text
    assert engine_route(source_ask(text).engine) == synth, "без названного движка сет идёт кэш -> платная генерация -> синтез"
    assert set_seconds(text) == 120.0 and set_length(text) == 2
    assert parse(text).intent is intent.DJ


def test_n309_phrase_has_no_contradicting_negation_and_the_step_documents_the_money_path():
    step = _steps()["n309_bg_command_valera"]
    assert "электрон" not in step["text"].lower()
    why = step["_why"] if "_why" in step else step["acceptance"]["_comment"]
    assert "ДЕНЬГИ" in why and "_named_cell" in why and "#4176" in why


def test_n310_still_accepts_dj_set_as_the_music_start():
    assert "dj_set" in gen.MUSIC_START_TOOLS


# ── п.3в: n312 -- стоп сета ──────────────────────────────────────────────────────────────────────────────────


def test_router_stop_calls_dj_set_stop_then_stop_music():
    """Стоп «выключай музыку» при играющем сете: роутер зовёт ОБА тула, гейт акта 3 (``stop_music``) честен."""
    try:
        from rob_box_voice.core.media_command_grammar import MediaIntent, MediaCommand
        from rob_box_voice.core.media_router import MediaState, plan_media_command
    except ImportError:
        pytest.skip("rob_box_voice вне PYTHONPATH (в CI есть)")
    plan = plan_media_command(MediaCommand(intent=MediaIntent.STOP), MediaState(dj_enabled=True))
    assert [c.name for c in plan.tool_calls] == ["dj_set", "stop_music"]
    assert plan.tool_calls[0].arguments == {"action": "stop"}
    assert plan.say_ok == gen.robot_said("music.stop_ok")


def _stop_log(said: str, track: str = "set01234:2") -> List[str]:
    """Окно шага n312 при стопе идущего сета: ``dj_set(stop)`` снимает деку с id трека, затем идемпотентный ``stop_music``."""
    return [
        _mcp(T0 + 60, "📥 Запрос выполнения: dj_set с параметрами {'action': 'stop'}"),
        stopped(T0 + 60.5, track),
        _mcp(T0 + 61, "📥 Запрос выполнения: stop_music с параметрами {}"),
        _mcp(T0 + 61.1, "🎵 [music v2] stop track_id=None reason=user_stop"),
        _mcp(T0 + 61.2, "[stop_music] was_playing=False"),
        _mcp(T0 + 61.3, "✅ Инструмент stop_music выполнен успешно"),
        _tts(said),
        DONE,
    ]


def test_n312_set_stop_by_the_router_is_green_with_the_stop_ok_phrase():
    got = _verdict("n312_stop_music", *_stop_log("Выключил музыку."))
    assert got["pass"], got


@pytest.mark.parametrize("said", [
    "Сейчас ничего не играет — на всякий случай всё остановил.",  # дека была пуста -- гонка с концом (n312 28.10)
    "Включаю музыку.",  # media.done после стопа (#4075)
])
def test_n312_forbidden_phrases_stay_red_for_a_set_stop(said):
    got = _verdict("n312_stop_music", *_stop_log(said))
    assert not got["pass"] and got["acc"]["forbidden_said"], said


def test_n312_stop_without_a_real_track_on_the_deck_is_red():
    """Только стоп вхолостую (``track_id=None``) -- стопа с треком нет, шаг не засчитывается."""
    log = [line for line in _stop_log("Выключил музыку.") if "set01234" not in line]
    assert not _verdict("n312_stop_music", *log)["pass"]
# ── п.4: n707 -- песня названа во фразе ──────────────────────────────────────────────────────────────────────


def test_n707_names_the_song_and_keeps_the_order_before_play():
    act7 = next(a for a in gen.ACTS if a["n"] == 7)["steps"]
    labels = [s["label"] for s in act7]
    assert labels.index("n707_gen_track_info") < labels.index("n708_gen_play") < labels.index("n709_stop_generated")
    n707 = act7[labels.index("n707_gen_track_info")]
    low = n707["text"].lower()
    assert "енот" in low, "песня названа во фразе (предмет n702)"
    assert "это за трек" not in low and "этот трек" not in low, "неоднозначное «этот трек» до play"
    assert "енот" in act7[labels.index("n702_generate_raccoon")]["text"].lower()
    assert n707["acceptance"]["expected_tool_calls"] == ["music_library"]
    assert n707["skip_when"]["reason"] == "NO_SONG" and n707["sleep_before_sec"] == 60
    assert "#4176" in n707["acceptance"]["_comment"]


# ── JSON на диске и таблица ──────────────────────────────────────────────────────────────────────────────────


def test_json_on_disk_matches_the_generator_for_every_step_this_card_touches():
    on_disk = {}
    for path in NIGHT.glob("night_marathon_act*_v1.json"):
        if path.name.endswith("_acceptance_v1.json"):
            continue
        for s in json.loads(path.read_text(encoding="utf-8"))["steps"]:
            on_disk[s["label"]] = s
    steps = _steps()
    for label in ("n312_stop_music", "n707_gen_track_info", "n908_search_borsch_memory", "n1010_five_facts"):
        assert on_disk[label] == steps[label], label


def test_skip_when_table_has_exactly_the_known_reasons():
    assert set(gen.SKIP_WHEN) == {"NO_SONG", "NOT_SILENCED", "TRACK_ENDED"}
