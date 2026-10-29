# -*- coding: utf-8 -*-
"""#4077 (аудит сценария марафона 27.10), очередь 1: длина реплик до прогона (П2), песня и зависимые шаги (П1), пустые
шаги (П3), акт 5, n210.

Шаги судит тот же код, что у живого харнесса и сухого стенда: ``e2e_tool_match.step_verdict`` (acceptance) и
``patterns_missing`` (patterns). Строки лога -- формат строк робота; их источники (f-строки кода) держит
``test_gen_night_marathon_results.py::test_marker_sources_exist_in_robot_code``. Raw 27.10 -- из аудита
``AUDIT_marathon_scenario_2026-10-27.md`` (прогон ``night_20261027_082157``).

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_4077_marathon_scenario_queue1.py -v --no-cov
"""

from __future__ import annotations

import json
import os
import re
import shutil
import stat
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict

import pytest

from tests.unit.e2e_scripts.test_issue_2890_act_isolation import (
    BASH,
    _bash_path,
    _require_bash_sees,
    bash_required,
)

ROOT = Path(__file__).resolve().parents[3]
NIGHT = ROOT / ".github" / "e2e" / "scenarios" / "night"
for _dir in (ROOT / "scripts" / "e2e", ROOT / ".github" / "workflows" / "scripts"):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402
import phrase_budget as pb  # noqa: E402

DONE = "📊 [turn] turn_id=t1 epoch=1 agent=main stage=done status=finished latency_ms=900 llm_calls=1 llm_provider=x tokens=10"
MUSIC_OK = "✅ Инструмент request_music выполнен успешно"
STARTED = "🎵 [music v2] started track_id=req95964:01:A:f7de93b5 clock_beat=4"


def _steps() -> Dict[str, Dict[str, Any]]:
    out = {s["label"]: s for a in gen.ACTS for s in a["steps"]}
    out.update({s["label"]: s for s in gen._manual_steps()})
    return out


def _verdict(label: str, log: str) -> Dict[str, Any]:
    step = _steps()[label]
    missing = etm.patterns_missing(log, step["patterns"])
    acc = etm.step_verdict(step.get("acceptance", {}), log, label=label)
    return {"missing": missing, "acc": acc, "pass": not missing and acc["pass"]}


def _tts(text: str) -> str:
    return (
        "[tts_node-4] [INFO] [1792795079.69] [tts_node]: 🔊 TTS: speech_id=764009bb, dialogue_id=None, "
        "batch=None None/None, voice=default, lang=default, text='%s'" % text
    )


def _source(route: str, flags: str = "") -> str:
    return (
        "[mcp_server-9] [INFO] [1792795079.69] [mcp_server]: 🎚️ [source] asked=[%s] genre=ambient engine=renardo "
        "кандидаты=['%s'] → %s (code) gen=ok" % (flags, route, route)
    )


# ── П2: бюджет реплики в окне VAD ─────────────────────────────────────────────────────────────────────────────

#: Кусок лога акта 6 прогона 27.10 (run 40481144618), вид строк артефакта: префикс с табами, три повтора лога.
LOG_N607 = "\n".join(
    [
        "e2e-voice\tE2E voice test (atomic harness)\t2026-10-27T09:36:55.1000000Z >>> === STEP n607_vibe_chill "
        '(safe=n607_vibe_chill): voice=anton text="Робот, а теперь помягче, поставь спокойную атмосферу, Валера уснёт '
        'за рулём." ===',
        "e2e-voice\tE2E voice test (atomic harness)\t2026-10-27T09:36:59.2609971Z >>> STEP n607_vibe_chill: ⚠️ реплика "
        "12.378563s при бюджете 12.50s — запас < 1s, шаг на грани отказа",
        "e2e-voice\tE2E voice test (atomic harness)\t2026-10-27T09:36:59.9880092Z >>> STEP n607_vibe_chill: PLAY attempt 1/3",
        "e2e-voice\tE2E voice test (atomic harness)\t2026-10-27T09:37:12.3782540Z >>>   PLAY_DONE",
        # повтор того же вывода в конце файла (артефакт печатает лог трижды): пара нулевой длины
        "e2e-voice\tE2E voice test (atomic harness)\t2026-10-27T09:39:12.4748193Z >>> STEP n607_vibe_chill: PLAY attempt 1/3",
        "e2e-voice\tE2E voice test (atomic harness)\t2026-10-27T09:39:12.4748475Z >>>   PLAY_DONE",
        "e2e-voice\tE2E voice test (atomic harness)\t2026-10-27T09:40:00.0000000Z >>> === STEP n302_bg_sasha "
        '(safe=n302_bg_sasha): voice=anton text="Борис, блок питания гудит из-за земли, я вчера полночи его слушал и '
        'знаю эту ноту наизусть." ===',
        "e2e-voice\tE2E voice test (atomic harness)\t2026-10-27T09:40:01.0000000Z >>> STEP n302_bg_sasha: ❌ реплика "
        "13.783375s не влезает в окно VAD робота (бюджет 12.50s)",
        "e2e-voice\tE2E voice test (atomic harness)\t2026-10-27T09:41:00.0000000Z >>> === STEP n303_bg_boris "
        '(safe=n303_bg_boris): voice=ermil text="Саша, у тебя разводка." ===',
        "e2e-voice\tE2E voice test (atomic harness)\t2026-10-27T09:41:01.0000000Z >>> STEP n303_bg_boris: PLAY backlog attempt 1/3",
        "e2e-voice\tE2E voice test (atomic harness)\t2026-10-27T09:41:08.4500000Z >>>   PLAY_DONE",
    ]
)


def test_run_log_gives_exact_harness_value_or_play_interval_and_ignores_log_copies():
    got = {m.label: m for m in pb.parse_run_log(LOG_N607)}
    assert (
        got["n607_vibe_chill"].eq_s == pytest.approx(12.378563)
        and got["n607_vibe_chill"].how == "harness"
    )
    assert got["n607_vibe_chill"].voice == "anton" and got[
        "n607_vibe_chill"
    ].text.startswith("Робот, а теперь помягче")
    assert got["n302_bg_sasha"].eq_s == pytest.approx(
        13.783375
    ), "отказ `❌ реплика … не влезает` тоже замер"
    assert (
        got["n303_bg_boris"].eq_s == pytest.approx(7.45)
        and got["n303_bg_boris"].how == "play"
    )
    assert got["n303_bg_boris"].voice == "ermil"


def test_measure_is_valid_only_for_the_same_text_and_voice():
    text = "Робот, поставь сет из трёх треков подряд и объявляй каждый перед началом."
    store = {"nX": {"voice": "anton", "text_sha1": pb.text_key(text), "eq_s": 8.65}}
    found = pb.verdict("nX", "anton", text, store)
    assert (found.source, found.eq_s, found.level) == (pb.MEASURED, 8.65, pb.OK)
    edited = pb.verdict("nX", "anton", text + " Быстро.", store)
    assert (
        edited.source == pb.ESTIMATE
    ), "правка текста обнуляет замер: оценка по символам"
    other_voice = pb.verdict("nX", "ermil", text, store)
    assert other_voice.source == pb.ESTIMATE


@pytest.mark.parametrize(
    "eq_s, level",
    [
        (9.0, pb.OK),
        (11.5, pb.OK),
        (11.57, pb.TIGHT),
        (12.0, pb.TIGHT),
        (12.03, pb.TOO_LONG),
        (12.38, pb.TOO_LONG),
        (13.78, pb.TOO_LONG),
    ],
)
def test_measured_levels_follow_the_budget(eq_s, level):
    """27.10: n201b 11.57 с и n412 11.60 с -- тугие (предупреждение), 2b n701 12.03 с, n607 12.38 с, n302 13.78 с -- красные."""
    store = {"nX": {"voice": "anton", "text_sha1": pb.text_key("x"), "eq_s": eq_s}}
    assert pb.verdict("nX", "anton", "x", store).level == level


def test_unmeasured_phrase_is_judged_by_the_worst_pace_of_its_voice():
    limit = pb.char_limit("anton")
    assert pb.verdict("nY", "anton", "а" * limit, {}).level == pb.OK
    assert pb.verdict("nY", "anton", "а" * (limit + 1), {}).level == pb.TOO_LONG
    assert pb.char_limit("ermil") > pb.char_limit(
        "anton"
    ), "Борис (ermil) читает быстрее Саши (anton)"
    assert pb.char_limit("неизвестный") == pb.char_limit(
        "anton"
    ), "голос без замеров -- по самому медленному"


def test_estimate_does_not_underrate_any_played_phrase_of_the_27_10_run():
    """Оценка не оптимистичнее замера ни для одной сыгранной фразы >= 55 символов (аудит брал 0.145 -- n607 0.163 не ловил)."""
    store = pb.load_store(gen.PHRASE_STORE_PATH)
    long_ones = [(k, v) for k, v in store.items() if v["chars"] >= 55]
    # #3971: замер n809 снят -- реплика сменилась («на синтезаторе»), замер был чужой
    # Шифу 28.10: замер n210 снят -- фразу обернули вейком («Робот, … , робот.»), текст другой; новый даст --learn
    # #4176: замер n707 снят -- песня названа во фразе («про енота»), текст другой; новый даст --learn
    # #4176: замер n309 снят -- команда просит диджей-сет «на две минуты» (длинная музыка перед n312), текст другой
    assert len(long_ones) >= 57
    for label, entry in long_ones:
        assert entry["eq_s"] <= entry["chars"] * pb.sec_per_char(entry["voice"]), label


def test_shipped_measures_belong_to_the_current_texts_and_none_is_over_the_ceiling():
    """Словарь замеров не содержит чужих (устаревших) записей, а красных реплик в сценарии нет."""
    store = pb.load_store(gen.PHRASE_STORE_PATH)
    current = gen.phrases()
    stale = [
        k
        for k, v in store.items()
        if k not in current
        or v["text_sha1"] != pb.text_key(current[k][1])
        or v["voice"] != current[k][0]
    ]
    assert not stale, stale
    assert all(v["eq_s"] <= pb.EQ_ERROR_S for v in store.values())
    red, tight = gen.phrase_report()
    assert not red, [v.describe() for v in red]
    assert all(
        v.source == pb.MEASURED for v in tight
    ), "тугой может быть только замер: оценка выше порога -- красная"


def test_edited_phrases_of_the_audit_are_short_and_no_longer_over_the_window():
    """П2: n302, n602, n605, n607, n711 (+ n702 и 2b n701) -- были 74-91 символ, теперь влезают без замера."""
    phrases = gen.phrases()
    for label in (
        "n302_bg_sasha",
        "n602_dj_on",
        "n605_dj_set_plan",
        "n607_vibe_chill",
        "n711_renardo_not_broken",
        "n702_generate_raccoon",
        "n701_sasha_back_recognized",
        "n1001_recall_night",
    ):
        voice, text = phrases[label]
        assert len(text) <= pb.char_limit(voice), (label, len(text))
        assert pb.verdict(label, voice, text, gen.PHRASE_STORE).level == pb.OK, label
    assert len(phrases["n702_generate_raccoon"][1]) <= 60


def test_step_refuses_a_phrase_that_would_not_fit_the_vad_window():
    """Реплика, не влезающая в окно VAD, -- ошибка сборки, а не `FAIL scenario_too_long` посреди акта (27.10)."""
    old_n302 = "Борис, блок питания гудит из-за земли, я вчера полночи его слушал и знаю эту ноту наизусть."
    with pytest.raises(ValueError, match="окно VAD"):
        gen.step("nX_long", old_n302)
    original = dict(gen.PHRASE_STORE)
    gen.PHRASE_STORE["nX_long"] = {
        "voice": "anton",
        "text_sha1": pb.text_key(old_n302),
        "eq_s": 9.0,
    }
    try:
        assert (
            gen.step("nX_long", old_n302)["text"] == old_n302
        ), "замер в пределах потолка снимает оценку по символам"
    finally:
        gen.PHRASE_STORE.clear()
        gen.PHRASE_STORE.update(original)


def test_check_fails_on_an_over_budget_phrase_of_a_hand_written_act(
    monkeypatch, capsys
):
    """``--check`` видит и рукописные акты 2b/2c: красная реплика -- код возврата 1 и её имя в выводе."""
    long_text = "Робот, " + "очень длинная реплика " * 5
    phrases = dict(gen.phrases())
    phrases["n701_sasha_back_recognized"] = ("anton", long_text)
    monkeypatch.setattr(gen, "phrases", lambda: phrases)
    assert gen.check() == 1
    out = capsys.readouterr().out
    assert "n701_sasha_back_recognized" in out and "окно VAD" in out


def test_learn_keeps_only_measures_of_the_current_text(tmp_path, monkeypatch):
    voice, text = gen.phrases()["n201b_sasha_backstory"]
    log = "\n".join(
        [
            "e2e-voice\tx\t2026-10-27T09:00:00.0000000Z >>> === STEP n201b_sasha_backstory (safe=n201b_sasha_backstory): "
            'voice=%s text="%s" ===' % (voice, text),
            "e2e-voice\tx\t2026-10-27T09:00:01.0000000Z >>> STEP n201b_sasha_backstory: ⚠️ реплика 11.565875s при бюджете 12.50s",
            "e2e-voice\tx\t2026-10-27T09:01:00.0000000Z >>> === STEP n302_bg_sasha (safe=n302_bg_sasha): voice=anton "
            'text="СТАРЫЙ ТЕКСТ" ===',
            "e2e-voice\tx\t2026-10-27T09:01:01.0000000Z >>> STEP n302_bg_sasha: ❌ реплика 13.783375s не влезает",
        ]
    )
    path = tmp_path / "act3_999.log"
    path.write_text(log, encoding="utf-8")
    store_path = tmp_path / "store.json"
    monkeypatch.setattr(gen, "PHRASE_STORE_PATH", str(store_path))
    monkeypatch.setattr(gen, "PHRASE_STORE", {})
    assert gen.learn([str(path)]) == 0
    saved = json.loads(store_path.read_text(encoding="utf-8"))["steps"]
    assert set(saved) == {
        "n201b_sasha_backstory"
    }, "n302: текст в логе не равен текущему -- замера нет"
    assert (
        saved["n201b_sasha_backstory"]["eq_s"] == 11.57
        and saved["n201b_sasha_backstory"]["run"] == "999"
    )


# ── П1: песня этого прогона и зависимые шаги ───────────────────────────────────────────────────────────────────


def test_act7_chain_is_rebuilt_under_the_fork_and_has_no_destructive_step():
    act7 = next(a for a in gen.ACTS if a["n"] == 7)
    assert [s["label"] for s in act7["steps"]] == [
        "n701_silence_check",
        "n702_generate_raccoon",
        "n704_music_library",
        "n705_gen_search_rain",
        "n707_gen_track_info",
        "n708_gen_play",
        "n709_stop_generated",
        "n711_renardo_not_broken",
        "n712_stop_all",
    ]
    manifest = gen.build()["night_marathon_manifest.json"]
    assert next(a for a in manifest["acts"] if a["act"] == 7)["steps"] == 9
    assert manifest["total_steps"] == 132  # +n1010_five_facts (#4157)
    assert "n910_estimate_duration" not in _steps()
    for step in act7["steps"]:
        assert len(step["text"]) <= 66, step["label"]
    # удаления в акте нет: ни шагов, ни паттернов, ни слов в тексте
    assert not [
        s
        for s in act7["steps"]
        if "удали" in s["text"].lower() or "delete" in json.dumps(s["patterns"])
    ]


def test_song_followups_wait_for_the_song_and_skip_without_it():
    steps = _steps()
    assert (
        steps["n707_gen_track_info"]["sleep_before_sec"] == 60
    ), "песня пишется ~90-150 с"
    for label in ("n708_gen_play", "n709_stop_generated"):
        assert "sleep_before_sec" not in steps[label], label
    # харнесс читает ровно то, что пишет генератор: skip_rule разбирает поле без ошибок
    import e2e_events as eev

    assert {
        lbl: r.reason for lbl, r in eev.skip_rules({"steps": list(steps.values())})
    } == {
        "n707_gen_track_info": "NO_SONG",
        "n708_gen_play": "NO_SONG",
        "n709_stop_generated": "NO_SONG",
        "n805b_unsilence": "NOT_SILENCED",  # #3971: «говори» без тишины от n805 не играется
        "n312_stop_music": "NO_MUSIC_N310",  # #4176/#4196: дека пуста -- «выключи» нечего выключать (каскад от n310)
        "n507_wake_under_music": "NO_MUSIC_N506",  # #4196: «вейк под музыкой» без музыки -- не PASS
        "n511_stop_music": "NO_MUSIC_N506",  # #4196: «глуши» без музыки -- не FAIL каскадом
    }


def test_song_skip_does_not_swallow_other_refusals_of_the_act():
    """Отказ чужой просьбы (не «сгенерируй») и живая песня без провала шаги про песню не гасят."""
    skip = re.compile(gen.NO_SONG_LOG)
    for line in (
        "🎚️ [source] asked=[] genre=— кандидаты=['decline:engine'] → decline:engine (code) gen=ok",
        "🎚️ [source] asked=[own] genre=— кандидаты=['synth:club'] → synth:club (code) gen=ok",
        "🎤 [song] «Енот» minimax:V6 жанр=rock task=t1 credits=1 track=gen_ab12 за 96 с",
    ):
        assert not skip.search(line), line
    assert skip.search(
        "🎚️ [source] asked=[generated,vocal] genre=— кандидаты=['decline:no_key'] → decline:no_key (code)"
    )


# ── П3: пустые шаги ─────────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "label, said, ok",
    [
        ("n610_dj_state_honest", "Диджей-режим сейчас выключен.", True),
        ("n610_dj_state_honest", "Диджей-режим не включён, сет снят.", True),
        ("n610_dj_state_honest", "Да, диджей-режим включён и играет.", False),
        ("n610_dj_state_honest", "Режим включён, всё работает.", False),
        ("n808_repeat_last", "Пятьдесят шесть.", True),
        ("n808_repeat_last", "Семью восемь — 56.", True),
        ("n808_repeat_last", "Конечно, повторяю: я не расслышал вопрос.", False),
        ("n909_foreign_language", "По-китайски: 晚安!", True),
        ("n909_foreign_language", "Всем спокойной ночи.", False),
        ("n1001_recall_night", "Ты пьёшь зелёный чай без сахара.", True),
        ("n1001_recall_night", "Ты пьёшь зелёный.", True),
        ("n1001_recall_night", "Приходил Борис, мы слушали музыку.", False),
        ("n1001_recall_night", "Случайно вспомнилось, что было жарко.", False),
        ("n1002_unknown_still_unknown", "Нет, они не назвались.", True),
        ("n1002_unknown_still_unknown", "Не знаю, о ком ты.", False),
        ("n1008_memory_survives_reset", "Ты не ешь лук.", True),
        ("n1008_memory_survives_reset", "Тебя зовут Саша.", False),
    ],
)
def test_empty_steps_now_have_a_check_that_can_turn_red(label, said, ok):
    """27.10 эти шаги были зелёными при любом ответе (n610, n808, n909) или недостижимыми (n1001): теперь ответ решает."""
    log = "\n".join(
        [
            _tts(said),
            "🗣️ SpeechEvent started speech_id=ab12cd34 agent=main epoch=1 text_len=20",
            DONE,
        ]
    )
    assert _verdict(label, log)["pass"] is ok, (label, said)


def test_n808_repeats_the_answer_it_follows_n803_in_act8():
    labels = [s["label"] for a in gen.ACTS if a["n"] == 8 for s in a["steps"]]
    assert (
        labels.index("n808_repeat_last") == labels.index("n803_barge_in_new_topic") + 1
    )
    assert (
        "56|пятьдесят шесть"
        in _steps()["n808_repeat_last"]["acceptance"]["expected_keywords"]
    )


def test_n1001_n1002_n1008_phrases_are_short_and_do_not_contain_their_own_answer():
    steps = _steps()
    for label, answer in (
        ("n1001_recall_night", ("зелён", "зелен", "чай")),
        ("n1002_unknown_still_unknown", ("не назвал",)),
        ("n1008_memory_survives_reset", ("лук",)),
    ):
        text = steps[label]["text"].lower()
        assert len(text) <= 60 and not any(a in text for a in answer), label
    alternatives = steps["n1002_unknown_still_unknown"]["acceptance"][
        "expected_keywords"
    ][0].split("|")
    assert alternatives == gen.NAMES_UNKNOWN_ANSWER.split("|")
    assert "не знаю" not in alternatives, "«не знаю» засчитывало «не знаю, о ком ты»"
    assert (
        "не представил" not in alternatives
    ), "слово вопроса -- эхо вопроса (ревью PR #4105)"


# ── акт 5: n502/n504/n506 ───────────────────────────────────────────────────────────────────────────────────

#: Raw 27.10 (аудит, run 40480199644): вместо синта и правки играющего — трек библиотеки.
LIBRARY_EXEC = "🎵 [music v2] exec track_id=gen_5b4e8031_1793093054 bpm=60"


def test_n502_asks_the_synth_and_checks_the_synth_route_not_any_music_tool():
    step = _steps()["n502_start_renardo"]
    assert "синтезатор" in step["text"] and "рендардо" not in step["text"].lower()
    ok = "\n".join([_source("synth:ambient", "own"), MUSIC_OK, STARTED, DONE])
    assert _verdict("n502_start_renardo", ok)["pass"]
    raw_2710 = "\n".join(
        [
            "🎤 STT: робот сыграй на рем дарда спокойный ночной бит",
            LIBRARY_EXEC,
            MUSIC_OK,
            "🎵 [music v2] started track_id=gen_5b4e8031_1793093054 clock_beat=4",
            DONE,
        ]
    )
    verdict = _verdict("n502_start_renardo", raw_2710)
    assert not verdict["pass"] and verdict["missing"] == [
        gen.ROUTE_SYNTH,
        gen.SYNTH_STARTED,
    ], "27.10 PASS был ложным (#4070): ни синт-пути, ни синт-трека на деке (#3971)"
    cache = "\n".join([_source("cache:gen_5b4e8031", "own"), MUSIC_OK, STARTED, DONE])
    assert not _verdict("n502_start_renardo", cache)["pass"], "трек кэша — не синт"
    both = "\n".join([_source("synth:ambient"), LIBRARY_EXEC, MUSIC_OK, STARTED, DONE])
    assert not _verdict("n502_start_renardo", both)[
        "pass"
    ], "синт-маршрут и трек библиотеки на деке — подмена"


#: #4029 (PR #4116): правка играющего встала на такте того же трека (``engine/track_edit.py``).
EDIT_OK = "✅ Инструмент edit_music выполнен успешно"
EDIT_APPLIED = (
    "🎚️ [music v2] edit applied track_id=req95964:01:A:f7de93b5 at_beat=9856 bpm=147"
)


@pytest.mark.parametrize("label", ["n504_add_bass", "n506_faster", "n505_vibe_dark"])
def test_editing_the_playing_track_by_a_new_library_track_is_not_a_result(label):
    """27.10: n504 и n506 зелёные потому, что тул успешен, а поставлен ДРУГОЙ трек библиотеки. После #4029 правка --
    ``edit_music`` на такте; любой новый трек на деке (``[music v2] exec``) -- подмена (#3971).
    """
    ok = "\n".join([EDIT_OK, EDIT_APPLIED, DONE])
    assert _verdict(label, ok)["pass"]
    substituted = "\n".join([EDIT_OK, EDIT_APPLIED, LIBRARY_EXEC, DONE])
    verdict = _verdict(label, substituted)
    assert not verdict["pass"] and gen.TRACK_EXEC in verdict["acc"]["reason"]


def test_n504_n506_are_no_longer_red_by_design_after_4029():
    """#4029 влит (PR #4116): у n504-n506 нет пометки «красный по делу», шаги ждут правку играющего."""
    steps = _steps()
    for label in ("n504_add_bass", "n505_vibe_dark", "n506_faster"):
        assert "_known_red" not in steps[label], label
        assert gen.TRACK_EXEC in steps[label]["acceptance"]["must_not_call"], label


def test_n711_checks_the_synth_route_after_the_generation_series():
    step = _steps()["n711_renardo_not_broken"]
    assert "синтезатор" in step["text"]
    assert _verdict(
        "n711_renardo_not_broken",
        "\n".join([_source("synth:club"), MUSIC_OK, STARTED, DONE]),
    )["pass"]
    assert not _verdict(
        "n711_renardo_not_broken",
        "\n".join([_source("cache:gen_ab12"), MUSIC_OK, STARTED, DONE]),
    )["pass"]
    assert not _verdict(
        "n711_renardo_not_broken", "\n".join([LIBRARY_EXEC, MUSIC_OK, STARTED, DONE])
    )["pass"]


# ── n210 ────────────────────────────────────────────────────────────────────────────────────────────────────


def _n210(said: str, verdict_name: str = "") -> bool:
    lines = [
        "👤 Speaker: безымянный профиль id=1a2b3c4d (кличка 'Наблюдатель') confidence=0.684 (380 ms)"
    ]
    if verdict_name:
        lines = ["👤 Speaker: '%s' confidence=0.850 (380 ms)" % verdict_name]
    return _verdict("n210_grisha_no_name", "\n".join(lines + [_tts(said), DONE]))[
        "pass"
    ]


def test_n210_mentioning_a_name_is_not_addressing_the_stranger_by_it():
    """27.10 (run 40473977959): «…не перепутаю с Сашей» краснило шаг стемом «Саш», хотя голос опознан верно."""
    assert _n210(
        "Приятно познакомиться! Раз имени не скажешь, не перепутаю с Сашей и Борисом."
    )
    assert _n210("Хорошо, имя называть не обязательно. Я тебя запомню по голосу.")
    for addressed in (
        "Привет, Саша!",
        "Саша, это ты?",
        "Ты Борис, да?",
        "Это Борис.",
        "Борис, привет.",
        "Ну что, ты — Саша.",
    ):
        assert not _n210(addressed), addressed


def test_n210_still_forbids_the_biometric_verdict_and_the_facts_of_others():
    assert not _n210(
        "Хорошо, не буду называть.", verdict_name="Борис"
    ), "опознание незнакомца как Бориса"
    assert not _n210("Я знаю, что тут любят Спартак."), "факт другого диктора"
    assert not _n210("Тут часто приносят пиццу."), "факт другого диктора"
    assert gen.addressed_as("Борис")[:2] == ["Борис,", "Борис!"]


# ── раннер ──────────────────────────────────────────────────────────────────────────────────────────────────


def test_runner_checks_the_scenario_before_the_dry_run_exit_and_any_dispatch():
    """П2: марафон не стартует с репликой, которая не влезет в окно VAD (27.10: n302, n702 нашлись посреди актов)."""
    src = (ROOT / "scripts" / "e2e" / "run_night_marathon.sh").read_text(
        encoding="utf-8"
    )
    check = src.index('gen_night_marathon.py" --check')
    assert check < src.index(
        'if [ "$DRY_RUN" = "1" ]; then'
    ), "DRY_RUN показывает план УЖЕ проверенного сценария"
    assert check < src.index(
        "# --- дедлайн: во сколько прекратить запускать новые акты"
    ), "до первого диспатча"
    assert (
        "exit 2" in src[check : check + 700]
    ), "красный --check -- марафон не стартует"


# ── n1002: вопрос не содержит собственного ответа (ревью PR #4105, круг 2) ──────────────────────────────────


def _speech_log(said: str) -> str:
    return "\n".join(
        [
            _tts(said),
            "🗣️ SpeechEvent started speech_id=ab12cd34 agent=main epoch=1 text_len=20",
            DONE,
        ]
    )


def test_no_keyword_alternative_of_the_act10_questions_is_in_the_question_itself():
    """Слово вопроса в ``expected_keywords`` засчитывает эхо вопроса: 'не представил' стояло в стемах n1002, а вопрос
    -- 'тех, кто не представился' (ревью PR #4105: 'Тех, кто не представился, я знаю по именам: Гриша и Валера.' -- PASS).
    """
    steps = _steps()
    for label in (
        "n1001_recall_night",
        "n1002_unknown_still_unknown",
        "n1008_memory_survives_reset",
    ):
        question = steps[label]["text"].lower()
        for group in steps[label]["acceptance"]["expected_keywords"]:
            for alt in group.lower().split("|"):
                assert alt.strip() not in question, (label, alt)


@pytest.mark.parametrize(
    "said, ok",
    [
        # эхо вопроса и ложь про имена -- красные (ревью PR #4105, дословно)
        ("Тех, кто не представился, я знаю по именам: Гриша и Валера.", False),
        ("Да, знаю: тех, кто не представился, зовут Гриша и Валера.", False),
        ("Не знаю, о ком ты.", False),
        ("Не знаю, но по именам знаю Гришу.", False),
        # честное «по именам не знаю» -- зелёное
        ("Нет, по именам не знаю.", True),
        ("Тех, кто не представился, я по именам не знаю.", True),
        ("Нет, не знаю их по именам.", True),
        ("Нет, имён не знаю, они не называли.", True),
        ("Нет, имен не знаю.", True),
        ("Нет, они не назвались.", True),
        ("Это незнакомые мне люди, без имён.", True),
    ],
)
def test_n1002_echo_of_the_question_is_red_and_honest_ignorance_of_names_is_green(
    said, ok
):
    assert (
        _verdict("n1002_unknown_still_unknown", _speech_log(said))["pass"] is ok
    ), said


# ── консоль Windows (cp1252): раннер локальный (ревью PR #4105, круг 2) ──────────────────────────────────────

#: Консоль Windows по умолчанию: stdout в cp1252, режим UTF-8 выключен. CI (Linux) этого не видит -- задаём явно.
CP1252_CONSOLE = {"PYTHONUTF8": "0", "PYTHONIOENCODING": "cp1252"}


def _cp1252_env() -> Dict[str, str]:
    env = {k: v for k, v in os.environ.items() if k not in CP1252_CONSOLE}
    env.update(CP1252_CONSOLE)
    return env


def _marathon_tree(tmp_path: Path) -> Path:
    """Копия генератора и сценариев ночи: писать и портить их можно, не трогая рабочее дерево."""
    tree = tmp_path / "repo"
    shutil.copytree(
        ROOT / "scripts" / "e2e",
        tree / "scripts" / "e2e",
        ignore=shutil.ignore_patterns("tests", "__pycache__"),
    )
    shutil.copytree(NIGHT, tree / ".github" / "e2e" / "scenarios" / "night")
    # #3971: генератор пишет поля судьи харнесса и сверяет их его таблицами (``e2e_events.COMPARES``)
    harness = tree / ".github" / "workflows" / "scripts"
    harness.mkdir(parents=True)
    shutil.copy2(
        ROOT / ".github" / "workflows" / "scripts" / "e2e_events.py",
        harness / "e2e_events.py",
    )
    return tree


def _run_generator(tree: Path, *args: str):
    proc = subprocess.run(
        [
            sys.executable,
            str(tree / "scripts" / "e2e" / "gen_night_marathon.py"),
            *args,
        ],
        capture_output=True,
        env=_cp1252_env(),
        timeout=120,
    )
    return (
        proc.returncode,
        proc.stdout.decode("utf-8", errors="replace"),
        proc.stderr.decode("utf-8", errors="replace"),
    )


def test_generator_check_prints_cyrillic_and_emoji_on_a_cp1252_console():
    """Раннер зовёт ``--check`` на Windows-хосте без PYTHONUTF8: print() кириллицы и эмодзи падал UnicodeEncodeError,
    и раннер говорил 'сценарий не прошёл --check' про годный сценарий."""
    code, out, err = _run_generator(ROOT, "--check")
    assert code == 0, err[-1500:]
    assert "JSON актуальны" in out and "UnicodeEncodeError" not in err


def test_generator_emit_and_learn_print_on_a_cp1252_console_too(tmp_path):
    tree = _marathon_tree(tmp_path)
    for args, marker in (((), "рукописных"), (("--learn",), "замеров в словаре")):
        code, out, err = _run_generator(tree, *args)
        assert code == 0 and marker in out, (args, err[-1500:])


def _runner_env(tmp_path: Path, **extra: str) -> Dict[str, str]:
    env = {
        "PATH": os.environ.get("PATH", ""),
        "SKIP_SENTINEL": "1",
        "POLL_SEC": "0",
        "DISPATCH_WAIT_SEC": "0",
        "MAX_WAIT": "5",
        "REPORT": _bash_path(tmp_path / "report"),
        "PY_BIN": _bash_path(Path(sys.executable)),
        "HOME": os.environ.get("HOME", str(tmp_path)),
        "SYSTEMROOT": os.environ.get("SYSTEMROOT", ""),
    }
    env.update(extra)
    return env


@bash_required
def test_runner_dry_run_starts_on_a_cp1252_host(tmp_path):
    """Тот же раннер, что у координатора: DRY_RUN доходит до конца, строка --check -- в журнале раннера."""
    _require_bash_sees(tmp_path)
    runner = ROOT / "scripts" / "e2e" / "run_night_marathon.sh"
    proc = subprocess.run(
        [BASH, _bash_path(runner)],
        capture_output=True,
        timeout=120,
        env=_runner_env(tmp_path, DRY_RUN="1", ACTS="1", **CP1252_CONSOLE),
    )
    out = proc.stdout.decode(
        "utf-8", errors="replace"
    )  # байты как есть: text=True съел бы \r
    assert proc.returncode == 0, out + proc.stderr.decode("utf-8", errors="replace")
    assert "gen --check: JSON актуальны" in out and "DRY_RUN=1" in out
    assert (
        b"\r" not in proc.stdout
    ), "хвост \\r от print() Windows прилипал к строкам журнала раннера"


def _break_scenario(tree: Path, how: str) -> None:
    night = tree / ".github" / "e2e" / "scenarios" / "night"
    if how == "stale_json":
        victim = next(night.glob("night_marathon_act1_*_v1.json"))
        with open(victim, "a", encoding="utf-8", newline="\n") as fh:
            fh.write("\n")
    elif how == "phrase_over_the_window":
        victim = night / "night_marathon_act2b_identity_question_v1.json"
        data = json.loads(victim.read_text(encoding="utf-8"))
        step = next(
            s for s in data["steps"] if s["label"] == "n701_sasha_back_recognized"
        )
        step["text"] = "Робот, " + "очень длинная реплика " * 6
        with open(victim, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    else:
        victim = night / "night_marathon_act2c_voice_conflict_question_v1.json"
        victim.write_text("{ не json", encoding="utf-8")


@bash_required
@pytest.mark.parametrize(
    "how", ["stale_json", "phrase_over_the_window", "unreadable_hand_written_act"]
)
def test_runner_does_not_start_and_dispatches_nothing_when_check_is_red(tmp_path, how):
    """Красный ``--check`` -- ``exit 2`` до первого ``gh workflow run``, а не просто строка в тексте скрипта."""
    _require_bash_sees(tmp_path)
    tree = _marathon_tree(tmp_path)
    _break_scenario(tree, how)
    fake_dir = tmp_path / "bin"
    fake_dir.mkdir()
    gh = fake_dir / "gh"
    gh.write_text(
        '#!/usr/bin/env bash\necho "$@" >> "$FAKE_GH_LOG"\nexit 0\n',
        encoding="utf-8",
        newline="\n",
    )
    gh.chmod(gh.stat().st_mode | stat.S_IEXEC)
    gh_log = tmp_path / "gh.log"
    env = _runner_env(
        tmp_path, ACTS="1", FAKE_GH_LOG=_bash_path(gh_log), **CP1252_CONSOLE
    )
    env["PATH"] = f"{_bash_path(fake_dir)}{os.pathsep}{env['PATH']}"
    proc = subprocess.run(
        [BASH, _bash_path(tree / "scripts" / "e2e" / "run_night_marathon.sh")],
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=120,
        env=env,
    )
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "FATAL: сценарий марафона не прошёл" in proc.stderr
    assert (
        not gh_log.exists() or not gh_log.read_text(encoding="utf-8").strip()
    ), "до диспатча не дошло"
    assert (
        "UnicodeEncodeError" not in proc.stderr
    ), "красный по делу, а не по кодировке консоли"


def test_check_names_the_hand_written_act_it_cannot_read(tmp_path):
    """``_manual_steps()`` молча глотал нечитаемый JSON 2b/2c: ``--check`` не видел их реплик и говорил 'всё в порядке'."""
    tree = _marathon_tree(tmp_path)
    _break_scenario(tree, "unreadable_hand_written_act")
    code, out, err = _run_generator(tree, "--check")
    assert code == 1, out + err
    assert "night_marathon_act2c_voice_conflict_question_v1.json" in out
