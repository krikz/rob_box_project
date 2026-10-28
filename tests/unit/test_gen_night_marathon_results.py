# -*- coding: utf-8 -*-
"""Карточка #3971: харнесс ночного марафона судит по РЕЗУЛЬТАТУ действия, а не по имени тула и не по фразе.

Разбор марафонов 23.10 (13:29 и 16:15), §1.A: v1-имена (``set_dj_mode``) и обязательный ``get_music_state`` краснили
верные ответы робота, а «фраза есть → PASS» пропускала пустые обещания. Тесты держат четыре инварианта SSoT
``scripts/e2e/gen_night_marathon.py``: нет v1-имён, нет обязательного ``get_music_state``, у шагов-действий есть
маркер результата, в акте 8 тишину снимает явное «Робот, говори». Сверка шагов с логом -- те же функции, что у
живого харнесса (``e2e_tool_match.step_verdict`` и ``patterns_missing``), второго судьи нет.

Разбор 26.10 (#3971 R4, R6, R19-R22; #3880; задача H2 плана итерации 2): шаги судят по РЕЗУЛЬТАТУ того, что влито
26.10 -- каст голосов #4038 (Гриша и Валера женские), восстановление #4034 (поле ``recover``), стоп с охватом #4035,
лицо команды #4041, скорость и тон #4039. Строки-образцы ниже -- формат строк робота; их источник (f-строка в
коде) держит ``test_marker_sources_exist_in_robot_code``.
"""
from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "e2e"))
sys.path.insert(0, str(REPO_ROOT / ".github" / "workflows" / "scripts"))

import e2e_events as eev  # noqa: E402
import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

#: Шаги, которые просят ДЕЙСТВИЕ с музыкой: каждому обязан сопутствовать маркер результата.
ACTION_LABELS = {
    "n310_execute_backlog_lru", "n312_stop_music",
    "n502_start_renardo", "n504_add_bass", "n505_vibe_dark", "n506_faster", "n511_stop_music",
    "n602_dj_on", "n604_dj_rock", "n605_dj_set_plan", "n607_vibe_chill", "n608_dj_off",
    "n702_generate_raccoon", "n708_gen_play", "n709_stop_generated",
    "n711_renardo_not_broken", "n712_stop_all",
    "n809_music_bed", "n812_stop_music",
}
#: Шаги, где музыка обязана заиграть или смениться: фраза потолка «Музыка пока не заиграла» -- ложь (#3971 R22).
MUSIC_MUST_START_LABELS = {
    "n310_execute_backlog_lru", "n502_start_renardo", "n504_add_bass", "n505_vibe_dark", "n506_faster",
    "n602_dj_on", "n604_dj_rock", "n607_vibe_chill", "n708_gen_play", "n711_renardo_not_broken", "n809_music_bed",
}
#: Вопросы о состоянии музыки: ответ из снимка ``<music_state>``, тул не нужен.
STATE_QUESTION_LABELS = {
    "n501_silence_check", "n503_confirm_playing", "n512_silence_after_stop",
    "n601_silence_check", "n603_confirm_dj_playing", "n609_silence_after_dj",
    "n701_silence_check", "n813_silence_final", "n1005_music_state_final",
}

TURN_LINE = "📊 [turn] turn_id=t1 epoch=1 agent=main stage=%s status=%s latency_ms=900 llm_calls=1 llm_provider=x tokens=10"


def _steps() -> Dict[str, Dict[str, Any]]:
    return {s["label"]: s for a in gen.ACTS for s in a["steps"]}


def _acc(label: str) -> Dict[str, Any]:
    return _steps()[label].get("acceptance", {})


def _patterns(label: str) -> List[str]:
    return _steps()[label]["patterns"]


def _checked_fields(step: Dict[str, Any]) -> str:
    """Поля, которые читает харнесс (без ``_comment``/``_why`` -- там пояснения людям)."""
    acc = step.get("acceptance", {})
    return json.dumps({"patterns": step["patterns"], "expected_tool_calls": acc.get("expected_tool_calls"),
                       "must_not_call": acc.get("must_not_call"), "keywords": acc.get("expected_keywords")},
                      ensure_ascii=False)


def _verdict(label: str, log: str) -> Dict[str, Any]:
    """Вердикт шага по логу: patterns + acceptance, как в харнессе (оба должны пройти)."""
    step = _steps()[label]
    missing = etm.patterns_missing(log, step["patterns"])
    acc = etm.step_verdict(step.get("acceptance", {}), log, label=label)
    return {"missing_patterns": missing, "acceptance": acc, "pass": not missing and acc["pass"]}


def test_no_v1_dj_tool_and_no_mandatory_get_music_state():
    """Сценарии и gate актов не содержат ``set_dj_mode`` и не требуют ``get_music_state``."""
    for label, step in _steps().items():
        fields = _checked_fields(step)
        assert "set_dj_mode" not in fields, label
        assert "get_music_state" not in (step.get("acceptance", {}).get("expected_tool_calls") or []), label
        assert "get_music_state" not in step["patterns"], label
    for act in gen.ACTS:
        assert "set_dj_mode" not in act["gate_tools"], act["n"]
        assert "get_music_state" not in act["gate_tools"], act["n"]


def test_generated_json_has_no_v1_names_and_is_current():
    """Файлы на диске == то, что собирает генератор, и в них нет обязательных v1-ожиданий."""
    assert gen.check() == 0
    for path in (REPO_ROOT / ".github" / "e2e" / "scenarios" / "night").glob("night_marathon_act[3-9]*_v1.json"):
        data = json.loads(path.read_text(encoding="utf-8"))
        if path.name.endswith("_acceptance_v1.json"):
            assert "set_dj_mode" not in data["expected_tool_calls"], path.name
            assert "get_music_state" not in data["expected_tool_calls"], path.name
            continue
        for step in data["steps"]:
            acc = step.get("acceptance", {})
            assert "set_dj_mode" not in json.dumps(step["patterns"] + acc.get("expected_tool_calls", []) +
                                                   acc.get("must_not_call", []), ensure_ascii=False), step["label"]
            assert "get_music_state" not in acc.get("expected_tool_calls", []), step["label"]


def test_state_questions_check_the_phrase_not_a_tool():
    """Вопросы «тихо?/что играет?» проверяют фразу (и отсутствие ложных утверждений), а не вызов тула."""
    steps = _steps()
    assert STATE_QUESTION_LABELS <= set(steps)
    for label in STATE_QUESTION_LABELS:
        acc = steps[label]["acceptance"]
        assert acc["expected_tool_calls"] == [], label
        assert acc.get("expected_keywords") or acc.get("must_not_say"), label
    for label in ("n501_silence_check", "n512_silence_after_stop", "n601_silence_check", "n609_silence_after_dj",
                  "n701_silence_check", "n813_silence_final"):
        assert steps[label]["acceptance"]["expected_keywords"] == [gen.SILENCE_ANSWER], label
    for label in ("n503_confirm_playing", "n603_confirm_dj_playing"):
        assert steps[label]["acceptance"]["must_not_say"][0] == gen.SILENCE_CLAIM, label


def test_every_music_action_step_has_a_result_marker():
    """Шаг-действие: успех тула (``✅ Инструмент … выполнен успешно``), конец хода и запрет ``stage=timeout``."""
    steps = _steps()
    assert ACTION_LABELS <= set(steps)
    for label in ACTION_LABELS:
        step = steps[label]
        pats = step["patterns"]
        assert any(p.startswith("✅ Инструмент ") and p.endswith(" выполнен успешно") for p in pats), label
        assert gen.TURN_DONE in pats or gen.TURN_WITH_LLM in pats, label
        assert gen.TURN_TIMEOUT in step["acceptance"]["must_not_call"], label
    assert gen.MUSIC_STARTED in _patterns("n602_dj_on")
    # #3971 (сверка 27.10): «на синтезаторе» -- не любой трек на деке, а синт-трек (вид результата ``synth``)
    for label in ("n502_start_renardo", "n809_music_bed", "n711_renardo_not_broken"):
        assert gen.SYNTH_STARTED in _patterns(label), label
    for label in ("n511_stop_music", "n608_dj_off", "n709_stop_generated", "n712_stop_all", "n812_stop_music"):
        assert gen.MUSIC_STOPPED in _patterns(label), label


def test_speech_without_result_is_not_a_pass():
    """Ложный PASS закрыт: робот сказал «поставлю ритм» (ход закончился), но тула и трека нет -- красный."""
    log = "\n".join([
        "🗣️ SpeechEvent started speech_id=ab12cd34 agent=main epoch=1 text_len=20",
        TURN_LINE % ("done", "finished"),
    ])
    for label in ("n809_music_bed", "n502_start_renardo", "n602_dj_on", "n604_dj_rock", "n605_dj_set_plan",
                  "n608_dj_off", "n711_renardo_not_broken"):
        assert not _verdict(label, log)["pass"], label


def test_n605_needs_both_announcement_and_result():
    """n605 просит и сет, и объявления вслух: ``SpeechEvent started`` (#3880) И успех тула; одно из двух -- красный."""
    ended = TURN_LINE % ("done", "finished")
    speech = "🗣️ SpeechEvent started speech_id=ab12cd34 agent=main epoch=1 text_len=20"
    done = "✅ Инструмент dj_set выполнен успешно"
    assert _verdict("n605_dj_set_plan", "\n".join([speech, done, ended]))["pass"]
    assert not _verdict("n605_dj_set_plan", "\n".join([done, ended]))["pass"]
    assert not _verdict("n605_dj_set_plan", "\n".join([speech, ended]))["pass"]


def test_dj_set_with_result_markers_passes_n602_n604_n608():
    """Робот v2 включил сет, сменил стиль и выключил через ``dj_set`` -- шаги зелёные (run 132908 их краснил)."""
    done = "✅ Инструмент dj_set выполнен успешно"
    ended = TURN_LINE % ("done", "finished")
    started = "🎵 [music v2] started track_id=set1:01:A clock_beat=4"
    stopped = "🎵 [music v2] stop track_id=set1:01:A reason=user_stop"
    assert _verdict("n602_dj_on", "\n".join(["📤 Отправлен запрос ab: dj_set", done, started, ended]))["pass"]
    assert _verdict("n604_dj_rock", "\n".join(["📤 Отправлен запрос ab: dj_set", done, ended]))["pass"]
    assert _verdict("n607_vibe_chill", "\n".join(["📤 Отправлен запрос ab: dj_set", done, ended]))["pass"]
    assert _verdict("n608_dj_off", "\n".join(["📤 Отправлен запрос ab: dj_set", done, stopped, ended]))["pass"]


def test_turn_timeout_blocks_the_pass():
    """Ход убит по дедлайну (``stage=timeout``): даже с речью и успехом тула шаг красный."""
    log = "\n".join([
        "✅ Инструмент dj_set выполнен успешно",
        TURN_LINE % ("done", "finished"),
        TURN_LINE % ("timeout", "failed"),
    ])
    assert not _verdict("n604_dj_rock", log)["pass"]
    assert _verdict("n711_renardo_not_broken", TURN_LINE % ("timeout", "failed"))["pass"] is False


def test_tool_failure_is_not_a_result():
    """Тул вызван, но завершился ошибкой (``❌ Инструмент … завершился с ошибкой``) -- не результат."""
    log = "\n".join(["📤 Отправлен запрос ab: request_music",
                     "❌ Инструмент request_music завершился с ошибкой: boom", TURN_LINE % ("done", "finished")])
    assert not _verdict("n504_add_bass", log)["pass"]


def test_live_music_edit_is_the_edit_tool_on_the_playing_track_and_n506_forbids_voice_speed():
    """n504-n506 (#4029, PR #4116): правка играющего -- ``edit_music`` и ``edit applied`` того же трека; новый заказ
    (dj_set|request_music|generate_music) -- не правка; «ускорь темп» не крутит голос робота (#3976)."""
    ended = TURN_LINE % ("done", "finished")
    applied = "🎚️ [music v2] edit applied track_id=req93513:01:A:bce191fa at_beat=9856 bpm=147"
    edit = "\n".join(["✅ Инструмент edit_music выполнен успешно", applied, ended])
    for label in ("n504_add_bass", "n505_vibe_dark", "n506_faster"):
        assert _verdict(label, edit)["pass"], label
        for tool in ("dj_set", "request_music", "generate_music"):
            new_order = "\n".join(["✅ Инструмент %s выполнен успешно" % tool, ended])
            assert not _verdict(label, new_order)["pass"], (label, tool)
        claimed = "\n".join(["✅ Инструмент edit_music выполнен успешно", ended])
        assert not _verdict(label, claimed)["pass"], (label, "успех тула без правки на такте")
    forbidden = _acc("n506_faster")["must_not_call"]
    assert "set_speed" in forbidden and "set_pitch" in forbidden
    voice_instead = "\n".join(["📤 Отправлен запрос ab: set_speed", "✅ Инструмент set_speed выполнен успешно", ended])
    assert not _verdict("n506_faster", voice_instead)["pass"]
    voice_and_music = "\n".join(["tool_calls=(ToolCall(name='set_speed'),)", edit])
    assert not _verdict("n506_faster", voice_and_music)["pass"]


def test_n702_is_marked_known_red_under_kie_quota():
    """Харнесс не умеет условие по ``<providers>`` (дамп запроса вырезается): n702 помечен, а не взломан."""
    step = _steps()["n702_generate_raccoon"]
    assert "KIE" in step["_known_red"] and "decline" in step["_known_red"]
    assert "generate_music" in step["acceptance"]["expected_tool_calls"]
    assert not any("perform" in p for p in step["patterns"])


def test_act8_unsilence_precedes_n806_and_n805_is_unchanged():
    """Решение Шифу 23.10: тишину снимает только «Робот, говори»; вейк тишину НЕ снимает."""
    act8 = next(a for a in gen.ACTS if a["n"] == 8)
    labels = [s["label"] for s in act8["steps"]]
    steps = {s["label"]: s for s in act8["steps"]}
    i805, i806 = labels.index("n805_hard_stop"), labels.index("n806_long_count_again")
    unsilence = [i for i, s in enumerate(act8["steps"]) if "говори" in s["text"].lower()]
    assert unsilence and i805 < unsilence[0] < i806
    step = act8["steps"][unsilence[0]]
    assert step["text"].startswith("Робот, говори")
    assert "tier1 kind=unsilence" in step["patterns"]
    assert step.get("silent_ok") is True
    assert steps["n805_hard_stop"]["text"] == "Робот, всё, хватит, замолчи."
    assert steps["n806_long_count_again"]["patterns"] == [gen.SPEECH_STARTED]


def test_patterns_are_valid_regex():
    import re
    for label, step in _steps().items():
        for pat in step["patterns"]:
            re.compile(pat)


# ── разбор 26.10: строки робота (формат f-строк кода, см. test_marker_sources_exist_in_robot_code) ─────────────

DONE = TURN_LINE % ("done", "finished")


def _tts(text: str) -> str:
    """Что робот ПРОИЗНЁС (канал ``robot_speech``: ``🔊 TTS: … text='…'``)."""
    return ("[tts_node-4] [INFO] [1792795079.69] [tts_node]: 🔊 TTS: speech_id=764009bb, dialogue_id=None, "
            "batch=None None/None, voice=default, lang=default, text='%s'" % text)


def _speaker(name: str, conf: float) -> str:
    return "[speaker_id_node-8] [INFO] [1792790345.07] [speaker_id_node]: 👤 Speaker: '%s' confidence=%.3f (336 ms)" % (
        name, conf)


def _param(name: str, value: str) -> str:
    return "[tts_node-4] [INFO] [1792792911.93] [tts_node]: 🎚️ %s изменён: %s" % (name, value)


def _face(emotion: str) -> str:
    return ("[dialog_node-3] [INFO] [1792796472.28] [dialog_node]: 🎭 [turn-v2] лицо команды на done: emotion=%s "
            "gesture=- face=llm turn=de10cf091894" % emotion)


TALE_START = ("[dialog_node-3] [INFO] [1792792979.94] [dialog_node]: 🧭 [turn-v2] выступление старт kind=tale "
              "backing=calm segments=7 → {'status': 'pending', 'block_id': 'perform-ee2404b9048f'} turn=ee2404b9048f")
TALE_END = ("[dialog_node-3] [INFO] [1792793074.35] [dialog_node]: 🎤 [perform] perform-: конец (done) сегментов 7, "
            "символов 917, synced=True, подложка stopped")
TALE_BLOCK_END = ("[dialog_node-3] [INFO] [1792793074.36] [dialog_node]: 🗓️ [plan] PlanEvent block_end block=perform-ee24 "
                  "seg=7 synced=true spoken_chars=917 text_len=0 offset_ms=- after=-/- eta_ms=0 reason=done")
TALE_V1_COMMAND = ("[dialog_node-3] [INFO] [1792792979.88] [dialog_node]: 🧾 [command] provider=deepseek mode=tool_choice "
                   "verdict=command actions=[] speech_dropped=0")


def test_cast_grisha_and_valera_speak_in_feminine_and_tts_is_minimax():
    """#4038 (решение Шифу 26.10): Гриша и Валера -- женские голоса; реплики Гриши в женском роде, «дяди» нет."""
    steps = _steps()
    assert steps["n210_grisha_no_name"]["text"].startswith("Робот, я мимо шла.")
    assert steps["n210_grisha_no_name"]["voice"] == gen.GRISHA
    assert "курьерша уехала" in steps["n312_stop_music"]["text"]
    texts = " ".join(s["text"] for s in steps.values())
    assert "мимо шёл" not in texts and "дядя Гриша" not in texts and "курьер уехал" not in texts
    for name, payload in gen.build().items():
        if name.endswith("_acceptance_v1.json"):
            assert payload["voice_provenance"]["tts"] == "minimax", name


def test_every_generated_act_names_its_cast_for_the_privacy_filter():
    """Ревью #4034: e2e-БД дикторов уходит в публичные артефакты -- харнесс выносит только имена ролей поля ``cast``."""
    assert gen.CAST == {gen.SASHA: "Саша", gen.BORIS: "Борис", gen.GRISHA: "Гриша", gen.VALERA: "Валера"}
    voices = {s["voice"] for s in _steps().values()}
    assert voices <= set(gen.CAST), voices - set(gen.CAST)
    for name, payload in gen.build().items():
        if name.endswith("_v1.json") and not name.endswith("_acceptance_v1.json"):
            assert payload["cast"] == ["Саша", "Борис", "Гриша", "Валера"], name


def test_cast_steps_are_not_marked_voice_limit_red():
    """Голоса разведены -- шаги атрибуции не помечены «предел голосов»; n306 принимает «незнаком|неопознан»."""
    steps = _steps()
    for label in ("n210_grisha_no_name", "n304_bg_grisha_unknown", "n305_bg_valera_unknown", "n306_who_was_talking",
                  "n307_count_voices", "n309_bg_command_valera", "n311_who_asked"):
        assert "_known_red" not in steps[label], label
    assert _acc("n306_who_was_talking")["expected_keywords"] == ["Борис", "незнаком|неопознан"]
    honest = "\n".join([_tts("Борис -- знакомый, а двое других голосов неопознанные."), DONE])
    assert _verdict("n306_who_was_talking", honest)["pass"]


def test_act1_checks_the_wake_itself_no_wake_is_not_recovered():
    """Акт 1 -- проверка вейка: харнесс не лечит потерянный вейк «Робот!» (no_wake), тишину -- лечит (#4034)."""
    act1 = gen.build()["night_marathon_act1_wakeup_v1.json"]
    assert act1["recover"] == ["silenced"]
    for step in act1["steps"]:
        assert eev.recover_kinds(act1, step) == ("silenced",), step["label"]
    act2 = gen.build()["night_marathon_act2_acquaintance_v1.json"]
    assert "recover" not in act2  # остальные акты -- по умолчанию харнесса (все виды)
    assert eev.recover_kinds(act2, act2["steps"][0]) == ("no_wake", "silenced")


def test_n412_tale_is_judged_by_perform_events_and_never_retried():
    """R4: v2 пишет actions=[] и «выступление старт» -- v1-паттерн краснил верную сказку, ретрай заказывал вторую."""
    step = _steps()["n412_skazka_multivoice"]
    assert "retry_acceptance" not in step, "после старта выступления повтор заказал бы вторую сказку"
    speech = _tts("Жила-была Красная Шапочка, бабушка жила за лесом.")
    assert _verdict("n412_skazka_multivoice", "\n".join([TALE_V1_COMMAND, TALE_START, speech, TALE_END]))["pass"]
    assert _verdict("n412_skazka_multivoice", "\n".join([TALE_START, speech, TALE_BLOCK_END]))["pass"]
    started_only = _verdict("n412_skazka_multivoice", "\n".join([TALE_V1_COMMAND, TALE_START, speech]))
    assert not started_only["pass"] and started_only["missing_patterns"] == [gen.PERFORM_DONE]
    stopped = TALE_END.replace("(done)", "(stopped)")
    assert not _verdict("n412_skazka_multivoice", "\n".join([TALE_START, speech, stopped]))["pass"]
    poem = TALE_START.replace("kind=tale", "kind=poem")
    assert not _verdict("n412_skazka_multivoice", "\n".join([poem, speech, TALE_END]))["pass"]


def test_step_waiting_for_a_performance_cannot_be_retried():
    """R4 структурой: шаг с паттерном старта выступления и ``retry`` генератор не соберёт -- повтор = второе."""
    with pytest.raises(ValueError, match="retry"):
        gen.step("nX_tale", "Робот, расскажи сказку.", patterns=[gen.perform_started("tale"), gen.PERFORM_DONE],
                 retry=1)
    assert gen.step("nX_tale", "Робот, расскажи сказку.", patterns=[gen.perform_started("tale")])["patterns"]
    for label, step in _steps().items():
        if any(p.startswith(gen.PERFORM_STARTED_MARK) for p in step["patterns"]):
            assert "retry_acceptance" not in step, label


def test_performance_question_is_not_the_requested_speech():
    """R22 (робот 23.10 22:53:09): «выключить и зачитать?» дал ``SpeechEvent started`` -- n804/n806/n810 краснеют."""
    started = "🗣️ SpeechEvent started speech_id=88c39b53 agent=personality epoch=11 text_len=100"
    asked = _tts("Сейчас играет поставь фоном ровный ритм · 125 BPM, выключить и зачитать?")
    rap = _tts("Йо, енотик по ночам паяет провода, бит качает, и ему не беда.")
    for label in ("n804_long_story", "n806_long_count_again", "n810_rap_over_music"):
        assert gen.PERFORM_ASKS_TO_REPLACE in _acc(label)["must_not_say"], label
        assert not _verdict(label, "\n".join([asked, started, DONE]))["pass"], label
    assert _verdict("n810_rap_over_music", "\n".join([rap, started, DONE]))["pass"]


def test_n902_n903_silent_sound_is_judged_by_tool_result():
    """Решение Шифу 26.10: тихий звук/анимация по просьбе -- норма; шаг судит успех тула, а не речь."""
    steps = _steps()
    assert steps["n902_sound_doorbell"]["silent_ok"] is True and steps["n903_sound_siren_anim"]["silent_ok"] is True
    sound = "✅ Инструмент play_sound выполнен успешно"
    anim = "✅ Инструмент play_animation выполнен успешно"
    assert _verdict("n902_sound_doorbell", "\n".join([sound, DONE]))["pass"]
    assert not _verdict("n902_sound_doorbell", "\n".join(["📤 Отправлен запрос ab: play_sound", DONE]))["pass"]
    assert _verdict("n903_sound_siren_anim", "\n".join([sound, anim, DONE]))["pass"]
    assert _verdict("n903_sound_siren_anim", "\n".join([sound, anim, _tts("Виу-виу!"), DONE]))["pass"]
    assert not _verdict("n903_sound_siren_anim", "\n".join([sound, DONE]))["pass"], "мигалку просили тоже"


def _anim_request(name: str) -> str:
    return "[mcp_server]: 📥 Запрос выполнения: play_animation с параметрами {'animation': '%s', 'duration': 4}" % name


def _anim_accepted(name: str) -> str:
    """Путь ``success=True`` тула (``tools/animation.py``): «Принято {data} (показывает диалог по ExpressionRequest)»."""
    return ("[mcp_server]: [play_animation] Принято {'animation': '%s', 'duration': 4.0} "
            "(показывает диалог по ExpressionRequest)" % name)


def test_n904_n905_emotion_shown_by_face_or_animation():
    """v2 показывает эмоцию лицом команды (#4041); тул play_animation не обязателен, но засчитывается только его успех."""
    assert _verdict("n904_anim_sad", "\n".join([_face("sad"), DONE]))["pass"]
    assert _verdict("n904_anim_sad", "\n".join([_anim_request("sad"), _anim_accepted("sad"), DONE]))["pass"]
    assert not _verdict("n904_anim_sad", "\n".join([_face("happy"), DONE]))["pass"], "грусть, а не радость"
    assert not _verdict("n904_anim_sad", DONE)["pass"]
    assert _verdict("n905_anim_victory", "\n".join([_face("victory"), DONE]))["pass"]
    assert _verdict("n905_anim_victory", "\n".join([_face("happy"), DONE]))["pass"]
    assert _steps()["n904_anim_sad"]["silent_ok"] is True
    for label in ("n904_anim_sad", "n905_anim_victory"):
        assert "play_animation" not in _acc(label).get("expected_tool_calls", []), label


@pytest.mark.parametrize("label,emotion", [("n904_anim_sad", "sad"), ("n905_anim_victory", "victory"),
                                           ("n906_anekdot_anim", "happy")])
def test_animation_request_without_success_is_not_the_emotion(label, emotion):
    """Ревью #4046: строка ЗАПРОСА тула совпадала с паттерном -- упавший вызов (неизвестная анимация, нет face)
    засчитывался. Теперь нужен успех тула или лицо команды."""
    speech = "🗣️ SpeechEvent started speech_id=ab12cd34 agent=main epoch=1 text_len=40"
    failed = "\n".join([_anim_request(emotion), "❌ Инструмент play_animation завершился с ошибкой: Неизвестная анимация",
                        speech, DONE])
    assert not _verdict(label, failed)["pass"], label
    ok = "\n".join([_anim_request(emotion), _anim_accepted(emotion), speech, DONE])
    assert _verdict(label, ok)["pass"], label
    assert not _verdict(label, "\n".join([_anim_accepted("idle"), speech, DONE]))["pass"], "не та анимация"


def test_music_must_start_steps_forbid_the_timeout_phrase():
    """#3971 R22 / n809: «Музыка пока не заиграла.», а трек стартовал через 1,4 с -- ложь; n605 не ужесточён."""
    steps = _steps()
    assert MUSIC_MUST_START_LABELS <= set(steps)
    lie = _tts("Музыка пока не заиграла.")
    for label in MUSIC_MUST_START_LABELS:
        assert etm.step_verdict(_acc(label), lie)["forbidden_said"], label
    assert not etm.step_verdict(_acc("n605_dj_set_plan"), lie)["forbidden_said"], "#4006 ещё не влит"
    # #3971: n809 просит синт («на синтезаторе») -- путь synth и синт-трек req… на деке
    route = "🎚️ [source] asked=[] genre=— кандидаты=['synth:club'] → synth:club (code) gen=ok"
    ok = "\n".join([route, "✅ Инструмент request_music выполнен успешно", "🎵 [music v2] started track_id=req1:01:A",
                    DONE])
    assert _verdict("n809_music_bed", ok)["pass"]
    lie = "\n".join([_tts("Музыка пока не заиграла."), route, "✅ Инструмент request_music выполнен успешно",
                     "🎵 [music v2] started track_id=req1:01:A", DONE])
    assert not _verdict("n809_music_bed", lie)["pass"]


def test_music_stop_on_empty_deck_is_not_a_result():
    """n812 23.10: ``stop track_id=None`` + ``was_playing=False`` давал PASS -- теперь красный; нужен трек на деке."""
    idle = "\n".join(["✅ Инструмент stop_music выполнен успешно",
                      "🎵 [music v2] stop track_id=None reason=user_stop",
                      "[mcp_server]: [stop_music] [stop_music] was_playing=False", DONE])
    real = "\n".join(["✅ Инструмент stop_music выполнен успешно",
                      "🎵 [music v2] stop track_id=req95964:01:A:f7de93b5 reason=user_stop",
                      "[mcp_server]: [stop_music] [stop_music] was_playing=True", DONE])
    for label in ("n312_stop_music", "n511_stop_music", "n709_stop_generated", "n712_stop_all", "n812_stop_music"):
        assert not _verdict(label, idle)["pass"], label
        assert _verdict(label, real)["pass"], label
    no_flag = real.replace("was_playing=True", "was_playing=False")
    assert not _verdict("n812_stop_music", no_flag)["pass"]


def test_n811_stops_speech_and_keeps_music():
    """#4035: «хватит читать, а музыку оставь» -- Tier-1 stop_speech; деку не трогать."""
    cancel = "🗣️ SpeechEvent cancelled speech_id=88c39b53 agent=personality epoch=11 text_len=100 reason=stopped"
    tier1 = "🧭 [turn-v2] tier1 kind=stop_speech args={'keep_music': True} turn=ab12"
    assert _verdict("n811_stop_speech_keep_music", "\n".join([cancel, tier1]))["pass"]
    stop_all = tier1.replace("stop_speech", "stop_all")
    assert not _verdict("n811_stop_speech_keep_music", "\n".join([cancel, stop_all]))["pass"]
    killed = "🎵 [music v2] stop track_id=req1:01:A reason=user_stop"
    assert not _verdict("n811_stop_speech_keep_music", "\n".join([cancel, tier1, killed]))["pass"]
    outro = "📥 Запрос выполнения: stop_music с параметрами {'boundary': 'bar'}"
    assert not _verdict("n811_stop_speech_keep_music", "\n".join([cancel, tier1, outro]))["pass"]


@pytest.mark.parametrize("label,ok,bad", [
    ("n409_speed_down", [_param("minimax_speed", "0.8")], [_param("minimax_speed", "1.2"),
                                                           "✅ Инструмент set_speed выполнен успешно"]),
    ("n411_pitch_bass", [_param("pitch_shift", "0.8")], [_param("pitch_shift", "1.2"),
                                                         "✅ Инструмент set_pitch выполнен успешно"]),
])
def test_prosody_steps_judge_the_effect_not_the_call(label, ok, bad):
    """#4039: тул меняет ось активного провайдера, нода пишет «<ось> изменён: <значение>» -- это и есть результат."""
    for line in ok:
        assert _verdict(label, "\n".join([line, DONE]))["pass"], (label, line)
    for line in bad:
        assert not _verdict(label, "\n".join([line, DONE]))["pass"], (label, line)


def _speed_said(provider: str, param: str, said: str) -> str:
    """Ответ тула скорости (``system.SetSpeedTool``: ``[set_speed] Speed [<провайдер>, <ось>]: <фраза>``)."""
    return "[mcp_server]: [set_speed] Speed [%s, %s]: %s" % (provider, param, said)


def test_n410_faster_than_before_not_above_normal():
    """Ревью #4046: после n409 (0.8) шаг ±0.2 даёт 1.0 -- это «быстрее, чем было»; красить его за «не выше нормы» нельзя."""
    after_slow = "\n".join([_speed_said("minimax", "minimax_speed", "Говорю быстрее: скорость обычная"),
                            _param("minimax_speed", "1"), DONE])
    assert _verdict("n410_speed_up", after_slow)["pass"]
    from_norm = "\n".join([_speed_said("yandex", "yandex_speed", "Говорю быстрее: скорость 1.2"),
                           _param("yandex_speed", "1.2"), DONE])
    assert _verdict("n410_speed_up", from_norm)["pass"]
    for bad in (
        "\n".join([_param("minimax_speed", "1.2"), DONE]),  # нода изменилась, но шаг «быстрее» тул не подтвердил
        "\n".join([_speed_said("minimax", "minimax_speed", "Говорю медленнее: скорость 0.6"),
                   _param("minimax_speed", "0.6"), DONE]),
        "\n".join([_speed_said("minimax", "minimax_speed", "Скорость уже максимальная"), DONE]),
        "\n".join(["✅ Инструмент set_speed выполнен успешно", DONE]),
        "\n".join([_speed_said("minimax", "pitch_shift", "Говорю быстрее"), _param("pitch_shift", "1.2"), DONE]),
    ):
        assert not _verdict("n410_speed_up", bad)["pass"], bad


def test_n411b_reset_accepts_value_or_already_normal_only():
    """«Говори нормально»: оси снова 1 либо тул прочитал, что они уже на норме (не каскад от n410/n411)."""
    reset = "\n".join([_param("minimax_speed", "1"), _param("pitch_shift", "1"), DONE])
    assert _verdict("n411b_prosody_normal", reset)["pass"]
    already = "\n".join(["[mcp_server]: [set_speed] Speed [minimax, minimax_speed]: Скорость уже нормальная",
                         "[mcp_server]: [set_pitch] Pitch [minimax, pitch_shift]: Голос уже нормальный", DONE])
    assert _verdict("n411b_prosody_normal", already)["pass"]
    half = "\n".join([_param("minimax_speed", "1"), _param("pitch_shift", "1.2"), DONE])
    assert not _verdict("n411b_prosody_normal", half)["pass"], "тон не вернули"
    assert not _verdict("n411b_prosody_normal", "\n".join([_param("minimax_speed", "1.2"), DONE]))["pass"]
    labels = [s["label"] for a in gen.ACTS if a["n"] == 4 for s in a["steps"]]
    assert labels.index("n411_pitch_bass") < labels.index("n411b_prosody_normal") < labels.index(
        "n412_skazka_multivoice")


def test_vacuum_patterns_replaced_by_results():
    """R6: «Speaker|identify candidates|speaker_id» и «provider=<любой>» совпадали с каждым ходом."""
    for label in ("n202a_sasha_warmup", "n202b_sasha_warmup_hum", "n202c_sasha_warmup_guess"):
        assert _verdict(label, "\n".join([_speaker("Саша", 0.914), DONE]))["pass"], label
        assert not _verdict(label, "\n".join([_speaker("Саша", 0.73), DONE]))["pass"], label
        assert not _verdict(label, "\n".join([_speaker("Борис", 0.91), DONE]))["pass"], label
        assert not _verdict(label, "👤 Speaker: unknown (311 ms)\n" + DONE)["pass"], label
    for label in ("n205a_boris_warmup", "n205b_boris_warmup_spartak", "n205c_boris_warmup_tease"):
        assert _verdict(label, "\n".join([_speaker("Борис", 0.88), DONE]))["pass"], label
        assert not _verdict(label, "\n".join([_speaker("Саша", 0.88), DONE]))["pass"], label
    # Решение Шифу 28.10 («пусть сохраняет»): на «запоминай» робот вправе записать сводку о Саше в память -- запрета
    # memory_save у n202a нет; шаг судит механизм (голос опознан), а не запись факта.
    saved = "\n".join([_speaker("Саша", 0.93), "✅ Инструмент memory_save выполнен успешно", DONE])
    assert _verdict("n202a_sasha_warmup", saved)["pass"], "запись в память на «запоминай» -- не провал (28.10)"
    assert "memory_save" not in _acc("n202a_sasha_warmup")["must_not_call"]
    unknown = "\n".join(["👤 Speaker: unknown (311 ms)", "✅ Инструмент memory_save выполнен успешно", DONE])
    assert not _verdict("n202a_sasha_warmup", unknown)["pass"], "запись в память не заменяет опознание голоса"
    synth = "🗣️ SpeechEvent synthesized speech_id=e492c4cd agent=personality epoch=8 text_len=134 provider=%s"
    honest = "\n".join([_tts("Сейчас я на MiniMax, голос male-qn-qingse."), synth % "minimax", DONE])
    assert _verdict("n402_current_voice", honest)["pass"]
    assert not _verdict("n402_current_voice", "\n".join([_tts("Голосом Антона."), synth % "minimax", DONE]))["pass"]
    assert _verdict("n404_speak_after_set", "\n".join([synth % "yandex", DONE]))["pass"]
    assert not _verdict("n404_speak_after_set", "\n".join([synth % "minimax", DONE]))["pass"], "голос не сменён"


def test_n1002_and_n1007_judge_honesty_not_narrow_words():
    """R20: честное «не назвались/неопознанные» -- зелёное; утечка -- по фразам ОКНА, а не по фактам памяти."""
    for said in ("Нет, они не назвались.", "Они так и остались неопознанными.", "Без имён, я их не узнал."):
        assert _verdict("n1002_unknown_still_unknown", "\n".join([_tts(said), DONE]))["pass"], said
    from_memory = _tts("Историю я не держу. Помню из памяти: ты Саша, а Борис болеет за Спартак, был енот.")
    assert _verdict("n1007_no_leak", "\n".join([from_memory, DONE]))["pass"], "факты долгой памяти -- не утечка"
    leak = _tts("Ты спрашивал, знаю ли я тех, кто не представился, по именам.")
    assert not _verdict("n1007_no_leak", "\n".join([leak, DONE]))["pass"]
    # #3971: у каждого шага в ``must_not_call`` есть ход, закрытый заглушкой; подстрок фактов памяти там нет
    assert _acc("n1007_no_leak")["must_not_call"] == [gen.TURN_FALLBACK], "подстрока по всему логу ловила факты памяти"


def test_no_song_rule_marks_the_steps_about_this_runs_song_and_nothing_destructive_is_left():
    """#4077 П1: песни прогона нет (баланс KIE 0 / провал заказа / отказ пути) -- n707-n709 SKIP NO_SONG; удаления нет."""
    import re

    steps = _steps()
    for label in ("n707_gen_track_info", "n708_gen_play", "n709_stop_generated"):
        # ревью #4046: харнесс поле читает (e2e_events.skip_rule) -- без «_», как остальные поля для харнесса
        assert steps[label]["skip_when"] == {"reason": "NO_SONG", "log": gen.NO_SONG_LOG}, label
        assert eev.skip_rule(steps[label]).reason == "NO_SONG", label
    for label in ("n701_silence_check", "n702_generate_raccoon", "n704_music_library", "n705_gen_search_rain",
                  "n711_renardo_not_broken", "n712_stop_all"):
        assert "skip_when" not in steps[label], label
    assert not any("_skip_when" in s for s in steps.values())
    # KIE_QUOTA влит в NO_SONG: одна таблица, без второго ключа на ту же беду; NOT_SILENCED (#3971, n805b) -- другая беда
    assert set(gen.SKIP_WHEN) == {"NO_SONG", "NOT_SILENCED", "NO_MUSIC_N310", "NO_MUSIC_N506"}  # #4196: дека пуста (каскад)
    assert [label for label, s in steps.items() if s.get("skip_when", {}).get("reason") == "NO_SONG"] == [
        "n707_gen_track_info", "n708_gen_play", "n709_stop_generated"]
    for label in ("n703_merge_komar", "n706_gen_save_for_boris", "n710_gen_delete", "n710b_gen_delete_confirm"):
        assert label not in steps, label
    assert "'action': 'delete'" not in json.dumps([s["patterns"] for s in steps.values()], ensure_ascii=False)
    for line in ("[mcp_server]: 🎼 [cover] баланс KIE: 0.0", "🎼 [cover] баланс KIE: 0",
                 "[voice_assistant]: 🎤 [song] не вышло: задача t1 без сохранённых треков",
                 "🎤 [song] сбой: TimeoutError: ", "🎤 [song] не заиграла: not_started",
                 "🎚️ [source] asked=[generated,vocal,robot_voice] genre=— кандидаты=['decline:no_balance'] → "
                 "decline:no_balance (code) gen=no_balance"):
        assert re.search(gen.NO_SONG_LOG, line), line
    for fine in ("🎼 [cover] баланс KIE: 1712", "🎼 [cover] баланс KIE: 0.5", "🎼 [cover] баланс KIE: неизвестен",
                 "🎤 [song] «Енот-механик» minimax:V6 жанр=rock task=t1 credits=1 track=gen_ab12 за 96 с",
                 "🎚️ [source] asked=[generated,vocal,robot_voice] genre=— кандидаты=['generate:robot_song:V6'] → "
                 "generate:robot_song:V6 (code) gen=ok",
                 "🎚️ [source] asked=[] genre=— кандидаты=['decline:engine'] → decline:engine (code) gen=ok"):
        assert not re.search(gen.NO_SONG_LOG, fine), fine


def test_n702_asks_the_generate_robot_song_cell_of_the_fork_not_a_live_performance():
    """Вилка Шифу 27.10 (#4070/#4079): «сгенерируй песню своим голосом» -- generate:robot_song; пение вживую -- ошибка."""
    step = _steps()["n702_generate_raccoon"]
    assert "сгенерируй" in step["text"] and "своим голосом" in step["text"] and len(step["text"]) <= 60
    assert gen.ROUTE_GENERATE_ROBOT_SONG in step["patterns"]
    assert gen.PERFORM_STARTED_MARK in step["acceptance"]["must_not_call"]
    ok = "\n".join(["🎚️ [source] asked=[generated,vocal,robot_voice] genre=— кандидаты=['generate:robot_song:V6'] → "
                    "generate:robot_song:V6 (code) gen=ok", "✅ Инструмент generate_music выполнен успешно", DONE])
    assert _verdict("n702_generate_raccoon", ok)["pass"]
    live = "\n".join(["🎚️ [source] пение song слова=— кандидаты=[live:song] → live:song (code)",
                      "🧭 [turn-v2] выступление старт kind=song backing=none segments=3", DONE])
    assert not _verdict("n702_generate_raccoon", live)["pass"]
    wrong_cell = ok.replace("generate:robot_song:V6 (code)", "cache:gen_ab12 (code)")
    assert not _verdict("n702_generate_raccoon", wrong_cell)["pass"]


# ── #4025 (PR #4056): удаление трека в два шага - вопрос, затем «да» ─────────────────────────────────────────
# #4077 П1: шаги n710/n710b сняты из марафона (удаление бьёт по живой библиотеке, а к треку ЭТОГО прогона шаг не
# привязан -- нужен track_id песни, П5). Маркеры результата удаления остаются в генераторе для их возврата, и их
# контракт с логом контейнера держит тест ниже.

QUESTION = "Удалить «Енот-мастер»? Скажи «да, удаляй»."
#: Что тул отдал в лог контейнера, когда спросил вместо удаления: ``mcp_server.py`` (ROS-логгер, WARNING).
REFUSED = ("[mcp_server-9] [WARN] [1792795162.30] [mcp_server]: ❌ Инструмент music_library завершился с ошибкой: "
           + QUESTION)
#: Строки, которой в логе контейнера НЕТ: python-логгер ``rob_box_dialog.execute`` уровня INFO (27.10: 0 строк за 5 ч).
EXECUTE_REJECTED = ("[dialog_node-3] [2026-10-24 01:39:22 - rob_box_dialog.execute:231 - INFO] 📨 [execute] событие "
                    "music_library t5/0 rejected needs_confirmation ход_ждёт=True запрос=True")


@pytest.mark.skipif(shutil.which("grep") is None, reason="нужен настоящий grep -E, как у харнесса")
def test_delete_markers_are_lines_of_the_container_log_not_python_info_logger():
    """27.10 06:00 UTC: за 5 ч в логе voice-assistant 0 строк ``📨 [execute] событие`` (python-логгер INFO не выводится).
    Маркер удаления -- строка ROS-логгера mcp_server; ``[execute]`` в паттернах и ``must_not_call`` шагов запрещён."""
    steps = _steps()
    everything = [p for s in steps.values() for p in s["patterns"] + s.get("acceptance", {}).get("must_not_call", [])]
    assert not [p for p in everything if "[execute]" in p or "\\[execute\\]" in p]
    # настоящий grep -E, как check_patterns харнесса: вопрос -- HIT, стёрли -- MISS; строка python-логгера -- MISS
    erased = "[mcp_server-9] [INFO] [1792795162.34] [mcp_server]: ✅ Инструмент music_library выполнен успешно"
    for line, hit in ((REFUSED, True), (erased, False), (EXECUTE_REJECTED, False)):
        found = subprocess.run(["grep", "-E", "--", gen.LIBRARY_DELETE_ASKED], input=line, capture_output=True,
                               text=True, encoding="utf-8").returncode == 0
        assert found is hit, line


def _src(rel: str) -> str:
    return (REPO_ROOT / rel).read_text(encoding="utf-8")


@pytest.mark.parametrize("rel,fragment", [
    ("src/rob_box_dialog/rob_box_dialog/turn.py",
     "выступление {'правка' if cmd.edit_pending else 'старт'} kind={cmd.perform.kind}"),
    ("src/rob_box_dialog/rob_box_dialog/turn.py", 'f"{LOG_MARK} tier1 kind={command.kind}'),
    ("src/rob_box_dialog/rob_box_dialog/plan/perform.py", 'LOG_MARK = "🎤 [perform]"'),
    ("src/rob_box_dialog/rob_box_dialog/plan/perform.py", 'self.log(f"конец ({reason}) сегментов'),
    ("src/rob_box_dialog/rob_box_dialog/plan/plan_events.py", 'f"PlanEvent {event.kind} block='),
    ("src/rob_box_dialog/rob_box_dialog/grammar.py", 'Tier1Command("stop_speech"'),
    ("src/rob_box_dialog/rob_box_dialog/phrases/ru.yaml", 'media.timeout: &not_started ["Музыка пока не заиграла."]'),
    ("src/rob_box_dialog/rob_box_dialog/phrases/ru.yaml",
     'perform.ask_replace_music: ["Сейчас играет {title}, выключить и зачитать?"]'),
    ("src/rob_box_voice/rob_box_voice/core/dialog_v2_ports.py",
     'f"🎭 [turn-v2] лицо команды на {stage}: emotion={plan.emotion}'),
    ("src/rob_box_voice/rob_box_voice/tts_node.py", 'f"🎚️ {param.name} изменён: {getattr(self, param.name):g}"'),
    ("src/rob_box_voice/rob_box_voice/speaker_id_node.py", "f\"👤 Speaker: '{match.name}' confidence={match.confidence:.3f} \""),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py", 'f"Speed [{provider}, {axis.param}]: {result.message}"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py", 'f"Pitch [{provider}, {axis.param}]: {result.message}"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py", '"normal": _Step(0, "Нормальная скорость", "Скорость уже нормальная")'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py", '"normal": _Step(0, "Нормальный голос", "Голос уже нормальный")'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py", '"faster": _Step(1, "Говорю быстрее", "Скорость уже максимальная")'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py", 'message=f"{step.said}: {describe(applied)}"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/animation.py",
     'self.log_info(f"Принято {data} (показывает диалог по ExpressionRequest)")'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/animation.py", 'data["animation"] = resolved'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py", 'self.log_info(f"[stop_music] was_playing={was_playing}")'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/player_owner.py", 'f"🎵 [music v2] stop track_id={last} reason={reason}"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/gen/cover.py", 'f"🎼 [cover] баланс KIE: {value if value is not None'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/base.py", 'self.node.get_logger().info(f"[{self.name}] {message}")'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py",
     'f"❌ Инструмент {tool_name} завершился с ошибкой: {result.error}"'),
    # #4062: текст вопроса - одна фраза каталога (её берут и тул через media_phrases, и диалог v2 по причине)
    ("src/rob_box_dialog/rob_box_dialog/phrases/ru.yaml",
     'music.library_fail.needs_confirmation: ["Удалить {name}? Скажи «да, удаляй»."]'),
    ("src/rob_box_voice/rob_box_voice/core/media_phrases.py",
     'return _phrase("music.library_fail.needs_confirmation", name=f"«{title}»" if title else LIBRARY_UNNAMED)'),
    ("src/rob_box_dialog/rob_box_dialog/knowledge.py", 'LIBRARY_UNNAMED = "этот трек"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/gen_library.py",
     "return _refused(NEEDS_CONFIRMATION, library_delete_question_text(title), track_id=track_id, title=title)"),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/gen_library.py", 'NEEDS_CONFIRMATION = "needs_confirmation"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/gen_library.py",
     'confirm is True or str(confirm).lower() == "true"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py",
     'f"📥 Запрос выполнения: {tool_name} с параметрами {parameters}"'),
    # #4077: вилка пения -- путь заказа, провал песни и трек библиотеки на деке
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/source_route.py",
     'f"🎚️ [source] {ask.describe()} кандидаты={listed} → {route.id} ({route.provider or \'code\'}) "'),
    ("src/rob_box_music/rob_box_music/source.py", "asked=[{','.join(flags)}] genre="),
    ("src/rob_box_music/rob_box_music/source.py", 'Option(f"{SYNTH}:{key}"'),
    ("src/rob_box_music/rob_box_music/source.py", 'Option(f"{GENERATE}:{kind}:{kn.DEFAULT_GEN_MODEL}"'),
    ("src/rob_box_music/rob_box_music/source.py", 'Option(f"{DECLINE}:{reason or \'no_source\'}"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/gen/song.py", 'f"🎤 [song] не вышло: {exc}"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/gen/song.py", 'f"🎤 [song] сбой: {type(exc).__name__}: {exc}"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/gen/song.py", 'f"🎤 [song] не заиграла: '),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/player_owner.py", 'f"🎵 [music v2] exec track_id={program.track_id} deck='),
])
def test_marker_sources_exist_in_robot_code(rel, fragment):
    """Паттерны генератора собраны из f-строк кода робота: поменяли строку -- этот тест краснеет раньше марафона."""
    assert fragment in _src(rel), rel


def test_prosody_and_emotion_tables_match_robot_knowledge():
    """Оси скорости/тона и сцены эмоций -- одна таблица в коде робота; генератор с ней сверяется."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "tts_prosody_for_gen", REPO_ROOT / "src/rob_box_voice/rob_box_voice/tts_prosody.py")
    prosody = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = prosody  # dataclass с отложенными аннотациями ищет свой модуль в sys.modules
    try:
        spec.loader.exec_module(prosody)
    finally:
        sys.modules.pop(spec.name, None)
    assert set(gen.SPEED_PARAMS) == {axis.param for axis in prosody.SPEED_AXES.values()}
    assert gen.PITCH_PARAM == prosody.PITCH_AXIS.param
    assert prosody.SPEED_AXES["minimax"].normal == 1.0 and prosody.PITCH_AXIS.normal == 1.0
    knowledge = _src("src/rob_box_dialog/rob_box_dialog/knowledge.py")
    for scene in ('"sad": "sad"', '"happy": "happy"', '"victory": "victory"'):
        assert scene in knowledge, scene
