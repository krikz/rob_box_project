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
    "n702_generate_raccoon", "n703_merge_komar", "n708_gen_play", "n709_stop_generated", "n710b_gen_delete_confirm",
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
        assert any(p.startswith("✅ Инструмент ") and p.endswith(" выполнен успешно") for p in pats) \
            or label == "n703_merge_komar", label
        assert gen.TURN_DONE in pats or gen.TURN_WITH_LLM in pats, label
        assert gen.TURN_TIMEOUT in step["acceptance"]["must_not_call"], label
    assert gen.MUSIC_STARTED in _patterns("n502_start_renardo")
    assert gen.MUSIC_STARTED in _patterns("n602_dj_on")
    assert gen.MUSIC_STARTED in _patterns("n809_music_bed")
    assert gen.MUSIC_STARTED in _patterns("n711_renardo_not_broken")
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
    assert _verdict("n703_merge_komar", TURN_LINE % ("timeout", "failed"))["pass"] is False


def test_tool_failure_is_not_a_result():
    """Тул вызван, но завершился ошибкой (``❌ Инструмент … завершился с ошибкой``) -- не результат."""
    log = "\n".join(["📤 Отправлен запрос ab: request_music",
                     "❌ Инструмент request_music завершился с ошибкой: boom", TURN_LINE % ("done", "finished")])
    assert not _verdict("n504_add_bass", log)["pass"]


def test_live_music_edit_accepts_v2_tools_and_n506_forbids_voice_speed():
    """n504-n506: честная попытка -- dj_set|request_music|generate_music; «ускорь темп» не крутит голос робота (#3976)."""
    ended = TURN_LINE % ("done", "finished")
    for tool in gen.MUSIC_EDIT_TOOLS:
        ok = "\n".join(["✅ Инструмент %s выполнен успешно" % tool, ended])
        for label in ("n504_add_bass", "n505_vibe_dark", "n506_faster"):
            assert _verdict(label, ok)["pass"], (label, tool)
    forbidden = _acc("n506_faster")["must_not_call"]
    assert "set_speed" in forbidden and "set_pitch" in forbidden
    voice_instead = "\n".join(["📤 Отправлен запрос ab: set_speed", "✅ Инструмент set_speed выполнен успешно", ended])
    assert not _verdict("n506_faster", voice_instead)["pass"]
    voice_and_music = "\n".join(["tool_calls=(ToolCall(name='set_speed'),)",
                                 "✅ Инструмент dj_set выполнен успешно", ended])
    assert not _verdict("n506_faster", voice_and_music)["pass"]


def test_n702_is_marked_known_red_under_kie_quota():
    """Харнесс не умеет условие по ``<providers>`` (дамп запроса вырезается): n702 помечен, а не взломан."""
    step = _steps()["n702_generate_raccoon"]
    assert "KIE" in step["_known_red"]
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
    for label in MUSIC_MUST_START_LABELS:
        assert gen.MUSIC_NOT_STARTED in _acc(label)["must_not_say"], label
    assert gen.MUSIC_NOT_STARTED not in (_acc("n605_dj_set_plan").get("must_not_say") or []), "#4006 ещё не влит"
    ok = "\n".join(["✅ Инструмент request_music выполнен успешно", "🎵 [music v2] started track_id=req1:01:A", DONE])
    assert _verdict("n809_music_bed", ok)["pass"]
    lie = "\n".join([_tts("Музыка пока не заиграла."), "✅ Инструмент request_music выполнен успешно",
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
    saved = "\n".join([_speaker("Саша", 0.93), "✅ Инструмент memory_save выполнен успешно", DONE])
    assert not _verdict("n202a_sasha_warmup", saved)["pass"], "незапрошенный memory_save из бэклога"
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
    leak = _tts("Ты спрашивал про тех двоих, что ворчали на фоне.")
    assert not _verdict("n1007_no_leak", "\n".join([leak, DONE]))["pass"]
    assert not _acc("n1007_no_leak")["must_not_call"], "подстрока по всему логу ловила факты памяти"


def test_kie_quota_rule_marks_song_steps_and_n710_targets_this_runs_song():
    """R21: при балансе KIE 0 песни прогона нет -- n706-n710 помечены SKIP KIE_QUOTA, delete не «последний трек»."""
    import re

    steps = _steps()
    for label in ("n706_gen_save_for_boris", "n707_gen_track_info", "n708_gen_play", "n709_stop_generated",
                  "n710_gen_delete", "n710b_gen_delete_confirm"):
        # ревью #4046: харнесс поле читает (e2e_events.skip_rule) -- без «_», как остальные поля для харнесса
        assert steps[label]["skip_when"] == {"reason": "KIE_QUOTA", "log": gen.KIE_QUOTA_LOG}, label
        assert eev.skip_rule(steps[label]).reason == "KIE_QUOTA", label
    for label in ("n701_silence_check", "n702_generate_raccoon", "n711_renardo_not_broken"):
        assert "skip_when" not in steps[label], label
    assert not any("_skip_when" in s for s in steps.values())
    zero = "[mcp_server]: 🎼 [cover] баланс KIE: 0.0"
    assert re.search(gen.KIE_QUOTA_LOG, zero) and re.search(gen.KIE_QUOTA_LOG, "🎼 [cover] баланс KIE: 0")
    for funded in ("🎼 [cover] баланс KIE: 1712", "🎼 [cover] баланс KIE: 0.5", "🎼 [cover] баланс KIE: неизвестен"):
        assert not re.search(gen.KIE_QUOTA_LOG, funded), funded
    n710 = steps["n710_gen_delete"]
    assert "енот" in n710["text"] and "последний" not in n710["text"]
    search_only = "📥 Запрос выполнения: music_library с параметрами {'action': 'search', 'query': 'енот'}"
    searched = "\n".join([search_only, "✅ Инструмент music_library выполнен успешно", DONE])
    assert not _verdict("n710_gen_delete", searched)["pass"]
    assert not _verdict("n710b_gen_delete_confirm", searched)["pass"]


# ── #4025 (PR #4056): удаление трека в два шага - вопрос, затем «да» ─────────────────────────────────────────

QUESTION = "Удалить «Енот-мастер»? Скажи «да, удаляй»."
DELETE_ASK = "📥 Запрос выполнения: music_library с параметрами {'action': 'delete', 'track_id': 'a1b2'}"
DELETE_CONFIRM = ("📥 Запрос выполнения: music_library с параметрами "
                  "{'action': 'delete', 'track_id': 'a1b2', 'confirm': True}")
REFUSED = ("[dialog_node-3] [2026-10-24 01:39:22 - rob_box_dialog.execute:231 - INFO] 📨 [execute] событие "
           "music_library t5/0 rejected needs_confirmation ход_ждёт=True запрос=True")
TOOL_OK = "[mcp_server-9] [INFO] [1792795162.34] [mcp_server]: ✅ Инструмент music_library выполнен успешно"


def test_n710_passes_only_when_the_tool_asked_and_the_robot_said_the_question():
    """Шаг 1: PASS = вызов delete + отказ владельца ``needs_confirmation`` + вопрос с названием трека в речи."""
    asked = [DELETE_ASK, REFUSED, _tts(QUESTION), DONE]
    assert _verdict("n710_gen_delete", "\n".join(asked))["pass"]
    # трек стёрт без вопроса (успех тула + «Удалил»): красный, хотя вызов delete и конец хода на месте
    erased = [DELETE_ASK, TOOL_OK, _tts("Удалил «Енот-мастер» из библиотеки."), DONE]
    assert not _verdict("n710_gen_delete", "\n".join(erased))["pass"]
    # тул отказал, но человек услышал общее «не вышло» (нет фразы по причине): красный
    general = [DELETE_ASK, REFUSED, _tts("С библиотекой музыки не вышло."), DONE]
    assert not _verdict("n710_gen_delete", "\n".join(general))["pass"]
    # робот сам сказал вопрос, а отказа владельца в логе нет (тул не спрашивал): красный
    assert not _verdict("n710_gen_delete", "\n".join([DELETE_ASK, _tts(QUESTION), DONE]))["pass"]
    # вопрос без вызова delete (болтовня) и отказ без слов: красные
    assert not _verdict("n710_gen_delete", "\n".join([REFUSED, _tts(QUESTION), DONE]))["pass"]
    assert not _verdict("n710_gen_delete", "\n".join([DELETE_ASK, REFUSED, DONE]))["pass"]


def test_n710_checks_the_refusal_and_the_question_and_never_the_tool_success():
    step = _steps()["n710_gen_delete"]
    acc = step["acceptance"]
    assert acc["expected_keywords"] == ["удалить", "да, удаляй"] and acc["must_not_say"] == ["удалил"]
    assert step["_result_markers"] == {"tools": [], "music": ""}
    assert gen.tool_refused("music_library", "needs_confirmation") in step["patterns"]
    assert not any(p.startswith("✅ Инструмент ") for p in step["patterns"]), "успех тула здесь - стёртый трек"


def test_n710b_passes_only_when_yes_became_a_confirmed_delete():
    """Шаг 2: PASS = ``delete`` с ``confirm`` + успех тула + «Удалил» из результата; повторный вопрос - красный."""
    deleted = _tts("Удалил «Енот-мастер» из библиотеки.")
    assert _verdict("n710b_gen_delete_confirm", "\n".join([DELETE_CONFIRM, TOOL_OK, deleted, DONE]))["pass"]
    reordered = ("📥 Запрос выполнения: music_library с параметрами "
                 "{'confirm': 'true', 'track_id': 'a1b2', 'action': 'delete'}")
    assert _verdict("n710b_gen_delete_confirm", "\n".join([reordered, TOOL_OK, deleted, DONE]))["pass"]
    # delete без confirm (модель не поняла «да»): красный, даже с чужим успехом и словом «Удалил» в речи
    assert not _verdict("n710b_gen_delete_confirm", "\n".join([DELETE_ASK, TOOL_OK, deleted, DONE]))["pass"]
    # confirm был, но тул снова спросил: трек цел, красный
    again = [DELETE_CONFIRM, REFUSED, _tts(QUESTION), DONE]
    assert not _verdict("n710b_gen_delete_confirm", "\n".join(again))["pass"]
    # confirm был, тул снова спросил, а успех - от соседнего поиска и «Удалил» сказал сам робот: красный (must_not)
    lying = [DELETE_CONFIRM, REFUSED, TOOL_OK, deleted, DONE]
    assert not _verdict("n710b_gen_delete_confirm", "\n".join(lying))["pass"]
    # поиск вместо удаления
    searching = DELETE_ASK.replace("'delete'", "'search'")
    assert not _verdict("n710b_gen_delete_confirm", "\n".join([searching, TOOL_OK, deleted, DONE]))["pass"]


def test_n710b_follows_the_question_and_retries_once_while_the_question_does_not():
    """Повтор «да, удаляй» безопасен (подтверждение засчитывается, только если тул спрашивал про этот трек);
    вопрос n710 повторять незачем."""
    steps = _steps()
    assert steps["n710b_gen_delete_confirm"]["retry_acceptance"] == 1
    assert "retry_acceptance" not in steps["n710_gen_delete"]
    order = [s["label"] for s in gen.ACTS[6]["steps"]]
    assert order.index("n710b_gen_delete_confirm") == order.index("n710_gen_delete") + 1, "«да» идёт сразу за вопросом"


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
    ("src/rob_box_dialog/rob_box_dialog/execute.py",
     "f\"📨 [execute] событие {request.tool if request else '?'} {event.ref} {event.status} \""),
    ("src/rob_box_dialog/rob_box_dialog/execute.py", "f\"{event.reason or '-'} ход_ждёт="),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/gen_library.py", 'NEEDS_CONFIRMATION = "needs_confirmation"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/gen_library.py",
     'confirm is True or str(confirm).lower() == "true"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py",
     'f"📥 Запрос выполнения: {tool_name} с параметрами {parameters}"'),
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
