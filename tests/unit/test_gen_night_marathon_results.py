# -*- coding: utf-8 -*-
"""Карточка #3971: харнесс ночного марафона судит по РЕЗУЛЬТАТУ действия, а не по имени тула и не по фразе.

Разбор марафонов 23.10 (13:29 и 16:15), §1.A: v1-имена (``set_dj_mode``) и обязательный ``get_music_state`` краснили
верные ответы робота, а «фраза есть → PASS» пропускала пустые обещания. Тесты держат четыре инварианта SSoT
``scripts/e2e/gen_night_marathon.py``: нет v1-имён, нет обязательного ``get_music_state``, у шагов-действий есть
маркер результата, в акте 8 тишину снимает явное «Робот, говори». Сверка шагов с логом -- те же функции, что у
живого харнесса (``e2e_tool_match.step_verdict`` и ``patterns_missing``), второго судьи нет.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Dict, List

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "e2e"))
sys.path.insert(0, str(REPO_ROOT / ".github" / "workflows" / "scripts"))

import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

#: Шаги, которые просят ДЕЙСТВИЕ с музыкой: каждому обязан сопутствовать маркер результата.
ACTION_LABELS = {
    "n310_execute_backlog_lru", "n312_stop_music",
    "n502_start_renardo", "n504_add_bass", "n505_vibe_dark", "n506_faster", "n511_stop_music",
    "n602_dj_on", "n604_dj_rock", "n605_dj_set_plan", "n607_vibe_chill", "n608_dj_off",
    "n702_generate_raccoon", "n703_merge_komar", "n708_gen_play", "n709_stop_generated",
    "n711_renardo_not_broken", "n712_stop_all",
    "n809_music_bed", "n812_stop_music",
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
