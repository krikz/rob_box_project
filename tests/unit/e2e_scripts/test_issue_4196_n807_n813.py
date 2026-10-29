# -*- coding: utf-8 -*-
"""#4196 (марафон-5, run 40801693422): n807 и n813 судят по сути, а не по артефакту харнесса.

* n807: ``SpeechEvent cancelled ... spoken_chars=0`` у реплики без ``started`` -- выброшенный пред-синтез при правке,
  не обрыв речи. Обрыв звучащей реплики по-прежнему красный. Повтор счёта (#4207) ловит ``no_repeat_numbers``.
* n813: фраза шага -- вопрос словами; ответ-команда «Молчу» красит шаг.
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "e2e"))
sys.path.insert(0, str(REPO_ROOT / ".github" / "workflows" / "scripts"))

import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

P = "[tts_node-5] [INFO] [1790801815.0]: "


def ev(kind: str, sid: str, tail: str = "") -> str:
    return f"{P}🗣️ SpeechEvent {kind} speech_id={sid} agent=tars epoch=1 text_len=93 {tail}".rstrip()


def tts(sid: str, text: str) -> str:
    return f"{P}🔊 TTS: speech_id={sid}, dialogue_id=None, batch=None 1/1, voice=default, lang=default, text={text!r}"


def log(*lines: str) -> str:
    return "\n".join(["✅ ПРИНЯТО: робот и после каждого числа добавляй слово шаг", *lines])


N807 = {"expected_tool_calls": [], "must_not_call": ["🛑 [interrupt]", "SpeechEvent cancelled"], "no_repeat_numbers": True}


def test_unspoken_presynth_cancel_is_not_a_failure():
    text = log(ev("queued", "ae101af6"), ev("cancelled", "ae101af6", "spoken_chars=0 reason=cancelled"))
    verdict = etm.step_verdict(N807, text)
    assert verdict["pass"], verdict["reason"]
    assert verdict["forbidden_calls"] == []


def test_cancel_of_a_sounding_segment_stays_red():
    for tail in ("spoken_chars=44 reason=stopped", "spoken_chars=0 reason=stopped"):
        text = log(ev("started", "91b3cb4b"), ev("cancelled", "91b3cb4b", tail))
        verdict = etm.step_verdict(N807, text)
        assert not verdict["pass"], tail
        assert verdict["forbidden_calls"] == ["SpeechEvent cancelled"], tail


def test_interrupt_decision_stays_red_even_with_unspoken_cancel():
    text = log(ev("cancelled", "ae101af6", "spoken_chars=0"), "🛑 [interrupt] stop via=interrupt_request")
    assert etm.step_verdict(N807, text)["forbidden_calls"] == ["🛑 [interrupt]"]


def test_drop_unspoken_cancels_keeps_other_lines():
    text = "\n".join([ev("started", "aaaaaaaa"), ev("cancelled", "bbbbbbbb", "spoken_chars=0"),
                      ev("cancelled", "aaaaaaaa", "spoken_chars=0"), ev("cancelled", "cccccccc", "spoken_chars=7")])
    kept = etm.drop_unspoken_cancels(text).split("\n")
    assert len(kept) == 3 and not any("bbbbbbbb" in line for line in kept)


def test_count_restart_after_edit_is_red_m5_shape():
    """Лог М5: 14-19 прозвучали, затем правка пошла с «девять» -- числа 14-19 дважды."""
    text = log(
        ev("started", "s4"), tts("s4", "Четырнадцать, пятнадцать, шестнадцать,"),
        ev("started", "s5old"), tts("s5old", "Семнадцать, восемнадцать, девятнадцать"),
        tts("ae101af6", "Один шаг, два шага, три шага, четыре шага, пять шагов, шесть шагов, семь шагов, восемь шагов."),
        ev("cancelled", "ae101af6", "spoken_chars=0 reason=cancelled"),
        ev("started", "s6"), tts("s6", "Девять шагов, десять шагов, одиннадцать шагов, двенадцать шагов, "
                                       "тринадцать шагов, четырнадцать шагов, пятнадцать шагов, шестнадцать шагов."),
        ev("started", "s7"), tts("s7", "Семнадцать шагов, восемнадцать шагов, девятнадцать шагов, двадцать шагов."),
    )
    verdict = etm.step_verdict(N807, text)
    assert not verdict["pass"]
    assert verdict["forbidden_calls"] == []  # отмена несказанного не причина
    assert "четырнадцать x2" in verdict["repeated_numbers"] and "девятнадцать x2" in verdict["repeated_numbers"]
    assert "один" not in " ".join(verdict["repeated_numbers"])  # несказанный пред-синтез не считается


def test_correct_continuation_is_green():
    text = log(
        ev("started", "s4"), tts("s4", "Четырнадцать, пятнадцать, шестнадцать,"),
        ev("started", "s5"), tts("s5", "Семнадцать шагов, восемнадцать шагов, девятнадцать шагов, двадцать шагов."),
        ev("finished", "s5", "spoken_chars=93"),
    )
    verdict = etm.step_verdict(N807, text)
    assert verdict["pass"], verdict["reason"]


def test_no_started_events_falls_back_to_robot_speech():
    text = log("🔊 TTS: speech_id=a1, text='Девять, десять'", "🔊 TTS: speech_id=a2, text='Девять, десять'")
    # без SpeechEvent started (v1/сухой лог) -- общий канал речи, повтор виден
    assert etm.repeated_numerals(text) == ["девять x2", "десять x2"]


def test_flag_off_does_not_check_numbers():
    text = log(ev("started", "a"), tts("a", "Девять, девять"))
    acc = {"expected_tool_calls": [], "must_not_call": []}
    assert etm.step_verdict(acc, text)["pass"]


def test_numeral_word_boundaries():
    text = log(ev("started", "a"), tts("a", "Опять одиночка, пять раз. Пять!"))
    assert etm.repeated_numerals(text) == ["пять x2"]


def test_n813_command_ack_is_not_an_answer_about_music():
    acc = {"expected_tool_calls": [], "must_not_call": [], "expected_keywords": [gen.SILENCE_STATE_ANSWER],
           "must_not_say": [gen.FALSE_FAILURE_PHRASE, gen.SILENCE_COMMAND_ACK]}
    ack = log(tts("a", "Молчу, Саша. Тишина так тишина."))
    verdict = etm.step_verdict(acc, ack)
    assert not verdict["pass"] and verdict["forbidden_said"] == [gen.SILENCE_COMMAND_ACK]
    honest = log(tts("b", "Сейчас ничего не играет, Саша."))
    assert etm.step_verdict(acc, honest)["pass"]
    command_tikho = log(tts("c", "Тихо, Саша. Молчу."))
    assert not etm.step_verdict(acc, command_tikho)["pass"]
