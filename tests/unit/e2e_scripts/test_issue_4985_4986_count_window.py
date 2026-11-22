"""#4986 (акт 8 марафона 20.11, run 44808584641): окно счёта судит свои числа, а не чужой шаг.

* Составное числительное -- одно число: «двадцать один» это 21, а не 20 и 1. Регэксп 1..20 резал его на два слова,
  и десять «двадцать» подряд давали ложный повтор и «20->20».
* Речь до собственного ``✅ ПРИНЯТО`` шага принадлежит предыдущему шагу: PLAY с barge-in ставится на 3 с раньше,
  и окно n807 захватило счёт n806 целиком.

Настоящий повтор после правки и пропуск в своих числах по-прежнему красные. Строки -- синтетика по форме лога
робота из карточки, не цитаты людей. Филлерный случай #4985 -- в test_issue_4985_count_filler.py.
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "ci" / "e2e"))

import e2e_tool_match as etm  # noqa: E402

D = "[dialog_node-3] [INFO] [1795205550.0]: "
P = "[tts_node-5] [INFO] [1795205550.0]: "


def tts(sid: str, text: str) -> str:
    return f"{P}🔊 TTS: speech_id={sid}, dialogue_id=None, batch=None 1/1, voice=default, lang=default, text={text!r}"


def ev(kind: str, sid: str, tail: str = "") -> str:
    return f"{P}🗣️ SpeechEvent {kind} speech_id={sid} agent=tars epoch=1 text_len=15 {tail}".rstrip()


def joined(*lines: str) -> str:
    return "\n".join(lines)


# Форма строки из карточки #4986: счёт предыдущего шага, затем своя фраза и продолжение.
COUNT_OF_PREVIOUS_STEP = joined(
    "✅ ПРИНЯТО: робот посчитай вслух от одного до сорока спокойно",
    tts("be102897", "Один, два, три, четыре, пять, шесть, семь, восемь, девять, десять."),
    ev("started", "be102897"),
    tts(
        "8dca8978",
        "Одиннадцать, двенадцать, тринадцать, четырнадцать, пятнадцать, шестнадцать, "
        "семнадцать, восемнадцать, девятнадцать, двадцать.",
    ),
    ev("started", "8dca8978"),
    tts(
        "8b0e1a46",
        "Двадцать один, двадцать два, двадцать три, двадцать четыре, двадцать пять, двадцать шесть, "
        "двадцать семь, двадцать восемь, двадцать девять, тридцать.",
    ),
    ev("started", "8b0e1a46"),
)
OWN_PHRASE = "✅ ПРИНЯТО: робот и после каждого числа добавляй слово шаг"
CONTINUATION = joined(
    ev("started", "25bdebaf"),
    tts("25bdebaf", "Тридцать один шаг, тридцать два шага, тридцать три шага."),
)

N807 = {"expected_tool_calls": [], "must_not_call": [], "no_repeat_numbers": True, "count_continues": True}


def test_compound_numerals_are_one_number():
    """«двадцать один» -- 21, не 20 и 1: десять «двадцать» больше не дают 20->20 и повтор «один»."""
    speech = "Двадцать один, двадцать два, двадцать три."
    assert etm._spoken_numbers(speech) == [21, 22, 23]
    text = joined(OWN_PHRASE, ev("started", "a"), tts("a", speech + " Двадцать четыре шага."))
    assert etm.repeated_numerals(text) == []
    assert etm.count_continuation_failures(text) == []


def test_previous_step_count_is_not_this_step_speech():
    """#4986: окно n807 захватило счёт n806. Числа 1..30 до собственного ПРИНЯТО -- не речь этого шага."""
    text = joined(COUNT_OF_PREVIOUS_STEP, OWN_PHRASE, CONTINUATION)
    assert etm.repeated_numerals(text) == []
    assert etm.count_continuation_failures(text) == []
    assert etm.step_verdict(N807, text)["pass"], etm.step_verdict(N807, text)["reason"]


def test_restart_after_the_edit_stays_red():
    """Настоящий повтор после правки ловится и на составных: 21, затем снова с единицы."""
    text = joined(COUNT_OF_PREVIOUS_STEP, OWN_PHRASE, ev("started", "b"), tts("b", "Один шаг, два шага."))
    repeated = etm.repeated_numerals(text)
    assert repeated == []  # повтор -- внутри своего окна, числа предыдущего шага не считаются
    failures = etm.count_continuation_failures(text)
    assert "count restarted from one after the edit" in failures
    assert not etm.step_verdict(N807, text)["pass"]


def test_repeat_inside_own_window_stays_red():
    """Повтор числа в собственной речи шага -- по-прежнему красный, отсечение чужого окна его не прячет."""
    text = joined(
        OWN_PHRASE,
        ev("started", "a"),
        tts("a", "Двадцать один шаг."),
        ev("started", "b"),
        tts("b", "Двадцать один шаг, двадцать два шага."),
    )
    assert "21 x2" in etm.repeated_numerals(text)
    assert not etm.step_verdict(N807, text)["pass"]


def test_single_accept_window_is_not_trimmed():
    """Окно с одним ПРИНЯТО -- своё целиком: barge-in чужой речи в него не занёс, резать нечего."""
    text = joined(OWN_PHRASE, ev("started", "a"), tts("a", "Девять."), ev("started", "b"), tts("b", "Девять шаг."))
    assert "9 x2" in etm.repeated_numerals(text)


def test_gap_in_compound_continuation_stays_red():
    """Пропуск в составных ловится: 21, затем сразу 23."""
    text = joined(OWN_PHRASE, ev("started", "a"), tts("a", "Двадцать один шаг, двадцать три шага."))
    failures = etm.count_continuation_failures(text)
    assert any("21->23" in item for item in failures)
