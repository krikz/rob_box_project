# -*- coding: utf-8 -*-
"""#4985 / #4986 (акт 8 марафона 20.11, run 44808584641): окно счёта судит свой план, а не филлер и не чужой шаг.

* #4985 n806: acceptance снят по филлеру «Секунду, думаю.» за 2,3 с до плана счёта. Окно, где есть только
  промежуточная фраза и нет собственного плана, -- «счёт ещё не начался», а не «обрыв на нуле».
* #4986 n807: окно шага с barge-in захватило счёт предыдущего шага, а регэксп резал «двадцать один» на 20 и 1.
  Составное -- одно число; речь до собственного ``ПРИНЯТО`` шага принадлежит предыдущему шагу.

Строки -- синтетика по форме лога робота из карточек, не цитаты людей.
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


# Форма строки из карточки #4985: филлер хода, затем план счёта.
FILLER = joined(
    f"{D}🧭 [turn-v2] промежуточная фраза: Секунду, думаю.",
    tts("ce9f33bd", "Секунду, думаю."),
    ev("started", "ce9f33bd"),
)
PLAN = joined(
    f"{D}🧭 [turn-v2] выступление старт kind=poem backing=none segments=5 → {{'status': 'pending'}}",
    tts("be102897", "Один, два, три, четыре, пять, шесть, семь, восемь, девять, десять."),
    ev("started", "be102897"),
)

N806 = {"expected_tool_calls": [], "must_not_call": [], "min_count_end": 20}


def test_filler_only_window_is_not_a_silent_cut():
    """#4985: до плана в окне один филлер. Раньше count_reach=0 и «silent cut», теперь судить нечего."""
    assert etm.filler_speech_ids(FILLER) == {"ce9f33bd"}
    assert etm.count_reach(FILLER) == 0
    assert not etm.count_plan_started(FILLER)
    verdict = etm.step_verdict(N806, FILLER)
    assert verdict["pass"], verdict["reason"]
    assert verdict["count_end_failures"] == []


def test_real_cut_after_the_plan_stays_red():
    """Настоящий обрыв: план начался и дошёл только до десяти -- по-прежнему красный (#4077)."""
    text = joined(
        FILLER, tts("s1", "Один, два, три, четыре, пять, шесть, семь, восемь, девять, десять."), ev("started", "s1")
    )
    assert etm.count_plan_started(text)
    assert etm.count_reach(text) == 10
    verdict = etm.step_verdict(N806, text)
    assert not verdict["pass"]
    assert any("count reaches 10" in item for item in verdict["count_end_failures"])


def test_plan_past_twenty_is_green_even_with_a_filler():
    text = joined(FILLER, PLAN)
    assert etm.count_reach(text) == 50  # segments=5 × 10, филлер в план не входит
    assert etm.step_verdict(N806, text)["pass"]


def test_filler_without_the_marker_is_still_judged():
    """Филлер, не помеченный строкой ``промежуточная фраза``, от речи не отличим -- окно судится как раньше."""
    bare = joined(tts("ce9f33bd", "Секунду, думаю."), ev("started", "ce9f33bd"))
    assert etm.filler_speech_ids(bare) == set()
    assert etm.count_plan_started(bare)
    assert etm.count_reach(bare) == 0
    assert not etm.step_verdict(N806, bare)["pass"]
