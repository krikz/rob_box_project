# -*- coding: utf-8 -*-
"""#4077 М9: n802 ложный PASS -- счёт урезан до десяти, харнесс ждал только SpeechEvent started."""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "e2e"))
sys.path.insert(0, str(REPO_ROOT / "scripts" / "ci" / "e2e"))

import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

P = "[tts_node-5] [INFO] [1790801815.0]: "
T = "[dialog_node-3] [INFO] [1790801815.0]: "


def ev(kind: str, sid: str, tail: str = "") -> str:
    return f"{P}🗣️ SpeechEvent {kind} speech_id={sid} agent=tars epoch=1 text_len=80 {tail}".rstrip()


def tts(sid: str, text: str) -> str:
    return f"{P}🔊 TTS: speech_id={sid}, dialogue_id=None, batch=None 1/1, voice=default, lang=default, text={text!r}"


def log(*lines: str) -> str:
    return "\n".join(["✅ ПРИНЯТО: робот посчитай вслух от одного до сорока", *lines])


N802 = {"expected_tool_calls": [], "must_not_call": [], "min_count_end": 20}


def test_m9_cut_to_ten_is_red():
    """М9 raw: «Считаю, Саша. Один… Десять.» + SpeechEvent started -- раньше PASS, теперь FAIL."""
    text = log(
        tts("s1", "Считаю, Саша. Один. Два. Три. Четыре. Пять. Шесть. Семь. Восемь. Девять. Десять."),
        ev("started", "s1"),
    )
    assert etm.count_reach(text) == 10
    verdict = etm.step_verdict(N802, text)
    assert not verdict["pass"], verdict
    assert verdict["count_reach"] == 10
    assert any("count too short" in r for r in [verdict["reason"]])


def test_code_poem_four_segments_is_green():
    """counting.full_count на «до сорока» -- poem segments=4; barge-in режет окно на первом сегменте."""
    text = log(
        f"{T}🧭 [turn-v2] выступление старт kind=poem backing=- segments=4 → {{'status': 'pending'}}",
        tts("s1", "Один. Два. Три. Четыре. Пять. Шесть. Семь. Восемь. Девять. Десять."),
        ev("started", "s1"),
    )
    assert etm.count_reach(text) == 40
    verdict = etm.step_verdict(N802, text)
    assert verdict["pass"], verdict["reason"]
    assert verdict["count_reach"] == 40


def test_speech_past_twenty_is_green_without_perform():
    text = log(
        tts("s1", "Один. Пять. Десять. Пятнадцать. Двадцать. Двадцать один."),
        ev("started", "s1"),
    )
    assert etm.count_reach(text) >= 20
    assert etm.step_verdict(N802, text)["pass"]


def test_soroka_bound_in_promise_is_not_reach_forty():
    """«Считаю до сорока. Один… Десять.» -- «сорока» родительный, не достижение 40."""
    text = log(tts("s1", "Считаю до сорока. Один. Десять."), ev("started", "s1"))
    assert etm.count_reach(text) == 10
    assert not etm.step_verdict(N802, text)["pass"]


def test_flag_off_keeps_old_green():
    text = log(tts("s1", "Один. Десять."), ev("started", "s1"))
    assert etm.step_verdict({"expected_tool_calls": [], "must_not_call": []}, text)["pass"]


def test_n802_and_n806_declare_min_count_end():
    steps = {s["label"]: s for a in gen.ACTS if a["n"] == 8 for s in a["steps"]}
    for label in ("n802_long_count", "n806_long_count_again"):
        assert steps[label]["acceptance"].get("min_count_end") == 20, label
