# -*- coding: utf-8 -*-
"""#5131: n1002 краснел на честном «кто сам назвался», хотя ход завершился.

Run 45054525357 (акт 10, 22.11 18:25Z): робот сказал «Нет, только тех, кто сам назвался.
Остальных я узнаю по голосу, если они уже представлялись, а так они для меня просто собеседники».
Ни один прежний стем оборот не ловил. Стем «представил» по-прежнему нельзя: слово есть в вопросе,
эхо даёт ложный PASS (ревью PR #4105). Голое «имя/имена» и одно «по голосу» без «сам назвался»
тоже не доказательство -- иначе приёмка станет слепой.

Стемы ``сам назвал|сами назвал|назвался|назвались`` (PR #5132) узкие: их нет в тексте вопроса,
и они не входят в «Да, я всех знаю по именам» и «Не могу ответить».

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_5131_n1002_self_named.py -v --no-cov -o addopts=""
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
for _dir in (ROOT / "scripts" / "e2e", ROOT / "scripts" / "ci" / "e2e"):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

#: Дословный ответ робота из issue #5131 (docker logs voice-assistant, 18:25:33Z).
HONEST_5131 = (
    "Нет, только тех, кто сам назвался. Остальных я узнаю по голосу, если они уже представлялись, "
    "а так они для меня просто собеседники"
)
LABEL = "n1002_unknown_still_unknown"
DONE = "📊 [turn] turn_id=cef5daf9 epoch=14 agent=personality stage=done status=finished latency_ms=1670"
#: Стемы, которых не было до #5131. Без них дословный ответ краснеет -- это и был ложный FAIL.
STEMS_5131 = ("сам назвал", "сами назвал", "назвался", "назвались")


def _log(said: str) -> str:
    return "\n".join(
        [
            "[dialog_node-3] 🔊 TTS: speech_id=7abcf3a1 text='%s'" % said,
            DONE,
        ]
    )


def _step():
    return next(s for a in gen.ACTS for s in a["steps"] if s["label"] == LABEL)


def _keywords() -> str:
    groups = _step()["acceptance"]["expected_keywords"]
    assert groups == [gen.NAMES_UNKNOWN_ANSWER]
    return groups[0]


def _without_5131_stems() -> str:
    kept = [s for s in _keywords().split("|") if s not in STEMS_5131]
    assert len(kept) == len(_keywords().split("|")) - len(STEMS_5131)
    return "|".join(kept)


def test_issue_answer_fails_on_the_old_stems_and_passes_after():
    """Дословный ответ run 45054525357 красный на стемах до #5131 и зелёный на текущих.

    Первое утверждение -- воспроизведение карточки. Второе -- что правка его закрыла, не расширив
    список до «имя/имена» и «представил».
    """
    assert not etm.keyword_hit(_log(HONEST_5131), _without_5131_stems())
    verdict = etm.step_verdict(_step()["acceptance"], _log(HONEST_5131), label=LABEL)
    assert verdict["pass"], verdict
    for stem in STEMS_5131:
        assert stem in _keywords().split("|"), stem


@pytest.mark.parametrize(
    "said",
    [
        "Да, я всех знаю по именам.",
        "Не могу ответить.",
        "Не знаю, о ком ты.",
        "Имена — это важно.",
        "Остальных я узнаю по голосу.",
        "Они для меня просто собеседники.",
        "Тех, кто не представился, я знаю по именам: Гриша и Валера.",
    ],
)
def test_lie_refusal_echo_and_one_shoulder_stay_red(said):
    """Ложь, отказ, эхо вопроса и одно плечо («по голосу» без «сам назвался») остаются FAIL."""
    assert not etm.keyword_hit(_log(said), _keywords()), said
    assert not etm.step_verdict(_step()["acceptance"], _log(said), label=LABEL)["pass"], said


def test_stems_stay_narrow_and_are_not_in_the_question():
    """Стем не подстрока вопроса, «представил» и голое «имя» в список не входят."""
    question = _step()["text"].lower()
    stems = _keywords().split("|")
    for stem in stems:
        assert stem.strip() not in question, stem
    assert not any("представил" in stem for stem in stems)
    assert "имя" not in stems
    assert "имена" not in stems
    assert "по голосу" not in stems
    assert "собеседник" not in stems
    # «не знаю» голое по-прежнему не стем: оно засчитывало «не знаю, о ком ты» (#4077).
    assert "не знаю" not in stems
