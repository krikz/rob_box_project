"""Таблица вейков из профиля робота даёт то же поведение правил, что вейки ядра по умолчанию (ADR-0167 Ш1б-2).

Хосты робота теперь передают таблицу профиля явно (``Session(wake_words=...)``, ``dialog_node._wake``); до шага
правила брали ``DEFAULT_WAKE_WORDS`` ядра молча. На корпусе реплик робота ответы правил с таблицей профиля и с
умолчанием совпадают - то есть перенос вейков в профиль поведение робота не меняет.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rob_box_dialog import grammar, interrupt, reply_grammar, tool_ask, track_ref
from rob_box_dialog import words as w
from rob_box_dialog.profile import load_profile
from rob_box_dialog.session import Session

REPO = Path(__file__).resolve().parents[3]
ROBOT_PROFILE = REPO / "docker" / "sound" / "config" / "assistant.yaml"

PHRASES = (
    "Робот, стоп",
    "Стоп, Робот, пожалуйста",
    "робот хватит читать а музыку оставь",
    "Роббокс, замолчи",
    "Робот, громче",
    "Робот, да",
    "Робот, нет",
    "Робот, меня зовут Гриша",
    "Робот, зови меня Тамарой",
    "Робот, это я",
    "мы разные",
    "Робот, привет",
    "Робот, а потом включи музыку",
    "Робот, стой",
    "Робот, который час?",
    "Робот, включи эту песню",
    "Робби, добрый вечер",
    "робокс тише",
    "работа тишина",
    "Робот, покекай роботом",
    "который час",
    "привет",
    "",
)

pytestmark = pytest.mark.skipif(not ROBOT_PROFILE.is_file(), reason="нет docker/ (окружение без профиля робота)")


@pytest.fixture(scope="module")
def robot_wake():
    return load_profile(ROBOT_PROFILE).wake_personality


def _rules(text, snap, wake):
    tier1 = grammar.match(text, snap)
    return (
        None if tier1 is None else (tier1.kind, tier1.disputed, repr(dict(tier1.args)), tier1.rest),
        w.has_wake_word(text, wake),
        w.strip_wake_word(text, wake),
        w.stop_verdict(text, wake),
        w.policy_word(text, wake),
        w.wake_command(text, wake),
        w.asks_question(text, wake),
        w.asks_robot_to_act(text, wake),
        w.slipped_wake_rest(text, wake),
        reply_grammar.classify_reply(text, wake_words=wake),
        reply_grammar.classify_same(text, wake_words=wake),
        reply_grammar.classify_rename(text, wake_words=wake),
        track_ref.names_generated_topic(text, ("песн",), wake),
        interrupt.on_interrupt(text, snap, wake).policy,
    )


@pytest.mark.parametrize("text", PHRASES)
def test_profile_table_matches_core_default(text, robot_wake):
    with_profile = Session("personality", clock=lambda: 0.0, wake_words=robot_wake).snapshot
    by_default = Session("personality", clock=lambda: 0.0).snapshot
    assert with_profile.wake_words == tuple(robot_wake)
    assert _rules(text, with_profile, robot_wake) == _rules(text, by_default, None)


def test_tool_question_answer_reads_the_session_table(robot_wake):
    question = next(iter(tool_ask.kn.TOOL_QUESTIONS.values()))
    for text in ("Робот, да", "Робот, нет", "да", "Роббокс, давай"):
        assert tool_ask.answer(text, question, robot_wake) == tool_ask.answer(text, question)
