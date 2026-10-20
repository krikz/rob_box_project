"""#3599: «продолжай играть Вагнера» / «ещё Баха» — грамматика и поток заказа по имени, без LLM."""

from __future__ import annotations

import asyncio
from typing import Any, Dict, List, Tuple

import pytest

from rob_box_voice.core.media_command_grammar import MORE_WORDS, SET_CONTINUE_WORDS, MediaIntent, parse_media_command
from rob_box_voice.core.media_phrases import no_more_error, no_more_text
from rob_box_voice.core.media_router import MediaRouter, MediaState
from rob_box_voice.core.named_play import NamedPlayStatus, outcome_text, run_named_play


@pytest.mark.parametrize("text,name", [
    ("продолжай играть вагнера", "вагнера"),
    ("Робот, продолжай играть Вагнера", "вагнера"),
    ("ещё Вагнера", "вагнера"),
    ("давай ещё Баха", "баха"),
    ("еще баха", "баха"),
    ("дальше моцарта", "моцарта"),
    ("сыграй ещё Баха", "баха"),
])
def test_more_of_the_composer_is_play_named_with_the_flag(text, name):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.PLAY_NAMED and command.more and command.closed and command.name == name


@pytest.mark.parametrize("text", [
    "продолжай", "ещё", "давай ещё", "дальше", "ещё раз калинку", "повтори вагнера", "сыграй вагнера",
    "ещё музыку", "сыграй ещё громче", "продолжай сэт",
])
def test_not_a_more_command(text):
    assert not parse_media_command(text).more


def test_one_table_of_continue_words():
    assert MORE_WORDS == SET_CONTINUE_WORDS - {"повтори"}


def test_router_sends_it_to_the_named_play_flow_with_the_flag():
    plan = MediaRouter().route("продолжай играть вагнера", MediaState())
    assert plan is not None and plan.play_name == "вагнера" and plan.command.more and plan.tool_calls == ()


class _Tools:
    def __init__(self, answer: Tuple[bool, str]) -> None:
        self.answer = answer
        self.calls: List[Tuple[str, Dict[str, Any]]] = []

    async def __call__(self, name: str, args: Dict[str, Any]) -> Tuple[bool, str]:
        self.calls.append((name, args))
        return self.answer


def _run(tools, name, more):
    return asyncio.new_event_loop().run_until_complete(run_named_play(tools, name, more))


def test_more_skips_the_title_lookup_and_plays_via_request_music():
    played = {"title": "Jesu Joy", "composer": "Бах", "source": "rtttl", "replacement_for": None}
    tools = _Tools((True, repr({"ok": True, "melody_id": "jesujoyo", "played": played})))
    out = _run(tools, "баха", True)
    assert out.status is NamedPlayStatus.PLAYED and out.hit.key == "jesujoyo" and out.tools_done == ("request_music",)
    assert tools.calls == [("request_music", {"intent": "melody", "text": "баха"})]
    assert outcome_text(out, "Баха") == "Ставлю «Jesu Joy», Бах — мелодией, нот для полной аранжировки нет."


def test_no_more_is_spoken_by_code_not_by_the_llm():
    phrase = no_more_text("вагнера", "Ride Of The Valkyries")
    out = _run(_Tools((False, no_more_error(phrase))), "вагнера", True)
    assert out.status is NamedPlayStatus.NO_MORE and outcome_text(out, "Вагнера") == phrase
    assert phrase == ("По запросу «вагнера» новых пьес больше нет, всё уже играло. "
                      "Могу повторить «Ride Of The Valkyries» или сыграть что-то другое.")


def test_any_other_failure_of_a_more_request_goes_to_the_llm():
    out = _run(_Tools((False, "мелодия не заиграла: not_found")), "вагнера", True)
    assert out.status is NamedPlayStatus.MISS


def test_a_plain_order_does_not_set_more():
    calls: List[str] = []

    async def execute(name, args):
        calls.append(name)
        return False, "не найдена"

    out = asyncio.new_event_loop().run_until_complete(run_named_play(execute, "вагнера"))
    assert out.status is NamedPlayStatus.MISS and calls == ["lookup_melody"]
