"""«Найди похожих» и склейка в диалоге (issue #4469, ADR-0148): классы тулов, срез хода, подтверждение одной строкой через
окно ответа тула, фразы кода из данных владельца, коллаж уходит в чат отдельным выводом ``kind=photo``."""

from __future__ import annotations

import asyncio
import json

import pytest

from rob_box_core.tool_catalog import telegram_visible_tools
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases, tool_ask
from rob_box_dialog.address import SOURCE_TELEGRAM, VOICE
from rob_box_dialog.chats import PHOTO_KIND, ChatPorts
from rob_box_dialog.command import ToolCall
from rob_box_dialog.respond import ActionResult, asks, failed, phrase_from_result
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools, groups_for

SNAP = Session("telegram").snapshot
QUESTION = "Склеиваю 3 записи в «Денчик» (71cda108)? Скажи «да, склей»."  # литерал: ловит дрейф ru.yaml и тула
MERGE = ToolCall("merge_people", {"target": "71cda108", "numbers": [1, 3, 5]})
RULE = kn.TOOL_QUESTIONS[("merge_people", "needs_merge_confirmation")]


def _res(tool, status, data=None, reason=None):
    return ActionResult("t/0", 0, "telegram", tool, kn.TOOL_CLASSES[tool], status, reason, data or {})


def _asked():
    return _res("merge_people", "rejected", {"question": QUESTION, "count": 3}, "needs_merge_confirmation")


def test_new_tools_have_classes_and_are_in_the_chat_slice_only():
    assert kn.TOOL_CLASSES["find_similar"] == kn.TOOL_CLASSES["merge_people"] == "action.local"
    assert {"find_similar", "merge_people"} <= {e.name for e in telegram_visible_tools()}
    assert {"find_similar", "merge_people"} <= set(kn.TOOL_GROUPS[kn.FACES_GROUP].tools)
    voice_tools = {e.name for e in available_tools(SNAP, None, "personality", "найди похожих на 71cda108")}
    assert not {"find_similar", "merge_people"} & voice_tools, "голосу склейка и коллаж не даются"


@pytest.mark.parametrize("text", ["найди похожих на 71cda108", "найди дубли", "склей их"])
def test_the_request_opens_the_faces_group_for_the_chat_agent(text):
    names = {e.name for e in available_tools(SNAP, None, "telegram", text)}
    assert {"find_similar", "merge_people"} <= names


def test_numbers_without_words_keep_the_group_after_the_collage_turn():
    """«1, 3, 5 - да» слов группы не содержит: после хода с find_similar группа остаётся в срезе (carry)."""
    assert kn.FACES_GROUP in groups_for(SNAP, "1, 3, 5 - да", carry=(kn.FACES_GROUP,), agent="telegram")
    names = {e.name for e in available_tools(SNAP, None, "telegram", "1, 3, 5 - да", carry=(kn.FACES_GROUP,))}
    assert "merge_people" in names


def test_confirmation_is_one_line_built_by_code_and_is_not_a_failure():
    result = _asked()

    assert asks(result) and not failed(result)
    assert phrase_from_result(result).text == QUESTION


def test_yes_confirms_the_same_call_and_numbers_are_not_an_answer():
    session = Session("telegram")
    assert tool_ask.note(session, [MERGE], [_asked()]).tool == "merge_people"

    assert tool_ask.reply(session, "1, 3, 5 - да", SOURCE_TELEGRAM) is None, "номера идут к модели, а не в «да»"
    assert tool_ask.reply(session, "да", VOICE).yes is True
    confirmed = tool_ask.reply(session, "да, склей", SOURCE_TELEGRAM).asked.confirmed()
    assert confirmed.tool == "merge_people"
    assert dict(confirmed.args) == {"target": "71cda108", "numbers": [1, 3, 5], "confirm": True}


@pytest.mark.parametrize(
    "text, verdict",
    [
        ("да", True),
        ("да, склей", True),
        ("склей записи", True),
        ("нет", False),
        ("не надо", False),
        ("не склеивай", False),
        ("да, склей все", None),
        ("1 и 3", None),
        ("", None),
    ],
)
def test_answer_grammar_of_the_merge_question(text, verdict):
    assert tool_ask.answer(text, RULE) is verdict


def test_no_is_said_by_code_and_the_decline_phrase_exists():
    assert phrases.text(RULE.declined) == "Не склеиваю, записи остались как были."


def test_done_phrases_come_from_the_owners_facts():
    merged = _res("merge_people", "done", {"summary": "Склеено 3, теперь у Денчик 14 снимков."})
    similar = _res("find_similar", "done", {"summary": "Похожих на 71cda108: 5, коллаж выше. Назови номера."})

    assert phrase_from_result(merged).text == "Склеено 3, теперь у Денчик 14 снимков."
    assert phrase_from_result(similar).text == "Похожих на 71cda108: 5, коллаж выше. Назови номера."


@pytest.mark.parametrize(
    "tool, reason, fragment",
    [
        ("merge_people", "no_collage", "Коллажа нет"),
        ("merge_people", "collage_mismatch", "другое лицо"),
        ("merge_people", "bad_number", "Такого номера"),
        ("merge_people", "bad_selection", "Не понял выбор"),
        ("merge_people", "merge_failed", "не вышло"),
        ("merge_people", "same_person", "саму с собой"),
        ("merge_people", "merge_unconfirmed", "не подтвердила"),
        ("merge_people", "person_not_found", "в базе нет"),
        ("find_similar", "exhibition_mode", "режиме выставки"),
        ("find_similar", "collage_failed", "Коллаж собрать"),
        ("find_similar", "owner_unavailable", "не отвечает"),
    ],
)
def test_refusals_say_the_reason_and_are_failures(tool, reason, fragment):
    result = _res(tool, "rejected", {"reason": reason}, reason)

    assert failed(result) and fragment in phrase_from_result(result).text


# --- коллаж уходит в чат ----------------------------------------------------------------------------------------


class Base:
    def __init__(self, results):
        self.results = results

    async def execute(self, actions, snapshot, turn_id):
        return self.results

    def stage(self, *args, **kwargs):
        return None


def _ports(results):
    sent = []
    return (
        ChatPorts(Base(results), 7, lambda ref, text, sid, **extra: sent.append((ref, text, extra)), lambda line: None),
        sent,
    )


def _photos(sent):
    return [(ref, json.loads(text)) for ref, text, extra in sent if extra.get("kind") == PHOTO_KIND]


def test_collage_goes_to_the_chat_as_one_photo_output_marked_as_a_collage():
    ports, sent = _ports(
        [
            _res(
                "find_similar",
                "done",
                {
                    "id": "71cda108",
                    "photo_id": "71cda108",
                    "photo_kind": "collage",
                    "caption": "Похожие на 71cda108 (Денчик)",
                },
            )
        ]
    )

    asyncio.run(ports.execute([ToolCall("find_similar", {"id": "71cda108"})], SNAP, "t"))

    assert _photos(sent) == [
        ("telegram:7", {"id": "71cda108", "caption": "Похожие на 71cda108 (Денчик)", "kind": "collage"})
    ]


def test_a_plain_photo_payload_has_no_collage_mark_and_a_refused_search_sends_nothing():
    ports, sent = _ports(
        [
            _res("person_photo", "done", {"id": "fff371fe", "photo_id": "fff371fe", "caption": "c"}),
            _res("find_similar", "rejected", {"photo_id": "71cda108"}, "exhibition_mode"),
        ]
    )

    asyncio.run(
        ports.execute([ToolCall("person_photo", {"id": "fff371fe"}), ToolCall("find_similar", {"id": "x"})], SNAP, "t")
    )

    assert _photos(sent) == [("telegram:7", {"id": "fff371fe", "caption": "c"})]
