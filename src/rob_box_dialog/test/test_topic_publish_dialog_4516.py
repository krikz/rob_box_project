"""Публикация в топик в диалоге (issue #4516, шаг 2а, ADR-0148): класс тула, группа слов только у пульта TG и ТАРС,
подтверждение одной строкой через окно ответа тула, фразы кода из данных тула."""

from __future__ import annotations

import pytest

from rob_box_core.tool_catalog import telegram_visible_tools
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases, tool_ask
from rob_box_dialog.address import SOURCE_OPERATOR, SOURCE_TELEGRAM, VOICE
from rob_box_dialog.command import ToolCall
from rob_box_dialog.respond import ActionResult, asks, failed, phrase_from_result
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools, groups_for

# литерал: ловит дрейф ru.yaml и тула
QUESTION = 'Публикую в /animations/trigger (std_msgs/msg/String) {"data": "hi"}, 1 раз? Скажи «да, публикуй».'
CALL = ToolCall("topic_publish", {"topic": "/animations/trigger", "fields": {"data": "hi"}})
RULE = kn.TOOL_QUESTIONS[("topic_publish", "needs_publish_confirmation")]
PUBLISH_TEXTS = ["опубликуй в топик /animations/trigger", "отправь в топик /x сообщение", "публикация в /x"]


def _names(text, agent):
    return {t.name for t in available_tools(Session(agent).snapshot, None, agent, text)}


def _res(status, data=None, reason=None):
    return ActionResult(
        "t/0", 0, "telegram", "topic_publish", kn.TOOL_CLASSES["topic_publish"], status, reason, data or {}
    )


def _asked():
    return _res("rejected", {"question": QUESTION}, "needs_publish_confirmation")


def test_publish_is_an_action_not_a_query_and_in_the_chat_slice():
    assert kn.TOOL_CLASSES["topic_publish"] == "action.local"
    assert "topic_publish" in {e.name for e in telegram_visible_tools()}
    assert kn.TOOL_GROUPS["publish"].tools == ("topic_publish",)
    assert "publish" in kn.AGENT_SCOPED_GROUPS


@pytest.mark.parametrize("agent", ["telegram", "operator"])
@pytest.mark.parametrize("text", PUBLISH_TEXTS)
def test_publish_words_open_the_tool_for_tg_and_tars(agent, text):
    assert "topic_publish" in _names(text, agent)


@pytest.mark.parametrize("text", PUBLISH_TEXTS + ["что в топике /battery_state", "какие есть топики"])
def test_voice_never_gets_the_publish_tool(text):
    assert "topic_publish" not in _names(text, "personality")
    assert "publish" not in groups_for(Session("personality").snapshot, text, agent="personality")


@pytest.mark.parametrize("text", ["что в топике /battery_state", "какие есть топики", "померяй частоту /scan"])
def test_reading_words_do_not_open_the_publish_tool(text):
    assert "topic_publish" not in _names(text, "telegram")


def test_slice_stays_within_cap_when_everything_is_asked_at_once():
    for text in ("опубликуй в топик, покажи топики, vesc и логи", "частота топика /scan и опубликуй в топик /x"):
        assert len(_names(text, "telegram")) <= kn.TOOLS_VISIBLE_MAX


def test_confirmation_is_one_line_built_by_code_and_is_not_a_failure():
    result = _asked()

    assert asks(result) and not failed(result)
    assert phrase_from_result(result).text == QUESTION


@pytest.mark.parametrize("source", [SOURCE_TELEGRAM, SOURCE_OPERATOR, VOICE])
def test_yes_confirms_the_same_call_with_confirm_true(source):
    session = Session("telegram")
    assert tool_ask.note(session, [CALL], [_asked()]).tool == "topic_publish"

    reply = tool_ask.reply(session, "да, публикуй", source)

    assert reply.yes is True
    confirmed = reply.asked.confirmed()
    assert confirmed.tool == "topic_publish"
    assert dict(confirmed.args) == {"topic": "/animations/trigger", "fields": {"data": "hi"}, "confirm": True}


@pytest.mark.parametrize(
    "text, verdict",
    [
        ("да", True),
        ("да, публикуй", True),
        ("публикуй", True),
        ("опубликуй это сообщение", True),
        ("нет", False),
        ("не надо", False),
        ("не публикуй", False),
        ("да, публикуй 5 раз", None),
        ("публикуй в /cmd_vel", None),
        ("", None),
    ],
)
def test_answer_grammar_of_the_publish_question(text, verdict):
    assert tool_ask.answer(text, RULE) is verdict


def test_no_is_said_by_code():
    assert phrases.text(RULE.declined) == "Не публикую, ничего не отправлено."


def test_done_phrase_is_the_codes_sentence_from_the_tool_data():
    text = "Опубликовал в /animations/trigger (std_msgs/msg/String) 1 раз, подписчиков 2."

    assert phrase_from_result(_res("done", {"summary": text})).text == text


@pytest.mark.parametrize(
    "reason",
    [
        "topic_forbidden",
        "type_forbidden",
        "no_such_topic",
        "no_endpoints",
        "ambiguous_type",
        "bad_fields",
        "bad_count",
        "bad_topic",
        "publish_failed",
        "ros_unavailable",
        "type_unavailable",
    ],
)
def test_every_refusal_says_the_code_text_and_is_a_failure(reason):
    text = "Отказ: /cmd_vel - топик движения, публиковать запрещено."
    result = _res("rejected", {"summary": text, "reason": reason}, reason)

    assert failed(result) and phrase_from_result(result).text == text
