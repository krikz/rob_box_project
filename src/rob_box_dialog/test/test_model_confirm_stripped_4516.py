"""Подтверждение тула даёт только человек (ADR-0148, ревью #4525): ``confirm`` от МОДЕЛИ код снимает,
подтверждённый вызов приходит только через окно ответа ``tool_ask`` (``Asked.confirmed()``). Решает таблица
``knowledge.TOOL_QUESTIONS``."""

from __future__ import annotations

import asyncio
import logging

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import SOURCE_TELEGRAM, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine, unconfirmed_by_model

LOG = logging.getLogger("t4516")


class _Turn:
    turn_id = "t1"


class Ports:
    """Исполнитель как у тулов с вопросом: без ``confirm`` - вопрос, с ``confirm`` - готово."""

    def __init__(self, answers):
        self.answers, self.executed, self.said = list(answers), [], []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0) if self.answers else Answer(Command(), "deepseek")

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, call in enumerate(actions):
            args = dict(call.args)
            self.executed.append((call.tool, args))
            reason = {"topic_publish": "needs_publish_confirmation", "merge_people": "needs_merge_confirmation"}[
                call.tool
            ]
            asked = args.get("confirm") is not True
            data = {"question": "Спрашиваю?"} if asked else {"summary": "Сделано."}
            out.append(
                ActionResult(
                    f"{turn_id}/{n}",
                    snapshot.epoch,
                    snapshot.agent,
                    call.tool,
                    kn.TOOL_CLASSES[call.tool],
                    "rejected" if asked else "done",
                    reason if asked else None,
                    data,
                )
            )
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *a, **k):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _engine(ports):
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(Session("telegram"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))


def _run(engine, text):
    asyncio.run(engine.run(Addressed(text, SOURCE_TELEGRAM)))


PUBLISH = {"topic": "/animations/trigger", "fields": {"data": "hi"}}
MERGE = {"target": "71cda108", "numbers": [1, 3, 5]}
CASES = [
    ("topic_publish", PUBLISH, "опубликуй в топик /animations/trigger привет", "да, публикуй"),
    ("merge_people", MERGE, "склей их", "да, склей"),
]


@pytest.mark.parametrize("tool,args,request_text,yes", CASES)
def test_model_confirm_true_is_stripped_and_the_tool_asks(tool, args, request_text, yes, caplog):
    ports = Ports([Answer(Command(actions=(ToolCall(tool, {**args, "confirm": True}),)), "deepseek")])
    engine = _engine(ports)

    with caplog.at_level(logging.WARNING):
        _run(engine, request_text)

    assert ports.executed == [(tool, args)], "модель прислала confirm=true, а до тула дошёл вызов без него"
    assert "confirm от модели снят" in caplog.text and tool in caplog.text
    assert "Спрашиваю?" in " ".join(ports.said)


@pytest.mark.parametrize("tool,args,request_text,yes", CASES)
def test_human_yes_runs_the_same_call_with_confirm_through_tool_ask(tool, args, request_text, yes):
    ports = Ports([Answer(Command(actions=(ToolCall(tool, {**args, "confirm": True}),)), "deepseek")])
    engine = _engine(ports)
    _run(engine, request_text)

    _run(engine, yes)

    assert ports.executed == [(tool, args), (tool, {**args, "confirm": True})]


@pytest.mark.parametrize("tool,args,request_text,yes", CASES)
def test_model_cannot_confirm_even_after_the_question_without_the_human(tool, args, request_text, yes):
    ports = Ports([Answer(Command(actions=(ToolCall(tool, {**args, "confirm": True}),)), "deepseek")] * 2)
    engine = _engine(ports)

    _run(engine, request_text)
    _run(engine, request_text)  # человек «да» не сказал: модель снова шлёт confirm=true

    assert [a.get("confirm") for _, a in ports.executed] == [None, None]


def test_strip_is_by_the_question_table_not_by_tool_names(caplog):
    cmd = Command(
        actions=(
            ToolCall("topic_publish", {**PUBLISH, "confirm": True}),
            ToolCall("music_library", {"action": "delete", "track_ref": "x", "confirm": True}),
            ToolCall("set_volume", {"confirm": True}),  # тула нет в таблице вопросов: ключ остаётся
        )
    )
    with caplog.at_level(logging.WARNING):
        out = unconfirmed_by_model(LOG, _Turn(), cmd)

    assert "confirm" not in out.actions[0].args and "confirm" not in out.actions[1].args
    assert out.actions[2].args == {"confirm": True}
    assert out.actions[0].args["topic"] == "/animations/trigger"


def test_command_without_confirm_is_returned_as_is():
    cmd = Command(actions=(ToolCall("topic_publish", dict(PUBLISH)),))
    assert unconfirmed_by_model(LOG, _Turn(), cmd) is cmd
