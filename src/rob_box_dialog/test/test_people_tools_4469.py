"""Пульт Telegram по лицам в диалоге (issue #4469, ADR-0148): классы тулов, фраза исхода строится кодом, снимок уходит в
чат отдельным выводом ``kind=photo`` только по исполненному ``person_photo``."""

from __future__ import annotations

import asyncio

import pytest

from rob_box_core.tool_catalog import telegram_visible_tools
from rob_box_dialog import knowledge as kn
from rob_box_dialog import respond
from rob_box_dialog.command import ToolCall
from rob_box_dialog.execute import Executor, OwnerEvent
from rob_box_dialog.chats import PHOTO_KIND, ChatPorts
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session

SNAP = Session("telegram").snapshot


def _run(coro):
    return asyncio.run(coro)


def _executor(answer):
    sent = []

    def owner(request):
        sent.append(request)
        asyncio.get_running_loop().call_soon(ex.on_event, answer(request))

    ex = Executor(owner)
    return ex, sent


def test_tools_have_classes_and_are_in_the_chat_agent_slice():
    assert kn.TOOL_CLASSES["people_list"] == kn.TOOL_CLASSES["person_photo"] == "query"
    assert kn.TOOL_CLASSES["name_person"] == "action.local"
    assert {"people_list", "person_photo", "name_person"} <= {e.name for e in telegram_visible_tools()}


def test_name_person_done_is_said_by_code_from_the_owners_reply():
    ex, _ = _executor(lambda r: OwnerEvent(r.turn_id, "done", data={
        "id": "fff371fe", "stored_name": "Кирилл", "previous_name": None}))

    [res] = _run(ex.run([ToolCall("name_person", {"id": "fff371fe", "name": "кирилл"})], SNAP, "t1"))

    assert res.status == "done"
    assert respond.phrase_from_result(res).text == "Готово: лицо fff371fe теперь Кирилл."


def test_name_person_rename_phrase_names_the_previous_name():
    ex, _ = _executor(lambda r: OwnerEvent(r.turn_id, "done", data={
        "id": "fff371fe", "stored_name": "Кирилл", "previous_name": "Иван"}))

    [res] = _run(ex.run([ToolCall("name_person", {"id": "fff371fe", "name": "Кирилл"})], SNAP, "t1"))

    assert respond.phrase_from_result(res).text == "Готово: лицо fff371fe (было «Иван») теперь Кирилл."


def test_name_person_done_without_the_owners_proof_is_not_done():
    ex, _ = _executor(lambda r: OwnerEvent(r.turn_id, "done", data={"id": "fff371fe"}))  # имя из базы не вернулось

    [res] = _run(ex.run([ToolCall("name_person", {"id": "fff371fe", "name": "Кирилл"})], SNAP, "t1"))

    assert (res.status, res.reason) == ("rejected", kn.NO_EVIDENCE_REASON)
    assert "Готово" not in respond.phrase_from_result(res).text


@pytest.mark.parametrize("reason,fragment", [
    ("person_not_found", "в базе нет"), ("bad_name", "до 40 знаков"), ("ambiguous_id", "больше одного"),
    ("bad_id", "8 знаков"), ("owner_unavailable", "не отвечает"),
])
def test_name_person_refusal_says_the_owners_reason(reason, fragment):
    ex, _ = _executor(lambda r: OwnerEvent(r.turn_id, "rejected", reason))

    [res] = _run(ex.run([ToolCall("name_person", {"id": "fff371fe", "name": "Кирилл"})], SNAP, "t1"))

    text = respond.phrase_from_result(res).text
    assert fragment in text and "Готово" not in text


@pytest.mark.parametrize("reason,fragment", [
    ("exhibition_mode", "режиме выставки"), ("person_not_found", "в базе нет"), ("no_snapshot", "нет снимка"),
])
def test_person_photo_refusal_says_the_owners_reason(reason, fragment):
    ex, _ = _executor(lambda r: OwnerEvent(r.turn_id, "rejected", reason))

    [res] = _run(ex.run([ToolCall("person_photo", {"id": "fff371fe"})], SNAP, "t1"))

    assert fragment in respond.phrase_from_result(res).text


class Base:
    def __init__(self, results):
        self.results = results

    async def execute(self, actions, snapshot, turn_id):
        return self.results

    def stage(self, *args, **kwargs):
        return None


def _ports(results):
    sent = []
    return ChatPorts(Base(results), 7, lambda ref, text, sid, **extra: sent.append((ref, text, extra)),
                     lambda line: None), sent


def _res(tool, status, data=None, reason=None):
    return ActionResult("t/0", 0, "telegram", tool, kn.TOOL_CLASSES[tool], status, reason, data or {})


def test_executed_person_photo_goes_to_the_chat_as_a_photo_output():
    ports, sent = _ports([_res("person_photo", "done", {
        "id": "fff371fe", "photo_id": "fff371fe", "caption": "fff371fe - без имени"})])

    _run(ports.execute([ToolCall("person_photo", {"id": "fff371fe"})], SNAP, "t"))

    photos = [(ref, text, extra) for ref, text, extra in sent if extra.get("kind") == PHOTO_KIND]
    assert len(photos) == 1 and photos[0][0] == "telegram:7"
    assert '"id": "fff371fe"' in photos[0][1] and "path" not in photos[0][1] and "fff371fe - без имени" in photos[0][1]


@pytest.mark.parametrize("result", [
    _res("person_photo", "rejected", {"reason": "exhibition_mode"}, "exhibition_mode"),
    _res("person_photo", "done", {"id": "fff371fe"}),  # владелец пути не дал
    _res("people_list", "done", {"photo_id": "fff371fe"}),  # чужой тул путь не уносит
])
def test_no_photo_without_an_executed_person_photo_with_a_path(result):
    ports, sent = _ports([result])

    _run(ports.execute([ToolCall(result.tool, {})], SNAP, "t"))

    assert [extra for _r, _t, extra in sent if extra.get("kind") == PHOTO_KIND] == []
