"""Исполнитель: успех — только событие владельца, нет события к дедлайну — timeout (ADR-0150 §5.2-5.3)."""

import asyncio
import logging
import threading

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import respond
from rob_box_dialog.command import ToolCall
from rob_box_dialog.execute import Executor, OwnerEvent, deadline_s
from rob_box_dialog.session import Session, SpeakerRef

_SESSION = Session("personality")
_SESSION.set_speaker(SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="confirmed"))
SNAP = _SESSION.snapshot  # собеседник в снимке есть: тулы памяти без него не идут (test_memory.py)


def _run(coro):
    return asyncio.run(coro)


class Owner:
    """Владелец-заглушка: отвечает событием на запрос (или молчит), запоминает запросы."""

    def __init__(self, answer=None):
        self.requests = []
        self.answer = answer
        self.executor = None

    def __call__(self, request):
        self.requests.append(request)
        if self.answer is not None:
            asyncio.get_running_loop().call_soon(self.executor.on_event, self.answer(request))


def _executor(owner, **kw):
    ex = Executor(owner, **kw)
    owner.executor = ex
    return ex


def test_done_only_by_owner_event_and_phrase_from_it():
    owner = Owner(lambda r: OwnerEvent(r.turn_id, "done", data={"fact": "Саша любит зелёный чай", "fact_id": 7}))
    ex = _executor(owner)
    [res] = _run(ex.run([ToolCall("memory_save", {"fact": "Саша любит зелёный чай"})], SNAP, "t1"))
    req = owner.requests[0]
    assert (req.turn_id, req.tool, req.cls, req.timeout_s) == ("t1/0", "memory_save", "action.local", 3.0)
    assert (res.status, res.cls, res.turn_id) == ("done", "action.local", "t1/0")
    assert respond.phrase_from_result(res).text == "Запомнил: Саша любит зелёный чай."


def test_no_event_until_class_deadline_is_honest_timeout(monkeypatch, caplog):
    owner = Owner()  # молчит
    ex = _executor(owner)
    monkeypatch.setattr("rob_box_dialog.execute.deadline_s", lambda cls: 0.05)
    with caplog.at_level(logging.WARNING):
        [res] = _run(ex.run([ToolCall("set_voice", {"voice": "alena"})], SNAP, "t2"))
    assert res.status == "timeout"
    assert respond.phrase_from_result(res).text == "Не дождался подтверждения — не знаю, вышло ли."
    assert "нет события" in caplog.text


def test_rejected_with_reason_reaches_phrase():
    owner = Owner(lambda r: OwnerEvent(r.turn_id, "rejected", "busy", data={"sound": "robot_happy"}))
    ex = _executor(owner)
    [res] = _run(ex.run([ToolCall("play_sound", {"sound": "robot_happy"})], SNAP, "t3"))
    assert (res.status, res.reason, res.cls) == ("rejected", "busy", "action.media")
    assert respond.phrase_from_result(res).emotion == "sad"


def test_unknown_event_is_stale_and_ignored(caplog):
    owner = Owner(lambda r: OwnerEvent(r.turn_id, "done"))
    ex = _executor(owner)

    async def scenario():
        out = await ex.run([ToolCall("get_current_time", {})], SNAP, "t4")
        ex.on_event(OwnerEvent("t4/0", "done"))  # повтор после закрытия ожидания
        ex.on_event(OwnerEvent("чужой", "done"))
        return out

    with caplog.at_level(logging.WARNING):
        [res] = _run(scenario())
    assert res.status == "done"
    assert caplog.text.count("stale_event") == 2


def test_late_event_after_timeout_is_stale_not_success(monkeypatch, caplog):
    monkeypatch.setattr("rob_box_dialog.execute.deadline_s", lambda cls: 0.02)
    owner = Owner()
    ex = _executor(owner)

    async def scenario():
        out = await ex.run([ToolCall("memory_save", {"fact": "x"})], SNAP, "t5")
        ex.on_event(OwnerEvent("t5/0", "done"))
        return out

    with caplog.at_level(logging.WARNING):
        [res] = _run(scenario())
    assert res.status == "timeout"
    assert "stale_event ref=t5/0" in caplog.text


def test_actions_run_in_command_order_one_request_each():
    owner = Owner(lambda r: OwnerEvent(r.turn_id, "done", data={**r.args, "fact_id": 1}))
    ex = _executor(owner)
    calls = [ToolCall("get_current_time", {}), ToolCall("memory_save", {"fact": "a"})]
    res = _run(ex.run(calls, SNAP, "t6"))
    assert [r.turn_id for r in owner.requests] == ["t6/0", "t6/1"]
    assert [r.tool for r in res] == ["get_current_time", "memory_save"]
    assert respond.join(res).text == "Запомнил: a."  # query.done — речь LLM по данным, шаблона нет


def test_pending_promise_then_final_by_event_ref():
    late = []
    owner = Owner(lambda r: OwnerEvent(r.turn_id, "pending", data={"target": "кухня"}, event_ref="goal-1"))
    ex = _executor(owner, on_late=late.append)

    async def scenario():
        out = await ex.run([ToolCall("navigate_to_waypoint", {"name": "кухня"})], SNAP, "t7")
        ex.on_event(OwnerEvent("goal-1", "done"))
        await asyncio.sleep(0)
        return out

    [res] = _run(scenario())
    assert (res.status, res.event_ref) == ("pending", "goal-1")
    assert respond.phrase_from_result(res).text == "Еду к кухня."
    assert [(r.status, r.tool) for r in late] == [("done", "navigate_to_waypoint")]


def test_event_from_other_thread_is_delivered_to_loop():
    ex = None

    def send(request):
        threading.Timer(0.01, ex.on_event, args=(OwnerEvent(request.turn_id, "done"),)).start()

    ex = Executor(send)
    [res] = _run(ex.run([ToolCall("set_volume", {"level": 50})], SNAP, "t8"))
    assert res.status == "done"


def test_send_failure_is_rejected_not_crash():
    def send(request):
        raise OSError("нет транспорта")

    [res] = _run(Executor(send).run([ToolCall("memory_save", {"fact": "x"})], SNAP, "t9"))
    assert (res.status, res.reason) == ("rejected", "error")


def test_unknown_status_from_owner_is_not_success():
    owner = Owner(lambda r: OwnerEvent(r.turn_id, "started"))
    [res] = _run(_executor(owner).run([ToolCall("memory_save", {"fact": "x"})], SNAP, "t10"))
    assert (res.status, res.reason) == ("rejected", "bad_event")


@pytest.mark.parametrize("cls", sorted(kn.ACTION_CLASSES))
def test_every_class_has_finite_deadline(cls):
    assert 0 < deadline_s(cls) <= 120


# -- эпоха и перебивание (PR-9 #3703, ADR-0150 §6.2) ---------------------------------------------------------------


def test_owner_event_of_old_epoch_is_dropped_as_stale_epoch(caplog):
    epoch = [0]

    def answer(request):
        epoch[0] = 1  # пока владелец исполнял, человек сказал «стой»: эпоха +1
        return OwnerEvent(request.turn_id, "done")

    owner = Owner(answer)
    ex = _executor(owner, epoch=lambda: epoch[0])
    with caplog.at_level(logging.WARNING):
        [res] = _run(ex.run([ToolCall("memory_save", {"fact": "x"})], SNAP, "t20"))
    assert (res.status, res.reason) == ("cancelled", "stale_epoch")
    assert "stale_epoch memory_save t20/0 epoch=0 now=1" in caplog.text


def test_late_result_of_old_epoch_does_not_reach_on_late(caplog):
    late, epoch = [], [0]
    owner = Owner(lambda r: OwnerEvent(r.turn_id, "pending", event_ref="goal-9"))
    ex = _executor(owner, on_late=late.append, epoch=lambda: epoch[0])

    async def scenario():
        out = await ex.run([ToolCall("navigate_to_waypoint", {"name": "кухня"})], SNAP, "t21")
        epoch[0] = 1
        ex.on_event(OwnerEvent("goal-9", "done"))
        return out

    with caplog.at_level(logging.WARNING):
        [res] = _run(scenario())
    assert res.status == "pending" and late == []
    assert "stale_epoch navigate_to_waypoint" in caplog.text


def test_turn_cancel_cuts_interruptible_request_but_motion_send_survives(caplog):
    """Отмена хода (перебивание): ``query`` рвётся вместе с ходом, отправка движения доживает до события владельца —
    его исход старой эпохи уходит строкой ``stale_epoch``, а не в ход (§5.1: motion непрерываем)."""
    epoch, sent = [0], []

    async def send(request):
        await asyncio.sleep(0.05)  # владелец исполняет
        sent.append(request.tool)
        ex.on_event(OwnerEvent(request.turn_id, "done"))

    ex = Executor(send, epoch=lambda: epoch[0])

    async def scenario():
        turn = asyncio.ensure_future(ex.run([ToolCall("get_current_time", {}),
                                             ToolCall("navigate_to_waypoint", {"name": "кухня"})], SNAP, "t22"))
        await asyncio.sleep(0.01)
        turn.cancel()
        epoch[0] = 1
        with pytest.raises(asyncio.CancelledError):
            await turn
        await asyncio.sleep(0.1)

    with caplog.at_level(logging.WARNING):
        _run(scenario())
    assert sent == ["navigate_to_waypoint"], "query отменён до владельца, движение доехало"
    assert "stale_epoch navigate_to_waypoint t22/1" in caplog.text
