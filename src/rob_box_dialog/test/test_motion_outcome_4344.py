"""Итог тула движения говорит код по исходу, а не обрыв транспорта (#4344) и не «Приехал.» класса (#3789).

HIL 04.11 21:45 (raw в #4344): поездка 35,7 с - через 10,0 с транспорт MCP вернул «Timeout ожидания результата
инструмента», робот сказал «Не могу поехать.», а сам доехал (Nav2 SUCCEEDED); итог не прозвучал. HIL 04.11 21:51:18
(raw в #3789): ``stop_navigation`` без активной цели - тул ответил ``no_active_goal``, робот сказал «Приехал.».

Время в тестах исполнителя сжато (:data:`SCALE`): сроки класса и транспорта - те же, что в проде, умноженные на SCALE.
"""

import asyncio

import pytest

from rob_box_dialog import execute
from rob_box_dialog import knowledge as kn
from rob_box_dialog import respond
from rob_box_dialog.command import ToolCall
from rob_box_dialog.execute import ActionRequest, Executor, OwnerEvent, transport_timeouts
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session

SNAP = Session("personality").snapshot
SCALE = 0.002
#: Срок транспорта до #4344 (``ROSMCPToolProvider.default_timeout``): им транспорт ждал любой тул.
OLD_TRANSPORT_S = 10.0
#: Ответ моста MCP на обрыв ожидания (``LLMToolCallAdapter.execute_tool_call_sync``, лог HIL 21:45:51.389).
TRANSPORT_TIMEOUT_TEXT = "Timeout ожидания результата инструмента"
_REAL_DEADLINE = execute.deadline_s

ARRIVED = {"outcome": "succeeded", "arrival": kn.NAV_ARRIVED, "elapsed_s": 35.7, "target": "хилтест"}
ABORTED = {"outcome": "aborted", "reason": "aborted", "target": "хилтест"}
CANCELED = {"outcome": "canceled", "reason": "canceled", "target": "хилтест"}


def _drive(tool, takes_s, status, data, transport_s=None):
    """Ход с одним тулом движения: транспорт ждёт итог ``transport_s`` (по умолчанию - :func:`transport_timeouts`),
    тул отвечает через ``takes_s``; всё время умножено на :data:`SCALE`. Итог - фраза кода по результату хода."""

    async def send(request: ActionRequest):
        wait = transport_s if transport_s is not None else transport_timeouts()[request.tool] / SCALE
        if takes_s > wait:  # мост бросает ожидание и отвечает ошибкой (как ``execute_tool_call_sync``)
            await asyncio.sleep(wait * SCALE)
            ex.on_event(OwnerEvent(request.turn_id, "rejected", TRANSPORT_TIMEOUT_TEXT))
            return
        await asyncio.sleep(takes_s * SCALE)
        ex.on_event(OwnerEvent(request.turn_id, status, data.get("reason"), {**dict(request.args), **data}))

    ex = Executor(send)
    [res] = asyncio.run(ex.run([ToolCall(tool, {"waypoint": "хилтест"})], SNAP, "t4344"))
    return res, respond.phrase_from_result(res).text


@pytest.fixture
def scaled(monkeypatch):
    monkeypatch.setattr(execute, "deadline_s", lambda cls: _REAL_DEADLINE(cls) * SCALE)


def test_old_ten_second_transport_reproduces_the_hil_lie(scaled):
    """Контроль: при транспорте 10 с поездка 35,7 с давала «Не могу поехать.» - это и был баг HIL 21:45."""
    res, text = _drive("navigate_to_waypoint", 35.7, "done", ARRIVED, transport_s=OLD_TRANSPORT_S)
    assert (res.status, text) == ("rejected", "Не могу поехать.")


@pytest.mark.parametrize("tool", ["navigate_to_waypoint", "navigate_to_coordinates"])
def test_trip_longer_than_ten_seconds_says_arrived_not_cannot_go(scaled, tool):
    """Поездка 35,7 с, Nav2 SUCCEEDED: итог «Приехал на точку хилтест.», а не «Не могу поехать.» на 10-й секунде."""
    res, text = _drive(tool, 35.7, "done", ARRIVED)
    assert (res.status, text) == ("done", "Приехал на точку хилтест.")


def test_trip_aborted_after_ten_seconds_says_not_reached(scaled):
    """ABORTED через 25 с (HIL 21:42:09, ``move_direction``): фраза по ABORTED, а не отказ транспорта."""
    res, text = _drive("navigate_to_waypoint", 25.0, "rejected", ABORTED)
    assert (res.status, text) == ("rejected", "Не доехал до точки хилтест: навигация прервалась.")


def test_move_direction_aborted_after_ten_seconds_says_aborted(scaled):
    res, text = _drive("move_direction", 25.0, "rejected", {"outcome": "aborted", "reason": "aborted"})
    assert (res.status, text) == ("rejected", "Не доехал: навигация прервалась.")


def test_trip_canceled_after_ten_seconds_says_canceled(scaled):
    res, text = _drive("navigate_to_waypoint", 30.0, "rejected", CANCELED)
    assert (res.status, text) == ("rejected", "Не доехал до точки хилтест: движение отменили.")


def test_no_outcome_within_class_deadline_is_honest_timeout_not_cannot_go(scaled):
    """Тул молчит дольше срока класса (120 с): честное «время вышло», а не «Не могу поехать.»."""
    res, text = _drive("navigate_to_waypoint", 300.0, "done", ARRIVED)
    assert (res.status, text) == ("timeout", "Не доехал — время вышло.")


def test_transport_waits_at_least_the_class_deadline_for_every_tool_with_one():
    """Один источник - срок класса (``ACTION_CLASSES``): транспорт любого тула с таким сроком ждёт не меньше срока +
    запаса; у тулов движения это > 10 с старого транспорта."""
    waits = transport_timeouts()
    for tool, cls in kn.TOOL_CLASSES.items():
        if kn.ACTION_CLASSES[cls].timeout_s is None:
            assert tool not in waits, f"{tool}: срок задаёт владелец - транспорт ждёт как раньше"
            continue
        assert waits[tool] >= execute.deadline_s(cls) + kn.ACTION_GRACE_S, tool
    motion = [t for t, c in kn.TOOL_CLASSES.items() if c == "action.motion"]
    assert {"navigate_to_waypoint", "navigate_to_coordinates", "move_direction", "stop_navigation"} <= set(motion)
    assert min(waits[t] for t in motion) > kn.ACTION_CLASSES["action.motion"].timeout_s > OLD_TRANSPORT_S


def test_ros_mcp_provider_hands_the_motion_wait_to_the_bridge():
    """Провайдер, как его собирает ``dialog_node._mcp_provider``, отдаёт мосту срок тула движения, а не 10 с."""
    pytest.importorskip("rob_box_harness")
    from rob_box_harness.executors.ros_mcp import ROSMCPToolProvider

    seen = {}

    class Bridge:
        def execute_tool_call_sync(self, tool_name, parameters, timeout=None):
            seen[tool_name] = timeout
            return {"success": True, "data": dict(ARRIVED)}

    provider = ROSMCPToolProvider(Bridge(), tool_timeouts=transport_timeouts())
    provider.update_tools([{"type": "function", "function": {"name": "navigate_to_waypoint", "parameters": {}}}])
    asyncio.run(provider.invoke("navigate_to_waypoint", {}))
    assert seen["navigate_to_waypoint"] >= kn.ACTION_CLASSES["action.motion"].timeout_s + kn.ACTION_GRACE_S


# -- стоп навигации и тулы движения, что никуда не едут (#3789) --------------------------------------------------------


def _result(tool, status, data=None, reason=None):
    return ActionResult(turn_id="t3789/0", epoch=0, agent="personality", tool=tool, cls=kn.TOOL_CLASSES[tool],
                        status=status, reason=reason, data=data or {})


@pytest.mark.parametrize("status,data,reason,text", [
    ("done", {"outcome": kn.STOP_IDLE}, None, "Я и так стою."),
    ("done", {"outcome": kn.STOP_CONFIRMED}, None, "Останавливаюсь."),
    ("rejected", {"outcome": "cancel_refused"}, "Nav2 не принял отмену цели (код 1)",
     "Не уверен, что остановился: навигация не подтвердила отмену."),
    ("rejected", {"outcome": "unavailable"}, "Nav2 cancel_goal: сервис недоступен",
     "Не уверен, что остановился: навигация не подтвердила отмену."),
    ("timeout", {}, None, "Не уверен, что остановился: навигация не подтвердила отмену."),
])
def test_stop_navigation_phrase_by_cancel_outcome(status, data, reason, text):
    """HIL 21:51:18: ``no_active_goal`` - «Я и так стою.», а не «Приехал.»; отмена принята - «Останавливаюсь.»."""
    assert respond.phrase_from_result(_result("stop_navigation", status, data, reason)).text == text


def test_stop_navigation_through_executor_no_active_goal_is_not_arrived():
    async def send(request):
        ex.on_event(OwnerEvent(request.turn_id, "done", data={"outcome": kn.STOP_IDLE}))

    ex = Executor(send)
    [res] = asyncio.run(ex.run([ToolCall("stop_navigation", {})], SNAP, "t3789"))
    assert (res.status, respond.phrase_from_result(res).text) == ("done", "Я и так стою.")


def test_cancel_kinds_of_the_tool_are_the_knowledge_ones():
    """Контракт с тулом: виды исхода отмены ``action_outcome`` - значения из ``knowledge`` (один источник)."""
    ao = pytest.importorskip("rob_box_mcp_tools.core.action_outcome")
    assert (ao.CANCEL_CONFIRMED, ao.CANCEL_NO_ACTIVE_GOAL) == (kn.STOP_CONFIRMED, kn.STOP_IDLE)


#: Ключи класса ``action.motion``, что говорят о поездке («Приехал.», «Не могу поехать.», «время вышло»).
TRAVEL_KEYS = {kn.ACTION_CLASSES["action.motion"].phrases[s] for s in ("done", "rejected", "timeout")}
NOT_TRAVELLING = sorted(t for t, c in kn.TOOL_CLASSES.items() if c == "action.motion" and t not in kn.MOTION_TOOLS)


@pytest.mark.parametrize("tool", NOT_TRAVELLING)
@pytest.mark.parametrize("status", ["done", "rejected", "timeout"])
@pytest.mark.parametrize("data", [{}, {"outcome": "x", "reason": "x"}, {"outcome": kn.STOP_IDLE}])
def test_motion_class_tool_that_does_not_drive_never_says_travel_phrase(tool, status, data):
    """Класс закрыт: тул класса ``action.motion``, который базу не везёт (стоп, карта), ни при каком исходе и данных
    не говорит «Приехал.»/«Не могу поехать.»/«Не доехал — время вышло.» класса."""
    key = respond.phrase_key(_result(tool, status, data, data.get("reason")))
    assert key is not None and key.split(".")[:2] != ["motion", "rejected"] and key not in TRAVEL_KEYS, (tool, key)
