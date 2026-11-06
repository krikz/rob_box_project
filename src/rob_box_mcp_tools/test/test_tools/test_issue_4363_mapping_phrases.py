"""#4363: ответ на тулы картографии говорит код по данным исхода, а не общее «Приехал.»/«Не могу поехать.» (ADR-0148).

Живой 04.11 14:51: «новая карта мастерская» - старт карты озвучен «Приехал.», отказ «уже идёт картографирование»
(FSM-гард MCP) - «Не могу поехать.». Цепочка целиком, без LLM: тул (фейк сервисов RTABMap) / FSM-гард →
``MCPToolResult`` → ``ToolResult`` → ``owner_event`` → ``ActionResult`` → ``phrase_from_result``.
"""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.execute import ActionRequest
from rob_box_dialog.respond import ActionResult, phrase_from_result
from rob_box_dialog.execute import owner_event
from rob_box_llm.provider import ToolResult
from rob_box_mcp_tools.base import MCPToolResult
from rob_box_mcp_tools.mapping_state import MappingState

from . import test_mapping as tm


def _spoken(tool_name, args, mcp_result, agent="personality"):
    request = ActionRequest("t1/1", 0, agent, tool_name, args, kn.TOOL_CLASSES[tool_name], 120.0)
    tool_result = ToolResult(tool_call_id="c1", content=mcp_result.to_json(), is_error=not mcp_result.success,
                             data=mcp_result.data, message=mcp_result.message)
    event = owner_event(request, tool_result)
    result = ActionResult(request.turn_id, 0, agent, tool_name, request.cls, event.status, event.reason, event.data)
    return phrase_from_result(result).text


def _fsm_block(state, tool_name):
    """Отказ FSM-гарда ``mcp_server`` (его данные строит ``MappingState.block_data``)."""
    assert not state.is_tool_allowed(tool_name)
    return MCPToolResult(success=False, error="Идёт картографирование. Сначала скажи 'завершить картографирование'",
                         data=state.block_data())


@pytest.mark.unit
def test_start_mapping_says_new_map_started(monkeypatch):
    tool, _, _, _ = tm._start(monkeypatch)
    res = tool.execute(map_name="мастерская", new_location=True)
    assert _spoken("start_mapping", {"map_name": "мастерская"}, res) == (
        "Начал новую карту «мастерская». Води меня по комнатам, потом скажи «заверши картографирование».")


@pytest.mark.unit
def test_start_mapping_while_mapping_says_map_is_already_being_written():
    """Живой случай #4363: повторный старт во время записи карты - причина вслух, а не «Не могу поехать.»."""
    state = MappingState(active_map=lambda: {"map_id": "m1", "name": "мастерская"})
    state.set_mapping()
    res = _fsm_block(state, "start_mapping")
    assert res.data["reason"] == kn.MAP_ACTIVE_REASON
    assert _spoken("start_mapping", {"map_name": "мастерская"}, res) == (
        "Карта «мастерская» уже пишется. Когда объедем, скажи «заверши картографирование».")


@pytest.mark.unit
def test_start_mapping_while_unnamed_mapping():
    state = MappingState(active_map=lambda: {"map_id": "m1", "name": None})
    state.set_mapping()
    res = _fsm_block(state, "start_mapping")
    assert _spoken("start_mapping", {}, res) == "Карта уже пишется. Когда объедем, скажи «заверши картографирование»."


@pytest.mark.unit
def test_navigation_blocked_by_mapping_names_the_reason():
    state = MappingState(active_map=lambda: {"map_id": "m1", "name": "мастерская"})
    state.set_mapping()
    res = _fsm_block(state, "navigate_to_waypoint")
    assert _spoken("navigate_to_waypoint", {"waypoint": "а"}, res) == (
        "Идёт картографирование. Сначала скажи «заверши картографирование».")


@pytest.mark.unit
def test_start_mapping_service_failure_says_why(monkeypatch):
    tool, _, _, _ = tm._start(monkeypatch, mode_ready=False)
    res = tool.execute(map_name="кухня", new_location=True)
    assert _spoken("start_mapping", {"map_name": "кухня"}, res) == "С картой не получилось: сервис карт недоступен."


@pytest.mark.unit
def test_finish_mapping_says_map_saved(monkeypatch):
    tool, _ = tm._finish(monkeypatch, state=tm._FakeMappingState())
    res = tool.execute(map_name="квартира")
    assert _spoken("finish_mapping", {"map_name": "квартира"}, res) == "Карта «квартира» сохранена."


@pytest.mark.unit
def test_finish_mapping_names_the_active_map_when_no_name_given(monkeypatch):
    store = tm._real_store("мастерская")
    state = MappingState(active_map=store.get_active_map)
    state.set_mapping()
    tool, _ = tm._finish(monkeypatch, state=state)
    tool.waypoint_store = store
    res = tool.execute()
    assert _spoken("finish_mapping", {}, res) == "Карта «мастерская» сохранена."


@pytest.mark.unit
def test_finish_mapping_failure_is_not_motion(monkeypatch):
    tool, _ = tm._finish(monkeypatch, mode_ready=False, state=tm._FakeMappingState())
    res = tool.execute()
    assert _spoken("finish_mapping", {}, res) == "С картой не получилось: сервис карт недоступен."


@pytest.mark.unit
def test_load_map_says_loaded_and_unknown_map_is_named(monkeypatch):
    tool, _ = tm._load_tool(monkeypatch, state=tm._FakeMappingState())
    tool.waypoint_store = tm._real_store("офис")
    assert _spoken("load_map", {"map_name": "офис"}, tool.execute(map_name="офис")) == "Загрузил карту «офис»."
    assert _spoken("load_map", {"map_name": "дача"}, tool.execute(map_name="дача")) == (
        "Карты «дача» нет, ничего не загружал.")


@pytest.mark.unit
def test_optimize_map_says_optimized_or_partial(monkeypatch):
    tm._install_fake_service_modules(monkeypatch)
    module = tm._load_mapping_module(monkeypatch)
    tool = module.OptimizeMapTool(tm._FakeNode())
    assert _spoken("optimize_map", {}, tool.execute()) == "Карта оптимизирована."
    tool.backup_client.ready = False
    assert _spoken("optimize_map", {}, tool.execute()) == "Карту оптимизировал не полностью: часть шагов не удалась."


@pytest.mark.unit
def test_continue_mapping_says_continued(monkeypatch):
    tm._install_fake_service_modules(monkeypatch)
    tool = tm._load_mapping_module(monkeypatch).ContinueMappingTool(tm._FakeNode())
    assert _spoken("continue_mapping", {}, tool.execute()) == (
        "Продолжаю картографирование. Когда объедем, скажи «заверши картографирование».")


@pytest.mark.unit
def test_no_mapping_phrase_is_a_motion_phrase():
    """Без фикса тулы картографии брали «Приехал.»/«Не могу поехать.» класса motion; ``timeout`` (#4344: транспорт ждёт
    тул весь срок класса) - «Не доехал — время вышло.» класса."""
    for tool, entries in kn.MAP_TOOL_PHRASES.items():
        assert set(entries) == {"done", "rejected", "timeout"}, tool
