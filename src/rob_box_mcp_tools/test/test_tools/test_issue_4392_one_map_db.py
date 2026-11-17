"""#4392: база rtabmap одна - «Загрузил карту X» звучит, только если в базе и есть X (ADR-0148).

HIL 04.11 23:00-23:41: ``load_map("мастерская")`` перезагружал единственный ``/maps/rtabmap.db`` при любом имени и
переключал «активную» карту в таблице ``maps`` - «Загрузил карту «мастерская».», хотя базу уже переписал
``start_mapping("хил-офис")``. Теперь активная строка ``maps`` - единственное место правды о карте в базе: её
переключает только новое картографирование, ``load_map`` с чужим именем честно отказывает, ``list_maps`` и FSM
читают её же, ``delete_map`` забывает старые имена, а карту в базе не трогает. Цепочка фразы - как в #4363: тул →
``MCPToolResult`` → ``ToolResult`` → ``owner_event`` → ``phrase_from_result``.
"""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_mcp_tools.mapping_state import MappingState

from . import test_mapping as tm
from .test_issue_4363_mapping_phrases import _spoken


def _hil_store():
    """Как на роботе после HIL: «мастерская» записана, затем базу переписал «хил-офис» (активна последняя)."""
    return tm._real_store("двор", "мастерская", "хил-офис")


def _module(monkeypatch):
    tm._install_fake_service_modules(monkeypatch)
    return tm._load_mapping_module(monkeypatch)


@pytest.mark.unit
def test_load_overwritten_map_says_only_one_map_is_stored(monkeypatch):
    tool, node = tm._load_tool(monkeypatch, state=tm._FakeMappingState())
    tool.waypoint_store = _hil_store()
    res = tool.execute(map_name="мастерская")
    assert _spoken("load_map", {"map_name": "мастерская"}, res) == (
        "Карту «мастерская» не храню: база карт у меня одна — «хил-офис». Ничего не загружал.")
    assert node.clients["/rtabmap/rtabmap/load_database"].call_count == 0


@pytest.mark.unit
def test_load_overwritten_map_when_map_in_db_is_unnamed(monkeypatch):
    store = tm._real_store("мастерская", None)
    tool, _ = tm._load_tool(monkeypatch)
    tool.waypoint_store = store
    res = tool.execute(map_name="мастерская")
    assert _spoken("load_map", {"map_name": "мастерская"}, res) == (
        "Карту «мастерская» не храню: база карт у меня одна, и это не она. Ничего не загружал.")


@pytest.mark.unit
def test_load_map_phrase_names_map_in_db_not_the_argument(monkeypatch):
    """Без имени - перезагрузка базы, и в фразе имя карты из ``maps``, а не «Загрузил карту.» вслепую."""
    tool, _ = tm._load_tool(monkeypatch)
    tool.waypoint_store = _hil_store()
    assert _spoken("load_map", {}, tool.execute()) == "Загрузил карту «хил-офис»."


@pytest.mark.unit
def test_load_map_never_switches_active_map(monkeypatch):
    store = _hil_store()
    tool, _ = tm._load_tool(monkeypatch)
    tool.waypoint_store = store
    for name in ("мастерская", "двор", "дача", "хил-офис", ""):
        tool.execute(map_name=name)
        assert store.get_active_map()["name"] == "хил-офис"


@pytest.mark.unit
def test_store_has_no_name_based_active_switch():
    """Переключатель активной карты по имени (им ``load_map`` «загружал» чужую карту) удалён из владельца."""
    from rob_box_mcp_tools.waypoint_adapter import WaypointAdapter
    from rob_box_mcp_tools.waypoint_store import WaypointStore

    assert not hasattr(WaypointStore, "set_active_map_by_name")
    assert not hasattr(WaypointAdapter, "set_active_map_by_name")


@pytest.mark.unit
def test_mapping_state_reads_map_name_from_store_only():
    store = _hil_store()
    state = MappingState(active_map=store.get_active_map)
    assert state.get()["map_name"] == "хил-офис"
    store.create_map("кухня")
    state.set_mapping()
    assert state.get()["mode"] == "mapping" and state.get()["map_name"] == "кухня"
    assert state.block_data()[kn.MAP_ACTIVE_FIELD] == "кухня"


@pytest.mark.unit
def test_mapping_state_without_store_or_with_broken_store_has_no_name():
    assert MappingState().get()["map_name"] is None

    def boom():
        raise RuntimeError("БД карт недоступна")

    state = MappingState(active_map=boom)
    state.set_mapping()
    assert state.get() == {**state.get(), "mode": "mapping", "map_name": None, "map_id": None}


@pytest.mark.unit
def test_start_mapping_continuation_speaks_map_in_db_not_argument(monkeypatch):
    tool, _, _, _ = tm._start(monkeypatch)
    tool.waypoint_store = _hil_store()
    res = tool.execute(map_name="мастерская", new_location=False)
    assert res.success is True and res.data["map_name"] == "хил-офис"
    assert tool.waypoint_store.get_active_map()["name"] == "хил-офис"


@pytest.mark.unit
def test_start_new_map_becomes_the_only_loadable_map(monkeypatch):
    store = _hil_store()
    tool, _, _, _ = tm._start(monkeypatch)
    tool.waypoint_store = store
    assert tool.execute(map_name="кухня", new_location=True).success is True
    load, _ = tm._load_tool(monkeypatch)
    load.waypoint_store = store
    assert load.execute(map_name="кухня").success is True
    assert load.execute(map_name="хил-офис").data["reason"] == kn.MAP_NOT_STORED_REASON


@pytest.mark.unit
def test_finish_mapping_renames_map_in_db_and_says_that_name(monkeypatch):
    store = tm._real_store(None)
    tool, _ = tm._finish(monkeypatch, state=tm._FakeMappingState())
    tool.waypoint_store = store
    res = tool.execute(map_name="мастерская")
    assert _spoken("finish_mapping", {"map_name": "мастерская"}, res) == "Карта «мастерская» сохранена."
    assert store.get_active_map()["name"] == "мастерская"


@pytest.mark.unit
def test_list_maps_reports_single_map_and_stale_names(monkeypatch):
    store = tm._real_store("квартира", "квартира", "двор", None, "мастерская", "хил-офис")
    tool = _module(monkeypatch).ListMapsTool(tm._FakeNode(), store)
    res = tool.execute()
    assert res.success is True
    assert res.data == {"map_name": "хил-офис", "has_map": True, "stale_names": ["мастерская", "двор", "квартира"]}
    assert res.message == (
        "Храню одну карту — «хил-офис»: база карт у меня одна. Старые названия без карты: мастерская, двор, квартира.")


@pytest.mark.unit
def test_list_maps_without_any_map(monkeypatch):
    tool = _module(monkeypatch).ListMapsTool(tm._FakeNode(), tm._real_store())
    res = tool.execute()
    assert res.data == {"map_name": None, "has_map": False, "stale_names": []}
    assert res.message == "Готовой карты у меня нет."


@pytest.mark.unit
def test_list_maps_store_error_is_honest_failure(monkeypatch):
    tool = _module(monkeypatch).ListMapsTool(tm._FakeNode(), tm._RaisingStore())
    res = tool.execute()
    assert res.success is False and res.data["reason"] == "map_lookup_failed"


@pytest.mark.unit
def test_delete_stale_map_forgets_all_its_rows_and_waypoints(monkeypatch):
    store = tm._real_store("хил-офис")
    store.save_waypoint("окно", 1.0, 2.0)
    store.create_map("Хил-Офис")
    store.create_map("мастерская")
    tool = _module(monkeypatch).DeleteMapTool(tm._FakeNode(), store)
    res = tool.execute(map_name="хил-офис")
    assert res.success is True and res.data["removed"] == 2
    assert _spoken("delete_map", {"map_name": "хил-офис"}, res) == "Забыл карту «хил-офис»."
    assert [m["name"] for m in store.list_maps()] == ["мастерская"]
    rows = store._conn.execute("SELECT count(*) FROM waypoints").fetchone()[0]  # noqa: SLF001
    assert rows == 0


@pytest.mark.unit
def test_delete_map_in_db_refuses_and_keeps_it(monkeypatch):
    store = _hil_store()
    tool = _module(monkeypatch).DeleteMapTool(tm._FakeNode(), store)
    res = tool.execute(map_name="ХИЛ-ОФИС")
    assert res.success is False and res.data["reason"] == kn.MAP_DELETE_ACTIVE_REASON
    assert _spoken("delete_map", {"map_name": "ХИЛ-ОФИС"}, res) == (
        "Карту «ХИЛ-ОФИС» удалить не могу: она у меня в базе одна, её можно только перезаписать новой картой.")
    assert store.get_active_map()["name"] == "хил-офис" and len(store.list_maps()) == 3


@pytest.mark.unit
def test_delete_unknown_map_says_nothing_to_delete(monkeypatch):
    tool = _module(monkeypatch).DeleteMapTool(tm._FakeNode(), _hil_store())
    res = tool.execute(map_name="дача")
    assert res.success is False
    assert _spoken("delete_map", {"map_name": "дача"}, res) == "Карты «дача» у меня нет, удалять нечего."


@pytest.mark.unit
def test_delete_map_store_error_does_not_claim_success(monkeypatch):
    tool = _module(monkeypatch).DeleteMapTool(tm._FakeNode(), tm._RaisingStore())
    res = tool.execute(map_name="двор")
    assert res.success is False
    assert _spoken("delete_map", {"map_name": "двор"}, res) == "Не смог удалить карту «двор»."


@pytest.mark.unit
def test_new_map_tools_have_classes_and_mapping_group():
    assert kn.TOOL_CLASSES["list_maps"] == "query"
    assert kn.TOOL_CLASSES["delete_map"] == "action.local"
    assert set(kn.TOOL_GROUPS["maps"].tools) == {"list_maps", "delete_map", "load_map"}


@pytest.mark.unit
@pytest.mark.parametrize("text", ["Робот, какие карты ты знаешь?", "Робот, удали карту хил-офис",
                                  "Робот, забудь старую карту двор"])
def test_list_and_delete_requests_see_their_tools(text):
    from rob_box_dialog.session import SessionSnapshot
    from rob_box_dialog.tools_view import groups_for

    groups = groups_for(SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0), text)
    assert groups[0] == "maps", groups
