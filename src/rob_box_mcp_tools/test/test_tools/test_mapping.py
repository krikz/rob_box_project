import importlib.util
import sys
import types
from pathlib import Path

import pytest


class _ImmediateFuture:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._result


class _FakeClient:
    def __init__(self, srv_type, srv_name, ready=True, result=object()):
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.ready = ready
        self.result_value = result
        self.call_count = 0
        self.requests = []
        self.kwargs = {}

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.call_count += 1
        self.requests.append(request)
        return _ImmediateFuture(self.result_value)


class _FakeLogger:
    def __init__(self):
        self.info_messages = []
        self.warning_messages = []
        self.error_messages = []

    def info(self, msg):
        self.info_messages.append(msg)

    def warning(self, msg):
        self.warning_messages.append(msg)

    def error(self, msg):
        self.error_messages.append(msg)


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()
        self.clients = {}

    def get_logger(self):
        return self._logger

    def create_client(self, srv_type, srv_name, **kwargs):
        client = _FakeClient(srv_type, srv_name)
        client.kwargs = kwargs
        self.clients[srv_name] = client
        return client


def _install_fake_service_modules(monkeypatch):
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    rtabmap_msgs = types.ModuleType("rtabmap_msgs")
    rtabmap_msgs_srv = types.ModuleType("rtabmap_msgs.srv")
    rclpy = types.ModuleType("rclpy")
    rclpy_callback_groups = types.ModuleType("rclpy.callback_groups")

    class Empty:
        class Request:
            pass

    class DetectMoreLoopClosures:
        class Request:
            pass

    class GlobalBundleAdjustment:
        class Request:
            pass

    class CleanupLocalGrids:
        class Request:
            pass

    class PublishMap:
        class Request:
            pass

    class LoadDatabase:
        class Request:
            def __init__(self):
                self.database_path = ""
                self.clear = False

    class ReentrantCallbackGroup:
        pass

    std_srvs_srv.Empty = Empty
    rtabmap_msgs_srv.DetectMoreLoopClosures = DetectMoreLoopClosures
    rtabmap_msgs_srv.GlobalBundleAdjustment = GlobalBundleAdjustment
    rtabmap_msgs_srv.CleanupLocalGrids = CleanupLocalGrids
    rtabmap_msgs_srv.PublishMap = PublishMap
    rtabmap_msgs_srv.LoadDatabase = LoadDatabase
    rclpy_callback_groups.ReentrantCallbackGroup = ReentrantCallbackGroup
    rclpy.callback_groups = rclpy_callback_groups

    std_srvs.srv = std_srvs_srv
    rtabmap_msgs.srv = rtabmap_msgs_srv

    monkeypatch.setitem(sys.modules, "std_srvs", std_srvs)
    monkeypatch.setitem(sys.modules, "std_srvs.srv", std_srvs_srv)
    monkeypatch.setitem(sys.modules, "rtabmap_msgs", rtabmap_msgs)
    monkeypatch.setitem(sys.modules, "rtabmap_msgs.srv", rtabmap_msgs_srv)
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.callback_groups", rclpy_callback_groups)

    return Empty, DetectMoreLoopClosures, GlobalBundleAdjustment, CleanupLocalGrids, PublishMap, LoadDatabase, ReentrantCallbackGroup


class _FakeWaypointStore:
    def __init__(self):
        self.created_maps = []

    def create_map(self, name=None):
        map_id = f"map-{len(self.created_maps) + 1}"
        self.created_maps.append({"map_id": map_id, "name": name})
        return map_id


def _load_mapping_module(monkeypatch):
    tools_pkg = types.ModuleType("rob_box_mcp_tools.tools")
    tools_pkg.__path__ = [str(Path(__file__).resolve().parents[2] / "rob_box_mcp_tools" / "tools")]
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.tools", tools_pkg)
    sys.modules.pop("rob_box_mcp_tools.tools.mapping", None)

    module_path = Path(__file__).resolve().parents[2] / "rob_box_mcp_tools" / "tools" / "mapping.py"
    spec = importlib.util.spec_from_file_location("rob_box_mcp_tools.tools.mapping", module_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.mark.unit
def test_optimize_map_uses_rtabmap_service_types(monkeypatch):
    empty, detect, bundle, cleanup, publish, _load_database, _cb_group = _install_fake_service_modules(monkeypatch)
    OptimizeMapTool = _load_mapping_module(monkeypatch).OptimizeMapTool

    node = _FakeNode()
    tool = OptimizeMapTool(node)

    assert node.clients["/rtabmap/rtabmap/detect_more_loop_closures"].srv_type is detect
    assert node.clients["/rtabmap/rtabmap/global_bundle_adjustment"].srv_type is bundle
    assert node.clients["/rtabmap/rtabmap/cleanup_local_grids"].srv_type is cleanup
    assert node.clients["/rtabmap/rtabmap/publish_map"].srv_type is publish
    assert node.clients["/rtabmap/rtabmap/backup"].srv_type is empty

    assert isinstance(tool._detect_more_loop_closures_request, detect.Request)
    assert isinstance(tool._global_bundle_adjustment_request, bundle.Request)
    assert isinstance(tool._cleanup_local_grids_request, cleanup.Request)
    assert isinstance(tool._publish_map_request, publish.Request)
    assert isinstance(tool._backup_request, empty.Request)


@pytest.mark.unit
def test_optimize_map_waits_for_services_and_publishes_map(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    OptimizeMapTool = _load_mapping_module(monkeypatch).OptimizeMapTool

    node = _FakeNode()
    tool = OptimizeMapTool(node)

    result = tool.execute()

    assert result.success is True
    assert "loop closures" in result.message
    assert "bundle adjustment" in result.message
    assert "cleanup grids" in result.message
    assert "publish map" in result.message
    assert "backup" in result.message

    assert node.clients["/rtabmap/rtabmap/detect_more_loop_closures"].call_count == 1
    assert node.clients["/rtabmap/rtabmap/global_bundle_adjustment"].call_count == 1
    assert node.clients["/rtabmap/rtabmap/cleanup_local_grids"].call_count == 1
    assert node.clients["/rtabmap/rtabmap/publish_map"].call_count == 1
    assert node.clients["/rtabmap/rtabmap/backup"].call_count == 1


@pytest.mark.unit
def test_finish_mapping_publishes_map_after_switching_to_localization(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    FinishMappingTool = _load_mapping_module(monkeypatch).FinishMappingTool

    node = _FakeNode()
    tool = FinishMappingTool(node)

    result = tool.execute()

    assert result.success is True
    assert node.clients["/rtabmap/rtabmap/set_mode_localization"].call_count == 1
    assert node.clients["/rtabmap/rtabmap/publish_map"].call_count == 1


@pytest.mark.unit
def test_start_mapping_new_location_fails_without_load_database_and_does_not_create_map(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    StartMappingTool = _load_mapping_module(monkeypatch).StartMappingTool

    node = _FakeNode()
    waypoint_store = _FakeWaypointStore()
    tool = StartMappingTool(node, waypoint_store=waypoint_store)
    node.clients["/rtabmap/rtabmap/load_database"].ready = False

    result = tool.execute(map_name="РАНХиГС", new_location=True)

    assert result.success is False
    assert "load_database" in result.error
    assert waypoint_store.created_maps == []


@pytest.mark.unit
def test_mapping_service_clients_use_reentrant_callback_group(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)

    node = _FakeNode()
    start_tool = module.StartMappingTool(node)
    finish_tool = module.FinishMappingTool(node)
    optimize_tool = module.OptimizeMapTool(node)

    assert start_tool.set_mode_mapping_client.kwargs.get("callback_group") is not None
    assert start_tool.load_database_client.kwargs.get("callback_group") is not None
    assert finish_tool.set_mode_localization_client.kwargs.get("callback_group") is not None
    assert finish_tool.publish_map_client.kwargs.get("callback_group") is not None
    assert optimize_tool.publish_map_client.kwargs.get("callback_group") is not None


# ---------------------------------------------------------------- #3789: честный исход


class _Resp:
    def __init__(self, success=True, message=""):
        self.success = success
        self.message = message


class _NeverDoneFuture:
    def add_done_callback(self, callback):
        pass

    def result(self):
        return None


class _FakeMappingState:
    def __init__(self):
        self.localization_calls = []

    def set_localization(self, map_name=None, map_id=None):
        self.localization_calls.append((map_name, map_id))


def _finish(monkeypatch, *, mode=object(), publish=object(), mode_ready=True, state=None):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    node = _FakeNode()
    tool = module.FinishMappingTool(node, mapping_state=state)
    node.clients["/rtabmap/rtabmap/set_mode_localization"].result_value = mode
    node.clients["/rtabmap/rtabmap/set_mode_localization"].ready = mode_ready
    node.clients["/rtabmap/rtabmap/publish_map"].result_value = publish
    return tool, node


@pytest.mark.unit
def test_finish_mapping_ok_on_service_reply_and_updates_state(monkeypatch):
    state = _FakeMappingState()
    tool, _ = _finish(monkeypatch, state=state)
    result = tool.execute(map_name="квартира")
    assert result.success is True
    assert state.localization_calls == [("квартира", None)]
    assert "опубликовать не удалось" not in result.message


@pytest.mark.unit
def test_finish_mapping_fails_when_service_missing(monkeypatch):
    state = _FakeMappingState()
    tool, node = _finish(monkeypatch, mode_ready=False, state=state)
    result = tool.execute()
    assert result.success is False
    assert "set_mode_localization" in result.error
    assert result.data == {"outcome": "unavailable"}
    assert state.localization_calls == []  # FSM не переключаем, пока режим не включился
    assert node.clients["/rtabmap/rtabmap/publish_map"].call_count == 0


@pytest.mark.unit
def test_finish_mapping_fails_on_service_timeout(monkeypatch):
    state = _FakeMappingState()
    tool, node = _finish(monkeypatch, state=state)
    client = node.clients["/rtabmap/rtabmap/set_mode_localization"]
    client.call_async = lambda request: _NeverDoneFuture()
    monkeypatch.setattr(sys.modules["rob_box_mcp_tools.tools.mapping"], "wait_future", lambda future, timeout_sec: False)
    result = tool.execute()
    assert result.success is False
    assert result.data == {"outcome": "timeout"}
    assert state.localization_calls == []


@pytest.mark.unit
def test_finish_mapping_fails_on_service_refusal(monkeypatch):
    tool, _ = _finish(monkeypatch, mode=_Resp(success=False, message="нет базы"))
    result = tool.execute()
    assert result.success is False
    assert result.data == {"outcome": "service_error"}
    assert "нет базы" in result.error


@pytest.mark.unit
def test_finish_mapping_fails_on_empty_reply(monkeypatch):
    tool, _ = _finish(monkeypatch, mode=None)
    result = tool.execute()
    assert result.success is False
    assert result.data == {"outcome": "no_response"}


@pytest.mark.unit
def test_finish_mapping_reports_failed_map_publication_but_stays_success(monkeypatch):
    tool, node = _finish(monkeypatch)
    node.clients["/rtabmap/rtabmap/publish_map"].ready = False
    result = tool.execute()
    assert result.success is True
    assert "опубликовать не удалось" in result.message


def _load_tool(monkeypatch, *, db_ready=True, db=object(), mode=object(), state=None, no_client=False):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    node = _FakeNode()
    tool = module.LoadMapTool(node, mapping_state=state)
    if no_client:
        tool.load_database_client = None
    else:
        node.clients["/rtabmap/rtabmap/load_database"].ready = db_ready
        node.clients["/rtabmap/rtabmap/load_database"].result_value = db
    node.clients["/rtabmap/rtabmap/set_mode_localization"].result_value = mode
    return tool, node


@pytest.mark.unit
def test_load_map_without_client_is_clear_error_not_exception(monkeypatch):
    tool, _ = _load_tool(monkeypatch, no_client=True)
    result = tool.execute()
    assert result.success is False
    assert "load_database" in result.error
    assert result.message == "Сервис карт недоступен"


@pytest.mark.unit
def test_load_map_fails_when_load_database_not_ready(monkeypatch):
    state = _FakeMappingState()
    tool, node = _load_tool(monkeypatch, db_ready=False, state=state)
    result = tool.execute()
    assert result.success is False and result.data == {"outcome": "unavailable"}
    assert state.localization_calls == []
    assert node.clients["/rtabmap/rtabmap/set_mode_localization"].call_count == 0


@pytest.mark.unit
def test_load_map_fails_when_localization_switch_refused(monkeypatch):
    state = _FakeMappingState()
    tool, _ = _load_tool(monkeypatch, mode=_Resp(success=False, message="занято"), state=state)
    result = tool.execute()
    assert result.success is False
    assert "режим навигации не включился" in result.message
    assert state.localization_calls == []


@pytest.mark.unit
def test_load_map_ok_on_both_replies(monkeypatch):
    state = _FakeMappingState()
    tool, node = _load_tool(monkeypatch, state=state)
    result = tool.execute(map_name="офис")
    assert result.success is True
    assert node.clients["/rtabmap/rtabmap/load_database"].call_count == 1
    assert node.clients["/rtabmap/rtabmap/set_mode_localization"].call_count == 1
    assert state.localization_calls == [("офис", None)]

# ---------------------------------------------------------------- #3789 хвост: continue / start / load по имени


class _RaisingFuture:
    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        raise RuntimeError("сервис упал")


class _NamedStore(_FakeWaypointStore):
    def __init__(self, known=(), raises=None):
        super().__init__()
        self.known = set(known)
        self.raises = raises
        self.activated = []

    def set_active_map_by_name(self, name):
        if self.raises:
            raise self.raises
        self.activated.append(name)
        return name in self.known

    def get_active_map_id(self):
        return "map-active"


class _FakeStartState:
    def __init__(self):
        self.mapping_calls = []

    def set_mapping(self, map_name=None, map_id=None):
        self.mapping_calls.append((map_name, map_id))


def _continue(monkeypatch, *, ready=True, result=object()):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    node = _FakeNode()
    tool = module.ContinueMappingTool(node)
    client = node.clients["/rtabmap/rtabmap/set_mode_mapping"]
    client.ready = ready
    client.result_value = result
    return tool, client


@pytest.mark.unit
def test_continue_mapping_ok_only_on_service_reply(monkeypatch):
    tool, client = _continue(monkeypatch)
    result = tool.execute()
    assert result.success is True and client.call_count == 1


@pytest.mark.unit
def test_continue_mapping_fails_when_service_missing(monkeypatch):
    tool, client = _continue(monkeypatch, ready=False)
    result = tool.execute()
    assert result.success is False and result.data == {"outcome": "unavailable"}
    assert client.call_count == 0 and "Продолжаю" not in (result.message or "")


@pytest.mark.unit
def test_continue_mapping_fails_on_timeout(monkeypatch):
    tool, client = _continue(monkeypatch)
    client.call_async = lambda request: _NeverDoneFuture()
    monkeypatch.setattr(sys.modules["rob_box_mcp_tools.tools.mapping"], "wait_future", lambda future, timeout_sec: False)
    result = tool.execute()
    assert result.success is False and result.data == {"outcome": "timeout"}
    assert "Продолжаю" not in (result.message or "")


@pytest.mark.unit
def test_continue_mapping_fails_on_empty_reply_and_refusal(monkeypatch):
    tool, _ = _continue(monkeypatch, result=None)
    empty = tool.execute()
    assert empty.success is False and empty.data == {"outcome": "no_response"}

    tool, _ = _continue(monkeypatch, result=_Resp(success=False, message="занято"))
    refused = tool.execute()
    assert refused.success is False and refused.data == {"outcome": "service_error"}
    assert "занято" in refused.error


@pytest.mark.unit
def test_continue_mapping_fails_when_future_raises(monkeypatch):
    tool, client = _continue(monkeypatch)
    client.call_async = lambda request: _RaisingFuture()
    result = tool.execute()
    assert result.success is False and result.data == {"outcome": "no_response"}
    assert "сервис упал" in result.error


def _start(monkeypatch, *, mode=object(), db=object(), mode_ready=True):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    node = _FakeNode()
    store = _FakeWaypointStore()
    state = _FakeStartState()
    tool = module.StartMappingTool(node, waypoint_store=store, mapping_state=state)
    node.clients["/rtabmap/rtabmap/set_mode_mapping"].result_value = mode
    node.clients["/rtabmap/rtabmap/set_mode_mapping"].ready = mode_ready
    node.clients["/rtabmap/rtabmap/load_database"].result_value = db
    return tool, node, store, state


@pytest.mark.unit
def test_start_mapping_ok_creates_map_and_switches_state(monkeypatch):
    tool, _, store, state = _start(monkeypatch)
    result = tool.execute(map_name="кухня", new_location=True)
    assert result.success is True
    assert [m["name"] for m in store.created_maps] == ["кухня"]
    assert state.mapping_calls == [("кухня", "map-1")]


@pytest.mark.unit
@pytest.mark.parametrize(
    "kwargs, outcome",
    [
        ({"mode": None}, "no_response"),
        ({"mode": _Resp(success=False, message="нет")}, "service_error"),
        ({"mode_ready": False}, "unavailable"),
        ({"db": None}, "no_response"),
        ({"db": _Resp(success=False, message="файл")}, "service_error"),
    ],
)
def test_start_mapping_without_service_reply_creates_nothing(monkeypatch, kwargs, outcome):
    tool, node, store, state = _start(monkeypatch, **kwargs)
    result = tool.execute(map_name="кухня", new_location=True)
    assert result.success is False and result.data == {"outcome": outcome}
    assert store.created_maps == [] and state.mapping_calls == []
    assert "Начинаю" not in (result.message or "")


@pytest.mark.unit
def test_load_map_unknown_name_fails_and_reloads_nothing(monkeypatch):
    store = _NamedStore(known=["офис"])
    state = _FakeMappingState()
    tool, node = _load_tool(monkeypatch, state=state)
    tool.waypoint_store = store
    result = tool.execute(map_name="дача")
    assert result.success is False and result.data == {"outcome": "map_not_found"}
    assert result.message == "Карта «дача» не найдена, ничего не загружал"
    assert node.clients["/rtabmap/rtabmap/load_database"].call_count == 0
    assert node.clients["/rtabmap/rtabmap/set_mode_localization"].call_count == 0
    assert state.localization_calls == []


@pytest.mark.unit
def test_load_map_known_name_activates_and_loads(monkeypatch):
    store = _NamedStore(known=["офис"])
    state = _FakeMappingState()
    tool, node = _load_tool(monkeypatch, state=state)
    tool.waypoint_store = store
    result = tool.execute(map_name="офис")
    assert result.success is True
    assert store.activated == ["офис"]
    assert node.clients["/rtabmap/rtabmap/load_database"].call_count == 1
    assert state.localization_calls == [("офис", "map-active")]


@pytest.mark.unit
def test_load_map_store_error_fails_and_reloads_nothing(monkeypatch):
    store = _NamedStore(raises=RuntimeError("база повреждена"))
    tool, node = _load_tool(monkeypatch)
    tool.waypoint_store = store
    result = tool.execute(map_name="офис")
    assert result.success is False and result.data == {"outcome": "map_lookup_failed"}
    assert "база повреждена" in result.error
    assert node.clients["/rtabmap/rtabmap/load_database"].call_count == 0


@pytest.mark.unit
def test_load_map_without_name_reloads_current_even_if_store_knows_nothing(monkeypatch):
    store = _NamedStore()
    tool, node = _load_tool(monkeypatch)
    tool.waypoint_store = store
    result = tool.execute()
    assert result.success is True and store.activated == []
    assert node.clients["/rtabmap/rtabmap/load_database"].call_count == 1
