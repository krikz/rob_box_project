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


class _NeverFuture:
    """Future, который никогда не вызывает done_callback (для timeout-тестов)."""

    def __init__(self, result=None):
        self._result = result

    def add_done_callback(self, callback):
        pass

    def result(self):
        return self._result


class _FakeState:
    def __init__(self):
        self.mapping_calls = []
        self.localization_calls = []

    def is_mapping(self):
        return False

    def set_mapping(self, map_name=None, map_id=None):
        self.mapping_calls.append((map_name, map_id))

    def set_localization(self, map_name=None, map_id=None):
        self.localization_calls.append((map_name, map_id))


class _RenameStore:
    """WaypointStore с rename_map / get_active_map_id / set_active_map_by_name."""

    def __init__(self, map_id="map-1", found=True):
        self.map_id = map_id
        self.found = found
        self.renamed = []
        self.created = []

    def create_map(self, name=None):
        self.created.append(name)
        return "map-new"

    def get_active_map_id(self):
        return self.map_id

    def rename_map(self, map_id, name):
        self.renamed.append((map_id, name))

    def set_active_map_by_name(self, name):
        return self.found


@pytest.mark.unit
def test_start_mapping_success_new_location_creates_map(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    StartMappingTool = module.StartMappingTool

    node = _FakeNode()
    store = _RenameStore()
    state = _FakeState()
    tool = StartMappingTool(node, waypoint_store=store, mapping_state=state)

    result = tool.execute(map_name="квартира")

    assert result.success is True
    assert result.data == {"map_id": "map-new"}
    assert "квартира" in result.message
    assert store.created == ["квартира"]
    assert state.mapping_calls == [("квартира", "map-new")]
    # backup и set_mode_mapping и load_database вызваны
    assert node.clients["/rtabmap/rtabmap/backup"].call_count == 1
    assert node.clients["/rtabmap/rtabmap/set_mode_mapping"].call_count == 1
    assert node.clients["/rtabmap/rtabmap/load_database"].call_count == 1


@pytest.mark.unit
def test_start_mapping_continue_existing_map(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    StartMappingTool = module.StartMappingTool

    node = _FakeNode()
    store = _RenameStore()
    tool = StartMappingTool(node, waypoint_store=store)

    result = tool.execute(new_location=False)

    assert result.success is True
    assert result.data is None  # no new map created
    assert "существующей карте" in result.message
    assert store.created == []


@pytest.mark.unit
def test_start_mapping_set_mode_not_ready(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    StartMappingTool = module.StartMappingTool

    node = _FakeNode()
    tool = StartMappingTool(node)
    node.clients["/rtabmap/rtabmap/set_mode_mapping"].ready = False

    result = tool.execute()

    assert result.success is False
    assert "set_mode_mapping service не готов" in result.error


@pytest.mark.unit
def test_start_mapping_load_database_not_ready(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    StartMappingTool = module.StartMappingTool

    node = _FakeNode()
    tool = StartMappingTool(node)
    node.clients["/rtabmap/rtabmap/load_database"].ready = False

    result = tool.execute()

    assert result.success is False
    assert "load_database service не готов" in result.error


@pytest.mark.unit
def test_start_mapping_backup_failure_continues(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    StartMappingTool = module.StartMappingTool

    node = _FakeNode()
    tool = StartMappingTool(node)
    node.clients["/rtabmap/rtabmap/backup"].ready = False  # backup пропускается

    result = tool.execute(new_location=False)

    assert result.success is True  # работаем без backup
    assert "резервной копии" in result.message


@pytest.mark.unit
def test_start_mapping_set_mode_timeout(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    StartMappingTool = module.StartMappingTool

    node = _FakeNode()
    tool = StartMappingTool(node)
    node.clients["/rtabmap/rtabmap/set_mode_mapping"].result_value = None
    # call_async возвращает future, который никогда не завершится
    tool.set_mode_mapping_client.call_async = lambda req: _NeverFuture()

    result = tool.execute()

    assert result.success is False
    assert "Таймаут ожидания set_mode_mapping" in result.error


@pytest.mark.unit
def test_continue_mapping_success(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    ContinueMappingTool = module.ContinueMappingTool

    node = _FakeNode()
    tool = ContinueMappingTool(node)

    result = tool.execute()

    assert result.success is True
    assert "Продолжаю исследование" in result.message
    assert node.clients["/rtabmap/rtabmap/set_mode_mapping"].call_count == 1


@pytest.mark.unit
def test_continue_mapping_service_not_ready(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    ContinueMappingTool = module.ContinueMappingTool

    node = _FakeNode()
    tool = ContinueMappingTool(node)
    node.clients["/rtabmap/rtabmap/set_mode_mapping"].ready = False

    result = tool.execute()

    assert result.success is False
    assert "set_mode service не готов" in result.error


@pytest.mark.unit
def test_finish_mapping_renames_map(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    FinishMappingTool = module.FinishMappingTool

    node = _FakeNode()
    store = _RenameStore()
    state = _FakeState()
    tool = FinishMappingTool(node, waypoint_store=store, mapping_state=state)

    result = tool.execute(map_name="квартира")

    assert result.success is True
    assert store.renamed == [("map-1", "квартира")]
    assert state.localization_calls == [("квартира", "map-1")]
    assert node.clients["/rtabmap/rtabmap/set_mode_localization"].call_count == 1
    assert node.clients["/rtabmap/rtabmap/publish_map"].call_count == 1


@pytest.mark.unit
def test_finish_mapping_services_not_ready_warns_but_succeeds(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    FinishMappingTool = module.FinishMappingTool

    node = _FakeNode()
    tool = FinishMappingTool(node)
    node.clients["/rtabmap/rtabmap/set_mode_localization"].ready = False
    node.clients["/rtabmap/rtabmap/publish_map"].ready = False

    result = tool.execute()

    assert result.success is True  # graceful — предупреждения, не краш
    assert "Заканчиваю исследование" in result.message


@pytest.mark.unit
def test_finish_mapping_localization_timeout(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    FinishMappingTool = module.FinishMappingTool

    node = _FakeNode()
    tool = FinishMappingTool(node)
    tool.set_mode_localization_client.call_async = lambda req: _NeverFuture()

    result = tool.execute()

    assert result.success is True  # timeout → warning, но не краш
    assert "Заканчиваю исследование" in result.message


@pytest.mark.unit
def test_optimize_map_partial_failure(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    OptimizeMapTool = module.OptimizeMapTool

    node = _FakeNode()
    tool = OptimizeMapTool(node)
    # loop closures не готов → попадёт в failed, остальные — completed
    node.clients["/rtabmap/rtabmap/detect_more_loop_closures"].ready = False

    result = tool.execute()

    assert result.success is True
    assert "loop closures" not in result.message
    assert "bundle adjustment" in result.message
    assert "Не удалось: detect_more_loop_closures" in result.message


@pytest.mark.unit
def test_optimize_map_all_fail(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    OptimizeMapTool = module.OptimizeMapTool

    node = _FakeNode()
    tool = OptimizeMapTool(node)
    for srv in [
        "detect_more_loop_closures",
        "global_bundle_adjustment",
        "cleanup_local_grids",
        "publish_map",
        "backup",
    ]:
        node.clients[f"/rtabmap/rtabmap/{srv}"].ready = False

    result = tool.execute()

    assert result.success is False
    assert "RTABMap оптимизация не выполнена" in result.error


@pytest.mark.unit
def test_optimize_map_service_timeout_falls_to_failed(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    OptimizeMapTool = module.OptimizeMapTool

    node = _FakeNode()
    tool = OptimizeMapTool(node)
    tool.loop_closures_client.call_async = lambda req: _NeverFuture()

    result = tool.execute()

    assert result.success is True
    assert "Не удалось: detect_more_loop_closures" in result.message


@pytest.mark.unit
def test_load_map_success(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    LoadMapTool = module.LoadMapTool

    node = _FakeNode()
    store = _RenameStore(found=True)
    state = _FakeState()
    tool = LoadMapTool(node, waypoint_store=store, mapping_state=state)

    result = tool.execute(map_name="квартира")

    assert result.success is True
    assert "Карта: 'квартира'" in result.message
    assert node.clients["/rtabmap/rtabmap/load_database"].call_count == 1
    assert node.clients["/rtabmap/rtabmap/set_mode_localization"].call_count == 1
    assert state.localization_calls == [("квартира", "map-1")]


@pytest.mark.unit
def test_load_map_waypoint_not_found_warns(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    LoadMapTool = module.LoadMapTool

    node = _FakeNode()
    store = _RenameStore(found=False)
    tool = LoadMapTool(node, waypoint_store=store)

    result = tool.execute(map_name="несуществующая")

    assert result.success is True  # предупреждение, но продолжаем
    assert "Карта: 'несуществующая'" in result.message


@pytest.mark.unit
def test_load_map_services_not_ready_warns(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    LoadMapTool = module.LoadMapTool

    node = _FakeNode()
    tool = LoadMapTool(node)
    node.clients["/rtabmap/rtabmap/load_database"].ready = False
    node.clients["/rtabmap/rtabmap/set_mode_localization"].ready = False

    result = tool.execute()

    assert result.success is True
    assert "Карта загружена" in result.message


@pytest.mark.unit
def test_load_map_waypoint_store_missing_method(monkeypatch):
    _install_fake_service_modules(monkeypatch)
    module = _load_mapping_module(monkeypatch)
    LoadMapTool = module.LoadMapTool

    class _OldStore:
        def __init__(self):
            self.raised = False

        def get_active_map_id(self):
            return None

        def set_active_map_by_name(self, name):
            self.raised = True
            raise AttributeError("no method")

    node = _FakeNode()
    store = _OldStore()
    tool = LoadMapTool(node, waypoint_store=store)

    result = tool.execute(map_name="квартира")

    assert result.success is True
    assert store.raised is True  # AttributeError перехвачен