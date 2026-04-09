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