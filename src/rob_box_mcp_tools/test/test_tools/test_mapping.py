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

    def create_client(self, srv_type, srv_name):
        client = _FakeClient(srv_type, srv_name)
        self.clients[srv_name] = client
        return client


def _install_fake_service_modules(monkeypatch):
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    rtabmap_msgs = types.ModuleType("rtabmap_msgs")
    rtabmap_msgs_srv = types.ModuleType("rtabmap_msgs.srv")

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

    std_srvs_srv.Empty = Empty
    rtabmap_msgs_srv.DetectMoreLoopClosures = DetectMoreLoopClosures
    rtabmap_msgs_srv.GlobalBundleAdjustment = GlobalBundleAdjustment
    rtabmap_msgs_srv.CleanupLocalGrids = CleanupLocalGrids
    rtabmap_msgs_srv.PublishMap = PublishMap

    std_srvs.srv = std_srvs_srv
    rtabmap_msgs.srv = rtabmap_msgs_srv

    monkeypatch.setitem(sys.modules, "std_srvs", std_srvs)
    monkeypatch.setitem(sys.modules, "std_srvs.srv", std_srvs_srv)
    monkeypatch.setitem(sys.modules, "rtabmap_msgs", rtabmap_msgs)
    monkeypatch.setitem(sys.modules, "rtabmap_msgs.srv", rtabmap_msgs_srv)

    return Empty, DetectMoreLoopClosures, GlobalBundleAdjustment, CleanupLocalGrids, PublishMap


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
    empty, detect, bundle, cleanup, publish = _install_fake_service_modules(monkeypatch)
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