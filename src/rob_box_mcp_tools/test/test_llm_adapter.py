"""Regression tests for LLMToolCallAdapter supervisor-thread contract (issue #2131).

Background
----------
``avatar_supervisor`` cannot complete a single MCP tool call: every tool
request times out at 10 s, yet the response arrives 1.3–1.7 s *after* the
timeout. Root cause is a deadlock between the executor and the subscriber:

* ``supervisor_node.main()`` spun the node on a ``SingleThreadedExecutor``
  (``rclpy.spin(node)``).
* ``LLMToolCallAdapter`` subscribes to ``/mcp/result`` on a
  ``ReentrantCallbackGroup`` and blocks the *only* thread in
  ``execute_tool_call_sync()`` via ``threading.Event.wait(timeout)``.
* With one thread, ``on_result()`` cannot fire while the same thread is
  blocked in ``wait()``. The callback finally lands **after** the wait
  returns with a timeout — exactly the 1.3–1.7 s post-timeout arrival.

The fix in ADR-0072 switches ``avatar_supervisor`` to
``rclpy.executors.MultiThreadedExecutor`` (mirroring ``dialogue_node``).
These tests pin that contract so the deadlock can't sneak back.

Two-layer regression
--------------------
The "callback is delivered while the main thread is blocked" guarantee can
only be exercised against a *real* ``rclpy`` + ``MultiThreadedExecutor``,
so the integration test is gated behind ``pytest.importorskip("rclpy")``.

The unit layer pins the structural pre-conditions that make the
integration test meaningful, and runs anywhere — including developer
laptops and CI images without ROS. The structural tests are the
primary regression gate; the integration test confirms end-to-end
delivery when ROS is available.
"""

from __future__ import annotations

import ast
import importlib.util
import json
import os
import sys
import threading
import time
import types
from pathlib import Path

import pytest


# ---------------------------------------------------------------------------
# rclpy stub for environments without ROS (developer laptops, CI without ROS).
# Loaded BEFORE the SUT import so ``import rclpy.node`` etc. succeed.
# Mirrors the pattern from ``test_mcp_server_speaker_result.py``.
# ---------------------------------------------------------------------------


def _ensure_rclpy_stub() -> bool:
    """Install a minimal ``rclpy`` / ``std_msgs`` shim if rclpy isn't importable.

    Returns True if the stub was installed (rclpy is NOT real), False if
    rclpy is real. Layer 2 uses this to skip when only the stub is
    available — the integration test requires a real ROS environment.
    """
    try:
        import rclpy  # noqa: F401
        # Real rclpy: check it actually has C-backed executors.
        try:
            from rclpy.executors import MultiThreadedExecutor  # noqa: F401
        except ImportError:
            pass
        return False
    except ImportError:
        pass

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **kw: None  # type: ignore[attr-defined]
    rclpy.shutdown = lambda *a, **kw: None  # type: ignore[attr-defined]
    rclpy.ok = lambda: True  # type: ignore[attr-defined]

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object  # type: ignore[attr-defined]

    rclpy_qos = types.ModuleType("rclpy.qos")

    class _QoSProfile:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    rclpy_qos.QoSProfile = _QoSProfile  # type: ignore[attr-defined]
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(  # type: ignore[attr-defined]
        RELIABLE="reliable", BEST_EFFORT="best_effort"
    )
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")  # type: ignore[attr-defined]

    rclpy_cb = types.ModuleType("rclpy.callback_groups")
    rclpy_cb.ReentrantCallbackGroup = type(  # type: ignore[attr-defined]
        "ReentrantCallbackGroup", (), {}
    )

    rclpy_exec = types.ModuleType("rclpy.executors")

    class _SingleThreadedExecutor:
        def __init__(self, *a, **kw):
            pass

        def add_node(self, node):
            pass

        def spin(self):
            pass

        def shutdown(self, timeout_sec=None):
            pass

    class _MultiThreadedExecutor(_SingleThreadedExecutor):
        pass

    rclpy_exec.SingleThreadedExecutor = _SingleThreadedExecutor  # type: ignore[attr-defined]
    rclpy_exec.MultiThreadedExecutor = _MultiThreadedExecutor  # type: ignore[attr-defined]
    sys.modules["rclpy.executors"] = rclpy_exec

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class String:
        def __init__(self, *a, **kw):
            self.data = ""

    std_msgs_msg.String = String  # type: ignore[attr-defined]
    std_msgs.msg = std_msgs_msg  # type: ignore[attr-defined]

    for mod in (
        rclpy,
        rclpy_node,
        rclpy_qos,
        rclpy_cb,
        std_msgs,
        std_msgs_msg,
    ):
        sys.modules[mod.__name__] = mod
    sys.modules["std_msgs.msg"] = std_msgs_msg
    return True


# Module-level cache so we don't import rclpy twice; needed by Layer 2 to
# decide whether to skip without paying the import cost on every fixture.
_RCLPY_IS_REAL: bool = not _ensure_rclpy_stub()


@pytest.fixture(autouse=True)
def _stub_rclpy(monkeypatch):
    """Auto-install rclpy stub if missing; set harmless auth defaults."""
    _ensure_rclpy_stub()
    # Both env vars are read by RequestAuthenticator.from_env; we only need
    # a non-empty token so the constructor does not warn loudly on import.
    monkeypatch.setenv("ROB_BOX_MCP_TOKEN", "test-token-for-unit-tests-not-real")
    yield


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _read_source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _callback_group_aware_node(MockNodeCls, node_name: str = "test_supervisor"):
    """Wrap ``MockNode`` so it accepts the ``callback_group=`` kwarg
    that ``LLMToolCallAdapter`` passes to ``create_subscription``."""

    class _Node(MockNodeCls):
        def get_name(self) -> str:
            # rclpy.node.Node.get_name(); required by LLMToolCallAdapter.__init__
            # since #2132 (sender derived from node identity).
            return getattr(self, "node_name", node_name)

        def create_subscription(self, msg_type, topic, callback, qos=10, **_kw):
            return MockNodeCls.create_subscription(
                self, msg_type, topic, callback, qos
            )

        def create_publisher(self, msg_type, topic, qos=10, **_kw):
            return MockNodeCls.create_publisher(self, msg_type, topic, qos)

    return _Node(node_name)


@pytest.fixture
def callback_group_aware_node():
    """Yield a node compatible with ``LLMToolCallAdapter`` (mock rclpy)."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from conftest import MockNode

    return _callback_group_aware_node(MockNode)


# ---------------------------------------------------------------------------
# Layer 1: structural regressions (no rclpy required, runs everywhere)
# ---------------------------------------------------------------------------


class TestSupervisorExecutorContract:
    """Pin ``supervisor_node.main`` to MultiThreadedExecutor.

    On the buggy code, ``main()`` calls ``rclpy.spin(node)`` (which is a
    ``SingleThreadedExecutor``) and the /mcp/result callback starves. The
    fix is to wrap the node in ``rclpy.executors.MultiThreadedExecutor``
    explicitly, mirroring ``dialogue_node.main``.

    This test fails on the buggy code and passes on the fixed code,
    satisfying issue #2131 Definition of Done ("test that fails on the
    current code and is green after the fix").
    """

    @staticmethod
    def _supervisor_main_path() -> Path:
        return (
            Path(__file__).resolve().parents[2]
            / "rob_box_supervisor"
            / "rob_box_supervisor"
            / "supervisor_node.py"
        )

    def test_supervisor_path_resolution(self):
        """Sanity check: the resolved path exists."""
        assert self._supervisor_main_path().exists(), (
            f"resolved supervisor_node.py not found at {self._supervisor_main_path()}"
        )

    def test_supervisor_main_uses_multithreaded_executor(self):
        path = self._supervisor_main_path()
        assert path.exists(), f"supervisor_node.py not found at {path}"

        source = _read_source(path)
        tree = ast.parse(source)

        main_fn = next(
            (
                node
                for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name == "main"
            ),
            None,
        )
        assert main_fn is not None, "supervisor_node.main() not found"

        # Walk the AST of main() and look for the two structural tells.
        # 1) NO bare ``rclpy.spin(node)`` call (that is SingleThreadedExecutor).
        # 2) AT LEAST ONE reference to ``MultiThreadedExecutor``.
        has_bare_spin_call = False
        has_mte_reference = False

        for sub in ast.walk(main_fn):
            if isinstance(sub, ast.Call):
                func = sub.func
                if (
                    isinstance(func, ast.Attribute)
                    and func.attr == "spin"
                    and isinstance(func.value, ast.Name)
                    and func.value.id == "rclpy"
                ):
                    has_bare_spin_call = True
                if (
                    isinstance(func, ast.Attribute)
                    and func.attr == "MultiThreadedExecutor"
                ):
                    has_mte_reference = True

        assert not has_bare_spin_call, (
            "supervisor_node.main() must NOT call rclpy.spin(node): "
            "that is SingleThreadedExecutor and starves /mcp/result callback "
            "while execute_tool_call_sync blocks the main thread (issue #2131)."
        )
        assert has_mte_reference, (
            "supervisor_node.main() must reference MultiThreadedExecutor "
            "(mirroring dialogue_node.main) so the ReentrantCallbackGroup "
            "actually dispatches on_result from a worker thread."
        )


class TestLLMAdapterStructuralContract:
    """Pin the adapter side of the contract.

    ``LLMToolCallAdapter`` must subscribe to /mcp/result on a
    ReentrantCallbackGroup. Without it, switching the supervisor to
    MultiThreadedExecutor alone is not enough — the subscription would
    still belong to the default MutuallyExclusiveCallbackGroup and would
    not run concurrently with execute_tool_call_sync's wait().
    """

    @staticmethod
    def _llm_adapter_path() -> Path:
        return (
            Path(__file__).resolve().parents[1]
            / "rob_box_mcp_tools"
            / "llm_adapter.py"
        )

    def test_llm_adapter_path_resolution(self):
        """Sanity check: the resolved path exists. (If this fails, the
        `parents[N]` walk above is broken because the file moved.)"""
        assert self._llm_adapter_path().exists(), (
            f"resolved llm_adapter.py not found at {self._llm_adapter_path()}"
        )

    def test_llm_adapter_uses_reentrant_callback_group_for_mcp_result(self):
        path = self._llm_adapter_path()
        assert path.exists(), f"llm_adapter.py not found at {path}"

        source = _read_source(path)
        tree = ast.parse(source)

        cls = next(
            (
                node
                for node in ast.iter_child_nodes(tree)
                if isinstance(node, ast.ClassDef)
                and node.name == "LLMToolCallAdapter"
            ),
            None,
        )
        assert cls is not None, "LLMToolCallAdapter class not found"

        init_fn = next(
            (
                n
                for n in cls.body
                if isinstance(n, ast.FunctionDef) and n.name == "__init__"
            ),
            None,
        )
        assert init_fn is not None, "LLMToolCallAdapter.__init__ not found"

        init_src = ast.unparse(init_fn)
        assert "ReentrantCallbackGroup" in init_src, (
            "LLMToolCallAdapter.__init__ must instantiate "
            "ReentrantCallbackGroup — otherwise /mcp/result callbacks "
            "serialize behind the main thread even under MTE."
        )
        assert "/mcp/result" in init_src, (
            "LLMToolCallAdapter.__init__ must subscribe to /mcp/result."
        )
        assert "callback_group" in init_src, (
            "LLMToolCallAdapter.__init__ must pass callback_group= when "
            "creating the /mcp/result subscription; the default group would "
            "serialize the callback behind the main thread."
        )


# ---------------------------------------------------------------------------
# Layer 1b: behavioural unit test (mock rclpy, runs everywhere)
# ---------------------------------------------------------------------------


class TestExecuteToolCallSyncEventContract:
    """The wait()/Event.set() handshake in execute_tool_call_sync.

    This does not depend on the executor type. It pins the lower-level
    contract that the integration test below relies on: a callback fired
    from any thread must wake up ``execute_tool_call_sync`` via the
    ``result_event`` and deliver the cached result.
    """

    def test_execute_tool_call_sync_returns_when_callback_fires(
        self, callback_group_aware_node
    ):
        from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter
        from std_msgs.msg import String as StringMsg  # type: ignore

        adapter = LLMToolCallAdapter(callback_group_aware_node)
        adapter.timeout = 2.0  # generous for slow CI

        delivered: list[str] = []
        ready = threading.Event()

        def deliver_result():
            ready.wait(timeout=2.0)
            # Wait for execute_tool_call_sync to actually publish the request.
            # The mock node stores messages in MockPublisher.published_messages;
            # poll for up to 2 s.
            pub = callback_group_aware_node.get_publisher("/mcp/execute")
            assert pub is not None, "execute publisher missing"
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not pub.published_messages:
                time.sleep(0.005)
            assert pub.published_messages, (
                "execute_tool_call_sync did not publish within 2 s"
            )
            request = json.loads(pub.published_messages[-1].data)
            request_id = request["request_id"]

            msg = StringMsg()
            msg.data = json.dumps(
                {
                    "request_id": request_id,
                    "result": {"success": True, "data": {"value": 42}},
                }
            )
            adapter.on_result(msg)
            delivered.append(request_id)

        t = threading.Thread(target=deliver_result, daemon=True)
        t.start()
        ready.set()  # tell deliver_result to start the round trip

        result = adapter.execute_tool_call_sync(
            "test_tool", {"foo": "bar"}, timeout=3.0
        )
        t.join(timeout=2.0)

        assert delivered, "deliver_result thread did not run"
        assert result == {"success": True, "data": {"value": 42}}, (
            f"execute_tool_call_sync returned {result!r} instead of the "
            "delivered payload — the Event.set()/wait() handshake is broken."
        )

    def test_execute_tool_call_sync_times_out_when_callback_never_fires(
        self, callback_group_aware_node
    ):
        """Negative case: if on_result never fires, execute_tool_call_sync
        must time out and return the structured error. This pins the
        error contract that the supervisor's response builder relies on."""
        from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter

        adapter = LLMToolCallAdapter(callback_group_aware_node)
        adapter.timeout = 0.3

        result = adapter.execute_tool_call_sync(
            "test_tool", {"foo": "bar"}, timeout=0.3
        )

        assert result == {
            "success": False,
            "error": "Timeout ожидания результата инструмента",
        }, (
            f"execute_tool_call_sync returned {result!r} on timeout — the "
            "timeout error shape changed and may break callers that detect "
            "timeout by inspecting the payload."
        )


# ---------------------------------------------------------------------------
# Layer 2: end-to-end with real rclpy + MultiThreadedExecutor (skipped if
# rclpy is not installed).
# ---------------------------------------------------------------------------


@pytest.fixture
def real_node_and_executor(request):
    """Spin up a real rclpy node + MultiThreadedExecutor in a worker thread.

    Skipped automatically when rclpy is not importable (developer laptops,
    CI images without ROS). On the robot (with ROS installed), this test
    is the one that actually proves the supervisor fix works.
    """
    if not _RCLPY_IS_REAL:
        pytest.skip(
            "rclpy is mocked (not the real C-backed library) — Layer 2 "
            "integration test requires a real ROS environment"
        )
    rclpy = pytest.importorskip("rclpy")

    if not rclpy.ok():
        rclpy.init()
    from rclpy.executors import MultiThreadedExecutor

    node = rclpy.create_node("test_llm_adapter_e2e")

    from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter

    adapter = LLMToolCallAdapter(node)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    yield node, adapter

    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


class TestExecutorContractIntegration:
    """End-to-end: a tool call must complete on MultiThreadedExecutor.

    This is the test that *would* fail under the original SingleThreaded
    supervisor: with one thread blocked in wait(), the executor cannot
    dispatch the /mcp/result callback, and the call times out.
    """

    def test_tool_call_completes_on_multithreaded_executor(
        self, real_node_and_executor
    ):
        node, adapter = real_node_and_executor
        from std_msgs.msg import String  # type: ignore
        from rclpy.callback_groups import (  # type: ignore
            ReentrantCallbackGroup,
        )
        from rclpy.qos import (  # type: ignore
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        # Subscribe to /mcp/execute so we can capture the request_id, then
        # publish the matching /mcp/result. Both subscriptions share the
        # node so we don't need any cross-process plumbing.

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        cb_group = ReentrantCallbackGroup()

        request_id_box: list[str] = []

        def _on_execute(msg: String) -> None:
            try:
                payload = json.loads(msg.data)
                request_id_box.append(payload["request_id"])
            except Exception:  # noqa: BLE001
                pass

        execute_sub = node.create_subscription(
            String, "/mcp/execute", _on_execute, qos, callback_group=cb_group
        )
        result_pub = node.create_publisher(String, "/mcp/result", qos)

        def deliver_when_ready():
            # Poll up to 2 s for the request to land.
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not request_id_box:
                time.sleep(0.01)
            if not request_id_box:
                return
            time.sleep(0.05)  # ensure wait() is blocked by now
            rid = request_id_box[-1]
            msg = String()
            msg.data = json.dumps(
                {
                    "request_id": rid,
                    "result": {"success": True, "data": {"value": 99}},
                }
            )
            result_pub.publish(msg)

        threading.Thread(target=deliver_when_ready, daemon=True).start()

        start = time.monotonic()
        result = adapter.execute_tool_call_sync(
            "test_tool", {"foo": "bar"}, timeout=2.0
        )
        elapsed = time.monotonic() - start

        execute_sub.destroy()
        result_pub.destroy()

        assert result == {"success": True, "data": {"value": 99}}, (
            f"execute_tool_call_sync returned {result!r} after {elapsed:.3f}s "
            "— expected the delivered payload. Under SingleThreadedExecutor "
            "this times out because the main thread is blocked in wait()."
        )
        assert elapsed < 2.0, (
            f"Tool call took {elapsed:.3f}s, expected < 2 s. Slow path means "
            "the MTE callback delivery is degraded — investigate before merge."
        )
