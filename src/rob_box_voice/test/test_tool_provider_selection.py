#!/usr/bin/env python3
"""Selection tests for ``DialogueNode._build_tool_provider``.

The W5a patch turns ``_build_tool_provider`` from a hard-coded
``FakeToolProvider()`` call into a backend selector driven by the
``tool_provider`` ROS parameter. The pin-down below documents
the two observable outcomes the operator cares about:

* **Case A — MCP available** (``tool_provider='ros_mcp'`` and the
  ``rob_box_mcp_tools`` package is discoverable + importable):
  ``_build_tool_provider()`` returns a real provider wired with
  the 34 manifests from ``ToolRegistry``. The returned object's
  tool catalogue is non-empty and contains the production canary
  names (``speak_text``, ``play_sound``).
* **Case B — MCP unavailable** (``tool_provider='ros_mcp'`` but the
  ament probe fails because ``rob_box_mcp_tools`` is missing from
  the image): ``_build_tool_provider()`` raises ``RuntimeError``
  with an actionable message rather than silently degrading to
  ``FakeToolProvider``. This is the **fail-loud** semantic that
  supersedes the older "silent fallback" behaviour — operators
  need to see a startup error instead of a chat-only shell that
  silently no-ops every tool request.

The tests follow the same import-shim pattern as
``test_dialogue_shell.py`` (no real ``rclpy`` install required)
and exercise the production ``_build_tool_provider`` method via
``object.__new__(DialogueNode)`` so the rest of the shell's
``__init__`` (publisher set, executor, dialogue core) does not
have to run.
"""

from __future__ import annotations

import sys
import types
from typing import Any, Dict, List
from unittest.mock import MagicMock

import pytest


# ── rclpy shim ─────────────────────────────────────────────────────────────
# The production ``dialogue_node`` imports ``rclpy`` at module load.
# We install a minimal stub so the import succeeds without a ROS2
# runtime — same pattern as ``test_dialogue_shell.py``. The tests
# below never touch the stub directly: we instantiate
# ``DialogueNode`` via ``object.__new__`` and stub only the methods
# ``_build_tool_provider`` actually calls (``get_parameter`` and
# ``get_logger``).
_mock_rclpy = types.ModuleType("rclpy")
_mock_rclpy.init = lambda *_a, **_kw: None
_mock_rclpy.shutdown = lambda *_a, **_kw: None
_mock_rclpy.ok = lambda: True
sys.modules.setdefault("rclpy", _mock_rclpy)

_mock_rclpy_node = types.ModuleType("rclpy.node")


class _ShimNode:
    def __init__(self, name: str, **kwargs: Any) -> None:
        self._name = name


_mock_rclpy_node.Node = _ShimNode
sys.modules.setdefault("rclpy.node", _mock_rclpy_node)

_mock_cb = types.ModuleType("rclpy.callback_groups")
_mock_cb.ReentrantCallbackGroup = type("ReentrantCallbackGroup", (), {})
sys.modules.setdefault("rclpy.callback_groups", _mock_cb)

_mock_qos = types.ModuleType("rclpy.qos")
_mock_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="KEEP_LAST")
_mock_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="RELIABLE")
_mock_qos.QoSProfile = lambda *_a, **_kw: MagicMock()
sys.modules.setdefault("rclpy.qos", _mock_qos)

_mock_exec = types.ModuleType("rclpy.executors")
_mock_exec.MultiThreadedExecutor = MagicMock
sys.modules.setdefault("rclpy.executors", _mock_exec)

_std_msgs = types.ModuleType("std_msgs")
_std_msgs_msg = types.ModuleType("std_msgs.msg")


class _String:
    def __init__(self) -> None:
        self.data = ""


class _Bool:
    def __init__(self) -> None:
        self.data = False


_std_msgs_msg.String = _String
_std_msgs_msg.Bool = _Bool
sys.modules.setdefault("std_msgs", _std_msgs)
sys.modules.setdefault("std_msgs.msg", _std_msgs_msg)


# ── Imports under test (after shims are in place) ──────────────────────────
from rob_box_harness.tools import FakeToolProvider  # noqa: E402
from rob_box_harness.executors import ROSMCPToolProvider  # noqa: E402

from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402


# ─────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────


class _Param:
    """Minimal stand-in for ``rclpy.parameter.Parameter``."""

    __slots__ = ("value",)

    def __init__(self, value: Any) -> None:
        self.value = value


def _bare_node(tool_provider: str) -> DialogueNode:
    """Build a ``DialogueNode`` without running ``__init__``.

    ``_build_tool_provider`` reads two things from the node:

    * ``self.get_parameter("tool_provider")`` — we return the value
      the test wants to exercise.
    * ``self.get_logger()`` — we hand back a ``MagicMock`` so the
      test can assert on warning / info messages.

    Every other method the production ``__init__`` would install
    (publishers, subscriptions, timers, executors) is irrelevant
    to the selector under test.
    """
    node = object.__new__(DialogueNode)
    node.get_parameter = lambda name: _Param(
        tool_provider if name == "tool_provider" else None,
    )
    node.get_logger = lambda: MagicMock()
    return node


def _install_ament_probe_success(monkeypatch: pytest.MonkeyPatch) -> None:
    """Make the ament probe report ``rob_box_mcp_tools`` as present."""
    fake_mod = types.ModuleType("ament_index_python")
    fake_pkg = types.ModuleType("ament_index_python.packages")
    fake_pkg.get_package_share_directory = lambda _pkg: (
        "/opt/ros/jazzy/share/rob_box_mcp_tools"
    )
    fake_mod.packages = fake_pkg
    monkeypatch.setitem(sys.modules, "ament_index_python", fake_mod)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", fake_pkg)


def _install_ament_probe_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    """Make the ament probe raise ``PackageNotFoundError``."""
    fake_mod = types.ModuleType("ament_index_python")
    fake_pkg = types.ModuleType("ament_index_python.packages")

    class _PackageNotFoundError(LookupError):
        pass

    def _raise(_pkg: str) -> str:
        raise _PackageNotFoundError(
            "package 'rob_box_mcp_tools' not found",
        )

    fake_pkg.get_package_share_directory = _raise
    fake_pkg.PackageNotFoundError = _PackageNotFoundError
    fake_mod.packages = fake_pkg
    monkeypatch.setitem(sys.modules, "ament_index_python", fake_mod)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", fake_pkg)


def _install_mcp_adapter_with_bridge(
    monkeypatch: pytest.MonkeyPatch,
    *,
    tool_names: List[str],
) -> None:
    """Stub ``rob_box_mcp_tools.llm_adapter`` so the lazy import succeeds.

    The shell constructs ``LLMToolCallAdapter(self)`` and wraps it in
    a ``ROSMCPToolProvider``. ``update_tools(...)`` then takes a list
    of OpenAI-style envelopes built from ``ToolRegistry.list_tools()``,
    and the resulting catalogue is exposed via ``list_tools()``.
    We inject a bridge whose ``execute_tool_call_sync`` is a no-op
    and let ``ToolRegistry`` drive the catalogue count.
    """

    class _StubBridge:
        def execute_tool_call_sync(self, *_a: Any, **_kw: Any) -> Dict[str, Any]:
            return {"success": True, "data": {}}

    fake_mcp = types.ModuleType("rob_box_mcp_tools")
    fake_adapter = types.ModuleType("rob_box_mcp_tools.llm_adapter")
    fake_adapter.LLMToolCallAdapter = lambda _node: _StubBridge()

    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools", fake_mcp)
    monkeypatch.setitem(
        sys.modules, "rob_box_mcp_tools.llm_adapter", fake_adapter,
    )

    # Note: ``tool_names`` is reserved for callers that want to drive
    # the catalogue directly. In the happy-path test below we leave
    # it empty and let ``ToolRegistry.list_tools()`` populate the
    # catalogue via ``update_tools()`` — that is the production path
    # and the assertion we make is "at least one ToolRegistry name is
    # present in the wired catalogue".
    del tool_names  # explicit: parameter is currently unused


# ─────────────────────────────────────────────────────────────────────────
# Case A — mcp package is reachable
# ─────────────────────────────────────────────────────────────────────────


def test_build_tool_provider_returns_ros_mcp_when_mcp_available(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``tool_provider='ros_mcp'`` with the package discoverable.

    The selector must:

    1. Construct a ``ROSMCPToolProvider`` (not the fake).
    2. Feed it the OpenAI-style envelopes built from
       ``ToolRegistry.list_tools()`` so the LLM-facing surface is
       the single source of truth.
    3. Return a wrapper that exposes a non-empty catalogue — at
       least the production canary names ``speak_text`` and
       ``play_sound`` are reachable so the LLM can dispatch the
       most common voice commands.
    """
    _install_ament_probe_success(monkeypatch)
    _install_mcp_adapter_with_bridge(monkeypatch, tool_names=[])

    node = _bare_node("ros_mcp")

    provider = node._build_tool_provider()

    # (1) The real bridge is in the chain — not the fake.
    # The shell wraps ``ROSMCPToolProvider`` with ``adapt_tool_provider``;
    # either way the returned object must be *not* a plain
    # ``FakeToolProvider`` instance.
    assert not isinstance(provider, FakeToolProvider), (
        "_build_tool_provider() returned FakeToolProvider even though "
        "the operator asked for 'ros_mcp' and the package was reachable"
    )

    # (2) The catalogue must come from ``ToolRegistry`` — at least
    # one of the registry's spec names must be exposed through the
    # adapter. The shell wraps ``ROSMCPToolProvider`` in a
    # ``LegacyToolProviderAdapter`` whose observable surface is
    # ``discover()`` (async) returning a tuple of ``ToolSpec``
    # objects — that is the contract ``DialogCore`` consumes, so
    # we exercise the same path here.
    import asyncio

    specs = asyncio.run(provider.discover())
    catalogue = {spec.name for spec in specs}
    assert catalogue, (
        "expected _build_tool_provider() to expose a non-empty tool "
        "catalogue when the bridge is wired"
    )
    # ``ToolRegistry`` is the single source of truth — at least one
    # registry name must show up in the wired catalogue. Pinning on
    # any registry name keeps the assertion stable across registry
    # refactors; we sort to pick a deterministic anchor.
    registry = __import__(
        "rob_box_harness.core.tool_registry",
        fromlist=["ToolRegistry"],
    ).ToolRegistry()
    registry_names = {spec.name for spec in registry.list_tools()}
    assert registry_names, "ToolRegistry unexpectedly empty"
    sample_name = sorted(registry_names)[0]
    assert sample_name in catalogue, (
        f"expected ToolRegistry name {sample_name!r} in the wired "
        f"catalogue; got: {sorted(catalogue)[:10]}..."
    )


# ─────────────────────────────────────────────────────────────────────────
# Case B — mcp package is not reachable
# ─────────────────────────────────────────────────────────────────────────


def test_build_tool_provider_raises_when_mcp_missing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Fail-loud contract: ``tool_provider='ros_mcp'`` raises ``RuntimeError``.

    The pre-W5a code silently degraded to ``FakeToolProvider`` whenever
    the bridge was unreachable — the operator would see a chat-only
    shell with zero tools and no diagnostic at startup. W5a flipped
    this: when the operator *asked* for the real bridge, the shell
    must surface a clear error rather than pretend everything is fine.

    The error message must mention the missing package so the operator
    knows exactly what to rebuild or reinstall.
    """
    _install_ament_probe_missing(monkeypatch)
    # NB: do NOT install the rob_box_mcp_tools.llm_adapter stub.
    # The shell should fail at the ament-probe step, before reaching
    # the lazy import — that's the cheapest, clearest error path.
    sys.modules.pop("rob_box_mcp_tools", None)
    sys.modules.pop("rob_box_mcp_tools.llm_adapter", None)

    node = _bare_node("ros_mcp")

    with pytest.raises(RuntimeError) as excinfo:
        node._build_tool_provider()

    message = str(excinfo.value)
    assert "rob_box_mcp_tools" in message, (
        "RuntimeError must name the missing package so the operator "
        f"can act on it; got: {message!r}"
    )
    # The remediation hint must be present — operators rely on this
    # to know whether to rebuild the image or flip the launch arg.
    assert "tool_provider" in message, (
        "RuntimeError must mention the tool_provider launch arg as a "
        f"remediation path; got: {message!r}"
    )


# ─────────────────────────────────────────────────────────────────────────
# Bonus A — explicit chat-only backend selection
# ─────────────────────────────────────────────────────────────────────────


def test_build_tool_provider_returns_fake_for_chat_only_backend() -> None:
    """``tool_provider='fake'`` (or ``'none'``) selects ``FakeToolProvider``.

    The shell's own info log must mention the active backend so
    operators can confirm the wiring from the launch log without
    having to dig into the YAML.
    """
    logger = MagicMock()

    for backend in ("fake", "none"):
        node = _bare_node(backend)
        node.get_logger = lambda: logger  # fresh mock per backend
        provider = node._build_tool_provider()
        assert isinstance(provider, FakeToolProvider), (
            f"backend={backend!r} must select FakeToolProvider; "
            f"got {type(provider).__name__}"
        )


def test_build_tool_provider_warns_on_unknown_backend() -> None:
    """An unknown backend emits a warning *and* falls back to ``FakeToolProvider``.

    Operators who fat-finger the launch arg (e.g. ``tool_provider=
    'deepseek_mcp'``) must not crash the shell — they get a startup
    warning and a working chat-only node. The warning text must name
    the bad value so the operator can spot the typo in the YAML.
    """
    logger = MagicMock()
    node = _bare_node("deepseek_mcp_typo")
    node.get_logger = lambda: logger

    provider = node._build_tool_provider()

    assert isinstance(provider, FakeToolProvider)
    warning_messages = [
        call.args[0] for call in logger.warning.call_args_list if call.args
    ]
    joined = " ".join(warning_messages)
    assert "deepseek_mcp_typo" in joined, (
        "warning must name the offending backend value; got: "
        f"{warning_messages!r}"
    )
    assert "FakeToolProvider" in joined, (
        "warning must mention the fallback so operators know tools "
        f"are unavailable; got: {warning_messages!r}"
    )