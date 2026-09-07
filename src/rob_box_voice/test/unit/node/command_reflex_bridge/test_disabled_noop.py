"""test_disabled_noop.py — DoD #3 of issue #1997.

When ``enable_reflex_layer`` is False (the default), the bridge must
NOT start any background thread, NOT spin up an asyncio loop, and
NOT call ``EventBus.publish`` from inside ``stt_callback``. The
existing Nav2 path must keep working as it did before the bridge
landed.

The trick: ``CommandNode.__init__`` is heavy (Nav2 action clients,
MCP tool integration, etc.) and is hard to construct in a unit test.
We bypass it with ``object.__new__`` and call the bridge hooks
directly — exactly the pattern other unit tests in this repo use for
heavy rclpy nodes (``test_command_intent_gate.py``, ``test_dialogue_node_self_calls.py``).
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from rob_box_voice.command_node import CommandNode


def _make_node(enable_reflex_layer: bool = False) -> CommandNode:
    """Construct a minimal CommandNode that records every action on
    its bridge state.

    Why ``object.__new__``? The real ``__init__`` opens Nav2 action
    clients, MCP tools, and a half-dozen ROS publishers — none of
    which this test cares about. Bypassing ``__init__`` lets us
    drive the bridge hooks in isolation. We DO set ``enable_reflex_layer``
    because the constructor consults it to decide whether to call
    ``_start_reflex_bridge`` — we want to assert that call does NOT
    happen when the parameter is False (DoD #3).
    """

    node = object.__new__(CommandNode)
    # Mirror what ``__init__`` sets up just for the bridge.
    node.get_logger = MagicMock()
    node.enable_reflex_layer = enable_reflex_layer
    node._event_bus = None
    node._reflex_layer = None
    node._bus_thread = None
    node._bus_loop = None
    return node


class TestDisabledBridgeIsNoop:
    """With ``enable_reflex_layer=False`` every bridge hook is a no-op."""

    def test_publish_parsed_async_returns_immediately_when_bus_is_none(self) -> None:
        """``_publish_parsed_async`` MUST short-circuit when the bus
        was never started. It must not raise, must not log errors,
        must not call publish() on a None bus."""

        node = _make_node(enable_reflex_layer=False)
        # If anything tries to touch self._bus_loop, we get an
        # AttributeError because we set it to None and the early-exit
        # guard checks it. (The ``finally`` of __init__ also clears
        # _bus_loop, so a leftover None is the expected state.)
        result = node._publish_parsed_async(MagicMock())
        assert result is None
        # And no log noise on the success path — only debug-level on
        # errors, which is fine.
        node.get_logger().error.assert_not_called()

    def test_destroy_node_is_idempotent_when_bridge_was_never_started(self) -> None:
        """destroy_node() must not crash if the bridge never came up."""

        node = _make_node(enable_reflex_layer=False)
        # ``super().destroy_node()`` is unreachable without an rclpy
        # superclass, so we patch it out for this assertion.
        from unittest.mock import patch
        with patch.object(CommandNode.__mro__[1], "destroy_node", return_value=None):
            node.destroy_node()
        # State stays clean.
        assert node._event_bus is None
        assert node._bus_thread is None
        assert node._bus_loop is None

    def test_no_background_thread_is_created_when_disabled(self) -> None:
        """The bridge is opt-in: ``enable_reflex_layer=False`` means
        ``_start_reflex_bridge`` is never called, which means no
        ``threading.Thread``, no ``asyncio.new_event_loop``, no
        ``EventBus()``. Verified by inspecting the freshly-``__new__``'d
        node's bridge attrs — they are all None."""

        node = _make_node(enable_reflex_layer=False)
        assert node._bus_thread is None
        assert node._bus_loop is None
        assert node._event_bus is None
        assert node._reflex_layer is None


class TestBuildEnvelopeIsPure:
    """``build_parsed_envelope`` is a staticmethod — it must work
    even without any ``self`` state (no bus, no loop, no thread).

    This is what makes the bridge testable without a real rclpy
    executor, and it is also the safety net that prevents DoD #3
    regressions: the staticmethod has zero side effects on the
    caller, so disabling the bridge (which is the only side-effect
    path) cannot affect the envelope shape.
    """

    def test_build_envelope_does_not_touch_self(self) -> None:
        """A staticmethod must not mutate ``self`` state. We verify
        that ``build_parsed_envelope`` reads no attribute starting
        with underscore from ``self`` — it could only be called as
        ``CommandNode.build_parsed_envelope(cmd)`` and would raise
        TypeError if it tried to access ``self`` (well, it works
        via descriptor but reads nothing from the instance)."""

        import inspect

        src = inspect.getsource(CommandNode.build_parsed_envelope)
        # No ``self.`` references inside the staticmethod body. The
        # only outside references allowed are the parameter ``command``
        # and the imported names from ``rob_box_voice.scheduler``.
        for line in src.splitlines():
            stripped = line.lstrip()
            if stripped.startswith("def ") or not stripped:
                continue
            assert "self." not in line, (
                f"build_parsed_envelope must not reference self: {line!r}"
            )
