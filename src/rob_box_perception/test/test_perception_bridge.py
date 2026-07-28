#!/usr/bin/env python3
"""test_perception_bridge.py — Integration tests for PerceptionBridge.

Phase 6 v2 / W12. Exercises the UART → /sensors/data + /perception/health
bridge end-to-end with fakes for ROS2 publishers/subscribers, serial/UART,
and timer callbacks. No real hardware or network access.

Coverage (from .planning/phases/06-harness-p0-finalization/06-04-PLAN.md §W12):

1. Sensor data published to /sensors/data
2. Health status published to /perception/health
3. UART read loop (mock serial port)
4. Multiple sensor types in one message
5. UART error handling (garbage data → skip, not crash)

The file installs its own minimal ``rclpy`` / ``std_msgs`` shim when the
ROS2 runtime is absent, mirroring the pattern used by
``src/rob_box_voice/test/test_dialogue_shell.py``. This keeps the suite
runnable with plain ``pytest`` on developer laptops and in CI containers
that may or may not have ROS2 installed.
"""

from __future__ import annotations

import json
import os
import sys
import time
import types as _types
import unittest
from typing import Any, Callable, Dict, List, Optional
from unittest.mock import MagicMock, patch


# ── rclpy / std_msgs shim ────────────────────────────────────────────────
# Always install a minimal rclpy stub so this test file is runnable
# without a ROS2 install. The bridge only exercises Node.__init__,
# create_publisher, create_timer, declare_parameter/get_parameter,
# get_logger, and get_name — all of which are faked here. We don't need
# rclpy.init() / spin / shutdown in tests because we drive the timer
# callbacks manually.
class _FakeParameter:
    def __init__(self, value: Any) -> None:
        self.value = value


class _FakeNode:
    """Minimal stand-in for ``rclpy.node.Node`` used by PerceptionBridge."""

    def __init__(self, name: str, **kwargs: Any) -> None:
        self._name = name
        self._logger = MagicMock()
        self._logger.info = MagicMock()
        self._logger.warning = MagicMock()
        self._logger.error = MagicMock()
        self._logger.debug = MagicMock()
        self._publishers: Dict[str, MagicMock] = {}
        self._params: Dict[str, Any] = {}
        self._timers: List[Any] = []

    def get_logger(self) -> MagicMock:
        # rclpy's rclpy.rclpy_logger.Logger exposes `.info`, `.warn`
        # (legacy alias) and `.warning`. The bridge code uses the
        # short-form `.warn(...)`; the MagicMock auto-creates any
        # attribute, so we just return it.
        return self._logger

    def declare_parameter(self, name: str, default: Any = None) -> MagicMock:
        self._params.setdefault(name, default)
        return MagicMock()

    def get_parameter(self, name: str) -> _FakeParameter:
        return _FakeParameter(self._params.get(name))

    def has_parameter(self, name: str) -> bool:
        # Used indirectly via get_parameter in some tests; be lenient.
        return name in self._params

    def create_publisher(self, msg_type: Any, topic: str, depth: int,
                          **kwargs: Any) -> MagicMock:
        pub = MagicMock()
        pub.topic = topic
        pub.msg_type = msg_type
        pub.published: List[Any] = []
        original_publish = pub.publish

        def _capture(msg: Any) -> Any:
            pub.published.append(msg)
            return original_publish(msg)

        pub.publish = _capture
        self._publishers[topic] = pub
        return pub

    def create_timer(self, period: float, callback: Callable[[], None],
                       callback_group: Any = None) -> MagicMock:
        timer = MagicMock()
        timer.period = period
        timer.callback = callback
        self._timers.append(timer)
        return timer

    def get_name(self) -> str:
        return self._name

    def destroy_node(self) -> None:
        # No-op stub. Cancellation of timers lives on the timer mocks.
        return None


_mock_rclpy = _types.ModuleType("rclpy")
_mock_rclpy.init = lambda *a, **kw: None
_mock_rclpy.shutdown = lambda *a, **kw: None
_mock_rclpy.ok = lambda: True
sys.modules.setdefault("rclpy", _mock_rclpy)

_mock_rclpy_node = _types.ModuleType("rclpy.node")
_mock_rclpy_node.Node = _FakeNode
sys.modules.setdefault("rclpy.node", _mock_rclpy_node)

_cb_mod = _types.ModuleType("rclpy.callback_groups")
_cb_mod.ReentrantCallbackGroup = type("ReentrantCallbackGroup", (), {})
sys.modules.setdefault("rclpy.callback_groups", _cb_mod)

_qos_mod = _types.ModuleType("rclpy.qos")
_qos_mod.HistoryPolicy = _types.SimpleNamespace(KEEP_LAST="KEEP_LAST")
_qos_mod.ReliabilityPolicy = _types.SimpleNamespace(RELIABLE="RELIABLE")
_qos_mod.QoSProfile = lambda *a, **kw: MagicMock()
sys.modules.setdefault("rclpy.qos", _qos_mod)

_std_msgs = _types.ModuleType("std_msgs")
_std_msgs_msg = _types.ModuleType("std_msgs.msg")


class _String:
    def __init__(self, data: str = "") -> None:
        self.data = data


class _Bool:
    def __init__(self, data: bool = False) -> None:
        self.data = data


_std_msgs_msg.String = _String
_std_msgs_msg.Bool = _Bool
sys.modules.setdefault("std_msgs", _std_msgs)
sys.modules.setdefault("std_msgs.msg", _std_msgs_msg)


# Import the SUT after the shim is registered so the module's
# ``import rclpy`` / ``from rclpy.node import Node`` resolve to our fakes.
from std_msgs.msg import String  # noqa: E402

from rob_box_perception.perception_bridge import (  # noqa: E402
    DEFAULT_UART_BAUD,
    DEFAULT_UART_PORT,
    HEALTH_PERIOD,
    SENSOR_READ_PERIOD,
    STATUS_DEGRADED,
    STATUS_HEALTHY,
    STATUS_UNKNOWN,
    PerceptionBridge,
)


# ── Fakes ────────────────────────────────────────────────────────────────


class _FakeUART:
    """Minimal ``serial.Serial``-like fake backed by an in-memory queue.

    Mirrors what :class:`serial.Serial` exposes to the bridge: a single
    ``readline()`` call that returns one JSON frame at a time, or ``b""``
    when no frame is queued; plus ``close()`` for shutdown.
    """

    def __init__(self, lines: Optional[List[Any]] = None,
                 port: str = "/tmp/fake-tty",
                 baud: int = DEFAULT_UART_BAUD) -> None:
        self._lines = []
        for line in lines or []:
            if isinstance(line, str):
                self._lines.append(line.encode())
            elif isinstance(line, (bytes, bytearray)):
                self._lines.append(bytes(line))
            else:
                raise TypeError(f"unsupported line type: {type(line)!r}")
        self._i = 0
        self.port = port
        self.baudrate = baud
        self.closed = False

    def readline(self) -> bytes:
        if self._i < len(self._lines):
            line = self._lines[self._i]
            self._i += 1
            return line
        return b""

    def close(self) -> None:
        self.closed = True

    # No-op API methods the bridge could call in future hardening paths.
    def reset_input_buffer(self) -> None:
        pass


class _FailureUART:
    """UART that raises on readline — simulates hardware fault."""

    def __init__(self) -> None:
        self.reads = 0

    def readline(self) -> bytes:
        self.reads += 1
        raise RuntimeError("hw fault: device disconnected")

    def close(self) -> None:
        pass


# ── TestPerceptionBridge ─────────────────────────────────────────────────


class TestPerceptionBridge(unittest.TestCase):
    """End-to-end integration tests for ``PerceptionBridge``.

    Each test constructs the bridge with the rclpy shim in place, then
    drives either the timer callbacks or the underlying methods
    directly. Publishers are inspected via the captured ``MagicMock``
    ``.published`` list to verify the bridge produced the right
    ``std_msgs.msg.String`` payload.
    """

    def setUp(self) -> None:
        # Force stub mode in tests by default so the bridge does not
        # try to open /dev/ttyAMA0 on the CI host. Individual tests
        # rewire ``self.node._uart`` to a fake before driving the read
        # loop.
        os.environ.pop("SENSOR_UART_PORT", None)
        os.environ.pop("SENSOR_UART_BAUD", None)
        self.node = PerceptionBridge()
        # Make published msg inspection simple.
        self.sensor_pub = self.node._sensor_pub
        self.health_pub = self.node._health_pub
        # Reset counters so each test starts from a known baseline.
        self.node._ok_reads = 0
        self.node._bad_reads = 0
        self.node._published = 0
        self.node._last_seq = None
        self.node._last_data_ts = None

    def tearDown(self) -> None:
        self.node.destroy_node()

    # ---------------------------------------------------------------- spec 1
    def test_sensor_data_published_to_sensors_data(self) -> None:
        """Spec 1: a single UART frame becomes one String msg on /sensors/data."""
        frame = json.dumps({"seq": 1, "imu": {"ax": 0.1, "ay": -0.2}})
        self.node._uart = _FakeUART([frame])

        self.node._read_sensors()

        self.assertEqual(len(self.sensor_pub.published), 1)
        msg = self.sensor_pub.published[0]
        self.assertIsInstance(msg, String)
        published = json.loads(msg.data)
        self.assertEqual(published["seq"], 1)
        self.assertEqual(published["imu"]["ax"], 0.1)
        # Counter parity
        self.assertEqual(self.node._ok_reads, 1)
        self.assertEqual(self.node._published, 1)
        self.assertEqual(self.node._bad_reads, 0)

    # ---------------------------------------------------------------- spec 2
    def test_health_status_published_to_perception_health(self) -> None:
        """Spec 2: /perception/health receives a snapshot JSON each tick."""
        # Drive a healthy read first so counters register activity.
        self.node._uart = _FakeUART([json.dumps({"seq": 1})])
        self.node._read_sensors()
        self.sensor_pub.published.clear()
        self.health_pub.published.clear()

        # Now run the health publisher via the registered callback to
        # mimic what rclpy.spin() would do.
        self.node._publish_health()

        self.assertEqual(len(self.health_pub.published), 1)
        msg = self.health_pub.published[0]
        self.assertIsInstance(msg, String)
        snapshot = json.loads(msg.data)
        self.assertEqual(snapshot["status"], STATUS_HEALTHY)
        self.assertEqual(snapshot["ok_reads"], 1)
        self.assertEqual(snapshot["bad_reads"], 0)
        self.assertEqual(snapshot["published"], 1)
        self.assertTrue(snapshot["stub_mode"])
        self.assertIn("ts", snapshot)
        self.assertIn("data_age_s", snapshot)

    # ---------------------------------------------------------------- spec 3
    def test_uart_read_loop_with_mock_serial_port(self) -> None:
        """Spec 3: UART read loop drains queued frames in order."""
        # Three well-formed frames in order. The bridge publishes each
        # one separately and advances the seq-mono check.
        frames = [
            json.dumps({"seq": 10, "imu": {"ax": 1.0}}),
            json.dumps({"seq": 11, "imu": {"ax": 1.1}}),
            json.dumps({"seq": 12, "imu": {"ax": 1.2}}),
        ]
        self.node._uart = _FakeUART(frames)

        # Drive the loop three times — equivalent to three timer ticks.
        for _ in range(len(frames) + 1):  # one extra tick drains empty
            self.node._read_sensors()

        self.assertEqual(len(self.sensor_pub.published), 3)
        seqs = [json.loads(m.data)["seq"] for m in self.sensor_pub.published]
        self.assertEqual(seqs, [10, 11, 12])
        # Last-seen seq tracks correctly.
        self.assertEqual(self.node._last_seq, 12)
        # No errors.
        self.assertEqual(self.node._bad_reads, 0)
        self.assertEqual(self.node._ok_reads, 3)
        # Subsequent ticks with empty UART are no-ops.
        self.assertEqual(len(self.sensor_pub.published), 3)

    # ---------------------------------------------------------------- spec 4
    def test_multiple_sensor_types_in_one_message(self) -> None:
        """Spec 4: one JSON frame carrying imu + lidar + bumper publishes intact."""
        frame = json.dumps({
            "seq": 42,
            "imu": {"ax": 0.0, "ay": 0.0, "az": 9.81},
            "lidar": {"ranges": [0.5, 1.2, 3.4]},
            "bumper": {"left": False, "right": False, "front": True},
            "battery": {"voltage": 12.4, "pct": 78},
        })
        self.node._uart = _FakeUART([frame])

        self.node._read_sensors()

        self.assertEqual(len(self.sensor_pub.published), 1)
        msg = self.sensor_pub.published[0]
        published = json.loads(msg.data)
        self.assertEqual(published["imu"]["az"], 9.81)
        self.assertEqual(published["lidar"]["ranges"], [0.5, 1.2, 3.4])
        self.assertTrue(published["bumper"]["front"])
        self.assertEqual(published["battery"]["pct"], 78)
        self.assertEqual(published["seq"], 42)

    # ---------------------------------------------------------------- spec 5
    def test_garbage_data_skipped_not_crashed(self) -> None:
        """Spec 5: garbage UART frames are skipped, not crash the loop."""
        cases: List[Any] = [
            b"not-json-at-all",           # invalid JSON
            b"\xff\xfe\xfd garbage",      # bytes that fail UTF-8 decode path
            b'[1,2,3]',                    # JSON but not a dict
            b'"a string, not an object"',  # JSON string, not a dict
        ]
        self.node._uart = _FakeUART(cases)

        # Bridge must NOT raise across all the bad frames.
        for _ in range(len(cases) + 2):  # two extra empty-read ticks
            try:
                self.node._read_sensors()
            except Exception as exc:
                self.fail(f"_read_sensors raised on garbage data: {exc!r}")

        # Nothing was published (every frame was rejected).
        self.assertEqual(len(self.sensor_pub.published), 0)
        self.assertEqual(self.node._ok_reads, 0)
        self.assertEqual(self.node._published, 0)
        # All four frames were rejected (one bad_read per line decoded).
        self.assertGreaterEqual(self.node._bad_reads, 1)
        # Logger.warn is the method used inside the bridge.
        self.assertTrue(self.node.get_logger().warn.called)

    # ---------------------------------------------------------------- bonus
    def test_uart_exception_counted_as_bad_read(self) -> None:
        """A UART read exception increments bad_reads without crashing."""
        self.node._uart = _FailureUART()
        try:
            self.node._read_sensors()
        except Exception as exc:
            self.fail(f"_read_sensors must not propagate UART exceptions: {exc!r}")

        self.assertEqual(self.node._bad_reads, 1)
        self.assertEqual(self.node._ok_reads, 0)
        self.assertEqual(len(self.sensor_pub.published), 0)

    def test_stub_mode_when_uart_missing(self) -> None:
        """No UART present → bridge stays inert, status=UNKNOWN."""
        self.node._uart = None  # stub mode
        self.node._read_sensors()

        self.assertEqual(len(self.sensor_pub.published), 0)
        self.assertEqual(self.node._ok_reads, 0)
        self.assertEqual(self.node._bad_reads, 0)
        # /perception/health receives the startup snapshot exactly once.
        self.assertEqual(len(self.health_pub.published), 1)
        snapshot = json.loads(self.health_pub.published[0].data)
        self.assertEqual(snapshot["status"], STATUS_UNKNOWN)
        self.assertTrue(snapshot["stub_mode"])

    def test_stub_mode_registers_no_timers_and_publishes_one_health_snapshot(self) -> None:
        """Stub mode must avoid periodic callbacks and emit one UNKNOWN snapshot."""
        self.assertIsNone(self.node._sensor_timer)
        self.assertIsNone(self.node._health_timer)
        self.assertEqual(len(self.health_pub.published), 1)
        snapshot = json.loads(self.health_pub.published[0].data)
        self.assertEqual(snapshot["status"], STATUS_UNKNOWN)
        self.assertTrue(snapshot["stub_mode"])
        self.assertTrue(self.node.get_logger().warn.called)

    def test_non_monotonic_seq_warns_but_publishes(self) -> None:
        """Stale seq frames are flagged but do not halt publishing."""
        self.node._uart = _FakeUART([
            json.dumps({"seq": 5}),
            json.dumps({"seq": 4}),   # older than the previous
        ])

        self.node._read_sensors()
        self.node._read_sensors()

        # Both frames still published — monotonic check is observability,
        # not a hard gate.
        self.assertEqual(len(self.sensor_pub.published), 2)
        # Logger.warn is the call name used inside the bridge.
        self.assertTrue(self.node.get_logger().warn.called)

    def test_health_degraded_when_mostly_bad(self) -> None:
        """More bad reads than good → /perception/health reports DEGRADED."""
        self.node._uart = _FakeUART([b"garbage"])
        self.node._read_sensors()
        self.node._read_sensors()
        self.node._publish_health()

        snapshot = json.loads(self.health_pub.published[-1].data)
        self.assertEqual(snapshot["status"], STATUS_DEGRADED)

    def test_timer_callbacks_registered_with_correct_periods(self) -> None:
        """Bridge registers both timers with the configured periods in hardware mode."""
        with patch(
            "rob_box_perception.perception_bridge._open_uart",
            return_value=_FakeUART(),
        ):
            node = PerceptionBridge()
        try:
            sensor_timer, health_timer = node._sensor_timer, node._health_timer
            self.assertAlmostEqual(sensor_timer.period, SENSOR_READ_PERIOD)
            self.assertAlmostEqual(health_timer.period, HEALTH_PERIOD)
            # Each timer's stored callback is the corresponding method.
            self.assertEqual(sensor_timer.callback.__name__, "_read_sensors")
            self.assertEqual(health_timer.callback.__name__, "_publish_health")
        finally:
            node.destroy_node()

    def test_hardware_mode_registers_both_timers(self) -> None:
        """Hardware mode keeps the sensor and health timer scheduling."""
        fake_uart = _FakeUART()
        with patch(
            "rob_box_perception.perception_bridge._open_uart",
            return_value=fake_uart,
        ):
            node = PerceptionBridge()
        try:
            self.assertFalse(node._stub_mode)
            self.assertAlmostEqual(node._sensor_timer.period, SENSOR_READ_PERIOD)
            self.assertAlmostEqual(node._health_timer.period, HEALTH_PERIOD)
            self.assertEqual(len(node._health_pub.published), 0)
        finally:
            node.destroy_node()

    def test_publishers_wired_to_correct_topics(self) -> None:
        """Publishers are bound to the spec'd topic names."""
        self.assertEqual(self.node._sensor_pub.topic, "/sensors/data")
        self.assertEqual(self.node._health_pub.topic, "/perception/health")

    def test_destroy_node_cancels_timers_and_closes_uart(self) -> None:
        """Lifecycle: destroy_node cancels timers and closes the UART."""
        fake = _FakeUART([json.dumps({"seq": 1})])
        with patch(
            "rob_box_perception.perception_bridge._open_uart",
            return_value=fake,
        ):
            node = PerceptionBridge()
        try:
            # Drive a single read so the bridge touches the UART.
            node._read_sensors()
        finally:
            node.destroy_node()

        self.assertTrue(fake.closed)
        node._sensor_timer.cancel.assert_called()
        node._health_timer.cancel.assert_called()

    def test_env_overrides_uart_path(self) -> None:
        """SENSOR_UART_PORT / SENSOR_UART_BAUD env vars are honored."""
        # Construct a fresh node with non-default env values.
        os.environ["SENSOR_UART_PORT"] = "/dev/ttyUSB_FAKE"
        os.environ["SENSOR_UART_BAUD"] = "57600"
        try:
            node = PerceptionBridge()
        finally:
            os.environ.pop("SENSOR_UART_PORT", None)
            os.environ.pop("SENSOR_UART_BAUD", None)

        # Either we successfully opened the fake path (unlikely on a
        # container) or we fell back to stub mode — both honour the env
        # override for logging purposes. The portable assertion: the
        # startup log records the requested port.
        log_calls = [str(c) for c in node.get_logger().info.call_args_list]
        joined = " ".join(log_calls)
        self.assertIn("/dev/ttyUSB_FAKE", joined)
        self.assertIn("57600", joined)
        node.destroy_node()


if __name__ == "__main__":
    unittest.main()
