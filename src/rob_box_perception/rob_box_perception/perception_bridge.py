#!/usr/bin/env python3
"""perception_bridge.py - UART sensor bridge for Rob Box perception.

Phase 6 v2 / W11. Reads sensor MCU frames over UART and republishes them
as JSON on /sensors/data, with periodic health snapshots on
/perception/health. Out of scope: MCU firmware, micro-ROS removal.
"""

import json
import os
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_UART_PORT = "/dev/ttyAMA0"
DEFAULT_UART_BAUD = 115200
SENSOR_READ_PERIOD = 0.1   # 10 Hz
HEALTH_PERIOD = 1.0        # 1 Hz
STATUS_HEALTHY = "HEALTHY"
STATUS_DEGRADED = "DEGRADED"
STATUS_UNKNOWN = "UNKNOWN"


def _open_uart(port: str, baud: int) -> Optional[Any]:
    """Open the UART port. Returns None when hardware is absent (dev hosts)."""
    try:
        import serial  # type: ignore
    except ImportError:
        return None
    if not os.path.exists(port):
        return None
    try:
        return serial.Serial(port, baud, timeout=0.05)
    except Exception:
        return None


class PerceptionBridge(Node):
    """Reads UART sensor stream, republishes as JSON on /sensors/data."""

    def __init__(self) -> None:
        super().__init__("perception_bridge")

        # ---- Parameters (overridable from launch) -------------------------
        self.declare_parameter("sensor_read_period", SENSOR_READ_PERIOD)
        self.declare_parameter("health_period", HEALTH_PERIOD)
        self._sensor_period = float(self.get_parameter("sensor_read_period").value)
        self._health_period = float(self.get_parameter("health_period").value)

        # ---- UART ---------------------------------------------------------
        port = os.getenv("SENSOR_UART_PORT", DEFAULT_UART_PORT)
        baud = int(os.getenv("SENSOR_UART_BAUD", str(DEFAULT_UART_BAUD)))
        self._uart = _open_uart(port, baud)
        self._stub_mode = self._uart is None
        if self._stub_mode:
            self.get_logger().warn(
                f"Sensor UART {port} not available; reads will no-op until "
                f"hardware is attached."
            )

        # ---- Publishers ---------------------------------------------------
        self._sensor_pub = self.create_publisher(String, "/sensors/data", 10)
        self._health_pub = self.create_publisher(String, "/perception/health", 10)

        # ---- Counters -----------------------------------------------------
        self._ok_reads = 0
        self._bad_reads = 0
        self._published = 0
        self._last_seq: Optional[int] = None
        self._last_data_ts: Optional[float] = None
        self._status = STATUS_UNKNOWN

        # ---- Timers -------------------------------------------------------
        self._sensor_timer = self.create_timer(
            self._sensor_period, self._read_sensors
        )
        self._health_timer = self.create_timer(
            self._health_period, self._publish_health
        )

        self.get_logger().info(
            f"perception_bridge up (port={port} baud={baud} "
            f"stub={self._stub_mode} period={self._sensor_period}s)"
        )

    # -----------------------------------------------------------------------
    # Sensor loop
    # -----------------------------------------------------------------------

    def _read_sensors(self) -> None:
        if self._uart is None:
            return

        try:
            raw = self._uart.readline()
        except Exception as exc:
            self._bad_reads += 1
            self.get_logger().warn(f"UART read failed: {exc}")
            return

        if not raw:
            return

        try:
            line = raw.decode("utf-8", errors="replace").strip()
            data = json.loads(line)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self._bad_reads += 1
            self.get_logger().warn(f"Bad sensor frame: {exc}")
            return

        if not isinstance(data, dict):
            self._bad_reads += 1
            self.get_logger().warn("Sensor frame is not a JSON object")
            return

        seq = data.get("seq")
        if isinstance(seq, int):
            if self._last_seq is not None and seq <= self._last_seq:
                self.get_logger().warn(
                    f"Non-monotonic sensor seq: {seq} after {self._last_seq}"
                )
            self._last_seq = seq

        msg = String()
        msg.data = json.dumps(data, separators=(",", ":"))
        self._sensor_pub.publish(msg)
        self._ok_reads += 1
        self._published += 1
        self._last_data_ts = time.time()

    # -----------------------------------------------------------------------
    # Health loop
    # -----------------------------------------------------------------------

    def _publish_health(self) -> None:
        now = time.time()
        age = None if self._last_data_ts is None else (now - self._last_data_ts)

        if self._ok_reads == 0 and self._bad_reads == 0:
            status = STATUS_UNKNOWN
        elif self._bad_reads > self._ok_reads:
            status = STATUS_DEGRADED
        elif age is not None and age > (self._sensor_period * 5):
            status = STATUS_DEGRADED
        else:
            status = STATUS_HEALTHY

        self._status = status
        snapshot: Dict[str, Any] = {
            "status": status,
            "ok_reads": self._ok_reads,
            "bad_reads": self._bad_reads,
            "published": self._published,
            "stub_mode": self._stub_mode,
            "data_age_s": age,
            "ts": now,
        }
        msg = String()
        msg.data = json.dumps(snapshot, separators=(",", ":"))
        self._health_pub.publish(msg)

    # -----------------------------------------------------------------------
    # Lifecycle
    # -----------------------------------------------------------------------

    def destroy_node(self) -> None:
        try:
            self._sensor_timer.cancel()
            self._health_timer.cancel()
        except Exception:
            pass
        try:
            if self._uart is not None and not self._stub_mode:
                self._uart.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
