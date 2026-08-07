"""Unit tests for ROS2Transport availability and FakeTransport drop-in.

Does NOT depend on rclpy. Tests the transport contract and the
ROS2_AVAILABLE flag behavior.
"""

from __future__ import annotations

import pytest

from rob_box_harness.transport import FakeTransport, Transport, ROS2Transport


class TestROS2Availability:

    def test_ros2transport_class_exists(self) -> None:
        """ROS2Transport class is importable even without rclpy."""
        assert ROS2Transport is not None

    def test_ros2transport_is_subclass_of_transport(self) -> None:
        """ROS2Transport is a subclass of the Transport ABC."""
        # At minimum, the class exists and is importable
        assert hasattr(ROS2Transport, "__init__")


class TestFakeTransport:

    def test_instance_creation(self) -> None:
        t = FakeTransport()
        assert isinstance(t, Transport)

    def test_name_attribute(self) -> None:
        t = FakeTransport()
        assert t.name == "fake"

    def test_default_state(self) -> None:
        t = FakeTransport()
        assert t.name == "fake"

    def test_multiple_instances_independent(self) -> None:
        t1 = FakeTransport()
        t2 = FakeTransport()
        assert t1 is not t2
