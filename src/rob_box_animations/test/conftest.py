"""Pytest fixtures + ROS2 stubs for rob_box_animations unit tests.

The AnimationPlayer module imports ``rclpy``, ``rclpy.node``, and
``sensor_msgs.msg`` at module load time. In our CI / local-dev docker
images those packages aren't installed (they require colcon/apt), so we
register MagicMock stand-ins in ``sys.modules`` before any test module
imports the package under test.

A ``FakeNode`` class is provided so AnimationPlayer — which calls
``node.create_publisher(...)``, ``node.get_logger()``, etc. — can be
instantiated without a live ROS2 daemon.
"""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock
from pathlib import Path

import pytest


# ---------------------------------------------------------------------------
# ROS2 mocks (rclpy + sensor_msgs)
# ---------------------------------------------------------------------------

def _install_ros_mocks() -> None:
    """Register MagicMock stand-ins for rclpy and sensor_msgs.

    Idempotent — uses ``setdefault`` so a real rclpy install wins.
    """

    class FakeNode:
        """Minimal rclpy.node.Node stand-in.

        AnimationPlayer only uses:
          * get_logger()
          * create_publisher(msg_type, topic, queue_size)

        so that's all we implement here.
        """

        def __init__(self, name: str = "fake_node", **kwargs):
            self._name = name
            self._logger = MagicMock()
            self._logger.info = MagicMock()
            self._logger.warning = MagicMock()
            self._logger.warn = MagicMock()  # alias used by some callers
            self._logger.error = MagicMock()
            self._logger.debug = MagicMock()

        def get_logger(self):
            return self._logger

        def create_publisher(self, *args, **kwargs):
            return MagicMock()

        def get_clock(self):
            clock = MagicMock()
            clock.now = MagicMock(return_value=MagicMock())
            return clock

    mock_rclpy = MagicMock()
    mock_rclpy_node = MagicMock()
    mock_rclpy_node.Node = FakeNode

    mock_sensor_msgs = MagicMock()
    mock_sensor_msgs_msg = MagicMock()
    mock_sensor_msgs_msg.Image = MagicMock(name="Image")

    # Publish to any namespace a test might want to introspect.
    sys.modules.setdefault("rclpy", mock_rclpy)
    sys.modules.setdefault("rclpy.node", mock_rclpy_node)
    sys.modules.setdefault("sensor_msgs", mock_sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", mock_sensor_msgs_msg)


_install_ros_mocks()


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def fake_node():
    """A bare FakeNode for instantiating AnimationPlayer.

    Returned as a fresh instance per test so mock call history doesn't
    leak between tests.
    """
    return sys.modules["rclpy.node"].Node("test_animation_player")


@pytest.fixture
def sample_manifests_dir(tmp_path: Path) -> Path:
    """Build a tiny ``animations/manifests/`` tree with three valid YAMLs.

    Each manifest references a real (1×1) PNG so the loader's
    ``_validate_manifest`` (which checks that every frame image exists)
    accepts them.
    """
    try:
        from PIL import Image  # type: ignore[import-not-found]
    except ImportError:  # pragma: no cover — Pillow is in the test deps
        pytest.skip("Pillow is required for the AnimationPlayer test fixtures")

    manifests_dir = tmp_path / "manifests"
    manifests_dir.mkdir()

    # Write each manifest + a 1×1 placeholder PNG it references.
    for name in ("idle", "talking", "happy"):
        frame_path = tmp_path / f"{name}_frame.png"
        Image.new("RGB", (1, 1), color=(0, 0, 0)).save(frame_path)

        (manifests_dir / f"{name}.yaml").write_text(
            f"""name: {name}
description: {name} test animation
version: "1.0"
author: tester
duration_ms: 100
loop: true
fps: 10
panels:
  - logical_group: main_display
    offset_ms: 0
    frames:
      - image: {frame_path.name}
        duration_ms: 100
"""
        )
    return tmp_path


@pytest.fixture
def player(fake_node, sample_manifests_dir):
    """A freshly-built AnimationPlayer wired to the sample manifests dir.

    The playback loop is mocked away — callers can opt-in to a real
    thread by patching ``AnimationPlayer._playback_loop`` with a no-op.
    """
    from rob_box_animations.animation_player import AnimationPlayer

    p = AnimationPlayer(fake_node, str(sample_manifests_dir))
    # Replace the long-running playback thread body with a no-op so we
    # can exercise state transitions deterministically without waiting
    # on real frame sleeps.
    p._playback_loop = lambda: None  # type: ignore[assignment]
    return p