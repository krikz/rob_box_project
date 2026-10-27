"""
conftest.py — Mock ROS2/rclpy dependencies for rob_box_teleop unit tests.

Loaded before any module that imports rclpy (mirrors the rob_box_voice
pattern). FakeNode keeps a parameter registry and records published
messages so tests can assert on topics without a ROS2 runtime.

Speech goes through rob_box_core.speech_client, which builds a real
``SpeechRequest``: the real rosidl package when it is built (CI), otherwise a
stub assembled from the repository's ``.msg`` files (Windows, no ROS).
"""

import sys
import types
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

DIALOG_MSGS = install_dialog_msgs_stub()
#: ``builtin_interfaces/Time`` of whichever message build is at hand (the real
#: one validates the type of ``SpeechRequest.stamp`` on assignment).
_TIME = type(DIALOG_MSGS.SpeechRequest().stamp)


# Per-test parameter overrides; a test sets them before instantiating the node.
PARAM_OVERRIDES = {}


@pytest.fixture
def node_params():
    """Reset PARAM_OVERRIDES and force non-SBUS node construction."""
    PARAM_OVERRIDES.clear()
    PARAM_OVERRIDES["use_sbus"] = False
    yield PARAM_OVERRIDES
    PARAM_OVERRIDES.clear()


class FakePublisher:
    """Records every published message for assertions."""

    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeClock:
    """rclpy Clock stub: ``now().to_msg()`` is a real ``builtin_interfaces/Time``."""

    def now(self):
        return self

    def to_msg(self):
        return _TIME(sec=7, nanosec=5)


class FakeNode:
    """Minimal rclpy.node.Node stub with a parameter registry."""

    def __init__(self, name, **kwargs):
        self._name = name
        self._params = {}
        self._logger = MagicMock()
        self.publishers = []

    def get_logger(self):
        return self._logger

    def declare_parameter(self, name, default=None):
        self._params[name] = PARAM_OVERRIDES.get(name, default)
        return MagicMock()

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params.get(name))

    def create_publisher(self, msg_type, topic, depth):
        pub = FakePublisher(topic)
        self.publishers.append((topic, pub))
        return pub

    def create_subscription(self, *args, **kwargs):
        return MagicMock()

    def create_timer(self, period, callback):
        return MagicMock()

    def create_service(self, *args, **kwargs):
        return MagicMock()

    def create_client(self, *args, **kwargs):
        return MagicMock()

    def get_name(self):
        return self._name

    def get_clock(self):
        return FakeClock()


class FakeTwist:
    """geometry_msgs/Twist with real numeric attributes."""

    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class FakeBool:
    """std_msgs/Bool with a real data attribute."""

    def __init__(self):
        self.data = False


def _install_ros_mocks():
    mock_rclpy = MagicMock()
    mock_rclpy_node = MagicMock()
    mock_rclpy_node.Node = FakeNode

    mock_sensor_msgs = MagicMock()
    mock_sensor_msgs_msg = MagicMock()
    mock_sensor_msgs_msg.Joy = MagicMock

    mock_geometry_msgs = MagicMock()
    mock_geometry_msgs_msg = MagicMock()
    mock_geometry_msgs_msg.Twist = FakeTwist

    mock_std_msgs = MagicMock()
    mock_std_msgs_msg = MagicMock()
    mock_std_msgs_msg.Bool = FakeBool

    mocks = {
        "rclpy": mock_rclpy,
        "rclpy.node": mock_rclpy_node,
        "sensor_msgs": mock_sensor_msgs,
        "sensor_msgs.msg": mock_sensor_msgs_msg,
        "geometry_msgs": mock_geometry_msgs,
        "geometry_msgs.msg": mock_geometry_msgs_msg,
        "std_msgs": mock_std_msgs,
        "std_msgs.msg": mock_std_msgs_msg,
    }
    for name, mock in mocks.items():
        sys.modules.setdefault(name, mock)


_install_ros_mocks()
