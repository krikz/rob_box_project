"""Test setup for rob_box_animations: package on sys.path, ROS stand-ins for face_node.

``face_director`` and ``face`` are pure Python and need only numpy. ``face_node``
imports rclpy / sensor_msgs / std_msgs / rob_box_dialog_msgs at module load;
when they are not installed (Windows, lint images) light stand-ins are
registered — a real ROS install wins (``setdefault``).
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class _Msg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class _Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ''


class _Image:
    def __init__(self):
        self.header = _Header()
        self.width = self.height = self.step = self.is_bigendian = 0
        self.encoding = ''
        self.data = b''


class _Node:
    def __init__(self, name):
        self.name = name
        self.params = {}
        self.publishers = []
        self.subscriptions = []
        self.timers = []

    def declare_parameter(self, name, default):
        self.params.setdefault(name, default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.params[name])

    def create_publisher(self, msg_type, topic, qos):
        pub = types.SimpleNamespace(topic=topic, sent=[])
        pub.publish = pub.sent.append
        self.publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subscriptions.append((topic, cb))

    def create_timer(self, period, cb):
        self.timers.append((period, cb))

    def count_publishers(self, topic):
        return 0

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 'stamp'))

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)


def _install_ros_stand_ins() -> None:
    if importlib.util.find_spec('rclpy') is not None:
        return
    rclpy = types.ModuleType('rclpy')
    rclpy.init = rclpy.shutdown = rclpy.spin = lambda *a, **k: None
    rclpy.ok = lambda: False
    node = types.ModuleType('rclpy.node')
    node.Node = _Node
    qos = types.ModuleType('rclpy.qos')
    qos.QoSProfile = _Msg
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    sensor = types.ModuleType('sensor_msgs.msg')
    sensor.Image = _Image
    std = types.ModuleType('std_msgs.msg')
    std.Bool = std.Int32 = std.String = _Msg
    dialog = types.ModuleType('rob_box_dialog_msgs.msg')
    dialog.ExpressionRequest = dialog.SpeechEnvelope = _Msg
    dialog.SpeechEvent = type('SpeechEvent', (_Msg,), {'KIND_STARTED': 2, 'KIND_FINISHED': 4, 'KIND_CANCELLED': 5})
    for name, mod in {
        'rclpy': rclpy, 'rclpy.node': node, 'rclpy.qos': qos,
        'sensor_msgs': types.ModuleType('sensor_msgs'), 'sensor_msgs.msg': sensor,
        'std_msgs': types.ModuleType('std_msgs'), 'std_msgs.msg': std,
        'rob_box_dialog_msgs': types.ModuleType('rob_box_dialog_msgs'), 'rob_box_dialog_msgs.msg': dialog,
    }.items():
        sys.modules.setdefault(name, mod)


_install_ros_stand_ins()
