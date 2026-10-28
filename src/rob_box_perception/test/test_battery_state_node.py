"""Тесты BatteryStateNode (#4002): /dynamic_joint_states -> /battery_state.

Как и test_context_aggregator.py, ставит минимальные стабы ``rclpy`` /
``control_msgs`` / ``sensor_msgs`` (conftest откатывает их после сбора),
поэтому гоняется обычным ``pytest`` без ROS 2. Реальный rclpy-запуск узла
здесь НЕ проверяется - только логика on_joint_states / publish_state.
"""

from __future__ import annotations

import math
import sys
import types as _types
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest


class _FakeNode:

    def __init__(self, name, **kwargs):
        self._params = {}
        self.publishers = {}
        self.subs = {}
        self.timers = []

    def get_logger(self):
        return MagicMock()

    def declare_parameter(self, name, default=None):
        self._params.setdefault(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self._params.get(name))

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subs[topic] = callback

    def create_publisher(self, msg_type, topic, depth):
        pub = MagicMock()
        pub.published = []
        pub.publish = pub.published.append
        self.publishers[topic] = pub
        return pub

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.to_msg.return_value = 'stamp'
        return clock

    def destroy_node(self):
        pass


class _BatteryState:
    POWER_SUPPLY_STATUS_UNKNOWN = 0
    POWER_SUPPLY_HEALTH_UNKNOWN = 0
    POWER_SUPPLY_TECHNOLOGY_LION = 2

    def __init__(self):
        self.header = SimpleNamespace(stamp=None)


def _module(name, **attrs):
    mod = _types.ModuleType(name)
    mod.__dict__.update(attrs)
    sys.modules[name] = mod
    return mod


_noop = lambda *a, **k: None  # noqa: E731
_module('rclpy', init=_noop, shutdown=_noop, spin=_noop)
_module('rclpy.node', Node=_FakeNode)
_module('control_msgs')
_module('control_msgs.msg', DynamicJointState=type('DynamicJointState', (), {}))
_module('sensor_msgs')
_module('sensor_msgs.msg', BatteryState=_BatteryState)

from rob_box_perception import battery_state_node as bsn  # noqa: E402


def _joint_state(joints):
    return SimpleNamespace(
        joint_names=[name for name, _ in joints],
        interface_values=[
            SimpleNamespace(interface_names=[i for i, _ in ifaces], values=[v for _, v in ifaces])
            for _, ifaces in joints
        ])


@pytest.fixture
def node(monkeypatch):
    clock = {'t': 1000.0}
    monkeypatch.setattr(bsn.time, 'monotonic', lambda: clock['t'])
    n = bsn.BatteryStateNode()
    n.clock = clock
    return n


def _last(node):
    return node.publishers['/battery_state'].published[-1]


def test_wiring(node):
    assert '/dynamic_joint_states' in node.subs
    assert '/battery_state' in node.publishers
    assert node.timers and node.timers[0][0] == pytest.approx(0.5)


def test_publishes_voltage_and_fraction_when_fresh(node):
    node.on_joint_states(_joint_state([
        ('left_front_wheel_joint', [('position', 1.0)]),
        ('battery', [('voltage', 41.0)]),
    ]))
    node.publish_state()
    msg = _last(node)
    assert msg.present is True
    assert msg.voltage == 41.0
    assert msg.percentage == pytest.approx(0.90)  # кривая Li-ion 10S
    assert msg.power_supply_status == _BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    assert msg.power_supply_technology == _BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
    assert msg.header.stamp == 'stamp'


def test_no_data_yet_is_not_present_and_not_full(node):
    node.publish_state()
    msg = _last(node)
    assert msg.present is False
    assert math.isnan(msg.percentage)
    assert math.isnan(msg.voltage)


def test_message_without_battery_joint_publishes_no_data(node):
    node.on_joint_states(_joint_state([('left_front_wheel_joint', [('position', 1.0)])]))
    node.publish_state()
    assert _last(node).present is False


def test_stale_voltage_turns_into_not_present(node):
    node.on_joint_states(_joint_state([('battery', [('voltage', 41.0)])]))
    node.clock['t'] += 5.01
    node.publish_state()
    msg = _last(node)
    assert msg.present is False
    assert math.isnan(msg.percentage)


def test_recovers_after_fresh_sample(node):
    node.on_joint_states(_joint_state([('battery', [('voltage', 41.0)])]))
    node.clock['t'] += 10.0
    node.publish_state()
    assert _last(node).present is False
    node.on_joint_states(_joint_state([('battery', [('voltage', 36.0)])]))
    node.publish_state()
    assert _last(node).present is True
    assert _last(node).percentage == pytest.approx(0.15)  # 36 В = 3.6 В/банку


def test_zero_voltage_sample_does_not_refresh(node):
    node.on_joint_states(_joint_state([('battery', [('voltage', 0.0)])]))
    node.publish_state()
    assert _last(node).present is False


def test_sag_under_load_is_smoothed_but_raw_voltage_published(node):
    """Просадка 41 -> 37 В на 2 с: voltage сырой, percentage почти не прыгает."""
    node.on_joint_states(_joint_state([('battery', [('voltage', 41.0)])]))
    node.clock['t'] += 2.0
    node.on_joint_states(_joint_state([('battery', [('voltage', 37.0)])]))
    node.publish_state()
    msg = _last(node)
    assert msg.voltage == 37.0
    assert msg.percentage > 0.80  # сырой 37 В дал бы 0.35


def test_first_sample_is_not_delayed_and_stale_resets_smoother(node):
    node.on_joint_states(_joint_state([('battery', [('voltage', 41.0)])]))
    node.publish_state()
    assert _last(node).percentage == pytest.approx(0.90)
    node.clock['t'] += 60.0  # данные устарели -> история забыта
    node.publish_state()
    assert _last(node).present is False
    node.on_joint_states(_joint_state([('battery', [('voltage', 36.0)])]))
    node.publish_state()
    assert _last(node).percentage == pytest.approx(0.15)  # без остатка от 41 В


def test_smoothing_tau_zero_disables_smoothing(monkeypatch):
    clock = {'t': 0.0}
    monkeypatch.setattr(bsn.time, 'monotonic', lambda: clock['t'])
    n = bsn.BatteryStateNode()
    n._smoother.tau_s = 0.0
    n.on_joint_states(_joint_state([('battery', [('voltage', 41.0)])]))
    clock['t'] += 2.0
    n.on_joint_states(_joint_state([('battery', [('voltage', 37.0)])]))
    n.publish_state()
    assert n.publishers['/battery_state'].published[-1].percentage == pytest.approx(0.35)
