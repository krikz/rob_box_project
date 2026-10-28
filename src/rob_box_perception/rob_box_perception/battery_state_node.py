#!/usr/bin/env python3
"""
Публикатор ``/battery_state`` (sensor_msgs/BatteryState) из ros2_control (#4002).

Broadcaster-а батареи в ros2_controllers (Humble) нет: напряжение приходит
только как joint ``battery`` / интерфейс ``voltage`` в ``/dynamic_joint_states``.
Узел берёт его оттуда и публикует ``BatteryState`` для потребителей
(``GetRobotStatusTool``, ``face_node``) на низкой частоте, не заставляя их
слушать 50 Гц поток ros2_control.

Подписывается на:
- /dynamic_joint_states (control_msgs/DynamicJointState)

Публикует:
- /battery_state (sensor_msgs/BatteryState): voltage, percentage 0..1
  (от напряжения, сглаженного EMA), present. Нет свежего напряжения -
  present=False и NaN (не «полная»).

Вся логика разбора и шкалы - в ``battery_state.py`` (чистые функции).
"""

import time

from control_msgs.msg import DynamicJointState

import rclpy
from rclpy.node import Node

from rob_box_perception.battery_state import (
    battery_fields,
    DEFAULT_SMOOTHING_TAU_S,
    DEFAULT_STALE_S,
    extract_battery_voltage,
    VoltageSmoother,
)

from sensor_msgs.msg import BatteryState


class BatteryStateNode(Node):
    """``/dynamic_joint_states`` (battery/voltage) -> ``/battery_state``."""

    def __init__(self):
        """Создать подписку и таймер публикации."""
        super().__init__('battery_state_publisher')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('stale_timeout', DEFAULT_STALE_S)
        # Постоянная времени EMA напряжения, с (0 - без сглаживания): процент
        # не должен прыгать от просадок под нагрузкой во время езды.
        self.declare_parameter('smoothing_tau', DEFAULT_SMOOTHING_TAU_S)

        self._stale_s = float(self.get_parameter('stale_timeout').value)
        self._smoother = VoltageSmoother(float(self.get_parameter('smoothing_tau').value))
        self._voltage = None
        self._voltage_at = None  # time.monotonic() последнего валидного замера

        # Топики литералами: AST-гард seam_without_consumer не видит параметры.
        self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self.on_joint_states, 10)
        self._pub = self.create_publisher(BatteryState, '/battery_state', 10)
        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self.publish_state)

    def on_joint_states(self, msg: DynamicJointState):
        """Запомнить последнее валидное напряжение батареи."""
        volts = extract_battery_voltage(msg.joint_names, msg.interface_values)
        if volts is not None:
            now = time.monotonic()
            self._voltage = volts
            self._voltage_at = now
            self._smoother.update(volts, now)

    def publish_state(self):
        """Опубликовать BatteryState (present=False, если данных нет)."""
        age = None
        if self._voltage_at is not None:
            age = time.monotonic() - self._voltage_at
        fields = battery_fields(self._voltage, age, self._stale_s, self._smoother.value)
        if not fields.present:
            self._smoother.reset()  # данные устарели: следующая выборка - без задержки
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = fields.voltage
        msg.percentage = fields.percentage
        msg.present = fields.present
        # Датчиков тока/заряда/температуры нет - честно NaN, не нули.
        nan = float('nan')
        msg.current = nan
        msg.charge = nan
        msg.capacity = nan
        msg.design_capacity = nan
        msg.temperature = nan
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.location = '10S4P Li-ion (2x smartONE 10S2P)'
        self._pub.publish(msg)


def main(args=None):
    """Точка входа."""
    rclpy.init(args=args)
    node = BatteryStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
