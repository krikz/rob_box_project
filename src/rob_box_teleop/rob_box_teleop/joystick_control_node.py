#!/usr/bin/env python3
"""
Joystick Control Node for Rob Box.

Provides direct joystick control with voice feedback:
- Detects joystick connection
- ARM button activates motors with voice confirmation
- Publishes cmd_vel_joy for robot control
"""

import os
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class JoystickControlNode(Node):
    """Node for direct joystick control with voice feedback."""

    def __init__(self):
        super().__init__("joystick_control_node")

        # Parameters
        self.declare_parameter("device_path", "/dev/input/js0")
        self.declare_parameter("device_name", "ExpressLRS Joystick")
        self.declare_parameter("check_interval", 2.0)
        self.declare_parameter("axis_linear_x", 1)
        self.declare_parameter("axis_angular_z", 4)
        self.declare_parameter("enable_button", 0)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 15.0)
        self.declare_parameter("deadzone", 0.1)
        self.declare_parameter("enable_voice_feedback", True)

        self.device_path = self.get_parameter("device_path").value
        self.device_name = self.get_parameter("device_name").value
        self.check_interval = self.get_parameter("check_interval").value
        self.axis_linear = self.get_parameter("axis_linear_x").value
        self.axis_angular = self.get_parameter("axis_angular_z").value
        self.enable_button = self.get_parameter("enable_button").value
        self.max_linear = self.get_parameter("max_linear_speed").value
        self.max_angular = self.get_parameter("max_angular_speed").value
        self.deadzone = self.get_parameter("deadzone").value
        self.enable_voice = self.get_parameter("enable_voice_feedback").value

        # State
        self.device_connected = False
        self.last_joy_msg: Optional[Joy] = None
        self.was_enabled = False  # Для отслеживания изменения состояния

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel_joy", 10)
        self.tts_pub = self.create_publisher(String, "/tts/speak", 10)

        # Subscriber
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Timer for device monitoring
        self.device_check_timer = self.create_timer(self.check_interval, self.check_device)

        self.get_logger().info(
            f"🎮 Joystick Control Node started\n"
            f"   Device: {self.device_path}\n"
            f"   Expected: {self.device_name}\n"
            f"   Enable button: {self.enable_button} (hold to enable)\n"
            f"   Max speeds: linear={self.max_linear} m/s, angular={self.max_angular} rad/s"
        )

        # Initial device check
        self.check_device()

    def check_device(self):
        """Check if joystick device is connected."""
        device_exists = Path(self.device_path).exists()

        if device_exists and not self.device_connected:
            # Device just connected
            self.device_connected = True
            self.get_logger().info(f"✅ Joystick connected: {self.device_path}")
            if self.enable_voice:
                self.speak("Джойстик подключен")

        elif not device_exists and self.device_connected:
            # Device disconnected
            self.device_connected = False
            self.was_enabled = False
            self.get_logger().warn(f"⚠️  Joystick disconnected: {self.device_path}")
            if self.enable_voice:
                self.speak("Джойстик отключен")
            # Stop robot
            self.publish_stop()

    def joy_callback(self, msg: Joy):
        """Process joystick messages."""
        self.last_joy_msg = msg

        # Check enable button (must be held down)
        button_pressed = False
        if len(msg.buttons) > self.enable_button:
            button_pressed = msg.buttons[self.enable_button] == 1

        # Voice feedback on state change
        if button_pressed and not self.was_enabled:
            self.get_logger().info("🚀 Enable button pressed - Ready to drive!")
            if self.enable_voice:
                self.speak("Моторы активированы, готов к езде")
            self.was_enabled = True
        elif not button_pressed and self.was_enabled:
            self.get_logger().info("🛑 Enable button released")
            if self.enable_voice:
                self.speak("Моторы отключены")
            self.was_enabled = False
            self.publish_stop()

        # Publish cmd_vel only if button is held
        if button_pressed:
            self.publish_cmd_vel(msg)
        else:
            self.publish_stop()

    def publish_cmd_vel(self, joy_msg: Joy):
        """Convert joystick to velocity commands."""
        twist = Twist()

        # Get axis values with deadzone
        if len(joy_msg.axes) > self.axis_linear:
            linear_raw = -joy_msg.axes[self.axis_linear]  # Invert Y axis
            twist.linear.x = self.apply_deadzone(linear_raw) * self.max_linear

        if len(joy_msg.axes) > self.axis_angular:
            angular_raw = -joy_msg.axes[self.axis_angular]  # Invert for natural control
            twist.angular.z = self.apply_deadzone(angular_raw) * self.max_angular

        self.cmd_vel_pub.publish(twist)

    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to axis value."""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale to full range after deadzone
        sign = 1.0 if value > 0 else -1.0
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * scaled

    def publish_stop(self):
        """Publish zero velocity."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def speak(self, text: str):
        """Publish text-to-speech message."""
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        self.get_logger().info(f"🗣️  TTS: {text}")


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
