#!/usr/bin/env python3
"""
Joystick Control Node for Rob Box.

Provides direct joystick control with voice feedback:
- Reads ExpressLRS BLE joystick directly via Bleak
- ARM button activates motors with voice confirmation
- Publishes cmd_vel_joy for robot control
"""

import asyncio
import os
import struct
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    from bleak import BleakClient, BleakScanner
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False


class JoystickControlNode(Node):
    """Node for direct joystick control with voice feedback."""

    def __init__(self):
        super().__init__("joystick_control_node")

        # Parameters
        self.declare_parameter("use_ble", True)  # True for BLE, False for /dev/input/js0
        self.declare_parameter("ble_mac", "8C:4F:00:C2:04:96")
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

        self.use_ble = self.get_parameter("use_ble").value
        self.ble_mac = self.get_parameter("ble_mac").value
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
        self.was_enabled = False
        self.ble_client: Optional[BleakClient] = None
        self.joy_axes = [0.0] * 8  # 8 axes for ExpressLRS
        self.joy_buttons = [0] * 16  # 16 buttons

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel_joy", 10)
        self.tts_pub = self.create_publisher(String, "/tts/speak", 10)
        self.joy_pub = self.create_publisher(Joy, "joy", 10)

        # Subscriber (only if not using BLE)
        if not self.use_ble:
            self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Timer for publishing joy messages from BLE data
        if self.use_ble:
            self.joy_timer = self.create_timer(0.05, self.publish_joy_from_ble)  # 20Hz

        self.get_logger().info(
            f"🎮 Joystick Control Node started\n"
            f"   Mode: {'BLE Direct' if self.use_ble else 'HID via joy_node'}\n"
            f"   Device: {self.ble_mac if self.use_ble else self.device_path}\n"
            f"   Enable button: {self.enable_button} (hold to enable)\n"
            f"   Max speeds: linear={self.max_linear} m/s, angular={self.max_angular} rad/s"
        )

        # Start BLE connection if enabled
        if self.use_ble:
            if not BLEAK_AVAILABLE:
                self.get_logger().error("❌ Bleak library not available! Install python3-bleak")
                raise RuntimeError("Bleak not available")
            # Start BLE connection in separate thread
            import threading
            self.ble_thread = threading.Thread(target=self._run_ble_connection, daemon=True)
            self.ble_thread.start()

    def _run_ble_connection(self):
        """Run BLE connection in separate thread with its own event loop."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self.connect_ble_joystick())
        finally:
            loop.close()

    async def connect_ble_joystick(self):
        """Connect to BLE joystick and setup notifications."""
        self.get_logger().info(f"🔍 Scanning for {self.device_name} ({self.ble_mac})...")
        
        try:
            # Connect directly without scanning (device may already be paired)
            self.get_logger().info(f"🔌 Connecting directly to {self.ble_mac}...")
            self.get_logger().info(f"   (Bypassing scan - using known MAC address)")
            
            async with BleakClient(self.ble_mac) as client:
                self.ble_client = client
                self.device_connected = True
                self.get_logger().info(f"✅ Connected!")
                
                if self.enable_voice:
                    self.speak("Джойстик подключен по блютус")
                
                # Find HID Report characteristic (UUID 0x2A4D or custom)
                # ExpressLRS typically uses HID Report for input
                hid_report_uuid = "00002a4d-0000-1000-8000-00805f9b34fb"
                
                # Subscribe to all notify characteristics
                for service in client.services:
                    for char in service.characteristics:
                        if "notify" in char.properties:
                            self.get_logger().info(f"📡 Subscribing to {char.uuid}")
                            await client.start_notify(char.uuid, self.ble_notification_handler)
                
                # Keep connection alive
                while client.is_connected:
                    await asyncio.sleep(1.0)
                
                self.get_logger().warn("⚠️  BLE connection lost")
                self.device_connected = False
                if self.enable_voice:
                    self.speak("Джойстик отключен")
                    
        except Exception as e:
            self.get_logger().error(f"❌ BLE error: {e}")
            self.device_connected = False

    def ble_notification_handler(self, sender, data: bytearray):
        """Handle BLE notifications from joystick."""
        # ExpressLRS joystick sends HID reports
        # Standard HID joystick report format (varies by device):
        # Byte 0: Buttons (bitmap)
        # Bytes 1-2: X axis (int16)
        # Bytes 3-4: Y axis (int16)
        # Bytes 5-6: Z axis (int16)
        # Bytes 7-8: RZ axis (int16)
        # etc.
        
        if len(data) < 2:
            return
        
        try:
            # Parse buttons (first byte typically)
            if len(data) >= 1:
                button_byte = data[0]
                for i in range(8):
                    self.joy_buttons[i] = 1 if (button_byte & (1 << i)) else 0
            
            # Parse axes (as int16, normalized to [-1.0, 1.0])
            offset = 1
            axis_idx = 0
            while offset + 1 < len(data) and axis_idx < 8:
                raw_value = struct.unpack_from('<h', data, offset)[0]  # signed int16 little-endian
                # Normalize from [-32768, 32767] to [-1.0, 1.0]
                self.joy_axes[axis_idx] = raw_value / 32768.0
                offset += 2
                axis_idx += 1
                
        except Exception as e:
            self.get_logger().debug(f"Parse error: {e}")

    def publish_joy_from_ble(self):
        """Publish Joy message from BLE data and process it."""
        if not self.device_connected:
            return
        
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = self.joy_axes.copy()
        joy_msg.buttons = self.joy_buttons.copy()
        
        # Publish to /joy topic (for compatibility with joy_node)
        self.joy_pub.publish(joy_msg)
        
        # Process for cmd_vel
        self.joy_callback(joy_msg)

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
        if node.ble_client and node.ble_client.is_connected:
            # Disconnect BLE in blocking way
            import asyncio
            loop = asyncio.new_event_loop()
            loop.run_until_complete(node.ble_client.disconnect())
            loop.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
