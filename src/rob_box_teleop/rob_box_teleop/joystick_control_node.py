#!/usr/bin/env python3
"""
Joystick Control Node for Rob Box.

Provides direct joystick control with voice feedback:
- Reads ExpressLRS SBUS receiver via serial port
- ARM channel activates motors with voice confirmation
- Publishes cmd_vel_joy for robot control
"""

import struct
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from rob_box_teleop.sbus import (
    SBUS_CHANNEL_CENTER,
    SBUS_FLAG_REJECT,
    SBUS_FRAME_SIZE,
    SBUS_STALE_TIMEOUT,
    decode_channels,
    read_frame,
)

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class JoystickControlNode(Node):
    """Node for SBUS joystick control with voice feedback."""

    def __init__(self):
        super().__init__("joystick_control_node")

        # Parameters
        self.declare_parameter("use_sbus", True)  # True for SBUS serial, False for /dev/input/js0
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("serial_baudrate", 100000)
        self.declare_parameter("device_path", "/dev/input/js0")
        self.declare_parameter("device_name", "ExpressLRS SBUS")
        self.declare_parameter("channel_roll", 0)  # SBUS channel for roll (0-15)
        self.declare_parameter("channel_pitch", 1)  # SBUS channel for pitch
        self.declare_parameter("channel_throttle", 2)  # SBUS channel for throttle
        self.declare_parameter("channel_yaw", 3)  # SBUS channel for yaw
        self.declare_parameter("channel_arm", 4)  # SBUS channel for ARM switch
        self.declare_parameter("axis_linear_x", 1)  # For non-SBUS mode
        self.declare_parameter("axis_angular_z", 4)
        self.declare_parameter("enable_button", 0)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 15.0)
        self.declare_parameter("deadzone", 0.1)
        self.declare_parameter("enable_voice_feedback", True)

        self.use_sbus = self.get_parameter("use_sbus").value
        self.serial_port = self.get_parameter("serial_port").value
        self.serial_baudrate = self.get_parameter("serial_baudrate").value
        self.device_path = self.get_parameter("device_path").value
        self.device_name = self.get_parameter("device_name").value
        self.ch_roll = self.get_parameter("channel_roll").value
        self.ch_pitch = self.get_parameter("channel_pitch").value
        self.ch_throttle = self.get_parameter("channel_throttle").value
        self.ch_yaw = self.get_parameter("channel_yaw").value
        self.ch_arm = self.get_parameter("channel_arm").value
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
        self.serial_conn: Optional[serial.Serial] = None
        self.sbus_channels = [SBUS_CHANNEL_CENTER] * 16  # SBUS center value (172-1811 range)
        self.sbus_packet_count = 0
        self.last_valid_packet_time = 0.0  # monotonic time of last valid (non-failsafe) frame
        self.sbus_link_ok = True

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel_joy", 10)
        self.tts_pub = self.create_publisher(String, "/tts/speak", 10)
        self.joy_pub = self.create_publisher(Joy, "joy", 10)

        # Subscriber (only if not using SBUS)
        if not self.use_sbus:
            self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Timer for publishing joy messages from SBUS data
        if self.use_sbus:
            self.joy_timer = self.create_timer(0.05, self.publish_joy_from_sbus)  # 20Hz

        self.get_logger().info(
            f"🎮 Joystick Control Node started\n"
            f"   Mode: {'SBUS Serial' if self.use_sbus else 'HID via joy_node'}\n"
            f"   Device: {self.serial_port if self.use_sbus else self.device_path}\n"
            f"   ARM channel: {self.ch_arm} (switch to enable)\n"
            f"   Max speeds: linear={self.max_linear} m/s, angular={self.max_angular} rad/s"
        )

        # Start SBUS reader if enabled
        if self.use_sbus:
            if not SERIAL_AVAILABLE:
                self.get_logger().error("❌ pyserial library not available! Install python3-serial")
                raise RuntimeError("pyserial not available")
            # Start SBUS reader in separate thread
            self.sbus_thread = threading.Thread(target=self._run_sbus_reader, daemon=True)
            self.sbus_thread.start()

    def _run_sbus_reader(self):
        """Run SBUS reader in separate thread with auto-reconnect."""
        while True:
            try:
                self.get_logger().info(f"🔌 Opening SBUS serial port: {self.serial_port}")
                # SBUS: 100000 baud, 8E2 (8 data bits, Even parity, 2 stop bits)
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.serial_baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_EVEN,
                    stopbits=serial.STOPBITS_TWO,
                    timeout=1.0
                )
                self.device_connected = True
                self.get_logger().info(f"✅ SBUS connected on {self.serial_port}")
                if self.enable_voice:
                    self.speak("Джойстик подключен по СБАС")

                # Read SBUS packets continuously
                while self.serial_conn and self.serial_conn.is_open:
                    packet = self._read_sbus_packet()
                    if packet:
                        self._parse_sbus_packet(packet)

            except serial.SerialException as e:
                self.get_logger().warn(f"⚠️  Serial connection failed: {e}")
                self.device_connected = False
                if self.serial_conn:
                    try:
                        self.serial_conn.close()
                    except:
                        pass
                self.get_logger().info("🔄 Retrying in 5 seconds...")
                time.sleep(5)
            except Exception as e:
                self.get_logger().error(f"❌ SBUS reader error: {e}")
                time.sleep(5)

    def _read_sbus_packet(self) -> Optional[bytes]:
        """Read and validate one SBUS packet (25 bytes).

        Delegates to :func:`rob_box_teleop.sbus.read_frame`, which validates
        header/footer/channel range and resyncs with a sliding window, so a
        0x0F byte inside channel data cannot permanently desync the reader
        (issue #1345: throttle pulsing 0.999/0.0).
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return None

        try:
            return read_frame(self.serial_conn.read)
        except Exception as e:
            self.get_logger().debug(f"SBUS read error: {e}")
            return None

    def _parse_sbus_packet(self, packet: bytes):
        """Parse and store a 25-byte SBUS packet.

        Frames flagged frame-lost/failsafe carry unreliable channel data: on
        a link-loss transition channels are reset to neutral so the robot
        stops instead of driving with stale values.
        """
        if len(packet) != SBUS_FRAME_SIZE:
            return

        flags = packet[23]
        if flags & SBUS_FLAG_REJECT:
            self.sbus_channels = [SBUS_CHANNEL_CENTER] * 16
            if self.sbus_link_ok:
                self.sbus_link_ok = False
                self.get_logger().warn(
                    f"⚠️ SBUS link loss (flags=0x{flags:02x}) — channels reset to neutral"
                )
            return

        try:
            channels = decode_channels(packet)
        except ValueError as e:
            self.get_logger().debug(f"SBUS parse error: {e}")
            return

        self.sbus_channels = channels
        self.sbus_packet_count += 1
        self.last_valid_packet_time = time.monotonic()
        if not self.sbus_link_ok:
            self.sbus_link_ok = True
            self.get_logger().info("✅ SBUS link recovered")

        # Log first few packets for debugging
        if self.sbus_packet_count <= 5 or self.sbus_packet_count % 100 == 0:
            self.get_logger().info(
                f"📨 SBUS packet #{self.sbus_packet_count}: "
                f"Ch1={channels[0]} Ch2={channels[1]} Ch3={channels[2]} Ch4={channels[3]} "
                f"ARM(Ch{self.ch_arm + 1})={channels[self.ch_arm]}"
            )

    def publish_joy_from_sbus(self):
        """Publish Joy message from SBUS data and process it."""
        if not self.device_connected:
            return

        # If no valid (non-failsafe) frame arrived recently, treat the link
        # as dead and publish neutral instead of holding the last commanded
        # throttle. VESC's own 0.5s timeout would eventually relax the
        # motors, but we react faster and keep /joy honest.
        if time.monotonic() - self.last_valid_packet_time > SBUS_STALE_TIMEOUT:
            channels = [SBUS_CHANNEL_CENTER] * 16
        else:
            channels = self.sbus_channels

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()

        # Convert SBUS channels (172-1811) to Joy axes (-1.0 to 1.0)
        # SBUS center: 992, min: 172, max: 1811
        axes = []
        for ch in channels:
            # Normalize to [-1.0, 1.0]
            normalized = (ch - 992.0) / 819.5  # 819.5 = (1811-172)/2
            normalized = max(-1.0, min(1.0, normalized))  # Clamp
            axes.append(normalized)

        joy_msg.axes = axes

        # Convert digital channels to buttons (threshold at 1500)
        buttons = []
        for ch in channels:
            buttons.append(1 if ch > 1500 else 0)

        joy_msg.buttons = buttons

        # Publish to /joy topic
        self.joy_pub.publish(joy_msg)

        # Process for cmd_vel (use ARM channel as enable)
        self.joy_callback_sbus(joy_msg)

    def joy_callback_sbus(self, msg: Joy):
        """Process SBUS joystick messages.

        When disarmed, we do NOT publish to cmd_vel_joy so that twist_mux
        times out the joystick source and falls through to lower-priority
        inputs (web, nav2, voice). A single stop message is sent on the
        armed→disarmed transition for immediate braking.
        """
        self.last_joy_msg = msg

        # ARM channel is used as enable (>1500 = armed)
        button_pressed = False
        if len(msg.buttons) > self.ch_arm:
            button_pressed = msg.buttons[self.ch_arm] == 1

        # Voice feedback on state change
        if button_pressed and not self.was_enabled:
            self.get_logger().info("🚀 ARM channel HIGH - Ready to drive!")
            if self.enable_voice:
                self.speak("Моторы активированы, готов к езде")
            self.was_enabled = True
        elif not button_pressed and self.was_enabled:
            self.get_logger().info("🛑 ARM channel LOW")
            if self.enable_voice:
                self.speak("Моторы отключены")
            self.was_enabled = False
            # Send ONE stop message for immediate braking, then let twist_mux timeout
            self.publish_stop()

        # Publish cmd_vel only if armed; when disarmed, don't publish
        # so twist_mux times out cmd_vel_joy and uses lower-priority sources
        if button_pressed:
            self.publish_cmd_vel_sbus(msg)

    def publish_cmd_vel_sbus(self, joy_msg: Joy):
        """Convert SBUS joystick to velocity commands."""
        twist = Twist()

        # Use configured channels for control
        # Typical mapping: Ch1=Roll, Ch2=Pitch, Ch3=Throttle, Ch4=Yaw
        if len(joy_msg.axes) > self.ch_pitch:
            # Pitch (forward/backward) -> linear.x
            linear_raw = joy_msg.axes[self.ch_pitch]
            twist.linear.x = self.apply_deadzone(linear_raw) * self.max_linear

        if len(joy_msg.axes) > self.ch_yaw:
            # Yaw (left/right) -> angular.z
            angular_raw = joy_msg.axes[self.ch_yaw]
            twist.angular.z = self.apply_deadzone(angular_raw) * self.max_angular

        self.cmd_vel_pub.publish(twist)


    def joy_callback(self, msg: Joy):
        """Process joystick messages (for non-SBUS mode).

        Same logic as SBUS mode: only publish when enabled, let twist_mux
        timeout handle the fallthrough when disabled.
        """
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
            # Send ONE stop message for immediate braking, then let twist_mux timeout
            self.publish_stop()

        # Publish cmd_vel only if button is held; when released, don't publish
        # so twist_mux times out cmd_vel_joy and uses lower-priority sources
        if button_pressed:
            self.publish_cmd_vel(msg)

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
        if node.serial_conn and node.serial_conn.is_open:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
