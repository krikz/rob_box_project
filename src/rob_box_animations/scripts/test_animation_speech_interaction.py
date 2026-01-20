#!/usr/bin/env python3
"""
Test script to validate animation and speech interaction.

This script simulates the sequence:
1. Set emotion (manual animation)
2. Start speaking (should switch to talking immediately)
3. Stop speaking (should return to idle)

Run this script to manually test the fix for animation blocking speech.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import argparse


class AnimationSpeechTester(Node):
    """Test node for animation and speech interaction"""

    def __init__(self):
        super().__init__("animation_speech_tester")

        # Publisher for animation requests
        self.animation_pub = self.create_publisher(String, "/voice/animation/request", 10)

        # Publisher for TTS state
        self.tts_state_pub = self.create_publisher(String, "/voice/tts/state", 10)

        # Subscriber for animation status
        self.status_sub = self.create_subscription(String, "/animation_player/status", self.status_callback, 10)

        self.current_animation = None
        self.get_logger().info("✅ Animation Speech Tester ready")

    def status_callback(self, msg):
        """Track current animation"""
        if "Animation:" in msg.data:
            parts = msg.data.split(",")
            for part in parts:
                if "Animation:" in part:
                    self.current_animation = part.split(":")[1].strip()

    def request_animation(self, animation_name: str):
        """Request an animation"""
        msg = String()
        msg.data = animation_name
        self.animation_pub.publish(msg)
        self.get_logger().info(f"📤 Requested animation: {animation_name}")

    def set_tts_state(self, state: str):
        """Set TTS state"""
        msg = String()
        msg.data = state
        self.tts_state_pub.publish(msg)
        self.get_logger().info(f"📢 TTS state: {state}")

    def run_test_sequence(self):
        """Run the test sequence"""
        self.get_logger().info("🧪 Starting test sequence...")
        self.get_logger().info("")

        # Step 1: Set emotion (happy)
        self.get_logger().info("Step 1: Setting emotion to 'happy'")
        self.request_animation("happy")
        time.sleep(0.5)  # Allow animation to start

        # Step 2: Start speaking (should switch to talking immediately)
        self.get_logger().info("Step 2: Robot starts speaking (state: playing)")
        self.get_logger().info("   ⚠️  EXPECTED: Should switch to 'talking' animation IMMEDIATELY")
        self.set_tts_state("playing")
        time.sleep(1.0)  # Give time for animation switch

        # Check if animation switched to talking
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_animation and "talking" in self.current_animation.lower():
            self.get_logger().info("   ✅ SUCCESS: Animation switched to talking!")
        else:
            self.get_logger().warn(f"   ❌ FAIL: Animation is still '{self.current_animation}', expected 'talking'")

        # Step 3: Continue speaking for a few seconds
        self.get_logger().info("Step 3: Robot continues speaking...")
        time.sleep(3.0)

        # Step 4: Stop speaking
        self.get_logger().info("Step 4: Robot stops speaking (state: idle)")
        self.get_logger().info("   ⚠️  EXPECTED: Should return to 'idle' animation")
        self.set_tts_state("idle")
        time.sleep(1.0)  # Give time for animation switch

        # Check if animation switched to idle
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_animation and "idle" in self.current_animation.lower():
            self.get_logger().info("   ✅ SUCCESS: Animation returned to idle!")
        else:
            self.get_logger().warn(f"   ❌ FAIL: Animation is still '{self.current_animation}', expected 'idle'")

        self.get_logger().info("")
        self.get_logger().info("🏁 Test sequence complete!")
        self.get_logger().info("")
        self.get_logger().info("✅ If both tests passed, the fix is working correctly.")
        self.get_logger().info("   The robot should speak immediately after emotion is set, not wait for animation.")


def main(args=None):
    parser = argparse.ArgumentParser(description="Test animation and speech interaction")
    parser.add_argument("--duration", type=float, default=10.0, help="Duration before stopping test (default: 10s)")
    cli_args = parser.parse_args()

    rclpy.init(args=args)
    node = AnimationSpeechTester()

    # Wait for connections
    node.get_logger().info("Waiting for animation_player to be ready...")
    time.sleep(2.0)

    try:
        # Run test sequence
        node.run_test_sequence()

        # Spin for a bit to see final state
        node.get_logger().info(f"Spinning for {cli_args.duration}s to observe behavior...")
        end_time = time.time() + cli_args.duration
        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
