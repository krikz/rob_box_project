#!/usr/bin/env python3
"""
robot_sensor_hub Testing Script

Subscribe to /device/snapshot and display sensor data.
Publish test commands to /device/command.

Usage:
    python3 test_sensor_hub.py
"""

import rclpy
from rclpy.node import Node
from robot_sensor_hub_msg.msg import DeviceSnapshot, DeviceCommand, DeviceData
import time


# Constants from firmware
DEVICE_TYPE_AHT30 = 0
DEVICE_TYPE_HX711 = 1
DEVICE_TYPE_FAN = 2

DATA_TYPE_TEMPERATURE = 1
DATA_TYPE_HUMIDITY = 2
DATA_TYPE_WEIGHT = 3
DATA_TYPE_SPEED = 4
DATA_TYPE_RPM = 5

COMMAND_SET_SPEED = 0
COMMAND_TARE_SCALE = 1


class SensorHubTest(Node):
    def __init__(self):
        super().__init__('sensor_hub_test')
        
        # Subscriber
        self.snapshot_sub = self.create_subscription(
            DeviceSnapshot,
            '/device/snapshot',
            self.snapshot_callback,
            10
        )
        
        # Publisher
        self.command_pub = self.create_publisher(
            DeviceCommand,
            '/device/command',
            10
        )
        
        self.get_logger().info("🚀 sensor_hub_test node started")
        self.get_logger().info("Waiting for /device/snapshot messages...")
        
        self.last_snapshot_time = None
    
    def snapshot_callback(self, msg: DeviceSnapshot):
        """Process device snapshot"""
        
        # Calculate frequency
        current_time = time.time()
        if self.last_snapshot_time is not None:
            freq = 1.0 / (current_time - self.last_snapshot_time)
        else:
            freq = 0.0
        self.last_snapshot_time = current_time
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"📊 DEVICE SNAPSHOT (freq: {freq:.2f} Hz)")
        self.get_logger().info(f"Timestamp: {msg.timestamp} ns")
        self.get_logger().info(f"Device count: {len(msg.devices)}")
        
        # Parse devices
        temps = []
        hums = []
        weights = []
        fan_speeds = []
        fan_rpms = []
        
        for d in msg.devices:
            if d.device_type == DEVICE_TYPE_AHT30:
                if d.data_type == DATA_TYPE_TEMPERATURE:
                    temps.append((d.device_id, d.value, d.error_code))
                elif d.data_type == DATA_TYPE_HUMIDITY:
                    hums.append((d.device_id, d.value, d.error_code))
            
            elif d.device_type == DEVICE_TYPE_HX711:
                if d.data_type == DATA_TYPE_WEIGHT:
                    weights.append((d.device_id, d.value, d.error_code))
            
            elif d.device_type == DEVICE_TYPE_FAN:
                if d.data_type == DATA_TYPE_SPEED:
                    fan_speeds.append((d.device_id, d.value, d.error_code))
                elif d.data_type == DATA_TYPE_RPM:
                    fan_rpms.append((d.device_id, d.value, d.error_code))
        
        # Display results
        if temps:
            self.get_logger().info("\n🌡️  TEMPERATURES:")
            for device_id, value, error in temps:
                status = "✅" if error == 0 else f"❌ (err={error})"
                self.get_logger().info(f"  AHT30[{device_id}]: {value:>6.2f}°C {status}")
        
        if hums:
            self.get_logger().info("\n💧 HUMIDITY:")
            for device_id, value, error in hums:
                status = "✅" if error == 0 else f"❌ (err={error})"
                self.get_logger().info(f"  AHT30[{device_id}]: {value:>6.2f}% {status}")
        
        if weights:
            self.get_logger().info("\n⚖️  WEIGHT:")
            for device_id, value, error in weights:
                status = "✅" if error == 0 else f"❌ (err={error})"
                self.get_logger().info(f"  HX711[{device_id}]: {value:>8.2f}g {status}")
        
        if fan_speeds or fan_rpms:
            self.get_logger().info("\n🌀 FANS:")
            # Combine speed and RPM by device_id
            for device_id in set([s[0] for s in fan_speeds] + [r[0] for r in fan_rpms]):
                speed_data = next((s for s in fan_speeds if s[0] == device_id), None)
                rpm_data = next((r for r in fan_rpms if r[0] == device_id), None)
                
                speed_str = f"{speed_data[1]*100:>5.1f}%" if speed_data else "  N/A"
                rpm_str = f"{rpm_data[1]:>6.0f} RPM" if rpm_data else "   N/A"
                
                self.get_logger().info(f"  Fan[{device_id}]: {speed_str} | {rpm_str}")
        
        self.get_logger().info(f"{'='*60}\n")
    
    def set_fan_speed(self, fan_id: int, speed: float):
        """Set fan speed (0.0-1.0)"""
        cmd = DeviceCommand()
        cmd.device_type = DEVICE_TYPE_FAN
        cmd.device_id = fan_id
        cmd.command_code = COMMAND_SET_SPEED
        cmd.param_1 = max(0.0, min(1.0, speed))  # Clamp to [0, 1]
        cmd.param_2 = 0.0
        
        self.command_pub.publish(cmd)
        self.get_logger().info(f"🎛️  Set Fan[{fan_id}] speed to {speed*100:.0f}%")
    
    def tare_scale(self):
        """Tare/zero the load cell"""
        cmd = DeviceCommand()
        cmd.device_type = DEVICE_TYPE_HX711
        cmd.device_id = 0
        cmd.command_code = COMMAND_TARE_SCALE
        cmd.param_1 = 0.0
        cmd.param_2 = 0.0
        
        self.command_pub.publish(cmd)
        self.get_logger().info("⚖️  Tare command sent to HX711")


def main(args=None):
    rclpy.init(args=args)
    
    node = SensorHubTest()
    
    try:
        # Spin in separate thread
        from threading import Thread
        spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        
        # Interactive commands
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("Commands:")
        node.get_logger().info("  f1 <speed>  - Set fan 1 speed (0.0-1.0)")
        node.get_logger().info("  f2 <speed>  - Set fan 2 speed (0.0-1.0)")
        node.get_logger().info("  tare        - Zero the load cell")
        node.get_logger().info("  quit        - Exit")
        node.get_logger().info("="*60 + "\n")
        
        while rclpy.ok():
            try:
                cmd = input("> ").strip().lower()
                
                if cmd == "quit" or cmd == "exit":
                    break
                
                elif cmd == "tare":
                    node.tare_scale()
                
                elif cmd.startswith("f1 "):
                    try:
                        speed = float(cmd.split()[1])
                        node.set_fan_speed(0, speed)
                    except (ValueError, IndexError):
                        node.get_logger().warn("Usage: f1 <speed>  (e.g., f1 0.75)")
                
                elif cmd.startswith("f2 "):
                    try:
                        speed = float(cmd.split()[1])
                        node.set_fan_speed(1, speed)
                    except (ValueError, IndexError):
                        node.get_logger().warn("Usage: f2 <speed>  (e.g., f2 0.5)")
                
                elif cmd:
                    node.get_logger().warn(f"Unknown command: {cmd}")
            
            except EOFError:
                break
            except KeyboardInterrupt:
                break
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
