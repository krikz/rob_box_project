# robot_sensor_hub_msg

Custom ROS2 messages for **robot_sensor_hub** ESP32 sensor node.

## Messages

### DeviceData.msg
Individual device reading (sensor value or actuator state).

```
uint8 device_type     # Device type: 0=AHT30, 1=HX711, 2=FAN
uint8 device_id       # Device ID (AHT30 channel, fan number, etc.)
uint8 data_type       # Data type: 1=temp, 2=humidity, 3=weight, 4=speed, 5=rpm
float32 value         # Measured value
uint8 error_code      # Error code (0=OK, >0=error)
```

**Device Types:**
- `0` - AHT30 (Temperature/Humidity sensor)
- `1` - HX711 (Load cell weight sensor)
- `2` - FAN (PWM fan with tachometer)

**Data Types:**
- `1` - TEMPERATURE (°C)
- `2` - HUMIDITY (%)
- `3` - WEIGHT (grams/kg)
- `4` - SPEED (0.0-1.0 PWM duty)
- `5` - RPM (revolutions per minute)

### DeviceCommand.msg
Command for device control.

```
uint8 device_type      # Target device type
uint8 device_id        # Target device ID
uint8 command_code     # Command: 0=SET_SPEED, 1=TARE_SCALE
float32 param_1        # Command parameter 1
float32 param_2        # Command parameter 2
```

**Command Codes:**
- `0` - SET_SPEED (for fans, param_1 = 0.0-1.0)
- `1` - TARE_SCALE (for load cell, zero/tare)

### DeviceSnapshot.msg
Aggregated snapshot of all device readings.

```
int64 timestamp           # Timestamp in nanoseconds
DeviceData[] devices      # Array of device data
```

Published at 1 Hz, contains:
- AHT30 data: temp + humidity × 8 channels (up to 16 values)
- HX711 data: weight (1 value)
- Fan data: speed + rpm × 2 fans (4 values)
- **Total**: up to 21 DeviceData in one snapshot

## Topics

### Published by ESP32

**`/device/snapshot`** (`robot_sensor_hub_msg/DeviceSnapshot`)
- Frequency: 1 Hz
- Contains all sensor readings

### Subscribed by ESP32

**`/device/command`** (`robot_sensor_hub_msg/DeviceCommand`)
- Control commands for fans and load cell

## Usage Examples

### Echo all sensor data
```bash
ros2 topic echo /device/snapshot robot_sensor_hub_msg/msg/DeviceSnapshot
```

### Set fan 1 speed to 75%
```bash
ros2 topic pub --once /device/command robot_sensor_hub_msg/msg/DeviceCommand \
  "{device_type: 2, device_id: 0, command_code: 0, param_1: 0.75, param_2: 0.0}"
```

### Tare load cell (zero)
```bash
ros2 topic pub --once /device/command robot_sensor_hub_msg/msg/DeviceCommand \
  "{device_type: 1, device_id: 0, command_code: 1, param_1: 0.0, param_2: 0.0}"
```

### Filter data in Python
```python
import rclpy
from rclpy.node import Node
from robot_sensor_hub_msg.msg import DeviceSnapshot

class SensorFilter(Node):
    def __init__(self):
        super().__init__('sensor_filter')
        self.sub = self.create_subscription(
            DeviceSnapshot,
            '/device/snapshot',
            self.snapshot_callback,
            10
        )
    
    def snapshot_callback(self, msg):
        # Filter temperatures
        temps = [d.value for d in msg.devices 
                 if d.device_type == 0 and d.data_type == 1]
        
        # Filter humidity
        hums = [d.value for d in msg.devices 
                if d.device_type == 0 and d.data_type == 2]
        
        # Weight
        weights = [d.value for d in msg.devices 
                   if d.device_type == 1 and d.data_type == 3]
        
        self.get_logger().info(
            f"Temps: {temps}, Hums: {hums}, Weight: {weights}"
        )
```

## Building

This package is automatically built in the `micro-ros-agent` Docker container.

**Manual build:**
```bash
cd /path/to/rob_box_project
colcon build --packages-select robot_sensor_hub_msg
source install/setup.bash

# Verify
ros2 interface list | grep robot_sensor_hub
```

## Hardware

See **robot_sensor_hub** firmware repository:
- https://github.com/krikz/robot_sensor_hub (branch: humble)

**Components:**
- ESP32-C3
- TCA9548A I2C multiplexer (8 channels)
- AHT30 × 8 (temp/humidity sensors)
- HX711 + load cell
- 2× Noctua PWM fans with tachometer

## Documentation

Full setup guide: `docs/guides/MICROROS_SETUP.md`
