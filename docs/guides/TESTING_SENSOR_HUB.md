# Testing robot_sensor_hub Integration

## Prerequisites

1. **ESP32 robot_sensor_hub** firmware flashed and running
2. **micro-ros-agent** container running on Main Pi
3. **robot_sensor_hub_msg** package built

## Build Custom Messages

### On Main Pi (or in Docker)

```bash
cd /path/to/rob_box_project

# Build messages
colcon build --packages-select robot_sensor_hub_msg

# Source workspace
source install/setup.bash

# Verify messages are available
ros2 interface list | grep robot_sensor_hub

# Expected output:
# robot_sensor_hub_msg/msg/DeviceCommand
# robot_sensor_hub_msg/msg/DeviceData
# robot_sensor_hub_msg/msg/DeviceSnapshot
```

### In Docker (micro-ros-agent)

Messages are automatically built in the Docker image:

```bash
# Rebuild image
cd docker/main
docker compose build micro-ros-agent

# Run container
docker compose up -d micro-ros-agent

# Verify inside container
docker exec -it rob_box-micro-ros-agent-1 bash
ros2 interface list | grep robot_sensor_hub
```

## Start micro-ROS Agent

### Option 1: Docker Compose (Main Pi)

```bash
cd docker/main
docker compose up -d micro-ros-agent

# Check logs
docker logs -f rob_box-micro-ros-agent-1
```

**Expected output:**
```
✅ Device: /dev/ttyUSB0 (USB serial)
✅ Permissions: OK
🚀 Starting micro-ROS agent on /dev/ttyUSB0 at 115200 baud...
[INFO] [uros_agent]: Agent running...
[INFO] [uros_agent]: Client connected
```

### Option 2: Manual (for testing)

```bash
# Serial transport
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# UDP transport (if ESP32 uses WiFi)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Verify Topics

```bash
# List topics
ros2 topic list

# Should see:
# /device/snapshot
# /device/command
# /parameter_events
# /rosout

# Check message types
ros2 topic info /device/snapshot
# Type: robot_sensor_hub_msg/msg/DeviceSnapshot
# Publisher count: 1

ros2 topic info /device/command
# Type: robot_sensor_hub_msg/msg/DeviceCommand
# Subscriber count: 1
```

## Test Subscription

### Echo raw data
```bash
ros2 topic echo /device/snapshot
```

**Example output:**
```yaml
timestamp: 123456789000
devices:
- device_type: 0   # AHT30
  device_id: 0
  data_type: 1     # TEMPERATURE
  value: 25.3
  error_code: 0
- device_type: 0
  device_id: 0
  data_type: 2     # HUMIDITY
  value: 45.7
  error_code: 0
- device_type: 1   # HX711
  device_id: 0
  data_type: 3     # WEIGHT
  value: 123.45
  error_code: 0
- device_type: 2   # FAN
  device_id: 0
  data_type: 4     # SPEED
  value: 0.75
  error_code: 0
- device_type: 2
  device_id: 0
  data_type: 5     # RPM
  value: 1850.0
  error_code: 0
```

### Test with Python script

```bash
# Run interactive test
python3 scripts/test_sensor_hub.py
```

**Expected output:**
```
🚀 sensor_hub_test node started
Waiting for /device/snapshot messages...

============================================================
📊 DEVICE SNAPSHOT (freq: 1.00 Hz)
Timestamp: 123456789000 ns
Device count: 21

🌡️  TEMPERATURES:
  AHT30[0]:  25.30°C ✅
  AHT30[1]:  24.85°C ✅
  AHT30[2]:  26.12°C ✅
  ...

💧 HUMIDITY:
  AHT30[0]:  45.70% ✅
  AHT30[1]:  47.20% ✅
  ...

⚖️  WEIGHT:
  HX711[0]:   123.45g ✅

🌀 FANS:
  Fan[0]:  75.0% |  1850 RPM
  Fan[1]:  50.0% |  1200 RPM
============================================================

Commands:
  f1 <speed>  - Set fan 1 speed (0.0-1.0)
  f2 <speed>  - Set fan 2 speed (0.0-1.0)
  tare        - Zero the load cell
  quit        - Exit

>
```

### Interactive commands

```bash
# Set fan 1 to 100%
> f1 1.0
🎛️  Set Fan[0] speed to 100%

# Set fan 2 to 50%
> f2 0.5
🎛️  Set Fan[1] speed to 50%

# Tare the scale
> tare
⚖️  Tare command sent to HX711

# Exit
> quit
```

## Test Publishing Commands

### Set fan speed
```bash
ros2 topic pub --once /device/command robot_sensor_hub_msg/msg/DeviceCommand \
  "{device_type: 2, device_id: 0, command_code: 0, param_1: 0.75, param_2: 0.0}"
```

### Tare load cell
```bash
ros2 topic pub --once /device/command robot_sensor_hub_msg/msg/DeviceCommand \
  "{device_type: 1, device_id: 0, command_code: 1, param_1: 0.0, param_2: 0.0}"
```

## Check Message Frequency

```bash
ros2 topic hz /device/snapshot

# Expected: ~1.0 Hz (ESP32 publishes every 1 second)
# average rate: 1.000
# min: 0.995s max: 1.005s std dev: 0.003s window: 10
```

## Troubleshooting

### No topics visible

**Check agent connection:**
```bash
docker logs rob_box-micro-ros-agent-1 | grep "Client connected"
```

If no client connected:
- Check ESP32 serial monitor for errors
- Verify serial port (`ls -l /dev/ttyUSB*`)
- Check baud rate matches (115200)
- Try manual agent: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200`

### Topics exist but no data

**Check ESP32 output:**
```bash
# ESP32 serial monitor should show:
I (XXX) sensor_hub: Published snapshot with 13 devices
```

If not publishing:
- ESP32 may not be connected to agent
- Check WiFi connection (if using UDP transport)
- Verify sensor initialization (AHT30, HX711)

### Import error for robot_sensor_hub_msg

**Rebuild and source:**
```bash
colcon build --packages-select robot_sensor_hub_msg
source install/setup.bash

# Verify
python3 -c "from robot_sensor_hub_msg.msg import DeviceSnapshot; print('OK')"
```

### Permission denied on /dev/ttyUSB0

**Add user to dialout group:**
```bash
sudo usermod -aG dialout $USER
# Logout and login again

# Or in Docker: use privileged: true
```

## Integration Testing

### Test with Nav2 (emergency stop on overheat)

```python
import rclpy
from rclpy.node import Node
from robot_sensor_hub_msg.msg import DeviceSnapshot
from geometry_msgs.msg import Twist

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        self.snapshot_sub = self.create_subscription(
            DeviceSnapshot, '/device/snapshot', self.check_temp, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def check_temp(self, msg):
        # Get all temperatures
        temps = [d.value for d in msg.devices 
                 if d.device_type == 0 and d.data_type == 1]
        
        if temps and max(temps) > 85.0:  # > 85°C
            self.get_logger().error(f"OVERHEAT! Max temp: {max(temps):.1f}°C")
            # Stop robot
            self.cmd_vel_pub.publish(Twist())
```

### Test adaptive fan control

```python
def check_temp(self, msg):
    temps = [d.value for d in msg.devices if d.data_type == 1]
    if temps:
        max_temp = max(temps)
        # Linear mapping: 40°C = 30%, 80°C = 100%
        fan_speed = max(0.3, min(1.0, (max_temp - 40.0) / 40.0))
        
        # Set both fans
        for fan_id in [0, 1]:
            cmd = DeviceCommand()
            cmd.device_type = 2
            cmd.device_id = fan_id
            cmd.command_code = 0
            cmd.param_1 = fan_speed
            self.command_pub.publish(cmd)
```

## Performance Metrics

Expected performance:
- **Publish rate**: 1 Hz (DeviceSnapshot)
- **Latency**: < 50 ms (serial) or < 100 ms (WiFi UDP)
- **Message size**: ~200-400 bytes per snapshot (depends on active sensors)
- **CPU usage** (ESP32): < 10%
- **RAM usage** (ESP32): < 50 KB

Monitor with:
```bash
# Topic bandwidth
ros2 topic bw /device/snapshot

# Latency (if header.stamp is set)
ros2 topic delay /device/snapshot

# Message rate
ros2 topic hz /device/snapshot
```

## CI/CD Integration

The `build-micro-ros-agent` job in `.github/workflows/build-main-services.yml` will:
1. Build robot_sensor_hub_msg package
2. Build micro-ros-agent Docker image
3. Push to GHCR

Verify after push:
```bash
# Pull latest image
docker pull ghcr.io/krikz/rob_box:micro-ros-agent-humble-latest

# Check if messages are available
docker run --rm ghcr.io/krikz/rob_box:micro-ros-agent-humble-latest \
  ros2 interface list | grep robot_sensor_hub
```
