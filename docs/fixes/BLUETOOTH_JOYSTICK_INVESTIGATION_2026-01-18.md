# Bluetooth Joystick Investigation - ExpressLRS

**Date:** 2026-01-18  
**Device:** ExpressLRS Joystick (MAC: 8C:4F:00:C2:04:96)  
**Status:** ❌ Not working on Raspberry Pi, ✅ Working on dev machine

## Summary

ExpressLRS Joystick successfully connects via Bluetooth HID on Ubuntu 22.04 (dev machine) but fails to create joystick device node on Raspberry Pi 5 (Ubuntu Plucky).

## Working Configuration (Dev Machine)

### System Info
- **OS:** Ubuntu 22.04 (Jammy)
- **Kernel:** 6.8.0-90-generic
- **Architecture:** x86_64

### Bluetooth Connection
```bash
bluetoothctl info 8C:4F:00:C2:04:96
```
**Output:**
```
Device 8C:4F:00:C2:04:96 (public)
        Name: ExpressLRS Joystick
        Alias: ExpressLRS Joystick
        Paired: yes
        Connected: yes
        UUID: Human Interface Device (00001812-0000-1000-8000-00805f9b34fb)
        Modalias: bluetooth:vE502pBBABd1001
        Battery Percentage: 0x64 (100)
```

### Kernel Recognition
```bash
sudo dmesg | grep -i express
```
**Output:**
```
[168197.261103] input: ExpressLRS Joystick as /devices/virtual/misc/uhid/0005:E502:BBAB.0014/input/input40
[168197.261202] hid-generic 0005:E502:BBAB.0014: input,hidraw2: BLUETOOTH HID v10.01 Joystick [ExpressLRS Joystick] on 28:11:a8:3a:01:d2
```

### Device Node Creation
```bash
ls -la /dev/input/js*
```
**Output:**
```
crw-rw-r--+ 1 root input 13, 0 Jan 18 14:45 /dev/input/js0
```

### Input Device Info
```bash
cat /proc/bus/input/devices | grep -A 10 "ExpressLRS"
```
**Output:**
```
N: Name="ExpressLRS Joystick"
S: Sysfs=/devices/virtual/misc/uhid/0005:E502:BBAB.0014/input/input40
U: Uniq=8c:4f:00:c2:04:96
H: Handlers=event18 js0 
B: PROP=0
B: EV=1b
B: KEY=ffff00000000 0 0 0 0
B: ABS=3007f
B: MSC=10
```

**✅ Key Point:** `Handlers=event18 js0` - joydev module automatically creates `/dev/input/js0`

### HID Driver Info
```bash
cat /sys/class/input/input40/device/uevent
```
**Output:**
```
DRIVER=hid-generic
HID_ID=0005:0000E502:0000BBAB
HID_NAME=ExpressLRS Joystick
HID_PHYS=28:11:a8:3a:01:d2
HID_UNIQ=8c:4f:00:c2:04:96
MODALIAS=hid:b0005g0001v0000E502p0000BBAB
```

### Loaded Modules
```bash
lsmod | grep joydev
```
**Output:**
```
joydev                 32768  0
```

## Failed Configuration (Raspberry Pi Robot)

### System Info
- **OS:** Ubuntu Plucky (25.04)
- **Kernel:** 6.14.0-1016-raspi
- **Architecture:** arm64 (Raspberry Pi 5)
- **IP:** 10.1.1.20

### Bluetooth Connection Attempts
```bash
bluetoothctl connect 8C:4F:00:C2:04:96
```
**Result:** ✅ "Connection successful" reported by bluetoothctl

```bash
bluetoothctl info 8C:4F:00:C2:04:96
```
**Output (when connected):**
```
Device 8C:4F:00:C2:04:96 (public)
        Name: ExpressLRS Joystick
        Connected: yes
        UUID: Human Interface Device (00001812-0000-1000-8000-00805f9b34fb)
```

### Device Node Creation
```bash
ls -la /dev/input/js*
```
**Result:** ❌ No such file or directory

```bash
ls -la /dev/input/
```
**Output:** Only event0-4, mice - NO js* devices

### Input Device Recognition
```bash
cat /proc/bus/input/devices
```
**Result:** ❌ ExpressLRS Joystick NOT listed in /proc/bus/input/devices

```bash
cat /proc/bus/input/devices | grep -i joystick
```
**Result:** No output (device not registered by kernel input subsystem)

### Loaded Modules
```bash
lsmod | grep joydev
```
**Output:**
```
joydev                 32768  0
```
**✅ Module is loaded** but not binding to Bluetooth HID devices

### Installed Packages
```bash
dpkg -l | grep joy
```
**Output:**
```
ii  joystick           1:1.8.1-2build1
ii  evtest             1:1.35-1
ii  inputattach        1:1.8.1-2build1
```

## Problem Analysis

### Root Cause
The `joydev` kernel module is loaded on both systems, but on Raspberry Pi (kernel 6.14.0-raspi) it **does not automatically bind to Bluetooth HID joystick devices**, while on x86_64 (kernel 6.8.0-generic) it does.

### Possible Reasons
1. **Kernel version difference:** 6.8.0 vs 6.14.0 - possible regression or configuration change in newer kernel
2. **Architecture difference:** x86_64 vs arm64 (raspi) - different HID subsystem behavior
3. **Ubuntu version difference:** Jammy (22.04) vs Plucky (25.04) - systemd/udev rules changes
4. **HID device descriptor:** ExpressLRS may use non-standard HID descriptor that worked in 6.8 but broken in 6.14

### Why Bluetooth Reports "Connected" but No Device Node
- Bluetooth stack (BlueZ) successfully pairs and connects at HCI/L2CAP level
- HID profile (UUID 00001812) is established
- uhid kernel driver creates virtual HID device
- **BUT:** joydev module doesn't recognize the device capabilities and doesn't create js0

### Comparison with Standard Joysticks
Most commercial game controllers (Xbox, PS4/PS5) use well-tested HID descriptors that work across kernel versions. ExpressLRS is a DIY/hobbyist transmitter converted to Bluetooth joystick, possibly with non-standard HID report descriptor.

## Attempted Solutions

### ❌ Solution 1: Install joystick utilities
```bash
sudo apt-get install joystick jstest-gtk evtest
```
**Result:** Packages installed, but no effect on device recognition

### ❌ Solution 2: Use joy_linux_node instead of joy_node
**Finding:** `ros-kilted-joy-linux` package exists but `joy_linux_node` executable doesn't exist
```bash
docker exec teleop ls /opt/ros/kilted/lib/joy/
# Output: game_controller_node, joy_enumerate_devices, joy_node (NO joy_linux_node)
```

### ❌ Solution 3: Manually load/reload joydev
```bash
sudo modprobe -r joydev && sudo modprobe joydev
```
**Result:** No change, device still not recognized

## Alternative Approaches to Investigate

### 1. Use `/dev/input/eventX` directly
If Bluetooth creates event device (even without js0), could use evdev library:
- Python library: `python-evdev`
- ROS 2 could read raw event device
- **Issue:** Need to identify which event# corresponds to joystick

### 2. Custom udev Rule
Create rule to force joydev binding:
```bash
# /etc/udev/rules.d/99-expresslrs-joystick.rules
SUBSYSTEM=="input", ATTRS{name}=="ExpressLRS Joystick", RUN+="/sbin/modprobe joydev"
```

### 3. Kernel Parameter
Try forcing joydev for all HID devices:
```bash
# Add to /boot/firmware/cmdline.txt
joydev.always_attach=1
```

### 4. Downgrade Kernel
Revert Raspberry Pi to older kernel (6.8.x or 6.10.x) where joydev binding worked correctly.

### 5. USB Connection Instead
If ExpressLRS has USB-C port, connect via USB cable:
- Eliminates Bluetooth complexity
- More reliable for real-time control
- Lower latency

### 6. Use Different Transmitter
Test with mainstream game controller:
- Xbox Series X/S controller (Bluetooth)
- PS5 DualSense controller
- Both have well-tested Linux HID support

## ROS 2 Integration Requirements

Once `/dev/input/js0` exists, ROS 2 setup would be:

```yaml
# joystick_params.yaml
joy_node:
  ros__parameters:
    device_id: 0  # /dev/input/js0
    device_name: ""
    deadzone: 0.05
    autorepeat_rate: 20.0
```

```python
# joystick_control_node.py
- Subscribes to /joy (sensor_msgs/Joy)
- Publishes /cmd_vel_joy (geometry_msgs/Twist)
- Voice feedback via /tts/speak
- Priority 100 in twist_mux
```

## Current Project State

### Created Files
- `src/rob_box_teleop/` - ROS 2 package
- `docker/main/teleop/Dockerfile` - Container image
- `docker/main/docker-compose.yaml` - Service definition
- `docker/main/scripts/teleop/start_teleop.sh` - Startup script (currently broken)
- `.github/workflows/` - CI/CD integration

### Blocked Items
- ❌ Cannot test joystick_control_node (no /dev/input/js0)
- ❌ Cannot test joy_node (no device to read from)
- ❌ Cannot test voice feedback integration
- ❌ Cannot test twist_mux priority switching

## Recommendations

### Immediate Action
1. **Test with USB cable** if ExpressLRS supports wired mode
2. **Try Xbox/PS5 controller** to verify ROS 2 stack works with standard hardware
3. **Check kernel logs** on robot during connection: `sudo dmesg -w` while pairing

### Medium-term Solutions
1. **Create custom HID driver** for ExpressLRS if descriptor is non-standard
2. **Use evdev directly** instead of joystick API (read from /dev/input/eventX)
3. **File kernel bug report** if this is regression in 6.14.0-raspi

### Long-term Solutions
1. **Switch to wired USB connection** for joystick (most reliable)
2. **Use different transmitter** with proven Linux Bluetooth support
3. **Wait for kernel update** that fixes joydev binding on arm64 Bluetooth HID

## Related Documentation
- ROS 2 joy package: http://wiki.ros.org/joy
- Linux Input Subsystem: https://www.kernel.org/doc/Documentation/input/
- BlueZ HID Profile: http://www.bluez.org/

## Next Steps
1. User to put ExpressLRS in pairing mode and retry connection on robot
2. Monitor `dmesg` during connection to see if ANY input device is created
3. Try USB connection if available
4. Test with alternative Bluetooth game controller
