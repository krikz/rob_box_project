# LSLIDAR N10 Setup Guide

## Hardware Connection

### LSLIDAR N10 Specifications
- **Model**: N10 Navigation & Obstacle Avoidance LiDAR
- **Type**: 2D 360° Single-Line LiDAR
- **Range**: 0.02m - 12m @ 70% reflectivity
- **Accuracy**: ±3cm (0-6m), ±4.5cm (6-12m)
- **FOV**: 360°
- **Resolution**: 0.48° - 0.96°
- **Data Rate**: 4500 points/sec
- **Dimensions**: φ52 × 36.1mm
- **Weight**: 60g
- **IP Rating**: IPX-4

### Network Configuration

**Default LiDAR Settings:**
- IP Address: `192.168.1.200`
- UDP Port: `2368`
- Subnet Mask: `255.255.255.0`

**Raspberry Pi Vision Setup:**

1. **Configure static IP for ethernet:**
```bash
# Edit netplan configuration
sudo nano /etc/netplan/01-netcfg.yaml
```

Add ethernet interface:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.1.100/24  # Pi IP in LiDAR subnet
      nameservers:
        addresses: [8.8.8.8]
  wifis:
    wlan0:
      dhcp4: no
      addresses:
        - 10.1.1.21/24  # Main network
      routes:
        - to: default
          via: 10.1.1.1
      nameservers:
        addresses: [10.1.1.1, 8.8.8.8]
      access-points:
        "WiFi_Network":
          password: "your_password"
```

Apply configuration:
```bash
sudo netplan apply
```

2. **Test LiDAR connectivity:**
```bash
# Ping LiDAR
ping 192.168.1.200

# Listen for UDP packets
sudo tcpdump -i eth0 port 2368 -c 10
```

### Physical Mounting

Mount LSLIDAR on robot top center:
- **Position**: Front of robot, 0.25m from base_link
- **Height**: 0.2m above base_link
- **Orientation**: Aligned with robot forward (X-axis)

URDF frame hierarchy:
```
base_link
  └─ lslidar_n10 (position: x=0.25m, y=0.0m, z=0.2m)
```

## Software Setup

### Docker Container

Container already configured in `docker/vision/docker-compose.yaml`:

```yaml
lslidar:
  build:
    context: .
    dockerfile: lslidar/Dockerfile
  container_name: lslidar
  network_mode: host  # Required for UDP multicast
  privileged: true
```

### Build and Start

**On Vision Pi:**
```bash
cd ~/rob_box_project/docker/vision

# Build LiDAR container
docker-compose build lslidar

# Start LiDAR (stops if already running)
docker-compose up -d lslidar

# Check logs
docker logs lslidar -f
```

Expected output:
```
Zenoh router is ready!
Starting LSLIDAR N10 driver...
[INFO] LSLIDAR N10 connected
[INFO] Publishing scan on /scan topic
[INFO] Scan rate: ~10 Hz
```

### Verify Data Stream

**Check ROS 2 topics:**
```bash
# List topics (should see /scan)
ros2 topic list | grep scan

# Monitor scan data
ros2 topic hz /scan

# View scan messages
ros2 topic echo /scan --once
```

Expected output:
```
average rate: 10.000
  min: 0.100s max: 0.100s std dev: 0.00100s window: 10

ranges: [12.0, 11.95, ..., 0.35, 0.02]  # 4500 points
angle_min: -3.14159
angle_max: 3.14159
```

## RTAB-Map Integration

RTAB-Map automatically receives `/scan` topic for:
- **2D Grid Map**: Obstacle detection for navigation
- **Loop Closure**: Scan matching for drift correction
- **Localization**: ICP alignment with visual odometry

Configuration in `docker/main/config/rtabmap_config.yaml`:
```yaml
subscribe_scan: true
scan_topic: /scan

# Scan parameters
Mem/LaserScanNormalK: "0"  # Disabled (2D LiDAR)
Grid/FromDepth: "true"     # Combine with depth camera
Grid/CellSize: "0.05"      # 5cm grid resolution
Grid/RangeMax: "5.0"       # Use LiDAR up to 5m
```

## Troubleshooting

### LiDAR Not Detected

**Check network:**
```bash
# Verify eth0 has IP 192.168.1.100
ip addr show eth0

# Ping LiDAR
ping 192.168.1.200
```

**Check UDP packets:**
```bash
# Install tcpdump if needed
sudo apt-get install tcpdump

# Listen for LiDAR data
sudo tcpdump -i eth0 port 2368 -n
```

### No /scan Topic

**Check container:**
```bash
# Container status
docker ps -a | grep lslidar

# Full logs
docker logs lslidar

# Restart container
docker restart lslidar
```

**Check Zenoh connection:**
```bash
# Verify Zenoh router running
docker logs zenoh-router-vision | grep "listening"

# Check ROS 2 discovery
ros2 node list | grep lslidar
```

### Low Scan Rate

**Check system load:**
```bash
# CPU usage
htop

# Memory usage (should be <80%)
free -h
```

**Reduce other processes:**
```bash
# Temporarily stop camera
docker stop oak-d

# Check scan rate improves
ros2 topic hz /scan
```

### Scan Data Quality Issues

**Check LiDAR positioning:**
- Ensure no obstacles blocking 360° view
- Mount height should be >20cm from ground
- Keep away from reflective surfaces (glass, mirrors)

**Adjust filtering in `lidar_config.yaml`:**
```yaml
min_range: 0.05  # Increase if too noisy near LiDAR
max_range: 8.0   # Decrease if long-range unreliable
```

## Performance Notes

**Resource Usage:**
- CPU: ~5-10% (lslidar_driver)
- RAM: ~200MB
- Network: ~1 Mbps UDP

**Combined System Load (Vision Pi):**
- OAK-D Camera: 4-5GB RAM
- LSLIDAR: 200MB RAM
- Total: ~5.2GB / 7.8GB (67% - good!)

**Scan Integration with RTAB-Map:**
- Visual Odometry: RGB-D camera (primary)
- Scan Matching: LiDAR (secondary, loop closure)
- Grid Map: Combined depth + laser for navigation
- Odometry Fusion: Visual + ICP for accuracy

## Related Files

- **URDF**: `src/rob_box_description/urdf/rob_box.xacro` (line 268)
- **Docker**: `docker/vision/lslidar/Dockerfile`
- **Config**: `docker/vision/config/lidar_config.yaml`
- **RTAB-Map**: `docker/main/config/rtabmap_config.yaml`
