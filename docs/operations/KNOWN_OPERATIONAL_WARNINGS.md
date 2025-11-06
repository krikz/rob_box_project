# Known Operational Warnings

**Purpose:** Document expected operational warnings and errors that are NOT deployment failures.

**Last Updated:** November 6, 2025

---

## 📋 Overview

This document lists known warnings and errors that appear in container logs during normal operation. These are filtered out by the deployment workflow (`L-Deploy and Verify.yml`) to prevent false positive issue creation.

---

## 🔍 Vision Pi Known Issues

### 1. micro-ros-agent: Serial Port Not Found

**Pattern:** `Serial port /dev/ttyUSB0 still not available`

**Container:** `micro-ros-agent`

**Severity:** INFO (not an error)

**Reason:** ESP32 sensor hub is optional hardware. The micro-ros-agent waits for it but continues if not present.

**Impact:** None. Robot operates normally without ESP32.

**Solution:** If you have an ESP32 sensor hub, ensure:
- USB cable is connected to Vision Pi
- Device appears in `lsusb` output
- Device is mapped in `docker-compose.yaml`: `devices: ["/dev/ttyUSB0:/dev/ttyUSB0"]`

---

### 2. promtail: Loki Connection Errors

**Pattern:** `error sending batch.*Loki.*connection`

**Container:** `promtail`

**Severity:** WARNING (non-critical)

**Reason:** Monitoring system (Loki) is disabled by default to save resources.

**Impact:** None on robot operation. Logs won't be centralized.

**Solution:** If you want monitoring:
```bash
cd ~/rob_box_project/docker/vision
./scripts/enable_monitoring.sh
```

---

### 3. Zenoh: Scouting Delay Elapsed

**Pattern:** `Scouting delay elapsed before start conditions are met`

**Container:** All ROS2 containers

**Severity:** WARNING

**Reason:** Zenoh DDS discovery timing during startup. Nodes start before router is fully initialized.

**Impact:** None. Connections establish after brief delay.

**Solution:** No action needed. This is normal startup behavior.

---

### 4. perception: Missing Nodes During Startup

**Pattern:** `Нода не найдена: /oak_d_node` (Node not found: /oak_d_node)

**Container:** `perception` (context_aggregator node)

**Severity:** WARNING

**Reason:** Aggregator starts before camera node, briefly reports missing nodes.

**Impact:** None. Resolves after all nodes start.

**Solution:** No action needed. Startup sequence timing.

---

### 5. LED Matrix: Unknown Logical Groups

**Pattern:** `Unknown logical group: 'wheel_front_right'`

**Container:** `perception` (led_matrix_compositor)

**Severity:** WARNING

**Reason:** LED groups defined for optional hardware (wheel LEDs) that may not be present.

**Impact:** None. Only affects LED animations on wheels if hardware is missing.

**Solution:** No action needed unless you want wheel LEDs (hardware not installed).

---

## 🔧 Main Pi Known Issues

### 6. nav2: Robot Out of Bounds

**Pattern:** `Robot is out of bounds of the costmap`

**Container:** `nav2`

**Severity:** WARNING (transient)

**Reason:** During initialization, robot hasn't received odometry yet. Costmap initializes before TF tree is complete.

**Impact:** None. Resolves after odometry starts publishing.

**Solution:** No action needed. Initialization timing issue.

---

### 7. rtabmap: TF Transform Errors

**Pattern:** `Cannot transform tag pose from "camera_rgb_camera_optical_frame" to "base_link"`

**Container:** `rtabmap`

**Severity:** ERROR (transient)

**Reason:** Camera node publishes AprilTag detections before robot_state_publisher establishes full TF tree.

**Impact:** None. AprilTags work after TF tree is complete (~5 seconds).

**Solution:** No action needed. Startup sequence timing.

---

### 8. ros2-control: CAN ERROR-ACTIVE State

**Pattern:** `CAN controller state: ERROR-ACTIVE`

**Container:** `ros2-control`

**Severity:** INFO (not an error)

**Reason:** Normal CAN bus state for VESC motor controllers. "ERROR-ACTIVE" means controller is operational and monitoring for errors.

**Impact:** None. Motors work normally.

**Solution:** No action needed. This is the expected CAN state.

**Reference:** CAN bus has three states:
- ERROR-ACTIVE (normal, operational)
- ERROR-PASSIVE (too many errors, limited transmission)
- BUS-OFF (offline due to critical errors)

---

### 9. kdl_parser: Root Link Inertia Warning

**Pattern:** `The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia`

**Container:** `robot-state-publisher`, `nav2`

**Severity:** WARNING

**Reason:** Known limitation in KDL (Kinematics and Dynamics Library) used by ROS2.

**Impact:** None. Only affects advanced dynamics calculations (not used).

**Solution:** No action needed. Accepted ROS2 limitation.

**Reference:** https://github.com/ros/kdl_parser/issues/27

---

### 10. controller_manager: No Real-Time Kernel

**Pattern:** `No real-time kernel detected`

**Container:** `ros2-control`

**Severity:** WARNING

**Reason:** Raspberry Pi OS doesn't use real-time kernel by default.

**Impact:** Slightly higher control loop jitter (~1ms). Acceptable for indoor robot.

**Solution:** No action needed. Installing RT kernel is complex and benefits are minimal for this application.

---

## 📊 Impact Summary

| Issue | Severity | Robot Impact | Action Needed |
|-------|----------|--------------|---------------|
| micro-ros-agent serial | INFO | None | None |
| promtail Loki | WARNING | None | Optional: enable monitoring |
| Zenoh scouting | WARNING | None | None |
| perception missing nodes | WARNING | None | None |
| LED unknown groups | WARNING | None | Optional: add LED hardware |
| nav2 out of bounds | WARNING | None | None |
| rtabmap TF errors | ERROR | None | None |
| CAN ERROR-ACTIVE | INFO | None | None |
| kdl_parser inertia | WARNING | None | None |
| No RT kernel | WARNING | None | Optional: install RT kernel |

**Summary:** All listed issues are expected operational behavior. Robot functions normally.

---

## 🚨 Real Deployment Failures

These errors **ARE** deployment failures and should trigger alerts:

### Vision Pi Critical Issues
- OAK-D camera: `No device found` → Camera disconnected
- voice-assistant: `DEEPSEEK_API_KEY not found` → Missing API key in .env.secrets
- lslidar: `Device not found` → LIDAR not connected

### Main Pi Critical Issues
- ros2-control: `Failed to initialize hardware interface` → VESC not responding
- rtabmap: `Database locked` → Multiple SLAM instances running
- nav2: `Failed to load map` → Map file missing or corrupted

---

## 🔧 Workflow Integration

The deployment workflow (`.github/workflows/L-Deploy and Verify.yml`) uses grep filters with shell variables to exclude known issues.

### Vision Pi Log Filtering

**Error exclusion (using `EXCLUDE_COMMON` variable):**
```bash
# Common patterns (used on both Pi)
EXCLUDE_COMMON="Serial port /dev/ttyUSB0|Loki.*connection|entry too far behind"
EXCLUDE_COMMON="$EXCLUDE_COMMON|could not inspect container|could not fetch logs"
EXCLUDE_COMMON="$EXCLUDE_COMMON|Scouting delay elapsed|No such container"

# Filter critical errors
CRITICAL=$(echo "$LOGS" | \
  grep -iE "(CRITICAL|FATAL|ERROR|...)" | \
  grep -vE "$EXCLUDE_COMMON" || true)
```

**Warning exclusion (using `EXCLUDE_WARN_COMMON` variable):**
```bash
# Нода не найдена = "Node not found" (Russian)
EXCLUDE_WARN_COMMON="Scouting delay elapsed|Нода не найдена|Unknown logical group"

# Filter warnings
WARNINGS=$(echo "$LOGS" | \
  grep -iE "(WARN|WARNING)" | \
  grep -vE "$EXCLUDE_WARN_COMMON" || true)
```

### Main Pi Log Filtering

**Error exclusion (using `EXCLUDE_MAIN` variable):**
```bash
# Common patterns
EXCLUDE_COMMON="Serial port /dev/ttyUSB0|Loki.*connection|..."

# Main Pi specific exclusions (includes common + additional patterns)
EXCLUDE_MAIN="$EXCLUDE_COMMON|Robot is out of bounds|Cannot transform tag pose"
EXCLUDE_MAIN="$EXCLUDE_MAIN|Sensor origin.*out of map bounds|CAN controller state: ERROR-ACTIVE"

# Filter critical errors
CRITICAL=$(echo "$LOGS" | \
  grep -iE "(CRITICAL|FATAL|ERROR|...)" | \
  grep -vE "$EXCLUDE_MAIN" || true)
```

**Warning exclusion (using `EXCLUDE_WARN_MAIN` variable):**
```bash
# Нода не найдена = "Node not found" (Russian)
EXCLUDE_WARN_MAIN="Scouting delay elapsed|could not find a connection.*tree"
EXCLUDE_WARN_MAIN="$EXCLUDE_WARN_MAIN|Нода не найдена|Unknown logical group"
EXCLUDE_WARN_MAIN="$EXCLUDE_WARN_MAIN|root link.*inertia|No real-time kernel"

# Filter warnings
WARNINGS=$(echo "$LOGS" | \
  grep -iE "(WARN|WARNING)" | \
  grep -vE "$EXCLUDE_WARN_MAIN" || true)
```

### Benefits of Variable-Based Approach
- **Maintainability:** Easy to add/remove patterns in one place
- **Readability:** Clear separation of Vision Pi vs Main Pi patterns
- **DRY principle:** Common patterns defined once and reused

---

## 📚 Related Documentation

- [AI Troubleshooting Checklist](../development/AI_TROUBLESHOOTING_CHECKLIST.md)
- [Agent Guide](../development/AGENT_GUIDE.md)
- [Deployment Workflow](../../.github/workflows/L-Deploy%20and%20Verify.yml)

---

**Maintained by:** Rob Box Development Team  
**Questions?** Create an issue with label `question` and `documentation`
