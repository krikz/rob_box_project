---
name: motor-testing
description: Motor testing, movement calibration and odometry verification for Rob Box robot. Use when testing motor response, calibrating gear_ratio or min_duty, verifying odometry accuracy, or diagnosing movement issues (speed amplification, turning problems, deadzone compensation).
---

# Motor Testing & Calibration

Procedures for testing motor response, calibrating speed parameters, and verifying odometry accuracy on Rob Box.

## Prerequisites

1. **Zenoh connectivity** established (see `zenoh-dev-setup` skill)
2. **Robot running** — all Docker containers on Main Pi (10.1.1.10) are up
3. **Teleop disarmed** — joystick not publishing to avoid twist_mux conflicts
4. **Clear space** — at least 3m straight line in front of robot
5. **Measuring tape** ready for physical distance verification

### Environment Setup (dev machine)

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Zenoh env (adjust paths)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI=$(pwd)/local_test/zenoh_local_session.json5
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Verify topics
ros2 topic list | grep -E "odom|cmd_vel"
# Must see: /odom, /cmd_vel_web (among others)
```

## Quick Start — Movement Test

### 1. Mark robot start position on floor

### 2. Run move_test.py

```bash
# Usage: python3 local_test/move_test.py [speed_m/s] [duration_s]

# 10 cm test (default): 0.1 m/s × 1s
python3 local_test/move_test.py

# 50 cm test: 0.1 m/s × 5s
python3 local_test/move_test.py 0.1 5.0

# 1 meter test: 0.2 m/s × 5s
python3 local_test/move_test.py 0.2 5.0

# 2 meter test: 0.2 m/s × 10s
python3 local_test/move_test.py 0.2 10.0
```

### 3. Read output

The script reports:
- **START/END** — odom x,y positions
- **ODOM DISTANCE** — calculated from odom
- **EXPECTED** — speed × duration
- **RATIO** — actual/expected (target: 1.0 ± 0.05)

### 4. Measure with ruler

Compare physical distance (ruler) with ODOM DISTANCE and EXPECTED.

## Key Parameters

All in `src/rob_box_description/urdf/rob_box_ros2_control.xacro`:

| Parameter | Current Value | Effect |
|-----------|--------------|--------|
| `gear_ratio` | 2.46 | Motor-to-wheel speed reduction. Higher = slower wheel speed |
| `min_duty` | 0.04 | Deadzone compensation. Minimum PWM sent to VESC when speed > 0 |
| `wheel_radius` | 0.1143 m | Physical wheel radius |
| `max_rps` | 6.5 | Maximum motor revolutions per second |
| `poles` | 30 | VESC motor pole count |

### How max_speed is computed (in vesc_handler.cpp)

```
max_speed = 2π × (max_rps / gear_ratio) × wheel_radius
```

With current values: `2π × (6.5 / 2.46) × 0.1143 = 1.897 m/s`

## Calibration Procedures

### Calibrating gear_ratio

**When:** Odometry consistently over- or undershoots physical distance at moderate speeds (0.1-0.2 m/s) with min_duty=0.0.

**Formula:**
```
gear_ratio_new = gear_ratio_old / ratio
```
where `ratio = physical_distance / expected_distance`

**Procedure:**
1. Set `min_duty` to `0.0` temporarily (eliminates deadzone distortion)
2. Run 1m test: `python3 local_test/move_test.py 0.2 5.0`
3. Measure physical distance with ruler
4. Calculate ratio = ruler_distance / expected_distance
5. New gear_ratio = current_gear_ratio / ratio
6. Update `gear_ratio` in xacro, rebuild and restart `robot-state-publisher` container on robot
7. Repeat until ratio is within 0.95-1.05

**Historical calibration data (gear_ratio iterations):**

| gear_ratio | min_duty | Test | Odom | Ruler | Ratio | Notes |
|-----------|---------|------|------|-------|-------|-------|
| 1.0 (implicit) | 0.04 | 10cm | 30cm | ~30cm | 3.0 | Way too fast, min_duty amplifies |
| 1.0 (implicit) | 0.0 | 50cm | 18.4cm | — | 0.37 | Too slow without deadzone comp |
| 5.0 | 0.0 | 50cm | 108cm | — | 2.17 | Overshoot |
| 2.3 | 0.0 | 50cm | 47cm | — | 0.94 | Close |
| 2.16 | 0.0 | 100cm | 87cm | — | 0.87 | Still under |
| **2.46** | **0.0** | **100cm** | **98.3cm** | **99cm** | **0.983** | **✅ Calibrated** |
| **2.46** | **0.0** | **200cm** | **206cm** | **206cm** | **1.03** | **✅ Confirmed** |
| 2.46 | 0.04 | 10cm | 17.9cm | — | 1.79 | min_duty distorts short/slow |

### Calibrating min_duty

**When:** Robot fails to turn or move at low duty cycles (nav2 sends very small angular velocities).

**Tradeoff:** `min_duty > 0` helps motors overcome friction/deadzone but amplifies low-speed movements. Effect is proportionally larger at shorter distances / lower speeds.

**Current decision:** `min_duty=0.04` — acceptable for nav2 turning, distorts short low-speed tests.

**Testing procedure:**
1. Set desired `min_duty` in xacro
2. Rebuild and restart
3. Test turning: set a nav2 goal that requires 90°+ rotation
4. Test straight line: `python3 local_test/move_test.py 0.2 5.0` (1m)
5. If robot can't turn → increase min_duty
6. If straight line overshoots → decrease min_duty

## twist_mux Priority

Commands go through twist_mux. Only one source controls at a time:

| Source | Topic | Priority | Timeout |
|--------|-------|----------|---------|
| Emergency | — | 255 | — |
| Joystick | /cmd_vel_joy | 100 | 0.5s |
| Web | /cmd_vel_web | 50 | 1.0s |
| Voice | /cmd_vel_voice | 25 | — |
| Nav2 | /cmd_vel | 10 | — |

**move_test.py publishes to `/cmd_vel_web` (priority 50).** Ensure joystick is disarmed (not publishing), otherwise it blocks at priority 100.

## Troubleshooting

### Robot doesn't move
1. Check teleop is disarmed: `ros2 topic hz /cmd_vel_joy` — should show no messages
2. Check twist_mux output: `ros2 topic echo /cmd_vel_out --once`
3. Check VESC containers: `sshpass -p 'open' ssh ros2@10.1.1.10 'docker ps | grep control'`

### Robot moves but distance is wrong
1. If ratio > 1.2 → gear_ratio too low or min_duty too high
2. If ratio < 0.8 → gear_ratio too high
3. Always calibrate gear_ratio with min_duty=0.0 first, then add min_duty back

### Robot can't turn (4WD skid-steer)
- Skid-steer needs extra torque for turning due to wheel scrub
- Symptoms: nav2 sends angular velocity but robot barely rotates, duty values < 0.03 in logs
- Fix: increase `min_duty` (current: 0.04) or increase nav2's angular velocity limits

### Odom drifts during straight-line test
- Check wheel_separation in `docker/main/config/controllers/controller_manager.yaml`
- Uneven tire pressure or surface friction causes drift
- Small drift (<5°) is normal for skid-steer

## Related Files

| File | Purpose |
|------|---------|
| `local_test/move_test.py` | Movement test script |
| `src/rob_box_description/urdf/rob_box_ros2_control.xacro` | gear_ratio, min_duty, wheel params |
| `src/vesc_nexus/vesc_ackermann/src/vesc_handler.cpp` | Speed calculation, deadzone compensation |
| `docker/main/config/controllers/controller_manager.yaml` | diff_drive_controller params |
| `docker/main/config/twist_mux/twist_mux.yaml` | Input priorities and timeouts |
