# 🚀 Quick Reference: Nav2 + ICP Odometry Tuning

**Дата:** 2025-12-16  
**Применимо к:** Nav2 с ICP-based одометрией (RTAB-Map ICP Odometry)

---

## ⚡ Quick Fixes

### Проблема: "No valid trajectories out of 650!"

**Симптом:**
```
[ERROR] [DWBLocalPlanner]: No valid trajectories out of 650!
[ERROR] [DWBLocalPlanner]: 1.00: BaseObstacle/Trajectory Hits Obstacle.
```

**Решение 1: Проверьте velocity limits**
```yaml
# nav2_params.yaml
max_vel_x: 0.15-0.20 m/s  # НЕ меньше 0.12!
max_vel_theta: 0.5-0.6 rad/s  # НЕ меньше 0.4!
```

**Решение 2: Добавьте transform_tolerance**
```yaml
# nav2_params.yaml
local_costmap:
  transform_tolerance: 0.5

global_costmap:
  transform_tolerance: 0.5
```

---

### Проблема: "Message Filter dropping message"

**Симптом:**
```
[INFO] Message Filter dropping message: frame 'lslidar_n10'
timestamp earlier than all data in transform cache
```

**Решение:**
```yaml
# nav2_params.yaml - local/global costmap
transform_tolerance: 0.5  # ОБЯЗАТЕЛЬНО для ICP!
```

**Почему:** ICP processing delay = 30-125ms

---

### Проблема: "Rejecting all added loop closures"

**Симптом:**
```
[WARN] Rejecting loop closures (390 <-> 268)
maximum graph error ratio of 3.411 (edge -22->319, abs error=0.034m)
RGBD/OptimizeMaxError is 3.000000
```

**Решение:**
```yaml
# docker-compose.yaml - rtabmap args
-p RGBD/OptimizeMaxError:5.0  # Было 3.0 (default)
```

**Формула:**
```
ICP std dev: ~10mm
Threshold: 10mm × 5.0 = 50mm
Observed: 34mm < 50mm → ✅ ACCEPTED
```

---

### Проблема: "Oscillation/Trajectory is oscillating"

**Симптом:**
```
[ERROR] 0.62: Oscillation/Trajectory is oscillating.
No valid trajectories out of 650!
```

**Решение:**
```yaml
# nav2_params.yaml - FollowPath critics
Oscillation.scale: 0.5  # Было default 1.0
```

**Почему:** ICP uncertainty ±7mm вызывает ложные осцилляции

---

### Проблема: Робот задевает препятствия

**Симптом:**
- Backup recovery behavior
- Столкновения с объектами

**Решение:**
```yaml
# nav2_params.yaml - local/global costmap
footprint: "[[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]"  # 1m×1m

inflation_layer:
  inflation_radius: 0.55  # local
  inflation_radius: 0.6   # global
```

**Логика:**
```
Real robot: 600×600mm
+ Safety:   200mm per side
= Total:    1000×1000mm
```

---

## 📋 Recommended Parameters

### Nav2 Controller
```yaml
controller_server:
  FollowPath:
    max_vel_x: 0.15-0.20
    max_vel_theta: 0.5-0.6
    transform_tolerance: 0.5
    Oscillation.scale: 0.5

velocity_smoother:
  max_velocity: [0.2, 0.0, 0.6]
  max_accel: [0.2, 0.0, 0.6]
```

### Costmaps
```yaml
local_costmap:
  transform_tolerance: 0.5
  footprint: "[[-0.5,-0.5], [0.5,-0.5], [0.5,0.5], [-0.5,0.5]]"
  inflation_radius: 0.55

global_costmap:
  transform_tolerance: 0.5
  footprint: "[[-0.5,-0.5], [0.5,-0.5], [0.5,0.5], [-0.5,0.5]]"
  inflation_radius: 0.6
```

### RTAB-Map
```yaml
args:
  -p Odom/ResetCountdown:1
  -p RGBD/OptimizeMaxError:5.0
```

---

## 🔍 Diagnostics

### Check ICP Performance
```bash
docker logs rtabmap 2>&1 | grep "Odom:" | tail -20
```

**Good values:**
```
ratio: 0.75-0.95
std dev: 0.006-0.008m
delay: 0.030-0.125s
```

### Check Loop Closures
```bash
docker logs rtabmap 2>&1 | grep -i reject | tail -10
```

**Expected:** No rejections

### Check Costmap Data
```bash
docker logs nav2 2>&1 | grep "Message Filter" | tail -10
```

**Expected:** No dropping messages

### Check Oscillations
```bash
docker logs nav2 2>&1 | grep Oscillation | tail -20
```

**Expected:** <30% oscillation rejections

---

## ⚙️ Parameter Ranges

| Parameter | Min | Recommended | Max | Notes |
|-----------|-----|-------------|-----|-------|
| `max_vel_x` | 0.10 | 0.15-0.20 | 0.30 | Indoor |
| `max_vel_theta` | 0.40 | 0.50-0.60 | 0.80 | ~29-46°/s |
| `transform_tolerance` | 0.30 | 0.50 | 1.00 | For ICP |
| `RGBD/OptimizeMaxError` | 3.0 | 4.0-5.0 | 7.0 | For ICP |
| `Oscillation.scale` | 0.3 | 0.5 | 1.0 | Lower = relaxed |
| `footprint` | 0.6m | 1.0m | 1.2m | Real + margin |
| `inflation_radius` | 0.5m | 0.55-0.6m | 0.8m | Safety |

---

## 🚨 Common Mistakes

### ❌ DON'T: Ultra-low velocities
```yaml
max_vel_x: 0.12  # TOO LOW!
max_vel_theta: 0.4  # TOO LOW!
```
**Result:** No valid trajectories

### ❌ DON'T: Forget transform_tolerance
```yaml
local_costmap:
  # transform_tolerance: missing!  ← WRONG
```
**Result:** Dropping lidar data

### ❌ DON'T: Use exact robot size as footprint
```yaml
footprint: "[[-0.3,-0.3], [0.3,-0.3], ...]"  # 600mm - TOO TIGHT!
```
**Result:** Constant collisions

### ❌ DON'T: Keep default OptimizeMaxError with ICP
```yaml
# -p RGBD/OptimizeMaxError:3.0  ← Too strict for ICP!
```
**Result:** Rejected loop closures → map drift

---

## 📚 Full Documentation

См. [NAV2_NAVIGATION_TUNING_2025-12-16.md](NAV2_NAVIGATION_TUNING_2025-12-16.md)

---

**Обновлено:** 2025-12-16
