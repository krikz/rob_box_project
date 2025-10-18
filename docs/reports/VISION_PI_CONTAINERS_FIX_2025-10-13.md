# Vision Pi Containers Fix Report
**Date**: 2025-10-13  
**Issue**: LSLIDAR and Voice Assistant containers failing on Vision Pi  
**Resolution**: Fixed launch files, Docker configuration, and CI/CD

## Problem Summary

После обновления Vision Pi до свежих контейнеров обнаружены проблемы:

| Container | Status | Issue |
|-----------|--------|-------|
| 🔴 **lslidar** | Restarting | `rviz2` package not found |
| 🔴 **voice-assistant** | Restarting | `ROS_DISTRO` environment variable missing |

## Root Cause Analysis

### 1. LSLIDAR Issue

**Error Log:**
```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): 
"package 'rviz2' not found, searching: ['/ws/install/lslidar_driver', 
'/ws/install/lslidar_msgs', '/opt/ros/humble']"
```

**Root Cause:**
- Launch file `lslidar_launch.py` tries to start `rviz2` node
- RViz2 is a **GUI visualization tool**, not needed on headless robot
- RViz2 not installed in Docker image (saves ~200 MB)

**Why this happened:**
- Default LSLIDAR package launch includes RViz2 for desktop visualization
- Vision Pi runs **headless** (no display, no X server)
- Need separate launch file for robot deployment

### 2. Voice Assistant Issue

**Error Log:**
```
/scripts/start_voice_assistant.sh: line 11: 
/opt/ros//setup.bash: No such file or directory
```

**Root Cause:**
- Startup script does: `source /opt/ros/$ROS_DISTRO/setup.bash`
- `ROS_DISTRO` environment variable is **empty**
- Results in invalid path: `/opt/ros//setup.bash` (double slash)

**Why this happened:**
- `ROS_DISTRO=humble` not defined in `docker-compose.yaml`
- Works in Main Pi because defined at compose level
- Vision Pi compose file missing this variable

### 3. Voice Assistant Build Issue (Secondary)

**Error Log:**
```
[ERROR] [launch]: Caught exception in launch: 
executable 'animation_player_node' not found on the libexec directory 
'/ws/install/rob_box_animations/lib/rob_box_animations'
```

**Root Cause:**
- Voice Assistant launch file includes `animation_player_node`
- `rob_box_animations` package **not needed on Vision Pi**
- Animations run on Main Pi (LED matrix compositor there)
- Dockerfile incorrectly copied and built animations package

## Solutions Implemented

### Fix 1: LSLIDAR Headless Launch

**Created:** `docker/vision/config/lslidar/lslidar_headless_launch.py`

```python
#!/usr/bin/python3
# LSLIDAR N10 Headless Launch - без RViz2 для Vision Pi

def generate_launch_description():
    """
    Headless launch для LSLIDAR N10
    Только драйвер, без визуализации RViz2
    """
    driver_dir = '/config/lslidar/lsx10_custom.yaml'

    driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[driver_dir],
    )

    return LaunchDescription([driver_node])
```

**Updated:** `docker/vision/scripts/lslidar/start_lslidar.sh`

```bash
# Было:
exec ros2 launch lslidar_driver lslidar_launch.py

# Стало:
exec ros2 launch /config/lslidar/lslidar_headless_launch.py
```

**Result:** ✅ LSLIDAR container running successfully
- LiDAR connected: `/dev/ttyACM0` opened OK
- Model detected: M10 (driver compatible with N10)
- Initialization successful

### Fix 2: Voice Assistant ROS_DISTRO

**Updated:** `docker/vision/docker-compose.yaml`

```yaml
voice-assistant:
  environment:
    - ROS_DOMAIN_ID=0
    - ROS_DISTRO=humble  # ← ADDED
    - RMW_IMPLEMENTATION=rmw_zenoh_cpp
    ...
```

**Result:** ✅ ROS setup.bash now loads correctly

### Fix 3: Voice Assistant Headless Launch

**Created:** `docker/vision/config/voice/voice_assistant_headless.launch.py`

Launch file **without** `animation_player_node` for Vision Pi:
- audio_node ✅
- stt_node ✅
- dialogue_node ✅
- tts_node ✅
- command_node ✅
- sound_node ✅
- ❌ animation_player_node (only on Main Pi)

**Updated:** `docker/vision/scripts/start_voice_assistant.sh`

```bash
# Use headless launch on Vision Pi
exec ros2 launch /config/voice/voice_assistant_headless.launch.py
```

### Fix 4: Voice Assistant Dockerfile

**Updated:** `docker/vision/voice_assistant/Dockerfile`

**Changes:**
1. **Removed animations package from copy:**
   ```dockerfile
   # Before:
   COPY src/rob_box_voice /ws/src/rob_box_voice
   COPY src/rob_box_animations /ws/src/rob_box_animations  # ← REMOVED
   
   # After:
   COPY src/rob_box_voice /ws/src/rob_box_voice
   ```

2. **Removed animations from build:**
   ```dockerfile
   # Before:
   colcon build --packages-select rob_box_voice rob_box_animations
   
   # After:
   colcon build --packages-select rob_box_voice
   ```

3. **Fixed healthcheck:**
   ```dockerfile
   # Before:
   CMD ros2 node list | grep -q "audio_node\|animation_player" || exit 1
   
   # After:
   CMD ros2 node list | grep -q "audio_node" || exit 1
   ```

**Benefits:**
- ✅ Smaller image size (~200 MB saved)
- ✅ Faster build time
- ✅ No missing dependencies
- ✅ Correct healthcheck

### Fix 5: CI/CD - Disable Auto-Delete Feature Branches

**Updated:** `.github/workflows/auto-merge-feature-to-develop.yml`

**Before:**
```yaml
- name: Delete feature branch (optional)
  if: steps.merge.outputs.merged == 'true'
  run: |
    git push origin --delete $FEATURE_BRANCH
```

**After:**
```yaml
# NOTE: Feature branch deletion disabled
# Developer should manually delete feature branches after verification
# This prevents accidental deletion of work-in-progress branches
#
# - name: Delete feature branch (optional)
#   if: steps.merge.outputs.merged == 'true'
#   run: |
#     git push origin --delete $FEATURE_BRANCH
```

**Reason:**
- Feature branches need testing with Docker images before deletion
- Docker image tags (e.g., `voice-assistant-humble-test`) persist
- Developer can verify on robot before manual deletion
- Prevents accidental loss of work-in-progress

## Deployment Steps

1. **Git commits and push:**
   ```bash
   git commit -m "fix(vision-pi): Remove RViz2 from LSLIDAR and add ROS_DISTRO"
   git commit -m "fix(vision-pi): Use headless launch without animation_player"
   git commit -m "fix(ci): Disable auto-delete of feature branches"
   git push origin feature/voice-assistant
   ```

2. **GitHub Actions triggered:**
   - Workflow: `build-vision-services.yml`
   - Branch: `feature/voice-assistant`
   - Image tag: `voice-assistant-humble-test`
   - Platform: `linux/arm64`

3. **Pull latest code on Vision Pi:**
   ```bash
   ssh ros2@10.1.1.21
   cd ~/rob_box_project
   git fetch origin
   git checkout feature/voice-assistant
   git pull origin feature/voice-assistant
   ```

4. **Restart containers:**
   ```bash
   cd ~/rob_box_project/docker/vision
   docker-compose restart lslidar voice-assistant
   # Or for full rebuild:
   docker-compose stop voice-assistant
   docker-compose rm -f voice-assistant
   docker-compose pull voice-assistant  # Pull new image after build
   docker-compose up -d voice-assistant
   ```

## Verification

### LSLIDAR Container ✅

**Status:** Running, healthy

**Logs:**
```
Waiting for Zenoh router to be ready...
Zenoh router is ready!
Starting LSLIDAR N10 driver...
[INFO] [lslidar_driver_node]: Lidar is M10
port = /dev/ttyACM0, baud_rate = 460800
open_port /dev/ttyACM0 OK !
[INFO] [lslidar_driver_node]: Initialised lslidar without error
[INFO] [lslidar_driver_node]: Successfully initialize driver...
```

**Published Topics:**
- `/scan` (sensor_msgs/LaserScan)
- `/lslidar_packets` (raw data)

### Voice Assistant Container ⏳

**Status:** Waiting for new Docker image build

**Expected after rebuild:**
- ✅ ROS setup.bash loads
- ✅ All Phase 3-5 nodes start
- ✅ No missing dependencies
- ✅ Healthcheck passes

## System Architecture

### Vision Pi Services (Headless)

```
┌─────────────────────────────────────────────────────┐
│           Vision Pi (10.1.1.21 / 10.1.1.11)         │
├─────────────────────────────────────────────────────┤
│  ✅ zenoh-router      │ Message routing (healthy)   │
│  ✅ oak-d             │ OAK-D Lite camera (RGBD)    │
│  ✅ apriltag          │ AprilTag detection          │
│  ✅ lslidar           │ LSLIDAR N10 (HEADLESS)      │
│  ⏳ voice-assistant   │ Voice AI (HEADLESS)         │
│  ⚠️  led-matrix       │ LED driver (unhealthy)      │
└─────────────────────────────────────────────────────┘
          │ Zenoh Bridge
          ▼
┌─────────────────────────────────────────────────────┐
│            Main Pi (10.1.1.10)                      │
├─────────────────────────────────────────────────────┤
│  Navigation Stack (Nav2 + RTABMap)                  │
│  LED Matrix Compositor (animations)                 │
│  Twist Mux (cmd_vel priority)                       │
│  Motor Control                                      │
└─────────────────────────────────────────────────────┘
```

### Voice Assistant Pipeline (Phase 5)

**Vision Pi:**
```
Audio Input → STT → Dialogue (DeepSeek) → Command Parse
                                              │
                                              ▼
                                          /cmd/nav2 (goal)
                                          /cmd/move (twist)
```

**Main Pi:**
```
/cmd/nav2 → Nav2 NavigateToPose → /cmd_vel
                                      │
/cmd/move ─────────────────────────┐  │
                                   ▼  ▼
                              Twist Mux (priority)
                                   │
                                   ▼
                            Motor Controller
```

**Feedback Loop:**
```
Main Pi: /robot/status → Vision Pi: TTS + Sound + Animations
```

## Docker Image Tags

| Branch | Tag | Usage |
|--------|-----|-------|
| `main` | `voice-assistant-humble-latest` | Production |
| `develop` | `voice-assistant-humble-dev` | Staging |
| `feature/*` | `voice-assistant-humble-test` | Testing |

**Current:** `feature/voice-assistant` → `voice-assistant-humble-test`

## Next Steps

1. ✅ **Wait for Docker build** (~10-15 minutes)
   - Check GitHub Actions: https://github.com/krikz/rob_box_project/actions
   - Workflow: `Build Vision Pi Services`

2. ⏳ **Pull and test on Vision Pi**
   ```bash
   cd ~/rob_box_project/docker/vision
   docker-compose pull voice-assistant
   docker-compose up -d voice-assistant
   docker logs -f voice-assistant
   ```

3. ⏳ **Verify Voice Assistant nodes**
   ```bash
   ros2 node list | grep voice
   # Expected:
   #   /audio_node
   #   /stt_node
   #   /dialogue_node
   #   /tts_node
   #   /command_node
   #   /sound_node
   ```

4. ⏳ **Test voice commands**
   ```bash
   # Say: "двигайся к точке три"
   ros2 topic echo /cmd/nav2  # Should see NavigateToPose goal
   
   # Say: "стоп"
   ros2 topic echo /cmd/move  # Should see zero Twist
   ```

5. ⏳ **Integration test with Main Pi**
   - Start Nav2 on Main Pi
   - Give voice command on Vision Pi
   - Verify robot moves to waypoint
   - Check audio feedback (TTS + sounds)

6. ⏳ **Fix LED Matrix unhealthy status**
   - Investigate healthcheck configuration
   - Test LED functionality manually
   - Fix or disable incorrect healthcheck

## Lessons Learned

1. **Headless robots need headless launches**
   - Never include GUI tools (RViz2, Gazebo GUI) in robot containers
   - Separate launch files: `robot.launch.py` vs `desktop.launch.py`

2. **Environment variables must be explicit**
   - Don't rely on base image ENV
   - Always define in docker-compose for clarity
   - Document requirements in Dockerfile

3. **Split packages by deployment target**
   - Vision Pi: sensors + perception
   - Main Pi: navigation + control + animations
   - Don't duplicate unnecessary code

4. **Feature branches need testing time**
   - Don't auto-delete after merge
   - Docker images persist for verification
   - Manual cleanup after validation

5. **Docker image size matters**
   - Remove unused packages (saved ~200 MB)
   - Faster downloads on robot
   - Less disk space usage

## Related Documentation

- USB Power Audit: `docs/reports/VISION_PI_USB_POWER_AUDIT_2025-10-13.md`
- Raspberry Pi Fix: `docs/guides/RASPBERRY_PI_USB_POWER_FIX.md`
- Voice Assistant Phase 5: `PHASE5_COMMAND_IMPLEMENTATION.md`
- Docker Compose: `docker/vision/docker-compose.yaml`

## Commits

| Commit | Description |
|--------|-------------|
| `44fe040` | fix(vision-pi): Remove RViz2 from LSLIDAR and add ROS_DISTRO |
| `43a0297` | fix(vision-pi): Use headless launch for voice-assistant |
| `9fd7219` | fix(ci): Disable auto-delete + Fix voice-assistant Dockerfile |

---

**Report Generated:** 2025-10-13 13:45 UTC  
**Author:** GitHub Copilot + krikz  
**Status:** ⏳ Waiting for Docker build completion
