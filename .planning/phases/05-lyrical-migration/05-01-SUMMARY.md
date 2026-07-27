# 05-01-SUMMARY.md — Wave 1: Mechanical humble→lyrical Migration

> **Date:** 2026-07-27  
> **Plan:** 05-01-PLAN.md  
> **Status:** COMPLETE  
> **Commit:** 2a887391

---

## Summary

Выполнена механическая миграция ROS 2 Humble → Lyrical Luth для ВСЕХ файлов проекта, кроме Nav2 и DepthAI (отложены на Wave 2).

**90 files changed, 241 insertions(+), 241 deletions(-)** — чистая замена строк.

---

## Tasks Completed

### Task 1: Foundation — Base Dockerfiles + .env + docker-compose ✅
- `docker/base/Dockerfile.ros2-zenoh`: FROM ros:lyrical-ros-base, ros-lyrical-rmw-zenoh-cpp, LD_LIBRARY_PATH updated
- `docker/base/Dockerfile.rtabmap`: FROM introlab3it/rtabmap_ros:lyrical-latest, 4 ros packages updated
- `docker/base/Dockerfile.pcl`: 3 ros packages updated (pcl-conversions, pcl-ros, diagnostic-updater)
- 7 .env files: ROS_DISTRO=lyrical
- 2 docker-compose.yaml: ${ROS_DISTRO:-lyrical}, LD_LIBRARY_PATH updated

### Task 2: Main Pi Dockerfiles (10 services) ✅
- lslidar, robot_state_publisher, rtabmap, teleop, twist_mux, micro_ros_agent, ros2_control, vesc_nexus, perception, zenoh-router
- perception: added `--break-system-packages` to pip3 install
- nav2/Dockerfile: UNTOUCHED (Wave 2)

### Task 3: Vision Pi Dockerfiles (9+ services) ✅
- oak-d, apriltag, ceiling-camera, led_matrix, voice_assistant, voice_base, telegram_bot, zenoh-router
- mock_llm, scenario_runner: added `--break-system-packages`
- led_matrix, voice_base, telegram_bot: added `--break-system-packages` to all pip3 install commands
- vision/zenoh-router: changed FROM to `ros:lyrical-ros-base` (D-03-OVERRIDE — depthai-ros:lyrical doesn't exist)
- voice_resources, supercollider: unchanged (non-ROS)

### Task 4: CI/CD Workflows + Scripts ✅
- 12 workflow files: ROS_DISTRO: lyrical, ubuntu-26.04, required-ros-distributions: lyrical
- 39 scripts: /opt/ros/lyrical/ paths
- switch_version.sh (both): lyrical in supported list, comments fixed
- patch_rtabmap_launch.py: /opt/ros/lyrical/share/rtabmap_launch/
- src/ros2leds/Dockerfile: migrated (was missed in initial scope)

### Task 5: Final Verification ✅
- No nav2/depthai files in diff
- No behaviotree-cpp-v3 outside nav2
- No double-replace artifacts
- All pip install have --break-system-packages
- All pip-using Dockerfiles have python3-pip
- No humble LD_LIBRARY_PATH
- No source /opt/ros/humble in scripts

---

## Decisions Applied

| Decision | Application |
|----------|-------------|
| D-01 | Migration from develop (Humble) → feature/lyrical |
| D-02 | DepthAI untouched (Wave 2) |
| D-03 | Zenoh architecture unchanged; vision/zenoh-router base changed to ros:lyrical-ros-base |
| D-04 | RTAB-Map uses lyrical-latest tag (confirmed available) |
| D-05 | Migration before Milestone 2 |
| D-06 | Based on develop branch |

---

## Deviations

1. **vision/zenoh-router: FROM changed to ros:lyrical-ros-base** (D-03-OVERRIDE)
   - Original Dockerfile used `FROM luxonis/depthai-ros:humble-latest` — arch incorrect
   - `depthai-ros:lyrical` doesn't exist, so changed to `ros:lyrical-ros-base`
   - Role unchanged: standalone zenoh router container

2. **src/ros2leds/Dockerfile: initially missed, now migrated**
   - Was in `src/` directory, outside docker/ scan
   - Added --break-system-packages + humble→lyrical

---

## Known Remaining Issues (for Wave 2 / CI)

- **micro_ros_agent**: `git clone -b lyrical https://github.com/micro-ROS/micro_ros_setup.git` — the `lyrical` branch may not exist. Expected to fail in CI, needs fix in Wave 2.
- **nav2/Dockerfile**: Still has `behaviortree-cpp-v3` — intentionally left for Wave 2 source-build
- **voice_base**: `ros-lyrical-nav2-msgs` — Nav2 msgs not in Lyrical apt. May fail CI build.
- **docker/build/data/runner*/**: CI cache artifacts contain old humble paths — not version-controlled, runtime artifacts

---

## Next Steps

1. User runs: `gh workflow run "L: Build All Services" --ref feature/lyrical -f build_base_images=true`
2. Analyze CI logs for build failures
3. Fix issues iteratively (per 05-ACCEPTANCE.md cycle)
4. When base images build green → Phase 2 (all services)
5. When all services green → Wave 2 (05-02-PLAN.md: Nav2 + DepthAI source-build)
