# Docker Build Context Fix for Local Runner Workflows

**Date**: 2025-10-26  
**Issue**: GitHub Actions run #18818952962 failed with build context errors  
**Status**: ✅ Fixed

## Problem Description

### Symptoms
Local runner workflows (`build-main-services-local.yml`, `build-vision-services-local.yml`, `build-main-services-local-parallel.yml`) were failing with errors:

```
failed to calculate checksum of ref ...: "/src/...": not found
```

Build logs showed very small context transfers (e.g., "transferring context: 2B done"), indicating incorrect build context.

### Root Cause

**Inconsistent Build Context**:
- **GitHub Actions workflows** (correct): Used `context: .` (repository root)
- **Local runner workflows** (incorrect): Used `context: docker/main` or `context: docker/vision`

**Impact**: Dockerfiles with `COPY src/...` commands could not access the source code because the build context was limited to the docker subdirectory.

## Services Affected

### Main Pi Services
Services that need repository root context (have `COPY src/...` in Dockerfile):
- ✅ **perception** - Copies `src/rob_box_perception/` and `src/rob_box_perception_msgs/`
- ✅ **ros2_control** - Copies `src/vesc_nexus/` and `src/rob_box_description/`
- ✅ **vesc_nexus** - Copies `src/vesc_nexus/src/vesc_msgs/` and `src/vesc_nexus/src/vesc_nexus/`
- ✅ **micro_ros_agent** - Copies `src/robot_sensor_hub_msg/`

Services that don't need root context (no `COPY src/...`):
- rtabmap
- robot_state_publisher
- nav2
- twist_mux
- lslidar (but had missing Boost dependencies - also fixed)
- zenoh-router

### Vision Pi Services
Services that need repository root context:
- ✅ **led-matrix** (led_matrix) - Copies `src/ros2leds/`
- ✅ **voice-assistant** (voice_assistant) - Copies `src/rob_box_voice/`, `src/rob_box_animations/`, `src/rob_box_perception_msgs/`

Services that don't need root context:
- oak-d
- apriltag
- ceiling-camera
- voice_base
- zenoh-router

## Changes Made

### 1. build-main-services-local.yml
```diff
- docker buildx build ... docker/main
+ docker buildx build ... .
```

### 2. build-vision-services-local.yml
```diff
- docker buildx build ... docker/vision
+ docker buildx build ... .
```

### 3. build-main-services-local-parallel.yml
Fixed all 6 build jobs:
- rtabmap
- robot_state_publisher
- lslidar
- perception
- nav2
- zenoh-router

Changed context from `docker/main` to `.` in all occurrences.

### 4. docker/main/lslidar/Dockerfile
Added missing Boost dependencies:
```diff
 RUN apt-get update && apt-get install -y \
     ... \
+    libboost-dev \
+    libboost-thread-dev \
     && rm -rf /var/lib/apt/lists/*
```

This fixes: "Could NOT find Boost (missing: Boost_INCLUDE_DIR thread)" error.

## Verification Status

### GitHub Actions Workflows
✅ **Already correct** - No changes needed:
- `build-main-services.yml` - Uses `context: .` for services that need it
- `build-vision-services.yml` - Uses `context: .` for services that need it

### Local Runner Workflows
✅ **Fixed** - Build context changed to repository root:
- `build-main-services-local.yml`
- `build-vision-services-local.yml`
- `build-main-services-local-parallel.yml`

## Testing Recommendations

1. **Manual trigger on build server**:
   ```bash
   # On GitHub Actions -> Build Main Pi Services (Self-Hosted) -> Run workflow
   # Select services: perception,ros2_control,vesc_nexus,micro_ros_agent
   ```

2. **Verify all services build successfully** without checksum errors

3. **Check local registry**:
   ```bash
   curl http://localhost:5000/v2/_catalog
   ```

## Key Learnings

1. **Build context must match Dockerfile expectations**: If a Dockerfile has `COPY src/...`, the build context must include the `src/` directory.

2. **Consistency between workflows**: GitHub Actions and local runner workflows should use the same build context for the same services.

3. **Service-specific context**: Services that only copy from their own service directory can use a narrower context (e.g., `docker/main/rtabmap`), but services that need source code must use repository root.

4. **Base images vs service images**: Base images (rtabmap, depthai, pcl) don't need source code. Service images that build ROS packages need access to `src/`.

## Related Documentation
- [AGENT_GUIDE.md](docs/development/AGENT_GUIDE.md) - Docker architecture
- [DOCKER_STANDARDS.md](docs/development/DOCKER_STANDARDS.md) - Docker best practices
- [IMPLEMENTATION_SUMMARY_LOCAL_REGISTRY.md](IMPLEMENTATION_SUMMARY_LOCAL_REGISTRY.md) - Local registry setup

## References
- Failed run: https://github.com/krikz/rob_box_project/actions/runs/18818952962
- Fix commit: 4798405
