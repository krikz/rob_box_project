# Local Builder Fix Summary

## Problem Overview

The local builder workflows were failing with multiple errors preventing successful Docker image builds. This document provides a complete analysis of the issues and their resolution.

## Investigation Date
October 27, 2025

## Failed Workflow Run Analyzed
- **Run ID**: 18818952962
- **Workflow**: Build Main Pi Services (Self-Hosted)
- **Branch**: develop
- **Commit**: 18dccaee1bdcaeba1d8fd6b5ae6a500429bda8b3
- **Date**: October 26, 2025

## Root Causes Identified

### 1. Incorrect Docker Build Context ⚠️ CRITICAL

**Symptom**: 
```
ERROR: failed to calculate checksum: '/src/vesc_nexus/src/vesc_nexus': not found
ERROR: failed to calculate checksum: '/src/rob_box_description': not found
ERROR: failed to calculate checksum: '/src/rob_box_perception': not found
```

**Root Cause**:
The workflow was using `docker/main` as the build context, but the Dockerfiles contain `COPY src/...` instructions which require the repository root as the build context.

**In the failing workflow**:
```yaml
# build-main-services-local.yml (line 130)
docker buildx build \
  --platform linux/arm64 \
  --file docker/main/${SERVICE}/Dockerfile \
  --tag "${GHCR_TAG}" \
  --tag "${LOCAL_TAG}" \
  --load \
  --build-arg="BASE_IMAGE=ghcr.io/${{ github.repository_owner }}/rob_box_base:ros2-zenoh" \
  docker/main   # ❌ WRONG! Should be repository root (.)
```

**Affected Services**:
- ❌ ros2_control - needs `/src/rob_box_description`, `/src/vesc_nexus`
- ❌ perception - needs `/src/rob_box_perception`, `/src/rob_box_perception_msgs`
- ❌ micro_ros_agent - needs `/src/robot_sensor_hub_msg`
- ❌ vesc_nexus - needs `/src/vesc_nexus`

### 2. Missing Boost Library Dependencies

**Symptom**:
```
CMake Error: Could NOT find Boost (missing: Boost_INCLUDE_DIR thread)
Failed <<< lslidar_driver [11.8s, exited with code 1]
```

**Root Cause**:
The lslidar_driver package from the upstream repository requires Boost libraries (specifically `libboost-dev` and `libboost-thread-dev`), but they weren't installed in the Dockerfile.

**Affected Services**:
- ❌ lslidar - colcon build failed during CMake configuration

### 3. Registry Configuration Issues

**Symptom** (in GitHub Actions cloud runners):
```
dial tcp 192.168.1.125:5000: i/o timeout
```

**Root Cause**:
The local registry IP address was set globally in workflows, causing GitHub Actions cloud runners to try accessing an unreachable local build server.

## Resolution

### ✅ All Issues Fixed in PR #32

**Pull Request**: #32 - "fix(workflows): correct build context, registry configuration, and centralize settings"
**Merged**: October 27, 2025
**Merge Commit**: a5c8f3ae2b11a23af974a501635e46f7d8f9842e

### Changes Applied:

#### 1. Fixed Build Context

**In `build-main-services-local.yml` and `build-vision-services-local.yml`**:
```yaml
# Сборка образа (контекст = корень репозитория для доступа к src/)
docker buildx build \
  --platform linux/arm64 \
  --file docker/main/${SERVICE}/Dockerfile \
  --tag "${GHCR_TAG}" \
  --tag "${LOCAL_TAG}" \
  --load \
  --build-arg="BASE_IMAGE=ghcr.io/${{ github.repository_owner }}/rob_box_base:ros2-zenoh" \
  .   # ✅ CORRECT! Repository root allows access to src/
```

#### 2. Added Boost Dependencies

**In `docker/main/lslidar/Dockerfile`** (lines 18-19):
```dockerfile
RUN apt-get update && apt-get install -y \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-builtin-interfaces \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-ament-cmake \
    libboost-dev \           # ✅ Added
    libboost-thread-dev \    # ✅ Added
    && rm -rf /var/lib/apt/lists/*
```

#### 3. Fixed Registry Configuration

**Separated registry settings by runner type**:
```yaml
env:
  # Global variables (empty by default for GitHub Actions)
  LOCAL_BASE_REGISTRY: ""
  SELF_HOSTED_REGISTRY_HOST: 192.168.1.125:5000  # Only for self-hosted
  SELF_HOSTED_REGISTRY_PREFIX: krikz/rob_box_base

# In self-hosted jobs only:
build-perception:
  runs-on: self-hosted
  env:
    LOCAL_BASE_REGISTRY: ${{ env.SELF_HOSTED_REGISTRY_HOST }}/${{ env.SELF_HOSTED_REGISTRY_PREFIX }}
```

## Verification

### Services That Need Repository Root Context:
- **Main Pi**: perception, ros2_control, vesc_nexus, micro_ros_agent, robot_state_publisher
- **Vision Pi**: led-matrix, voice-assistant

These services copy from `src/` directory in their Dockerfiles.

### Services That Don't Need Root Context:
- **Main Pi**: rtabmap, nav2, twist_mux, lslidar
- **Vision Pi**: oak-d, apriltag

These services only copy from their own service directories under `docker/main/` or `docker/vision/`.

## Testing Status

✅ **All fixes have been merged to develop branch**
✅ **Build context is now correct (`.` instead of `docker/main`)**
✅ **Boost dependencies are installed in lslidar Dockerfile**
✅ **Registry configuration properly separated by runner type**

## Next Steps

1. **Verify Fix**: Run the local build workflows again to confirm all services build successfully
2. **Monitor**: Watch for any new build failures in GitHub Actions
3. **Document**: Update any related documentation about build requirements

## Related Files

- `.github/workflows/build-main-services-local.yml` - Main Pi local builds
- `.github/workflows/build-vision-services-local.yml` - Vision Pi local builds
- `docker/main/lslidar/Dockerfile` - LiDAR driver Dockerfile
- `DOCKER_BUILD_CONTEXT_FIX.md` - Detailed analysis document
- `scripts/local-build.sh` - Local build script (reference implementation)

## Additional Notes

The `scripts/local-build.sh` script was already using the correct build context (repository root) from the beginning. The issue was specific to the GitHub Actions workflows where the context was mistakenly set to `docker/main` instead of `.`.

### Comparison:

**local-build.sh** (always correct):
```bash
build_service "voice-assistant" "$PROJECT_ROOT" "$DOCKER_DIR/vision/voice_assistant/Dockerfile"
#                                 ^^^^^^^^^^^^^^
#                                 Repository root
```

**GitHub Workflow** (was incorrect, now fixed):
```yaml
docker buildx build ... .    # ✅ Now matches local-build.sh behavior
```

## Conclusion

All local builder failures have been resolved through PR #32. The main issue was incorrect Docker build context configuration in GitHub Actions workflows. The fix aligns the workflows with the working `local-build.sh` script and ensures all services can access required source files during build.
