# Docker Build Context Fix for Local Runner Workflows

**Date**: 2025-10-26  
**Updated**: 2025-10-27
**Issue**: GitHub Actions failures with build context and registry errors  
**Status**: ✅ Fixed

## Problem Description

### Issue 1: Build Context Mismatch (Fixed 2025-10-26)

**Symptoms:**
Local runner workflows (`build-main-services-local.yml`, `build-vision-services-local.yml`, `build-main-services-local-parallel.yml`) were failing with errors:

```
failed to calculate checksum of ref ...: "/src/...": not found
```

Build logs showed very small context transfers (e.g., "transferring context: 2B done"), indicating incorrect build context.

**Root Cause:**
- **GitHub Actions workflows** (correct): Used `context: .` (repository root)
- **Local runner workflows** (incorrect): Used `context: docker/main` or `context: docker/vision`

**Impact:** Dockerfiles with `COPY src/...` commands could not access the source code because the build context was limited to the docker subdirectory.

### Issue 2: Registry Configuration Error (Fixed 2025-10-27)

**Symptoms:**
GitHub Actions builds failing with timeout errors:

```
ERROR: failed to do request: Head "https://192.168.1.125:5000/v2/krikz/rob_box_base/manifests/rtabmap": dial tcp 192.168.1.125:5000: i/o timeout
```

Affected jobs:
- build-rtabmap
- build-lslidar
- build-nav2
- build-robot-state-publisher

**Root Cause:**
`LOCAL_BASE_REGISTRY=192.168.1.125:5000/krikz/rob_box_base` was set globally in workflow `env` section. This caused ALL jobs (including GitHub cloud runners on ubuntu-latest) to try accessing the local registry, which is not reachable from GitHub's infrastructure.

**Impact:** 
- GitHub Actions cloud runners couldn't build services
- Only self-hosted runner jobs (perception, ros2_control) would work
- Auto-merge workflows failed completely

### Issue 3: Hardcoded Configuration (Fixed 2025-10-27)

**Symptoms:**
IP address `192.168.1.125` was hardcoded in job-specific environment variables, making it difficult to change when moving to a different build server.

**Root Cause:**
```yaml
build-perception:
  env:
    LOCAL_BASE_REGISTRY: 192.168.1.125:5000/krikz/rob_box_base  # Hardcoded!
```

**Impact:**
- Difficult to maintain and update registry configuration
- Required editing multiple places when changing server
- Error-prone when updating configuration

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

### Fix 1: Build Context (2025-10-26)

**1. build-main-services-local.yml**
```diff
- docker buildx build ... docker/main
+ docker buildx build ... .
```

**2. build-vision-services-local.yml**
```diff
- docker buildx build ... docker/vision
+ docker buildx build ... .
```

**3. build-main-services-local-parallel.yml**
Fixed all 6 build jobs:
- rtabmap
- robot_state_publisher
- lslidar
- perception
- nav2
- zenoh-router

Changed context from `docker/main` to `.` in all occurrences.

**4. docker/main/lslidar/Dockerfile**
Added missing Boost dependencies:
```diff
 RUN apt-get update && apt-get install -y \
     ... \
+    libboost-dev \
+    libboost-thread-dev \
     && rm -rf /var/lib/apt/lists/*
```

This fixes: "Could NOT find Boost (missing: Boost_INCLUDE_DIR thread)" error.

### Fix 2: Registry Configuration (2025-10-27)

**Problem:** `LOCAL_BASE_REGISTRY` was set globally, affecting all runners.

**Solution:** Separate registry configuration by runner type.

**1. build-main-services.yml**
```diff
 env:
   REGISTRY: ghcr.io
   IMAGE_PREFIX: ghcr.io/${{ github.repository_owner }}/rob_box
   ROS_DISTRO: humble
-  LOCAL_BASE_REGISTRY: 192.168.1.125:5000/krikz/rob_box_base
+  # Локальный registry - НЕ ИСПОЛЬЗУЕТСЯ на GitHub cloud runners
+  LOCAL_BASE_REGISTRY: ""
+  # Конфигурация для self-hosted runners
+  SELF_HOSTED_REGISTRY_HOST: 192.168.1.125:5000
+  SELF_HOSTED_REGISTRY_PREFIX: krikz/rob_box_base

+build-ros2-control:
+  runs-on: self-hosted
+  env:
+    LOCAL_BASE_REGISTRY: ${{ env.SELF_HOSTED_REGISTRY_HOST }}/${{ env.SELF_HOSTED_REGISTRY_PREFIX }}
+
+build-perception:
+  runs-on: self-hosted
+  env:
+    LOCAL_BASE_REGISTRY: ${{ env.SELF_HOSTED_REGISTRY_HOST }}/${{ env.SELF_HOSTED_REGISTRY_PREFIX }}
```

**2. build-vision-services.yml**
```diff
 env:
-  LOCAL_BASE_REGISTRY: 192.168.1.125:5000/krikz/rob_box_base
+  LOCAL_BASE_REGISTRY: ""
```

All vision service jobs run on ubuntu-latest, so they use ghcr.io.

### Fix 3: Configuration Refactoring (2025-10-27)

**Problem:** IP addresses hardcoded in multiple job definitions.

**Solution:** Extract to global environment variables.

```yaml
# At workflow level - single point of configuration
env:
  SELF_HOSTED_REGISTRY_HOST: 192.168.1.125:5000
  SELF_HOSTED_REGISTRY_PREFIX: krikz/rob_box_base

# In job definitions - composed from variables
build-perception:
  env:
    LOCAL_BASE_REGISTRY: ${{ env.SELF_HOSTED_REGISTRY_HOST }}/${{ env.SELF_HOSTED_REGISTRY_PREFIX }}
```

**Benefits:**
- Single place to update registry configuration
- Easy to change when moving to different build server
- Clear separation of concerns
- No duplication

## Configuration Management

### Changing Self-Hosted Registry Location

To change the self-hosted registry location (e.g., when moving to a new build server), update these variables at the top of `build-main-services.yml`:

```yaml
env:
  # IP адрес и порт локального registry для self-hosted runners
  # Изменить при необходимости переноса на другой сервер
  SELF_HOSTED_REGISTRY_HOST: 192.168.1.125:5000  # ← Change this
  SELF_HOSTED_REGISTRY_PREFIX: krikz/rob_box_base  # ← And this if needed
```

**Example: Moving to a new server at 10.0.1.50:5000**
```yaml
SELF_HOSTED_REGISTRY_HOST: 10.0.1.50:5000
SELF_HOSTED_REGISTRY_PREFIX: krikz/rob_box_base
```

All jobs using self-hosted runners will automatically use the new configuration.

| Workflow Type | Runner Type | Registry Used | Configuration |
|--------------|-------------|---------------|---------------|
| **GitHub Actions** | ubuntu-latest | ghcr.io | `LOCAL_BASE_REGISTRY=""` (empty) |
| **GitHub Actions** | self-hosted | 192.168.1.125:5000 | Job-specific env var |
| **Local Workflows** | self-hosted | localhost:5000 | Workflow-specific |

### How It Works

**For GitHub cloud runners (ubuntu-latest):**
```yaml
env:
  LOCAL_BASE_REGISTRY: ""  # Empty at workflow level

build-args: |
  BASE_IMAGE=${{ env.LOCAL_BASE_REGISTRY || 'ghcr.io/krikz/rob_box_base' }}:ros2-zenoh
  # Empty string is falsy in bash, so || fallback triggers
  # Result: BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh ✅
```

**For self-hosted runners:**
```yaml
build-perception:
  runs-on: self-hosted
  env:
    LOCAL_BASE_REGISTRY: 192.168.1.125:5000/krikz/rob_box_base

  build-args: |
    BASE_IMAGE=${{ env.LOCAL_BASE_REGISTRY || 'ghcr.io/krikz/rob_box_base' }}:ros2-zenoh
    # Non-empty string is truthy, uses it directly
    # Result: BASE_IMAGE=192.168.1.125:5000/krikz/rob_box_base:ros2-zenoh ✅
```

### Job Runner Assignments

**Main Pi Services (build-main-services.yml):**
- `build-rtabmap` → ubuntu-latest → ghcr.io
- `build-robot-state-publisher` → ubuntu-latest → ghcr.io
- `build-nav2` → ubuntu-latest → ghcr.io
- `build-lslidar` → ubuntu-latest → ghcr.io
- `build-twist-mux` → ubuntu-latest → ghcr.io
- `build-micro-ros-agent` → ubuntu-latest → ghcr.io
- `build-ros2-control` → **self-hosted** → 192.168.1.125:5000
- `build-perception` → **self-hosted** → 192.168.1.125:5000

**Vision Pi Services (build-vision-services.yml):**
- `build-oak-d` → ubuntu-latest → ghcr.io
- `build-led-matrix` → ubuntu-latest → ghcr.io
- `build-ceiling-camera` → ubuntu-latest → ghcr.io
- `build-voice-assistant` → ubuntu-latest → ghcr.io

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

5. **Registry access boundaries**: Environment variables set at workflow level affect ALL jobs. Use job-specific env vars to separate behavior between GitHub cloud runners and self-hosted runners.

6. **Empty string fallback**: In GitHub Actions expressions, empty string `""` is falsy, allowing `||` fallback to work. Non-empty strings (even with spaces) are truthy and bypass the fallback.

7. **Runner accessibility**: GitHub cloud runners (ubuntu-latest) cannot access private networks (192.168.x.x). Self-hosted runners can access both public (ghcr.io) and private (local registry) resources.

8. **Configuration centralization**: Extract configuration values to workflow-level environment variables for easy maintenance. Use composition in job-level env vars to build complex values from simple parts.

9. **Single source of truth**: Keep IP addresses and other infrastructure configuration in one place at the top of the workflow file. This makes updates easier and reduces errors.

## Related Documentation
- [AGENT_GUIDE.md](docs/development/AGENT_GUIDE.md) - Docker architecture
- [DOCKER_STANDARDS.md](docs/development/DOCKER_STANDARDS.md) - Docker best practices
- [IMPLEMENTATION_SUMMARY_LOCAL_REGISTRY.md](IMPLEMENTATION_SUMMARY_LOCAL_REGISTRY.md) - Local registry setup

## References
- Failed run (build context): https://github.com/krikz/rob_box_project/actions/runs/18818952962
- Failed run (registry): https://github.com/krikz/rob_box_project/actions/runs/18821166593
- Fix commit (build context): 47984054e69bf26877f6ee8b76d3d2169481b89d
- Fix commit (registry): e442de2ffcd3c6af3de25446069484a96658ed18
- Fix commit (configuration): a7750e700bd1501c830cb9ed25d26aff32769afb
