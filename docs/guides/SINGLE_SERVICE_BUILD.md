# Quick Guide: Building Single Service

This guide explains how to quickly build a single Docker image using the `L: Build Single Service` workflow.

## Overview

The `L: Build Single Service` workflow allows you to build just one Docker image instead of all services, saving significant time during development.

**Time Comparison:**
- **All services**: 30-60 minutes ⏱️
- **Single service**: 1-5 minutes ⚡

**Architecture:**
```
┌─────────────────────────────────────────────────────────┐
│  GitHub Actions: L: Build Single Service               │
│                                                         │
│  Inputs:                                               │
│    ├─ Branch (main/develop/feature/*)                 │
│    ├─ Pi Type (main/vision/base)                      │
│    ├─ Service (voice-assistant, nav2, etc.)           │
│    └─ Push to Registry (true/false)                   │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  Self-Hosted Runner (Build Machine)                    │
│                                                         │
│  1. Checkout code from selected branch                 │
│  2. Determine build configuration                      │
│  3. Build Docker image with buildx                     │
│  4. Tag for GHCR and local registry                    │
│  5. Push to localhost:5000 (if enabled)                │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  Local Registry (localhost:5000)                       │
│                                                         │
│  Image ready for deployment to Raspberry Pi            │
└─────────────────────────────────────────────────────────┘
```

## Prerequisites

- Access to the GitHub repository
- Self-hosted runner must be configured and running
- Local registry (`localhost:5000`) must be available

## Usage

### Step-by-Step Instructions

1. **Navigate to Actions**
   - Go to the GitHub repository
   - Click on the "Actions" tab
   - Find "L: Build Single Service" in the workflows list

2. **Start the Workflow**
   - Click on "L: Build Single Service"
   - Click the "Run workflow" button (top right)

3. **Configure Build Parameters**

   **Branch** (required)
   - The branch to build from
   - Examples: `main`, `develop`, `feature/new-feature`

   **Pi Type** (required)
   - Choose from dropdown:
     - `main` - Services for Main Pi
     - `vision` - Services for Vision Pi
     - `base` - Base Docker images

   **Service** (required)
   - Enter the service name (see list below)
   - Both hyphenated and underscore formats accepted
   - Examples: `voice-assistant` or `voice_assistant`

   **Push to Registry** (optional)
   - Default: `true`
   - Set to `false` if you only want to build locally without pushing

   **Create Issue on Failure** (optional)
   - Default: `true`
   - Automatically creates a GitHub issue if the build fails
   - Issue includes error details, build configuration, and troubleshooting steps
   - Set to `false` to disable automatic issue creation

4. **Run the Workflow**
   - Click "Run workflow" button
   - Monitor progress in the Actions tab

## Available Services

### Main Pi Services

```
robot-state-publisher    (robot_state_publisher)
rtabmap
twist-mux               (twist_mux)
micro-ros-agent         (micro_ros_agent)
ros2-control            (ros2_control)
nav2
lslidar
perception
zenoh-router
```

### Vision Pi Services

```
oak-d
led-matrix              (led_matrix)
ceiling-camera
voice-assistant         (voice_assistant)
apriltag
zenoh-router
```

### Base Images

```
ros2-zenoh
rtabmap
depthai
pcl
```

## Common Use Cases

### 1. Quick Development Iteration

You're working on the voice assistant and need to test changes quickly:

```
Branch: develop
Pi Type: vision
Service: voice-assistant
Push to Registry: true
```

### 2. Testing Feature Branch

You created a feature branch with navigation improvements:

```
Branch: feature/nav-improvements
Pi Type: main
Service: nav2
Push to Registry: true
```

### 3. Rebuilding Base Image

You updated the ROS 2 + Zenoh base image:

```
Branch: main
Pi Type: base
Service: ros2-zenoh
Push to Registry: true
```

### 4. Local Build Only

You want to build without pushing to registry:

```
Branch: develop
Pi Type: vision
Service: oak-d
Push to Registry: false
```

### 5. Build Without Issue Creation

You're testing and don't want issues created for expected failures:

```
Branch: feature/experimental
Pi Type: main
Service: perception
Push to Registry: true
Create Issue on Failure: false
```

## Automatic Issue Creation

When a build fails and `create_issue_on_failure` is `true` (default), the workflow automatically creates a GitHub issue with:

- **Service and configuration details**
- **Error timestamp and workflow link**
- **Build configuration** (Dockerfile, context, base image)
- **Possible causes** of the failure
- **Recommended troubleshooting actions**
- **Quick commands** for debugging
- **Label:** `bug`, `build-failure`, `ci/cd`
- **Assignee:** `@krikz`

This helps track build problems and provides immediate guidance for fixing issues.

## Image Tags

```
Branch: develop
Pi Type: vision
Service: oak-d
Push to Registry: false
```

## Image Tags

Built images are tagged based on the branch:

| Branch | Tag | Example |
|--------|-----|---------|
| `main` | `latest` | `localhost:5000/krikz/rob_box:voice-assistant-humble-latest` |
| `develop` | `dev` | `localhost:5000/krikz/rob_box:voice-assistant-humble-dev` |
| Other | `local` | `localhost:5000/krikz/rob_box:voice-assistant-humble-local` |

## Troubleshooting

### Service Not Found

**Error:** "Unknown [Pi Type] service: [service-name]"

**Solution:** Check the service name spelling and ensure it matches one from the available services list above.

### Build Failed

**Error:** Build fails with missing base image

**Solution:** Build the required base image first. Check the error message to see which base image is needed, then build it:

```
Pi Type: base
Service: [required-base-image]
```

### Registry Push Failed

**Error:** "Error response from daemon: Get http://localhost:5000/..."

**Solution:** Ensure the local registry is running:

```bash
docker ps | grep registry
```

If not running, start it:

```bash
docker run -d -p 5000:5000 --restart=always --name registry registry:2
```

## Advanced Tips

### Service Name Formats

Both formats are accepted:
- Hyphenated: `voice-assistant`, `led-matrix`, `micro-ros-agent`
- Underscore: `voice_assistant`, `led_matrix`, `micro_ros_agent`

### Build Dependencies

Some services depend on base images. If a build fails with missing base image:

1. Build the base image first
2. Then build your service

**Example:** Building `oak-d` requires `depthai` base image:
1. First build: `base` → `depthai`
2. Then build: `vision` → `oak-d`

### Checking Build Results

After a successful build, you can verify the image:

```bash
# On the build machine
docker images | grep rob_box

# Check image size and creation date
docker images localhost:5000/krikz/rob_box:voice-assistant-humble-dev
```

## Related Documentation

- [CI/CD Pipeline](../CI_CD_PIPELINE.md) - Complete workflow documentation
- [Docker Standards](../development/DOCKER_STANDARDS.md) - Docker best practices
- [Local Build Guide](../development/LOCAL_BUILD.md) - Setting up local builds

## FAQ

**Q: Why use this instead of `L-Build All Services`?**
A: When you only changed one service, building all services wastes 25-55 minutes. This workflow builds just what you need.

**Q: Can I build multiple services at once?**
A: No, this workflow is designed for single service builds. For multiple services, use `L-Build Main Pi Services`, `L-Build Vision Pi Services`, or `L-Build All Services`.

**Q: What happens to the old image?**
A: Docker keeps the old image. You can clean up old images with `docker image prune` on the build machine.

**Q: Can I use this on GitHub Actions runners?**
A: No, this workflow requires `self-hosted` runner. For GitHub Actions runners, use the `G-*` workflows instead.

**Q: How do I know which base image a service needs?**
A: Check the service's Dockerfile or refer to the workflow summary output which shows this information.

**Q: What happens when a build fails?**
A: By default, the workflow automatically creates a GitHub issue with detailed error information, troubleshooting steps, and quick commands for debugging. You can disable this by setting `create_issue_on_failure` to `false`.

**Q: Can I disable automatic issue creation?**
A: Yes, set the `create_issue_on_failure` parameter to `false` when running the workflow. This is useful when testing experimental changes where failures are expected.
