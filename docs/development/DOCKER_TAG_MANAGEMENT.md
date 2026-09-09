# Docker Image Tag Management - Quick Reference

## Problem Solved

Previously, docker-compose files had hardcoded Docker image tags, requiring manual updates for each branch. Some services used `-latest`, others used `-test`, leading to inconsistencies.

**Before:**
```yaml
services:
  voice-assistant:
    image: ghcr.io/krikz/rob_box:voice-assistant-humble-test
    # После мерджа в develop поменяй на: voice-assistant-humble-dev  ❌
    # После мерджа в main поменяй на: voice-assistant-humble-latest  ❌
```

**After:**
```yaml
services:
  voice-assistant:
    image: ${SERVICE_IMAGE_PREFIX}:voice-assistant-${ROS_DISTRO}-${IMAGE_TAG}
    # Automatically uses correct tag based on branch ✅
```

## How It Works

### 1. Branch Detection
The system automatically detects your current git branch and selects appropriate Docker image tags:

| Branch Pattern | IMAGE_TAG | Use Case |
|----------------|-----------|----------|
| `main` | `latest` | Production deployment |
| `develop` | `dev` | Development/staging |
| `feature/*` | `test` | Feature testing |
| `copilot/*` | `test` | AI-assisted development |
| `release/1.0.0` | `rc-1.0.0` | Release candidates |
| `hotfix/1.0.1` | `hotfix-1.0.1` | Critical fixes |

### 2. Automatic Tag Setup

```bash
# From any directory in the project
source scripts/set-docker-tags.sh
```

This script will:
1. Detect current branch
2. Copy appropriate `.env` file to `docker/vision/.env` and `docker/main/.env`
3. Set environment variables for current shell session

### 3. Image Resolution

Docker Compose resolves images using this template:
```
${SERVICE_IMAGE_PREFIX}:${service_name}-${ROS_DISTRO}-${IMAGE_TAG}
```

**Example results:**
- Main branch: `ghcr.io/krikz/rob_box:voice-assistant-humble-latest`
- Develop branch: `ghcr.io/krikz/rob_box:voice-assistant-humble-dev`
- Feature branch: `ghcr.io/krikz/rob_box:voice-assistant-humble-test`

## Common Workflows

### Deploy Production (Raspberry Pi)

```bash
# SSH to Raspberry Pi
ssh ros2@10.1.1.21

# Navigate to project
cd ~/rob_box_project

# Switch to main branch
git checkout main
git pull

# Auto-configure tags
source scripts/set-docker-tags.sh

# Deploy
cd docker/vision
docker-compose pull
docker-compose up -d
```

### Test Development Changes

```bash
# On develop branch
git checkout develop
git pull

# Auto-configure for dev images
source scripts/set-docker-tags.sh

# Deploy dev images
cd docker/vision
docker-compose pull
docker-compose up -d
```

### Build Locally for Testing

```bash
# Build one service
./scripts/local-build.sh voice-assistant

# Build all Vision Pi services
./scripts/local-build.sh vision

# Use local image
cd docker/vision
IMAGE_TAG=local docker-compose up voice-assistant
```

### Switch Between Environments

```bash
# Use production images
export IMAGE_TAG=latest
docker-compose pull && docker-compose up -d

# Use development images
export IMAGE_TAG=dev
docker-compose pull && docker-compose up -d

# Use local images
export IMAGE_TAG=local
docker-compose up -d
```

## Files Reference

### Configuration Files

- `docker/.env.main` - Production configuration (IMAGE_TAG=latest)
- `docker/.env.develop` - Development configuration (IMAGE_TAG=dev)
- `docker/.env.feature` - Testing configuration (IMAGE_TAG=test)
- `docker/vision/.env` - Active configuration for Vision Pi (auto-generated)
- `docker/main/.env` - Active configuration for Main Pi (auto-generated)

### Scripts

- `scripts/set-docker-tags.sh` - Automatic tag detection and configuration
- `scripts/local-build.sh` - Local Docker image building
- `docker/vision/update_and_restart.sh` - Vision Pi deployment script
- `docker/main/update_and_restart.sh` - Main Pi deployment script

### Documentation

- `scripts/README.md` - Detailed script documentation with examples
- `docs/CI_CD_PIPELINE.md` - Updated CI/CD documentation
- `.actrc` - Configuration for running GitHub Actions locally

## Troubleshooting

### Wrong image tags being used

**Problem:**
```bash
docker-compose pull
# Downloads wrong tags (e.g., -latest when you want -dev)
```

**Solution:**
```bash
# Re-run tag setup
source scripts/set-docker-tags.sh

# Verify IMAGE_TAG
echo $IMAGE_TAG

# Or manually override
export IMAGE_TAG=dev
docker-compose pull
```

### Images not found

**Problem:**
```bash
docker-compose pull
ERROR: manifest for ghcr.io/krikz/rob_box:voice-assistant-humble-test not found
```

**Causes & Solutions:**

1. **Branch not built yet:** Feature branches create `-test` images only after GitHub Actions runs
   - Solution: Push to trigger build, or use `./scripts/local-build.sh`

2. **Wrong tag:** Check if the branch has corresponding images built
   - Solution: Use `IMAGE_TAG=latest` for production images

3. **Authentication:** Not logged in to ghcr.io
   - Solution: `docker login ghcr.io -u <username>`

### Local build fails

**Problem:**
```bash
./scripts/local-build.sh voice-assistant
# Build takes 30+ minutes or fails
```

**Cause:** Cross-compilation (x86_64 → ARM64) is slow

**Solutions:**
1. Build for native architecture: `./scripts/local-build.sh service linux/amd64`
2. Build on Raspberry Pi natively (faster)
3. Use GitHub Actions (10-15 min for ARM64)

## Best Practices

1. **Always run `set-docker-tags.sh` after switching branches**
   ```bash
   git checkout develop
   source scripts/set-docker-tags.sh
   ```

2. **Use local builds for rapid development**
   ```bash
   # Edit code
   IMAGE_TAG=local ./scripts/local-build.sh my-service
   IMAGE_TAG=local docker-compose up my-service
   # Test, repeat
   ```

3. **Test with dev images before production**
   ```bash
   # Test on develop first
   git checkout develop
   source scripts/set-docker-tags.sh
   # Deploy and validate
   
   # Then promote to main
   git checkout main
   git merge develop
   git push
   ```

4. **Use explicit IMAGE_TAG for CI/CD scripts**
   ```bash
   # In automated scripts, be explicit
   IMAGE_TAG=latest docker-compose pull
   ```

5. **Don't commit .env files with secrets**
   - `.env`, `.env.secrets` are in `.gitignore`
   - Only `.env.main`, `.env.develop`, `.env.feature` are version-controlled

## Integration with Existing Workflows

### GitHub Actions

Workflows already build images with correct tags:
- `main` branch → `-humble-latest`
- `develop` branch → `-humble-dev`
- `feature/*` branches → `-humble-test`

No changes needed to workflows - they work automatically!

### Update Scripts

Both `docker/vision/update_and_restart.sh` and `docker/main/update_and_restart.sh` now call `set-docker-tags.sh` automatically:

```bash
# Just run the update script
./docker/vision/update_and_restart.sh
# Automatically uses correct tags based on current branch
```

## Quick Command Reference

```bash
# Setup tags
source scripts/set-docker-tags.sh

# Build locally
./scripts/local-build.sh <service|vision|main|all>

# Deploy
cd docker/vision && docker-compose pull && docker-compose up -d

# Override tag
IMAGE_TAG=dev docker-compose pull

# Check config
docker-compose config | grep image:

# View active env
cat docker/vision/.env

# Run local workflow
act -j build-oak-d  # Requires 'act' tool
```

## Summary

**What Changed:**
- ✅ Docker-compose files now use environment variables
- ✅ Branch-specific .env files for automatic tag selection
- ✅ Scripts for automatic tag detection and local building
- ✅ Updated deployment scripts to use new system
- ✅ Comprehensive documentation

**What Didn't Change:**
- ❌ GitHub Actions workflows (they already worked correctly)
- ❌ Image naming convention (still `service-distro-tag`)
- ❌ Registry location (still `ghcr.io/krikz/rob_box`)

**Result:**
- No more manual tag updates
- Consistent image usage across all branches
- Easy switching between environments
- Local build capability for faster development
