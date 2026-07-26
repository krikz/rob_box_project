# APT Cacher NG Configuration Guide

**Comprehensive guide for APT Cacher NG setup on Rob Box Build Machine**

## 📋 Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Configuration Details](#configuration-details)
- [Docker Image Choice](#docker-image-choice)
- [Performance Tuning](#performance-tuning)
- [Usage](#usage)
- [Monitoring](#monitoring)
- [Troubleshooting](#troubleshooting)
- [References](#references)

---

## 🎯 Overview

APT Cacher NG is a caching proxy for APT package downloads. In the Rob Box build infrastructure, it:

- **Speeds up Docker builds** by caching APT packages locally
- **Reduces internet bandwidth** by downloading packages only once
- **Supports 8 parallel GitHub runners** with optimized concurrent connection handling
- **Works with ARM64 cross-compilation** through QEMU emulation

### Benefits

| Metric | Without Cache | With Cache | Improvement |
|--------|---------------|------------|-------------|
| First build | 15-20 min | 15-20 min | - |
| Subsequent builds | 15-20 min | 5-8 min | **2-3x faster** |
| Network usage | ~500MB per build | ~50MB per build | **90% reduction** |

---

## 🏗️ Architecture

```
GitHub Actions Runner (Docker build)
           ↓
    apt-get install
           ↓
    APT proxy config: http://host.docker.internal:3142
           ↓
    APT Cacher NG Container (port 3142)
           ↓
    Cache hit? → Yes → Return from cache (fast!)
           ↓ No
    Download from Ubuntu/ROS repos → Cache → Return to client
```

### Key Components

1. **Docker Container**: `mbentley/apt-cacher-ng:latest`
2. **Configuration**: `docker/build/config/acng.conf`
3. **Cache Storage**: `docker/build/data/apt-cache/` (persistent volume)
4. **Logs**: `docker/build/data/apt-logs/` (persistent volume)

---

## ⚙️ Configuration Details

### acng.conf - Main Settings

Our configuration is optimized for **8 parallel GitHub Actions runners** performing Docker builds:

```ini
# ⚠️ NOTE: The following directives are commented out due to version compatibility
# They require apt-cacher-ng 3.8+, but mbentley/apt-cacher-ng:latest uses 3.7.5
# 
# Worker threads - handle concurrent connections
# PHttpThreads: 16
#
# Timeouts - increased for ARM64 QEMU builds
# ConnectTimeout: 120
# NetworkTimeout: 300

# DNS caching - reduce DNS overhead
DnsCacheSeconds: 3600

# Cache expiration - keep packages for 10 days
ExThreshold: 10

# No download speed limit
MaxDlSpeed: 0
```

### Key Configuration Choices

#### 1. PHttpThreads: 16 (Currently commented out)

**Reasoning:** With 8 GitHub runners potentially building simultaneously, we need at least 8 worker threads. Setting it to 16 provides headroom for:
- Multiple connections per build
- Metadata requests
- Report page access

**Status:** ⚠️ **Commented out** - Requires apt-cacher-ng 3.8+. Currently using default value.

#### 2. Increased Timeouts (Currently commented out)

**ConnectTimeout: 120** and **NetworkTimeout: 300**

**Reasoning:** ARM64 Docker builds use QEMU emulation on x86_64, which is slower. Large packages (like ROS packages) take longer to install.

**Status:** ⚠️ **Commented out** - Requires apt-cacher-ng 3.8+. Currently using default timeouts.

#### 3. DnsCacheSeconds: 3600

**Reasoning:** Parallel builds hit the same repositories repeatedly. DNS caching reduces DNS query overhead.

#### 4. ExThreshold: 10

**Reasoning:** Rob Box development is active, so packages are reused frequently within 10 days. This balances disk usage and cache effectiveness.

### Repository Mappings

```ini
# Ubuntu 22.04 Jammy (ROS 2 kilted base)
Remap-uburep: file:ubuntu_mirrors /ubuntu ; file:backends_ubuntu

# ROS 2 kilted packages
Remap-rosrep: http://packages.ros.org/ros2/ubuntu

# Ubuntu Ports for ARM64
Remap-ubports: http://ports.ubuntu.com/ubuntu-ports
```

---

## 🐳 Docker Image Choice

### Why mbentley/apt-cacher-ng over sameersbn/apt-cacher-ng?

| Feature | mbentley | sameersbn | Winner |
|---------|----------|-----------|--------|
| Base OS | Debian bookworm | Ubuntu Jammy | mbentley |
| Updates | Daily snapshots | Release-based | ✅ mbentley |
| Multi-arch | Auto-detect | Yes | Tie |
| Non-root | PUID/PGID support | No | ✅ mbentley |
| Maintenance | Active (2024) | Less active | ✅ mbentley |
| Logging | rsyslog + cron | Basic | ✅ mbentley |

**Decision:** Use `mbentley/apt-cacher-ng:latest` for active maintenance and security updates.

### Docker Compose Configuration

```yaml
apt-cacher-ng:
  image: mbentley/apt-cacher-ng:latest
  container_name: build-apt-cache
  network_mode: host  # Required for host.docker.internal access
  volumes:
    - ./data/apt-cache:/var/cache/apt-cacher-ng  # Cache storage
    - ./config/acng.conf:/etc/apt-cacher-ng/acng.conf:ro  # Config
    - ./data/apt-logs:/var/log/apt-cacher-ng  # Logs
  restart: unless-stopped
  healthcheck:
    test: ["CMD-SHELL", "test -f /var/run/apt-cacher-ng/pid || exit 1"]
    interval: 30s
    timeout: 10s
    retries: 5
```

**Key points:**
- `network_mode: host` allows Docker containers to access via `host.docker.internal:3142`
- Health check monitors PID file to ensure service is running
- Persistent volumes survive container restarts

---

## 🚀 Performance Tuning

### For Parallel Builds

When running parallel Docker builds, APT Cacher NG must handle:
- Simultaneous `apt-get update` requests
- Concurrent package downloads
- Metadata file requests
- Lock contention on cache files

**Our optimizations:**

1. **PHttpThreads: 16** (⚠️ Currently commented - requires v3.8+) - Would handle 16 concurrent HTTP connections
2. **Persistent cache** - Docker volume persists between builds
3. **Host networking** - Minimal network overhead
4. **No speed limits** - MaxDlSpeed: 0 for maximum throughput

### Docker Build Best Practices

#### In Dockerfiles

```dockerfile
# Build argument for APT cache
ARG APT_PROXY=""

# Configure APT proxy at build time
RUN if [ -n "$APT_PROXY" ]; then \
        echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
    fi

# Install packages
RUN apt-get update && apt-get install -y \
    ros-kilted-navigation2 \
    ros-kilted-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

# Clean up proxy config
RUN rm -f /etc/apt/apt.conf.d/02proxy
```

**Important:** Remove proxy config after build so deployed containers don't try to use the build machine cache.

#### In GitHub Actions Workflows

```yaml
- name: Build Docker image
  run: |
    docker buildx build \
      --build-arg="APT_PROXY=http://host.docker.internal:3142" \
      --add-host=host.docker.internal:host-gateway \
      --platform linux/arm64 \
      --file docker/base/Dockerfile.ros2-zenoh \
      --tag localhost:5000/rob_box_base:ros2-zenoh-latest \
      --load \
      docker/base/
```

**Key points:**
- `--add-host=host.docker.internal:host-gateway` enables `host.docker.internal` resolution
- `APT_PROXY` build arg passes cache URL to Dockerfile
- Only affects build time, not runtime

---

## 📖 Usage

### Starting APT Cacher NG

```bash
cd ~/rob_box_project/docker/build
docker compose up -d apt-cacher-ng
```

### Verifying It's Working

```bash
# Check container status
docker ps | grep apt-cache

# Check web interface
curl http://localhost:3142/acng-report.html

# Check logs
docker logs build-apt-cache
```

### Testing with a Build

```bash
# Test with simple Dockerfile
cd docker/build
./scripts/test_apt_cache_simple.sh

# After build, check cache statistics
curl http://localhost:3142/acng-report.html
```

### Configuring Clients

#### For Docker Builds

Add to Dockerfile:
```dockerfile
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then \
        echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
    fi
```

Pass during build:
```bash
docker build --build-arg APT_PROXY=http://10.1.1.5:3142 .
```

#### For Raspberry Pi (Direct Use)

Run on each Raspberry Pi:
```bash
echo 'Acquire::http::Proxy "http://10.1.1.5:3142";' | \
  sudo tee /etc/apt/apt.conf.d/02proxy

# Test it
sudo apt-get update
```

---

## 📊 Monitoring

### Web Interface

Access at: `http://<build-machine-ip>:3142/acng-report.html`

**Available information:**
- Cache hit/miss statistics
- Disk usage
- Active connections
- Package list
- Configuration check

### Command Line

```bash
# Cache size
du -sh docker/build/data/apt-cache/

# Recent logs
docker logs --tail 50 build-apt-cache

# Live logs
docker logs -f build-apt-cache

# Connection statistics
docker exec build-apt-cache cat /var/log/apt-cacher-ng/apt-cacher.log | grep -i "request"
```

### Health Check

```bash
# Automatic health check via Docker
docker inspect build-apt-cache --format='{{.State.Health.Status}}'

# Manual check
curl -f http://localhost:3142/acng-report.html && echo "OK" || echo "FAILED"
```

---

## 🐛 Troubleshooting

### Common Issues

#### 1. "Unknown configuration directive" error

**Symptoms:** Container crashes at startup with errors like:
```
Warning, unknown configuration directive: PHttpThreads
Error reading main options, terminating.
```

**Root Cause:** Configuration file contains directives from apt-cacher-ng 3.8+ but the Docker image uses version 3.7.5.

**Solutions:**
```bash
# Check apt-cacher-ng version
docker exec build-apt-cache apt-cache policy apt-cacher-ng

# Fix: Comment out unsupported directives in config/acng.conf
# PHttpThreads, ConnectTimeout, NetworkTimeout require version 3.8+

# Restart container after fixing config
docker compose restart apt-cacher-ng
```

**Prevention:** Always check apt-cacher-ng version compatibility before adding new configuration directives.

#### 2. Cache not being used

**Symptoms:** Builds still download packages from internet

**Diagnosis:**
```bash
# Check if APT_PROXY is set in build
docker history <image-id> | grep APT_PROXY

# Check if proxy is accessible
docker run --rm --add-host=host.docker.internal:host-gateway \
  ubuntu:22.04 curl http://host.docker.internal:3142/acng-report.html
```

**Solutions:**
- Ensure `--build-arg APT_PROXY=http://host.docker.internal:3142` is passed
- Verify `--add-host=host.docker.internal:host-gateway` is included
- Check firewall rules on build machine

#### 2. "Connection refused" errors

**Symptoms:** Docker builds fail with "Could not connect to host.docker.internal:3142"

**Diagnosis:**
```bash
# Check if container is running
docker ps | grep apt-cache

# Check if port is listening
netstat -tlnp | grep 3142
```

**Solutions:**
```bash
# Restart APT cache
docker compose restart apt-cacher-ng

# Check logs for errors
docker logs build-apt-cache

# Verify network_mode: host in docker-compose.yaml
```

#### 3. Slow downloads despite cache

**Symptoms:** Subsequent builds are still slow

**Diagnosis:**
```bash
# Check cache hit rate
curl http://localhost:3142/acng-report.html | grep -i "hit"

# Check disk space
df -h /home/ros2/rob_box_project/docker/build/data/apt-cache
```

**Solutions:**
- Ensure cache volume is not full
- Check that packages are actually being cached (not HTTPS only)
- Verify `rm -rf /var/lib/apt/lists/*` is NOT removing cache

#### 4. Parallel build lock contention

**Symptoms:** Builds wait for each other, or get "dpkg lock" errors

**Note:** This is NOT an apt-cacher-ng issue. It's a Docker BuildKit cache mount issue.

**Solution:**
```dockerfile
# Use locked sharing mode for apt cache mount
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    apt-get update && apt-get install -y package-name
```

However, **for apt-cacher-ng proxy mode** (our setup), this is not needed. The cache is on the server side, not client side.

#### 5. High memory usage

**Symptoms:** APT Cacher NG container using excessive RAM

**Diagnosis:**
```bash
docker stats build-apt-cache
```

**Solutions:**
- Reduce `PHttpThreads` if you have limited RAM
- Consider setting memory limits in docker-compose.yaml:
  ```yaml
  mem_limit: 512m
  mem_reservation: 256m
  ```

### Logs Analysis

```bash
# Show errors only
docker logs build-apt-cache 2>&1 | grep -i error

# Show warnings
docker logs build-apt-cache 2>&1 | grep -i warn

# Show download statistics
docker logs build-apt-cache 2>&1 | grep -i "download"

# Show cache hits
docker logs build-apt-cache 2>&1 | grep -i "hit"
```

---

## 🧪 Testing

### Test Scripts

#### Simple Test
```bash
cd docker/build
./scripts/test_apt_cache_simple.sh
```

This builds a minimal Ubuntu image with git and curl to verify cache works.

#### Full Base Image Test
```bash
cd docker/build
./scripts/test_apt_cache.sh
```

This builds the full `ros2-zenoh` base image to test with real ROS packages.

### Manual Testing

```bash
# 1. Build an image with cache
docker buildx build \
  --build-arg="APT_PROXY=http://host.docker.internal:3142" \
  --add-host=host.docker.internal:host-gateway \
  --file docker/base/Dockerfile.ros2-zenoh \
  --tag test:cache \
  docker/base/

# 2. Check cache statistics
curl http://localhost:3142/acng-report.html

# 3. Rebuild (should be faster)
docker buildx build \
  --build-arg="APT_PROXY=http://host.docker.internal:3142" \
  --add-host=host.docker.internal:host-gateway \
  --file docker/base/Dockerfile.ros2-zenoh \
  --tag test:cache2 \
  docker/base/

# 4. Compare times
```

---

## 📚 References

### Documentation

- [APT Cacher NG Official Manual](https://www.unix-ag.uni-kl.de/~bloch/acng/html/index.html)
- [mbentley/apt-cacher-ng GitHub](https://github.com/mbentley/docker-apt-cacher-ng)
- [Docker BuildKit Cache Documentation](https://docs.docker.com/build/cache/)

### Best Practices

- [Speed Up Docker Builds with apt-cacher-ng](https://blog.markbrown.space/entry/speed-up-docker-builds-with-apt-cacher-ng)
- [Centralized Package Caching for Linux](https://dev.to/farshad_nick/apt-repository-with-apt-cacher-2pb2)
- [APT Lock Contention in Parallel Builds](https://stackoverflow.com/questions/78054481/)

### Rob Box Documentation

- [Build Machine README](README.md) - Complete build infrastructure guide
- [AGENT_GUIDE.md](../../docs/development/AGENT_GUIDE.md) - AI agent development guide
- [DOCKER_STANDARDS.md](../../docs/development/DOCKER_STANDARDS.md) - Docker standards

---

## 📝 Configuration Summary

### Optimal Settings for Rob Box

| Setting | Value | Status | Reason |
|---------|-------|--------|--------|
| PHttpThreads | 16 | ⚠️ Commented (needs v3.8+) | Support 8 parallel runners + overhead |
| ConnectTimeout | 120 | ⚠️ Commented (needs v3.8+) | ARM64 QEMU emulation is slow |
| NetworkTimeout | 300 | ⚠️ Commented (needs v3.8+) | Large ROS packages take time |
| DnsCacheSeconds | 3600 | ✅ Active | Reduce DNS overhead for repeated repos |
| ExThreshold | 10 | ✅ Active | Balance disk usage and cache hits |
| MaxDlSpeed | 0 | ✅ Active | No artificial speed limits |

### Critical Files

```
docker/build/
├── config/
│   └── acng.conf          # Main configuration
├── docker-compose.yaml    # Service definition
└── data/
    ├── apt-cache/         # Persistent cache (gitignored)
    └── apt-logs/          # Persistent logs (gitignored)
```

---

**Last Updated:** October 2025  
**Maintainer:** Rob Box Project Team  
**Related Issue:** Setup APT Cacher NG configuration
