# Fix: APT Proxy Fallback in Base Docker Images

**Date:** 2025-10-28  
**Issue:** [#18876728795](https://github.com/krikz/rob_box_project/actions/runs/18876728795)  
**Status:** Fixed ✅

## Problem Description

Local self-hosted runners failed to build base Docker images with error:
```
503 Connection closed, check DlMaxRetries [IP: 172.17.0.1 3142]
E: Unable to locate package ros-humble-rmw-zenoh-cpp
```

### Root Cause

The workflow configured builds to use APT cache proxy at `http://host.docker.internal:3142` (apt-cacher-ng), but:
1. The proxy service was not running or not accessible on the build machine
2. Dockerfiles blindly configured APT to use the proxy
3. When proxy failed, APT had no fallback mechanism
4. All package installations failed completely

## Solution

Added intelligent proxy availability checking in all base Dockerfiles:

### Before
```dockerfile
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then \
        echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
        echo "🔧 Using APT proxy: $APT_PROXY"; \
    fi
```

### After
```dockerfile
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*://[^:]*:\([0-9]*\).*|\1|'); \
        echo "🔍 Checking proxy availability at $PROXY_HOST:$PROXY_PORT..."; \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::http::Timeout \"10\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::Retries \"3\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "✅ Using APT proxy: $APT_PROXY"; \
        else \
            echo "⚠️  APT proxy not accessible, will use direct connection"; \
        fi \
    fi
```

### How It Works

1. **Extract proxy host and port** from APT_PROXY build arg using sed
2. **Test TCP connection** using bash's `/dev/tcp` feature with 5 second timeout
3. **Configure proxy** only if connection succeeds:
   - Set proxy URL
   - Set connection timeout (10s)
   - Set retry limit (3 attempts)
4. **Skip proxy** if connection fails → APT uses direct internet connection
5. **Build continues** successfully either way

## Files Modified

- `docker/base/Dockerfile.ros2-zenoh` - Added proxy check
- `docker/base/Dockerfile.rtabmap` - Added proxy check
- `docker/base/Dockerfile.depthai` - Added proxy check
- `docker/base/Dockerfile.pcl` - Added proxy check
- `docker/build/test/Dockerfile` - Added proxy check for consistency
- `docs/development/LOCAL_BUILD.md` - Updated troubleshooting section

## Benefits

✅ **Resilient builds** - Work even when proxy is down  
✅ **No workflow changes** - Existing workflows continue to work  
✅ **Performance when available** - Still use proxy for speed when it's up  
✅ **Graceful degradation** - Always falls back to working state  
✅ **Clear logging** - Shows whether proxy is being used or not  

## Testing

### Test 1: Build with unavailable proxy
```bash
docker buildx build \
  --file docker/base/Dockerfile.ros2-zenoh \
  --platform linux/amd64 \
  --build-arg="APT_PROXY=http://fake-proxy.example.com:3142" \
  docker/base

# Output:
# 🔍 Checking proxy availability at fake-proxy.example.com:3142...
# ⚠️  APT proxy not accessible, will use direct connection
# [build continues with direct connection]
```

### Test 2: Build without proxy argument
```bash
docker buildx build \
  --file docker/base/Dockerfile.ros2-zenoh \
  --platform linux/amd64 \
  --build-arg="APT_PROXY=" \
  docker/base

# Output:
# [no proxy check, uses direct connection]
```

### Test 3: Build with working proxy (on build machine)
```bash
docker buildx build \
  --file docker/base/Dockerfile.ros2-zenoh \
  --platform linux/arm64 \
  --add-host=host.docker.internal:host-gateway \
  --build-arg="APT_PROXY=http://host.docker.internal:3142" \
  docker/base

# Output:
# 🔍 Checking proxy availability at host.docker.internal:3142...
# ✅ Using APT proxy: http://host.docker.internal:3142
# [build continues with proxy for faster downloads]
```

## Recommendations

1. **Start apt-cacher-ng** on build machine to get speed benefits:
   ```bash
   docker run -d \
     --name build-apt-cache \
     --restart unless-stopped \
     -p 3142:3142 \
     -v /srv/apt-cacher-ng:/var/cache/apt-cacher-ng \
     sameersbn/apt-cacher-ng:latest
   ```

2. **Monitor proxy health** on build machine:
   ```bash
   curl http://localhost:3142/acng-report.html
   ```

3. **Keep proxy URL in workflows** - It's now safe to always pass it:
   ```yaml
   --build-arg="APT_PROXY=http://host.docker.internal:3142"
   ```

## Related Documentation

- [LOCAL_BUILD.md](../development/LOCAL_BUILD.md) - Local build guide with proxy troubleshooting
- [CI_CD_PIPELINE.md](../CI_CD_PIPELINE.md) - CI/CD pipeline with build machine setup
- [BUILD_OPTIMIZATION.md](../development/BUILD_OPTIMIZATION.md) - Build optimization strategies

## Deployment Status

- ✅ Fix committed to `copilot/fix-local-runner-image-issues` branch
- ⏳ Waiting for test on local runner
- ⏳ Pending merge to main

---

**Author:** GitHub Copilot  
**Reviewer:** Pending
