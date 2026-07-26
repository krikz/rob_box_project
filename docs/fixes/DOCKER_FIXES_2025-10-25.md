# Docker Container Fixes - 2025-10-25

## Summary

Fixed 3 Docker container issues identified in diagnostic report:
1. **robot-state-publisher** (Main Pi) - Critical: Added debug logging for crash diagnosis
2. **voice-assistant** (Vision Pi) - Non-critical: Added proper healthcheck
3. **led-matrix** (Vision Pi) - Non-critical: Added proper healthcheck

## Problem Analysis

### 1. robot-state-publisher - Critical Issue

**Symptom**: Container constantly restarting with exit code 250

**Error Message**:
```
Failed to read config from /tmp/zenoh_session_config.json5: No such file or directory (os error 2)
[ERROR] [rmw_zenoh_cpp]: Invalid configuration file /tmp/zenoh_session_config.json5
terminate called after throwing an instance of 'rclcpp::exceptions::RCLBadAlloc'
```

**Root Cause**: The `ros_with_namespace.sh` script was failing to create `/tmp/zenoh_session_config.json5`, but there was no error handling or debug logging to identify why.

**Possible Scenarios**:
1. `/config/zenoh_session_config.json5` not mounted correctly
2. `/tmp` directory not writable
3. Copy operation failing silently

**Solution**: Enhanced `ros_with_namespace.sh` with comprehensive error checking:
- Check source file exists before copying
- Check /tmp is writable
- Verify file was created successfully
- Provide detailed error messages with directory listings
- Exit early with clear status codes

### 2. voice-assistant & led-matrix - Healthcheck Issues

**Symptom**: Containers running correctly but showing "unhealthy" status

**Error Message**:
```
"ExitCode": 1, "Output": "/bin/sh: 1: ros2: not found"
```

**Root Cause**: Healthcheck commands were trying to use `ros2` command without sourcing ROS 2 environment first.

**Solution**: Added process-based healthchecks following existing patterns in the codebase:
- Use `pgrep -f` to check if processes are running
- No need to source ROS environment
- Simple, fast, and reliable
- Consistent with other services (twist-mux, micro-ros-agent, ros2-control)

## Changes Made

### Files Modified

1. `docker/main/scripts/ros_with_namespace.sh` (+23 lines)
2. `docker/vision/scripts/ros_with_namespace.sh` (+23 lines)
3. `docker/vision/docker-compose.yaml` (+12 lines)

**Total**: 3 files, 58 lines added, 0 lines removed

### Detailed Changes

#### 1. ros_with_namespace.sh (Main Pi & Vision Pi)

**Before**:
```bash
GENERATED_CONFIG="/tmp/zenoh_session_config.json5"
cp /config/zenoh_session_config.json5 "$GENERATED_CONFIG"
sed -i "s|// namespace: \"my/namespace\"|namespace: \"robots/$ROBOT_ID\"|g" "$GENERATED_CONFIG"
```

**After**:
```bash
GENERATED_CONFIG="/tmp/zenoh_session_config.json5"

# Debug: проверяем существование source файла
if [ ! -f "/config/zenoh_session_config.json5" ]; then
    echo "❌ ОШИБКА: /config/zenoh_session_config.json5 не найден!"
    echo "📁 Содержимое /config:"
    ls -la /config/ || echo "Директория /config не существует"
    exit 1
fi

# Debug: проверяем доступность /tmp для записи
if [ ! -w "/tmp" ]; then
    echo "❌ ОШИБКА: Директория /tmp недоступна для записи!"
    exit 1
fi

# Копируем template конфиг
echo "📋 Копируем /config/zenoh_session_config.json5 -> $GENERATED_CONFIG"
cp /config/zenoh_session_config.json5 "$GENERATED_CONFIG"

# Проверяем что файл скопирован
if [ ! -f "$GENERATED_CONFIG" ]; then
    echo "❌ ОШИБКА: Не удалось создать $GENERATED_CONFIG"
    exit 1
fi

sed -i "s|// namespace: \"my/namespace\"|namespace: \"robots/$ROBOT_ID\"|g" "$GENERATED_CONFIG"
```

**Benefits**:
- Clear error messages with context
- Early failure detection
- Easy troubleshooting via container logs
- No silent failures

#### 2. docker-compose.yaml (Vision Pi)

**voice-assistant** - Added healthcheck:
```yaml
healthcheck:
  test: ["CMD-SHELL", "pgrep -f 'python3.*voice' || exit 1"]
  interval: 15s
  timeout: 5s
  start_period: 20s
  retries: 3
```

**led-matrix** - Added healthcheck:
```yaml
healthcheck:
  test: ["CMD-SHELL", "pgrep -f led_matrix || exit 1"]
  interval: 10s
  timeout: 5s
  start_period: 15s
  retries: 3
```

**Pattern Explanation**:
- `pgrep -f <pattern>`: Search for process by pattern
- `|| exit 1`: Return error if process not found
- `interval`: How often to check
- `timeout`: Max time for check to complete
- `start_period`: Grace period at startup
- `retries`: Failures before marking unhealthy

## Testing

### Validation Performed

1. ✅ YAML syntax validation with `yamllint`
2. ✅ Bash script syntax validation with `bash -n`
3. ✅ Healthcheck patterns match existing containers
4. ✅ No breaking changes
5. ✅ Minimal scope (only 3 files changed)

### Next Steps (Requires Raspberry Pi Hardware)

1. Deploy changes to Main Pi and Vision Pi
2. Monitor robot-state-publisher logs for clear diagnostics
3. Verify voice-assistant shows "healthy" status
4. Verify led-matrix shows "healthy" status
5. Confirm robot-state-publisher starts successfully

### Expected Outcomes

**If robot-state-publisher still fails**:
- Will now see clear error message indicating the problem
- Will see directory listing to verify mount configuration
- Can diagnose root cause from logs

**If voice-assistant/led-matrix**:
- Should now show "healthy" status
- Can verify with `docker ps` showing "(healthy)" instead of "(unhealthy)"

## Design Decisions

### Why Process-Based Healthchecks?

**Considered Options**:
1. Source ROS environment in healthcheck
   ```yaml
   test: ["CMD-SHELL", "source /opt/ros/kilted/setup.bash && ros2 node list | grep voice_assistant || exit 1"]
   ```
   ❌ Complex, slow, requires full ROS environment

2. HTTP endpoint check
   ```yaml
   test: ["CMD-SHELL", "curl localhost:8080/health || exit 1"]
   ```
   ❌ Would require adding health endpoint to services

3. **Process-based check** (CHOSEN)
   ```yaml
   test: ["CMD-SHELL", "pgrep -f led_matrix || exit 1"]
   ```
   ✅ Simple, fast, reliable
   ✅ Matches existing pattern in codebase
   ✅ No additional dependencies

### Why Debug Logging in ros_with_namespace.sh?

**Alternative**: Add debug in Dockerfile
❌ Would require Docker image rebuild
❌ Harder to update/modify

**Chosen**: Add in mounted script
✅ Can be updated without rebuild
✅ No container restart needed
✅ Easy to modify for additional diagnostics

## References

- **Original Issue**: Diagnostic report identifying 3 container problems
- **Docker Standards**: `docs/development/DOCKER_STANDARDS.md`
- **Agent Guide**: `docs/development/AGENT_GUIDE.md`
- **Existing Patterns**: Other services in docker-compose.yaml (twist-mux, micro-ros-agent)

## Commit History

1. `Initial plan` - Analysis and planning
2. `fix: Add debug logging and healthchecks for Docker containers` - Implementation
3. `docs: Update PR description with complete summary` - Documentation

---

**Date**: 2025-10-25  
**Author**: GitHub Copilot  
**Status**: Ready for testing on hardware
