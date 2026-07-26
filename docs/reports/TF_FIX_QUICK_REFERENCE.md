# TF Transformation Fix - Quick Reference Card

## 🎯 What Was Fixed

**Problem:** RTAB-Map couldn't receive TF transformations → SLAM broken  
**Cause:** `robot-state-publisher` not using Zenoh wrapper like other services  
**Solution:** Added wrapper + environment variable to match other services

---

## 📦 Files Changed

1. **docker/main/docker-compose.yaml** (2 changes)
   - ➕ Added `ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5`
   - 🔄 Changed command to use `ros_with_namespace.sh` wrapper

2. **docker/main/scripts/robot_state_publisher/start_robot_state_publisher.sh** (simplified)
   - ➖ Removed redundant wrapper call (now in docker-compose)

3. **docs/reports/TF_TRANSFORMATION_FIX.md** (new)
   - 📚 Full documentation with testing instructions

---

## 🚀 Quick Deploy & Test

### 1. Deploy to Main Pi
```bash
sshpass -p 'open' ssh ros2@10.1.1.20 \
  'cd ~/rob_box_project/docker/main && ./update_and_restart.sh'
```

### 2. Verify Service is Running
```bash
docker ps | grep robot-state-publisher
```
Expected: ✅ Container running

### 3. Check Zenoh Config
```bash
docker exec robot-state-publisher env | grep ZENOH_SESSION_CONFIG_URI
```
Expected: ✅ `/tmp/zenoh_session_config.json5`

### 4. Verify TF Topics
```bash
docker exec robot-state-publisher bash -c \
  'source /opt/ros/kilted/setup.bash && ros2 topic list | grep tf'
```
Expected: ✅ `/tf` and `/tf_static` listed

### 5. Check TF Content
```bash
docker exec robot-state-publisher bash -c \
  'source /opt/ros/kilted/setup.bash && ros2 topic echo /tf_static --once'
```
Expected: ✅ Transformations including `base_link → lslidar_n10`

### 6. Verify RTAB-Map
```bash
docker logs rtabmap --tail 100 | grep -i "transform\|odometry"
```
Expected:
- ✅ "Odometry update rate: ~10Hz"
- ❌ NO "does not exist" errors

---

## 🔍 Troubleshooting

### Issue: Container not starting
```bash
docker logs robot-state-publisher
```
Look for: Script errors, missing files

### Issue: TF topics empty
```bash
docker exec robot-state-publisher bash -c \
  'ps aux | grep robot_state_publisher'
```
Verify: Process is running

### Issue: RTAB-Map still showing errors
```bash
# Check if Zenoh router is working
docker ps | grep zenoh-router

# Check if TF is being published over Zenoh
curl http://localhost:8000/@/local/subscriber | grep tf
```

---

## ✅ Success Indicators

| Check | Command | Expected Result |
|-------|---------|----------------|
| Service Running | `docker ps \| grep robot-state-publisher` | Container UP |
| Zenoh Config | `docker exec ... env \| grep ZENOH` | `/tmp/zenoh_session_config.json5` |
| TF Topics | `ros2 topic list \| grep tf` | `/tf`, `/tf_static` |
| TF Data | `ros2 topic echo /tf_static --once` | Transform messages |
| RTAB-Map OK | `docker logs rtabmap \| grep "Odometry update"` | ~10Hz rate |

---

## 📞 Support

- Full docs: `docs/reports/TF_TRANSFORMATION_FIX.md`
- GitHub Issue: [Link to PR]
- Contact: Robot team

**Status:** ✅ Ready for deployment testing
