# AprilTag Integration - Quick Reference

## 🎯 What Changed?

AprilTag detection is now **integrated into the OAK-D container** instead of running as a separate container.

## 📊 Before vs After

| Aspect | Before | After |
|--------|--------|-------|
| **Containers** | oak-d + apriltag (2) | oak-d only (1) |
| **RAM Usage** | ~500-600 MB | ~300-400 MB |
| **Topics Published** | Same | Same |
| **Detection Quality** | Unchanged | Unchanged |
| **Maintenance** | 2 containers to manage | 1 container |

## 🔍 Key Changes

### Docker Compose
- ❌ Removed `apriltag:` service section
- ✏️ Updated `oak-d:` comments

### OAK-D Container
- ➕ Added `ros-humble-apriltag-ros` package
- ➕ New launch file: `oakd_with_apriltag.launch.py`
- ✏️ Startup script uses new launch file

### CI/CD
- ❌ Removed apriltag build job
- ✏️ Updated validation scripts

## 🚀 Deployment

### Quick Deploy
```bash
cd ~/rob_box_project/docker/vision
docker-compose down && docker-compose pull oak-d && docker-compose up -d
```

### Verify
```bash
# Check container is running
docker ps | grep oak-d

# Check topics
docker exec oak-d ros2 topic list | grep apriltag

# Expected output: /apriltag/detections
```

## 🔧 Troubleshooting

### Container won't start?
```bash
docker logs oak-d --tail 100
```

### AprilTag not detecting?
```bash
# Check config is mounted
docker exec oak-d ls -la /config/apriltag/

# Check topic is publishing
docker exec oak-d ros2 topic hz /apriltag/detections
```

### Rollback needed?
```bash
git checkout <previous-commit>
cd docker/vision
docker-compose down && docker-compose up -d
```

## 📝 Important Notes

1. **Topic names unchanged** - `/apriltag/detections` still the same
2. **Config unchanged** - `/config/apriltag/apriltag_config.yaml` still used
3. **Main Pi unchanged** - No changes needed on Main Pi
4. **Performance same** - AprilTag detection quality unchanged

## 📚 Full Documentation

- **Migration Guide**: `MIGRATION_APRILTAG_INTEGRATION.md`
- **Architecture**: `docs/architecture/SYSTEM_OVERVIEW.md`
- **Docker Standards**: `docs/development/DOCKER_STANDARDS.md`

## ✅ Success Indicators

After deployment, you should see:

```bash
$ docker ps
CONTAINER ID   IMAGE                                    STATUS
abc123def456   ghcr.io/krikz/rob_box:oak-d-humble-...  Up 2 minutes

$ docker exec oak-d ros2 node list
/camera
/apriltag

$ docker exec oak-d ros2 topic list | grep apriltag
/apriltag/detections
/apriltag/detections/image
```

---

**Need help?** Check `MIGRATION_APRILTAG_INTEGRATION.md` or create a GitHub issue.
