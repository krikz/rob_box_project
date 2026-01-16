# Scripts Directory

Utility scripts organized by purpose for Rob Box development and operations.

## 📂 Structure

### [build/](build/)
Docker image building and CI/CD utilities
- `local-build.sh` - Local Docker builds
- `set-docker-tags.sh` - Manage Docker tags based on git branch
- `patch_*_workflows.sh` - CI/CD workflow patches
- 📖 See [build/README_DOCKER_MANAGEMENT.md](build/README_DOCKER_MANAGEMENT.md) for detailed Docker documentation

### [diagnostics/](diagnostics/)
System diagnostic and troubleshooting tools
- `diagnose_data_flow.sh` - Camera data flow Vision→Main
- `monitor_system.sh` - Robot system monitoring
- `check_lidar_orientation.py` - LiDAR verification
- `diagnose_zenoh_port_conflict.sh` - Zenoh networking

### [maintenance/](maintenance/)
Project maintenance utilities
- `analyze_stl_meshes.py` - 3D model analysis
- `generate_apriltags.py` - AprilTag generation
- `audit_documentation.sh` - Documentation audit
- `reorganize_docs.sh` - Legacy doc reorganization

### [setup/](setup/)
Raspberry Pi setup and configuration
- `setup_main_pi.sh` - Main Pi configuration
- `setup_vision_pi.sh` - Vision Pi configuration
- `install_aliases.sh` - Install development aliases

### [testing/](testing/)
Testing and validation scripts
- `test_docker_*.sh` - Docker build testing
- `validate_dockerfiles.sh` - Dockerfile linting
- `validate_zenoh_*.sh` - Zenoh config validation
- `test_monitoring_config.py` - Monitoring validation

### [utils/](utils/)
Runtime utilities and control
- `start_*/stop_*.sh` - Service control (joystick, GUI, RViz)
- `view_*.sh` - Data visualization (camera, wheels)

### [visualization/](visualization/)
Animation and graphics tools
- `visualize_animations.py` - LED animation previewer
- `generate_animation_frames.py` - Frame generator

## 🚀 Quick Examples

```bash
# Build Docker image locally
./scripts/build/local-build.sh voice-assistant

# Monitor robot system
./scripts/diagnostics/monitor_system.sh

# Setup new Raspberry Pi
./scripts/setup/setup_vision_pi.sh

# Validate configuration
./scripts/testing/validate_dockerfiles.sh

# Start robot GUI
./scripts/utils/start_robot_gui.sh
```

## 📚 Related Documentation

- [Docker Management Guide](build/README_DOCKER_MANAGEMENT.md) - Comprehensive Docker documentation
- [CI/CD Pipeline](../docs/CI_CD_PIPELINE.md)
- [Docker Standards](../docs/development/DOCKER_STANDARDS.md)
- [Agent Guide](../docs/development/AGENT_GUIDE.md)
