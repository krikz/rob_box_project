# GitHub Copilot Instructions for Rob Box Project

## 🎯 Project Overview

**Rob Box (РОББОКС)** is an autonomous wheeled rover project built with ROS 2 Humble on two Raspberry Pi 4 computers. The robot features SLAM capabilities, voice interaction, LED displays, and sensor integration for indoor delivery tasks.

### Key Technologies
- **ROS 2 Humble Hawksbill** - Robot Operating System framework
- **Zenoh DDS** - Optimized middleware for network communication (rmw_zenoh_cpp)
- **RTAB-Map** - RGB-D + 2D LiDAR SLAM system
- **Docker + Docker Compose** - Containerized architecture
- **Python 3.10+** - Primary programming language
- **C++** - Performance-critical components

### Hardware Architecture
- **Main Pi** (10.1.1.10 eth0 / 10.1.1.20 wlan0) - RTAB-Map SLAM, navigation, VESC motor control
- **Vision Pi** (10.1.1.11 eth0 / 10.1.1.21 wlan0) - OAK-D camera, LSLIDAR N10, AprilTag, ReSpeaker mic

---

## 📂 Project Structure

```
rob_box_project/
├── .github/                    # GitHub Actions workflows and CI/CD
│   ├── workflows/              # Build, test, lint workflows
│   └── copilot-instructions.md # This file
├── docker/                     # Docker infrastructure
│   ├── base/                   # Base images (ros2-zenoh, rtabmap, depthai, pcl)
│   ├── main/                   # Main Pi services (rtabmap, nav2, zenoh-router)
│   └── vision/                 # Vision Pi services (oak-d, lslidar, apriltag, voice)
├── src/                        # ROS 2 packages (source code)
│   ├── rob_box_voice/          # Voice assistant (STT, TTS, dialogue, commands)
│   ├── rob_box_perception/     # Perception nodes (health monitor, context aggregator)
│   ├── rob_box_description/    # URDF robot model
│   ├── rob_box_bringup/        # Launch files for complete system
│   ├── rob_box_animations/     # LED animation system
│   ├── led_matrix_driver/      # LED matrix hardware driver
│   └── vesc_nexus/             # VESC motor controller (git submodule)
├── docs/                       # Comprehensive documentation
│   ├── architecture/           # System design, hardware, software architecture
│   ├── development/            # Developer guides (AGENT_GUIDE.md is critical!)
│   ├── guides/                 # User guides (setup, troubleshooting)
│   └── packages/               # Package-specific documentation
└── scripts/                    # Utility scripts
```

---

## 🔑 Critical Files to Review Before Making Changes

### 1. **AGENT_GUIDE.md** ⭐ MUST READ FIRST
**Location:** `docs/development/AGENT_GUIDE.md`
**Purpose:** Comprehensive guide for AI agents with examples, Docker architecture, Zenoh setup, deployment workflows

**Read this before:**
- Any Docker changes
- Modifying configurations
- Adding new services
- Debugging network issues

### 2. **DOCKER_STANDARDS.md** ⭐ REQUIRED FOR DOCKER WORK
**Location:** `docs/development/DOCKER_STANDARDS.md`
**Purpose:** Docker file organization, volume mounting rules, critical anti-patterns

**Key Rules:**
- ❌ **NEVER** `COPY config/` in Dockerfile - configs are mounted via volumes!
- ❌ **NEVER** `COPY scripts/` in Dockerfile - scripts are mounted via volumes!
- ✅ **ONLY** use Dockerfile for `RUN apt-get install`, `RUN git clone`, `RUN colcon build`
- All services use `network_mode: host` and depend on `zenoh-router`

### 3. **PYTHON_STYLE_GUIDE.md**
**Location:** `docs/development/PYTHON_STYLE_GUIDE.md`
**Purpose:** Python coding standards, naming conventions, ROS 2 patterns

**Standards:**
- Use `black` (line length 120) for formatting
- Use `isort` for import sorting
- Use `flake8` for linting
- Type hints required for public APIs
- Docstrings follow Google style

### 4. **CI_CD_PIPELINE.md**
**Location:** `docs/CI_CD_PIPELINE.md`
**Purpose:** GitHub Actions workflows, automatic Docker builds, merge strategies

**Workflow:**
- `feature/*` → auto-merge to `develop` (builds changed services)
- `develop` → auto-merge to `main` (builds ALL services)
- Docker images tagged as `*-humble-latest` (main), `*-humble-dev` (develop)

---

## 🐳 Docker Development Rules

### Dockerfile Best Practices

```dockerfile
# ✅ GOOD - Install packages in Dockerfile
FROM rob_box_base:ros2-zenoh
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-msgs \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# ✅ GOOD - Build ROS packages
WORKDIR /workspace
COPY src/rob_box_voice ./src/rob_box_voice
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select rob_box_voice

# ❌ BAD - DON'T copy configs (they're mounted via volumes!)
COPY config/ /config/  # WRONG! Requires rebuild on config change
```

### docker-compose.yaml Patterns

```yaml
services:
  my_service:
    image: ghcr.io/krikz/rob_box:my-service-humble-latest
    container_name: my_service
    network_mode: host  # ✅ ALWAYS use host networking
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp  # ✅ ALWAYS use Zenoh
      - ZENOH_CONFIG=/config/zenoh_session_config.json5
      - LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib
    volumes:
      - ./config:/config:ro  # ✅ Mount config directory
      - ./scripts:/scripts:ro  # ✅ Mount scripts directory
    depends_on:
      - zenoh-router  # ✅ ALWAYS depend on zenoh-router
    restart: unless-stopped
```

### File Organization

```
docker/vision/
├── docker-compose.yaml           # Service orchestration
├── config/                       # ✅ All configs here (mounted, not copied)
│   ├── zenoh_router_config.json5
│   ├── zenoh_session_config.json5
│   └── oak-d/                    # Service-specific configs
│       └── camera_params.yaml
├── scripts/                      # ✅ All scripts here (mounted, not copied)
│   ├── update_and_restart.sh     # Utility scripts
│   └── oak-d/                    # Service-specific scripts
│       └── start_oak_d.sh
└── oak-d/                        # ✅ ONLY Dockerfile (no configs/scripts)
    └── Dockerfile
```

---

## 🐍 Python Coding Standards

### File Structure Template

```python
#!/usr/bin/env python3
"""
Module docstring: Brief description of what this module does.

Detailed explanation of functionality, usage patterns, and examples.
"""

# Standard library imports
import os
import sys
from typing import List, Optional

# Third-party imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Local imports
from rob_box_voice.utils import audio_utils


class MyNode(Node):
    """
    Brief class description.
    
    Detailed explanation of the node's purpose, functionality, and behavior.
    
    Attributes:
        sample_rate (int): Audio sample rate in Hz
        publisher: ROS 2 publisher for messages
    
    Example:
        >>> node = MyNode()
        >>> rclpy.spin(node)
    """
    
    def __init__(self) -> None:
        """Initialize the node with parameters and publishers."""
        super().__init__('my_node')
        
        # Declare parameters
        self.declare_parameter('sample_rate', 16000)
        self.sample_rate = self.get_parameter('sample_rate').value
        
        # Create publishers
        self.publisher = self.create_publisher(String, '/topic', 10)
        
        self.get_logger().info('Node initialized')
    
    def process_data(self, data: bytes) -> str:
        """
        Process input data and return result.
        
        Args:
            data: Raw bytes to process
            
        Returns:
            Processed string result
            
        Raises:
            ValueError: If data is empty
        """
        if not data:
            raise ValueError("Data cannot be empty")
        return data.decode('utf-8')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = MyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Naming Conventions

- **Variables/functions:** `snake_case` (e.g., `sample_rate`, `process_audio()`)
- **Classes:** `PascalCase` (e.g., `AudioNode`, `ReSpeakerInterface`)
- **Constants:** `SCREAMING_SNAKE_CASE` (e.g., `MAX_BUFFER_SIZE`, `DEFAULT_RATE`)
- **ROS 2 node names:** `snake_case` (e.g., `'audio_node'`, `'dialogue_node'`)
- **Private attributes:** `_protected_var`, `__private_var`

### Logging (DO NOT use print())

```python
# ✅ GOOD - Use ROS 2 logger
self.get_logger().debug('Detailed debug information')
self.get_logger().info('Normal operation message')
self.get_logger().warn('Warning about potential issue')
self.get_logger().error('Error occurred, but recoverable')
self.get_logger().fatal('Critical error, cannot continue')

# ❌ BAD - Don't use print()
print('This is a message')  # NEVER do this in ROS nodes!
```

---

## 🤖 ROS 2 Specific Patterns

### Parameter Declaration

```python
# ✅ GOOD - Declare with defaults, then get values
self.declare_parameter('sample_rate', 16000)
self.declare_parameter('device_name', 'default_device')
self.sample_rate = self.get_parameter('sample_rate').value
self.device_name = self.get_parameter('device_name').value

# Add parameter callback for dynamic reconfiguration
self.add_on_set_parameters_callback(self.parameters_callback)
```

### Topic Naming Convention

```python
# ROS 2 topic naming follows pattern: /<namespace>/<topic_name>
# ✅ GOOD - Clear, hierarchical naming
'/audio/audio'              # Raw audio data
'/audio/vad'                # Voice Activity Detection
'/audio/speech_detected'    # Speech detection event
'/dialogue/text'            # Dialogue text output
'/led/animation'            # LED animation commands

# ❌ BAD - Unclear or flat naming
'/audio'                    # Too generic
'/my_topic'                 # Not descriptive
'/AudioData'                # Wrong case
```

### Quality of Service (QoS)

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For real-time sensor data (ok to drop old messages)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

# For important events (must not lose messages)
event_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)
```

---

## 🌐 Networking & Zenoh

### IP Address Convention

- **Ethernet (eth0):** Used for data transfer between Pis
  - Main Pi: `10.1.1.10`
  - Vision Pi: `10.1.1.11`
- **WiFi (wlan0):** Used for SSH access and management
  - Main Pi: `10.1.1.20`
  - Vision Pi: `10.1.1.21`

### Zenoh Configuration

All ROS 2 nodes use Zenoh middleware with these environment variables:

```yaml
environment:
  - RMW_IMPLEMENTATION=rmw_zenoh_cpp
  - ZENOH_CONFIG=/config/zenoh_session_config.json5
  - ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  - ZENOH_ROUTER_CHECK_ATTEMPTS=10
  - LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib
```

**Important:** Always use Ethernet IPs (10.1.1.10, 10.1.1.11) in Zenoh router configurations, NOT WiFi IPs!

---

## 📊 Monitoring System

### Overview

Rob Box uses a lightweight monitoring stack for observing system health and logs:

- **Grafana** (port 3000) - Web dashboard for visualization
- **Prometheus** (port 9090) - Metrics collection
- **Loki** (port 3100) - Log aggregation
- **cAdvisor** (port 8080) - Container metrics on both Pis
- **Promtail** (port 9080) - Log forwarding

### Quick Start

```bash
# Enable monitoring
cd ~/rob_box_project/docker/main && ./scripts/enable_monitoring.sh
cd ~/rob_box_project/docker/vision && ./scripts/enable_monitoring.sh

# Access Grafana
http://10.1.1.10:3000  (admin/robbox)

# Disable monitoring
cd ~/rob_box_project/docker/main && ./scripts/disable_monitoring.sh
cd ~/rob_box_project/docker/vision && ./scripts/disable_monitoring.sh
```

### Resource Usage

- **Main Pi:** ~320MB RAM (idle), ~570MB RAM (active)
- **Vision Pi:** ~70MB RAM (idle), ~120MB RAM (active)

### When to Use Monitoring

**Enable when:**
- ✅ Debugging performance issues
- ✅ Load testing
- ✅ Configuring new features
- ✅ Remote robot operation

**Disable when:**
- ✅ Autonomous operation (save resources)
- ✅ Maximum performance needed
- ✅ Battery conservation required

### Documentation

- [Monitoring System Guide](../docs/guides/MONITORING_SYSTEM.md) - Complete documentation
- [Quick Reference](../docs/MONITORING_QUICK_REF.md) - Command reference

---

## 🔐 Security & Secrets Management

### API Keys & Credentials

**CRITICAL:** Never commit API keys or secrets to git!

```bash
# ✅ GOOD - Use .env.secrets file (gitignored)
docker/vision/.env.secrets:
  DEEPSEEK_API_KEY=your_key_here
  YANDEX_API_KEY=your_key_here
  YANDEX_FOLDER_ID=your_folder_here

# In docker-compose.yaml
services:
  voice-assistant:
    env_file:
      - .env.secrets  # Load secrets from file

# ❌ BAD - Hardcoded secrets
environment:
  - DEEPSEEK_API_KEY=sk-1234567890abcdef  # NEVER DO THIS!
```

### .gitignore Entries

Ensure these are in `.gitignore`:
```
.env.secrets
*.env.local
*_secrets.yaml
```

---

## 🧪 Testing & Quality Assurance

### Pre-commit Hooks

The project uses pre-commit hooks for automated quality checks:

```bash
# Install once
pip install pre-commit
pre-commit install

# Manually run all checks
pre-commit run --all-files
```

### Linting Tools

```bash
# Format Python code (automatic)
black src/rob_box_voice/ --line-length=120

# Sort imports (automatic)
isort src/rob_box_voice/ --profile black

# Check code quality
flake8 src/rob_box_voice/ --max-line-length=120

# Check YAML files
yamllint -c .yamllint.yml docker/
```

### Running Tests

```bash
# ROS 2 package tests
cd /workspace
colcon test --packages-select rob_box_voice
colcon test-result --verbose

# Python unit tests (if pytest is used)
pytest src/rob_box_voice/test/
```

---

## 🚀 Development Workflow

### Adding a New Feature

1. **Create feature branch from develop:**
   ```bash
   git checkout develop
   git pull origin develop
   git checkout -b feature/my-awesome-feature
   ```

2. **Make changes following standards:**
   - Read relevant documentation (AGENT_GUIDE.md, DOCKER_STANDARDS.md)
   - Follow Python style guide
   - Update documentation if needed
   - Add tests for new functionality

3. **Test locally:**
   ```bash
   # Build Docker image (if needed)
   cd docker/vision/oak-d
   docker build -t test:local .
   
   # Run linters
   black --check src/
   flake8 src/
   yamllint docker/
   ```

4. **Commit and push:**
   ```bash
   git add .
   git commit -m "feat: add awesome feature for navigation"
   git push origin feature/my-awesome-feature
   ```

5. **GitHub Actions automatically:**
   - Builds changed services
   - Runs linters and tests
   - Creates Docker images with tag `*-humble-test`
   - Auto-merges to `develop` if successful

### Commit Message Convention

Use [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation only
- `style`: Code style (formatting, no logic change)
- `refactor`: Code refactoring
- `perf`: Performance improvement
- `test`: Adding tests
- `chore`: Maintenance (dependencies, CI/CD)

**Examples:**
```
feat(voice): add command node for navigation integration

Implements command_node.py to process navigation commands
from dialogue system. Publishes to /cmd_vel for robot movement.

Closes #42

---

fix(docker): add missing nav2-msgs dependency to voice-assistant

Voice assistant command_node requires nav2_msgs package.
Added to apt-get install in Dockerfile.

---

docs(readme): update hardware specifications

Added ReSpeaker Mic Array v2.0 and ESP32 sensor hub details.
```

---

## 🔍 Debugging & Troubleshooting

### Accessing Raspberry Pi via SSH

**IMPORTANT:** Always use `sshpass` for automated SSH commands (password is 'open'):

```bash
# Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21

# Main Pi
sshpass -p 'open' ssh ros2@10.1.20

# Execute remote command without interactive login
sshpass -p 'open' ssh ros2@10.1.1.21 'docker ps'
```

### Docker Container Debugging

```bash
# Check container status
docker ps

# View logs
docker logs oak-d --tail 100

# Follow logs in real-time
docker logs -f rtabmap

# Execute command inside container
docker exec -it oak-d bash

# Check ROS 2 topics inside container
docker exec oak-d ros2 topic list
docker exec oak-d ros2 topic echo /camera/rgb/image_raw
```

### Monitoring Scripts

```bash
# Vision Pi - General system monitoring
cd ~/rob_box_project/docker
./monitor_system.sh

# Vision Pi - Real-time camera monitoring
cd ~/rob_box_project/docker/vision
./realtime_monitor.sh

# Vision Pi - Camera diagnostics
cd ~/rob_box_project/docker/vision
./diagnose.sh

# Local machine - Full data flow diagnostics
cd /path/to/rob_box_project/docker
wsl bash ./diagnose_data_flow.sh
```

### Common Issues & Solutions

**Issue:** "Did not receive data since 5 seconds" in RTAB-Map
**Solution:** Run `diagnose_data_flow.sh` to check Vision Pi → Main Pi communication

**Issue:** Docker image rebuild takes 5-10 minutes after config change
**Solution:** Configs should be in `docker/*/config/` and mounted via volumes, NOT copied in Dockerfile

**Issue:** ModuleNotFoundError for ROS package
**Solution:** Add missing package to Dockerfile: `RUN apt-get install -y ros-humble-<package-name>`

**Issue:** Zenoh connection issues
**Solution:** Check that all services depend on `zenoh-router` and use correct session config

---

## 📝 Documentation Standards

### When to Update Documentation

Update documentation when you:
- Add new features or services
- Change Docker architecture
- Modify configuration files
- Fix significant bugs
- Change deployment procedures

### Documentation Structure

```
docs/
├── architecture/          # System design, hardware specs
├── development/           # Developer guides (this is where AI guides live!)
├── guides/                # User guides (setup, operation)
├── packages/              # Package-specific documentation
└── reports/               # Investigation reports, fixes
```

### Writing Good Documentation

**DO:**
- Use clear, concise language
- Include code examples
- Add command-line examples with expected output
- Use emojis for visual hierarchy (⭐ ✅ ❌ 🔧 📝)
- Link to related documentation

**DON'T:**
- Write walls of text without examples
- Use vague language ("might work", "probably")
- Skip error cases and edge conditions
- Forget to update README.md if it's a major change

---

## 🎯 Common Tasks Quick Reference

### Deploy Updated Code to Raspberry Pi

```bash
# Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && ./update_and_restart.sh'

# Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20 \
  'cd ~/rob_box_project/docker/main && ./update_and_restart.sh'
```

### Add New ROS 2 Package

```bash
# 1. Create package structure
cd src/
ros2 pkg create --build-type ament_python my_package \
  --dependencies rclpy std_msgs

# 2. Add to Dockerfile
RUN apt-get install -y ros-humble-my-dependency

# 3. Build in Dockerfile
COPY src/my_package ./src/my_package
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select my_package

# 4. Add to docker-compose.yaml with proper volumes
```

### Add New Docker Service

Follow the workflow in `DOCKER_STANDARDS.md` section "Workflow для добавления нового сервиса"

---

## 🔗 Related Documentation Links

**Critical reads:**
- [AGENT_GUIDE.md](../docs/development/AGENT_GUIDE.md) - Comprehensive AI agent guide
- [DOCKER_STANDARDS.md](../docs/development/DOCKER_STANDARDS.md) - Docker organization rules
- [PYTHON_STYLE_GUIDE.md](../docs/development/PYTHON_STYLE_GUIDE.md) - Python coding standards

**Architecture:**
- [SYSTEM_OVERVIEW.md](../docs/architecture/SYSTEM_OVERVIEW.md) - Complete system architecture
- [HARDWARE.md](../docs/architecture/HARDWARE.md) - Hardware specifications
- [SOFTWARE.md](../docs/architecture/SOFTWARE.md) - Software stack details

**Development:**
- [BUILD_OPTIMIZATION.md](../docs/development/BUILD_OPTIMIZATION.md) - Docker build optimization
- [LINTING_GUIDE.md](../docs/development/LINTING_GUIDE.md) - Linting setup and usage
- [TESTING_GUIDE.md](../docs/development/TESTING_GUIDE.md) - Testing best practices

**Operations:**
- [CI_CD_PIPELINE.md](../docs/CI_CD_PIPELINE.md) - GitHub Actions workflows
- [TROUBLESHOOTING.md](../docs/guides/TROUBLESHOOTING.md) - Common issues and solutions

---

## 💡 Tips for Effective AI Assistance

### Before Writing Code

1. **Read AGENT_GUIDE.md** - It contains critical project context
2. **Check existing patterns** - Look at similar files in the codebase
3. **Review recent commits** - Understand recent changes: `git log -10 --oneline`
4. **Check documentation** - Especially for Docker and Python standards

### When Suggesting Changes

1. **Be specific** - Reference exact file paths and line numbers
2. **Show examples** - Include before/after code snippets
3. **Explain why** - Not just what to change, but why it's better
4. **Consider impact** - Will this require Docker rebuild? Config changes?

### Error Handling

1. **Quote exact errors** - Copy full error messages with stack traces
2. **Provide context** - What were you trying to do? Which Pi? Which container?
3. **Show what you tried** - List debugging steps already attempted
4. **Check logs first** - `docker logs <container>` often reveals the issue

### Testing Suggestions

1. **Test locally first** - Don't rely on CI/CD for basic testing
2. **Check all affected systems** - Changes might impact both Pis
3. **Verify with monitoring tools** - Use provided scripts to validate
4. **Watch resource usage** - Monitor CPU, memory, network on Raspberry Pi

---

## 📚 Project-Specific Glossary

**Terms you'll encounter:**

- **Vision Pi / Main Pi** - The two Raspberry Pi 4 computers in the robot
- **Zenoh** - Zero Overhead Network Protocol - optimized DDS middleware
- **RTAB-Map** - Real-Time Appearance-Based Mapping - SLAM system
- **OAK-D** - OpenCV AI Kit with Depth - stereo camera by Luxonis
- **LSLIDAR N10** - 2D LiDAR scanner for mapping
- **ReSpeaker** - USB microphone array for voice assistant
- **VESC** - Vedder Electronic Speed Controller - motor controllers
- **AprilTag** - Fiducial marker system for localization
- **VAD** - Voice Activity Detection
- **DoA** - Direction of Arrival (sound source direction)
- **STT** - Speech-to-Text
- **TTS** - Text-to-Speech
- **QoS** - Quality of Service (ROS 2 communication reliability)

---

**Last Updated:** October 2025
**Maintained By:** Rob Box Project Team
**For Questions:** See existing Issues or create new one in GitHub
