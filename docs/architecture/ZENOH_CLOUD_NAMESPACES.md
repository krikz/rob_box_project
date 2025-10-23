# Zenoh Cloud Connectivity & Namespace Configuration

## 📋 Table of Contents
- [Overview](#overview)
- [Zenoh Namespace Feature](#zenoh-namespace-feature)
- [Current Implementation](#current-implementation)
- [Cloud Topology](#cloud-topology)
- [Configuration Details](#configuration-details)
- [How It Works](#how-it-works)
- [Testing & Validation](#testing--validation)
- [Troubleshooting](#troubleshooting)

---

## Overview

This document explains how Rob Box robot integrates with the cloud Zenoh router at `zenoh.robbox.online:7447` using Zenoh's **namespace** feature for robot isolation.

### Key Concepts

1. **Zenoh Namespace**: A prefix automatically added to all key expressions (topics)
2. **Robot ID**: Unique identifier for each robot (e.g., `RBXU100001`)
3. **Cloud Router**: Central Zenoh router at `zenoh.robbox.online:7447`
4. **Topic Isolation**: Each robot's topics are prefixed with `robots/{ROBOT_ID}/`

---

## Zenoh Namespace Feature

### What is a Zenoh Namespace?

From [Zenoh documentation](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5):

> **Namespace prefix.**
> If specified, all outgoing key expressions will be automatically prefixed with specified string,
> and all incoming key expressions will be stripped of specified prefix.
> 
> The namespace prefix should satisfy all key expression constraints and additionally it can not contain wild characters ('*').
> 
> Namespace is applied to the session.
> E.g. if session has a namespace of "1" then `session.put("my/keyexpr", my_message)` will put a message into `1/my/keyexpr`. Same applies to all other operations within this session.

### How It Works

```
Without namespace:
  Publish:   /cmd_vel       →  Zenoh:  /cmd_vel
  Subscribe: /odom          →  Zenoh:  /odom

With namespace "robots/RBXU100001":
  Publish:   /cmd_vel       →  Zenoh:  robots/RBXU100001/cmd_vel
  Subscribe: /odom          →  Zenoh:  robots/RBXU100001/odom
  
  BUT from robot's perspective, topics are still /cmd_vel and /odom!
  The namespace is transparent to ROS 2 nodes.
```

**Benefits:**
- ✅ Complete topic isolation between robots
- ✅ Transparent to ROS 2 code (no changes needed)
- ✅ Easy cloud monitoring: subscribe to `robots/**`
- ✅ Easy per-robot monitoring: subscribe to `robots/RBXU100001/**`
- ✅ No conflicts with ROS_DOMAIN_ID

---

## Current Implementation

### Robot ID Configuration

**Location:** `docker/vision/.env` and `docker/main/.env`

```bash
# Robot ID for namespace isolation
ROBOT_ID=RBXU100001
```

**Format Requirements:**
- Alphanumeric characters only
- No special characters except underscores
- Unique per robot
- Recommended format: `RBXU` + 6 digits (e.g., `RBXU100001`, `RBXU100002`)

### Namespace Wrapper Script

**Location:** `docker/vision/scripts/ros_with_namespace.sh`

This script:
1. Reads `ROBOT_ID` environment variable
2. Copies base session config to `/tmp/zenoh_session_config.json5`
3. Uncomments and sets `namespace: "robots/{ROBOT_ID}"`
4. Sets `ZENOH_SESSION_CONFIG_URI` to generated config
5. Launches the ROS node

**Key Code:**
```bash
# Раскомментируем и заменяем namespace
sed -i "s|// namespace: \"my/namespace\"|namespace: \"robots/$ROBOT_ID\"|g" "$GENERATED_CONFIG"

# Обновляем ZENOH_SESSION_CONFIG_URI на сгенерированный файл
export ZENOH_SESSION_CONFIG_URI="$GENERATED_CONFIG"
```

### Docker Compose Integration

**Location:** `docker/vision/docker-compose.yaml`

All ROS services use the wrapper:

```yaml
oak-d:
  environment:
    - ROBOT_ID=${ROBOT_ID}  # Для генерации namespace
  command: ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_oak_d.sh"]

lslidar:
  environment:
    - ROBOT_ID=${ROBOT_ID}
  command: ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_lslidar.sh"]

# ... similar for apriltag, led-matrix-driver, voice-assistant, perception
```

---

## Cloud Topology

### Network Architecture

```
                    ┌──────────────────────────┐
                    │  zenoh.robbox.online     │ (Cloud)
                    │       :7447              │
                    │  Storage: robots/**      │
                    └────────────┬─────────────┘
                                 │
                                 │ TCP/TLS
                                 │
                   ┌─────────────▼──────────────┐
                   │    Main Pi Zenoh Router    │
                   │      (10.1.1.10:7447)      │
                   │      mode: router          │
                   └──────┬──────────────┬──────┘
                          │              │
                   TCP/IP │              │ TCP/IP
                          │              │
          ┌───────────────▼──┐      ┌───▼──────────────────┐
          │ Vision Pi Zenoh  │      │  Main Pi ROS Nodes   │
          │ Router           │      │  (via rmw_zenoh_cpp) │
          │ (10.1.1.11:7447) │      │  - rtabmap           │
          │ mode: router     │      │  - nav2              │
          └──────┬───────────┘      │  - twist_mux         │
                 │                  └──────────────────────┘
          ┌──────▼────────────┐
          │  Vision Pi Nodes  │
          │  (via rmw_zenoh)  │
          │  - oak-d          │
          │  - lslidar        │
          │  - apriltag       │
          │  - voice          │
          └───────────────────┘
```

### Topic Flow Example

For robot `RBXU100001` publishing `/cmd_vel`:

```
1. ROS Node publishes:     /cmd_vel
2. rmw_zenoh adds prefix:  robots/RBXU100001/cmd_vel
3. Zenoh Session sends:    robots/RBXU100001/cmd_vel
4. Local router forwards:  robots/RBXU100001/cmd_vel
5. Main Pi router sends:   robots/RBXU100001/cmd_vel → zenoh.robbox.online
6. Cloud storage stores:   robots/RBXU100001/cmd_vel
```

---

## Configuration Details

### Vision Pi Router Config

**Location:** `docker/vision/config/zenoh_router_config.json5`

```json5
{
  mode: "router",
  connect: {
    endpoints: [
      "tcp/10.1.1.10:7447"  // Connect to Main Pi router
    ],
  },
  listen: {
    endpoints: ["tcp/[::]:7447"],
  },
  scouting: {
    multicast: { enabled: false }
  }
}
```

**Purpose:** Routes Vision Pi topics to Main Pi

### Main Pi Router Config

**Location:** `docker/main/config/zenoh_router_config.json5`

```json5
{
  mode: "router",
  connect: {
    endpoints: [
      "tcp/zenoh.robbox.online:7447"  // Connect to cloud
    ],
  },
  listen: {
    endpoints: ["tcp/[::]:7447"],
  },
  plugins: {
    rest: {
      http_port: 8000,
    },
    storage_manager: {
      storages: {
        robot_data: {
          key_expr: "robots/**",  // Store all robot data
          volume: {
            id: "memory",
          },
        },
      },
    },
  },
}
```

**Purpose:** Routes all robot topics to cloud and stores them locally

### Session Config (Both Pis)

**Location:** `docker/*/config/zenoh_session_config.json5`

Base config (namespace commented out by default):

```json5
{
  mode: "peer",
  connect: {
    endpoints: ["tcp/localhost:7447"],  // Connect to local router
  },
  listen: {
    endpoints: ["tcp/localhost:0"],  // Random port
  },
  
  // Namespace is commented by default
  // It's uncommented by ros_with_namespace.sh wrapper
  // namespace: "my/namespace",
}
```

**Generated config** at runtime in `/tmp/zenoh_session_config.json5`:

```json5
{
  mode: "peer",
  connect: {
    endpoints: ["tcp/localhost:7447"],
  },
  listen: {
    endpoints: ["tcp/localhost:0"],
  },
  
  // Uncommented and set by ros_with_namespace.sh
  namespace: "robots/RBXU100001",
}
```

---

## How It Works

### Step-by-Step Flow

#### 1. Container Startup

When a Vision Pi service starts:

```bash
docker-compose up oak-d
```

Docker executes:

```yaml
command: ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_oak_d.sh"]
environment:
  - ROBOT_ID=RBXU100001
  - ZENOH_CONFIG=/config/zenoh_session_config.json5
```

#### 2. Namespace Configuration

`ros_with_namespace.sh` executes:

```bash
#!/bin/bash
ROBOT_ID=RBXU100001  # From environment

# Generate config with namespace
GENERATED_CONFIG="/tmp/zenoh_session_config.json5"
cp /config/zenoh_session_config.json5 "$GENERATED_CONFIG"
sed -i "s|// namespace: \"my/namespace\"|namespace: \"robots/$ROBOT_ID\"|g" "$GENERATED_CONFIG"

# Point to generated config
export ZENOH_SESSION_CONFIG_URI="$GENERATED_CONFIG"

# Launch node
exec /scripts/start_oak_d.sh
```

#### 3. ROS Node Launch

OAK-D node starts with:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5

ros2 launch depthai_ros_driver camera.launch.py
```

#### 4. Topic Publication

When OAK-D publishes `/camera/rgb/image_raw`:

```
ROS Node → rmw_zenoh → Zenoh Session (with namespace) → Local Router → Main Router → Cloud
```

Zenoh sees: `robots/RBXU100001/camera/rgb/image_raw`

#### 5. Cloud Subscription

Cloud can subscribe to:

```bash
# All robots, all topics
robots/**

# Specific robot, all topics
robots/RBXU100001/**

# Specific robot, specific topic
robots/RBXU100001/camera/rgb/image_raw

# All robots, camera topics only
robots/*/camera/**
```

---

## Testing & Validation

### Verify Namespace Configuration

**On Vision Pi:**

```bash
# Check environment variable
docker exec oak-d env | grep ROBOT_ID
# Expected: ROBOT_ID=RBXU100001

# Check generated config
docker exec oak-d cat /tmp/zenoh_session_config.json5 | grep namespace
# Expected: namespace: "robots/RBXU100001",

# Check if config was applied
docker logs oak-d 2>&1 | grep "Robot ID\|Namespace"
# Expected:
# 🤖 Robot ID: RBXU100001
# 📡 Namespace: robots/RBXU100001
```

**On Main Pi:**

```bash
# Check Zenoh router REST API for topics
curl http://localhost:8000/@/local/subscriber | jq

# Should see topics prefixed with robots/RBXU100001/
# Example:
# {
#   "key": "robots/RBXU100001/camera/rgb/image_raw",
#   ...
# }
```

### Test Cloud Connectivity

**From development machine:**

```bash
# Install Zenoh tools
cargo install zenoh --features=unstable

# Subscribe to all robot topics
z_sub -e "tcp/zenoh.robbox.online:7447" -k "robots/**"

# Subscribe to specific robot
z_sub -e "tcp/zenoh.robbox.online:7447" -k "robots/RBXU100001/**"

# Query available topics
z_get -e "tcp/zenoh.robbox.online:7447" -s "robots/**"
```

### Validate Topic Isolation

**Test with multiple robots:**

```bash
# Robot 1 (RBXU100001)
# Publishes: /cmd_vel → robots/RBXU100001/cmd_vel

# Robot 2 (RBXU100002)  
# Publishes: /cmd_vel → robots/RBXU100002/cmd_vel

# Verify no cross-talk:
z_sub -e "tcp/zenoh.robbox.online:7447" -k "robots/RBXU100001/cmd_vel"
# Should only see messages from Robot 1
```

---

## Troubleshooting

### Issue: Topics Not Appearing in Cloud

**Symptoms:**
- Cloud doesn't see robot topics
- `z_sub` returns no data

**Check:**

1. **Verify ROBOT_ID is set:**
   ```bash
   docker exec oak-d env | grep ROBOT_ID
   ```

2. **Check generated config:**
   ```bash
   docker exec oak-d cat /tmp/zenoh_session_config.json5 | grep namespace
   ```

3. **Verify cloud connection:**
   ```bash
   # On Main Pi
   curl http://localhost:8000/@/router/status
   ```

4. **Check Zenoh router logs:**
   ```bash
   docker logs zenoh-router
   docker logs zenoh-router-vision
   ```

**Solution:**
- If namespace is not set: Check `.env` file has `ROBOT_ID=...`
- If namespace is wrong: Restart containers: `docker-compose restart`
- If no cloud connection: Check firewall, network connectivity

### Issue: Wrong Namespace Prefix

**Symptoms:**
- Topics appear as `robots/default/...` instead of `robots/RBXU100001/...`

**Cause:** ROBOT_ID not passed to container

**Solution:**

```bash
# Check docker-compose.yaml includes:
environment:
  - ROBOT_ID=${ROBOT_ID}

# Check .env file:
cat docker/vision/.env | grep ROBOT_ID
# Should show: ROBOT_ID=RBXU100001

# Restart with explicit env var:
ROBOT_ID=RBXU100001 docker-compose up -d
```

### Issue: Namespace Not Applied to Some Nodes

**Symptoms:**
- Some topics have namespace, some don't

**Cause:** Service not using ros_with_namespace.sh wrapper

**Solution:**

Check `docker-compose.yaml`:

```yaml
# ✅ CORRECT
services:
  my-node:
    command: ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_node.sh"]
    environment:
      - ROBOT_ID=${ROBOT_ID}

# ❌ WRONG (missing wrapper)
services:
  my-node:
    command: ["/scripts/start_node.sh"]
```

### Issue: Cannot Subscribe to Cloud Topics

**Symptoms:**
- `z_sub` fails to connect
- Connection timeout

**Check:**

1. **Cloud router accessibility:**
   ```bash
   telnet zenoh.robbox.online 7447
   # Should connect
   ```

2. **Firewall rules:**
   ```bash
   # On Main Pi
   sudo ufw status
   # Port 7447 should be allowed
   ```

3. **DNS resolution:**
   ```bash
   nslookup zenoh.robbox.online
   ping zenoh.robbox.online
   ```

**Solution:**
- Add firewall rule: `sudo ufw allow 7447/tcp`
- Check VPN/network connectivity
- Verify cloud router is running

---

## ROS 2 Domain ID vs Zenoh Namespace

### Key Differences

| Feature | ROS_DOMAIN_ID | Zenoh Namespace |
|---------|---------------|-----------------|
| **Scope** | Network-level isolation | Topic-level prefix |
| **Purpose** | Prevent DDS cross-talk | Organize topics hierarchically |
| **Visibility** | Nodes in different domains can't see each other | All topics visible, just prefixed |
| **Cloud** | Not visible in topic names | Visible in key expressions |
| **Use Case** | Multiple robots on same network | Cloud organization |

### Rob Box Uses Both

```
ROS_DOMAIN_ID=0  (default)
  └── All nodes communicate via DDS/Zenoh
      └── Zenoh namespace: robots/RBXU100001/
          └── All topics prefixed: robots/RBXU100001/cmd_vel, etc.
```

**Benefits:**
- ROS_DOMAIN_ID: Local network isolation
- Zenoh namespace: Cloud organization and monitoring

---

## Security Considerations

### Current Setup (Non-TLS)

```
Main Pi ──TCP (unencrypted)──> zenoh.robbox.online:7447
```

**Risks:**
- Traffic not encrypted
- No authentication
- Anyone can publish/subscribe

**Acceptable for:**
- Development
- Testing
- Closed networks

### Future: TLS/QUIC Setup

**Recommended for production:**

```json5
{
  connect: {
    endpoints: [
      "tls/zenoh.robbox.online:7448"  // TLS on port 7448
    ]
  },
  transport: {
    link: {
      tls: {
        root_ca_certificate: "/certs/ca.pem",
        connect_certificate: "/certs/robot.pem",
        connect_private_key: "/certs/robot.key",
        verify_name_on_connect: true,
      }
    }
  }
}
```

**Benefits:**
- Encrypted traffic
- Mutual authentication
- Certificate-based access control

---

## References

### Official Documentation

1. **Zenoh Configuration:**
   - https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5

2. **rmw_zenoh Design:**
   - https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md

3. **ROS 2 Namespaces:**
   - https://design.ros2.org/articles/topic_and_service_names.html

4. **Zenoh Key Expressions:**
   - https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/Key%20Expressions.md

### Project Files

- `docker/vision/scripts/ros_with_namespace.sh` - Namespace wrapper script
- `docker/vision/config/zenoh_session_config.json5` - Vision Pi session config
- `docker/main/config/zenoh_router_config.json5` - Main Pi router config (cloud connection)
- `docker/vision/.env` - ROBOT_ID configuration
- `docs/architecture/SOFTWARE.md` - Zenoh architecture overview
- `docs/development/AGENT_GUIDE.md` - AI agent development guide

---

## Changelog

### 2025-10-23 - Initial Documentation
- Documented Zenoh namespace feature
- Explained current implementation with ros_with_namespace.sh
- Added cloud topology diagrams
- Created troubleshooting guide
- Added testing procedures

---

**Author:** AI Agent (GitHub Copilot)  
**Last Updated:** 2025-10-23  
**Status:** Active - Namespace implementation working in production
