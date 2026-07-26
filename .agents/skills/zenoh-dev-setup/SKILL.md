---
name: zenoh-dev-setup
description: "Zenoh DDS middleware setup for connecting a development machine to Rob Box robot via ROS 2. Use when setting up Zenoh communication, debugging topic discovery, configuring router/session, or troubleshooting dev-to-robot connectivity over WiFi/Ethernet network."
---

# Zenoh Dev Machine Setup

Connect a local development machine to Rob Box robot via Zenoh DDS middleware (rmw_zenoh_cpp).

## Network Topology

```
Dev Machine (10.1.1.249, wlp1s0 WiFi)
  └── Local Zenoh Router (listens 10.1.1.249:7447)
        └── connects to Main Pi Router (10.1.1.10:7447, eth0)
              └── connects to Vision Pi Router (10.1.1.11:7447, eth0)
              └── connects to Cloud (zenoh.robbox.online:7447)
```

## Prerequisites

```bash
# Install ROS 2 kilted + Zenoh middleware
sudo apt install ros-kilted-ros-base ros-kilted-ros2cli ros-kilted-rmw-zenoh-cpp
```

## Quick Start (3 steps)

### Step 1: Start local Zenoh router

```bash
cd ~/rob_box_project
source /opt/ros/kilted/setup.bash
ZENOH_ROUTER_CONFIG_URI=$(pwd)/local_test/zenoh_local_router.json5 \
  ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Step 2: Set environment (every new terminal)

```bash
source /opt/ros/kilted/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI=$(pwd)/local_test/zenoh_local_session.json5
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

### Step 3: Reset ros2 daemon (first time only)

```bash
ros2 daemon stop
# Wait 2 seconds, then:
ros2 topic list   # Should show robot topics
```

## Critical Rules (from past fixes)

### 1. Environment variable name
```bash
# ❌ WRONG — rmw_zenoh_cpp ignores this
export ZENOH_CONFIG=/path/to/config.json5

# ✅ CORRECT — for ROS nodes / CLI
export ZENOH_SESSION_CONFIG_URI=/path/to/config.json5

# ✅ CORRECT — for Zenoh router daemon
export ZENOH_ROUTER_CONFIG_URI=/path/to/config.json5
```
**Source:** `docs/fixes/RVIZ_ZENOH_CRITICAL_FIX_2025-11-19.md`

### 2. Mode must be "peer", not "client"
```json5
// ❌ WRONG — client mode doesn't discover topics through local router
{ "mode": "client" }

// ✅ CORRECT — peer mode with localhost listen (as on all Pi nodes)
{
  "mode": "peer",
  "listen": { "endpoints": ["tcp/localhost:0"] }
}
```
**Source:** `docs/fixes/RVIZ_ZENOH_CRITICAL_FIX_2025-11-19.md`

### 3. Namespace must match robot
Robot publishes with `namespace: "robots/RBXU100001"`. Dev session config MUST have the same namespace to see topics without prefix:
```json5
// In session config:
namespace: "robots/RBXU100001"
```
Without matching namespace → `ros2 topic list` shows only `/parameter_events` and `/rosout`.

### 4. WiFi interface binding (#iface)
Router config must specify correct interface name:
```json5
// Check your interface: ip link show | grep -E "^[0-9]"
// Common: wlp1s0, wlo1, wlan0
connect: {
  endpoints: ["tcp/10.1.1.10:7447#iface=wlp1s0"]
}
```
**Source:** `docs/fixes/ZENOH_ETHERNET_QUICKFIX.md`

### 5. ros2 daemon caching
The ros2 CLI daemon caches discovery. After changing Zenoh config or restarting router:
```bash
ros2 daemon stop
sleep 2
ros2 topic list   # Fresh discovery
```

## Config Files Reference

| File | Mode | Purpose |
|------|------|---------|
| `local_test/zenoh_local_router.json5` | router | Dev router → bridges to Main Pi 10.1.1.10:7447 |
| `local_test/zenoh_local_session.json5` | peer | ROS nodes/CLI → connects to local router, has namespace |
| `local_test/zenoh_local_session_no_namespace.json5` | peer | Same but no namespace (see raw prefixed topics) |
| `local_test/zenoh_client_config.json5` | peer | Peer to local router with gossip autoconnect |
| `local_test/zenoh_simple_peer.json5` | peer | Direct peer to Main Pi (no local router needed) |
| `local_test/zenoh_client.json5` | client | Direct client to Main Pi (simplest, limited) |

**Recommended setup:** Router (`zenoh_local_router.json5`) + Session (`zenoh_local_session.json5`)

## Troubleshooting

### No topics visible (only /parameter_events, /rosout)

Check in order:
1. **Network**: `ping 10.1.1.10` and `nc -z 10.1.1.10 7447` (port open?)
2. **Router running**: `ps aux | grep rmw_zenohd`
3. **Namespace mismatch**: Use `zenoh_local_session.json5` (has `namespace: "robots/RBXU100001"`)
4. **Stale daemon**: `ros2 daemon stop`, wait 2s, retry
5. **Wrong env var**: Must be `ZENOH_SESSION_CONFIG_URI`, not `ZENOH_CONFIG`
6. **Wrong mode**: Session must be `mode: "peer"`, not `"client"`

### Topics visible but no data / low Hz

1. **WiFi bandwidth**: Zenoh over WiFi may throttle. Expect ~10 Hz for odom (vs 50 Hz on LAN)
2. **Interface binding**: Ensure `#iface=wlp1s0` matches actual WiFi interface

### Router starts but doesn't bridge

1. Check robot router is up: `sshpass -p 'open' ssh ros2@10.1.1.10 'docker ps | grep zenoh'`
2. Check TCP connection established: `ss -tnp | grep 7447`
3. Check versions match: Robot uses zenohd v1.6.2, local must use compatible zenoh-c version

## Verification Commands

```bash
# Connectivity check
ping -c 2 10.1.1.10 && nc -z -w 3 10.1.1.10 7447 && echo "OK"

# Topic discovery
ros2 topic list | wc -l    # Should be >50 when robot is running

# Odometry check
ros2 topic hz /odom         # ~10-50 Hz depending on WiFi/Ethernet

# Odom position
ros2 topic echo /odom --once | grep -A3 "position"

# Robot containers
sshpass -p 'open' ssh ros2@10.1.1.10 'docker ps --format "table {{.Names}}\t{{.Status}}"'
```

## Related Documentation

- `docs/fixes/RVIZ_ZENOH_CRITICAL_FIX_2025-11-19.md` — env var + mode fix
- `docs/fixes/ZENOH_ETHERNET_QUICKFIX.md` — #iface binding
- `docs/fixes/ZENOH_FIX_SUMMARY_2025-11-10.md` — port conflict + bidirectional TCP
- `docs/fixes/RVIZ_ZENOH_NAMESPACE_INVESTIGATION.md` — namespace investigation
- `docs/architecture/NETWORK_TOPOLOGY.md` — full network architecture
- `scripts/testing/validate_zenoh_config.sh` — automated config validator
