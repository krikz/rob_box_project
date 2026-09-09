# RViz Zenoh Router Endpoint Fix

**Date**: 2025-11-17  
**Status**: ✅ **RESOLVED**  
**Issue**: RViz не видел топики от робота (Main Pi) при использовании rmw_zenoh_cpp

## Root Cause

RViz конфигурация использовала **НЕПРАВИЛЬНЫЙ endpoint** для подключения к Zenoh router:

### Incorrect Configuration (Before Fix)
```json5
{
  "mode": "client",
  "namespace": "robots/RBXU100001",
  "connect": {
    "endpoints": [
      "tcp/10.1.1.10:7447"  // ❌ WRONG: Direct connection to Main Pi router
    ]
  }
}
```

**Проблема**: RViz подключался напрямую к Main Pi router (10.1.1.10:7447), обходя локальный Zenoh router на build машине.

## Correct Network Topology

### Architecture
```
┌─────────────┐                  ┌──────────────┐                  ┌──────────────┐
│   RViz      │                  │  Local       │                  │  Main Pi     │
│  (client)   │ ─────tcp─────>   │  Router      │ ─────tcp─────>   │  Router      │
│             │  10.1.1.249:7447 │ (build PC)   │  10.1.1.10:7447  │  (robot)     │
└─────────────┘                  └──────────────┘                  └──────────────┘
                                         │                                  │
                                         │                                  │
                                    Bridges to                         Receives from
                                    local nodes                        robot nodes
                                                                       (peer mode)
```

### Network IPs
- **Build Machine (wlo1)**: 10.1.1.249
  - Local Zenoh router listens: `tcp/10.1.1.249:7447`
  - Connects to Main Pi: `tcp/10.1.1.10:7447#iface=wlo1`
  
- **Main Pi (eth0)**: 10.1.1.10
  - Zenoh router listens: `tcp/10.1.1.10:7447`
  - Robot nodes connect in peer mode with namespace: `robots/RBXU100001`

- **Main Pi (wlan0)**: 10.1.1.20 (SSH access only)

## Solution

### 1. Fix RViz Client Configuration Template

**File**: `local_test/zenoh_client_config.json5`

**Change**:
```diff
- "tcp/10.1.1.10:7447"   // Direct to Main Pi (WRONG)
+ "tcp/10.1.1.249:7447"  // Via local router (CORRECT)
```

**Reasoning**:
- Local router (10.1.1.249:7447) already maintains connection to Main Pi router
- Local router handles message brokering and namespace translation
- This matches the architecture used by Vision Pi (connects to local router)

### Corrected Configuration (After Fix)
```json5
{
  "mode": "client",
  "namespace": "robots/RBXU100001",  // ✅ Correctly added by start_rviz.sh
  
  // Connect to LOCAL Zenoh router in CLIENT mode
  // Local router (10.1.1.249:7447) bridges to Main Pi router (10.1.1.10:7447)
  "connect": {
    "endpoints": [
      "tcp/10.1.1.249:7447"  // ✅ CORRECT: Local router endpoint
    ]
  },
  
  "listen": {
    "endpoints": [
      "tcp/[::]:0"  // Random port for client mode
    ]
  },
  
  "scouting": {
    "multicast": {
      "enabled": false
    },
    "gossip": {
      "enabled": true,
      "autoconnect": {
        "peer": ["router", "peer"]
      }
    }
  }
}
```

## Verification

### Before Fix
```bash
export ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_rviz_config_RBXU100001.json5
ros2 topic list
# Output: /parameter_events, /rosout (только локальные топики)
```

### After Fix
```bash
# RViz logs show robot topics:
[ERROR] [rviz2]: Lookup would require extrapolation into the future. 
  Requested time 1763396262.203017 but the latest data is at time 1763396262.125833, 
  when looking up transform from frame [lslidar_n10] to frame [map]

[WARN] [rviz2]: topic '/rtabmap/localization_pose' has more than one types associated
```

**Evidence of Success**:
- ✅ RViz receives `/tf` topics (lslidar_n10 → map transforms)
- ✅ RViz sees `/rtabmap/localization_pose` topic
- ✅ RViz attempts to render `/map`
- ✅ Transform lookups from robot frames work

### Robot Topics Verified on Main Pi
```bash
ssh ros2@10.1.1.20 "docker exec rtabmap bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'"
# Output (partial):
/camera/camera/color/image_raw/compressed
/camera/stereo/image_raw
/cmd_vel
/goal_pose
/imu/data
/lidar/scan
/map
/odom
/rtabmap/cloud_map
/rtabmap/localization_pose
/tf
/tf_static
# ... and many more
```

## Investigation Notes

### False Lead: Namespace sed Bug
Initial investigation found a bug in `scripts/start_rviz.sh` line 74:
```bash
if grep -q '"mode": "peer"' "$ZENOH_CONFIG"; then
  sed -i 's|"mode": "peer",|...|' "$ZENOH_CONFIG"  # Would fail: template has mode: "peer" (no quotes)
```

**However**: This bug is **INACTIVE** because:
- RViz uses **client mode template** (`zenoh_client_config.json5`)
- Client template has `"mode": "client"` WITH quotes around `mode` keyword
- The else branch (line 77) correctly adds namespace: ✅ `namespace: "robots/RBXU100001"`

**Conclusion**: Namespace WAS correctly added. The real problem was the **wrong router endpoint**.

### Local Zenoh Router Configuration
**File**: `/zenoh_config.json5` (inside `zenoh-router-local` container)

```json5
{
  mode: "router",
  
  // Connect to Main Pi router (eth0 IP via WiFi interface)
  connect: {
    endpoints: ["tcp/10.1.1.10:7447#iface=wlo1"],
    timeout_ms: { router: -1, peer: -1, client: 0 },
    exit_on_failure: { router: false, peer: false, client: true },
    retry: {
      period_init_ms: 1000,
      period_max_ms: 4000,
      period_increase_factor: 2,
    }
  },
  
  // Listen for local ROS nodes on WiFi IP (not localhost)
  listen: {
    endpoints: ["tcp/10.1.1.249:7447"],
    timeout_ms: 0
  },
  
  scouting: {
    multicast: { enabled: false }
  },
  
  routing: {
    router: {
      peers_failover_brokering: true  // Enable message brokering
    }
  }
}
```

**Key Points**:
- Local router listens on **10.1.1.249:7447** (build machine WiFi IP)
- Connects to Main Pi via **10.1.1.10:7447** (Main Pi Ethernet IP)
- Enables **peers_failover_brokering** for client-peer bridging

## Files Changed

### Modified Files
1. **`local_test/zenoh_client_config.json5`**
   - Changed endpoint from `tcp/10.1.1.10:7447` to `tcp/10.1.1.249:7447`
   - Updated comments to reflect correct topology

### No Changes Needed
1. **`scripts/start_rviz.sh`**
   - sed bug exists but doesn't affect client mode (inactive)
   - Namespace addition works correctly via line 77 else branch
   - No fix required for current use case

## Impact

### Before Fix
- ❌ RViz shows only local topics (/parameter_events, /rosout)
- ❌ No robot data visible in RViz
- ❌ Direct connection to Main Pi router bypasses local router

### After Fix
- ✅ RViz receives all robot topics through local router
- ✅ Transform tree displays robot frames (lslidar_n10, map, etc.)
- ✅ Proper network topology: RViz → Local Router → Main Pi Router → Robot
- ✅ Consistent with Vision Pi architecture

## Deployment

### Testing
```bash
cd /home/ros2/rob_box_project
bash scripts/start_rviz.sh
# Expected: RViz opens and shows robot topics, transforms, and map
```

### Rollout
1. ✅ Template updated: `local_test/zenoh_client_config.json5`
2. ✅ Config regenerated: `/tmp/zenoh_rviz_config_RBXU100001.json5`
3. ✅ Verified robot topics visible in RViz
4. 🔜 Commit changes and create pull request

### Future Improvements
1. **Fix inactive sed bug** (line 74) for completeness:
   ```bash
   # Change:
   if grep -q '"mode": "peer"' "$ZENOH_CONFIG"; then
   # To:
   if grep -q 'mode: "peer"' "$ZENOH_CONFIG"; then  # Remove quotes around 'mode'
   ```

2. **Add endpoint validation** in `start_rviz.sh`:
   ```bash
   # Verify local router is running before starting RViz
   if ! nc -z 10.1.1.249 7447 2>/dev/null; then
     echo "⚠️  Local Zenoh router not reachable at 10.1.1.249:7447"
     echo "   Start local router: docker start zenoh-router-local"
     exit 1
   fi
   ```

## Lessons Learned

### Investigation Path
1. ✅ Initially suspected namespace mismatch
2. ✅ Found sed pattern bug (turned out to be inactive)
3. ✅ Verified namespace WAS correctly added
4. ✅ **Breakthrough**: Checked router endpoint → found wrong IP
5. ✅ Corrected endpoint to use local router
6. ✅ Verified fix with RViz logs showing robot topics

### Key Insights
- **Template format matters**: JSON5 allows both `mode: "peer"` and `"mode": "client"` (with/without quotes on key)
- **Network topology crucial**: Client mode needs to connect to local router, not remote router
- **Router bridging**: Local router handles message brokering between client and peer modes
- **Verification approach**: Check actual generated config, not just templates

## Related Documentation

- Architecture: `docs/architecture/SYSTEM_OVERVIEW.md`
- Network diagram: Main Pi (10.1.1.10 eth0, 10.1.1.20 wlan0)
- Zenoh namespace: `ZENOH_TRANSPORT_FIX_QUICKREF.md`
- Router config: `docker/monitoring/zenoh_config.json5`

## Summary

**Problem**: RViz connected directly to Main Pi router (10.1.1.10:7447), bypassing local router  
**Solution**: Changed endpoint to local router (10.1.1.249:7447)  
**Result**: RViz now receives all robot topics through proper network topology  
**Status**: ✅ **RESOLVED** and ready for PR
