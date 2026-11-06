#!/usr/bin/env python3
"""Patch rtabmap.launch.py to fix two issues on the deployed robot image.

Patch 1 — inject Grid/Sensor=0 and other parameters
  that are otherwise auto-overridden by the rtabmap node constructor.

  Root cause:
    rtabmap C++ node forces Grid/Sensor=0 when subscribe_scan=true,
    UNLESS Grid/Sensor is already present as an EXPLICIT ROS2 parameter
    in the node's parameters dict.

    Parameters from cfg:= (INI file) are loaded into rtabmap's INTERNAL
    parameter map, not the ROS2 parameter server → auto-override wins.

  Fix:
    Insert required parameters directly into the Node(parameters=[{...}])
    dict in rtabmap.launch.py BEFORE the Mem/IncrementalMemory entry.
    This makes them explicit ROS2 node parameters → auto-override is skipped.

Patch 2 — migrate the AprilTag input remap from the deprecated internal
  topic name "tag_detections" to the current "apriltag/detections".

  Root cause:
    rtabmap_ros (humble-latest) CoreWrapper subscribes to BOTH
    "tag_detections" (legacy, deprecated) and "apriltag/detections" (current).
    The launch file remaps only the legacy name:

        ("tag_detections", LaunchConfiguration('tag_topic')),

    so detections arrive through the legacy subscription and rtabmap logs:

        [WARN] "tag_detections" input topic name for apriltag_msgs is
        deprecated, remap "apriltag" input topic name instead.

  Fix:
    Remap the CURRENT internal name instead:

        ("apriltag/detections", LaunchConfiguration('tag_topic')),

    The legacy "tag_detections" subscription then stays unmapped (no data,
    no warning) and the current "apriltag/detections" subscription receives
    the data on the configured tag_topic (=/detections) without warnings.

Usage:
  Called from start_rtabmap.sh before 'exec \"$@\"'.
"""

import sys

LAUNCH_FILE = "/opt/ros/humble/share/rtabmap_launch/launch/rtabmap.launch.py"
SENTINEL = '"Mem/IncrementalMemory"'

# Parameters to inject into the rtabmap Node parameters dict (Patch 1)
INJECTED_PARAMS = """\
                "Grid/Sensor": "0",             # LiDAR only for occupancy grid stability
                "Grid/MaxObstacleHeight": "1.5",  # Obstacles above 1.5m ignored
                "Grid/MinGroundHeight": "-0.1",   # Filter floor points below -0.1m
                "Grid/RangeMin": "0.2",
                "Grid/RangeMax": "10.0",
                "Grid/CellSize": "0.05",
                "Reg/Force3DoF": "true",        # Ground robot: constrain to 2D
                "Reg/Strategy": "1",            # ICP registration
                "Icp/VoxelSize": "0.05",
                "Icp/MaxCorrespondenceDistance": "0.1",
                "Icp/CorrespondenceRatio": "0.05",
                "Mem/IncrementalMemory": "false",  # Localization mode, don't grow WM online
                "RGBD/OptimizeMaxError": "5.0",
                "Optimizer/Strategy": "1",      # Force g2o, avoid GTSAM localization failures
                "Optimizer/GravitySigma": "0",
                "Odom/ResetCountdown": "1",
"""

# Patch 2: legacy remap -> current remap (AprilTag detections)
LEGACY_REMAP = "(\"tag_detections\", LaunchConfiguration('tag_topic')),"
CURRENT_REMAP = "(\"apriltag/detections\", LaunchConfiguration('tag_topic')),"

# Patch 3 (#4495): parameters of the icp_odometry node.
#   cfg:=/config/rtabmap/rtabmap.ini is NOT applied: `ros2 param get /rtabmap/icp_odometry
#   Odom/ResetCountdown` gave 0 (ini says 1), Icp/PointToPlane gave true (ini says false), and
#   the Info.parameters row of the HIL rtabmap.db shows the same (HIL Depot 06.11). So
#   icp_odometry ran with rtabmap defaults. After one failed registration (fast in-place turn,
#   far walls) it stayed `lost` for the rest of the run (8309 of 8946 frames) and the whole
#   stack drove on open-loop wheel odometry. Odom/ResetCountdown=1 resets ICP odometry after one
#   lost frame: it resumes from the last good pose plus the wheel guess and builds a new local
#   map, instead of staying lost forever. Explicit node parameters win over the ini.
ICP_NODE_MARKER = "executable='icp_odometry'"
ICP_PARAMS_OPEN = "parameters=[{"
ICP_SENTINEL = '"Odom/ResetCountdown"'
ICP_INJECTED_PARAMS = """
                "Odom/ResetCountdown": "1",     # #4495: do not stay lost forever after one failed ICP"""


def patch_icp_odometry(content: str):
    """Patch 3: explicit parameters into the icp_odometry Node dict. Returns ``(content, patched)``.

    Idempotent: looks for the sentinel only inside the icp_odometry Node block (the rtabmap
    Node dict of Patch 1 carries the same key). No icp_odometry Node — warning, nothing changed.
    """
    start = content.find(ICP_NODE_MARKER)
    if start < 0:
        print("[patch_rtabmap_launch] WARN: icp_odometry Node not found — skipping icp patch.", file=sys.stderr)
        return content, False
    end = content.find("Node(", start)
    block = content[start:] if end < 0 else content[start:end]
    if ICP_SENTINEL in block:
        print("[patch_rtabmap_launch] icp_odometry parameters already injected.")
        return content, False
    opening = block.find(ICP_PARAMS_OPEN)
    if opening < 0:
        print("[patch_rtabmap_launch] WARN: icp_odometry parameters dict not found — skipping.", file=sys.stderr)
        return content, False
    at = start + opening + len(ICP_PARAMS_OPEN)
    print("[patch_rtabmap_launch] icp_odometry parameters injected (#4495).")
    return content[:at] + ICP_INJECTED_PARAMS + content[at:], True


def patch() -> bool:
    try:
        content = open(LAUNCH_FILE).read()
    except OSError as e:
        print(f"[patch_rtabmap_launch] ERROR reading {LAUNCH_FILE}: {e}", file=sys.stderr)
        return False

    patched_remap = False
    patched_params = False

    # ---- Patch 2: AprilTag remap (idempotent) ----
    if CURRENT_REMAP in content:
        print("[patch_rtabmap_launch] Remap already migrated to apriltag/detections.")
    elif LEGACY_REMAP in content:
        content = content.replace(LEGACY_REMAP, CURRENT_REMAP, 1)
        patched_remap = True
        print("[patch_rtabmap_launch] Remap migrated: tag_detections -> apriltag/detections.")
    else:
        print(
            "[patch_rtabmap_launch] WARN: neither legacy nor current apriltag remap " "found — skipping remap patch.",
            file=sys.stderr,
        )

    # ---- Patch 1: inject parameters (idempotent) ----
    if "Grid/Sensor" in content:
        print("[patch_rtabmap_launch] Parameters already injected (Grid/Sensor present).")
    elif SENTINEL in content:
        content = content.replace(SENTINEL, INJECTED_PARAMS + SENTINEL, 1)
        patched_params = True
        print("[patch_rtabmap_launch] Parameters injected into Node parameters dict.")
    else:
        print(
            f"[patch_rtabmap_launch] ERROR: sentinel {SENTINEL!r} not found in {LAUNCH_FILE}.",
            file=sys.stderr,
        )
        return False

    # ---- Patch 3: icp_odometry parameters (idempotent, #4495) ----
    content, patched_icp = patch_icp_odometry(content)
    patched_params = patched_params or patched_icp

    if not (patched_remap or patched_params):
        print(f"[patch_rtabmap_launch] Already fully patched: {LAUNCH_FILE}")
        return True

    try:
        open(LAUNCH_FILE, "w").write(content)
    except OSError as e:
        print(f"[patch_rtabmap_launch] ERROR writing {LAUNCH_FILE}: {e}", file=sys.stderr)
        return False

    print(f"[patch_rtabmap_launch] ✅ Patched {LAUNCH_FILE}: remap={patched_remap}, params={patched_params}")
    return True


if __name__ == "__main__":
    success = patch()
    sys.exit(0 if success else 1)
