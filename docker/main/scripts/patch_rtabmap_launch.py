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
LEGACY_REMAP = '("tag_detections", LaunchConfiguration(\'tag_topic\')),'
CURRENT_REMAP = '("apriltag/detections", LaunchConfiguration(\'tag_topic\')),'


def patch() -> bool:
    try:
        content = open(LAUNCH_FILE).read()
    except OSError as e:
        print(f"[patch_rtabmap_launch] ERROR reading {LAUNCH_FILE}: {e}", file=sys.stderr)
        return False

    original = content
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
            "[patch_rtabmap_launch] WARN: neither legacy nor current apriltag remap "
            "found — skipping remap patch.",
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
