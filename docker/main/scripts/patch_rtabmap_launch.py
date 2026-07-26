#!/usr/bin/env python3
"""Patch rtabmap.launch.py to inject Grid/Sensor=0 and other parameters
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

Usage:
  Called from start_rtabmap.sh before 'exec "$@"'.
"""

import sys

LAUNCH_FILE = "/opt/ros/kilted/share/rtabmap_launch/launch/rtabmap.launch.py"
SENTINEL = '"Mem/IncrementalMemory"'

# Parameters to inject into the rtabmap Node parameters dict
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


def patch() -> bool:
    try:
        content = open(LAUNCH_FILE).read()
    except OSError as e:
        print(f"[patch_rtabmap_launch] ERROR reading {LAUNCH_FILE}: {e}", file=sys.stderr)
        return False

    if "Grid/Sensor" in content:
        print(f"[patch_rtabmap_launch] Already patched: {LAUNCH_FILE}")
        return True

    if SENTINEL not in content:
        print(
            f"[patch_rtabmap_launch] ERROR: sentinel {SENTINEL!r} not found in {LAUNCH_FILE}.",
            file=sys.stderr,
        )
        return False

    patched = content.replace(SENTINEL, INJECTED_PARAMS + SENTINEL, 1)

    try:
        open(LAUNCH_FILE, "w").write(patched)
    except OSError as e:
        print(f"[patch_rtabmap_launch] ERROR writing {LAUNCH_FILE}: {e}", file=sys.stderr)
        return False

    print(
        f"[patch_rtabmap_launch] ✅ Patched {LAUNCH_FILE}: "
        "Grid/Sensor=0 and other params injected into Node parameters dict."
    )
    return True


if __name__ == "__main__":
    success = patch()
    sys.exit(0 if success else 1)
