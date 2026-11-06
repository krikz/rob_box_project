#!/usr/bin/env python3
"""Patch rtabmap.launch.py: the ONE source of RTAB-Map parameters on Main Pi (#4495).

Parameters of the ``rtabmap`` and ``icp_odometry`` nodes live here, in
:data:`RTABMAP_PARAMS` and :data:`ICP_ODOMETRY_PARAMS`, and nowhere else. They are written into
the ``Node(parameters=[{...}])`` dicts of rtabmap.launch.py as explicit ROS 2 parameters, so
``ros2 param get /rtabmap/rtabmap <name>`` and ``ros2 param get /rtabmap/icp_odometry <name>``
show exactly what runs.

Why not ``cfg:=rtabmap.ini`` (removed in #4495, it was never applied):
  * rtabmap reads only the ``[Core]`` section of the ini, keys like ``Icp\\PointToPlane``
    (rtabmap 0.24.2 corelib/src/Parameters.cpp, ``readINIImpl()``). Our file had ``[Icp]``,
    ``[Odom]``, ... sections, and both nodes logged
    ``readINIImpl() Section "Core" in /config/rtabmap/rtabmap.ini doesn't exist``
    (Main Pi, ``docker logs rtabmap``, 06.11). ``ros2 param get`` and the HIL rtabmap.db
    showed rtabmap defaults instead of the ini values.
  * Even a working ini is invisible to ``ros2 param get`` on the rtabmap node: CoreWrapper
    declares ROS parameters with rtabmap DEFAULTS and only explicit overrides change them
    (rtabmap_ros 0.23.13-humble rtabmap_slam/src/CoreWrapper.cpp, ``onInit``).
  * CoreWrapper rewrites ``config_path`` with ALL its parameters on shutdown
    (``CoreWrapper::saveParameters``): the ini is a file rtabmap itself owns, not a config.
  Priority, when both exist: explicit ROS parameter > ini ``[Core]`` > rtabmap default.

Patch 1 — :data:`RTABMAP_PARAMS` into the rtabmap Node dict (before ``Mem/IncrementalMemory``,
  which the launch file sets itself from ``localization:=``). Grid/Sensor must be explicit:
  the rtabmap node forces Grid/Sensor=0 with subscribe_scan=true unless it is already set.

Patch 2 — migrate the AprilTag input remap from the deprecated internal topic name
  "tag_detections" to the current "apriltag/detections". rtabmap_ros CoreWrapper subscribes
  to both; the launch file remaps only the legacy one, and rtabmap warns
  ``"tag_detections" input topic name for apriltag_msgs is deprecated``. Remapping the
  current name leaves the legacy subscription unmapped (no data, no warning).

Patch 3 — :data:`ICP_ODOMETRY_PARAMS` into the icp_odometry Node dict.

Usage:
  Called from start_rtabmap.sh before 'exec \"$@\"'.
"""

import sys

LAUNCH_FILE = "/opt/ros/humble/share/rtabmap_launch/launch/rtabmap.launch.py"
SENTINEL = '"Mem/IncrementalMemory"'

# ── Parameters (the only place; decisions per parameter — docs/research/2026-11-05-nav2-hil-tuning.md §9.6) ──
# Everything not listed runs with the rtabmap default (rtabmap 0.24.2, Parameters.h).
RTABMAP_PARAMS = {
    "Grid/Sensor": "0",  # LiDAR only for the occupancy grid
    "Grid/MaxObstacleHeight": "1.5",  # obstacles above 1.5 m ignored
    "Grid/MinGroundHeight": "-0.1",  # floor points below -0.1 m filtered
    "Grid/RangeMin": "0.2",
    "Grid/RangeMax": "10.0",
    "Grid/CellSize": "0.05",
    "Reg/Force3DoF": "true",  # ground robot: x, y, yaw
    "Reg/Strategy": "1",  # ICP registration for loop closure and proximity
    "Icp/VoxelSize": "0.05",
    "Icp/MaxCorrespondenceDistance": "0.1",
    "Icp/CorrespondenceRatio": "0.05",
    "RGBD/OptimizeMaxError": "5.0",  # reject loop closures with a large graph error
    "Optimizer/Strategy": "1",  # g2o: GTSAM failed on ill-posed graphs in localization
    "Optimizer/GravitySigma": "0",  # no IMU gravity constraints
}

# icp_odometry: Odom/* exist only here (CoreWrapper erases Odom/* from the rtabmap node).
#   Odom/ResetCountdown=1 (#4495, HIL Depot 06.11): without it, after one failed registration
#   (fast in-place turn, far walls) ICP odometry stayed `lost` for the rest of the run (8309 of
#   8946 frames). With 1 it resets after one lost frame and resumes from the last good pose.
ICP_ODOMETRY_PARAMS = {
    "Odom/ResetCountdown": "1",
}

# Patch 2: legacy remap -> current remap (AprilTag detections)
LEGACY_REMAP = "(\"tag_detections\", LaunchConfiguration('tag_topic')),"
CURRENT_REMAP = "(\"apriltag/detections\", LaunchConfiguration('tag_topic')),"

ICP_NODE_MARKER = "executable='icp_odometry'"
ICP_PARAMS_OPEN = "parameters=[{"
ICP_SENTINEL = f'"{next(iter(ICP_ODOMETRY_PARAMS))}"'


def render_params(params: dict) -> str:
    """``{"A/B": "1"}`` → lines ``"A/B": "1",`` indented as the launch file dicts."""
    return "".join(f'\n                "{key}": "{value}",' for key, value in params.items())


INJECTED_PARAMS = render_params(RTABMAP_PARAMS).lstrip("\n") + "\n"
ICP_INJECTED_PARAMS = render_params(ICP_ODOMETRY_PARAMS)


def patch_icp_odometry(content: str):
    """Patch 3: explicit parameters into the icp_odometry Node dict. Returns ``(content, patched)``.

    Idempotent: looks for the sentinel only inside the icp_odometry Node block.
    No icp_odometry Node — warning, nothing changed.
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
