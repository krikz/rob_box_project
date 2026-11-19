#!/usr/bin/env python3
"""Join successful Nav2 goal checks to GT from an archived M1 rosbag2 recording."""

from __future__ import annotations

import argparse
import json
import math
import re
import sqlite3
from contextlib import closing
from pathlib import Path
from typing import Any, Iterator

TRACE_MARKER = "HIL_GOAL_CHECK_REACHED "
SECOND_NS = 1_000_000_000
GT_TOPIC = "/sim/ground_truth/odom"
DRIVE_TOPIC = "/diff_drive_controller/cmd_vel_unstamped"
Sample = dict[str, Any]


def parse_trace(line: str) -> tuple[int, dict[str, str]]:
    """Read the wall-clock log stamp and the exact controller input fields."""
    prefix, fields = line.split(TRACE_MARKER, 1)
    match = re.search(r"\[(\d+)\.(\d{1,9})\]", prefix)
    if match is None:
        raise ValueError("goal checker trace has no wall-clock log stamp")
    seconds, fraction = match.groups()
    return int(seconds) * SECOND_NS + int(fraction.ljust(9, "0")), dict(field.split("=", 1) for field in fields.split())


def read_route_goal(route: str) -> tuple[float, float, float]:
    """Read the original route target, converting route.py's heading to radians."""
    match = re.search(r"GOAL_START M1 target=\(([^)]+)\)", route)
    if match is None:
        raise ValueError("route.log has no original M1 target")
    x, y, yaw_deg = map(float, match[1].split(","))
    return x, y, math.radians(yaw_deg)


def angle_error(yaw: float, goal_yaw: float) -> float:
    """Absolute shortest angle between two headings."""
    return abs(math.remainder(yaw - goal_yaw, math.tau))


def nearest_gt(samples: list[Sample], target_ns: int) -> Sample | None:
    """Find a GT header stamp within 100 ms. Report the signed offset."""
    if not samples or target_ns <= 0:
        return None
    match = min(samples, key=lambda row: abs(row["stamp_ns"] - target_ns))
    delta_ms = (match["stamp_ns"] - target_ns) / 1e6
    return dict(match, delta_ms=delta_ms) if abs(delta_ms) <= 100 else None


def compare(
    fields: dict[str, str], goal: tuple[float, float, float], gt: list[Sample], commands: list[Sample]
) -> Sample:
    """Keep Nav2 errors in its frame and GT errors against the original goal."""
    if fields["robot_frame"] != fields["goal_frame"]:
        raise ValueError("controller pose and transformed goal have different frames")
    gx, gy, gyaw = goal
    gt = [
        dict(s, xy_error=math.hypot(s["x"] - gx, s["y"] - gy), yaw_error=angle_error(s["yaw"], gyaw))
        for s in gt
    ]
    decision_ns, pose_ns = int(fields["sim_ns"]), int(fields["robot_ns"])
    nav_x, nav_y, nav_yaw = (float(fields[f"robot_{component}"]) for component in ("x", "y", "yaw"))
    goal_x, goal_y, goal_yaw = (float(fields[f"goal_{component}"]) for component in ("x", "y", "yaw"))
    return {
        "nav2": fields,
        "robot_pose_age_ms": (decision_ns - pose_ns) / 1e6 if pose_ns > 0 else None,
        "nav2_xy_error": math.hypot(nav_x - goal_x, nav_y - goal_y),
        "nav2_yaw_error": angle_error(nav_yaw, goal_yaw),
        "gt_at_nav2_pose_stamp": nearest_gt(gt, pose_ns),
        "gt_at_decision_tick": nearest_gt(gt, decision_ns),
        "gt_500ms_after_decision": nearest_gt(gt, decision_ns + SECOND_NS // 2),
        "gt_1s_after_decision": nearest_gt(gt, decision_ns + SECOND_NS),
        "drive_commands_by_receipt_near_decision": commands,
    }


def bag_messages(
    databases: list[Path], topic: str, bounds: tuple[int, int] | None = None
) -> Iterator[tuple[int, int, Any]]:
    """Read only the selected topic from the SQLite bag files."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    for database in databases:
        with closing(sqlite3.connect(f"{database.as_uri()}?mode=ro", uri=True)) as connection:
            row = connection.execute("SELECT id, type FROM topics WHERE name = ?", (topic,)).fetchone()
            if row is None:
                continue
            topic_id, message_type = row
            msg_type = get_message(message_type)
            query = "SELECT id, timestamp, data FROM messages WHERE topic_id = ?"
            args = (topic_id,)
            if bounds is not None:
                query += " AND timestamp BETWEEN ? AND ?"
                args += bounds
            for mid, arrival, data in connection.execute(query + " ORDER BY timestamp", args):
                yield mid, arrival, deserialize_message(data, msg_type)


def gt_sample(mid: int, message: Any) -> Sample:
    """Extract a planar pose and velocity with the simulation header stamp."""
    position, q = message.pose.pose.position, message.pose.pose.orientation
    stamp = message.header.stamp
    velocity = message.twist.twist
    return {
        "bag_id": mid,
        "stamp_ns": stamp.sec * SECOND_NS + stamp.nanosec,
        "frame": message.header.frame_id,
        "x": position.x,
        "y": position.y,
        "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        "vx": velocity.linear.x,
        "vy": velocity.linear.y,
        "wz": velocity.angular.z,
    }


def main() -> None:
    """Print the report as JSON. Run inside an environment with ROS2 messages."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    artifact = args.run_dir.resolve()
    goal = read_route_goal((artifact / "route.log").read_text(encoding="utf-8"))
    traces = (artifact / "goal_check_reached.log").read_text(encoding="utf-8").splitlines()
    databases = sorted((artifact / "bag").glob("*.db3"))
    if not traces or not databases:
        parser.error("expected a nonempty goal_check_reached.log and SQLite rosbag2 files in bag/")
    comparisons = []
    for line in traces:
        wall_ns, fields = parse_trace(line)
        gt = [
            gt_sample(mid, msg)
            for mid, _, msg in bag_messages(databases, GT_TOPIC, (wall_ns - 4 * SECOND_NS, wall_ns + 4 * SECOND_NS))
        ]
        commands = [
            dict(bag_id=mid, arrival_delta_ms=(arrival - wall_ns) / 1e6, vx=m.linear.x, vy=m.linear.y, wz=m.angular.z)
            for mid, arrival, m in bag_messages(databases, DRIVE_TOPIC, (wall_ns - 300_000_000, wall_ns + 600_000_000))
        ]
        comparisons.append(compare(fields, goal, gt, commands))
    print(json.dumps({"comparisons": comparisons}, indent=2))


if __name__ == "__main__":
    main()
