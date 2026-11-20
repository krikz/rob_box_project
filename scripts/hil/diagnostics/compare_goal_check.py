#!/usr/bin/env python3
"""Join successful Nav2 goal checks to GT from an archived route rosbag2 recording.

Default goal name is M1, so an old invocation without ``--goal`` still reads the
M1 target. Pass ``--goal NAME`` to join any route goal the same way.

Route sim stamps are quantized to 10 ms (``route.py`` prints ``sim=`` with two
decimals). The wall estimate of GOAL_END therefore has a ±5 ms quantization
uncertainty, reported as ``goal_end_wall_precision_ms``.
"""

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
# route.py prints sim= with two decimals, so a GOAL_END sim stamp is a 10 ms bin.
ROUTE_SIM_QUANTUM_MS = 10
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


def read_route_goal(route: str, goal_name: str = "M1") -> tuple[float, float, float]:
    """Read the original route target, converting route.py's heading to radians.

    ``goal_name`` defaults to M1 so callers that predate ``--goal`` keep the old
    target. The name is escaped before it is interpolated into the pattern.
    """
    match = re.search(rf"GOAL_START {re.escape(goal_name)} target=\(([^)]+)\)", route)
    if match is None:
        raise ValueError(f"route.log has no original {goal_name} target")
    x, y, yaw_deg = map(float, match[1].split(","))
    return x, y, math.radians(yaw_deg)


def read_route_goal_end(route: str, goal_name: str = "M1") -> dict[str, Any] | None:
    """Read GOAL_END result, sim stamp and err. Missing line is not an error.

    Real line today::

        GOAL_END M1 result=STATUS_SUCCEEDED sim=141.52 frame=1565 dur_sim=75.8 err=0.216 ...

    ``sim`` is seconds quantized to 10 ms (two printed decimals). Returned
    ``sim_ns`` is that bin, not a sub-millisecond stamp.
    """
    match = re.search(
        rf"GOAL_END {re.escape(goal_name)} result=(\S+) sim=([\d.]+).*?\berr=([-\d.]+)",
        route,
    )
    if match is None:
        return None
    result, sim_s, err = match.groups()
    return {
        "result": result,
        "sim_s": float(sim_s),
        "sim_ns": int(round(float(sim_s) * 100)) * (SECOND_NS // 100),
        "err": float(err),
    }


def goal_end_wall_ns(decision_wall_ns: int, decision_sim_ns: int, goal_end_sim_ns: int) -> int:
    """Map a route sim stamp onto wall time through the decision marker.

    ``decision_sim_ns`` is the controller marker's sim stamp; ``goal_end_sim_ns``
    comes from route.log and is quantized to 10 ms, so the wall estimate shares
    that ±5 ms uncertainty (see ``ROUTE_SIM_QUANTUM_MS``).
    """
    return decision_wall_ns + (goal_end_sim_ns - decision_sim_ns)


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


def _empty_goal_end() -> Sample:
    """Null GOAL_END slots. A missing line must not invent a neighbour."""
    return {
        "goal_end_sim": None,
        "goal_end_result": None,
        "goal_end_err": None,
        "goal_end_wall_ns": None,
        "goal_end_wall_precision_ms": None,
        "gt_500ms_after_goal_end": None,
        "gt_1500ms_after_goal_end": None,
    }


def _goal_end_fields(
    goal_end: dict[str, Any] | None, decision_wall_ns: int, decision_ns: int, gt: list[Sample]
) -> Sample:
    """Attach GOAL_END identity and GT at +0.5 s / +1.5 s of the route sim stamp."""
    if goal_end is None:
        return _empty_goal_end()
    end_sim_ns = int(goal_end["sim_ns"])
    return {
        "goal_end_sim": goal_end["sim_s"],
        "goal_end_result": goal_end["result"],
        "goal_end_err": goal_end["err"],
        "goal_end_wall_ns": goal_end_wall_ns(decision_wall_ns, decision_ns, end_sim_ns),
        "goal_end_wall_precision_ms": ROUTE_SIM_QUANTUM_MS,
        "gt_500ms_after_goal_end": nearest_gt(gt, end_sim_ns + SECOND_NS // 2),
        "gt_1500ms_after_goal_end": nearest_gt(gt, end_sim_ns + (3 * SECOND_NS) // 2),
    }


def compare(
    fields: dict[str, str],
    goal: tuple[float, float, float],
    gt: list[Sample],
    commands: list[Sample],
    goal_end: dict[str, Any] | None = None,
    decision_wall_ns: int = 0,
) -> Sample:
    """Keep Nav2 errors in its frame and GT errors against the original goal.

    ``goal_end`` is optional. When it is absent (old callers, or a route with no
    GOAL_END for the selected name) the goal_end_* fields are null and the
    decision-relative fields stay as they were.
    """
    if fields["robot_frame"] != fields["goal_frame"]:
        raise ValueError("controller pose and transformed goal have different frames")
    gx, gy, gyaw = goal
    gt = [dict(s, xy_error=math.hypot(s["x"] - gx, s["y"] - gy), yaw_error=angle_error(s["yaw"], gyaw)) for s in gt]
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
        **_goal_end_fields(goal_end, decision_wall_ns, decision_ns, gt),
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


def gt_query_bounds(wall_ns: int, decision_wall_ns: int) -> tuple[int, int]:
    """Wall window that covers GOAL_END + 1.5 s plus bag lag.

    Lower bound stays at the marker's wall−4 s. Upper bound is
    ``decision_wall_ns + 6 s``: the decision wall is the marker wall, and a
    later GOAL_END is shifted onto that axis before the query. Drive-command
    windows (−0.3..+0.6 s) are not touched.
    """
    return wall_ns - 4 * SECOND_NS, decision_wall_ns + 6 * SECOND_NS


def _load_run(
    artifact: Path, goal_name: str
) -> tuple[tuple[float, float, float], dict[str, Any] | None, list[str], list[Path]]:
    """Read the selected goal, its optional GOAL_END, traces and bag databases."""
    route = (artifact / "route.log").read_text(encoding="utf-8")
    goal = read_route_goal(route, goal_name)
    goal_end = read_route_goal_end(route, goal_name)
    traces = (artifact / "goal_check_reached.log").read_text(encoding="utf-8").splitlines()
    databases = sorted((artifact / "bag").glob("*.db3"))
    return goal, goal_end, traces, databases


def main(argv: list[str] | None = None) -> None:
    """Print the report as JSON. Run inside an environment with ROS2 messages.

    ``python3 compare_goal_check.py [--goal NAME] RUN_DIR``

    ``--goal`` defaults to M1. JSON keeps every previous field and adds
    goal_end_sim, goal_end_result, goal_end_err, goal_end_wall_ns,
    goal_end_wall_precision_ms, gt_500ms_after_goal_end and
    gt_1500ms_after_goal_end. A missing GOAL_END leaves those null.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--goal", default="M1", help="route goal name (default: M1)")
    args = parser.parse_args(argv)
    artifact = args.run_dir.resolve()
    goal, goal_end, traces, databases = _load_run(artifact, args.goal)
    if not traces or not databases:
        parser.error("expected a nonempty goal_check_reached.log and SQLite rosbag2 files in bag/")
    comparisons = []
    for line in traces:
        wall_ns, fields = parse_trace(line)
        decision_wall = (
            goal_end_wall_ns(wall_ns, int(fields["sim_ns"]), int(goal_end["sim_ns"])) if goal_end else wall_ns
        )
        gt = [
            gt_sample(mid, msg)
            for mid, _, msg in bag_messages(databases, GT_TOPIC, gt_query_bounds(wall_ns, decision_wall))
        ]
        commands = [
            dict(bag_id=mid, arrival_delta_ms=(arrival - wall_ns) / 1e6, vx=m.linear.x, vy=m.linear.y, wz=m.angular.z)
            for mid, arrival, m in bag_messages(databases, DRIVE_TOPIC, (wall_ns - 300_000_000, wall_ns + 600_000_000))
        ]
        comparisons.append(compare(fields, goal, gt, commands, goal_end, wall_ns))
    print(json.dumps({"goal": args.goal, "comparisons": comparisons}, indent=2))


if __name__ == "__main__":
    main()
