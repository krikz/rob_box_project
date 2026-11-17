#!/usr/bin/env python3
"""Merge runtime ROS 2 snapshots captured from the robot Pis into runtime.json.

Each ``--capture PI|HOST|CONTAINER|FILE`` names one snapshot written by
``architecture_runtime_snapshot.py``. One capture is enough: on ROB-BOX both
Pis see the same shared Zenoh graph, so a second capture (Vision Pi) only adds
evidence, and identical inputs merge to the same graph.

Output (schema_version 4), read by architecture_runtime_graph.py,
architecture_runtime_diff.py, architecture_runtime_findings.py and the
"Publish runtime summary" step of ``L: Architecture Audit``:

* ``nodes`` / ``topics`` / ``services`` / ``actions`` — sorted union of names;
* ``topic_info`` — per topic, the first capture's entry (including ``raw``)
  with endpoints from later captures appended when their (node, namespace)
  is new; ``present`` is true when any capture saw the topic;
* ``captures`` — ``[{"host", "pi", "container", "snapshot"}]`` in argument order;
* ``captured_from`` — ``remote:`` + Pi labels without spaces joined by ``+``;
* ``ros_domain_id`` / ``rmw_implementation`` — first non-empty value.
"""
from __future__ import annotations
import argparse, copy, json
from pathlib import Path

LIST_KEYS = ("nodes", "topics", "services", "actions")


def parse_capture(value):
    """``PI|HOST|CONTAINER|FILE`` -> dict; raises ArgumentTypeError on bad input."""
    parts = value.split("|")
    if len(parts) != 4 or not all(x.strip() for x in parts):
        raise argparse.ArgumentTypeError(f"expected PI|HOST|CONTAINER|FILE, got {value!r}")
    pi, host, container, path = (x.strip() for x in parts)
    return {"pi": pi, "host": host, "container": container, "path": Path(path)}


def union_lines(snapshots, key):
    return sorted({value for snapshot in snapshots for value in snapshot.get(key, [])})


def merge_topic_info(snapshots):
    topic_info = {}
    for snapshot in snapshots:
        for topic, info in snapshot.get("topic_info", {}).items():
            if topic not in topic_info:
                topic_info[topic] = copy.deepcopy(info)
                continue
            merged = topic_info[topic]
            for field in ("publishers", "subscribers"):
                existing = {(x.get("node"), x.get("namespace")) for x in merged.get(field, [])}
                for endpoint in info.get(field, []):
                    key = (endpoint.get("node"), endpoint.get("namespace"))
                    if key not in existing:
                        merged.setdefault(field, []).append(copy.deepcopy(endpoint))
            merged["present"] = bool(merged.get("present") or info.get("present"))
    return topic_info


def first_value(snapshots, key):
    for snapshot in snapshots:
        if snapshot.get(key):
            return snapshot[key]
    return None


def merge(captures, snapshots):
    """Merge ``snapshots`` (same order as ``captures``) into one runtime graph."""
    if not snapshots:
        raise ValueError("at least one snapshot is required")
    merged = {
        "schema_version": 4,
        "captured_at_unix": max(s.get("captured_at_unix", 0) for s in snapshots),
        "captured_from": "remote:" + "+".join(c["pi"].replace(" ", "") for c in captures),
        "ros_domain_id": first_value(snapshots, "ros_domain_id"),
        "rmw_implementation": first_value(snapshots, "rmw_implementation"),
    }
    for key in LIST_KEYS:
        merged[key] = union_lines(snapshots, key)
    merged["topic_info"] = merge_topic_info(snapshots)
    merged["captures"] = [
        {"host": c["host"], "pi": c["pi"], "container": c["container"], "snapshot": c["path"].name}
        for c in captures
    ]
    return merged


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--capture", type=parse_capture, action="append", required=True,
                   metavar="PI|HOST|CONTAINER|FILE", help="One snapshot; repeat for several Pis")
    p.add_argument("--output", type=Path, default=Path("architecture/runtime.json"))
    a = p.parse_args(argv)

    snapshots = [json.loads(c["path"].read_text(encoding="utf-8")) for c in a.capture]
    merged = merge(a.capture, snapshots)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(merged, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary = {f"{c['pi']} nodes": len(s.get("nodes", [])) for c, s in zip(a.capture, snapshots)}
    summary.update({"union_nodes": len(merged["nodes"]), "union_topics": len(merged["topics"])})
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
