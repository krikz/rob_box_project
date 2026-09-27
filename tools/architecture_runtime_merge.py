#!/usr/bin/env python3
"""Merge runtime ROS 2 snapshots captured from the physical robot Pis."""
from __future__ import annotations
import argparse, json
from pathlib import Path

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--main", type=Path, required=True)
    p.add_argument("--vision", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()

    snapshots = [json.loads(a.main.read_text(encoding="utf-8")),
                  json.loads(a.vision.read_text(encoding="utf-8"))]

    nodes = []
    topics = set()
    services = set()
    actions = set()
    topic_info = {}
    hosts = []

    for snap in snapshots:
        source = snap.get("captured_from", "unknown")
        host = source
        container = ""
        if source.startswith("remote:"):
            rest = source[len("remote:"):]
            if ":container:" in rest:
                host, container = rest.split(":container:", 1)
            else:
                host = rest
        host_entry = {"host": host, "container": container, "source": source}
        hosts.append(host_entry)

        for node in snap.get("nodes", []):
            nodes.append({"name": node, "host": host, "container": container})

        topics.update(snap.get("topics", []))
        services.update(snap.get("services", []))
        actions.update(snap.get("actions", []))

        for topic, info in snap.get("topic_info", {}).items():
            merged = topic_info.setdefault(topic, {
                "present": False, "publishers": [], "subscribers": []
            })
            merged["present"] = merged["present"] or info.get("present", False)
            for role in ("publishers", "subscribers"):
                for endpoint in info.get(role, []):
                    item = dict(endpoint)
                    item["host"] = host
                    item["container"] = container
                    merged[role].append(item)

    # Keep one identity per physical host + ROS node name.
    unique_nodes = {}
    for item in nodes:
        unique_nodes[(item["host"], item["name"])] = item

    merged = {
        "schema_version": 4,
        "captured_at_unix": max(s.get("captured_at_unix", 0) for s in snapshots),
        "captured_from": "multi-host:Main Pi + Vision Pi",
        "hosts": hosts,
        "nodes": sorted(unique_nodes.values(), key=lambda x: (x["host"], x["name"])),
        "node_names": sorted({x["name"] for x in unique_nodes.values()}),
        "topics": sorted(topics),
        "services": sorted(services),
        "actions": sorted(actions),
        "topic_info": topic_info,
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(merged, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "hosts": len(hosts),
        "nodes": len(merged["nodes"]),
        "topics": len(merged["topics"]),
        "services": len(merged["services"]),
        "actions": len(merged["actions"]),
    }, ensure_ascii=False, sort_keys=True))

if __name__ == "__main__":
    main()
