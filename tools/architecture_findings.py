#!/usr/bin/env python3
"""Find structural architecture warnings in a generated inventory."""
from __future__ import annotations
import argparse, json
from collections import defaultdict
from pathlib import Path

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("inventory", type=Path)
    parser.add_argument("--markdown", type=Path, default=Path("architecture/findings.md"))
    args = parser.parse_args()
    data = json.loads(args.inventory.read_text(encoding="utf-8"))
    findings = []
    publishers = defaultdict(set)
    subscribers = defaultdict(set)
    for item in data.get("ros_interfaces", []):
        if item["kind"] == "publish": publishers[item["name"]].add(item["file"])
        elif item["kind"] == "subscribe": subscribers[item["name"]].add(item["file"])
    for name, files in sorted(publishers.items()):
        if len(files) > 1:
            findings.append({"type":"multiple_publishers","severity":"review","interface":name,"evidence":sorted(files),"message":"More than one source file publishes this interface."})
    for name in sorted(set(publishers) | set(subscribers)):
        if name.startswith("~"): continue
        if name in publishers and name not in subscribers:
            findings.append({"type":"publish_without_static_subscriber","severity":"review","interface":name,"evidence":sorted(publishers[name]),"message":"No static subscriber declaration was found."})
        if name in subscribers and name not in publishers:
            findings.append({"type":"subscribe_without_static_publisher","severity":"review","interface":name,"evidence":sorted(subscribers[name]),"message":"No static publisher declaration was found."})
    classes = defaultdict(list)
    for item in data.get("classes", []): classes[item["name"]].append(item["file"])
    for name, files in sorted(classes.items()):
        if len(files) > 1:
            findings.append({"type":"duplicate_class_name","severity":"review","class":name,"evidence":sorted(files),"message":"The same class name exists in multiple files."})
    for item in data.get("python", []):
        if len(item.get("node_classes", [])) > 1:
            findings.append({"type":"multiple_node_classes_in_file","severity":"review","file":item["path"],"evidence":item["node_classes"],"message":"Multiple ROS node classes share one Python file."})
    out = {"schema_version":1,"summary":{"findings":len(findings),"multiple_publishers":sum(x["type"]=="multiple_publishers" for x in findings),"unpaired_interfaces":sum(x["type"] in {"publish_without_static_subscriber","subscribe_without_static_publisher"} for x in findings),"duplicate_class_names":sum(x["type"]=="duplicate_class_name" for x in findings)},"findings":findings}
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# Architecture Findings","","Deterministic structural review candidates generated from the static inventory.","These findings are not automatic architectural decisions.","","## Summary",""]
    lines += [f"- {key}: **{value}**" for key, value in out["summary"].items()]
    lines += ["","## Findings",""]
    if not findings: lines.append("No structural findings.")
    else:
        for i, item in enumerate(findings, 1):
            subject = item.get("interface") or item.get("class") or item.get("file") or "architecture"
            lines += [f"### {i}. {item['type']} — {subject}","",f"**Severity:** {item['severity']}","",item["message"],"","**Evidence:**"]
            lines += [f"- {evidence}" for evidence in item["evidence"]]
            lines.append("")
    args.markdown.write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(out["summary"], ensure_ascii=False, sort_keys=True))
    return 0

if __name__ == "__main__": raise SystemExit(main())