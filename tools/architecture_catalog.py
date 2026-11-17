#!/usr/bin/env python3
"""Render human-reviewable node and topic catalogs from static inventory."""
from __future__ import annotations
import argparse,json
from pathlib import Path

def main():
    p=argparse.ArgumentParser()
    p.add_argument("inventory",type=Path)
    p.add_argument("--node-output",type=Path,default=Path("architecture/node-catalog.md"))
    p.add_argument("--topic-output",type=Path,default=Path("architecture/topic-catalog.md"))
    a=p.parse_args()
    d=json.loads(a.inventory.read_text(encoding="utf-8"))
    node_classes=[x for x in d.get("classes",[]) if x.get("is_node")]
    launches=d.get("launches",[])
    lines=["# Node Catalog","","Static node evidence. A node is architectural only when its boundary and capability are justified by review.","",
           "| Node class | Package | Source | Interfaces | Launch evidence |","|---|---|---|---:|---|"]
    for n in node_classes:
        interfaces=[x for x in d.get("ros_interfaces",[]) if x.get("node_class")==n["name"] and x.get("file")==n["file"]]
        launch=[x for x in launches if x.get("package")==n.get("package")]
        lines.append(f"| {n['name']} | {n.get('package') or '—'} | {n['file']}:{n['line']} | {len(interfaces)} | {len(launch)} launch entries |")
    a.node_output.parent.mkdir(parents=True,exist_ok=True)
    a.node_output.write_text("\n".join(lines)+"\n",encoding="utf-8")

    lines=["# Topic / Interface Catalog","","Names and static endpoints are evidence. Semantic ownership must be filled only after review.","",
           "| Name | Type(s) | Publishers | Subscribers | Services | Clients |","|---|---|---:|---:|---:|---:|"]
    for t in d.get("topics",[]):
        types=", ".join(t.get("types",[])) or "—"
        lines.append(f"| {t['name']} | {types} | {len(t.get('publishers',[]))} | {len(t.get('subscribers',[]))} | {len(t.get('services',[]))} | {len(t.get('clients',[]))} |")
    a.topic_output.parent.mkdir(parents=True,exist_ok=True)
    a.topic_output.write_text("\n".join(lines)+"\n",encoding="utf-8")
    print(json.dumps({"nodes":len(node_classes),"topics":len(d.get("topics",[]))},ensure_ascii=False,sort_keys=True))
if __name__=="__main__":
    main()
