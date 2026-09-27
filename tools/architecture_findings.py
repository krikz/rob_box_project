#!/usr/bin/env python3
"""Generate deterministic architecture review candidates from static inventory."""
from __future__ import annotations
import argparse,json,re
from collections import defaultdict
from pathlib import Path
GENERIC_SEMANTIC_NAMES=re.compile(r"/?(person|user|speaker|identity|face|current_user|current_person)(/|$)",re.I)
def main():
    p=argparse.ArgumentParser(); p.add_argument("inventory",type=Path); p.add_argument("--markdown",type=Path,default=Path("architecture/findings.md")); p.add_argument("--json",type=Path,default=Path("architecture/findings.json")); a=p.parse_args()
    data=json.loads(a.inventory.read_text(encoding="utf-8")); findings=[]
    for t in data.get("topics",[]):
        pubs,subs=t.get("publishers",[]),t.get("subscribers",[])
        if len(pubs)>1: findings.append({"type":"multiple_publishers","severity":"review","subject":t["name"],"evidence":pubs,"message":"More than one static publisher exists. Verify whether ownership is intentionally multi-writer."})
        if pubs and not subs: findings.append({"type":"publish_without_static_subscriber","severity":"review","subject":t["name"],"evidence":pubs,"message":"Static publisher has no matching static subscriber."})
        if subs and not pubs: findings.append({"type":"subscribe_without_static_publisher","severity":"review","subject":t["name"],"evidence":subs,"message":"Static subscriber has no matching static publisher."})
        if GENERIC_SEMANTIC_NAMES.search(t["name"]): findings.append({"type":"semantic_topic_name_review","severity":"review","subject":t["name"],"evidence":t.get("files",[]),"message":"Identity/person-related topic name needs an explicit semantic contract and owner; avoid one name carrying detection, identity, speaker and current-user meanings."})
    classes=defaultdict(list)
    for c in data.get("classes",[]): classes[c["name"]].append(c["file"])
    for name,files in sorted(classes.items()):
        if len(files)>1: findings.append({"type":"duplicate_class_name","severity":"review","subject":name,"evidence":sorted(files),"message":"Same class name exists in multiple files; inspect for duplicate responsibility or harmless naming collision."})
    for item in data.get("python",[]):
        if len(item.get("node_classes",[]))>1: findings.append({"type":"multiple_node_classes_in_file","severity":"review","subject":item["path"],"evidence":item["node_classes"],"message":"Multiple ROS node classes share one file; verify whether this is an intentional boundary or hidden coupling."})
    launch_packages={x.get("package") for x in data.get("launches",[]) if x.get("package")}
    for pkg in sorted({c.get("package") for c in data.get("classes",[]) if c.get("is_node")} - launch_packages - {None}):
        findings.append({"type":"node_package_without_static_launch","severity":"review","subject":pkg,"evidence":[c["file"] for c in data["classes"] if c.get("is_node") and c.get("package")==pkg],"message":"Node classes exist in this package but no literal launch Node entry was found. They may be composed, started elsewhere, or orphaned."})
    out={"schema_version":2,"summary":{"findings":len(findings),"multiple_publishers":sum(x["type"]=="multiple_publishers" for x in findings),"unpaired_interfaces":sum(x["type"] in {"publish_without_static_subscriber","subscribe_without_static_publisher"} for x in findings),"semantic_topic_names":sum(x["type"]=="semantic_topic_name_review" for x in findings),"duplicate_class_names":sum(x["type"]=="duplicate_class_name" for x in findings),"launch_review":sum(x["type"]=="node_package_without_static_launch" for x in findings)},"findings":findings}
    a.json.parent.mkdir(parents=True,exist_ok=True); a.markdown.parent.mkdir(parents=True,exist_ok=True)
    a.json.write_text(json.dumps(out,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    lines=["# Architecture Findings","","Deterministic review candidates. Human review is required; no finding is an automatic merge/delete recommendation.","","## Summary",""]+[f"- {k}: **{v}**" for k,v in out["summary"].items()]+["","## Findings",""]
    for i,f in enumerate(findings,1):
        lines += [f"### {i}. {f['type']} — {f['subject']}","","**Severity:** "+f["severity"],"",f["message"],"","**Evidence:**"]+[f"- {json.dumps(e,ensure_ascii=False)}" for e in f["evidence"]]+[""]
    if not findings: lines.append("No structural review candidates were detected.")
    a.markdown.write_text("\n".join(lines),encoding="utf-8"); print(json.dumps(out["summary"],ensure_ascii=False,sort_keys=True))
if __name__=="__main__": main()
