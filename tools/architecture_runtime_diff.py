#!/usr/bin/env python3
"""Compare static topic evidence with a runtime ROS graph."""
from __future__ import annotations
import argparse,json
from pathlib import Path
def main():
    p=argparse.ArgumentParser(); p.add_argument("inventory",type=Path); p.add_argument("runtime",type=Path); p.add_argument("--markdown",type=Path,default=Path("architecture/runtime-diff.md")); a=p.parse_args()
    inv=json.loads(a.inventory.read_text(encoding="utf-8")); run=json.loads(a.runtime.read_text(encoding="utf-8"))
    static={x["name"] for x in inv.get("topics",[])}; live=set(run.get("topics",[]))
    result={"schema_version":1,"static_topics":len(static),"runtime_topics":len(live),"common":len(static&live),"static_not_runtime":sorted(static-live),"runtime_not_static":sorted(live-static)}
    a.markdown.parent.mkdir(parents=True,exist_ok=True)
    Path("architecture/runtime-diff.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    lines=["# Static to Runtime ROS2 Diff","","Name-level comparison only. Missing runtime topics may be legitimate for inactive components; runtime-only topics may come from external packages.","",f"- Static topics: **{len(static)}**",f"- Runtime topics: **{len(live)}**",f"- Common: **{len(static&live)}**","","## Declared statically, absent at runtime",""]
    lines += [f"- {x}" for x in sorted(static-live)] or ["- None"]
    lines += ["","## Runtime-only topics",""] + ([f"- {x}" for x in sorted(live-static)] or ["- None"])
    a.markdown.write_text("\n".join(lines)+"\n",encoding="utf-8")
    print(json.dumps(result,ensure_ascii=False,sort_keys=True))
if __name__=="__main__": main()
