#!/usr/bin/env python3
"""Capture a ROS 2 runtime graph for comparison with the static inventory."""
from __future__ import annotations
import argparse, json, shutil, subprocess, time
from pathlib import Path

def run(*args: str) -> list[str]:
    if not shutil.which(args[0]): raise SystemExit("ros2 CLI not found")
    p = subprocess.run(args, text=True, capture_output=True, timeout=30)
    if p.returncode: raise SystemExit(f"{args} failed: {p.stderr.strip()}")
    return [x.strip() for x in p.stdout.splitlines() if x.strip()]

def main() -> int:
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,default=Path("architecture/runtime.json"))
    args=parser.parse_args()
    nodes=run("ros2","node","list")
    topics=run("ros2","topic","list")
    services=run("ros2","service","list")
    actions=run("ros2","action","list")
    topic_info={}
    for topic in topics:
        try:
            p=subprocess.run(("ros2","topic","info",topic,"--verbose"),text=True,capture_output=True,timeout=10)
            topic_info[topic]={"returncode":p.returncode,"raw":p.stdout}
        except subprocess.TimeoutExpired:
            topic_info[topic]={"returncode":124,"raw":"timeout"}
    snapshot={"schema_version":1,"captured_at_unix":time.time(),"nodes":nodes,"topics":topics,"services":services,"actions":actions,"topic_info":topic_info}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(snapshot,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({k:len(v) if isinstance(v,list) else len(v) for k,v in snapshot.items() if k in {"nodes","topics","services","actions","topic_info"}},ensure_ascii=False,sort_keys=True))
    return 0

if __name__=="__main__": raise SystemExit(main())