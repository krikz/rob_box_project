#!/usr/bin/env python3
"""Capture a real ROS 2 graph for comparison with static architecture evidence."""
from __future__ import annotations
import argparse, json, os, shlex, shutil, subprocess, time
from pathlib import Path

def local_run(args, timeout=30):
    if not shutil.which(args[0]):
        raise RuntimeError("ros2 CLI not found on this runner")
    p=subprocess.run(args,text=True,capture_output=True,timeout=timeout)
    return {"returncode":p.returncode,"stdout":p.stdout,"stderr":p.stderr}

def remote_run(host,user,args,timeout=30):
    target=f"{user}@{host}" if user else host
    command=" ".join(shlex.quote(x) for x in args)
    p=subprocess.run(["ssh","-o","BatchMode=yes","-o","ConnectTimeout=10",target,command],
                     text=True,capture_output=True,timeout=timeout)
    return {"returncode":p.returncode,"stdout":p.stdout,"stderr":p.stderr}

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",type=Path,default=Path("architecture/runtime.json"))
    p.add_argument("--topics",default="",help="Comma-separated topic names to inspect deeply; empty means all")
    p.add_argument("--remote-host",default="")
    p.add_argument("--remote-user",default="")
    a=p.parse_args()
    remote=bool(a.remote_host)
    runner=remote_run if remote else local_run
    kwargs={"host":a.remote_host,"user":a.remote_user} if remote else {}

    def call(*args,timeout=30):
        result=runner(*args,timeout=timeout,**kwargs)
        if result["returncode"] != 0:
            raise RuntimeError(f"{' '.join(args)} failed: {result['stderr'].strip()}")
        return [x.strip() for x in result["stdout"].splitlines() if x.strip()]

    nodes=call("ros2","node","list")
    topics=call("ros2","topic","list")
    services=call("ros2","service","list")
    actions=call("ros2","action","list")
    selected=[x.strip() for x in a.topics.split(",") if x.strip()]
    inspect_topics=selected or topics
    topic_info={}
    for topic in inspect_topics:
        if topic not in topics:
            topic_info[topic]={"present":False}
            continue
        result=runner("ros2","topic","info",topic,"--verbose",timeout=15,**kwargs)
        raw=result["stdout"]
        publishers=[]; subscribers=[]; mode=None; current={}
        for line in raw.splitlines():
            s=line.strip()
            if s.startswith("Publisher count:"): mode="publishers"
            elif s.startswith("Subscription count:"): mode="subscribers"
            elif s.startswith("Node name:"):
                current["node"]=s.split(":",1)[1].strip()
                if mode:
                    (publishers if mode=="publishers" else subscribers).append(dict(current))
                current={}
            elif s.startswith("Node namespace:"):
                current["namespace"]=s.split(":",1)[1].strip()
        topic_info[topic]={
            "present":result["returncode"]==0,
            "returncode":result["returncode"],
            "publishers":publishers,
            "subscribers":subscribers,
            "raw":raw,
        }

    snapshot={
        "schema_version":2,
        "captured_at_unix":time.time(),
        "captured_from":"remote:"+a.remote_host if remote else "local",
        "ros_domain_id":os.getenv("ROS_DOMAIN_ID"),
        "rmw_implementation":os.getenv("RMW_IMPLEMENTATION"),
        "nodes":nodes,"topics":topics,"services":services,"actions":actions,
        "topic_info":topic_info,
    }
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(snapshot,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({
        "nodes":len(nodes),"topics":len(topics),"services":len(services),
        "actions":len(actions),"inspected_topics":len(topic_info)
    },ensure_ascii=False,sort_keys=True))

if __name__=="__main__":
    main()
