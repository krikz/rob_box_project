#!/usr/bin/env python3
"""Capture a real ROS 2 graph for comparison with static architecture evidence."""
from __future__ import annotations
import argparse, json, os, re, shlex, shutil, subprocess, sys, time
from pathlib import Path

def local_run(args, timeout=30):
    if not shutil.which(args[0]):
        raise RuntimeError("ros2 CLI not found on this runner")
    p=subprocess.run(args,text=True,capture_output=True,timeout=timeout)
    return {"returncode":p.returncode,"stdout":p.stdout,"stderr":p.stderr}

def remote_run(args, host="", user="", container="", timeout=30):
    target=f"{user}@{host}" if user else host
    inner=" ".join(shlex.quote(x) for x in args)
    if container:
        inner=f"source /opt/ros/humble/setup.bash && {inner}"
        command=f"docker exec {shlex.quote(container)} bash -lc {shlex.quote(inner)}"
    else:
        command=inner
    ssh=["ssh","-o","StrictHostKeyChecking=no","-o","UserKnownHostsFile=/dev/null","-o","ConnectTimeout=10","-o","ServerAliveInterval=5","-o","ServerAliveCountMax=3",target,command]
    if os.getenv("SSHPASS"):
        if not shutil.which("sshpass"):
            raise RuntimeError("SSHPASS is set but sshpass is not installed")
        ssh=["sshpass","-e",*ssh]
    p=subprocess.run(ssh,
                     text=True,capture_output=True,timeout=timeout)
    return {"returncode":p.returncode,"stdout":p.stdout,"stderr":p.stderr}

ENDPOINT_FIELDS={"Node namespace":"namespace","Topic type":"type","Endpoint type":"endpoint_type",
                 "Reliability":"reliability","Durability":"durability"}

def parse_topic_info(raw):
    """Parse ``ros2 topic info --verbose`` into (publishers, subscribers).

    Each endpoint block starts with ``Node name:`` and is followed by its own
    ``Node namespace:``, type and QoS lines, so fields are attached to the block
    opened by the last ``Node name:`` line.
    """
    publishers=[]; subscribers=[]; mode=None; current=None
    for line in raw.splitlines():
        s=line.strip()
        if s.startswith("Publisher count:"): mode="publishers"; current=None
        elif s.startswith("Subscription count:"): mode="subscribers"; current=None
        elif s.startswith("Node name:") and mode:
            current={"node":s.split(":",1)[1].strip()}
            (publishers if mode=="publishers" else subscribers).append(current)
        elif current is not None and ":" in s:
            key,value=(x.strip() for x in s.split(":",1))
            if key in ENDPOINT_FIELDS: current[ENDPOINT_FIELDS[key]]=value
    return publishers,subscribers

TOPIC_MARK="=====TOPIC "; RC_MARK="=====RC "; ENV_MARK="=====ENV "; END="====="
REMOTE_ENV=("ROS_DOMAIN_ID","RMW_IMPLEMENTATION")
# A block cut short (no RC marker) must not swallow the next topic's block.
BLOCK_RE=re.compile(r"^=====TOPIC (?P<topic>[^\n]*?)=====\n(?P<raw>(?:(?!^=====TOPIC ).)*?)=====RC (?P<rc>\d+)=====$",re.M|re.S)
ENV_RE=re.compile(r"^=====ENV (?P<name>\w+)=(?P<value>.*?)=====$",re.M)

def batch_script(topics):
    """Bash run inside the container: env block + one delimited block per topic.

    ``ros2 topic info`` stdout is kept byte-for-byte between the markers (the RC
    marker is not forced onto a new line), so ``raw`` matches the per-topic path.
    """
    lines=[f'[ -n "${{{name}+x}}" ] && printf \'{ENV_MARK}%s=%s{END}\\n\' {name} "${name}"' for name in REMOTE_ENV]
    lines.append("for t in "+" ".join(shlex.quote(t) for t in topics)+"; do" if topics else "for t in; do")
    lines.append(f'  printf \'{TOPIC_MARK}%s{END}\\n\' "$t"')
    lines.append('  ros2 topic info "$t" --verbose')
    lines.append(f'  printf \'{RC_MARK}%s{END}\\n\' "$?"')
    lines.append("done")
    return "\n".join(lines)+"\n"

def split_batch(stdout):
    """Split batch output into ({env name: value}, {topic: {"returncode", "stdout"}})."""
    env={m["name"]:m["value"] for m in ENV_RE.finditer(stdout)}
    blocks={m["topic"]:{"returncode":int(m["rc"]),"stdout":m["raw"]} for m in BLOCK_RE.finditer(stdout)}
    return env,blocks

def batch_capture(runner,topics,timeout=None,**kwargs):
    """One ssh + one docker exec for every topic; ({env}, {topic: result}) or None on failure."""
    timeout=timeout or min(600,60+2*len(topics))  # job budget is 15 min
    try:
        result=runner(["bash","-c",batch_script(topics)],timeout=timeout,**kwargs)
    except subprocess.TimeoutExpired:
        print(f"batch capture timed out after {timeout}s",file=sys.stderr)
        return None
    if result["returncode"]!=0:
        print(f"batch capture failed rc={result['returncode']}: {result['stderr'].strip()[-500:]}",file=sys.stderr)
        return None
    return split_batch(result["stdout"])

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",type=Path,default=Path("architecture/runtime.json"))
    p.add_argument("--topics",default="",help="Comma-separated topic names to inspect deeply; empty means all")
    p.add_argument("--remote-host",default="")
    p.add_argument("--remote-user",default="")
    p.add_argument("--remote-container",default="oak-d")
    p.add_argument("--per-topic",action="store_true",help="Remote: one ssh per topic instead of one batch call")
    a=p.parse_args()
    remote=bool(a.remote_host)
    runner=remote_run if remote else local_run
    kwargs={"host":a.remote_host,"user":a.remote_user,"container":a.remote_container} if remote else {}

    def call(*args,timeout=30):
        last=None
        attempts=3 if remote else 1
        for attempt in range(1, attempts + 1):
            result=runner(list(args),timeout=timeout,**kwargs)
            if result["returncode"] == 0:
                return [x.strip() for x in result["stdout"].splitlines() if x.strip()]
            last=result
            if attempt < attempts:
                time.sleep(2)
        raise RuntimeError(
            f"{' '.join(args)} failed after {attempts} attempts: "
            f"{last['stderr'].strip()}"
        )

    nodes=call("ros2","node","list")
    topics=call("ros2","topic","list")
    services=call("ros2","service","list")
    actions=call("ros2","action","list")
    selected=[x.strip() for x in a.topics.split(",") if x.strip()]
    inspect_topics=selected or topics
    present=[t for t in inspect_topics if t in topics]
    env={name:os.getenv(name) for name in REMOTE_ENV}
    batched={}; batch_used=False
    if remote:
        # ROS_DOMAIN_ID / RMW_IMPLEMENTATION must come from the container, not this runner.
        batch=batch_capture(runner,[] if a.per_topic else present,**kwargs) or batch_capture(runner,[],**kwargs)
        env={name:(batch[0].get(name) if batch else None) for name in REMOTE_ENV}
        if batch and not a.per_topic:
            batched={t:r for t,r in batch[1].items() if r["returncode"]==0 and r["stdout"].strip()}
            batch_used=True
    retried=[t for t in present if t not in batched] if batch_used else []
    if retried:
        print(f"batch: {len(batched)} topics ok, retrying {len(retried)} individually: {retried[:20]}",file=sys.stderr)
    topic_info={}
    for topic in inspect_topics:
        if topic not in topics:
            topic_info[topic]={"present":False}
            continue
        if topic in batched:
            result=batched[topic]
        else:
            result=runner(["ros2","topic","info",topic,"--verbose"],timeout=15,**kwargs)
        if remote and result["returncode"] != 0:
            for _ in range(2):
                time.sleep(2)
                result=runner(["ros2","topic","info",topic,"--verbose"],timeout=15,**kwargs)
                if result["returncode"] == 0:
                    break
        raw=result["stdout"]
        publishers,subscribers=parse_topic_info(raw)
        topic_info[topic]={
            "present":result["returncode"]==0,
            "returncode":result["returncode"],
            "publishers":publishers,
            "subscribers":subscribers,
            "raw":raw,
        }

    snapshot={
        "schema_version":3,
        "captured_at_unix":time.time(),
        "captured_from":("remote:"+a.remote_host+":container:"+a.remote_container if remote and a.remote_container else "remote:"+a.remote_host if remote else "local"),
        "ros_domain_id":env["ROS_DOMAIN_ID"],
        "rmw_implementation":env["RMW_IMPLEMENTATION"],
        "nodes":nodes,"topics":topics,"services":services,"actions":actions,
        "topic_info":topic_info,
    }
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(snapshot,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({
        "nodes":len(nodes),"topics":len(topics),"services":len(services),
        "actions":len(actions),"inspected_topics":len(topic_info),
        "batched_topics":len(batched),"retried_topics":len(retried),
    },ensure_ascii=False,sort_keys=True))

if __name__=="__main__":
    main()
