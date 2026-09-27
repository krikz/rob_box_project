#!/usr/bin/env python3
"""Build a deterministic static architecture inventory for ROB-BOX."""
from __future__ import annotations
import argparse, ast, json
from collections import defaultdict
from pathlib import Path
import yaml

SKIP_DIRS={".git",".venv","venv","__pycache__","build","install","log","node_modules",".mypy_cache",".pytest_cache"}
ROS_CALLS={"create_subscription":"subscribe","create_publisher":"publish","create_service":"service","create_client":"client","create_action_client":"action_client"}
ROS_NAME_ARG={name:1 for name in ROS_CALLS}
NODE_BASES={"Node","LifecycleNode","ComposableNode"}

def literal(node):
    try: return ast.literal_eval(node)
    except Exception: return None

def dotted_name(node):
    if isinstance(node,ast.Name): return node.id
    if isinstance(node,ast.Attribute):
        parent=dotted_name(node.value)
        return f"{parent}.{node.attr}" if parent else node.attr
    if isinstance(node,ast.Subscript): return dotted_name(node.value)
    return None

def package_for(path,src):
    try: rel=path.relative_to(src)
    except ValueError: return None
    return rel.parts[0] if rel.parts else None

def scan_python(src):
    files=[]; classes=[]; interfaces=[]
    if not src.exists(): return files,classes,interfaces
    for path in sorted(src.rglob("*.py")):
        if any(part in SKIP_DIRS for part in path.parts): continue
        rel=path.relative_to(src.parent).as_posix(); package=package_for(path,src)
        try: tree=ast.parse(path.read_text(encoding="utf-8"),filename=rel)
        except (OSError,SyntaxError,UnicodeDecodeError): continue
        node_classes=[]; class_by_line=[]
        for cls in [x for x in ast.walk(tree) if isinstance(x,ast.ClassDef)]:
            bases={dotted_name(base) for base in cls.bases}
            is_node=bool(bases & NODE_BASES) or any(base and base.rsplit(".",1)[-1] in NODE_BASES for base in bases)
            if is_node: node_classes.append(cls.name)
            class_by_line.append((cls.lineno,cls.name,is_node))
            doc=ast.get_docstring(cls)
            classes.append({"name":cls.name,"file":rel,"line":cls.lineno,"package":package,"is_node":is_node,
                            "bases":sorted(x for x in bases if x),
                            "methods":[x.name for x in cls.body if isinstance(x,(ast.FunctionDef,ast.AsyncFunctionDef))],
                            "docstring":doc.splitlines()[0] if doc else None})
        files.append({"path":rel,"package":package,"node_classes":sorted(node_classes)})
        for call in [x for x in ast.walk(tree) if isinstance(x,ast.Call)]:
            method=call.func.attr if isinstance(call.func,ast.Attribute) else None
            if method not in ROS_CALLS or len(call.args)<=ROS_NAME_ARG[method]: continue
            name=literal(call.args[ROS_NAME_ARG[method]])
            if not isinstance(name,str): continue
            owner=None
            for line,cls_name,is_node in sorted(class_by_line):
                if is_node and line<=call.lineno: owner=cls_name
            interfaces.append({"kind":ROS_CALLS[method],"name":name,"file":rel,"line":call.lineno,
                               "package":package,"node_class":owner,"type":dotted_name(call.args[0]) if call.args else None})
    return files,classes,interfaces

def scan_compose(path):
    if not path.exists(): return []
    data=yaml.safe_load(path.read_text(encoding="utf-8")) or {}; result=[]
    for service,cfg in (data.get("services") or {}).items():
        if not isinstance(cfg,dict): continue
        deps=cfg.get("depends_on") or []
        if isinstance(deps,dict): deps=list(deps)
        result.append({"name":service,"container_name":cfg.get("container_name"),"image":cfg.get("image"),
                       "profiles":cfg.get("profiles",[]),"network_mode":cfg.get("network_mode"),
                       "privileged":bool(cfg.get("privileged")),"depends_on":sorted(deps),
                       "command":cfg.get("command"),"entrypoint":cfg.get("entrypoint"),
                       "restart":cfg.get("restart"),"healthcheck":bool(cfg.get("healthcheck"))})
    return result

def scan_launch_files(root):
    result=[]
    for path in sorted(root.rglob("*.launch.py")):
        if any(part in SKIP_DIRS for part in path.parts): continue
        rel=path.relative_to(root).as_posix()
        try: tree=ast.parse(path.read_text(encoding="utf-8"),filename=rel)
        except (OSError,SyntaxError,UnicodeDecodeError): continue
        for call in [x for x in ast.walk(tree) if isinstance(x,ast.Call)]:
            if not isinstance(call.func,ast.Name) or call.func.id!="Node": continue
            values={}
            for kw in call.keywords:
                if kw.arg in {"package","executable","name","namespace","output"}: values[kw.arg]=literal(kw.value)
            if values: result.append({"file":rel,**values})
    return result

def scan_packages(src):
    if not src.exists(): return []
    return [{"name":p.name,"path":p.relative_to(src.parent).as_posix()} for p in sorted(src.iterdir()) if p.is_dir() and (p/"package.xml").exists()]

def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--root",type=Path,default=Path("."))
    parser.add_argument("--output",type=Path,default=Path("architecture/inventory.json"))
    parser.add_argument("--markdown",type=Path,default=Path("architecture/inventory.md")); args=parser.parse_args()
    root=args.root.resolve()
    compose=[]
    for path in [root/"docker/main/docker-compose.yaml",root/"docker/vision/docker-compose.yaml",root/"docker/quest/docker-compose.yaml"]:
        services=scan_compose(path)
        if services: compose.append({"file":path.relative_to(root).as_posix(),"services":services})
    python_files,classes,interfaces=scan_python(root/"src"); packages=scan_packages(root/"src"); launches=scan_launch_files(root)
    topics=defaultdict(lambda:{"publishers":[],"subscribers":[],"services":[],"clients":[],"actions":[],"files":[],"types":[]})
    for item in interfaces:
        t=topics[item["name"]]; t["files"].append(item["file"])
        if item["type"]: t["types"].append(item["type"])
        key={"publish":"publishers","subscribe":"subscribers","service":"services","client":"clients","action_client":"actions"}.get(item["kind"])
        if key: t[key].append({"file":item["file"],"package":item["package"],"node_class":item["node_class"],"line":item["line"]})
    for t in topics.values():
        t["files"]=sorted(set(t["files"])); t["types"]=sorted(set(t["types"]))
    inventory={"schema_version":2,"repository":{"root":str(root)},"containers":compose,"packages":packages,"launches":launches,
               "python":python_files,"classes":sorted(classes,key=lambda x:(x["file"],x["line"])),"ros_interfaces":interfaces,
               "topics":[{"name":n,**v} for n,v in sorted(topics.items())],
               "summary":{"containers":sum(len(x["services"]) for x in compose),"packages":len(packages),
                          "python_files":len(python_files),"classes":len(classes),"node_classes":sum(x["is_node"] for x in classes),
                          "ros_interfaces":len(interfaces),"unique_interfaces":len(topics),"launch_nodes":len(launches)}}
    args.output.parent.mkdir(parents=True,exist_ok=True); args.markdown.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(inventory,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    lines=["# Architecture Inventory","","Deterministic static evidence; not an architectural verdict.","","## Summary",""]
    lines += [f"- {k}: **{v}**" for k,v in inventory["summary"].items()]
    lines += ["","## Launch nodes","","| Package | Executable | Name | Namespace | File |","|---|---|---|---|---|"]
    for n in launches: lines.append(f"| {n.get('package') or '—'} | {n.get('executable') or '—'} | {n.get('name') or '—'} | {n.get('namespace') or '—'} | {n['file']} |")
    lines += ["","## ROS interfaces","","| Kind | Name | Type | Package | Node class | File | Line |","|---|---|---|---|---|---|---|"]
    for i in interfaces: lines.append(f"| {i['kind']} | {i['name']} | {i.get('type') or '—'} | {i.get('package') or '—'} | {i.get('node_class') or '—'} | {i['file']} | {i['line']} |")
    lines += ["","## Architectural classes","","| Class | Node? | Package | File | Bases | Responsibility hint |","|---|:---:|---|---|---|---|"]
    for cls in inventory["classes"]:
        doc=(cls["docstring"] or "—").replace("|","\\|")
        lines.append(f"| {cls['name']} | {'yes' if cls['is_node'] else 'no'} | {cls.get('package') or '—'} | {cls['file']} | {', '.join(cls['bases']) or '—'} | {doc} |")
    lines += ["","## Interpretation","","Static evidence must be reconciled with launch configuration and the live ROS graph.",""]
    args.markdown.write_text("\n".join(lines),encoding="utf-8"); print(json.dumps(inventory["summary"],ensure_ascii=False,sort_keys=True))
if __name__=="__main__": main()
