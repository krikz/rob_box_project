#!/usr/bin/env python3
"""Build a deterministic static architecture inventory for ROB-BOX."""
from __future__ import annotations
import argparse, ast, json
from collections import defaultdict
from pathlib import Path
import yaml

SKIP_DIRS = {".git", ".venv", "venv", "__pycache__", "build", "install", "log", "node_modules", ".mypy_cache", ".pytest_cache"}
ROS_CALLS = {
    "create_subscription": "subscribe",
    "create_publisher": "publish",
    "create_service": "service",
    "create_client": "client",
    "create_action_client": "action_client",
}
ROS_NAME_ARG = {
    "create_subscription": 1,
    "create_publisher": 1,
    "create_service": 1,
    "create_client": 1,
    "create_action_client": 1,
}
NODE_BASES = {"Node", "LifecycleNode", "ComposableNode"}

def literal(node):
    try:
        return ast.literal_eval(node)
    except Exception:
        return None

def dotted_name(node):
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        parent = dotted_name(node.value)
        return f"{parent}.{node.attr}" if parent else node.attr
    return None

def scan_python(src: Path):
    files, classes, interfaces = [], [], []
    for path in sorted(src.rglob("*.py")):
        if any(part in SKIP_DIRS for part in path.parts):
            continue
        rel = path.relative_to(src.parent).as_posix()
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=rel)
        except (OSError, SyntaxError, UnicodeDecodeError):
            continue

        node_classes = []
        for cls in [x for x in ast.walk(tree) if isinstance(x, ast.ClassDef)]:
            bases = {dotted_name(base) for base in cls.bases}
            if bases & NODE_BASES or any(
                base and base.rsplit(".", 1)[-1] in NODE_BASES for base in bases
            ):
                node_classes.append(cls.name)

            doc = ast.get_docstring(cls)
            classes.append({
                "name": cls.name,
                "file": rel,
                "line": cls.lineno,
                "bases": sorted(x for x in bases if x),
                "methods": [
                    x.name for x in cls.body
                    if isinstance(x, (ast.FunctionDef, ast.AsyncFunctionDef))
                ],
                "docstring": doc.splitlines()[0] if doc else None,
            })

        files.append({"path": rel, "node_classes": sorted(node_classes)})

        for call in [x for x in ast.walk(tree) if isinstance(x, ast.Call)]:
            method = call.func.attr if isinstance(call.func, ast.Attribute) else None
            if method not in ROS_CALLS:
                continue
            name_index = ROS_NAME_ARG[method]
            if len(call.args) <= name_index:
                continue
            interface_name = literal(call.args[name_index])
            if not isinstance(interface_name, str):
                continue
            interfaces.append({
                "kind": ROS_CALLS[method],
                "name": interface_name,
                "file": rel,
                "line": call.lineno,
                "node_classes": sorted(node_classes),
            })
    return files, classes, interfaces

def scan_compose(path: Path):
    if not path.exists():
        return []
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    result = []
    for service, cfg in (data.get("services") or {}).items():
        if not isinstance(cfg, dict):
            continue
        deps = cfg.get("depends_on") or []
        if isinstance(deps, dict):
            deps = list(deps)
        result.append({
            "name": service,
            "container_name": cfg.get("container_name"),
            "image": cfg.get("image"),
            "profiles": cfg.get("profiles", []),
            "network_mode": cfg.get("network_mode"),
            "privileged": bool(cfg.get("privileged")),
            "depends_on": sorted(deps),
            "command": cfg.get("command"),
            "entrypoint": cfg.get("entrypoint"),
            "restart": cfg.get("restart"),
            "healthcheck": bool(cfg.get("healthcheck")),
        })
    return result

def scan_packages(src: Path):
    if not src.exists():
        return []
    return [
        {"name": path.name, "path": path.relative_to(src.parent).as_posix()}
        for path in sorted(src.iterdir())
        if path.is_dir() and (path / "package.xml").exists()
    ]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("."))
    parser.add_argument("--output", type=Path, default=Path("architecture/inventory.json"))
    parser.add_argument("--markdown", type=Path, default=Path("architecture/inventory.md"))
    args = parser.parse_args()
    root = args.root.resolve()

    compose = []
    for path in [
        root / "docker/main/docker-compose.yaml",
        root / "docker/vision/docker-compose.yaml",
        root / "docker/quest/docker-compose.yaml",
    ]:
        services = scan_compose(path)
        if services:
            compose.append({
                "file": path.relative_to(root).as_posix(),
                "services": services,
            })

    python_files, classes, interfaces = scan_python(root / "src")
    packages = scan_packages(root / "src")

    topics = defaultdict(lambda: {
        "publishers": [], "subscribers": [], "services": [], "clients": [], "files": []
    })
    for item in interfaces:
        topic = topics[item["name"]]
        topic["files"].append(item["file"])
        key = {
            "publish": "publishers",
            "subscribe": "subscribers",
            "service": "services",
            "client": "clients",
        }.get(item["kind"])
        if key:
            topic[key].append(item["file"])

    for topic in topics.values():
        for key in topic:
            topic[key] = sorted(set(topic[key]))

    inventory = {
        "schema_version": 1,
        "repository": {"root": str(root)},
        "containers": compose,
        "packages": packages,
        "python": python_files,
        "classes": sorted(classes, key=lambda x: (x["file"], x["line"])),
        "ros_interfaces": interfaces,
        "topics": [
            {"name": name, **value}
            for name, value in sorted(topics.items())
        ],
        "summary": {
            "containers": sum(len(x["services"]) for x in compose),
            "packages": len(packages),
            "python_files": len(python_files),
            "classes": len(classes),
            "ros_interfaces": len(interfaces),
            "unique_interfaces": len(topics),
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(inventory, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    lines = [
        "# Architecture Inventory", "",
        f"Static inventory generated from {root}.", "",
        "## Summary", "",
    ]
    lines += [f"- {key}: **{value}**" for key, value in inventory["summary"].items()]
    lines += [
        "", "## Containers", "",
        "| Compose | Service | Image | Depends on | Healthcheck |",
        "|---|---|---|---|---|",
    ]
    for stack in compose:
        for service in stack["services"]:
            lines.append(
                f"| {stack['file']} | {service['name']} | "
                f"{service['image'] or '—'} | "
                f"{', '.join(service['depends_on']) or '—'} | "
                f"{'yes' if service['healthcheck'] else 'no'} |"
            )

    lines += ["", "## Node classes / ROS interfaces", ""]
    for item in python_files:
        if not item["node_classes"]:
            continue
        lines += [
            f"### {item['path']}", "",
            "Node classes: " + ", ".join(item["node_classes"]),
        ]
        for interface in interfaces:
            if interface["file"] == item["path"]:
                lines.append(
                    f"- {interface['kind']} {interface['name']} "
                    f"(line {interface['line']})"
                )
        lines.append("")

    lines += [
        "## Architectural classes", "",
        "| Class | File | Bases | Responsibility hint |",
        "|---|---|---|---|",
    ]
    for cls in inventory["classes"]:
        lines.append(
            f"| {cls['name']} | {cls['file']} | "
            f"{', '.join(cls['bases']) or '—'} | "
            f"{(cls['docstring'] or '—').replace('|', '\\|')} |"
        )

    lines += [
        "", "## Interpretation", "",
        "This file is evidence, not an architectural verdict.",
        "Next layers add launch/entrypoint mapping, runtime ROS graph comparison,",
        "feature/capability ownership, duplicate detection and semantic topic review.",
        "",
    ]
    args.markdown.write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(inventory["summary"], ensure_ascii=False, sort_keys=True))

if __name__ == "__main__":
    main()
