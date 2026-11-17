#!/usr/bin/env python3
"""Deterministic review candidates from a live ROS 2 graph (runtime.json).

Static findings (architecture_findings.py) only see literal names in Python.
This tool reads what really runs on the robot and reports:

* ``dead_input``   — topic has subscribers but no publisher;
* ``dead_output``  — topic has publishers but no subscriber;
* ``type_mismatch`` — endpoints of one topic use different message types
  (they never match, data silently does not flow);
* ``qos_incompatible`` — BEST_EFFORT publisher -> RELIABLE subscriber, or
  VOLATILE publisher -> TRANSIENT_LOCAL subscriber (no data on that pair);
* ``multiple_writers`` — more than one node publishes the topic;
* ``duplicate_publishers`` — one node holds several publishers on one topic.

Infrastructure topics and library helper nodes are ignored (same lists as
architecture_runtime_graph.py). Each finding has a stable ``key``; with
``--baseline`` findings are split into known and NEW, and ``--fail-on-new``
turns new findings into a non-zero exit.

``scope`` is ``repo`` when one of the endpoint nodes is ours (its name is
created in ``src/``: ``Node('x')``, ``NODE_NAME = 'x'``, a setup.py entry point
or a launch ``name='x'``) or the topic is declared in the static inventory;
``external`` otherwise (nav2, rtabmap, drivers).
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from architecture_runtime_graph import (  # noqa: E402
    LAUNCH_NAME,
    RepoIndex,
    is_helper_node,
    is_infra_topic,
    names_from_python_module,
)
from architecture_runtime_snapshot import parse_topic_info  # noqa: E402

TITLES = {
    "type_mismatch": "Type mismatch (data never flows)",
    "qos_incompatible": "Incompatible QoS (data never flows on that pair)",
    "dead_input": "Dead inputs: subscribers without any publisher",
    "dead_output": "Dead outputs: publishers without any subscriber",
    "multiple_writers": "Multiple writer nodes on one topic",
    "duplicate_publishers": "One node holds several publishers on one topic",
}
ORDER = list(TITLES)
NODE_NAME_CONSTANT = re.compile(r"\bNODE_NAME\s*(?::\s*str)?\s*=\s*['\"]([\w/]+)['\"]")
TEST_PATH = re.compile(r"(^|/)(test|tests)/|(^|/)test_[^/]*\.py$|/scripts/(example|test)_")


def repo_node_names(repo_root):
    """Leaf names of ROS nodes created by code in ``src/`` (tests excluded)."""
    root = Path(repo_root)
    names = set()
    for path in (root / "src").rglob("*.py"):
        rel = path.relative_to(root).as_posix()
        if TEST_PATH.search(rel) or {"build", "install"} & set(path.parts):
            continue
        names.update(names_from_python_module(path))
        text = path.read_text(encoding="utf-8", errors="ignore")
        names.update(NODE_NAME_CONSTANT.findall(text))
        if path.name.endswith("launch.py"):
            names.update(LAUNCH_NAME.findall(text))
    index = RepoIndex(root)
    index.entry_point("")
    names.update(index._entry_points or {})
    return {name.rsplit("/", 1)[-1] for name in names}


def fq_node(endpoint):
    namespace = (endpoint.get("namespace") or "/").rstrip("/")
    return f"{namespace}/{endpoint.get('node')}"


def endpoints(info):
    """Publishers/subscribers of one topic; re-parse ``raw`` when present.

    Snapshots taken before the parser fix (#3103) stored shifted namespaces
    and no type/QoS; the raw ``ros2 topic info --verbose`` text is the truth.
    """
    if info.get("raw"):
        return parse_topic_info(info["raw"])
    return info.get("publishers", []), info.get("subscribers", [])


def static_names(inventory):
    if not inventory:
        return set()
    return {t["name"] for t in inventory.get("topics", []) if t.get("name", "").startswith("/")}


def finding(kind, topic, detail, evidence, scope):
    return {
        "type": kind,
        "topic": topic,
        "detail": detail,
        "key": f"{kind}|{topic}|{detail}",
        "scope": scope,
        "evidence": evidence,
    }


def analyze(runtime, inventory=None, own_nodes=None):
    declared = static_names(inventory)
    own = own_nodes or set()
    findings = []
    for topic in sorted(runtime.get("topic_info", {})):
        if is_infra_topic(topic):
            continue
        pubs, subs = endpoints(runtime["topic_info"][topic])
        pubs = [e for e in pubs if not is_helper_node(fq_node(e))]
        subs = [e for e in subs if not is_helper_node(fq_node(e))]
        ours = any(e.get("node", "").rsplit("/", 1)[-1] in own for e in pubs + subs)
        scope = "repo" if ours or topic in declared else "external"
        writers = sorted({fq_node(e) for e in pubs})
        readers = sorted({fq_node(e) for e in subs})

        if readers and not writers:
            findings.append(finding("dead_input", topic, "", {"subscribers": readers}, scope))
        if writers and not readers:
            findings.append(finding("dead_output", topic, "", {"publishers": writers}, scope))

        types = Counter(e["type"] for e in pubs + subs if e.get("type"))
        if len(types) > 1:
            by_type = {t: sorted({fq_node(e) for e in pubs + subs if e.get("type") == t}) for t in sorted(types)}
            findings.append(finding("type_mismatch", topic, " vs ".join(sorted(types)), by_type, scope))

        bad_pairs = set()
        for p in pubs:
            for s in subs:
                if p.get("reliability") == "BEST_EFFORT" and s.get("reliability") == "RELIABLE":
                    bad_pairs.add((fq_node(p), fq_node(s), "reliability BEST_EFFORT -> RELIABLE"))
                if p.get("durability") == "VOLATILE" and s.get("durability") == "TRANSIENT_LOCAL":
                    bad_pairs.add((fq_node(p), fq_node(s), "durability VOLATILE -> TRANSIENT_LOCAL"))
        for pub, sub, why in sorted(bad_pairs):
            findings.append(
                finding("qos_incompatible", topic, f"{pub} -> {sub}", {"why": why, "pub": pub, "sub": sub}, scope)
            )

        if len(writers) > 1:
            findings.append(finding("multiple_writers", topic, "", {"publishers": writers}, scope))

        for node, count in sorted(Counter(fq_node(e) for e in pubs).items()):
            if count > 1:
                findings.append(
                    finding("duplicate_publishers", topic, node, {"node": node, "publishers": count}, scope)
                )
    return findings


def load_baseline(path):
    if not path or not Path(path).exists():
        return set()
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return {item["key"] for item in data.get("known", [])}


def build_report(runtime, inventory=None, baseline=None, own_nodes=None):
    findings = analyze(runtime, inventory, own_nodes)
    known = baseline or set()
    for item in findings:
        item["new"] = bool(known) and item["key"] not in known
    keys = {f["key"] for f in findings}
    counts = Counter(f["type"] for f in findings)
    return {
        "schema_version": 1,
        "captured_from": runtime.get("captured_from"),
        "captured_at_unix": runtime.get("captured_at_unix"),
        "baseline_used": bool(known),
        "summary": {
            "findings": len(findings),
            "repo_scope": sum(f["scope"] == "repo" for f in findings),
            "new": sum(f["new"] for f in findings),
            "resolved_since_baseline": len(known - keys),
            **{kind: counts.get(kind, 0) for kind in ORDER},
        },
        "resolved": sorted(known - keys),
        "findings": sorted(findings, key=lambda f: (ORDER.index(f["type"]), f["scope"] != "repo", f["key"])),
    }


def evidence_text(item):
    ev = item["evidence"]
    if item["type"] == "type_mismatch":
        return "; ".join(f"`{t}`: {', '.join(nodes)}" for t, nodes in ev.items())
    if item["type"] == "qos_incompatible":
        return ev["why"]
    if item["type"] == "duplicate_publishers":
        return f"{ev['publishers']} publishers"
    nodes = ev.get("publishers") or ev.get("subscribers") or []
    return ", ".join(nodes)


def render_markdown(report):
    lines = [
        "# Runtime ROS 2 Findings",
        "",
        "Review candidates from the live graph. `repo` = our node takes part or the topic is declared in our code; "
        "`external` = nav2/rtabmap/drivers only.",
        "A snapshot shows only what was connected at capture time.",
        "",
        "## Summary",
        "",
    ]
    lines += [f"- {k}: **{v}**" for k, v in report["summary"].items()]
    if report["baseline_used"] and report["resolved"]:
        lines += ["", "## Resolved since baseline", ""] + [f"- `{k}`" for k in report["resolved"]]
    for kind in ORDER:
        items = [f for f in report["findings"] if f["type"] == kind]
        if not items:
            continue
        lines += ["", f"## {TITLES[kind]} ({len(items)})", ""]
        for scope in ("repo", "external"):
            scoped = [f for f in items if f["scope"] == scope]
            if not scoped:
                continue
            rows = ["| | Topic | Detail | Evidence |", "|---|---|---|---|"]
            for f in scoped:
                mark = "🆕" if f["new"] else ""
                rows.append(f"| {mark} | `{f['topic']}` | {f['detail'] or '—'} | {evidence_text(f)} |")
            if scope == "repo":
                lines += rows
            else:
                lines += ["", f"<details><summary>external ({len(scoped)})</summary>", ""] + rows + ["", "</details>"]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runtime", type=Path)
    parser.add_argument("--inventory", type=Path, help="static inventory.json, used for repo/external scope")
    parser.add_argument("--repo-root", type=Path, default=Path("."), help="checkout used to recognise our nodes")
    parser.add_argument("--baseline", type=Path, help="known findings; new ones are marked")
    parser.add_argument("--fail-on-new", action="store_true", help="exit 1 when findings not in the baseline appear")
    parser.add_argument("--write-baseline", type=Path, help="write current findings as a baseline file")
    parser.add_argument("--json", type=Path, default=Path("architecture/runtime-findings.json"))
    parser.add_argument("--markdown", type=Path, default=Path("architecture/runtime-findings.md"))
    args = parser.parse_args()

    runtime = json.loads(args.runtime.read_text(encoding="utf-8"))
    inventory = json.loads(args.inventory.read_text(encoding="utf-8")) if args.inventory else None
    report = build_report(runtime, inventory, load_baseline(args.baseline), repo_node_names(args.repo_root))
    for path in (args.json, args.markdown):
        path.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    args.markdown.write_text(render_markdown(report), encoding="utf-8")
    if args.write_baseline:
        known = [{"key": f["key"], "scope": f["scope"]} for f in report["findings"]]
        args.write_baseline.parent.mkdir(parents=True, exist_ok=True)
        args.write_baseline.write_text(
            json.dumps({"schema_version": 1, "known": known}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    print(json.dumps(report["summary"], ensure_ascii=False, sort_keys=True))
    if args.fail_on_new and report["summary"]["new"]:
        print(f"::error::{report['summary']['new']} new runtime finding(s), see runtime-findings.md", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
