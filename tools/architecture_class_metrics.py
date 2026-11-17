#!/usr/bin/env python3
"""Per-class size, complexity, cohesion and test metrics for ROB-BOX.

Metrics (production code only; tests/examples are excluded and used only as
evidence of test references):

* ``loc`` — class span in lines; ``methods`` — own methods.
* ``wmc`` — Weighted Methods per Class: sum of method cyclomatic complexity.
* ``max_cc`` / ``max_cc_method`` — most complex method.
* ``attributes`` — distinct ``self.<name>`` assigned anywhere in the class.
* ``tcc`` — Tight Class Cohesion: share of method pairs that touch at least one
  common ``self`` attribute (1.0 = cohesive, ~0 = unrelated methods).
* ``responsibilities`` — LCOM4: connected groups of methods, where methods are
  linked by a shared ``self`` attribute or a direct ``self.method()`` call.
  More than one group means the class does several unrelated jobs.
  Dunder methods (``__init__`` etc.) and methods that touch no ``self`` state
  (``stateless_methods``: Protocol/ABC stubs, helpers) are left out.
* ``ros_endpoints`` — create_publisher/subscription/service/client/timer calls.
* ``god_class`` — Lanza & Marinescu rule adapted for Python without ATFD:
  ``wmc >= 47 and tcc < 1/3 and loc >= 500``.
* ``test_refs`` — test files that mention the class name. This is a proxy,
  NOT coverage.
* ``line_coverage`` — only when ``--coverage`` points to a coverage.py JSON
  report (``coverage json``); ``null`` otherwise. Never guessed.
"""

from __future__ import annotations

import argparse
import ast
import json
import re
from collections import defaultdict
from itertools import combinations
from pathlib import Path

SKIP_DIRS = {".git", ".venv", "venv", "__pycache__", "build", "install", "log", "node_modules"}
TEST_PATH = re.compile(r"(^|/)(test|tests)/|(^|/)test_[^/]*\.py$|(^|/)conftest\.py$|/scripts/(example|test)_")
ROS_ENDPOINT_CALLS = {
    "create_publisher",
    "create_subscription",
    "create_service",
    "create_client",
    "create_timer",
}
IDENTIFIER = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
GOD_WMC = 47
GOD_TCC = 1 / 3
GOD_LOC = 500
LARGE_FILE_LOC = 2000


def cyclomatic_complexity(node):
    """McCabe complexity of a function body (nested defs included)."""
    score = 1
    for child in ast.walk(node):
        if isinstance(child, (ast.If, ast.For, ast.AsyncFor, ast.While, ast.IfExp, ast.ExceptHandler, ast.Assert)):
            score += 1
        elif isinstance(child, ast.BoolOp):
            score += len(child.values) - 1
        elif isinstance(child, ast.comprehension):
            score += 1 + len(child.ifs)
        elif isinstance(child, ast.match_case):
            score += 1
    return score


def self_usage(method):
    """Return (self attributes touched, self methods called) inside a method."""
    attrs, calls = set(), set()
    for child in ast.walk(method):
        if isinstance(child, ast.Attribute) and isinstance(child.value, ast.Name) and child.value.id == "self":
            attrs.add(child.attr)
        if (
            isinstance(child, ast.Call)
            and isinstance(child.func, ast.Attribute)
            and isinstance(child.func.value, ast.Name)
            and child.func.value.id == "self"
        ):
            calls.add(child.func.attr)
    return attrs, calls


def assigned_attributes(cls):
    names = set()
    for child in ast.walk(cls):
        targets = []
        if isinstance(child, ast.Assign):
            targets = child.targets
        elif isinstance(child, (ast.AnnAssign, ast.AugAssign)):
            targets = [child.target]
        for target in targets:
            for node in ast.walk(target):
                if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
                    names.add(node.attr)
    return names


def cohesion(methods, state=None):
    """Return (tcc, responsibilities, groups, stateless) for {name: (attrs, calls)}.

    Constructors and other dunders touch all state by design; counting them
    would glue every group together (classic LCOM4 false negative). Methods
    that touch no ``self`` state (Protocol/ABC stubs, static helpers) are not
    responsibilities of the object's state and are counted separately.
    ``state`` is the set of attributes the class assigns itself; inherited
    helpers such as ``self.get_logger()`` are not shared state.
    """
    candidates = [name for name in methods if not (name.startswith("__") and name.endswith("__"))]
    methods = {n: ((a - set(methods)) if state is None else (a & state), c) for n, (a, c) in methods.items()}
    stateless = sorted(
        n for n in candidates if not (methods[n][0] - set(methods)) and not (methods[n][1] & set(methods))
    )
    names = sorted(n for n in candidates if n not in stateless)
    fields = {name: methods[name][0] - set(methods) for name in names}
    pairs = list(combinations(names, 2))
    tcc = sum(1 for a, b in pairs if fields[a] & fields[b]) / len(pairs) if pairs else 1.0

    parent = {name: name for name in names}

    def find(name):
        while parent[name] != name:
            parent[name] = parent[parent[name]]
            name = parent[name]
        return name

    def union(a, b):
        parent[find(a)] = find(b)

    for a, b in pairs:
        if fields[a] & fields[b]:
            union(a, b)
    for name in names:
        for called in methods[name][1]:
            if called in parent:
                union(name, called)
    groups = defaultdict(list)
    for name in names:
        groups[find(name)].append(name)
    ordered = sorted((sorted(g) for g in groups.values()), key=lambda g: (-len(g), g))
    return round(tcc, 3), len(ordered), ordered, len(stateless)


def ros_endpoints(cls):
    count = 0
    for child in ast.walk(cls):
        if isinstance(child, ast.Call) and isinstance(child.func, ast.Attribute):
            if child.func.attr in ROS_ENDPOINT_CALLS:
                count += 1
    return count


def load_coverage(path, repo_root):
    """Map repo-relative file -> set of executed lines from coverage.py JSON."""
    if not path:
        return None
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    result = {}
    for name, info in data.get("files", {}).items():
        file_path = Path(name)
        if file_path.is_absolute():
            try:
                file_path = file_path.relative_to(repo_root)
            except ValueError:
                continue
        result[file_path.as_posix()] = (set(info.get("executed_lines", [])), set(info.get("missing_lines", [])))
    return result


def class_coverage(coverage, rel, start, end):
    if coverage is None or rel not in coverage:
        return None
    executed, missing = coverage[rel]
    hit = sum(1 for line in executed if start <= line <= end)
    miss = sum(1 for line in missing if start <= line <= end)
    return round(hit / (hit + miss), 3) if hit + miss else None


def iter_python(src):
    for path in sorted(src.rglob("*.py")):
        if SKIP_DIRS & set(path.parts):
            continue
        yield path


def collect(repo_root, coverage_path=None):
    repo_root = Path(repo_root).resolve()
    src = repo_root / "src"
    coverage = load_coverage(coverage_path, repo_root)
    production, tests = [], []
    for path in iter_python(src):
        rel = path.relative_to(repo_root).as_posix()
        (tests if TEST_PATH.search(rel) else production).append((path, rel))

    test_refs = defaultdict(set)
    for path, rel in tests:
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        for identifier in set(IDENTIFIER.findall(text)):
            test_refs[identifier].add(rel)

    classes, files = [], []
    for path, rel in production:
        try:
            text = path.read_text(encoding="utf-8")
            tree = ast.parse(text, filename=rel)
        except (OSError, SyntaxError, UnicodeDecodeError):
            continue
        files.append({"file": rel, "loc": len(text.splitlines())})
        for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
            own = [m for m in cls.body if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef))]
            complexity = {m.name: cyclomatic_complexity(m) for m in own}
            usage = {m.name: self_usage(m) for m in own}
            tcc, responsibilities, groups, stateless = cohesion(usage, assigned_attributes(cls))
            loc = cls.end_lineno - cls.lineno + 1
            wmc = sum(complexity.values())
            max_method = max(complexity, key=complexity.get) if complexity else None
            refs = sorted(test_refs.get(cls.name, ()))
            classes.append(
                {
                    "name": cls.name,
                    "file": rel,
                    "line": cls.lineno,
                    "package": rel.split("/")[1] if rel.count("/") > 1 else None,
                    "loc": loc,
                    "methods": len(own),
                    "wmc": wmc,
                    "max_cc": complexity[max_method] if max_method else 0,
                    "max_cc_method": max_method,
                    "attributes": len(assigned_attributes(cls)),
                    "tcc": tcc,
                    "responsibilities": responsibilities,
                    "responsibility_groups": groups,
                    "stateless_methods": stateless,
                    "ros_endpoints": ros_endpoints(cls),
                    "god_class": wmc >= GOD_WMC and tcc < GOD_TCC and loc >= GOD_LOC,
                    "test_refs": len(refs),
                    "test_ref_files": refs[:10],
                    "line_coverage": class_coverage(coverage, rel, cls.lineno, cls.end_lineno),
                }
            )
    classes.sort(key=lambda c: (c["file"], c["line"]))
    files.sort(key=lambda f: -f["loc"])
    return {
        "schema_version": 1,
        "thresholds": {"god_wmc": GOD_WMC, "god_tcc": round(GOD_TCC, 3), "god_loc": GOD_LOC},
        "coverage_source": str(coverage_path) if coverage_path else None,
        "summary": {
            "production_files": len(production),
            "test_files": len(tests),
            "classes": len(classes),
            "god_classes": sum(c["god_class"] for c in classes),
            "classes_without_test_refs": sum(1 for c in classes if c["test_refs"] == 0),
            "large_files": sum(1 for f in files if f["loc"] >= LARGE_FILE_LOC),
        },
        "classes": classes,
        "large_files": [f for f in files if f["loc"] >= LARGE_FILE_LOC],
    }


def fmt_cov(value):
    return "—" if value is None else f"{value:.0%}"


def render_markdown(data, top=25):
    classes = data["classes"]
    cov = data["coverage_source"]
    lines = [
        "# Class Metrics",
        "",
        "Static metrics for production classes (tests excluded). Review candidates, not verdicts.",
        "",
        "- **WMC** — sum of method cyclomatic complexity; **TCC** — share of method pairs sharing `self` state;",
        "- **Resp.** — LCOM4 groups of methods with no shared state/calls (1 = one responsibility);",
        f"- **God class** — WMC ≥ {GOD_WMC}, TCC < 0.33, LOC ≥ {GOD_LOC};",
        "- **Test refs** — test files mentioning the class (proxy, not coverage);",
        f"- **Coverage** — {'from ' + cov if cov else 'not collected in this run (—)'}.",
        "",
        "## Summary",
        "",
    ]
    lines += [f"- {k}: **{v}**" for k, v in data["summary"].items()]

    def table(title, rows):
        out = ["", f"## {title}", ""]
        if not rows:
            return out + ["None."]
        out += [
            "| Class | File | LOC | Methods | WMC | Max CC | TCC | Resp. | ROS | Test refs | Coverage |",
            "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
        for c in rows:
            max_cc = f"{c['max_cc']} `{c['max_cc_method']}`" if c["max_cc_method"] else "0"
            out.append(
                f"| {c['name']} | {c['file']}:{c['line']} | {c['loc']} | {c['methods']} | {c['wmc']} | {max_cc} | "
                f"{c['tcc']:.2f} | {c['responsibilities']} | {c['ros_endpoints']} | {c['test_refs']} | "
                f"{fmt_cov(c['line_coverage'])} |"
            )
        return out

    gods = sorted((c for c in classes if c["god_class"]), key=lambda c: -c["wmc"])
    lines += table("God classes", gods)
    lines += table(f"Top {top} by WMC", sorted(classes, key=lambda c: -c["wmc"])[:top])
    multi = [c for c in classes if c["methods"] >= 5 and c["responsibilities"] >= 3]
    lines += table(
        f"Most responsibilities (≥5 methods, top {top})",
        sorted(multi, key=lambda c: (-c["responsibilities"], -c["wmc"]))[:top],
    )
    untested = [c for c in classes if c["test_refs"] == 0 and c["wmc"] >= 20]
    lines += table("Complex classes (WMC ≥ 20) without any test reference", sorted(untested, key=lambda c: -c["wmc"]))
    lines += ["", f"## Files ≥ {LARGE_FILE_LOC} lines", ""]
    lines += [f"- {f['file']}: **{f['loc']}**" for f in data["large_files"]] or ["None."]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--root", type=Path, default=Path("."))
    parser.add_argument("--coverage", type=Path, help="coverage.py JSON report (optional)")
    parser.add_argument("--json", type=Path, default=Path("architecture/class-metrics.json"))
    parser.add_argument("--markdown", type=Path, default=Path("architecture/class-metrics.md"))
    args = parser.parse_args()
    data = collect(args.root, args.coverage)
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    args.markdown.write_text(render_markdown(data), encoding="utf-8")
    print(json.dumps(data["summary"], ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
