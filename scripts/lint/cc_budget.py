#!/usr/bin/env python3
"""CC-budget guard (ADR-0021 R1) for ``dialogue_node.py`` and new voice nodes.

Cyclomatic complexity budget:
  * regular methods  -> CC <= 15
  * ``__init__``     -> CC <= 20  (feature-flag conditionals)

Current over-limit methods are grandfathered via ``cc_budget_baseline.json``
(old code is OK until its refactor card lands, ADR-0021 stage 4).  The check
fails only on *new* violations: a method that exceeds the limit and is either
not in the baseline or has grown past its recorded grandfather value.

Usage:
  python scripts/lint/cc_budget.py                     # check (default)
  python scripts/lint/cc_budget.py <file> [<file>...]  # check given files
  python scripts/lint/cc_budget.py --update-baseline   # rewrite baseline
"""

from __future__ import annotations

import argparse
import ast
import json
import subprocess
import sys
from datetime import date
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
BASELINE_FILE = REPO_ROOT / "scripts" / "lint" / "cc_budget_baseline.json"
DEFAULT_TARGET = REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "dialogue_node.py"

METHOD_LIMIT = 15  # ADR-0021 R1
INIT_LIMIT = 20  # ADR-0021 R1, __init__ exemption

_DECISION_NODES = (
    ast.If,
    ast.While,
    ast.For,
    ast.ExceptHandler,
    ast.With,
    ast.Assert,
    ast.IfExp,
)
_COMPREHENSIONS = (ast.ListComp, ast.SetComp, ast.DictComp, ast.GeneratorExp)


def cyclomatic_complexity(func: ast.AST) -> int:
    """McCabe-style CC over a function body (base 1 + decision points)."""
    cc = 1
    for node in ast.walk(func):
        if isinstance(node, _DECISION_NODES):
            cc += 1
        elif isinstance(node, ast.BoolOp):
            cc += len(node.values) - 1
        elif isinstance(node, _COMPREHENSIONS):
            cc += 1 + sum(len(gen.ifs) for gen in node.generators)
        elif isinstance(node, ast.Match):
            cc += len(node.cases)
    return cc


def _collect_functions(body: list[ast.stmt], owner: str = "") -> list[tuple[str, ast.AST]]:
    """Return ``(qualified_name, node)`` for module funcs and class methods."""
    found: list[tuple[str, ast.AST]] = []
    for node in body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            found.append((f"{owner}{node.name}", node))
        elif isinstance(node, ast.ClassDef):
            found.extend(_collect_functions(node.body, owner=f"{owner}{node.name}."))
        elif isinstance(node, (ast.If, ast.Try)):
            nested = list(node.body) + list(getattr(node, "orelse", []))
            found.extend(_collect_functions(nested, owner))
    return found


def measure_file(path: Path) -> dict[str, int]:
    """Map qualified function names -> cyclomatic complexity for one file."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return {name: cyclomatic_complexity(node) for name, node in _collect_functions(tree.body)}


def _limit_for(name: str) -> int:
    return INIT_LIMIT if name.endswith(".__init__") else METHOD_LIMIT


def _rel(path: Path) -> str:
    """Stable baseline key: repo-relative when inside the tree, else absolute."""
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def _git_head() -> str:
    try:
        head = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        )
        return head.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _load_baseline() -> dict:
    if not BASELINE_FILE.exists():
        print(f"cc_budget: missing baseline {BASELINE_FILE}; run --update-baseline")
        sys.exit(2)
    return json.loads(BASELINE_FILE.read_text(encoding="utf-8"))


def cmd_update_baseline(files: list[Path], base_sha: str) -> int:
    """Snapshot every current over-limit method as a grandfathered entry."""
    exemptions: dict[str, dict[str, int]] = {}
    for path in files:
        rel = _rel(path)
        over = {name: cc for name, cc in sorted(measure_file(path).items()) if cc > _limit_for(name)}
        if over:
            exemptions[rel] = over
    baseline = {
        "version": 1,
        "created": date.today().isoformat(),
        "base_sha": base_sha or _git_head(),
        "limits": {"method": METHOD_LIMIT, "init": INIT_LIMIT},
        "exemptions": exemptions,
    }
    BASELINE_FILE.write_text(json.dumps(baseline, indent=2) + "\n", encoding="utf-8")
    print(f"cc_budget: baseline written to {BASELINE_FILE.relative_to(REPO_ROOT)}")
    print(f"cc_budget: base_sha={baseline['base_sha']} created={baseline['created']}")
    return 0


def cmd_check(files: list[Path], baseline: dict) -> int:
    """Report exceedances; fail only on entries the baseline does not cover."""
    exempt = baseline.get("exemptions", {})
    violations: list[tuple[str, str, int, int]] = []
    for path in files:
        rel = _rel(path)
        allowed = exempt.get(rel, {})
        measured = measure_file(path)
        for name, cc in sorted(measured.items()):
            limit = _limit_for(name)
            if cc <= limit:
                if name in allowed:
                    print(f"  [info] {rel}:{name} CC={cc} recovered; refresh baseline with --update-baseline")
                continue
            if name in allowed and cc <= allowed[name]:
                print(f"  [ok ] {rel}:{name} CC={cc} (limit {limit}, baseline {allowed[name]})")
            elif name not in allowed:
                violations.append((rel, name, cc, limit))
                print(f"  [FAIL] {rel}:{name} CC={cc} exceeds limit {limit} and is not in baseline")
            else:
                violations.append((rel, name, cc, limit))
                print(f"  [FAIL] {rel}:{name} CC={cc} grew past baseline {allowed[name]}")

    total = len(violations)
    if total:
        print(f"cc_budget: FAIL — {total} new CC violation(s); refactor or add to baseline.")
    else:
        print("cc_budget: OK — no new CC violations.")
    return 1 if total else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "files",
        nargs="*",
        type=Path,
        default=[DEFAULT_TARGET],
        help="Python files to scan (default: dialogue_node.py)",
    )
    parser.add_argument(
        "--update-baseline",
        action="store_true",
        help="Snapshot current over-limit methods into the baseline file",
    )
    parser.add_argument("--base-sha", default="", help="Override base commit SHA in baseline")
    args = parser.parse_args(argv)

    files = [path if path.is_absolute() else REPO_ROOT / path for path in args.files]
    for path in files:
        if not path.exists():
            print(f"cc_budget: no such file: {path}")
            return 2

    if args.update_baseline:
        return cmd_update_baseline(files, args.base_sha)
    return cmd_check(files, _load_baseline())


if __name__ == "__main__":
    raise SystemExit(main())
