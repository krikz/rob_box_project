#!/usr/bin/env python3
"""CC-budget guard (ADR-0021 R1) for active voice/harness/supervisor/mcp packages.

Cyclomatic complexity budget:
  * regular methods  -> CC <= 15
  * ``__init__``     -> CC <= 20  (feature-flag conditionals)

Current over-limit methods are grandfathered via ``cc_budget_baseline.json``
(old code is OK until its refactor card lands, ADR-0021 stage 4).  The check
fails only on *new* violations: a method that exceeds the limit and is either
not in the baseline or has grown past its recorded grandfather value.

Scope: every Python module under the active development packages
(``src/rob_box_voice/rob_box_voice``, ``src/rob_box_supervisor/rob_box_supervisor``,
``src/rob_box_harness/rob_box_harness``, ``src/rob_box_mcp_tools/rob_box_mcp_tools``,
``src/rob_box_quest/rob_box_quest``).
ADR-0021 explicitly applies the rule to "``dialogue_node.py`` and any new voice
nodes in ``rob_box_voice``"; extending it to the four sibling packages is the
least-surprise scope: these are where active development is happening, and any
new method added there must respect the budget. ``rob_box_quest`` (Meta Quest
telepresence, ADR-0080) was added in issue #2186 — without it the gate silently
fixes CC growth instead of stopping it (the ``WSSServer._on_json_cmd`` blast
balloon reached CC=107 undetected).

Usage:
  python scripts/lint/cc_budget.py                     # check (default scope)
  python scripts/lint/cc_budget.py <path> [<path>...]  # check given files/dirs
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

# Active packages whose modules are subject to R1. Paths are repo-relative
# strings so they survive worktree moves and so baseline keys stay stable.
_PACKAGE_ROOTS = (
    "src/rob_box_voice/rob_box_voice",
    "src/rob_box_supervisor/rob_box_supervisor",
    "src/rob_box_harness/rob_box_harness",
    "src/rob_box_mcp_tools/rob_box_mcp_tools",
    "src/rob_box_quest/rob_box_quest",
)
DEFAULT_TARGETS: tuple[Path, ...] = tuple(REPO_ROOT / p for p in _PACKAGE_ROOTS)

METHOD_LIMIT = 15  # ADR-0021 R1
INIT_LIMIT = 20  # ADR-0021 R1, __init__ exemption

_SKIP_DIR_NAMES = {"__pycache__", ".git"}

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
    # utf-8-sig silently strips a leading BOM if present so we can lint files
    # authored on Windows without choking. Valid Python (ASCII/UTF-8) is
    # untouched, so this is a safe widening of the previous strict decoder.
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    return {name: cyclomatic_complexity(node) for name, node in _collect_functions(tree.body)}


def _expand_targets(targets: list[Path]) -> list[Path]:
    """Resolve a mix of files and package directories to a flat list of .py files.

    Directories are walked recursively; ``__pycache__``/``.git`` are skipped.
    Order is stable (sorted) so baseline keys and CI output stay reproducible.
    """
    resolved: list[Path] = []
    for target in targets:
        if target.is_file():
            resolved.append(target)
            continue
        if not target.is_dir():
            print(f"cc_budget: no such path: {target}")
            sys.exit(2)
        for path in sorted(target.rglob("*.py")):
            if any(part in _SKIP_DIR_NAMES for part in path.parts):
                continue
            if path.name == "__init__.py":
                continue
            resolved.append(path)
    return resolved


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
    """Report exceedances; fail on phantom/under-baseline/growth.

    Five failure modes (ADR-0021 R1 baseline + R1-r1 ratchets, issue #2626):

    1. CC > limit and method not in baseline — new violation, refuse.
    2. CC > baseline entry — grew past grandfathered value, refuse.
    3. CC ≤ limit but method appears in baseline — recovered below
       limit without updating baseline. R-1b: previously a soft
       ``[info]`` that let regressions hide. Now FAIL so the author
       is forced to refresh the baseline (or remove the exempt).
    4. baseline has an entry for a function that does not exist in
       the source — phantom. R-1c: this is exactly the
       ``_build_single_provider`` shape that sat for weeks because
       the guard only walked *measured* functions.
    5. CC > 30 on a brand-new exempt without an ADR anchor — R-1e:
       preempts the WSSServer._on_json_cmd / _synthesize_and_play
       pattern (CC=107/124) where ``просто лимит + амнистия``
       didn't stop the growth.
    """
    exempt = baseline.get("exemptions", {})
    HARD_EXEMPT_CC = 30  # R-1e: 2× METHOD_LIMIT, ADR anchor required above this
    violations: list[tuple[str, str, int, int]] = []

    # R-1c: collect measured names per file so we can spot phantoms.
    measured_per_file: dict[str, dict[str, int]] = {}

    for path in files:
        rel = _rel(path)
        allowed = exempt.get(rel, {})
        measured = measure_file(path)
        measured_per_file[rel] = measured
        for name, cc in sorted(measured.items()):
            limit = _limit_for(name)
            if cc <= limit:
                # R-1b: under-baseline recovery — fail loudly so a silent
                # 'recovered' doesn't paper over the fact that the
                # baseline still claims a much higher CC. Author must
                # run ``--update-baseline`` (or remove the entry) in the
                # same PR.
                if name in allowed and cc < allowed[name]:
                    violations.append((rel, name, cc, limit))
                    print(
                        f"  [FAIL] {rel}:{name} CC={cc} recovered below "
                        f"baseline {allowed[name]}; refresh baseline with "
                        f"--update-baseline in the same PR (ADR-0021-r1 R-1b)"
                    )
                continue
            if name in allowed and cc <= allowed[name]:
                # R-1e: even an in-baseline over-limit entry must respect
                # the hard ceiling UNLESS the baseline already recorded
                # it at this height (R-1e is for *new* exemptions).
                if cc > HARD_EXEMPT_CC and allowed[name] <= HARD_EXEMPT_CC:
                    violations.append((rel, name, cc, limit))
                    print(
                        f"  [FAIL] {rel}:{name} CC={cc} crossed hard ceiling "
                        f"{HARD_EXEMPT_CC}; new exemptions above 2× limit "
                        f"require ADR anchor (ADR-0021-r1 R-1e)"
                    )
                else:
                    print(f"  [ok ] {rel}:{name} CC={cc} (limit {limit}, baseline {allowed[name]})")
            elif name not in allowed:
                # R-1e: a brand-new over-limit entry above the hard
                # ceiling requires an ADR; surface as FAIL with the
                # explicit hint.
                if cc > HARD_EXEMPT_CC:
                    violations.append((rel, name, cc, limit))
                    print(
                        f"  [FAIL] {rel}:{name} CC={cc} exceeds limit {limit} "
                        f"and is not in baseline; CC>{HARD_EXEMPT_CC} requires "
                        f"an ADR per ADR-0021-r1 R-1e"
                    )
                else:
                    violations.append((rel, name, cc, limit))
                    print(f"  [FAIL] {rel}:{name} CC={cc} exceeds limit {limit} and is not in baseline")
            else:
                violations.append((rel, name, cc, limit))
                print(f"  [FAIL] {rel}:{name} CC={cc} grew past baseline {allowed[name]}")

    # R-1c phantom-detection: every (path, method) in baseline must
    # actually exist in the scanned files. Otherwise the baseline
    # keeps claiming protection for code that's gone — exactly the
    # ``_build_single_provider`` blind spot.
    for rel, allowed in sorted(exempt.items()):
        measured = measured_per_file.get(rel, {})
        for name, baseline_cc in sorted(allowed.items()):
            if name not in measured:
                violations.append((rel, name, 0, baseline_cc))
                print(
                    f"  [FAIL] {rel}:{name} — phantom baseline entry "
                    f"(method not found in source); remove from "
                    f"cc_budget_baseline.json:exemptions (ADR-0021-r1 R-1c)"
                )

    total = len(violations)
    if total:
        print(f"cc_budget: FAIL — {total} violation(s); see [FAIL] lines above.")
        return 1
    print("cc_budget: OK — no new CC violations.")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "paths",
        nargs="*",
        type=Path,
        default=list(DEFAULT_TARGETS),
        help=(
            "Python files or package directories to scan "
            "(default: the five active packages under src/)"
        ),
    )
    parser.add_argument(
        "--update-baseline",
        action="store_true",
        help="Snapshot current over-limit methods into the baseline file",
    )
    parser.add_argument("--base-sha", default="", help="Override base commit SHA in baseline")
    args = parser.parse_args(argv)

    raw_targets = [path if path.is_absolute() else REPO_ROOT / path for path in args.paths]
    files = _expand_targets(raw_targets)
    if not files:
        print("cc_budget: no Python files found under the given targets")
        return 2

    if args.update_baseline:
        return cmd_update_baseline(files, args.base_sha)
    return cmd_check(files, _load_baseline())


if __name__ == "__main__":
    raise SystemExit(main())
