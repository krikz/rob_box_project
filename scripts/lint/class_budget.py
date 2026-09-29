#!/usr/bin/env python3
"""Class-size budget guard (ADR-0145; complements ADR-0021 R1 per-method CC budget).

Why: ADR-0021 bounds only the cyclomatic complexity of a *single method*.
Authors "extracted" code into more private methods of the SAME class, every
method stayed under CC 15 and the class kept growing: ``DialogueNode`` went
from 4062 LOC / 74 methods (18.08) to 9872 LOC / 218 methods, WMC 1184.
Moving code between methods of one class does not reduce the class.

Metrics (per top-level and nested class, nested ones are counted separately
under a qualified ``Outer.Inner`` name):

  * methods = number of direct ``def`` / ``async def`` in the class body;
  * WMC     = sum of the cyclomatic complexity of those methods
              (``cc_budget.cyclomatic_complexity``).

A class is "big" if ``WMC > 80`` or ``methods > 40``.  Big classes are
grandfathered in ``class_budget_baseline.json`` and ratcheted:

  * big class not in baseline           -> FAIL (new god class: split before merge);
  * WMC or methods grew                 -> FAIL (extract into a separate module/class;
                                           same-class private helpers do not reduce it);
  * WMC/methods shrank                  -> FAIL (refresh with ``--update-baseline``
                                           in the same PR, ratchet as ADR-0021-r1 R-1b);
  * in baseline but no longer big       -> FAIL (remove it from the baseline);
  * in baseline but class not found     -> FAIL (phantom, ADR-0021-r1 R-1c).

Scope: the same packages as ``cc_budget.py`` (``_PACKAGE_ROOTS``).  With explicit
paths only the scanned files are considered (no false phantoms).

Usage:
  python scripts/lint/class_budget.py                     # check (default scope)
  python scripts/lint/class_budget.py <path> [<path>...]  # check given files/dirs
  python scripts/lint/class_budget.py --update-baseline   # snapshot all big classes
"""

from __future__ import annotations

import argparse
import ast
import json
import sys
from datetime import date
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cc_budget  # noqa: E402

REPO_ROOT = cc_budget.REPO_ROOT
BASELINE_FILE = REPO_ROOT / "scripts" / "lint" / "class_budget_baseline.json"
DEFAULT_TARGETS = cc_budget.DEFAULT_TARGETS

WMC_LIMIT = 80
METHODS_LIMIT = 40

_FUNC_TYPES = (ast.FunctionDef, ast.AsyncFunctionDef)


def is_big(stats: dict[str, int]) -> bool:
    return stats["wmc"] > WMC_LIMIT or stats["methods"] > METHODS_LIMIT


def _collect_classes(body: list[ast.stmt], owner: str = "") -> dict[str, dict[str, int]]:
    """Map qualified class name -> ``{"wmc": N, "methods": M}``."""
    found: dict[str, dict[str, int]] = {}
    for node in body:
        if isinstance(node, ast.ClassDef):
            name = f"{owner}{node.name}"
            methods = [n for n in node.body if isinstance(n, _FUNC_TYPES)]
            found[name] = {
                "wmc": sum(cc_budget.cyclomatic_complexity(m) for m in methods),
                "methods": len(methods),
            }
            found.update(_collect_classes(node.body, owner=f"{name}."))
        elif isinstance(node, (ast.If, ast.Try)):
            nested = list(node.body) + list(getattr(node, "orelse", []))
            found.update(_collect_classes(nested, owner))
    return found


def measure_file(path: Path) -> dict[str, dict[str, int]]:
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    return _collect_classes(tree.body)


def _rel(path: Path) -> str:
    return cc_budget._rel(path)


def _load_baseline(baseline_file: Path) -> dict:
    if not baseline_file.exists():
        print(f"class_budget: missing baseline {baseline_file}; run --update-baseline")
        sys.exit(2)
    return json.loads(baseline_file.read_text(encoding="utf-8"))


def _summary(big: dict[str, dict[str, int]]) -> str:
    wmc = sum(s["wmc"] for s in big.values())
    methods = sum(s["methods"] for s in big.values())
    return f"class_budget: {len(big)} big classes (sum WMC={wmc}, methods={methods})"


def _measure_all(files: list[Path]) -> tuple[dict[str, dict[str, dict[str, int]]], dict[str, dict[str, int]]]:
    """Return (per-file measurements, big classes keyed ``path:Class``)."""
    per_file: dict[str, dict[str, dict[str, int]]] = {}
    big: dict[str, dict[str, int]] = {}
    for path in files:
        rel = _rel(path)
        per_file[rel] = measure_file(path)
        for name, stats in sorted(per_file[rel].items()):
            if is_big(stats):
                big[f"{rel}:{name}"] = stats
    return per_file, big


def cmd_update_baseline(files: list[Path], base_sha: str, baseline_file: Path) -> int:
    old: dict = json.loads(baseline_file.read_text(encoding="utf-8")) if baseline_file.exists() else {}
    per_file, big = _measure_all(files)
    scanned = set(per_file)

    classes: dict[str, dict[str, int]] = {}
    for key, stats in old.get("classes", {}).items():
        rel = key.partition(":")[0]
        if rel in scanned:
            continue
        if cc_budget._file_exists(rel):  # files outside the scan keep their entries
            classes[key] = stats
        else:
            print(f"  [drop] {key} (file no longer exists)")
    classes.update({key: dict(stats) for key, stats in big.items()})

    baseline: dict = {
        "version": 1,
        "created": date.today().isoformat(),
        "base_sha": base_sha or cc_budget._git_head(),
        "limits": {"wmc": WMC_LIMIT, "methods": METHODS_LIMIT},
        "classes": dict(sorted(classes.items())),
    }
    for key, value in old.items():
        if key.startswith("_"):
            baseline[key] = value
    baseline_file.write_text(json.dumps(baseline, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"class_budget: baseline written to {_rel(baseline_file)}")
    print(_summary(baseline["classes"]))
    return 0


def cmd_check(files: list[Path], baseline: dict) -> int:
    known: dict[str, dict[str, int]] = baseline.get("classes", {})
    per_file, big = _measure_all(files)
    failures = 0

    def fail(msg: str) -> None:
        nonlocal failures
        failures += 1
        print(f"  [FAIL] {msg}")

    for key, cur in big.items():
        old = known.get(key)
        desc = f"WMC={cur['wmc']} methods={cur['methods']}"
        if old is None:
            fail(
                f"{key} {desc} is a new big class (limits: WMC>{WMC_LIMIT} or methods>{METHODS_LIMIT}); "
                f"split it into separate modules/classes before merge (ADR-0145)"
            )
        elif cur["wmc"] > old["wmc"] or cur["methods"] > old["methods"]:
            fail(
                f"{key} grew: {desc} vs baseline WMC={old['wmc']} methods={old['methods']}; "
                f"extract into a separate module/class - same-class private helpers "
                f"do not reduce class size (ADR-0145)"
            )
        elif cur["wmc"] < old["wmc"] or cur["methods"] < old["methods"]:
            fail(
                f"{key} shrank: {desc} vs baseline WMC={old['wmc']} methods={old['methods']}; "
                f"refresh baseline with --update-baseline in the same PR to lock the gain"
            )
        else:
            print(f"  [ok ] {key} {desc}")

    # Baseline entries: files outside the scanned set are skipped only while they
    # still exist; a deleted/renamed file is a phantom in any run.
    for key, old in sorted(known.items()):
        rel, _, name = key.partition(":")
        if rel not in per_file:
            if not cc_budget._file_exists(rel):
                fail(
                    f"{key} - phantom baseline entry (file no longer exists); remove it from class_budget_baseline.json"
                )
            continue
        if name not in per_file[rel]:
            fail(
                f"{key} - phantom baseline entry (class not found in source); remove it from class_budget_baseline.json"
            )
        elif key not in big:
            fail(
                f"{key} is no longer big (WMC={per_file[rel][name]['wmc']} "
                f"methods={per_file[rel][name]['methods']}); remove it from class_budget_baseline.json"
            )

    print(_summary(big))
    if failures:
        print(f"class_budget: FAIL - {failures} violation(s); see [FAIL] lines above.")
        return 1
    print("class_budget: OK - no new or growing big classes.")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "paths",
        nargs="*",
        type=Path,
        default=list(DEFAULT_TARGETS),
        help="Python files or package directories to scan (default: cc_budget scope)",
    )
    parser.add_argument("--update-baseline", action="store_true", help="Snapshot all big classes")
    parser.add_argument("--base-sha", default="", help="Override base commit SHA in baseline")
    parser.add_argument("--baseline", type=Path, default=BASELINE_FILE, help="Baseline path override")
    args = parser.parse_args(argv)

    raw = [p if p.is_absolute() else REPO_ROOT / p for p in args.paths]
    files = cc_budget._expand_targets(raw)
    if not files:
        print("class_budget: no Python files found under the given targets")
        return 2
    if args.update_baseline:
        return cmd_update_baseline(files, args.base_sha, args.baseline)
    return cmd_check(files, _load_baseline(args.baseline))


if __name__ == "__main__":
    raise SystemExit(main())
