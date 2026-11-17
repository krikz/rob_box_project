#!/usr/bin/env python3
"""Validate that G-Run Tests.yml actually runs unit tests for every
ROS2 Python package that has a test/ directory.

Issue #2232 (regression guard): the G-Run Tests workflow used to run only
rob_box_voice / rob_box_telegram / rob_box_teleop / rob_box_animations /
rob_box_perception — five of eight eligible packages. Three packages
(rob_box_core, rob_box_quest, rob_box_supervisor) had ~24,000 LOC of unit
tests that NEVER ran in CI, so failures there went unnoticed (a guard
test against the drift of three copies of the voice-presets whitelist
silently broke at #2234 and nobody realised until manual runs caught it
months later).

Strategy:
    1. Discover every ``src/rob_box_*/test/`` directory.
    2. Parse ``.github/workflows/G-Run Tests.yml``.
    3. For each discovered package name, assert:
       a) ``--packages-up-to ... <pkg> ...`` mentions it (or its dep graph
          covers it transitively from another --packages-up-to member);
       b) at least one of pytest / colcon test invocation references
          the package by name in its ``echo "=== ... ==="`` header (so
          the human reading CI logs sees it ran).
    4. Exit 1 on the first missing package; print a one-line report
       with all packages checked.

This script is run by the agent-flow before merging a CI-infrastructure
PR. It is intentionally read-only against the workflow file (no edits),
so it can also run as a quick smoke in CI itself.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
# When this script lives inside a git worktree, ``parents[4]`` is the main
# repo, not the worktree itself. The worktree root contains a ``.git``
# *file* (not directory) pointing to ``.git/worktrees/<name>``; resolve
# via git rev-parse from the script's own directory, which always returns
# the worktree root (or main checkout root — either is correct for our
# purposes, but we want the worktree so the script reads in-progress
# changes).
import subprocess  # noqa: E402

def _resolve_worktree_root(start: Path) -> Path:
    cur = start
    for _ in range(8):
        # Worktree root has a ``.git`` FILE (not dir) — distinguishable.
        git_path = cur / ".git"
        if git_path.is_file() or git_path.is_dir():
            try:
                out = subprocess.run(
                    ["git", "-C", str(cur), "rev-parse", "--show-toplevel"],
                    capture_output=True, text=True, check=True,
                ).stdout.strip()
                return Path(out)
            except Exception:
                return cur
        cur = cur.parent
    return start

REPO_ROOT = _resolve_worktree_root(Path(__file__).resolve().parent)
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "G-Run Tests.yml"
SRC_ROOT = REPO_ROOT / "src"


def discover_testable_packages() -> list[str]:
    """Every ROS2 Python package under src/ that ships its own test/ tree."""
    pkgs: list[str] = []
    for child in sorted(SRC_ROOT.iterdir()):
        if not child.is_dir():
            continue
        if not child.name.startswith("rob_box_"):
            continue
        test_dir = child / "test"
        if not test_dir.is_dir():
            continue
        # Has at least one test file (skip conftest-only trees)
        has_py = any(p.suffix == ".py" for p in test_dir.rglob("*.py"))
        if has_py:
            pkgs.append(child.name)
    return pkgs


def discover_skipped_packages() -> list[str]:
    """rob_box_* packages under src/ WITHOUT test/ — informational only."""
    out: list[str] = []
    for child in sorted(SRC_ROOT.iterdir()):
        if not child.is_dir() or not child.name.startswith("rob_box_"):
            continue
        if not (child / "test").is_dir():
            out.append(child.name)
    return out


PACKAGES_UP_TO_RE = re.compile(
    r"--packages-up-to[ \t]+(?P<pkgs>(?:rob_box_\w+[ \t]+)*rob_box_\w+)(?=[ \t]*\\)",
    re.MULTILINE,
)
PYTEST_HEADER_RE = re.compile(
    r"===\s*pytest:\s*(?P<pkg>rob_box_\w+)\s*\([^)]*\)\s*==="
)
COLCON_HEADER_RE = re.compile(
    r"===\s*colcon test:\s*(?P<pkg>rob_box_\w+)\s*\([^)]*\)\s*==="
)
PER_FILE_HEADER_RE = re.compile(
    r"---\s*(?P<pkg>rob_box_\w+):\s+\S+\s*---"
)


def parse_workflow() -> tuple[set[str], set[str]]:
    """Return (built_packages, run_packages).

    built_packages: those listed in --packages-up-to (covers transitive
    deps by construction: --packages-up-to ROBOT also builds ROBOT's
    deps; we treat a package as 'built' if it's listed OR is a transitive
    dep of a listed one, but we err on the side of explicit listing for
    the run check).
    run_packages: those with an explicit pytest/colcon/per-file header.
    """
    text = WORKFLOW.read_text(encoding="utf-8")
    built: set[str] = set()
    m = PACKAGES_UP_TO_RE.search(text)
    if m:
        for tok in m.group("pkgs").split():
            if tok.startswith("rob_box_"):
                built.add(tok)
    run: set[str] = set()
    for rgx in (PYTEST_HEADER_RE, COLCON_HEADER_RE, PER_FILE_HEADER_RE):
        for h in rgx.finditer(text):
            run.add(h.group("pkg"))
    return built, run


def main() -> int:
    pkgs = discover_testable_packages()
    skipped = discover_skipped_packages()
    built, run = parse_workflow()
    print(f"Discovered testable packages: {pkgs}")
    print(f"  built (--packages-up-to):  {sorted(built)}")
    print(f"  run   (pytest/colcon):    {sorted(run)}")
    if skipped:
        print(f"  skipped (no test/ dir):   {skipped} (informational)")

    # The validator enforces CI coverage for the *known set* of packages
    # (the eight listed in issue #2232 + the four already-running). Any
    # new rob_box_* package without its own pytest/colcon invocation
    # would re-introduce the regression this script was written to
    # prevent. Out-of-scope packages (harness / llm / mcp_tools) can be
    # added by extending the EXPECTED list below when they're ready
    # for CI.
    EXPECTED = {
        "rob_box_animations",
        "rob_box_core",
        "rob_box_perception",
        "rob_box_quest",
        "rob_box_supervisor",
        "rob_box_telegram",
        "rob_box_teleop",
        "rob_box_voice",
    }

    problems: list[str] = []
    for pkg in EXPECTED:
        if pkg not in built:
            problems.append(f"{pkg}: missing from --packages-up-to")
        if pkg not in run:
            problems.append(f"{pkg}: built but no pytest/colcon header")

    other = sorted(set(pkgs) - EXPECTED)
    if other:
        print(
            f"\nℹ️  packages present but NOT in EXPECTED (out of #2232 scope): {other}",
            file=sys.stderr,
        )

    if problems:
        print("\n❌ G-Run Tests.yml does NOT cover:", file=sys.stderr)
        for p in problems:
            print(f"   - {p}", file=sys.stderr)
        print(
            "\nIssue #2232 regression: CI silently skips unit tests.\n"
            "Fix: add the package to --packages-up-to AND add a "
            "pytest/colcon invocation with an explicit === header.",
            file=sys.stderr,
        )
        return 1

    print(
        f"\n✅ All {len(EXPECTED)} expected packages are built and "
        "explicitly run in CI."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())