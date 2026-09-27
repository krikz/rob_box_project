#!/usr/bin/env python3
"""
Bulk-import existing ADR files into OpenSpec change folders.

Input:  docs/adr/NNNN-name.md
Output: <openspec-root>/changes/adr-NNNN-name/proposal.md  (skeleton only)

Per ADR-0038 (HYBRID + bulk-import legacy ADR, 2026-08-31):
  - Only creates proposal.md skeleton with link to original ADR
  - Does NOT create design.md / tasks.md / spec.md (worker fills when change is real)
  - Idempotent: skip if change folder already exists
  - Supports --dry-run for safe verification before real run

Usage:
  python scripts/import_adr_to_openspec.py
  python scripts/import_adr_to_openspec.py --dry-run
  python scripts/import_adr_to_openspec.py --repo-root .
  python scripts/import_adr_to_openspec.py --adr-dir docs/adr --openspec-root docs/research/openspec-pilot/openspec

Exits 0 on success (including dry-run), 1 on errors.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# Pattern: NNNN-name.md (4 digits, kebab-case name)
ADR_PATTERN = re.compile(r"^(?P<num>\d{4})-(?P<slug>.+)\.md$")

PROPOSAL_TEMPLATE = """# Proposal: ADR-{num} ({title})

## Why

Imported from [{rel_adr_path}]({rel_adr_path}) for tracking in OpenSpec.

Status: see original ADR.

## What Changes

See original ADR for full context and decision rationale.

## Capabilities

<!-- TODO: воркер дописывает при первом реальном изменении в этой области -->

## Impact

- Docs only — no runtime behavior changes
- Original ADR remains authoritative source of truth
"""

# .openspec.yaml — легитимный способ сказать OpenSpec что change docs-only
# (см. https://github.com/Fission-AI/OpenSpec/blob/main/docs/cli.md#openspec-archive)
OPENSPEC_YAML = """schema: spec-driven
created: 2026-08-31
goal: "Bulk-import of legacy ADR for OpenSpec tracking (docs-only)"
skip_specs: true
"""


def extract_title(adr_path: Path) -> str:
    """Extract title from first H1 heading in ADR file. Falls back to filename."""
    try:
        text = adr_path.read_text(encoding="utf-8")
    except OSError:
        return adr_path.stem
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("# "):
            return line[2:].strip()
    return adr_path.stem


def find_adr_files(adr_dir: Path) -> list[Path]:
    """Find all NNNN-*.md files in adr_dir, sorted by number."""
    files: list[Path] = []
    for p in adr_dir.iterdir():
        if p.is_file() and ADR_PATTERN.match(p.name):
            files.append(p)
    return sorted(files, key=lambda p: p.name)


def build_change_folder_name(adr_path: Path) -> str:
    """adr-0038-adopt-openspec.md -> adr-0038-adopt-openspec"""
    return f"adr-{adr_path.stem}"


def render_proposal(
    adr_path: Path, repo_root: Path, change_folder_name: str
) -> tuple[str, str]:
    """Render proposal.md content + filename for one ADR. Returns (title, body)."""
    title = extract_title(adr_path)
    try:
        rel = adr_path.relative_to(repo_root)
    except ValueError:
        rel = adr_path
    num_match = ADR_PATTERN.match(adr_path.name)
    assert num_match is not None
    num = num_match.group("num")
    body = PROPOSAL_TEMPLATE.format(
        num=num,
        title=title,
        rel_adr_path=str(rel).replace("\\", "/"),
    )
    return title, body


def import_one(
    adr_path: Path,
    repo_root: Path,
    changes_root: Path,
    dry_run: bool,
) -> tuple[str, str]:
    """Process one ADR. Returns ('created'|'skipped'|'dry-run', change_folder_name)."""
    folder_name = build_change_folder_name(adr_path)
    change_dir = changes_root / folder_name
    if change_dir.exists():
        return "skipped", folder_name
    _, body = render_proposal(adr_path, repo_root, folder_name)
    if dry_run:
        return "dry-run", folder_name
    change_dir.mkdir(parents=True, exist_ok=False)
    (change_dir / "proposal.md").write_text(body, encoding="utf-8")
    (change_dir / ".openspec.yaml").write_text(OPENSPEC_YAML, encoding="utf-8")
    return "created", folder_name


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Bulk-import legacy ADR files into OpenSpec change folders."
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path.cwd(),
        help="Repo root (default: current directory)",
    )
    parser.add_argument(
        "--adr-dir",
        type=Path,
        default=Path("docs/adr"),
        help="ADR directory relative to repo-root (default: docs/adr)",
    )
    parser.add_argument(
        "--openspec-root",
        type=Path,
        default=Path("docs/research/openspec-pilot/openspec"),
        help="OpenSpec root (default: docs/research/openspec-pilot/openspec)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would be done without writing files",
    )
    args = parser.parse_args(argv)

    repo_root: Path = args.repo_root.resolve()
    adr_dir: Path = (repo_root / args.adr_dir).resolve()
    openspec_root: Path = (repo_root / args.openspec_root).resolve()
    changes_root: Path = openspec_root / "changes"

    if not adr_dir.is_dir():
        print(f"ERROR: ADR dir not found: {adr_dir}", file=sys.stderr)
        return 1
    if not openspec_root.is_dir():
        print(f"ERROR: OpenSpec root not found: {openspec_root}", file=sys.stderr)
        return 1
    if not changes_root.is_dir():
        print(f"ERROR: changes/ not found: {changes_root}", file=sys.stderr)
        return 1

    adr_files = find_adr_files(adr_dir)
    if not adr_files:
        print(f"WARN: no ADR files matching NNNN-*.md in {adr_dir}", file=sys.stderr)
        return 0

    print(f"Repo root:       {repo_root}")
    print(f"ADR dir:         {adr_dir}")
    print(f"OpenSpec root:   {openspec_root}")
    print(f"Changes root:    {changes_root}")
    print(f"Found {len(adr_files)} ADR files")
    if args.dry_run:
        print("MODE: dry-run (no files will be written)")
    print()

    counts = {"created": 0, "skipped": 0, "dry-run": 0}
    for adr_path in adr_files:
        action, folder = import_one(adr_path, repo_root, changes_root, args.dry_run)
        counts[action] += 1
        num_match = ADR_PATTERN.match(adr_path.name)
        assert num_match
        print(f"  [{action:>7}] {num_match.group('num')} -> {folder}")

    print()
    print(
        f"Summary: created={counts['created']} "
        f"skipped={counts['skipped']} "
        f"dry-run={counts['dry-run']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())