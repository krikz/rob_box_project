#!/usr/bin/env python3
"""Fix unresolved git merge conflicts in renardo_lib .scd SynthDef files.

The renardo_lib package (pip) ships with unresolved merge conflict markers
in brass.scd, organ.scd, and tb303.scd.  SuperCollider cannot compile these
files, so the synths silently fail to load.

This script scans all .scd files under renardo_lib's sclang_code directory
and resolves any conflicts by keeping the newer (bottom / "theirs") version
which includes CrashServer metadata blocks.

Usage:
    python scripts/utils/fix_renardo_synthdef_conflicts.py
"""

import re
import sys
from pathlib import Path

CONFLICT_START = re.compile(r"^<<<<<<<.*$", re.MULTILINE)
CONFLICT_MID = re.compile(r"^=======\s*$", re.MULTILINE)
CONFLICT_END = re.compile(r"^>>>>>>>.*$", re.MULTILINE)


def resolve_conflicts(text: str) -> str:
    """Resolve all merge conflict blocks by keeping the 'theirs' (bottom) version."""
    result = text
    while CONFLICT_START.search(result):
        # Find markers
        m_start = CONFLICT_START.search(result)
        if not m_start:
            break
        m_mid = CONFLICT_MID.search(result, m_start.end())
        if not m_mid:
            break
        m_end = CONFLICT_END.search(result, m_mid.end())
        if not m_end:
            break

        # Keep "theirs" (between ======= and >>>>>>>)
        theirs = result[m_mid.end() : m_end.start()].strip("\n")

        # Replace entire conflict block
        result = result[: m_start.start()] + theirs + result[m_end.end() :]

    return result


def main() -> int:
    try:
        import renardo_lib
    except ImportError:
        print("renardo_lib not installed — nothing to fix")
        return 0

    sclang_dir = Path(renardo_lib.__file__).parent / "SynthDefManagement" / "sclang_code"
    if not sclang_dir.exists():
        print(f"sclang_code dir not found at {sclang_dir}")
        return 1

    fixed = 0
    for scd_file in sclang_dir.rglob("*.scd"):
        content = scd_file.read_text(encoding="utf-8", errors="replace")
        if CONFLICT_START.search(content):
            resolved = resolve_conflicts(content)
            scd_file.write_text(resolved, encoding="utf-8")
            print(f"  fixed: {scd_file.name}")
            fixed += 1

    if fixed:
        print(f"\n✅ Fixed {fixed} SynthDef file(s) with merge conflicts")
    else:
        print("✅ No merge conflicts found in .scd files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
