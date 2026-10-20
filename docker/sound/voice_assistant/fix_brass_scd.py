#!/usr/bin/env python3
"""Patch broken Renardo SynthDef sources shipped in the published package."""

from __future__ import annotations

import sys
from pathlib import Path


def main() -> int:
    workspace = Path("/ws")
    if str(workspace / "src" / "rob_box_voice") not in sys.path:
        sys.path.insert(0, str(workspace / "src" / "rob_box_voice"))

    import renardo_lib

    from rob_box_voice.core.renardo_synthdef_patches import apply_renardo_synthdef_patches

    sclang_dir = Path(renardo_lib.__file__).resolve().parent / "SynthDefManagement" / "sclang_code"
    if not sclang_dir.exists():
        raise FileNotFoundError(f"Renardo sclang_code dir not found: {sclang_dir}")

    patched_files = apply_renardo_synthdef_patches(sclang_dir)
    if patched_files:
        print(f"patched SynthDefs: {', '.join(patched_files)}")
    else:
        print("no SynthDef patches required")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
