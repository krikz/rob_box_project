#!/usr/bin/env python3
"""Patch broken Renardo SynthDef sources shipped in the published package."""

from __future__ import annotations

import sys
from pathlib import Path


def package_roots(workspace: Path) -> list[str]:
    """Каталоги ``src/<pkg>`` - корни python-пакетов до ``colcon build``.

    rob_box_voice.core импортирует rob_box_dialog (-> rob_box_core): одного
    rob_box_voice в sys.path мало (#3683).
    """
    src = workspace / "src"
    if not src.is_dir():
        return []
    return [str(p) for p in sorted(src.iterdir()) if p.is_dir() and p.name.startswith("rob_box_")]


def main() -> int:
    for root in reversed(package_roots(Path("/ws"))):
        if root not in sys.path:
            sys.path.insert(0, root)

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
