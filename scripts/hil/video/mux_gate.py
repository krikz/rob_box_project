#!/usr/bin/env python3
"""Тонкая обёртка: sandbox_prep.sh копирует на песочницу только scripts/hil/video.

Сам гейт — scripts/hil/checks/mux_gate.py (вердикт, sentinel HIL, инъекция).
Этот файл только находит его и в репо, и в копии песочницы ($VIDEO_DIR).
"""

from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_CANDIDATES = (
    _HERE.parent / "checks" / "mux_gate.py",  # репо: scripts/hil/checks/
    _HERE / "checks" / "mux_gate.py",  # копия песочницы, если checks положили рядом
    Path(os.environ.get("HIL_MUX_GATE_PY", "")),
)


def _gate() -> Path:
    for path in _CANDIDATES:
        if path.is_file():
            return path
    raise SystemExit(
        "mux_gate: не найден scripts/hil/checks/mux_gate.py рядом с обёрткой и не задан HIL_MUX_GATE_PY"
    )


if __name__ == "__main__":
    sys.argv[0] = str(_gate())
    runpy.run_path(str(_gate()), run_name="__main__")
