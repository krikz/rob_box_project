#!/usr/bin/env python3
"""Офлайн-чеклист приёмки доставки (docs/runbooks/delivery-acceptance.md).

Гоняет host/chuangong/tests/test_delivery_acceptance_scenarios.py.
Не SSH, не деплой, не утверждает живой пилот. Exit 0 — офлайн-сценарии зелёные.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
TESTS = REPO / "host" / "chuangong" / "tests" / "test_delivery_acceptance_scenarios.py"


def main() -> int:
    if not TESTS.is_file():
        print(f"FAIL: нет файла сценариев: {TESTS}", file=sys.stderr)
        return 2
    cmd = [
        sys.executable,
        "-m",
        "pytest",
        str(TESTS),
        "-v",
        "--tb=short",
        "-o",
        "addopts=",
        "-p",
        "no:cacheprovider",
    ]
    print("+", " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, cwd=str(REPO))
    if proc.returncode != 0:
        print("FAIL: офлайн-сценарии приёмки доставки", file=sys.stderr)
        return proc.returncode
    print("PASS: офлайн-сценарии S0–S7 (фикстуры). Живой пилот не выполнялся.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
