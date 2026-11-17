#!/usr/bin/env python3
"""Гард меток ``TEMP(ADR-0148`` (ADR-0148, ADR-0150 §14 A8; карточка #3684).

Заплатка «проверка текста ответа LLM» разрешена только временно и с меткой ``TEMP(ADR-0148`` в коде. Гард считает
вхождения метки (без закрывающей скобки: встречаются и ``TEMP(ADR-0148)``, и ``TEMP(ADR-0148):``) в ``src/`` вне
тестов и сравнивает с ``scripts/lint/temp_adr0148_baseline.json``.

* число выросло -> exit 1 (новая заплатка без решения Шифу);
* число упало   -> exit 0 и подсказка обновить baseline (``--update-baseline``): храповик затягивается вручную, чтобы
  параллельные PR, удаляющие заплатки, не конфликтовали из-за общего файла;
* цель ADR-0150 A8 — 0.

    python scripts/lint/temp_adr0148_check.py [--update-baseline] [--list]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

MARK = "TEMP(ADR-0148"
REPO = Path(__file__).resolve().parents[2]
BASELINE = Path(__file__).resolve().with_name("temp_adr0148_baseline.json")
_TEST_DIRS = {"test", "tests", "__pycache__", "node_modules", ".git"}
_SKIP_SUFFIXES = {".pyc", ".png", ".jpg", ".jpeg", ".gif", ".wav", ".mp3", ".ogg", ".onnx", ".bin", ".so", ".whl"}


def _is_test(rel: Path) -> bool:
    return any(part in _TEST_DIRS for part in rel.parts) or rel.name.startswith("test_") or rel.name == "conftest.py"


def iter_files(src: Path) -> Iterable[Path]:
    for path in sorted(src.rglob("*")):
        if path.is_file() and path.suffix.lower() not in _SKIP_SUFFIXES and not _is_test(path.relative_to(src)):
            yield path


def find_marks(src: Path) -> List[Tuple[str, int]]:
    """``(путь относительно src, номер строки)`` каждого вхождения метки; на строке считается каждое вхождение."""
    found: List[Tuple[str, int]] = []
    for path in iter_files(src):
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for lineno, line in enumerate(text.splitlines(), start=1):
            found.extend((path.relative_to(src).as_posix(), lineno) for _ in range(line.count(MARK)))
    return found


def load_baseline(path: Path = BASELINE) -> int:
    return int(json.loads(path.read_text(encoding="utf-8"))["count"])


def check(count: int, baseline: int) -> Tuple[int, str]:
    """``(код выхода, сообщение)``: рост -> 1; равно/меньше -> 0."""
    if count > baseline:
        return 1, (
            f"FAIL: {MARK} в src вне тестов: {count} > baseline {baseline}. Новая заплатка по тексту ответа LLM "
            "запрещена (ADR-0148): вынеси решение в код или сузь схему тула. Исключение только по решению Шифу "
            "с карточкой на удаление."
        )
    if count < baseline:
        return 0, (
            f"OK: {count} < baseline {baseline}. Затяни храповик: "
            "python scripts/lint/temp_adr0148_check.py --update-baseline"
        )
    return 0, f"OK: {MARK} в src вне тестов = {count} (baseline {baseline}); цель ADR-0150 A8 — 0."


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--src", type=Path, default=REPO / "src")
    ap.add_argument("--baseline", type=Path, default=BASELINE)
    ap.add_argument("--update-baseline", action="store_true")
    ap.add_argument("--list", action="store_true", help="напечатать каждое вхождение")
    args = ap.parse_args(argv)
    found = find_marks(args.src)
    if args.list:
        for rel, lineno in found:
            print(f"  src/{rel}:{lineno}")
    if args.update_baseline:
        args.baseline.write_text(json.dumps({"count": len(found)}, indent=2) + "\n", encoding="utf-8")
        print(f"baseline обновлён: {len(found)}")
        return 0
    code, message = check(len(found), load_baseline(args.baseline))
    print(message)
    return code


if __name__ == "__main__":
    sys.exit(main())
