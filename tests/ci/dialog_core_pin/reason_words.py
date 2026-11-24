"""Жёсткие чтения ``REASON_WORDS`` монорепо против карты пинованного тега (#5210).

``rob_box_mcp_tools`` собирается colcon-ом из монорепо, ``rob_box_music`` ставится
pip-ом из тега ядра. ``REASON_WORDS["ключ"]`` без ключа в пине — ``KeyError`` в
образе, хотя оба репо по отдельности зелёные. ``.get`` сюда не входит: его
отсутствие ключа образ не роняет.

Обход ``src/`` один. Новое жёсткое чтение добавляется в ``_HARD_READ_RE``, а не
копией обхода. Без префикса пина проверка падает, а не пропускается.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_HARD_READ_RE = re.compile(r"""REASON_WORDS\[(['"])([^'"\]]+)\1\]""")
_SKIP_WALK = {"__pycache__", ".git"}
# Синтетический ключ отрицательного контроля: в карте ядра его нет by construction.
SYNTHETIC_KEY = "pin_guard_synthetic_missing"


def hard_key_reads(repo: Path = _REPO) -> dict[str, list[str]]:
    """Имена ключей жёсткого чтения и ``файл:строка`` каждого. Пустой обход — ошибка."""
    found: dict[str, list[str]] = {}
    root = repo / "src"
    for path in sorted(root.rglob("*.py")):
        if _SKIP_WALK.intersection(path.parts):
            continue
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except UnicodeError:
            continue
        rel = path.relative_to(repo)
        for number, line in enumerate(lines, start=1):
            for match in _HARD_READ_RE.finditer(line):
                found.setdefault(match.group(2), []).append(f"{rel}:{number}")
    if not found:
        raise SystemExit(f"жёстких чтений REASON_WORDS в {root} не найдено")
    return found


def missing_in_pin(keys: set[str], pin_keys: set[str]) -> list[str]:
    """Ключи монорепо, которых нет в установленной карте."""
    return sorted(keys - pin_keys)


def pin_reason_words(python: str, pin: str) -> set[str]:
    """Ключи ``rob_box_music.source.REASON_WORDS`` из site-packages префикса, не из src/."""
    listing = "import json; from rob_box_music.source import REASON_WORDS; " "print(json.dumps(sorted(REASON_WORDS)))"
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    got = subprocess.run(
        [python, "-I", "-c", listing],
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
        check=False,
    )
    if got.returncode != 0:
        detail = (got.stderr or got.stdout).strip()
        raise SystemExit(f"не прочитал REASON_WORDS пина {pin}: {detail}")
    return set(json.loads(got.stdout.strip().splitlines()[-1]))


def refusal_lines(pin: str, missing: list[str], reads: dict[str, list[str]]) -> list[str]:
    """Текст падения: ключ, тег, место чтения. Пусто — пин покрывает все чтения."""
    lines = []
    for key in missing:
        where = "; ".join(reads.get(key, [])) or "синтетический ключ, в src/ его нет"
        lines.append(f"монорепо читает ключ {key}, в пине {pin} его нет ({where})")
    return lines


def reason_words_missing(
    python: str, pin: str, extra: set[str] | None = None
) -> tuple[dict[str, list[str]], list[str]]:
    """Чтения монорепо и строки отказа для ключей, которых нет в карте пина."""
    reads = hard_key_reads()
    asked = set(reads) | (extra or set())
    absent = missing_in_pin(asked, pin_reason_words(python, pin))
    return reads, refusal_lines(pin, absent, reads)
