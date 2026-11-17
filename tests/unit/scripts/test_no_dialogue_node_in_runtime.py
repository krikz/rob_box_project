"""#4354 A5: имени ROS-ноды ``/dialogue_node`` (диалог v1, удалён) нет в рабочем коде и конфигах робота.

Исключений нет: появилась ссылка - тест краснеет.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NODE_NAME = re.compile(r"(?<![A-Za-z0-9_])/dialogue_node(?![._a-z])")
SUFFIXES = {".py", ".yaml", ".yml", ".sh", ".json"}


def _runtime_files():
    for top in ("src", "docker", "config"):
        base = ROOT / top
        if not base.is_dir():
            continue
        for path in base.rglob("*"):
            if path.suffix not in SUFFIXES or not path.is_file():
                continue
            parts = set(path.relative_to(ROOT).parts)
            if parts & {"test", "tests", "build", "install", "__pycache__", "node_modules"}:
                continue
            yield path


def test_dialogue_node_name_is_absent_from_runtime_code():
    hits = []
    for path in _runtime_files():
        rel = path.relative_to(ROOT).as_posix()
        for number, line in enumerate(path.read_text(encoding="utf-8", errors="ignore").splitlines(), 1):
            if NODE_NAME.search(line):
                hits.append(f"{rel}:{number}: {line.strip()[:100]}")
    assert not hits, "ссылки на удалённую ноду /dialogue_node:\n" + "\n".join(hits)
