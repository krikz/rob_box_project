"""Ключи API из ``.env`` для сухого стенда и консоли (карточка #4163): разбор файла и загрузка в окружение.

Значения нигде не печатаются: наружу идут только имена файлов и число применённых переменных.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


def parse_env_file(path: Path) -> Dict[str, str]:
    """Разобрать ``KEY=VALUE``: пропускает пустые строки и ``#``, снимает ``export`` и одну пару кавычек. Значение
    берётся как есть: в ключе API законно бывают ``#`` и ``=``."""
    values: Dict[str, str] = {}
    try:
        raw = Path(path).read_text(encoding="utf-8-sig")
    except OSError:
        return values
    for line in raw.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("export "):
            stripped = stripped[len("export "):].lstrip()
        key, sep, value = stripped.partition("=")
        key = key.strip()
        if not sep or not key:
            continue
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        values[key] = value
    return values


def load_env_files(paths: Sequence[Path]) -> List[Tuple[Path, int]]:
    """Загрузить файлы в ``os.environ``, не затирая заданное: раньше стоящий файл важнее позднего, экспортированное в
    шелле - важнее любого файла. Пустое значение не применяется. Возвращает ``(путь, сколько переменных применено)``
    для файлов, которые есть."""
    loaded: List[Tuple[Path, int]] = []
    for path in paths:
        if not Path(path).is_file():
            continue
        applied = 0
        for key, value in parse_env_file(path).items():
            if value and not os.environ.get(key):
                os.environ[key] = value
                applied += 1
        loaded.append((Path(path), applied))
    return loaded


__all__ = ["load_env_files", "parse_env_file"]
