"""person_photo.py - снимок лица из каталога лиц в чат (issue #4469).

Тул ``person_photo`` (mcp_server) отдаёт только путь внутри корня лиц: ``<id>/reference.jpg`` или
``<id>/encounters/<файл>.jpg``. Файл читает бот: он монтирует ``./data/faces:/data/faces:ro`` (как ``face_card``).
Путь приходит из данных владельца, но проверяется здесь по строгому шаблону: наружу корня выйти нельзя.
"""

from __future__ import annotations

import json
import os
import re
from pathlib import Path
from typing import Optional, Tuple

#: Корень лиц в контейнере бота (то же имя, что ``rob_box_perception.face_store.DEFAULT_ROOT``).
FACES_ROOT_ENV = "FACES_ROOT"
DEFAULT_FACES_ROOT = "/data/faces"
_PATH_RE = re.compile(r"[0-9a-f\-]{8,36}/(?:reference|encounters/[0-9]{6}_face)\.jpg")
#: Подпись снимка в Telegram - до 1024 знаков; наша короче, это страховка.
CAPTION_MAX = 200


def faces_root() -> Path:
    return Path(os.environ.get(FACES_ROOT_ENV, DEFAULT_FACES_ROOT))


def parse_payload(text: str) -> Optional[Tuple[str, str]]:
    """``(путь, подпись)`` из текста ``DialogOutput(kind=photo)`` или ``None``, если это не наш JSON."""
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    path, caption = data.get("path") if isinstance(data, dict) else None, str((data or {}).get("caption") or "")
    return (path, caption[:CAPTION_MAX]) if isinstance(path, str) else None


def read_photo(rel_path: str, root: Optional[Path] = None) -> Optional[bytes]:
    """Байты снимка по пути из данных владельца; путь вне шаблона или файла нет - ``None``."""
    if not _PATH_RE.fullmatch(rel_path):
        return None
    base = (root or faces_root()).resolve()
    target = (base / rel_path).resolve()
    if base not in target.parents or not target.is_file():
        return None
    return target.read_bytes()
