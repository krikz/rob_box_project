"""Указатели current/previous на локальной ФС (ADR-0168 §7, фаза C, только указатели).

После успешной проверки подписи staged-релиза записывает previous=старый current, current=новый
атомарно (временный файл рядом + os.replace). Не переключает symlink живого compose.
Сети нет: байты release.yaml и подпись уже на диске, маркер stage — локальный файл.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path
from typing import Any

from . import minisign
from . import release_format as rf
from .stage import bundle_staged

CURRENT_NAME = "current"
PREVIOUS_NAME = "previous"


class PointerError(RuntimeError):
    def __init__(self, reason: str, detail: str) -> None:
        super().__init__(f"{reason}: {detail}")
        self.reason = reason
        self.detail = detail


def load(state_dir: Path) -> tuple[str | None, str | None]:
    return _read_name(state_dir / CURRENT_NAME), _read_name(state_dir / PREVIOUS_NAME)


def commit(
    state_dir: Path,
    releases_root: Path,
    node: str,
    data: bytes,
    signature: str,
    pubkey: Path,
) -> dict[str, str | None]:
    """Проверить подпись staged-релиза и сдвинуть указатели. Compose symlink не трогает."""
    release = verify_staged(releases_root, node, data, signature, pubkey)
    return advance(state_dir, release["release"])


def verify_staged(releases_root: Path, node: str, data: bytes, signature: str, pubkey: Path) -> dict[str, Any]:
    if node not in rf.ROLES:
        raise PointerError("bad_node", node)
    try:
        public = minisign.load_public_key(pubkey.read_text(encoding="utf-8"))
        minisign.verify(data, signature, public)
    except OSError as exc:
        raise PointerError("no_pubkey", f"{pubkey}: {exc}") from exc
    except minisign.SignatureError as exc:
        raise PointerError("bad_signature", str(exc)) from exc
    try:
        release = rf.loads(data)
    except rf.ReleaseError as exc:
        raise PointerError("bad_release", str(exc)) from exc
    dest = releases_root / release["release"] / node
    sha = release["roles"][node]["bundle"]["sha256"]
    if not bundle_staged(dest, sha):
        raise PointerError("release_not_staged", str(dest))
    return release


def advance(state_dir: Path, new_release: str) -> dict[str, str | None]:
    """previous=старый current, current=новый. Идемпотентно, если уже current."""
    rf.check_release_name(new_release)
    current, previous = load(state_dir)
    if current == new_release:
        return {"current": current, "previous": previous}
    if current is not None:
        _atomic_write(state_dir / PREVIOUS_NAME, current)
        previous = current
    _atomic_write(state_dir / CURRENT_NAME, new_release)
    return {"current": new_release, "previous": previous}


def _read_name(path: Path) -> str | None:
    try:
        raw = path.read_text(encoding="utf-8").strip()
    except OSError:
        return None
    if not raw:
        return None
    try:
        return rf.check_release_name(raw)
    except rf.ReleaseError as exc:
        raise PointerError("bad_pointer", f"{path}: {exc}") from exc


def _atomic_write(path: Path, value: str) -> None:
    if path.is_symlink():
        raise PointerError("compose_symlink_untouched", str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        fh.write(value)
        fh.write("\n")
    os.replace(tmp, path)
