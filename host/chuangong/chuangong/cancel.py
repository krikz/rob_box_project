"""Локальная отмена staged-релиза (ADR-0168 фаза C): пометить отменённым, current не трогать.

Сети нет. Compose symlink не переключает. Указатели читает через `pointers.load`, не вызывает
`pointers.advance`/`commit`. Тик агента маркер ещё не читает.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from . import pointers
from . import state as st
from .stage import MARKER

CANCEL_MARKER = ".chuangong-cancelled"


class CancelError(RuntimeError):
    def __init__(self, reason: str, detail: str) -> None:
        super().__init__(f"{reason}: {detail}")
        self.reason = reason
        self.detail = detail


def cancel_staged(state_dir: Path, releases_root: Path, node: str) -> dict[str, Any]:
    """Пометить staged-релиз отменённым. current/previous не меняет."""
    if node not in st.NODES:
        raise CancelError("bad_node", node)
    with st.write_lock(state_dir):
        return _cancel_staged(state_dir, releases_root, node)


def _cancel_staged(state_dir: Path, releases_root: Path, node: str) -> dict[str, Any]:
    before = pointers.load(state_dir)
    prev = st.load(state_dir / "state.json") or {}
    name = prev.get("staged")
    if not name:
        raise CancelError("nothing_staged", str(state_dir / "state.json"))
    dest = releases_root / name / node
    if not (dest / MARKER).is_file():
        raise CancelError("release_not_staged", str(dest))
    (dest / CANCEL_MARKER).write_text(name + "\n", encoding="utf-8")
    mode_raw = prev.get("mode") or st.Mode.OBSERVE.value
    record = st.new_record(
        prev.get("robot_id") or "",
        node,
        prev.get("channel") or "dev",
        st.Mode(mode_raw),
        prev,
    )
    record["staged"] = None
    record["cancelled"] = name
    record["state"] = st.State.CANCELLED.value
    st.save(state_dir / "state.json", record)
    after = pointers.load(state_dir)
    if after != before:
        raise CancelError("pointers_changed", f"{before} -> {after}")
    return record
