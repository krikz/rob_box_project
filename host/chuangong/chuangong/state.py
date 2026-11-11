"""Состояние агента chuangong: перечень состояний, ключи zenoh, запись state.json, метрика rbx_release_info.

Фразы о состоянии строит код из этих полей (ADR-0148): наблюдатель workflow печатает их как есть.
"""

from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any

from . import AGENT_VERSION

STATE_SCHEMA = 1
NODES = ("main", "vision", "sound")


class State(str, Enum):
    NO_RELEASE = "no_release"  # указателя канала в реестре нет
    AVAILABLE = "available"  # релиз найден и подпись верна, на диск не скачан (режим observe)
    STAGED = "staged"  # бандл и образы роли лежат на диске заранее
    FETCH_ERROR = "fetch_error"  # реестр недоступен, подпись не прошла, release.yaml битый
    STAGE_ERROR = "stage_error"  # нет места, обрыв скачивания, бандл не сошёлся, pull упал — повтор на следующем тике


ERROR_STATES = frozenset({State.FETCH_ERROR, State.STAGE_ERROR})


class Mode(str, Enum):
    OBSERVE = "observe"  # fetch + verify + report; вне state_dir ничего не пишет (по умолчанию)
    STAGE = "stage"  # плюс бандл в releases_root и `docker pull` образов роли по digest


def state_key(robot_id: str, node: str) -> str:
    """Ключ состояния узла. Сводный `robots/<id>/ops/chuangong/state` (без узла) — за старшим агентом, фаза C."""
    return f"robots/{robot_id}/ops/chuangong/state/{node}"


def state_selector(robot_id: str) -> str:
    return f"robots/{robot_id}/ops/chuangong/state/**"


def now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def new_record(robot_id: str, node: str, channel: str, mode: Mode, prev: dict[str, Any] | None) -> dict[str, Any]:
    """Каркас записи тика; `current/previous/staged` переносятся из прошлого состояния."""
    prev = prev or {}
    return {
        "schema": STATE_SCHEMA,
        "agent_version": AGENT_VERSION,
        "robot_id": robot_id,
        "node": node,
        "channel": channel,
        "mode": mode.value,
        "ts": now_iso(),
        "state": None,
        "reason": None,
        # current/previous меняет только активация (фаза C); в фазе B это всегда null — робот на старом деплое.
        "current": prev.get("current"),
        "previous": prev.get("previous"),
        "staged": prev.get("staged"),
        "available": None,
        "release_git": None,
        "hil_locked": None,
        "dirty": None,
        "dirty_sources": [],
        "stage": None,
        "disk": None,
        "dry_run": False,
    }


def load(path: Path) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def save(path: Path, record: dict[str, Any]) -> None:
    """Атомарная запись: временный файл рядом + rename."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        json.dump(record, fh, ensure_ascii=False, indent=2, sort_keys=True)
    os.replace(tmp, path)


def _label(value: Any) -> str:
    return str(value if value is not None else "none").replace("\\", "\\\\").replace('"', '\\"')


def metrics_text(record: dict[str, Any]) -> str:
    """Prometheus textfile: rbx_release_info{robot_id,node,release,state} 1 (+ флаги HIL и dirty)."""
    release = record.get("staged") or record.get("available") or record.get("current")
    labels = (
        f'robot_id="{_label(record["robot_id"])}",node="{_label(record["node"])}",'
        f'release="{_label(release)}",state="{_label(record["state"])}"'
    )
    base = f'robot_id="{_label(record["robot_id"])}",node="{_label(record["node"])}"'
    return (
        "# HELP rbx_release_info Релиз на узле робота по агенту chuangong (ADR-0168).\n"
        "# TYPE rbx_release_info gauge\n"
        f"rbx_release_info{{{labels}}} 1\n"
        "# HELP rbx_release_dirty 1 — на узле горячая заливка (hotpatch), активация запрещена.\n"
        "# TYPE rbx_release_dirty gauge\n"
        f"rbx_release_dirty{{{base}}} {1 if record.get('dirty') else 0}\n"
        "# HELP rbx_release_hil_locked 1 — на Main Pi замок HIL.\n"
        "# TYPE rbx_release_hil_locked gauge\n"
        f"rbx_release_hil_locked{{{base}}} {1 if record.get('hil_locked') else 0}\n"
    )
