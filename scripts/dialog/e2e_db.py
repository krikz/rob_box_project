"""Чтение e2e-БД диалога v2 (таблица ``dialog_events``, ``rob_box_dialog.e2e_events``) по ходам — для turn_metrics и
honesty_audit (ADR-0150 §14: A1, A2, A3, A13, A14; PR-8 #3702).

Только чтение файла БД: рантайм робота этот модуль не импортирует. Виды событий — ``rob_box_dialog.e2e_events.KINDS``.
"""

from __future__ import annotations

import json
import re
import sqlite3
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

TABLE = "dialog_events"
#: Служебный текст, которому не место в речи (A13): маркеры v1 и теги снимка. Только для аудита, не для рантайма.
SERVICE_MARKERS = re.compile(r"\[(?:Spkr|TG|URGENT_BACKLOG|SYSTEM|CRITICAL)[:\]]|</?system_context>|<music_state>"
                             r"|<speaker_id>|\bspeak_text\b", re.IGNORECASE)


@dataclass
class DbTurn:
    """Ход из e2e-БД: реплика человека, речь (начатая и озвученная), исходы действий, команды, агрегат."""

    turn_id: str
    agent: str = ""
    user_ts: Optional[float] = None
    user_text: str = ""
    started: List[Tuple[float, str]] = field(default_factory=list)
    spoken: List[str] = field(default_factory=list)
    actions: List[Dict[str, Any]] = field(default_factory=list)
    commands: List[Dict[str, Any]] = field(default_factory=list)
    aggregate: Dict[str, Any] = field(default_factory=dict)

    @property
    def first_sound_ms(self) -> Optional[float]:
        """A2: от реплики человека до начала первой речи хода, мс."""
        if self.user_ts is None or not self.started:
            return None
        return (min(ts for ts, _ in self.started) - self.user_ts) * 1000.0

    @property
    def tools_done(self) -> Tuple[str, ...]:
        return tuple(a.get("tool", "") for a in self.actions if a.get("status") == "done")

    @property
    def tools_attempted(self) -> Tuple[str, ...]:
        return tuple(a.get("tool", "") for a in self.actions)


def load(path: Path) -> List[DbTurn]:
    """Ходы в порядке первой записи."""
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    try:
        rows = db.execute(f"SELECT ts, turn_id, agent, kind, text, data_json FROM {TABLE} ORDER BY id").fetchall()
    finally:
        db.close()
    turns: Dict[str, DbTurn] = {}
    for ts, turn_id, agent, kind, text, data_json in rows:
        turn = turns.setdefault(turn_id, DbTurn(turn_id, agent))
        _apply(turn, float(ts), kind, text or "", json.loads(data_json or "{}"))
    return list(turns.values())


def _apply(turn: DbTurn, ts: float, kind: str, text: str, data: Dict[str, Any]) -> None:
    if kind == "user":
        turn.user_ts, turn.user_text = ts, text
    elif kind == "speech_started":
        turn.started.append((ts, text))
    elif kind == "speech_spoken":
        turn.spoken.append(text)
    elif kind == "action":
        turn.actions.append(data)
    elif kind == "command":
        turn.commands.append(data)
    elif kind == "turn":
        turn.aggregate = data


def marker_hits(turns: Sequence[DbTurn]) -> List[Tuple[str, str]]:
    """A13: ``(turn_id, текст)`` начатой речи со служебным текстом."""
    return [(t.turn_id, text) for t in turns for _, text in t.started if SERVICE_MARKERS.search(text)]


def per_provider(turns: Sequence[DbTurn]) -> Dict[str, Tuple[int, int]]:
    """A14: ``{провайдер: (ответов LLM, из них невалидных)}`` по событиям ``command`` (одно на ответ модели:
    ``data = {provider, valid, reason}``)."""
    out: Dict[str, Tuple[int, int]] = {}
    for cmd in (c for t in turns for c in t.commands):
        provider = str(cmd.get("provider") or "-")
        total, invalid = out.get(provider, (0, 0))
        out[provider] = (total + 1, invalid + (not cmd.get("valid", False)))
    return out


__all__ = ["DbTurn", "SERVICE_MARKERS", "TABLE", "load", "marker_hits", "per_provider"]
