"""renardo_events — реэкспорт офлайн-симулятора из ``rob_box_music.render.events`` (ADR-0149 §9 PR-1b).

Логика живёт в одном месте — ``rob_box_music/render/events.py``. Этот модуль остаётся
только ради старых потребителей (``core.classic_loudness``, тесты) и удаляется вместе
со старым путём (PR-13…15).
"""

from __future__ import annotations

from rob_box_music.knowledge import CHROMATIC, ROOTS as NOTE_NAMES, SCALES
from rob_box_music.render.events import (
    FX_KEYS,
    SLOTS,
    NoteEvent,
    PlayerSpec,
    Program,
    ProgramError,
    TimeVar,
    degree_to_midi,
    events_for,
    program_events,
    run_program,
    value_at,
)

__all__ = [
    "CHROMATIC",
    "FX_KEYS",
    "NOTE_NAMES",
    "NoteEvent",
    "PlayerSpec",
    "Program",
    "ProgramError",
    "SCALES",
    "SLOTS",
    "TimeVar",
    "degree_to_midi",
    "events_for",
    "program_events",
    "run_program",
    "value_at",
]
