"""Метка «кто сказал» в бэклоге диалога (ADR-0150 §7.5).

Функция, а не метод: ``DialogueNode`` в бюджете классов ADR-0145 расти не может.
"""

from __future__ import annotations

from typing import Any, Mapping

from rob_box_dialog.speaker_adapter import voice_tag
from rob_box_voice.core.epithets import is_stranger_epithet


def note_backlog_voice(node: Any, utterance_id: str, data: Mapping[str, Any]) -> None:
    """Биометрия приходит после STT: что она сказала о фразе (узнан/гипотеза/лучший кандидат, кличка) дописывается
    фразе в бэклоге — метка «кто сказал». Кличка-плейсхолдер «незнакомца» тёзок не различает."""
    backlog = getattr(node, "_backlog", None)
    if backlog is None:
        return
    if is_stranger_epithet(data.get("epithet")):
        data = {**data, "epithet": None}
    backlog.note_voice(utterance_id, voice_tag(data))
