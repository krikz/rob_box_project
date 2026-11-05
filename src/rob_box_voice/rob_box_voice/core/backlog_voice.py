"""Метка «кто сказал» в бэклоге диалога (ADR-0150 §7.5).

Функция, а не метод: ``DialogueNode`` в бюджете классов ADR-0145 расти не может.
"""

from __future__ import annotations

from typing import Any, Mapping

from rob_box_dialog.speaker_adapter import voice_tag


def note_backlog_voice(node: Any, utterance_id: str, data: Mapping[str, Any]) -> None:
    """Биометрия приходит после STT: что она сказала о фразе (узнан/гипотеза/лучший кандидат, кличка) дописывается
    фразе в бэклоге — метка «кто сказал». Плейсхолдер «незнакомца» сюда не доходит: его отсекает издатель
    (``speaker_id_node._published_epithet``, ADR-0164 этап 3)."""
    backlog = getattr(node, "_backlog", None)
    if backlog is None:
        return
    backlog.note_voice(utterance_id, voice_tag(data))
