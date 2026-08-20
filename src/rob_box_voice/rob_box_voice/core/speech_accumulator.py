#!/usr/bin/env python3
"""SpeechAccumulator — аккумулятор фоновой речи без wake-слова.

Чистый модуль: без I/O, без ROS2. DialogueNode держит один экземпляр,
кладет сюда распознанные фразы без wake-word и сливает их в
``<speech_backlog>`` при следующем wake-word (см. docs/plans/
2026-08-20-voice-backlog-accumulator-design.md).
"""

from __future__ import annotations

import time
from typing import List, Optional

DEFAULT_MAX_ENTRIES = 30

_INSTRUCTION = (
    "Ниже — фразы, услышанные без обращённого wake-слова до текущего "
    "обращения. Пользователь мог иметь в виду одну из них как запрос. "
    "Определи, о чём просили, и ответь по существу, не пересказывая "
    "эти фразы дословно."
)


def _xml_escape(value: str) -> str:
    return (
        value.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def format_ago_s(seconds: float) -> str:
    """Человекочитаемое «сколько назад» для записи бэклога."""
    seconds = max(0, int(seconds))
    if seconds < 60:
        return f"{seconds}с"
    minutes = seconds // 60
    if minutes < 60:
        return f"{minutes}м{seconds % 60}с"
    hours = minutes // 60
    return f"{hours}ч{minutes % 60}м"


class SpeechAccumulator:
    """Скользящее окно распознанной речи без wake-слова."""

    def __init__(
        self,
        window_sec: float = 180.0,
        max_entries: int = DEFAULT_MAX_ENTRIES,
    ) -> None:
        self.window_sec = window_sec
        self.max_entries = max_entries
        self._entries: List[dict] = []

    def add(
        self,
        text: str,
        speaker_tag: Optional[str] = None,
        speaker_name: Optional[str] = None,
    ) -> None:
        text = (text or "").strip()
        if not text:
            return
        self._entries.append(
            {
                "ts": time.time(),
                "text": text,
                "speaker_tag": speaker_tag,
                "speaker_name": speaker_name or "незнакомец",
            }
        )
        self._trim()

    def prune(self, now: Optional[float] = None) -> None:
        now = time.time() if now is None else now
        cutoff = now - self.window_sec
        self._entries = [e for e in self._entries if e["ts"] >= cutoff]
        self._trim()

    def _trim(self) -> None:
        if len(self._entries) > self.max_entries:
            self._entries = self._entries[-self.max_entries:]

    def is_empty(self) -> bool:
        return not self._entries

    def clear(self) -> None:
        self._entries.clear()

    def format_block(self, now: Optional[float] = None) -> Optional[str]:
        """XML-блок ``<speech_backlog>`` или ``None``, если пусто."""
        self.prune(now)
        if self.is_empty():
            return None
        now = time.time() if now is None else now
        lines = [
            "<speech_backlog>",
            f"  <instruction>{_xml_escape(_INSTRUCTION)}</instruction>",
        ]
        for entry in self._entries:
            ago_s = format_ago_s(now - entry["ts"])
            tag = entry["speaker_tag"] or "?"
            speaker = _xml_escape(entry["speaker_name"] or "незнакомец")
            text = _xml_escape(entry["text"])
            lines.append(
                f'  <entry speaker_tag="{tag}" speaker="{speaker}" '
                f'ago_s="{ago_s}">{text}</entry>'
            )
        lines.append("</speech_backlog>")
        return "\n".join(lines)
