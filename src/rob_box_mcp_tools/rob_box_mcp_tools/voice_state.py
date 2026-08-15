#!/usr/bin/env python3
"""
voice_state.py — in-memory хранилище выбранного голоса TTS (issue #1219, Q7/Q11).

LLM может персистентно сменить голос на диалог через инструмент
``set_voice(voice=...)``. Текущий голос хранится in-memory (промежуточное
решение до появления пользовательского профиля) с привязкой к speaker_id:
разные собеседники (когда распознавание спикера включено) получают свой голос.

Хранилище используется инструментами MCP (``SpeakTextTool`` /
``SetVoiceTool``) в процессе mcp_server — оба инструмента живут в одном
процессе, поэтому простого потокобезопасного dict достаточно.

Дизайн-решения (docs/design/LLM_VOICE_SELECTION_PROPOSAL.md):

* ``set_voice(voice=...)`` держит голос на диалог, пока не сменят (Q7);
* по умолчанию (без set_voice) — дефолтный голос TTS (Q7);
* при фолбеке провайдера используется дефолт фолбек-провайдера; при
  возврате исходного провайдера восстанавливается установленный голос (Q11)
  — это поведение обеспечивает tts_node (ререзолв по фактическому
  провайдеру), хранилище здесь — источник «установленного» голоса.
"""

from __future__ import annotations

import threading
from typing import Optional

# Issue #1219 — единый реестр голосов живёт в rob_box_voice (чистый Python).
# Ленивый импорт с fallback, чтобы пакет оставался импортируемым без
# rob_box_voice (CI linter / минимальные окружения).
try:
    from rob_box_voice.tts_voice_registry import default_voice_for
except ImportError:  # pragma: no cover — fallback для minimal environments
    def default_voice_for(provider: str) -> str:
        return ""


class VoiceStateStore:
    """Потокобезопасное in-memory хранилище current_voice по speaker_id.

    Ключ ``None``/``"default"`` — голос по умолчанию для неизвестного
    спикера. Когда распознавание спикера включено, ключ — speaker_id из
    /voice/speaker/result (пока профиль не сформирован — см. Q11).
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._voices: dict[str, str] = {}

    @staticmethod
    def _key(speaker_id: str | None) -> str:
        return speaker_id or "default"

    def set_voice(self, voice: str, speaker_id: str | None = None) -> None:
        """Запомнить установленный голос для спикера (или default)."""
        with self._lock:
            self._voices[self._key(speaker_id)] = voice

    def get_voice(self, speaker_id: str | None = None) -> Optional[str]:
        """Установленный голос спикера (None — голос не меняли)."""
        with self._lock:
            return self._voices.get(self._key(speaker_id))

    def resolve(
        self,
        provider: str,
        requested: str | None = None,
        speaker_id: str | None = None,
    ) -> tuple[str, bool]:
        """Фактический голос для speak_text (Q7).

        Приоритет:
        1. ``requested`` — разовый голос из speak_text(voice=...);
        2. ``current_voice`` — установленный set_voice (персистентно);
        3. дефолтный голос провайдера.

        Returns:
            ``(voice_used, fell_back)`` — голос и флаг, что пришлось
            откатиться на дефолт (запрошенный неизвестен).
        """
        try:
            from rob_box_voice.tts_voice_registry import resolve_voice as _resolve
        except ImportError:  # pragma: no cover — fallback для minimal environments
            def _resolve(provider: str, requested):
                return (requested or ""), False

        if requested:
            return _resolve(provider, requested)
        current = self.get_voice(speaker_id)
        if current:
            voice_used, _ = _resolve(provider, current)
            return voice_used, False
        return _resolve(provider, None)

    def default_for(self, provider: str) -> str:
        """Дефолтный голос провайдера (для результата set_voice)."""
        return default_voice_for(provider)

    def clear(self) -> None:
        """Сброс (тесты / смена диалога)."""
        with self._lock:
            self._voices.clear()
