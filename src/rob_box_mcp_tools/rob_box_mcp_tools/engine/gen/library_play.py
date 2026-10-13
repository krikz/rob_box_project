"""Трек кэша на деке движка (ADR-0158 §2.4, PR-2; тулом перестал быть в PR-9).

Одна функция — :func:`play_library_track`: её зовут тул ``music_library(action=play)`` и внутренние исполнители
(кавер PR-4, песня PR-7, сет PR-8, путь PR-3). Внутренний вызов — функция, а не тул по реестру; тул ``music_library`` нужен только просьбе человека.

Звук — через владельца плеера (``PlayerOwner``), как синтез: ``AudioTrack`` (wav частоты сервера,
``engine.gen.audio``) → ``render_audio`` → ``play(once=True)``; ``ok`` — только по ``started`` этого запуска.
Идущий сет закрывается до запуска (одна дека — одна музыка). Фразу запуска и отказа строит код (ADR-0148).
"""

from __future__ import annotations

import subprocess
import time
from typing import Any, Callable

from ...base import MCPToolResult
from .audio import AudioUnavailable, audio_track


def play_library_track(node: Any, library: Any, track_id: str, clock: Callable[[], float] = time.time,
                       run: Callable[..., Any] = subprocess.run) -> MCPToolResult:
    """Сыграть трек ``track_id`` кэша на деке владельца. ``node`` даёт ``_player_owner``, ``_dj_set_tool``
    (закрыть идущий сет) и ``_music_confirm`` (ожидание ``started``)."""
    from rob_box_music.render.audio import render_audio
    from rob_box_voice.core.media_phrases import library_play_text

    from ..tools_v2 import confirmed

    info = library.get_track_info(track_id)
    if not info.get("success"):
        return MCPToolResult(success=False, error=info.get("error"))
    owner = getattr(node, "_player_owner", None)
    if owner is None:
        return MCPToolResult(success=False, error="музыкальный движок недоступен — трек не заиграл")
    deck_id = f"gen_{track_id[:8]}_{int(clock())}"
    try:
        track = audio_track(info, deck_id, run=run)
        program = render_audio(track, "A")
    except AudioUnavailable as exc:
        return MCPToolResult(success=False, data={"reason": exc.reason}, error=f"трек не заиграл: {exc.reason}")
    title = str(info.get("title") or "")
    facts = {"enabled": False, "title": title, "source": "generated", "library_id": track_id,
             "bpm": info.get("bpm_nominal") or None}
    dj_set = getattr(node, "_dj_set_tool", None)
    if dj_set is not None:
        dj_set.close_set("music_library")
    result = confirmed(owner.play(program, dj=facts, once=True), getattr(node, "_music_confirm", None))
    data = {**result, "library_id": track_id, "title": title, "duration_s": round(track.duration_s, 1)}
    if not result.get("ok"):
        return MCPToolResult(success=False, data=data, error=f"трек не заиграл: {result.get('reason')}")
    library.increment_play_count(track_id)
    return MCPToolResult(success=True, data=data, message=library_play_text(title, track.duration_s))


__all__ = ["play_library_track"]
