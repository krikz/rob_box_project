"""
speak_helpers.py — Sentence splitting and TTS publish helpers.

Extracted from the legacy ``dialogue_node.py`` so the ROS2 shell stays
≤350 LOC while keeping the same race-protected TTS / sound awaiters
and SSML framing that the production pipeline needs.

These helpers are used by :class:`rob_box_voice.dialogue_node.DialogueNode`
to publish fallback responses (``_speak_direct``) and to release the
``speak_text`` / ``play_sound`` awaiters on ``/voice/tts/finished`` and
``/voice/sound/state``.

The eventual destination of this code is :mod:`rob_box_harness.effects`
(SpeakEffect / PlaySoundEffect); until that migration lands the
helpers live here to keep the W5 commit scoped.
"""

from __future__ import annotations

import asyncio
import json
import re
import threading
import uuid
from typing import Any, Callable, Dict, List, Optional

# Strip history marker prefix that some LLMs copy into output.
_HISTORY_MARKER_RE = re.compile(
    r"^\[(?:выполнено через|executed via):[^\]]*\]\s*",
    flags=re.IGNORECASE,
)


def strip_history_marker(text: str) -> str:
    """Return *text* with the leading ``[выполнено через: ...]`` marker removed."""
    if not text:
        return text
    return _HISTORY_MARKER_RE.sub("", text).strip()


def split_into_chunks(text: str, max_len: int = 200) -> List[str]:
    """Sentence-aware splitter — keeps TTS requests under the SSML limit."""
    raw = re.split(r"(?<=[.!?;])\s+", text.strip())
    chunks: List[str] = []
    buf = ""
    for part in raw:
        part = part.strip()
        if not part:
            continue
        candidate = (buf + " " + part).strip() if buf else part
        if len(candidate) <= max_len:
            buf = candidate
        else:
            if buf:
                chunks.append(buf)
            if len(part) > max_len:
                sub_parts = re.split(r"(?<=,)\s+", part)
                sub_buf = ""
                for sp in sub_parts:
                    sub_c = (sub_buf + " " + sp).strip() if sub_buf else sp
                    if len(sub_c) <= max_len:
                        sub_buf = sub_c
                    else:
                        if sub_buf:
                            chunks.append(sub_buf)
                        sub_buf = sp
                buf = sub_buf
            else:
                buf = part
    if buf:
        chunks.append(buf)
    return [c for c in chunks if c.strip()] or [text]


def build_ssml_payload(text: str, animation: str = "neutral") -> str:
    """Build the JSON string consumed by ``tts_node`` on ``/voice/dialogue/response``."""
    return json.dumps(
        {
            "ssml": f"<speak>{text}</speak>",
            "speech_id": str(uuid.uuid4()),
            "emotion": animation,
        },
        ensure_ascii=False,
    )


class EffectAwaiterRegistry:
    """Tracks in-flight TTS / sound awaiters keyed by speech_id.

    Mirrors the field set that lived inline in the legacy
    ``DialogueNode``:

    * ``tts_events`` — speech_id → ``asyncio.Event`` released on
      ``/voice/tts/finished``.
    * ``sound_done_event`` — single ``asyncio.Event`` released on
      ``/voice/sound/state`` ``"ready"``.

    The registry owns its own locks so the shell doesn't need to
    juggle threading.Lock instances inline. Callers wire up two
    callbacks (``release_tts`` / ``release_sound``) to schedule
    ``event.set()`` back on the asyncio loop thread-safely.
    """

    def __init__(
        self,
        *,
        release_tts: Callable[[Any], None],
        release_sound: Callable[[asyncio.Event], None],
    ) -> None:
        self._tts_events: Dict[str, Any] = {}
        self._tts_lock = threading.Lock()
        self._sound_done_event: Optional[Any] = None
        self._sound_lock = threading.Lock()
        self._release_tts = release_tts
        self._release_sound = release_sound

    # ── TTS side ────────────────────────────────────────────────────

    def register_tts(self, speech_id: str, event: asyncio.Event) -> None:  # type: ignore[type-arg]
        with self._tts_lock:
            self._tts_events[speech_id] = event

    @property
    def has_pending_tts(self) -> bool:
        """True while at least one TTS chunk is still playing / synthesising."""
        with self._tts_lock:
            return len(self._tts_events) > 0

    def pop_tts(self, speech_id: str) -> Optional[asyncio.Event]:  # type: ignore[type-arg]
        with self._tts_lock:
            return self._tts_events.pop(speech_id, None)

    def release_all_tts(self) -> None:
        with self._tts_lock:
            for event in self._tts_events.values():
                self._release_tts(event)
            self._tts_events.clear()

    def handle_tts_finished(self, payload: str) -> None:
        try:
            speech_id = json.loads(payload).get("speech_id", "")
        except (json.JSONDecodeError, TypeError, AttributeError):
            speech_id = (payload or "").strip()
        if not speech_id:
            return
        event = self.pop_tts(speech_id)
        if event is not None:
            self._release_tts(event)

    # ── Sound side ──────────────────────────────────────────────────

    def set_sound_event(self, event: asyncio.Event) -> None:  # type: ignore[type-arg]
        with self._sound_lock:
            self._sound_done_event = event

    def clear_sound_event(self) -> None:
        with self._sound_lock:
            event = self._sound_done_event
            self._sound_done_event = None
        if event is not None:
            self._release_sound(event)

    def handle_sound_state(self, payload: str) -> None:
        if (payload or "") != "ready":
            return
        with self._sound_lock:
            event = self._sound_done_event
        if event is not None:
            self._release_sound(event)


__all__ = [
    "strip_history_marker",
    "split_into_chunks",
    "build_ssml_payload",
    "EffectAwaiterRegistry",
]
