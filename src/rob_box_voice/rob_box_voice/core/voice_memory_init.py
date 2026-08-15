"""VoiceMemory init helper — encapsulated try-init + safe-save wrapper.

DialogueNode (and any future harness) needs the same boilerplate to:

1. Try to construct a :class:`VoiceMemory` from ``$VOICE_MEMORY_DB_PATH``
   and ``$OLLAMA_BASE_URL``, log a structured status line, and fall
   back to ``None`` on any error (including missing optional deps).
2. Save a turn with try/except that only logs a warning — never
   crashes the LLM loop.

Putting both here keeps the call-sites trivial and the env-var
contract in one place.
"""

from __future__ import annotations

import logging
import os
from typing import Any, Optional, Protocol


class _VoiceMemoryLike(Protocol):
    """Structural type — anything with ``save_turn`` + ``get_stats`` works.

    Both the legacy ``VoiceMemory`` (from :mod:`rob_box_voice.core.voice_memory`)
    and the harness ``MemoryStore`` (``SQLiteVoiceMemory``) satisfy this.
    """

    def save_turn(self, role: str, text: str, *args: Any, **kwargs: Any) -> Any:
        ...

    async def asave_turn(self, role: str, text: str, *args: Any, **kwargs: Any) -> Any:
        ...

    def get_stats(self) -> dict[str, Any]:
        ...


def init_voice_memory(
    *,
    db_path: str | None = None,
    ollama_base_url: str | None = None,
    logger: logging.Logger | None = None,
) -> Optional[_VoiceMemoryLike]:
    """Try to construct the legacy :class:`VoiceMemory`.

    Reads ``$VOICE_MEMORY_DB_PATH`` (default ``/data/voice_memory.db``)
    and ``$OLLAMA_BASE_URL`` (default ``http://localhost:11434``) when
    the explicit args are ``None``. Returns ``None`` on any failure.

    Args:
        db_path: Override for the SQLite path (default: env var).
        ollama_base_url: Override for the Ollama endpoint (default: env var).
        logger: Optional logger to receive init/status lines.
    """
    log = logger or logging.getLogger(__name__)
    try:
        from rob_box_voice.core.voice_memory import VoiceMemory as _VoiceMemory
    except ImportError:
        log.warning("⚠️ VoiceMemory unavailable — turn logging disabled")
        return None

    db = db_path or os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
    ollama = ollama_base_url or os.getenv("OLLAMA_BASE_URL", "http://localhost:11434")
    try:
        memory = _VoiceMemory(db_path=db, ollama_base_url=ollama)
        stats = memory.get_stats()
        log.info(
            f"🧠 VoiceMemory: {db} (turns={stats.get('turn_count', '?')}, "
            f"sessions={stats.get('session_count', '?')})"
        )
        return memory
    except Exception as exc:
        log.error(f"❌ VoiceMemory init failed: {exc}")
        return None


def safe_save_turn(
    memory: Optional[_VoiceMemoryLike],
    role: str,
    text: str,
    *,
    logger: logging.Logger | None = None,
) -> bool:
    """Save a turn to ``memory`` if available; swallow errors.

    Returns True if the save was attempted (regardless of outcome),
    False if ``memory`` was None and the call was skipped.
    """
    if memory is None:
        return False
    log = logger or logging.getLogger(__name__)
    try:
        memory.save_turn(role, text)
        return True
    except Exception as exc:
        log.warning(f"⚠️ memory save_turn({role}) failed: {exc}")
        return True  # attempted, just failed


async def asafe_save_turn(
    memory: Optional[_VoiceMemoryLike],
    role: str,
    text: str,
    *,
    logger: logging.Logger | None = None,
) -> bool:
    """Async variant of :meth:`safe_save_turn` -- never blocks the loop.

    Prefer this from the ROS event loop: the Ollama embedding inside
    ``asave_turn`` is awaited instead of blocking the callback thread.
    """
    if memory is None:
        return False
    log = logger or logging.getLogger(__name__)
    try:
        save = getattr(memory, "asave_turn", None)
        if save is not None:
            await save(role, text)
        else:
            memory.save_turn(role, text)
        return True
    except Exception as exc:
        log.warning(f"⚠️ memory asave_turn({role}) failed: {exc}")
        return True  # attempted, just failed


__all__ = ["init_voice_memory", "safe_save_turn", "asafe_save_turn"]