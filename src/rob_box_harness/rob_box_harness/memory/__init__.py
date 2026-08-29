"""Memory package — real and fake memory store implementations.

* :mod:`rob_box_harness.memory.base` — ``MemoryStore`` protocol и
  ``InMemoryStore`` (фейк для тестов).
* :mod:`rob_box_harness.memory.sqlite_voice` — живой ``SQLiteVoiceMemory``.

Раньше ``base.py`` лежал рядом с пакетом как ``memory.py``. Питон при
конфликте «модуль и пакет с одним именем» всегда выбирает пакет, поэтому
модуль был недостижим обычным импортом, и этот ``__init__`` подгружал его
вручную через ``importlib.util.spec_from_file_location`` по относительному
пути ``../memory.py``. Хак работал, но исполнял модуль под чужим именем
(``rob_box_harness._memory_module``): второй способ импортировать тот же
файл дал бы второй объект класса ``Turn``, и ``isinstance`` тихо вернул бы
``False``. Файл переехал внутрь пакета — хак больше не нужен.
"""

from __future__ import annotations

from rob_box_harness.memory.base import (
    SPEAKER_PROFILE_KEY,
    SPEAKER_SCOPE_PREFIX,
    FAQItem,
    Fact,
    InMemoryStore,
    MemoryStore,
    Turn,
    Waypoint,
    ensure_speaker_profile,
    get_speaker_profile,
    merge_speaker_facts,
    speaker_scope,
    touch_speaker,
)

# ── Lazy-load SQLiteVoiceMemory (avoids importing sqlite3 at package-init) ──

_LAZY_NAMES = {"SQLiteVoiceMemory"}


def __getattr__(name: str):
    if name in _LAZY_NAMES:
        from rob_box_harness.memory.sqlite_voice import SQLiteVoiceMemory as _svm

        globals()[name] = _svm
        return _svm
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "Turn",
    "Fact",
    "Waypoint",
    "FAQItem",
    "MemoryStore",
    "InMemoryStore",
    "SPEAKER_SCOPE_PREFIX",
    "SPEAKER_PROFILE_KEY",
    "speaker_scope",
    "get_speaker_profile",
    "ensure_speaker_profile",
    "touch_speaker",
    "merge_speaker_facts",
    "SQLiteVoiceMemory",
]
