"""Memory package — real and fake memory store implementations.

When both ``memory.py`` (module) and ``memory/`` (package) exist,
Python resolves ``rob_box_harness.memory`` to the **package**.
This init re-exports everything from the original module file so
existing ``from rob_box_harness.memory import MemoryStore`` imports
continue to work alongside the new ``SQLiteVoiceMemory``.
"""

from __future__ import annotations

import importlib.util
import os
import sys

# ── Re-export everything from the memory.py module ─────────────
_memory_py = os.path.join(os.path.dirname(__file__), "..", "memory.py")
_spec = importlib.util.spec_from_file_location(
    "rob_box_harness._memory_module",
    os.path.abspath(_memory_py),
)
_memory_mod = importlib.util.module_from_spec(_spec)
sys.modules["rob_box_harness._memory_module"] = _memory_mod
_spec.loader.exec_module(_memory_mod)

_memory_all = getattr(_memory_mod, "__all__", None)
if _memory_all is not None:
    for _name in _memory_all:
        globals()[_name] = getattr(_memory_mod, _name)
else:
    for _name in dir(_memory_mod):
        if not _name.startswith("_"):
            globals()[_name] = getattr(_memory_mod, _name)

# ── Lazy-load SQLiteVoiceMemory (avoids importing sqlite3 at package-init) ──

_LAZY_NAMES = {"SQLiteVoiceMemory"}


def __getattr__(name: str):
    if name in _LAZY_NAMES:
        from rob_box_harness.memory.sqlite_voice import SQLiteVoiceMemory as _svm

        globals()[name] = _svm
        return _svm
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    *_memory_all,
    "SQLiteVoiceMemory",
]
