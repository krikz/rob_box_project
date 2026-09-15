"""Pure text classification helpers used by the tool loop.

Issue #2630 — ``_is_pseudo_tool_call`` and ``_PSEUDO_TOOL_CALL_RE``
were previously defined in ``agent_core.py`` and imported from
``retry.py``, which produced a circular import when ``agent_core``
started importing tool-loop helpers. Promoting them here keeps the
dependency graph one-way: ``agent_core`` → ``tool_loop``, never
back. ``agent_core`` keeps its own private aliases for back-compat
with any external importer that still references them.
"""

from __future__ import annotations

import re

#: Pattern matching a tool call the model WROTE as text instead of
#: emitting through function-calling. The shape is ``"<name>{...}"`` or
#: ``"<name>(...)"`` — a single angle-bracketed line. Anything else
#: (paragraphs, prose around the call, ...) is NOT a pseudo-call; the
#: LLM probably meant it as natural language.
_PSEUDO_TOOL_CALL_RE: re.Pattern[str] = re.compile(r"^<[^<>]{1,160}>$")


def is_pseudo_tool_call(text: str) -> bool:
    """Return ``True`` when ``text`` is a tool call the model WROTE instead of made."""
    return bool(_PSEUDO_TOOL_CALL_RE.match((text or "").strip()))


__all__ = ["_PSEUDO_TOOL_CALL_RE", "is_pseudo_tool_call"]