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

# Issue #2760 — вызов тула, напечатанный текстом (``<function_calls><invoke …>``): тот же класс ошибки, что
# ``_PSEUDO_TOOL_CALL_RE`` ниже, но в полном синтаксисе и на несколько строк. Регекс и предикат — одна копия в
# ``rob_box_core.tool_call_markup``: ``tts_node`` («теги не звучат») берёт их оттуда же, второй копии нет.
from rob_box_core.tool_call_markup import TOOL_CALL_MARKUP_RE, is_tool_call_markup  # noqa: F401 — реэкспорт

#: Pattern matching a tool call the model WROTE as text instead of
#: emitting through function-calling. The shape is ``"<name>{...}"`` or
#: ``"<name>(...)"`` — a single angle-bracketed line. Anything else
#: (paragraphs, prose around the call, ...) is NOT a pseudo-call; the
#: LLM probably meant it as natural language.
_PSEUDO_TOOL_CALL_RE: re.Pattern[str] = re.compile(r"^<[^<>]{1,160}>$")


#: Маркеры конца хода v1: мастер-промпт учит писать ``done`` ПОСЛЕ последнего ``speak_text``. Ответ целиком из
#: маркера — не речь (#1217: deepseek первым ответом отдаёт голое «done» — ход молчит и ничего не делает). Одна
#: таблица на ядро и ноду; в режиме команд маркера нет (``command_bridge.V1_TRANSPORT_NOTE``: «done не пиши»), и
#: она уходит вместе с контрактом ``speak_text`` v1 (ADR-0150 PR-7 ч.7d / PR-15).
DONE_MARKERS: frozenset[str] = frozenset({"done", "task complete", "task_complete", "готово", "всё", "выполнено"})


def is_done_marker(text: str | None) -> bool:
    """Текст целиком — маркер конца хода (без учёта регистра и пробелов по краям)."""
    return (text or "").strip().lower() in DONE_MARKERS


def is_silent_text(text: str | None) -> bool:
    """Человеку нечего сказать: пусто или голый маркер конца хода."""
    return not (text or "").strip() or is_done_marker(text)


def is_pseudo_tool_call(text: str) -> bool:
    """Return ``True`` when ``text`` is a tool call the model WROTE instead of made."""
    return bool(_PSEUDO_TOOL_CALL_RE.match((text or "").strip()))


__all__ = [
    "_PSEUDO_TOOL_CALL_RE",
    "DONE_MARKERS",
    "TOOL_CALL_MARKUP_RE",
    "is_done_marker",
    "is_pseudo_tool_call",
    "is_silent_text",
    "is_tool_call_markup",
]
