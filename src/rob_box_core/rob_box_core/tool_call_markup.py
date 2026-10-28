"""Разметка вызова тула в тексте ответа - единственная копия (ADR-0150 PR-15, #3711, часть 15-core).

Модель иногда ПЕЧАТАЕТ вызов тула вместо того, чтобы его сделать (#2760). Эту разметку ловят два слоя:
цикл тулов ``rob_box_harness`` (восстановить намерение, ``tool_loop.markup_recovery``) и ``tts_node`` (последний рубеж:
не дать тегам прозвучать). Раньше регекс жил в двух копиях, потому что ``tts_node`` не должен тянуть ``rob_box_harness``
(минимальная сборка без него); слой ниже обоих - ``rob_box_core`` - снимает это ограничение: регекс и предикат здесь,
harness (``text_classify``) и ``tts_node`` импортируют их отсюда.

Чистый Python, без ROS.
"""

from __future__ import annotations

import re

#: Issue #2760 - вызов тула, напечатанный текстом в полном синтаксисе протокола tool-calls и на НЕСКОЛЬКО строк.
#: Живой лог Vision Pi, прогон 35704637846 (акт 2, шаги n204/n206 - оба про сохранение факта)::
#:
#:     spoken='<function_calls> <invoke name="memory_save">
#:             <parameter name="fact">Болеет за Спартак…</parameter> …'
#:     tools=[] finish_reason='stop'
#:
#: Последствий два, и второе хуже первого: ``tts_node`` прочитал теги вслух, а текст осел в окне истории как реплика
#: ассистента - дальше модель копировала его ходом позже (n204 -> n206).
#:
#: Регекс обязан переживать ``strip_markdown`` на стороне voice: тот снимает ``_..._`` парами через весь текст, и в
#: логе видно уже ``<functioncalls>`` / ``registerspeaker`` / ``memorysave``. Отсюда ``_?`` внутри имён тегов.
#:
#: ``search``, а не ``match``: смешанный ответ («Привет! <invoke …>») - такой же брак, как и чистая разметка. Ложных
#: срабатываний на речи не ожидается - эти теги человек не произносит.
TOOL_CALL_MARKUP_RE: re.Pattern[str] = re.compile(
    # Родной диалект MiniMax (официальный tool_calling_guide.md):
    # <minimax:tool_call><invoke name="..."><parameter name="...">.
    r"</?minimax:tool_call\s*>"
    # Диалект, который робот получил живьём 22.09 (Anthropic-style).
    r"|</?function_?calls\s*>"
    r"|</?tool_?calls?\s*>"
    r"|<\s*/?\s*(?:minimax:|antml:)?invoke(?:\s+name\s*=|\s*>)"
    r"|<\s*/?\s*(?:minimax:|antml:)?parameter(?:\s+name\s*=|\s*>)"
    r"|<\s*antml:",
    re.IGNORECASE,
)


def is_tool_call_markup(text: str | None) -> bool:
    """Issue #2760 - ``True``, если в тексте лежит разметка вызова тула.

    См. :data:`TOOL_CALL_MARKUP_RE`. Пустая строка / ``None`` -> ``False``.
    """
    if not text:
        return False
    return bool(TOOL_CALL_MARKUP_RE.search(text))


__all__ = ["TOOL_CALL_MARKUP_RE", "is_tool_call_markup"]
