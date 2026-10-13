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

from rob_box_core.tool_catalog import tool_names

#: Pattern matching a tool call the model WROTE as text instead of
#: emitting through function-calling. The shape is ``"<name>{...}"`` or
#: ``"<name>(...)"`` — a single angle-bracketed line. Anything else
#: (paragraphs, prose around the call, ...) is NOT a pseudo-call; the
#: LLM probably meant it as natural language.
_PSEUDO_TOOL_CALL_RE: re.Pattern[str] = re.compile(r"^<[^<>]{1,160}>$")


def is_pseudo_tool_call(text: str) -> bool:
    """Return ``True`` when ``text`` is a tool call the model WROTE instead of made."""
    return bool(_PSEUDO_TOOL_CALL_RE.match((text or "").strip()))


#: Issue #2760 — тот же класс ошибки, что ``_PSEUDO_TOOL_CALL_RE``, но в
#: полном синтаксисе протокола tool-calls и на НЕСКОЛЬКО строк, поэтому
#: однострочная regex выше его не ловит. Живой лог Vision Pi, прогон
#: 35704637846 (акт 2, шаги n204/n206 — оба про сохранение факта)::
#:
#:     spoken='<function_calls> <invoke name="memory_save">
#:             <parameter name="fact">Болеет за Спартак…</parameter> …'
#:     tools=[] finish_reason='stop'
#:
#: Последствий два, и второе хуже первого: tts_node прочитал теги вслух,
#: а текст осел в окне истории как реплика ассистента — дальше модель
#: копировала его ходом позже (n204 → n206).
#:
#: Регекс обязан переживать ``strip_markdown`` на стороне voice: тот
#: снимает ``_..._`` парами через весь текст, и в логе видно уже
#: ``<functioncalls>`` / ``registerspeaker`` / ``memorysave``. Отсюда
#: ``_?`` внутри имён тегов.
#:
#: ``search``, а не ``match``: смешанный ответ («Привет! <invoke …>») —
#: такой же брак, как и чистая разметка. Ложных срабатываний на речи не
#: ожидается — эти теги человек не произносит.
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


def is_tool_call_markup(text: str) -> bool:
    """Issue #2760 — ``True``, если в тексте лежит разметка вызова тула.

    См. :data:`TOOL_CALL_MARKUP_RE`. Пустая строка / ``None`` → ``False``.
    """
    if not text:
        return False
    return bool(TOOL_CALL_MARKUP_RE.search(text))


#: Issue #3598 — имена тегов, которые код сам положил в контекст хода
#: (``<system_context>``, ``<user_profile>``, ``<music_state>`` …).
_CONTEXT_TAG_RE: re.Pattern[str] = re.compile(r"<([A-Za-z][\w-]*)")


def _catalog_identifiers() -> tuple[str, ...]:
    """Имена тулов из единого каталога (``rob_box_core.tool_catalog``) + ``load_skill``.

    Берутся только идентификаторы с ``_``: однословное имя (``say``) — это
    обычное слово речи, а не служебный идентификатор.
    """
    return tuple(n for n in (*tool_names(), "load_skill") if "_" in n)


def quoted_turn_context(text: str, context: str | None) -> str | None:
    """Issue #3598 — финальный текст хода цитирует служебное, что код дал модели.

    TEMP(ADR-0148) — #3600: проверка читает текст ответа. Словарь не
    рукописный: теги — из ``<system_context>`` ЭТОГО хода (его собрал
    код), имена тулов — из единого каталога. Убрать вместе с
    авто-озвучкой финального текста (речь — только ``speak_text``).

    Живой лог 13.10 08:38 UTC: на «[TG] продолжай играть вагнера» модель
    ответила разбором от третьего лица — «… но ``<music_state>
    playing="no" …`` … Скажу коротко через ``speak_text`` …» — и авто-TTS
    прочитал его вслух шестью чанками (плюс дубль в Telegram). Человеку
    такие идентификаторы не говорят никогда, значит это не реплика.

    Проверяется СЫРОЙ текст модели: ``strip_markdown`` на стороне voice
    снимает ``_…_`` парами, и ``music_state`` превращается в
    ``musicstate`` — так #1882 и промахнулся в этом логе.

    Returns:
        Первый найденный идентификатор (``"<music_state>"`` или
        ``"speak_text"``) или ``None``.
    """
    if not text:
        return None
    low = text.lower()
    tags = sorted({t.lower() for t in _CONTEXT_TAG_RE.findall(context or "")})
    for tag in tags:
        if f"<{tag}" in low:
            return f"<{tag}>"
    for name in _catalog_identifiers():
        if re.search(rf"(?<!\w){re.escape(name)}(?!\w)", low):
            return name
    return None


__all__ = [
    "_PSEUDO_TOOL_CALL_RE",
    "TOOL_CALL_MARKUP_RE",
    "is_pseudo_tool_call",
    "is_tool_call_markup",
    "quoted_turn_context",
]
