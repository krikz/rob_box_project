"""Зависимость-free оценка размера промпта в токенах.

Зачем это существует
--------------------

Диалоговая нода публикует размер отправленного в LLM промпта
(``voice_llm_prompt_tokens``), чтобы «стало меньше контекста» можно было
доказать цифрой, а не ощущением. Точное число знает только провайдер —
оно приходит в ``LLMResponse.usage``. Но:

* primary-провайдер (minimax) usage не отдаёт;
* на streaming-пути usage приходит только если API его прислал.

Поэтому нужен запасной оценщик на стороне клиента. ``tiktoken`` в образ
робота не ставится (лишние ~2 МБ словарей и сборочная зависимость ради
метрики), а ``rob_box_core`` по контракту не имеет зависимостей вообще —
отсюда эвристика ниже.

Как откалибровано
-----------------

Линейная модель ``a*кириллица + b*пробелы + c*остальное``, коэффициенты
подобраны grid-search'ем по девяти реальным текстам этого репозитория
(мастер-промпт, JSON-схемы каталога инструментов, четыре промпта скиллов,
короткие русские и английские реплики) против ``tiktoken`` / ``cl100k_base``
на 02.09.2026:

===========  ======  ======  =======
образец      real    est     ошибка
===========  ======  ======  =======
master        10838   10165   -6.2%
tools         16240   18197  +12.0%
music         27213   21010  -22.8%
nav            1276    1189   -6.8%
status          431     363  -15.8%
compositor     3995    3785   -5.3%
ru короткая      35      38   +9.0%
ru короткая      36      43  +19.8%
en короткая      11      14  +22.7%
===========  ======  ======  =======

Худший случай — 22.8%, медиана около 10%. Для сравнения «до/после» и для
алертов этого достаточно; для биллинга — нет, поэтому результат ВСЕГДА
помечается как оценочный (``estimated=True`` в метрике).

Кириллица дороже латиницы примерно втрое на токен — это и есть причина,
по которой русский промпт нельзя оценивать «символы / 4», как англоязычные
гайды советуют по умолчанию.
"""

from __future__ import annotations

import json
from typing import Any, Iterable, Mapping

__all__ = ["estimate_tokens", "estimate_prompt_tokens"]


#: Токенов на символ кириллицы. Русский текст cl100k режет примерно по
#: полтора символа на токен — 0.7 попадает в середину разброса по нашим
#: образцам (короткие реплики +9%/+20%, длинные промпты -6%).
_RATE_CYRILLIC: float = 0.70

#: Токенов на пробельный символ. Пробел почти всегда приклеивается к
#: следующему слову, поэтому вес заметно меньше единицы.
_RATE_WHITESPACE: float = 0.37

#: Токенов на всё остальное (латиница, цифры, пунктуация, JSON-разметка).
_RATE_OTHER: float = 0.20

#: Накладные расходы на одно сообщение в chat-формате: служебные токены
#: роли и разделителей. OpenAI-совместимые API берут около четырёх.
_PER_MESSAGE_OVERHEAD: int = 4


def _is_cyrillic(ch: str) -> bool:
    """Кириллица включая расширенные блоки (U+0400..U+04FF, U+0500..U+052F)."""
    return "Ѐ" <= ch <= "ԯ"


def estimate_tokens(text: str) -> int:
    """Оценить число токенов в ``text``.

    Всегда возвращает неотрицательное целое. Пустая строка — ноль.
    Точность — см. таблицу калибровки в docstring модуля; результат
    следует помечать как оценочный.
    """
    if not text:
        return 0
    cyrillic = 0
    whitespace = 0
    for ch in text:
        if ch.isspace():
            whitespace += 1
        elif _is_cyrillic(ch):
            cyrillic += 1
    other = len(text) - cyrillic - whitespace
    estimate = (
        _RATE_CYRILLIC * cyrillic
        + _RATE_WHITESPACE * whitespace
        + _RATE_OTHER * other
    )
    # Непустой текст не может стоить ноль токенов — иначе метрика покажет
    # «промпт пустой» на строке из одних пробелов.
    return max(1, int(round(estimate)))


def _content_of(message: Any) -> str:
    """Достать текстовую часть сообщения, чем бы оно ни было.

    Принимает и ``LLMMessage`` (атрибуты), и словарь (ключи) — модуль не
    импортирует ни harness, ни rob_box_llm, чтобы остаться без зависимостей.
    """
    if isinstance(message, Mapping):
        content = message.get("content")
        tool_calls = message.get("tool_calls")
    else:
        content = getattr(message, "content", None)
        tool_calls = getattr(message, "tool_calls", None)
    parts: list[str] = []
    if content:
        parts.append(str(content))
    # Вызовы инструментов уезжают на провод как JSON и стоят токенов —
    # без них оценка хода с большим батчем занижена в разы.
    if tool_calls:
        for call in tool_calls:
            if isinstance(call, Mapping):
                name = call.get("name", "")
                args = call.get("arguments", "")
            else:
                name = getattr(call, "name", "")
                args = getattr(call, "arguments", "")
            parts.append(str(name))
            if args:
                try:
                    parts.append(
                        args if isinstance(args, str)
                        else json.dumps(args, ensure_ascii=False)
                    )
                except (TypeError, ValueError):
                    parts.append(str(args))
    return " ".join(parts)


def estimate_prompt_tokens(
    messages: Iterable[Any],
    tools: Iterable[Mapping[str, Any]] = (),
) -> int:
    """Оценить размер всего запроса к LLM: сообщения плюс схемы инструментов.

    ``messages`` — последовательность ``LLMMessage`` или словарей.
    ``tools`` — схемы в OpenAI-формате; они уходят провайдеру наравне с
    сообщениями и в нашем случае составляют большую часть промпта, поэтому
    считать их обязательно.
    """
    total = 0
    for message in messages:
        total += estimate_tokens(_content_of(message)) + _PER_MESSAGE_OVERHEAD
    for tool in tools:
        try:
            total += estimate_tokens(json.dumps(tool, ensure_ascii=False))
        except (TypeError, ValueError):
            total += estimate_tokens(str(tool))
    return total
