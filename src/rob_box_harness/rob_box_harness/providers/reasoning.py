"""Пер-ходовой thinking у MiniMax (issue #3220, инкремент #3136).

Голосовой путь держит thinking выключенным (``DEFAULT_THINKING_POLICY =
{"type": "disabled"}``): adaptive thinking MiniMax-M3 добавлял +10-20 с на
ход (коммит 6901a14e, 06.08). Договорённость с Шифу (#3136 п.3): думать
там, где есть время, — DJ-переход и ход сочинения/подбора музыки. Какой ход
думает, решает голосовой слой (``rob_box_voice.core.turn_reasoning``) и
выставляет :data:`TURN_REASONING` на время хода; провайдер только читает
флаг. Модуль чистый: без ROS и без I/O.

Формат API. Включённый thinking — это ОТСУТСТВИЕ поля ``thinking`` в
запросе (дефолт модели). Только такой режим видели вживую: с b5879b79
(05.08, ``thinking=None``) до 6901a14e (06.08) MiniMax-M3 отвечал блоками
``<think>…</think>`` в ``content``. ``{"type": "enabled"}`` из
``docs/guides/MINIMAX.md`` с API не сверялся (ADR-0142 §13), его не шлём.

Думает только ПЕРВЫЙ вызов хода — тот, что читает реплику (последнее
не-системное сообщение — ``user``). Вызовы после результатов тулов
(«готово, играю») идут без thinking: иначе каждый шаг агент-цикла
добавлял бы свои 10-20 с, и бюджет перехода (``DJ_REASONING_BUDGET_S``
в ``rob_box_voice.core.dj_mode``) не держался бы.
"""

from __future__ import annotations

import contextvars
import dataclasses
import logging
import re
from typing import Iterable

from rob_box_llm.provider import LLMMessage, LLMResponse, LLMSettings

__all__ = [
    "CORRECTION_PREFIX",
    "REASONING_COMPLETE_TIMEOUT_S",
    "REASONING_MAX_TOKENS_HEADROOM",
    "TURN_REASONING",
    "is_reasoning_call",
    "reasoning_settings",
    "strip_reasoning",
    "without_reasoning",
]

_log = logging.getLogger(__name__)

_THINK_OPEN = "<think>"
_THINK_CLOSE = "</think>"
_THINK_BLOCK_RE = re.compile(r"<think>.*?</think>\s*", flags=re.DOTALL)

#: ``True`` на время хода, которому разрешено думать. Ставит голосовой
#: слой, читает провайдер. Вне хода — ``False`` (thinking выключен).
TURN_REASONING: contextvars.ContextVar[bool] = contextvars.ContextVar(
    "rob_box_turn_reasoning", default=False
)

#: Запас ``max_tokens`` на рассуждение, токенов. ``<think>`` идёт в
#: ``content`` и расходует тот же лимит, что ответ; без запаса ход с
#: кодом трека (до ~700 токенов аргументов, #1883) срезался бы на
#: ``finish_reason='length'``. Число не замерено — поправить по живым
#: ``usage`` ходов с thinking.
REASONING_MAX_TOKENS_HEADROOM: int = 4096

#: Дедлайн ``complete()`` (не стрима) для хода с thinking, с. У
#: ``complete()`` first-chunk-гуард #2718 ждёт ВЕСЬ ответ, и 10 с рубили
#: бы каждый ход с рассуждением. У стрима дедлайн прежний: первым чанком
#: приходит ``<think>`` (или пустой чанк рассуждения, см.
#: ``rob_box_llm.providers.deepseek._is_reasoning_delta``), так что
#: мёртвый провайдер по-прежнему уходит в фолбек за 10 с.
REASONING_COMPLETE_TIMEOUT_S: float = 60.0


#: Префикс коррекции агент-цикла (``tool_loop.retry``: пустой ответ, вызов
#: тула текстом, срезанные аргументы). Это ``user``-сообщение, но реплики
#: оно не читает — вызов с ним это ретрай.
CORRECTION_PREFIX = "[SYSTEM CORRECTION]"


def is_reasoning_call(messages: Iterable[LLMMessage]) -> bool:
    """Думать ли этому вызову: ход с thinking и вызов читает реплику.

    Ретрай-коррекция (issue #3265) не думает: живой ход 01.10 11:10 —
    76 с thinking дали пустой ответ, ретрай с коррекцией думал снова
    (+39 с, первый тул на 115-й секунде).
    """
    if not TURN_REASONING.get():
        return False
    last = None
    for message in messages:
        if message.role != "system":
            last = message
    if last is None or last.role != "user":
        return False
    return not str(last.content or "").lstrip().startswith(CORRECTION_PREFIX)


def reasoning_settings(settings: LLMSettings) -> LLMSettings:
    """Настройки вызова с thinking: без поля ``thinking`` и с запасом токенов.

    ``extra["thinking"] = None`` — «поле не слать»: так его понимает
    ``rob_box_llm.providers.minimax.MiniMaxProvider._apply_thinking_policy``.
    """
    extra = dict(settings.extra)
    extra["thinking"] = None
    max_tokens = (settings.max_tokens or 0) + REASONING_MAX_TOKENS_HEADROOM
    return dataclasses.replace(settings, extra=extra, max_tokens=max_tokens)


def strip_reasoning(text: str) -> str:
    """Вырезать рассуждение ``<think>…</think>`` из ответа модели.

    Три формы: целый блок; блок, срезанный ``max_tokens`` (``<think>`` без
    ``</think>`` — всё после открывающего тега — рассуждение); ответ без
    открывающего тега (всё до последнего ``</think>`` — рассуждение).
    Текст без тегов возвращается как есть, байт в байт.
    """
    if not text or (_THINK_OPEN not in text and _THINK_CLOSE not in text):
        return text
    text = _THINK_BLOCK_RE.sub("", text)
    if _THINK_CLOSE in text:
        text = text.rsplit(_THINK_CLOSE, 1)[1]
    if _THINK_OPEN in text:
        text = text.split(_THINK_OPEN, 1)[0]
    return text.strip()


def without_reasoning(response: LLMResponse) -> LLMResponse:
    """Ответ без рассуждения в ``content`` (issue #3220).

    Рассуждение — не речь и не разметка вызова: ни восстановление вызова
    тула из текста (#2760), ни проверки «пустой ответ / done», ни история
    не должны его видеть. Тулы, ``finish_reason`` и ``usage`` не трогаем.
    Строка в лог — след на живом прогоне, что модель действительно думала.
    """
    content = response.content
    if not isinstance(content, str):
        return response
    cleaned = strip_reasoning(content)
    if cleaned == content:
        return response
    _log.info(
        "🧠 [#3220] рассуждение вырезано из ответа: %d → %d симв.",
        len(content),
        len(cleaned),
    )
    return dataclasses.replace(response, content=cleaned)
