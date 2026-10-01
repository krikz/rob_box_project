"""Issue #3269 — реплика, написанная в ``content`` рядом с ``set_voice``, — это речь.

Живой случай
------------

e2e ``mv03`` («расскажи сказку про Красную Шапочку разными голосами»),
прогоны 36775544782 / 36777967495, ``docker logs voice-assistant`` Vision
Pi. MiniMax-M3 восемь итераций подряд отвечала одной формой::

    content='Жила-была девочка, и звали её Красная Шапочка.'
    tool_calls=(set_voice(voice='zahar'),)
    content='Здравствуй, Красная Шапочка, куда ты идёшь?'
    tool_calls=(set_voice(voice='alena'),)
    ...

``speak_text`` не вызван ни разу. Цикл исполнял только ``set_voice``,
``content`` уходил в историю и нигде не звучал; после
``_MAX_TOOL_ITERATIONS=8`` прозвучала одна последняя реплика. Пользователь
вместо сказки услышал финал.

Почему озвучивать, а не только переучивать модель
-------------------------------------------------

Форма ответа однозначна: в Chat Completions ``content`` ассистента
предшествует его ``tool_calls`` — модель «сказала реплику и переключила
голос для следующей». Каждая реплика в логе написана голосом,
поставленным на ПРЕДЫДУЩЕЙ итерации (волк — после ``zahar``, внучка —
после ``alena``). RULE #VOICE-MULTI в промпте уже требовал ``speak_text``
— модель его не послушала; ещё одно правило в промпте вероятностно, а
текст, который она написала для слушателя, известен точно. Это тот же
класс, что #2760 (``markup_recovery``): намерение модели известно — его
исполняют, а не тратят ход на ретрай.

Узость — нарочно
----------------

Озвучивается ТОЛЬКО ``content`` пачки, где ВСЕ вызовы — ``set_voice``:

* в DJ- и музыкальных ходах ``content`` рядом с ``compose_music`` и
  прочими — мета-болтовня («Переход номер два отыгран…»), её
  dialogue_node специально не озвучивает (live 02.09). Такие пачки
  сюда не попадают;
* пачка с ``speak_text`` уже говорит сама — ``content`` рядом с ним
  обычно дублирует реплику (повтор #988);
* служебные ответы (``done``, псевдовызовы, разметка протокола) речью
  не считаются.

Реплика становится настоящим вызовом ``speak_text`` первым в пачке: он
идёт через тот же executor (VOICE-канал планировщика, DJ-лимит #2878,
acceptance-гейт, гард #1708) и через те же счётчики, что вызов модели. В
историю ложится вызов, а не голый ``content`` — модель на следующей
итерации видит правильную форму и может продолжить её сама.

Голос реплики
-------------

``speak_text`` встаёт в VOICE-очередь планировщика (fire-and-forget), а
``set_voice`` исполняется сразу. Без явного ``voice=`` реплика взяла бы
голос, который окажется установленным к моменту синтеза, — то есть уже
следующий. Поэтому реплика несёт ``voice=`` последнего ``set_voice``
этого хода. Для ПЕРВОЙ реплики хода такого нет (голос до хода харнессу
неизвестен) — она идёт без ``voice=``, и гонка с ``set_voice`` той же
пачки остаётся; это известное ограничение, а не скрытое.
"""

from __future__ import annotations

import logging
import re
import uuid
from dataclasses import dataclass, replace
from typing import Any, Iterable, Optional

from rob_box_harness.core.tool_loop.text_classify import (
    is_pseudo_tool_call,
    is_tool_call_markup,
)
from rob_box_llm.provider import LLMResponse, ToolCall

_LOG = logging.getLogger(__name__)

SET_VOICE_TOOL = "set_voice"
SPEAK_TOOL = "speak_text"

#: Маркер конца цикла, который модель пишет ПОСЛЕ последней реплики
#: (master prompt). Тот же набор, что ``agent_core._SILENT_DONE_MARKERS``
#: и ``rob_box_voice.core.speak_helpers.strip_done_marker``: целиком или
#: отдельной последней строкой — не речь. «Вот и всё.» в конце сказки —
#: речь (маркер не на своей строке), его не трогаем.
_DONE_TAIL_RE = re.compile(
    r"(?:^|\n)\s*(?:done|task[ _]complete|готово|всё|выполнено)\s*\.?\s*$",
    re.IGNORECASE,
)


@dataclass
class ContentSpeech:
    """Состояние одного хода: какой голос поставлен и звучал ли ``content``.

    ``voice`` — аргумент последнего ``set_voice`` этого хода (``None`` —
    в ходе ещё не меняли голос). ``spoke`` — хоть одна реплика из
    ``content`` уже озвучена; тогда и финальный ``content`` хода — реплика
    той же сказки (см. :func:`speak_closing_content`).
    """

    voice: Optional[str] = None
    spoke: bool = False


def _speakable(content: str) -> str:
    """Текст для слушателя или ``""``, если это служебный ответ."""
    text = _DONE_TAIL_RE.sub("", (content or "").strip()).strip()
    if not text or is_pseudo_tool_call(text) or is_tool_call_markup(text):
        return ""
    return text


def _speak_call(text: str, voice: Optional[str], call_id: str) -> ToolCall:
    args: dict[str, Any] = {"text": text}
    if voice:
        args["voice"] = voice
    return ToolCall(id=call_id, name=SPEAK_TOOL, arguments=args)


def _last_voice(calls: Iterable[ToolCall], previous: Optional[str]) -> Optional[str]:
    voice = previous
    for call in calls:
        requested = (call.arguments or {}).get("voice") if call.name == SET_VOICE_TOOL else None
        if isinstance(requested, str) and requested.strip():
            voice = requested.strip()
    return voice


def _offered(openai_tools: Iterable[dict]) -> set[str]:
    return {str((spec.get("function") or {}).get("name", "")) for spec in openai_tools}


def speak_content_beside_set_voice(
    response: LLMResponse,
    state: ContentSpeech,
    openai_tools: Iterable[dict],
) -> LLMResponse:
    """Вернуть пачку, где реплика из ``content`` — первый ``speak_text``.

    Ответ без подходящей формы возвращается как есть; ``state.voice``
    обновляется по ``set_voice`` любой пачки — голос следующей реплики.
    """
    calls = tuple(response.tool_calls or ())
    names = {call.name for call in calls}
    line = _speakable(response.content) if names == {SET_VOICE_TOOL} else ""
    voice_for_line = state.voice
    state.voice = _last_voice(calls, state.voice)
    if not line or SPEAK_TOOL not in _offered(openai_tools):
        return response
    state.spoke = True
    _LOG.warning(
        "AgentCore [issue 3269]: content рядом с set_voice → speak_text "
        "(voice=%s) text=%r",
        voice_for_line or "<текущий>",
        line[:80],
    )
    speak = _speak_call(line, voice_for_line, f"{calls[0].id}_content")
    return replace(response, content="", tool_calls=(speak,) + calls)


async def speak_closing_content(
    response: LLMResponse,
    state: ContentSpeech,
    tools: Any,
    spoken_texts: list[str],
) -> int:
    """Озвучить финальный ``content`` хода, где реплики уже шли из ``content``.

    Возвращает число озвученных реплик (0 или 1) — вызывающий прибавляет
    его к счётчикам ``speak_text``. Без этого финал сказки («…и спасли
    Красную Шапочку») терялся бы дважды: цикл его не исполняет, а
    dialogue_node при уже звучавшем ``speak_text`` пропускает финальный
    текст как дубль (#988).
    """
    line = _speakable(response.content) if state.spoke else ""
    if not line:
        return 0
    _LOG.warning(
        "AgentCore [issue 3269]: финальный content хода → speak_text "
        "(voice=%s) text=%r",
        state.voice or "<текущий>",
        line[:80],
    )
    await tools.execute(_speak_call(line, state.voice, f"call_content_final_{uuid.uuid4().hex[:8]}"))
    spoken_texts.append(line)
    return 1
