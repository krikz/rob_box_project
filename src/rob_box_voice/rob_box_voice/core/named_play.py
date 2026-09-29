"""named_play.py — заказ мелодии по имени кодом, без LLM (issue #3176).

Живой прогон 29.09.2026 05:54: «Робот, поставь к Элизе» → модель сказала
«Ставлю «К Элизе», погнали» с ``tools=[]``. Мелодия не зазвучала, заказ
потерян. Заставить MiniMax вызвать тул нельзя (ADR-0143), поэтому заказ по
имени исполняет роутер медиакоманд (как громкость и стоп, #3134):

1. ``lookup_melody(name=<название>)`` — только поиск, без звука. База
   (``RtttlLibrary.get``) всегда отдаёт лучшего по тексту кандидата
   (#2896), поэтому решение «это та самая мелодия» принимается по
   прозрачной сверке ``data["match"]`` (#2964): играем, только если
   совпали ВСЕ значимые слова запроса (``unmatched`` пуст) и ни одно
   слово не осталось непонятым для поиска (``ignored`` пуст — кириллица,
   которой нет в алиасах, до поиска просто не доходит: «гимн германии»
   без этой проверки сыграл бы гимн СССР). Иначе — промах.
2. ``compose_music(name=<ключ записи>)`` — ключ записи из шага 1, чтобы
   сыграла та же запись. Сначала без тембров: сохранённый пресет мелодии
   и подстройка играющего трека (ADR-0132 PR-7, #2950) подставляются
   тулом сами. Если тул ответил «найдена, но не задана аранжировка» —
   второй вызов с тембрами по умолчанию (:data:`DEFAULT_ARRANGEMENT`).

Промах — реплика уходит в LLM, как до #3176, и роутер ничего не говорит
(иначе был бы двойной ответ). Поток чистый: без ROS и без I/O, тулы
вызывает переданная корутина ``execute(name, args) -> (ok, content)``.
"""

from __future__ import annotations

import ast
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Awaitable, Callable, Dict, Mapping, Optional, Tuple

#: Тембры для ``compose_music(name=…)``, если у мелодии нет пресета.
#: Тул требует ``lead_synth`` + ``bass_synth`` + ``pad_synth`` при
#: ``name=`` (``ComposeMusicTool._missing_arrangement_fields``). Лид —
#: фортепиано: сухой соло-инструмент из ``MELODIC_LEAD_SYNTHS``, годится
#: для классики, игровых и киношных тем; бас и пэд — нейтральные из
#: описаний параметров тула.
DEFAULT_ARRANGEMENT: Mapping[str, str] = {
    "lead_synth": "pianovel",
    "bass_synth": "bass",
    "pad_synth": "warmpad",
}

#: Маркер ответа ``compose_music``: мелодия найдена, тембров не хватает.
ARRANGEMENT_MISSING_MARKER = "не задана аранжировка"

LOOKUP_TOOL = "lookup_melody"
COMPOSE_TOOL = "compose_music"

ToolExec = Callable[[str, Dict[str, Any]], Awaitable[Tuple[bool, str]]]


class NamedPlayStatus(str, Enum):
    """Исход заказа."""

    #: Мелодия играет — роутер говорит :func:`play_ok_text`.
    PLAYED = "played"
    #: Нет в базе или совпало не целиком — реплика в LLM, роутер молчит.
    MISS = "miss"
    #: Нашлась, но ``compose_music`` не прошёл — честная фраза.
    FAILED = "failed"


@dataclass(frozen=True)
class MelodyHit:
    """Запись базы, которую играем: ключ и название для фразы."""

    key: str
    title: str


@dataclass(frozen=True)
class NamedPlayOutcome:
    """Итог потока: исход, найденная запись, успешно вызванные тулы."""

    status: NamedPlayStatus
    hit: Optional[MelodyHit] = None
    tools_done: Tuple[str, ...] = field(default_factory=tuple)
    reason: str = ""


def play_ok_text(title: str) -> str:
    """Короткая фраза после успешного заказа."""
    return f"Ставлю «{title}»."


def play_fail_text(title: str) -> str:
    """Мелодия нашлась, а запустить её не вышло — говорим как есть."""
    return f"Нашёл «{title}», но включить не получилось."


def tool_data(content: str) -> Optional[Dict[str, Any]]:
    """``data`` MCP-тула из текста ``ToolResult.content``.

    Адаптер (``rob_box_harness.executors.core_adapter._result_content``)
    отдаёт успешный результат как ``message`` + ``\\n`` + ``repr(data)``
    (или один ``repr(data)``). ``repr`` словаря однострочный — это
    последняя строка. Не разобралось — ``None`` (вызывающий считает это
    промахом и отдаёт реплику LLM).
    """
    last = (content or "").rsplit("\n", 1)[-1].strip()
    if not last.startswith("{"):
        return None
    try:
        data = ast.literal_eval(last)
    except (ValueError, SyntaxError, MemoryError, RecursionError):
        return None
    return data if isinstance(data, dict) else None


def melody_hit(data: Optional[Mapping[str, Any]]) -> Tuple[Optional[MelodyHit], str]:
    """Полное совпадение из ответа ``lookup_melody`` или причина промаха."""
    if not data:
        return None, "lookup: нет data"
    match = data.get("match")
    if not isinstance(match, Mapping):
        return None, "lookup: нет match"
    if match.get("unmatched") != []:
        return None, f"lookup: не совпали слова {match.get('unmatched')!r}"
    ignored = match.get("ignored")
    if ignored != []:
        # ``None`` — mcp_tools старше #3176 (поля нет): не знаем, всё ли
        # слово запроса увидел поиск, — честнее отдать заказ LLM.
        return None, f"lookup: слова вне поиска {ignored!r}"
    key = str(data.get("name") or "")
    if not key:
        return None, "lookup: у записи нет ключа"
    title = str(data.get("display_title") or data.get("title") or key)
    return MelodyHit(key=key, title=title), ""


async def _compose(execute: ToolExec, hit: MelodyHit) -> bool:
    ok, content = await execute(COMPOSE_TOOL, {"name": hit.key})
    if ok or ARRANGEMENT_MISSING_MARKER not in (content or ""):
        return ok
    args: Dict[str, Any] = {"name": hit.key, **DEFAULT_ARRANGEMENT}
    ok, _content = await execute(COMPOSE_TOOL, args)
    return ok


async def run_named_play(execute: ToolExec, name: str) -> NamedPlayOutcome:
    """``lookup_melody`` → (полное совпадение) → ``compose_music``."""
    ok, content = await execute(LOOKUP_TOOL, {"name": name})
    if not ok:
        return NamedPlayOutcome(NamedPlayStatus.MISS, reason="lookup: не найдена")
    hit, reason = melody_hit(tool_data(content))
    if hit is None:
        return NamedPlayOutcome(
            NamedPlayStatus.MISS, tools_done=(LOOKUP_TOOL,), reason=reason
        )
    if await _compose(execute, hit):
        return NamedPlayOutcome(
            NamedPlayStatus.PLAYED, hit=hit, tools_done=(LOOKUP_TOOL, COMPOSE_TOOL)
        )
    return NamedPlayOutcome(
        NamedPlayStatus.FAILED, hit=hit, tools_done=(LOOKUP_TOOL,),
        reason="compose_music не прошёл",
    )


__all__ = [
    "ARRANGEMENT_MISSING_MARKER",
    "COMPOSE_TOOL",
    "DEFAULT_ARRANGEMENT",
    "LOOKUP_TOOL",
    "MelodyHit",
    "NamedPlayOutcome",
    "NamedPlayStatus",
    "melody_hit",
    "play_fail_text",
    "play_ok_text",
    "run_named_play",
    "tool_data",
]
