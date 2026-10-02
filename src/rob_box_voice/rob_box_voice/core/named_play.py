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
2. ``request_music(intent="melody", text=<название>)`` (ADR-0149 PR-11) — играет
   песню движка v2 и отвечает ``ok`` только по ``started`` (поиск в туле — то же
   правило :func:`melody_hit`). Старый шаг ``compose_music(name=…)`` удалён в PR-13a.

Промах — реплика уходит в LLM, как до #3176, и роутер ничего не говорит
(иначе был бы двойной ответ). Поток чистый: без ROS и без I/O, тулы
вызывает переданная корутина ``execute(name, args) -> (ok, content)``.
"""

from __future__ import annotations

import ast
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Awaitable, Callable, Dict, Mapping, Optional, Tuple

LOOKUP_TOOL = "lookup_melody"
REQUEST_TOOL = "request_music"

ToolExec = Callable[[str, Dict[str, Any]], Awaitable[Tuple[bool, str]]]


class NamedPlayStatus(str, Enum):
    """Исход заказа."""

    #: Мелодия играет — роутер говорит :func:`play_ok_text`.
    PLAYED = "played"
    #: Нет в базе или совпало не целиком — реплика в LLM, роутер молчит.
    MISS = "miss"
    #: Нашлась, но ``request_music`` не прошёл — честная фраза.
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


def user_phrase_title(play_name: str) -> str:
    """Слова юзера из заказа → имя трека для реплики (issue #3178).

    Живой прогон 29.09.2026: роутер называл трек архивным ``title`` базы
    («Ставлю «Fur Elise».», «Нашёл «Hall Of The Mountain King (Alton
    Towers Theme) 2»…») — не то слово, которым юзер попросил («к элизе»,
    «в пещере горного короля»). Юзер и так знает, что просил — фраза
    только подтверждает заказ его же словами, с заглавной буквы (русское
    предложение, не английский Title Case каждого слова).
    """
    return play_name[:1].upper() + play_name[1:] if play_name else play_name


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


async def run_named_play(execute: ToolExec, name: str) -> NamedPlayOutcome:
    """``lookup_melody`` → (полное совпадение) → ``request_music(intent="melody")``."""
    ok, content = await execute(LOOKUP_TOOL, {"name": name})
    if not ok:
        return NamedPlayOutcome(NamedPlayStatus.MISS, reason="lookup: не найдена")
    hit, reason = melody_hit(tool_data(content))
    if hit is None:
        return NamedPlayOutcome(
            NamedPlayStatus.MISS, tools_done=(LOOKUP_TOOL,), reason=reason
        )
    played, _content = await execute(REQUEST_TOOL, {"intent": "melody", "text": name})
    if played:
        return NamedPlayOutcome(
            NamedPlayStatus.PLAYED, hit=hit, tools_done=(LOOKUP_TOOL, REQUEST_TOOL)
        )
    return NamedPlayOutcome(
        NamedPlayStatus.FAILED, hit=hit, tools_done=(LOOKUP_TOOL,),
        reason=f"{REQUEST_TOOL} не прошёл",
    )


__all__ = [
    "LOOKUP_TOOL",
    "MelodyHit",
    "NamedPlayOutcome",
    "NamedPlayStatus",
    "REQUEST_TOOL",
    "melody_hit",
    "play_fail_text",
    "play_ok_text",
    "run_named_play",
    "tool_data",
    "user_phrase_title",
]
