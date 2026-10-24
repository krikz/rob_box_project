"""Библиотека генераций кодом (ADR-0150 §9.1 ревизия 5, PR-6 #3696): «включи что-нибудь из библиотеки» без LLM.

Живой прогон 21.10 (горячая заливка PR-6 ч.2): «Робот, включи что-нибудь из библиотеки» → промах заказа по имени
(``play_named name='из библиотеки'``) → ход LLM → ``music_library list`` → ``music_library play`` — две итерации LLM
на то, что решает код. Здесь: поиск по словам человека (или список лучших), трек выбирает :func:`pick_track`,
``music_library(play)``; фраза — первая строка результата тула (:func:`.music_turn.library_result_phrase`), её
строит тул из итога запуска (``started``), не LLM (ADR-0148).

Модуль без ROS: исполнитель тула — ``call_tool`` роутера (``media_plan_run.CallTool``).
"""

from __future__ import annotations

from typing import Any, Awaitable, Callable, List, Mapping, Optional, Sequence, Tuple

from rob_box_music.knowledge import RATING_DISLIKE

from .media_phrases import LIBRARY_FAIL_TEXT
from .media_router import MediaPlan, MediaToolCall
from .music_turn import LIBRARY_TOOL, library_result_phrase
from .named_play import tool_data

#: Сколько треков смотреть: поиск по словам и список лучших без слов.
SEARCH_LIMIT = 5
LIST_LIMIT = 20

CallTool = Callable[[Any], Awaitable[Tuple[bool, str]]]


def pick_track(tracks: Sequence[Mapping[str, Any]], by_relevance: bool) -> Optional[Mapping[str, Any]]:
    """Трек для «включи из библиотеки»: не разонравившийся (``rating`` выше ``RATING_DISLIKE``); по поиску — первый
    по выдаче, без слов — давно не игравший (сначала ни разу не игравшие), чтобы не крутить одно и то же."""
    liked = [t for t in tracks if t.get("id") and int(t.get("rating") or 0) > RATING_DISLIKE]
    if not liked:
        return None
    if by_relevance:
        return liked[0]
    return min(liked, key=lambda t: str(t.get("last_played_at") or ""))


def _first_line(content: str) -> str:
    return (content or "").split("\n", 1)[0].strip()


async def run_library_command(plan: MediaPlan, call_tool: CallTool) -> Tuple[bool, str, List[str]]:
    """``(успех, фраза, исполненные тулы)`` для плана ``plan.library`` ∈ {``list``, ``play``}."""
    if plan.library == "list":
        ok, content = await call_tool(MediaToolCall(LIBRARY_TOOL, {"action": "list", "limit": LIST_LIMIT}))
        return ok, library_result_phrase(not ok, content), [LIBRARY_TOOL] if ok else []
    query = plan.command.name
    args = ({"action": "search", "query": query, "limit": SEARCH_LIMIT} if query
            else {"action": "list", "limit": LIST_LIMIT, "sort_by": "rating"})
    ok, content = await call_tool(MediaToolCall(LIBRARY_TOOL, args))
    if not ok:
        return False, LIBRARY_FAIL_TEXT, []
    track = pick_track((tool_data(content) or {}).get("tracks") or [], by_relevance=bool(query))
    if track is None:  # «ничего не найдено» / «в библиотеке пусто» — фраза тула
        return False, _first_line(content) or LIBRARY_FAIL_TEXT, [LIBRARY_TOOL]
    ok, content = await call_tool(MediaToolCall(LIBRARY_TOOL, {"action": "play", "track_id": str(track["id"])}))
    return ok, library_result_phrase(not ok, content), [LIBRARY_TOOL]


__all__ = ["LIST_LIMIT", "SEARCH_LIMIT", "pick_track", "run_library_command"]
