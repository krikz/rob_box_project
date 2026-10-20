#!/usr/bin/env python3
"""
gen_library.py — один узкий тул ``music_library`` над кэшем сгенерированной музыки (ADR-0158 §2.5, PR-9).

Шесть тулов ``gen_*`` (list / search / save / play / delete / get_track_info) свёрнуты в один:
``music_library(action: enum, …)``. Действия — закрытый список; фразы об итоге строит код
(``media_phrases.library_*_text``, ADR-0148), а не модель.

  list    — что есть в библиотеке (recent / popular / rating)
  search  — подстрочный поиск по title / prompt / lyrics / genre / mood / notes
  info    — метаданные одного трека
  play    — трек кэша на деке движка (``engine.gen.library_play``, ADR-0158 PR-2)
  rate    — «нравится» / «не нравится»: рейтинг в БД; без ``track_id`` — играющий сейчас или последний
  update  — title / tags / mood / genre / notes
  delete  — удалить трек (SQLite + каталог с mp3 и wav)

Внутренние исполнители (кавер, песня, сет, путь) тул не зовут — они вызывают
``engine.gen.library_play.play_library_track`` напрямую.
Модуль держит ROS-импорты ленивыми — тул тестируется без демона.
"""

from __future__ import annotations

import subprocess
import time
from typing import Any, Callable, Dict, List, Optional

from rob_box_music import knowledge as kn

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from ..engine.gen.library import GeneratedMusicLibrary
from ..engine.gen.library_play import play_library_track

#: Действия, которым нужен ``track_id`` (rate берёт играющий / последний, если id не назван).
_NEEDS_TRACK = ("info", "play", "update", "delete")
_VERDICT_RATING = {"like": kn.RATING_LIKE, "dislike": kn.RATING_DISLIKE}


class MusicLibraryTool(MCPTool):
    """Библиотека сгенерированной музыки: список, поиск, воспроизведение, оценка, правка, удаление."""

    def __init__(self, node: Any, library: GeneratedMusicLibrary, clock: Callable[[], float] = time.time,
                 run: Callable[..., Any] = subprocess.run) -> None:
        super().__init__(node)
        self._library = library
        self._clock = clock
        self._run = run  # запуск ffmpeg (декод mp3 → wav сервера); тесты подставляют свой

    @property
    def name(self) -> str:
        return "music_library"

    @property
    def description(self) -> str:
        return (
            "Библиотека сгенерированной музыки робота (/data/music_library). action: list — «что у тебя есть "
            "сгенерированного»; search — найти трек по слову (query); info — подробности одного трека; "
            "play — включить трек по track_id (стоп — stop_music); rate — «этот трек нравится / не нравится» "
            "(verdict like / dislike; трек, который не нравится, больше не включается, но не удаляется); "
            "update — переименовать, теги, настроение, жанр, заметка; delete — удалить трек насовсем. "
            "track_id бери из list / search, не выдумывай. Ответ содержит фразу, которую надо сказать."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="action", type="string", required=True,
                description="Что сделать с библиотекой.",
                enum=["list", "search", "info", "play", "rate", "update", "delete"],
            ),
            MCPToolParameter(
                name="track_id", type="string", required=False, default="",
                description="UUID трека из list / search (info, play, update, delete; у rate — по умолчанию "
                            "играющий сейчас или последний игравший).",
            ),
            MCPToolParameter(
                name="query", type="string", required=False, default="",
                description="Слово поиска для search (1-200 символов).",
            ),
            MCPToolParameter(
                name="verdict", type="string", required=False, default="",
                description="Для rate: like — трек нравится, dislike — не нравится.",
                enum=["", "like", "dislike"],
            ),
            MCPToolParameter(
                name="limit", type="integer", required=False, default=20,
                description="Для list / search: максимум треков (до 100).",
            ),
            MCPToolParameter(
                name="sort_by", type="string", required=False, default="recent",
                description="Для list: recent — новые, popular — самые играемые, rating — лучшие.",
                enum=["recent", "popular", "rating"],
            ),
            MCPToolParameter(
                name="title", type="string", required=False, default="",
                description="Для update: новое название (пусто — не менять).",
            ),
            MCPToolParameter(
                name="tags", type="string", required=False, default="",
                description="Для update: теги через запятую (заменяют прежние).",
            ),
            MCPToolParameter(
                name="mood", type="string", required=False, default="",
                description="Для update: настроение.",
            ),
            MCPToolParameter(
                name="genre", type="string", required=False, default="",
                description="Для update: жанр.",
            ),
            MCPToolParameter(
                name="notes", type="string", required=False, default="",
                description="Для update: заметка.",
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    # starts_music=False намеренно: флаг на уровне тула, а музыку запускает только action=play. С True
    # ``TrackStartGuard`` («один запуск трека за ход») отказал бы ``play`` после ``list`` в том же ходе.

    def execute(self, action: str = "", track_id: str = "", **kwargs: Any) -> MCPToolResult:
        handler = {
            "list": self._list, "search": self._search, "info": self._info, "play": self._play,
            "rate": self._rate, "update": self._update, "delete": self._delete,
        }.get(str(action))
        if handler is None:
            return MCPToolResult(success=False, error=f"неизвестное действие '{action}'")
        if action in _NEEDS_TRACK and not track_id:
            return MCPToolResult(success=False, error=f"для '{action}' нужен track_id (из list / search)")
        return handler(track_id=track_id, **kwargs)

    # ── Действия ──────────────────────────────────────────────────────────

    def _list(self, limit: int = 20, sort_by: str = "recent", **_: Any) -> MCPToolResult:
        from rob_box_voice.core.media_phrases import library_list_text

        result = self._library.list_tracks(limit=_clamp(limit, 20, 100), sort_by=sort_by)
        if not result.get("success"):
            return MCPToolResult(success=False, error=result.get("error"))
        tracks = result["tracks"]
        total = self._library.count  # в библиотеке всего, а не в выборке по limit
        data = {"tracks": [_brief(t) for t in tracks], "total": total, "shown": len(tracks), "sort_by": sort_by}
        titles = [str(t.get("title") or "") for t in tracks]
        return MCPToolResult(success=True, data=data, message=library_list_text(titles, total))

    def _search(self, query: str = "", limit: int = 5, **_: Any) -> MCPToolResult:
        result = self._library.search_tracks(query, limit=_clamp(limit, 5, 20))
        if not result.get("success"):
            return MCPToolResult(success=False, error=result.get("error"))
        found = result["total"]
        message = f"По запросу «{query}» найдено {found}." if found else f"По запросу «{query}» ничего не найдено."
        data = {"query": query, "tracks": [_brief(t) for t in result["tracks"]], "total": found}
        return MCPToolResult(success=True, data=data, message=message)

    def _info(self, track_id: str, **_: Any) -> MCPToolResult:
        info = self._library.get_track_info(track_id)
        if not info.get("success"):
            return MCPToolResult(success=False, error=info.get("error"))
        return MCPToolResult(success=True, data=info)

    def _play(self, track_id: str, **_: Any) -> MCPToolResult:
        return play_library_track(self.node, self._library, track_id, clock=self._clock, run=self._run)

    def _rate(self, track_id: str = "", verdict: str = "", **_: Any) -> MCPToolResult:
        """Оценка кодом: рейтинг пишется в БД, фраза — из результата записи. Трек — названный, играющий сейчас
        на деке (``library_id`` фактов владельца) или последний игравший из кэша."""
        from rob_box_voice.core.media_phrases import LIBRARY_NO_TRACK_TEXT, library_rate_text

        if verdict not in _VERDICT_RATING:
            return MCPToolResult(success=False, error="для rate нужен verdict: like или dislike")
        target = track_id or self._current_track_id()
        if not target:
            return MCPToolResult(success=False, error=LIBRARY_NO_TRACK_TEXT)
        saved = self._library.meta.set_rating(target, _VERDICT_RATING[verdict])
        if not saved.get("success"):
            return MCPToolResult(success=False, error=saved.get("error"))
        return MCPToolResult(
            success=True, data={"library_id": target, "title": saved.get("title"), "rating": saved["rating"]},
            message=library_rate_text(str(saved.get("title") or ""), verdict == "like"))

    def _current_track_id(self) -> Optional[str]:
        """Играющий трек кэша (факты владельца), иначе последний игравший."""
        owner = getattr(self.node, "_player_owner", None)
        if owner is not None:
            facts = owner.now_playing()
            if facts.get("playing") and facts.get("library_id"):
                return str(facts["library_id"])
        last = self._library.meta.last_played()
        return str(last["id"]) if last else None

    def _update(self, track_id: str, title: str = "", tags: str = "", mood: str = "", genre: str = "",
                notes: str = "", **_: Any) -> MCPToolResult:
        fields: Dict[str, Any] = {name: value for name, value in
                                  (("title", title), ("mood", mood), ("genre", genre), ("notes", notes)) if value}
        if tags:
            fields["tags"] = [t.strip() for t in tags.split(",") if t.strip()]
        if not fields:
            return MCPToolResult(success=False, error="update: не названо ни одного поля")
        saved = self._library.meta.update(track_id, **fields)
        if not saved.get("success"):
            return MCPToolResult(success=False, error=saved.get("error"))
        return MCPToolResult(success=True, data=_brief(saved), message=f"Метаданные трека обновлены: {', '.join(fields)}.")

    def _delete(self, track_id: str, **_: Any) -> MCPToolResult:
        from rob_box_voice.core.media_phrases import library_delete_text

        title = str(self._library.get_track_info(track_id).get("title") or "")
        result = self._library.delete_track(track_id)
        if not result.get("success"):
            return MCPToolResult(success=False, error=result.get("error"))
        return MCPToolResult(success=True, message=library_delete_text(title))


def _clamp(value: Any, default: int, high: int) -> int:
    try:
        return max(1, min(high, int(value)))
    except (TypeError, ValueError):
        return default


def _brief(track: Dict[str, Any]) -> Dict[str, Any]:
    """Строка списка для модели: без текстов песен и служебных полей."""
    keys = ("id", "title", "kind", "genre", "mood", "tags", "rating", "play_count", "duration_ms",
            "created_at", "last_played_at")
    return {k: track.get(k) for k in keys}
