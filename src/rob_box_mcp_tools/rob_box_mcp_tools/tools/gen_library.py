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

``info`` / ``update`` / ``delete`` принимают только ``track_id``, который сам тул назвал в ответе ``list`` / ``search``
(:class:`NamedTracks`, #4025: n710 23.10 стёр посторонний трек по id, которого в ходе не было). ``delete`` необратим
(кредиты на перегенерацию стоят денег) и идёт в два вызова: без ``confirm`` тул отвечает ``needs_confirmation`` с
названием трека, ``confirm=true`` принимается только после такого вопроса (:class:`DeleteConfirmation`).

Внутренние исполнители (кавер, песня, сет, путь) тул не зовут — они вызывают
``engine.gen.library_play.play_library_track`` напрямую.
Модуль держит ROS-импорты ленивыми — тул тестируется без демона.
"""

from __future__ import annotations

import subprocess
import threading
import time
from collections import OrderedDict
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

from rob_box_music import knowledge as kn

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from ..engine.gen.library import GeneratedMusicLibrary
from ..engine.gen.library_play import play_library_track

#: Действия, которым нужен ``track_id`` (rate берёт играющий / последний, если id не назван).
_NEEDS_TRACK = ("info", "play", "update", "delete")
_VERDICT_RATING = {"like": kn.RATING_LIKE, "dislike": kn.RATING_DISLIKE}
#: Действия, где ``track_id`` обязан быть из ответа самого тула (``list`` / ``search``), а не из памяти модели (#4025).
_NAMED_ONLY = ("info", "update", "delete")
#: Причины отказа (поле ``reason`` результата): по ним диалог выбирает фразу, а не по тексту ошибки.
UNKNOWN_TRACK = "unknown_track"
NEEDS_CONFIRMATION = "needs_confirmation"
#: Сколько названных id помнит окно: до 100 строк на вызов ``list`` / ``search``, последние вызовы разговора.
NAMED_WINDOW_MAX = 200
#: Вопрос «удалить?» ждёт ответа человека не дольше этого (с, по часам тула); дальше спрашиваем заново.
CONFIRM_TTL_S = 120.0
#: Подтверждение в ту же секунду, что и вопрос, - не ответ человека (робот ещё говорит вопрос, человек отвечает, речь
#: распознаётся): так выглядит цепочка «удали, удали с confirm» одной командой модели. Раньше этого срока - снова вопрос.
CONFIRM_MIN_WAIT_S = 2.0


class NamedTracks:
    """Окно ``track_id``, которые тул назвал в ответах ``list`` / ``search`` (#4025, ADR-0148: допустимость id решает код).

    Модель подставляла id из пересказа, а не из ответа тула (n707/n710 23.10: ``info`` и ``delete`` по id, которого в
    ходе не было, - стёрт посторонний трек). Окно - последние ``limit`` названных id по давности обращения; тул один
    на процесс, поэтому окно общее для всех ходов и разговоров, пока жив сервер (перезапуск обнуляет - модель вызовет
    ``list``). Что оно защищает: от выдуманного id; чего НЕ защищает: от настоящего id из давнего ``list`` (один
    ``list`` на 100 строк называет всю небольшую библиотеку). От ошибочного ``delete`` настоящего трека оберегает
    только подтверждение (:class:`DeleteConfirmation`).
    """

    def __init__(self, limit: int = NAMED_WINDOW_MAX) -> None:
        self._limit = limit
        self._ids: "OrderedDict[str, None]" = OrderedDict()
        self._lock = threading.Lock()

    def name(self, track_ids: Iterable[Any]) -> None:
        """Тул назвал эти id в своём ответе."""
        with self._lock:
            for track_id in (str(t) for t in track_ids if t):
                self._ids[track_id] = None
                self._ids.move_to_end(track_id)
            while len(self._ids) > self._limit:
                self._ids.popitem(last=False)

    def __contains__(self, track_id: object) -> bool:
        key = str(track_id)  # нехешируемое (список, словарь) от небрежной модели - просто не названный id
        with self._lock:
            named = key in self._ids
            if named:
                self._ids.move_to_end(key)
            return named


class DeleteConfirmation:
    """Удаление трека в два вызова: вопрос - слово человека - подтверждённый вызов (#4025).

    Тул не слышит человека, но не даёт модели удалить трек одним вызовом: ``confirm=true`` засчитывается, только если
    этот же тул для этого же трека уже задал вопрос (:meth:`settle` вернул ``False``) не раньше ``CONFIRM_MIN_WAIT_S``
    и не позже ``CONFIRM_TTL_S`` назад. Подтверждение без вопроса, на другой трек, просроченное или слишком раннее -
    снова вопрос (слишком раннее срок вопроса не продлевает). Вопрос расходуется подтверждением.
    """

    def __init__(self, clock: Callable[[], float]) -> None:
        self._clock = clock
        self._asked: Optional[Tuple[str, float]] = None
        self._lock = threading.Lock()

    def settle(self, track_id: str, confirm: bool) -> bool:
        """``True`` - удалять; ``False`` - человека надо спросить (вопрос зафиксирован)."""
        now = self._clock()
        with self._lock:
            asked_id, asked_at = self._asked or ("", 0.0)
            waited = now - asked_at if asked_id == track_id else None
            if confirm and waited is not None and CONFIRM_MIN_WAIT_S <= waited <= CONFIRM_TTL_S:
                self._asked = None
                return True
            if not (confirm and waited is not None and waited < CONFIRM_MIN_WAIT_S):
                self._asked = (track_id, now)
            return False


class MusicLibraryTool(MCPTool):
    """Библиотека сгенерированной музыки: список, поиск, воспроизведение, оценка, правка, удаление."""

    event_source = "/voice/music/event"

    def __init__(self, node: Any, library: GeneratedMusicLibrary, clock: Callable[[], float] = time.time,
                 run: Callable[..., Any] = subprocess.run) -> None:
        super().__init__(node)
        self._library = library
        self._clock = clock
        self._run = run  # запуск ffmpeg (декод mp3 → wav сервера); тесты подставляют свой
        self._named = NamedTracks()  # id из ответов list / search: только их принимают info / update / delete (#4025)
        self._delete_question = DeleteConfirmation(clock)

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
            "update — переименовать, теги, настроение, жанр, заметка; delete — удалить трек насовсем: "
            "без confirm тул только спрашивает (needs_confirmation), confirm=true — после «да» человека. "
            "track_id для info / update / delete — только из ответа list / search, не выдумывай "
            "(иначе unknown_track). Ответ содержит фразу, которую надо сказать."
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
                description="UUID трека из ответа list / search (info, play, update, delete; у rate — по умолчанию "
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
            MCPToolParameter(
                name="confirm", type="boolean", required=False, default=False,
                description="Для delete: true — только после того, как человек ответил «да» на вопрос «Удалить "
                            "…?» (его задаёт сам тул ответом needs_confirmation). Без вопроса не засчитывается.",
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    # starts_music=False намеренно: флаг на уровне тула, а музыку запускает только action=play. С True
    # ``TrackStartGuard`` («один запуск трека за ход») отказал бы ``play`` после ``list`` в том же ходе.

    def execute(self, action: str = "", track_id: Any = "", **kwargs: Any) -> MCPToolResult:
        track_id = str(track_id) if track_id else ""  # список / словарь от небрежной модели - не TypeError, а неизвестный id
        handler = {
            "list": self._list, "search": self._search, "info": self._info, "play": self._play,
            "rate": self._rate, "update": self._update, "delete": self._delete,
        }.get(str(action))
        if handler is None:
            return MCPToolResult(success=False, error=f"неизвестное действие '{action}'")
        if action in _NEEDS_TRACK and not track_id:
            return MCPToolResult(success=False, error=f"для '{action}' нужен track_id (из list / search)")
        if action in _NAMED_ONLY and track_id not in self._named:
            return _refused(UNKNOWN_TRACK, f"Трек {track_id!r} я не называл: track_id берётся только из ответа "
                            "list / search этого разговора.", track_id=track_id,
                            hint="сначала action=list или search, track_id — из их ответа")
        return handler(track_id=track_id, **kwargs)

    # ── Действия ──────────────────────────────────────────────────────────

    def _list(self, limit: int = 20, sort_by: str = "recent", **_: Any) -> MCPToolResult:
        from rob_box_voice.core.media_phrases import library_list_text

        result = self._library.list_tracks(limit=_clamp(limit, 20, 100), sort_by=sort_by)
        if not result.get("success"):
            return MCPToolResult(success=False, error=result.get("error"))
        tracks = result["tracks"]
        self._named.name(t.get("id") for t in tracks)
        total = self._library.count  # в библиотеке всего, а не в выборке по limit
        data = {"tracks": [_brief(t) for t in tracks], "total": total, "shown": len(tracks), "sort_by": sort_by}
        titles = [str(t.get("title") or "") for t in tracks]
        return MCPToolResult(success=True, data=data, message=library_list_text(titles, total))

    def _search(self, query: str = "", limit: int = 5, **_: Any) -> MCPToolResult:
        result = self._library.search_tracks(query, limit=_clamp(limit, 5, 20))
        if not result.get("success"):
            return MCPToolResult(success=False, error=result.get("error"))
        found = result["total"]
        self._named.name(t.get("id") for t in result["tracks"])
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

    def _delete(self, track_id: str, confirm: Any = False, **_: Any) -> MCPToolResult:
        """Необратимо: без вопроса человеку не удаляем (:class:`DeleteConfirmation`), название трека - в вопросе."""
        from rob_box_voice.core.media_phrases import library_delete_text

        info = self._library.get_track_info(track_id)
        if not info.get("success"):
            return MCPToolResult(success=False, error=info.get("error"))
        title = str(info.get("title") or "")
        if not self._delete_question.settle(track_id, confirm is True or str(confirm).lower() == "true"):
            question = f"Удалить «{title}»? Скажи «да, удаляй»." if title else "Удалить этот трек? Скажи «да, удаляй»."
            return _refused(NEEDS_CONFIRMATION, question, track_id=track_id, title=title, question=question)
        result = self._library.delete_track(track_id)
        if not result.get("success"):
            return MCPToolResult(success=False, error=result.get("error"))
        return MCPToolResult(success=True, message=library_delete_text(title), data={"track_id": track_id, "title": title})


def _refused(reason: str, error: str, **data: Any) -> MCPToolResult:
    """Честный отказ: статус ``rejected`` и причина - в данных, фразу об исходе строит код по ``reason`` (ADR-0148)."""
    return MCPToolResult(success=False, error=error, message=error, data={"status": "rejected", "reason": reason, **data})


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
