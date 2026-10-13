"""Контекст пути музыки из кэша генераций и учёта кредитов + выбор и строка лога (ADR-0158 §2.2, §2.6; PR-3).

Чистая часть — ``rob_box_music.source`` (кандидаты, фильтры, фразы); здесь — то, что читает диск: треки
``/data/music_library`` (``engine.gen.library``) и остаток дневного лимита (``engine.gen.budget`` по ``gen_ledger``).

Исполнитель генерации — ``SongStudio`` (PR-7, ``engine.gen.song``): он поёт текущим голосом, поэтому
``gen_ready`` истинно только для просьбы спеть (``ask.vocal``) и только когда студия подключена; инструментал и кавер
(PR-4/PR-8) исполнителя не имеют — кандидатов ``generate:*`` для них нет, с причиной ``not_wired`` во фразе кода.
Баланс KIE на пути заказа не спрашивается (сеть): неизвестный баланс — ``None``, 402 провайдера приведёт к выбору заново.

Одна строка лога на заказ: ``[source] <просьба> кандидаты=[…] → <путь> (<кто выбрал>) gen=<почему нет>``.
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Any, Callable, List, Optional, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.choice import Chooser
from rob_box_music.source import CacheTrack, Route, SourceAsk, SourceContext, choose_route, source_candidates

_LOG = logging.getLogger(__name__)
#: Сколько треков кэша читать в контекст (на роботе 13.10 их 9).
CACHE_LIMIT = 200


def _hours_ago(stamp: Optional[str], now: datetime) -> Optional[float]:
    if not stamp:
        return None
    try:
        then = datetime.fromisoformat(stamp)
    except ValueError:
        return None
    then = then if then.tzinfo else then.replace(tzinfo=timezone.utc)
    return max(0.0, (now - then).total_seconds() / 3600.0)


def cache_track(row: Any, now: datetime) -> CacheTrack:
    """Строка ``generated_tracks`` → :class:`CacheTrack`. Строки до ADR-0158 (MiniMax) без ``kind``: песня или
    инструментал по ``is_instrumental``; стиль — ``style_key`` или ``genre``, если это ключ ``GEN_STYLES``."""
    instrumental = bool(row.get("is_instrumental"))
    kind = row.get("kind") or ("instrumental" if instrumental else "song")
    style = next((s for s in (row.get("style_key"), row.get("genre")) if s in kn.GEN_STYLES), "")
    title = str(row.get("title") or "")
    return CacheTrack(str(row["id"]), title, kind, style, instrumental, int(row.get("rating") or 0),
                      _hours_ago(row.get("last_played_at"), now), str(row.get("task_id") or f"{kind}|{title}"),
                      str(row.get("voice") or ""))


def cache_tracks(library: Any, now: Optional[datetime] = None) -> Tuple[CacheTrack, ...]:
    """Треки кэша, файл которых есть на диске; библиотеки нет или SQLite упал — пусто."""
    if library is None:
        return ()
    listed = library.list_tracks(limit=CACHE_LIMIT, sort_by="recent")
    now = now or datetime.now(timezone.utc)
    root = library.root_dir
    return tuple(cache_track(row, now) for row in listed.get("tracks", ())
                 if (root / str(row["id"]) / "track.mp3").exists())


def day_left(library: Any) -> float:
    """Остаток дневного лимита кредитов по ``gen_ledger`` (``engine.gen.budget``); библиотеки нет — 0."""
    if library is None:
        return 0.0
    from .gen.budget import Budget

    return Budget(library).remaining_today()


class SourceRouter:
    """Путь заказа: контекст (кэш, лимит) → ``choose_route`` → строка лога. ``library`` — ленивый доступ к
    ``GeneratedMusicLibrary`` (её поднимает ``mcp_server`` позже движка v2; нет — кэш пуст)."""

    def __init__(self, chooser: Chooser, library: Callable[[], Any] = lambda: None,
                 log: Callable[[], Any] = lambda: _LOG, mode: str = "auto", studio: Any = None) -> None:
        self._chooser = chooser
        self._library = library
        self._log = log
        self._mode = mode
        self._studio = studio  # ``SongStudio`` (PR-7): исполнитель генерации песни голосом робота; нет — ``not_wired``

    def context(self, ask: Optional[SourceAsk] = None) -> SourceContext:
        library = self._library()
        try:
            cache, left = cache_tracks(library), day_left(library)
        except Exception as exc:  # noqa: BLE001 — кэш не держит заказ: без него остаются синтез и отказ
            self._log().warning(f"⚠️ [source] кэш генераций не прочитан: {type(exc).__name__}: {exc}")
            cache, left = (), 0.0
        studio = self._studio
        if studio is None:
            return SourceContext(cache=cache, gen_ready=False, day_left=left, mode=self._mode)
        return SourceContext(cache=cache, gen_ready=bool(ask is not None and ask.vocal), has_key=studio.has_key(),
                             provider_ok=studio.provider_ok(), day_left=left, mode=self._mode,
                             voice=studio.voice_id())

    def route(self, ask: SourceAsk, piece: bool = False, gen_failed: str = "") -> Route:
        """``gen_failed`` — выбор заново без ``generate:*`` (В3): генерация упала или ещё идёт (``pending``)."""
        ctx = self.context(ask)
        if gen_failed:
            ctx = ctx.without_generate(gen_failed)
        options, reason = source_candidates(ask, ctx, piece)
        route = choose_route(ask, ctx, self._chooser, piece=piece)
        listed: List[str] = [o.id for o in options]
        self._log().info(f"🎚️ [source] {ask.describe()} кандидаты={listed} → {route.id} ({route.provider or 'code'}) "
                         f"gen={reason or 'ok'}")
        return route


__all__ = ["CACHE_LIMIT", "SourceRouter", "cache_track", "cache_tracks", "day_left"]
