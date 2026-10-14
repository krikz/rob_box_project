"""Контекст пути музыки из кэша генераций и учёта кредитов + выбор и строка лога (ADR-0158 §2.2, §2.6; PR-3).

Чистая часть — ``rob_box_music.source`` (кандидаты, фильтры, фразы); здесь — то, что читает диск: треки
``/data/music_library`` (``engine.gen.library``) и остаток дневного лимита (``engine.gen.budget`` по ``gen_ledger``).

Исполнители генерации: кавер пьесы (PR-4, ``engine.gen.cover``: ``SourceContext.cover_ready`` и последний известный
баланс KIE, обновляется в фоне, заказ сеть не ждёт) и песня голосом робота (PR-7, ``engine.gen.song.SongStudio``:
``gen_ready`` истинно только для просьбы спеть при подключённой студии; Silero — ``song_block``) и сет из генерации
(PR-8, ``engine.gen.gen_session.GenSetMaker``: ``route(..., for_set=True)`` — генерация готова, если готов исполнитель
сета; лимит сета ``set_left`` — из ``engine.gen.budget``, одно место) и одиночный инструментал
(``engine.gen.instrumental.InstrumentalMaker``: ``gen_ready`` для просьбы без пения при готовом исполнителе;
живой прогон 14.10 — «сгенерируй рок» уходил в кэш с ``not_wired``).

Одна строка лога на заказ: ``[source] <просьба> кандидаты=[…] → <путь> (<кто выбрал>) gen=<почему нет>``; сет — по
строке на каждый трек (:meth:`SourceRouter.route_slots`, решение Шифу 14.10: кэш → генерация → синтез на каждый трек).
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from dataclasses import replace
from typing import Any, Callable, Dict, List, Optional, Sequence, Set, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.choice import Chooser
from rob_box_music.source import (CACHE, GENERATE, CacheTrack, CoverWork, Route, SourceAsk, SourceContext,
                                  choose_route, choose_with_fallback, gen_kind, set_cache, source_candidates)

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
                      str(row.get("work_id") or ""), str(row.get("instrument") or ""), str(row.get("voice") or ""),
                      str(row.get("prompt") or ""))


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


def _set_attempt(_route: Route) -> str:
    """Генерация сета исполняется самим сетом (заказ недостающего по ходу): выбор остаётся."""
    return ""


class SourceRouter:
    """Путь заказа: контекст (кэш, лимит) → ``choose_with_fallback`` → строка лога. ``library`` — ленивый доступ к
    ``GeneratedMusicLibrary`` (её поднимает ``mcp_server`` позже движка v2; нет — кэш пуст); ``cover`` — исполнитель
    кавера пьесы (``engine.gen.cover.CoverMaker``, ADR-0158 PR-4): его готовность и последний баланс — в контекст;
    ``studio`` — ``SongStudio`` (PR-7): исполнитель песни голосом робота; ``instrumental`` —
    ``InstrumentalMaker``: одиночный инструментал."""

    def __init__(self, chooser: Chooser, library: Callable[[], Any] = lambda: None,
                 log: Callable[[], Any] = lambda: _LOG, mode: str = "auto", cover: Any = None,
                 studio: Any = None, sets: Any = None, instrumental: Any = None) -> None:
        self._chooser = chooser
        self._library = library
        self._log = log
        self._mode = mode
        self._cover = cover
        self._studio = studio  # ``SongStudio`` (PR-7); нет — песня ``not_wired``
        self._sets = sets  # ``GenSetMaker`` (PR-8): исполнитель сета из генерации
        self._instrumental = instrumental  # ``InstrumentalMaker``: одиночный инструментал; нет — ``not_wired``

    def context(self, ask: Optional[SourceAsk] = None, for_set: bool = False) -> SourceContext:
        library = self._library()
        try:
            cache, left = cache_tracks(library), day_left(library)
        except Exception as exc:  # noqa: BLE001 — кэш не держит заказ: без него остаются синтез и отказ
            self._log().warning(f"⚠️ [source] кэш генераций не прочитан: {type(exc).__name__}: {exc}")
            cache, left = (), 0.0
        cover = self._cover is not None and self._cover.ready()
        ctx = SourceContext(cache=cache, gen_ready=False, day_left=left, mode=self._mode, cover_ready=cover,
                            balance=self._cover.balance() if cover else None)
        if for_set:  # сет (PR-8): генерацию исполняет сет, лимит сета — engine.gen.budget
            return self._set_context(ctx)
        if ask is not None and ask.vocal:
            studio = self._studio
            return ctx if studio is None else replace(
                ctx, gen_ready=True, song_block=studio.blocked(), has_key=studio.has_key(),
                provider_ok=studio.provider_ok(), voice=studio.voice_id())
        maker = self._instrumental
        if maker is None or not maker.ready():
            return ctx
        return replace(ctx, gen_ready=True, provider_ok=maker.provider_ok(), balance=maker.balance())

    def _set_context(self, ctx: SourceContext) -> SourceContext:
        """Контекст сета (PR-8): генерация готова, если готов исполнитель сета; лимит сета — ``BudgetPolicy``."""
        from .gen.budget import BudgetPolicy

        sets = self._sets
        ready = sets is not None and sets.ready()
        return replace(ctx, gen_ready=ready, set_left=BudgetPolicy().set_credits,
                       balance=sets.balance() if ready else None)

    def route(self, ask: SourceAsk, piece: bool = False, work: Optional[CoverWork] = None,
              gen_failed: str = "", for_set: bool = False) -> Route:
        """``gen_failed`` — выбор заново без ``generate:*`` (В3): песня упала или ещё пишется (``pending``);
        ``for_set`` — путь сета (PR-8): недостающее заказывает сам сет."""
        ctx = self.context(ask, for_set)
        if gen_failed:
            ctx = ctx.without_generate(gen_failed)
        options, reason = source_candidates(ask, ctx, piece, work)
        attempt = _set_attempt if for_set else self._executable
        route = choose_with_fallback(ask, ctx, self._chooser, attempt=attempt, piece=piece, work=work)
        listed: List[str] = [o.id for o in options]
        self._log().info(f"🎚️ [source] {ask.describe()} кандидаты={listed} → {route.id} ({route.provider or 'code'}) "
                         f"gen={reason or 'ok'}")
        return route

    def route_slots(self, asks: Sequence[SourceAsk], theme_tags: str = "") -> List[Route]:
        """Путь каждого трека сета (ADR-0158 §2.7, В13): кэш стиля и темы трека (инструментал, ``set_cache``; взятый
        трек следующему не достаётся) → генерация (деньги, лимит дня и сета — жёсткие фильтры, уже заказанное этим
        сетом вычитается; второй вариант задачи закрывает следующий трек того же стиля без новой траты) → синтез.
        Строка лога ``[source] сет i/N`` на каждый трек."""
        ledger = SetLedger(self.context(asks[0] if asks else None, for_set=True), theme_tags)
        out = []
        for no, ask in enumerate(asks, 1):
            ctx, free = ledger.context(ask)
            options, reason = source_candidates(ask, ctx)
            route = choose_route(ask, ctx, self._chooser)
            ledger.take(ask, route, free)
            self._log().info(f"🎚️ [source] сет {no}/{len(asks)}: {ask.describe()} кандидаты={[o.id for o in options]} "
                             f"→ {route.id} ({route.provider or 'code'}) gen={reason or 'ok'}"
                             + (" (второй вариант задачи)" if route.kind == GENERATE and free else ""))
            out.append(route)
        return out

    def _executable(self, route: Route) -> str:
        """Кавер, песня и инструментал исполняются в фоне после выбора (мост играет сразу): выбор остаётся;
        исполнителя нет — ``not_wired``."""
        kind = gen_kind(route.id)
        if kind == "cover":
            return ""
        executor = self._instrumental if kind == "instrumental" else self._studio
        return "" if executor is not None and route.work is None else "not_wired"


class SetLedger:
    """Учёт пути сета по трекам: взятые треки кэша, потраченное заказами (вычитается из лимитов дня и сета) и
    запасные вторые варианты задач по стилю (задача — 2 варианта: следующий трек того же стиля бесплатен)."""

    def __init__(self, base: SourceContext, theme_tags: str = "") -> None:
        self._base, self._tags = base, theme_tags
        self._cost = kn.GEN_MODELS[kn.GEN_SET_MODEL].credits_per_task
        self._used: Set[str] = set()
        self._spare: Dict[str, int] = {}
        self.spent = 0.0

    def context(self, ask: SourceAsk) -> Tuple[SourceContext, bool]:
        """Контекст трека: кэш стиля и темы без взятых, лимиты за вычетом потраченного; ``True`` — есть запасной
        вариант стиля (генерация трека бесплатна)."""
        base = self._base
        free = self._spare.get(ask.genre, 0) > 0
        back = self._cost if free else 0.0
        pool = replace(base, cache=tuple(t for t in base.cache if t.id not in self._used))
        left = None if base.set_left is None else base.set_left - self.spent + back
        return replace(pool, cache=tuple(set_cache(ask, pool, self._tags)), day_left=base.day_left - self.spent + back,
                       set_left=left), free

    def take(self, ask: SourceAsk, route: Route, free: bool) -> None:
        if route.kind == CACHE and route.track is not None:
            self._used.add(route.track.id)
        elif route.kind == GENERATE and free:
            self._spare[ask.genre] -= 1
        elif route.kind == GENERATE:
            self.spent += self._cost
            self._spare[ask.genre] = self._spare.get(ask.genre, 0) + 1


__all__ = ["CACHE_LIMIT", "SetLedger", "SourceRouter", "cache_track", "cache_tracks", "day_left"]
