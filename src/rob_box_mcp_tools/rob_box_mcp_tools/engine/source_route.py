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

Одна строка лога на заказ: ``[source] <просьба> кандидаты=[…] → <путь> (<кто выбрал>) gen=<почему нет>
[work=«<названное>»|piece=«<подобранное кодом>»]`` (``work`` — что назвали, #4015; в конце строки: начало строки
читает генератор марафона, ``tests/unit/test_gen_night_marathon_results.py``); сет — по строке на каждый трек
(:meth:`SourceRouter.route_slots`, решение Шифу 14.10: кэш → генерация → синтез на каждый трек).
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from dataclasses import replace
from typing import Any, Callable, Dict, List, Optional, Sequence, Set, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.choice import Chooser
from rob_box_music.source import (CACHE, GENERATE, CacheTrack, CoverWork, Route, SourceAsk, SourceContext,
                                  choose_route, choose_with_fallback, cover_slot, gen_block, gen_kind, route_seconds,
                                  set_cache, set_candidates, source_candidates, theme_cache, theme_fit)

_LOG = logging.getLogger(__name__)
#: Причина «генерации нет» у трека сета, которому хватило кэша по теме (``source.REASON_WORDS``).
THEME_CACHE = "theme_cache"
#: Причина «генерации нет» у трека сета темы, которому взят трек кэша стиля: по теме в кэше нет ничего (29.10).
STYLE_CACHE = "style_cache"
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
                      str(row.get("prompt") or ""), float(row.get("duration_ms") or 0) / 1000.0)


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


def _named(work: Optional[CoverWork]) -> str:
    """Пьеса заказа — в строку лога ``[source]`` (#4015: причину отказа видно без чтения кода): названная человеком —
    `` work=«…»``, подобранная кодом (жанр, «что-нибудь», ``CoverWork.named``) — `` piece=«…»``; пьесы нет — «»."""
    return f" {'work' if work.named else 'piece'}=«{work.title}»" if work is not None else ""


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
                       balance=sets.balance() if ready else None,
                       cover_ready=ready and getattr(sets, "cover_ready", lambda: False)())

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
                         f"gen={reason or 'ok'}{_named(work)}")
        return route

    def route_slots(self, asks: Sequence[SourceAsk], theme_tags: str = "",
                    works: Sequence[CoverWork] = (), seconds: Optional[float] = None,
                    minimum: int = 1, gen_failed: str = "") -> List[Route]:
        """Путь каждого трека сета (ADR-0158 §2.7, В13, В14, В16): у темы есть произведения с нашим материалом
        (``works``, главная тема первой) — кавер произведения трека в его стиле (кэш того же кавера, затем
        ``generate:cover``; произведение — :meth:`SetLedger.work_for`); иначе кэш стиля и темы трека (инструментал,
        ``set_cache``; взятый трек следующему не достаётся) → генерация (деньги, лимит дня и сета — жёсткие фильтры,
        уже заказанное этим сетом вычитается; второй вариант задачи закрывает следующий трек того же стиля без новой
        траты) → синтез. Строка лога ``[source] сет i/N`` на каждый трек.
        ``seconds`` — сет «на N минут»: ``asks`` — запас просьб (по стилям по кругу), треки берутся, пока конец сета
        (``source.route_seconds``: треки за вычетом кроссфейдов + весь последний) не дойдёт до ``seconds`` минус
        ``SET_FILL_SLACK_S`` (не меньше ``minimum`` треков); лимит задач сета ``GEN_SET_MAX_TASKS`` и отказ пути тоже
        останавливают набор. ``gen_failed`` — генерацию в этом сете не заказывать (причина — в ``gen_block``):
        смена настроения сета (#4173) берёт только кэш и синтез."""
        base = self.context(asks[0] if asks else None, for_set=True)
        ledger = SetLedger(base.without_generate(gen_failed) if gen_failed else base, theme_tags, works)
        out = []
        body = end = 0.0  # секунды сета: треки до этого (без кроссфейда) и конец, если этот трек последний
        for no, ask in enumerate(asks, 1):
            if seconds and no > minimum and (end >= seconds - kn.SET_FILL_SLACK_S or ledger.tasks_full()):
                break
            work = ledger.work_for()
            ctx, free = ledger.context(ask, work)
            options, reason = set_candidates(ask, ctx, work)
            route = choose_route(ask, ctx, self._chooser, work=work, for_set=True)
            ledger.take(ask, route, free)
            end, body = body + route_seconds(route, True), body + route_seconds(route)
            of = no if seconds else f"{no}/{len(asks)}"
            self._log().info(f"🎚️ [source] сет {of}: {ask.describe()} кандидаты={[o.id for o in options]} "
                             f"→ {route.id} ({route.provider or 'code'}) gen={reason or 'ok'}{_named(work)}"
                             + (" (второй вариант задачи)" if route.kind == GENERATE and free else ""))
            out.append(route)
        return out

    def style_cache(self, ask: SourceAsk) -> Optional[CacheTrack]:
        """Первый трек кэша стиля сета без темы (``set_cache``: свежий раньше) — фон сета темы, пока пишется её первый
        трек (#3775); кэша нет — ``None``."""
        found = set_cache(ask, self.context(ask, for_set=True))
        return found[0] if found else None

    def _executable(self, route: Route) -> str:
        """Кавер, песня и инструментал исполняются в фоне после выбора (мост играет сразу): выбор остаётся;
        ``not_wired`` — только когда исполнителя этого вида нет. Названное произведение тут ни при чём (#4015: «спой
        <название>» при живой студии давало ``not_wired``): песню на чужое кандидаты не строят до выбора
        (``source.NAMED_SONG``)."""
        kind = gen_kind(route.id)
        if kind == "cover":
            return ""
        executor = self._instrumental if kind == "instrumental" else self._studio
        return "" if executor is not None else "not_wired"


class SetLedger:
    """Учёт пути сета по трекам: взятые треки кэша, потраченное заказами (вычитается из лимитов дня и сета) и
    запасные вторые варианты задач по ключу слота (задача — 2 варианта: следующий трек того же ключа бесплатен).
    Каверы темы (В16, Шифу 14.10 «одни и те же треки ставит»): у каждого произведения свой ключ ``cover|стиль|пьеса``,
    произведение трека — наименее игранное в сете и не то же, что у соседа; первый проход — по порядку темы."""

    def __init__(self, base: SourceContext, theme_tags: str = "", works: Sequence[CoverWork] = ()) -> None:
        self._base, self._tags, self._works = base, theme_tags, tuple(dict.fromkeys(works))
        self._cost = kn.GEN_MODELS[kn.GEN_SET_MODEL].credits_per_task
        self._used: Set[str] = set()
        self._spare: Dict[str, int] = {}
        self._plays: Dict[CoverWork, int] = {w: 0 for w in self._works}
        self._last: Optional[CoverWork] = None  # произведение предыдущего трека (кавер); иначе ``None``
        self.spent = 0.0

    def tasks_full(self) -> bool:
        """Задач генерации заказано ``GEN_SET_MAX_TASKS``, запасного варианта нет: следующий трек заказать нечем."""
        return round(self.spent / self._cost) >= kn.GEN_SET_MAX_TASKS and not any(self._spare.values())

    def work_for(self) -> Optional[CoverWork]:
        """Произведение кавера трека: реже всех сыгранное в сете (ничья — порядок темы), не сосед — пока есть другое;
        его второй вариант задачи берётся этим же ключом (:meth:`context`). Произведений нет — ``None``."""
        if not self._works:
            return None
        others = [w for w in self._works if w != self._last] or list(self._works)
        return min(others, key=lambda w: (self._plays[w], self._works.index(w)))

    def _key(self, ask: SourceAsk, work: Optional[CoverWork]) -> str:
        return cover_slot(ask.genre, work.title) if work is not None else ask.genre

    def context(self, ask: SourceAsk, work: Optional[CoverWork] = None) -> Tuple[SourceContext, bool]:
        """Контекст трека: кэш стиля и темы без взятых (и каверы, если у темы есть материал), лимиты за вычетом
        потраченного; ``True`` — есть запасной вариант задачи стиля или кавера ``work`` (генерация бесплатна).
        Тема без каверов: трек кэша по теме (``source.theme_cache``, близкий по словам, не только метка) есть — он, без
        заказа (``THEME_CACHE``): из N треков сета K берутся из кэша, N − K заказываются. Кэша темы нет и заказать
        нельзя (деньги, лимит, провайдер) — кэш стиля без темы: он лучше синтеза (В13)."""
        base = self._base
        free = self._spare.get(ask.genre, 0) > 0 or (work is not None and self._spare.get(self._key(ask, work), 0) > 0)
        back = self._cost if free else 0.0
        pool = replace(base, cache=tuple(t for t in base.cache if t.id not in self._used))
        covers = tuple(t for t in pool.cache if t.kind == "cover") if self._works else ()
        left = None if base.set_left is None else base.set_left - self.spent + back
        ctx = replace(pool, day_left=base.day_left - self.spent + back, set_left=left)
        cache, ctx = self._cache(ask, pool, ctx, work)
        if not cache and self._tags and not free and gen_block(ctx, self._cost):
            cache = set_cache(ask, pool)
        return replace(ctx, cache=tuple(cache) + covers), free

    def _cache(self, ask: SourceAsk, pool: SourceContext, ctx: SourceContext,
               work: Optional[CoverWork]) -> Tuple[List[CacheTrack], SourceContext]:
        """Кэш трека: у темы без каверов — треки по теме, и тогда без заказа (что есть по теме — сразу, заказ —
        только недостающим, Шифу 21.10, #3775); иначе кэш стиля с меткой темы, как было. По теме в кэше нет ничего
        во всём кэше (K = 0, :meth:`_theme_miss`) — сначала треки кэша того же стиля, заказ — только когда их не хватило
        (``knowledge.SET_THEME_MISS_STYLE_CACHE``, 29.10: «почему кэш пустой»)."""
        if not self._tags or work is not None:
            return set_cache(ask, pool, self._tags), ctx
        cache = theme_cache(ask, pool, self._tags)
        if cache:
            return cache, ctx.without_generate(THEME_CACHE)
        if not self._theme_miss():
            return cache, ctx
        cache = set_cache(ask, pool)
        return cache, (ctx.without_generate(STYLE_CACHE) if cache else ctx)

    def _theme_miss(self) -> bool:
        """По теме сета во всём кэше нет ни одного инструментала (любого стиля; по всему кэшу, а не по оставшемуся:
        взятые сетом треки темы — не «нет ничего», дальше дозаказ, как 21.10)."""
        return kn.SET_THEME_MISS_STYLE_CACHE and not any(
            t.kind == "instrumental" and t.rating >= 0 and theme_fit(self._tags, t) >= kn.THEME_FIT_MIN
            for t in self._base.cache)

    def take(self, ask: SourceAsk, route: Route, free: bool) -> None:
        cover = gen_kind(route.id) == "cover" or (route.kind == CACHE and getattr(route.track, "kind", "") == "cover")
        work = route.work if cover else None
        key = self._key(ask, work)
        if work is not None:
            self._plays[work] = self._plays.get(work, 0) + 1
        self._last = work
        if route.kind == CACHE and route.track is not None:
            self._used.add(route.track.id)
        elif route.kind == GENERATE and self._spare.get(key, 0) > 0:
            self._spare[key] -= 1
        elif route.kind == GENERATE:
            self.spent += self._cost
            self._spare[key] = self._spare.get(key, 0) + 1


__all__ = ["CACHE_LIMIT", "STYLE_CACHE", "THEME_CACHE", "SetLedger", "SourceRouter", "cache_track", "cache_tracks",
           "day_left"]
