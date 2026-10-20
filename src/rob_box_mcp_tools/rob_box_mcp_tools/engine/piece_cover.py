"""Кавер пьесы на деке: мост синтезом уже играет, кавер готовится в фоне и встаёт вместо него (ADR-0158 §2.9, PR-4).

Путь выбрал чойсер (``rob_box_music.source``: ``generate:cover``), мост — наш синтез той же пьесы — запустил
``request_music``. Здесь — что делать, когда кавер готов или не вышел:

* мост держит звук до результата или провала (``engine.gen.gen_session.BridgeHold``, ADR-0158 PR-8: живой прогон
  13.10 — мост доиграл 22 с, кавер встал через 64 с, 44 с тишины): готов — кавер входит кроссфейдом на конце
  прохода моста, на его ``started`` — фраза кода «А вот живая версия …»;
* человек тем временем остановил музыку или заказал другое — кавер остаётся в кэше (следующий заказ сыграет его
  оттуда), деку не трогаем, фразы нет;
* не вышел (деньги, сервис, отказ) — мост доигрывает финальным проходом, фраза кода с причиной (В3), если мост
  ещё звучит.
"""

from __future__ import annotations

import logging
from typing import Any, Callable, List, Mapping, Optional, Sequence, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.arrange.hook import theme_fit, tune_groups
from rob_box_music.source import CoverWork, Route, cover_failed_text, cover_ready_text

from .classic import pick_record, pick_score
from .gen.cover import CoverResult, Sources, melody_notes
from .score_library import PlanMaterials

_LOG = logging.getLogger(__name__)


def cover_work(staged: Optional[Mapping[str, Any]]) -> Optional[CoverWork]:
    """Собранная пьеса (``request_music._stage_classic``) → :class:`CoverWork`; пьеса не собралась — ``None``.
    Материал кавера — мелодия, из которой собран лид моста (В14: та же мелодия у моста и у кавера, без подмены);
    лида и хука нет — ``material`` пуст, кавер-кандидата нет."""
    if not staged or "program" not in staged or not staged.get("work"):
        return None
    material = str(staged.get("melody_id") or staged["work"]) if melody_notes(staged.get("track")) else ""
    return CoverWork(str(staged["work"]), material, str(staged.get("instrument") or ""),
                     str(staged.get("title") or ""))


#: ``{id: (запись RTTTL, название для голоса)}`` по id хуков (``tools_v2.library_records``).
Records = Callable[[Sequence[str]], Mapping[str, Tuple[Mapping[str, Any], str]]]


def theme_pieces(profile: Any, rtttls: Mapping[str, str]) -> List[Tuple[str, ...]]:
    """Произведения темы (В16): хуки по мелодии (``hook.tune_groups``), первым — произведение с ранним хуком строки
    таблицы тем (её первый хук — главная тема: ``space`` → starwars_6), дальше — в порядке поиска; не больше
    ``SET_COVER_PIECES``."""
    lead = kn.THEMES[profile.row].hooks if profile.row in kn.THEMES else ()
    groups = tune_groups(rtttls)
    rank = [min((lead.index(h) for h in g if h in lead), default=len(lead)) for g in groups]
    return [g for _r, _i, g in sorted(zip(rank, range(len(groups)), groups))][:kn.SET_COVER_PIECES]


def _piece_cover(piece: Tuple[str, ...], found: Mapping[str, Tuple[Mapping[str, Any], str]], loaded: Any,
                 ids: Sequence[str], theme: str, seed: int) -> Tuple[Optional[Tuple[CoverWork, Sources]], str]:
    """Мелодии кавера произведения по порядку попыток (В17): первой — партитура темы с мотивом ЭТИХ эталонов
    (``score_song``, порог ``CLASSIC_REF_MATCH_MIN``), затем сами RTTTL его хуков (путь пробы Валькирий 13.10) —
    другой исходник после отказа Suno. ``(произведение, мелодии)`` и строка лога."""
    title = found[piece[0]][1]
    refs = [str(found[h][0].get("rtttl") or "") for h in piece]
    pick = pick_score(loaded, ids, theme, seed=seed, references=refs) if ids else None
    sources: List[Tuple[str, Any]] = []
    said = ""
    if pick is not None and pick.found:
        fit = theme_fit(loaded[pick.melody_id], refs) or 0.0
        sources.append((str(pick.melody_id), pick.track))
        said = f"«{title}» ← {pick.melody_id} {fit:.2f}"
    for hook in piece:
        record = pick_record(found[hook][0], title, theme, seed=seed)
        if record.found:
            sources.append((f"rtttl:{hook}", record.track))
    if not sources:
        return None, f"«{title}» — не собрался ({pick.reason if pick is not None else 'партитур нет'})"
    said = said or f"«{title}» ← {sources[0][0]}"
    material = sources[0][0].removeprefix("rtttl:")
    return (CoverWork(title, material, "", title), tuple(sources)), f"{said} (запас {len(sources) - 1})"


def theme_covers(dj: Any, theme: str, seed: int, log: Any = _LOG,
                 records: Optional[Records] = None) -> List[Tuple[CoverWork, Sources]]:
    """Произведения темы сета для каверов (ADR-0158 В14, В16): хуки поиска по словам темы (нет — хуки строки)
    по мелодии в произведения (:func:`theme_pieces`, главная тема первой); у каждого — свой эталон и своя мелодия
    (:func:`_piece_cover`: партитура из первых ``SET_COVER_MATERIALS`` темы, где мотив этого эталона, иначе RTTTL).
    Эталонов нет — узнаваемость не доказать, каверов нет. ``(произведение, мелодии по порядку попыток)``; строка лога
    ``[cover] тема …`` — произведения и источник мелодии каждого."""
    profile = dj.theme_profile(theme, find=dj.quick_find)
    hooks = profile.found or profile.theme_hooks
    found = {h: r for h, r in (records(hooks) if records is not None else {}).items() if r[0].get("rtttl")}
    ids = tuple(profile.materials)[:kn.SET_COVER_MATERIALS]
    if not found:
        log.info(f"🎼 [cover] тема «{theme}»: каверов нет — эталонов RTTTL нет (материалов {len(ids)})")
        return []
    loaded = PlanMaterials(dj._scores, ids, log) if ids else {}  # noqa: SLF001
    out, said = [], []
    for piece in theme_pieces(profile, {h: str(found[h][0]["rtttl"]) for h in hooks if h in found}):
        cover, line = _piece_cover(piece, found, loaded, ids, theme, seed)
        said.append(f"{line} [{', '.join(piece)}]")
        if cover is not None:
            out.append(cover)
    log.info(f"🎼 [cover] тема «{theme}»: произведения {'; '.join(said)}")
    return out


class PieceCover:
    """Кавер после моста. ``maker`` — ``gen.cover.CoverMaker``; ``speak(text)`` — реплика робота."""

    def __init__(self, maker: Any, speak: Optional[Callable[[str], Any]] = None,
                 log: Callable[[], Any] = lambda: _LOG) -> None:
        self.maker = maker
        self._speak = speak
        self._log = log

    def start(self, route: Route, track: Any, hold: Any) -> Any:
        """Заказать кавер в фоне; ``hold`` — мост на деке (``BridgeHold``), держит звук до результата."""
        work: CoverWork = route.work  # type: ignore[assignment] — кавер без произведения не заказывается
        hold.on_live = lambda: self._say(cover_ready_text(work, route.ask))
        return self.maker.start(track, work, route.ask, lambda result: self.done(route, hold, result))

    def done(self, route: Route, hold: Any, result: CoverResult) -> None:
        work: CoverWork = route.work  # type: ignore[assignment]
        if result.reason:
            if hold.fail():
                self._say(cover_failed_text(work, result.reason))
            return
        if hold.deliver(result.track_id, work.spoken or work.title):
            self._log().info(f"🎼 [cover] «{work.title}» готов ({result.track_id}) — войдёт вместо моста "
                             f"{hold.bridge_id} кроссфейдом")
        else:
            self._log().info(f"🎼 [cover] «{work.title}» в кэше ({result.track_id}), деку не трогаю: "
                             f"на ней уже не мост {hold.bridge_id}")

    def _say(self, text: str) -> None:
        if text and self._speak is not None:
            self._speak(text)


__all__ = ["PieceCover", "Records", "cover_work", "theme_covers", "theme_pieces"]
