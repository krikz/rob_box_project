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
from typing import Any, Callable, Mapping, Optional, Tuple

from rob_box_music.instruments import lead_choice
from rob_box_music.source import CoverWork, Route, cover_failed_text, cover_ready_text
from rob_box_music.works import RTTTL, record_public_domain

from .classic import ClassicPick, pick_record
from .gen.cover import CoverResult

_LOG = logging.getLogger(__name__)


def cover_work(staged: Optional[Mapping[str, Any]]) -> Optional[CoverWork]:
    """Собранная пьеса (``request_music._stage_classic``) → :class:`CoverWork`; пьеса не собралась — ``None``."""
    if not staged or "program" not in staged or not staged.get("work"):
        return None
    return CoverWork(str(staged["work"]), bool(staged.get("public_domain")), str(staged.get("instrument") or ""),
                     str(staged.get("title") or ""))


def public_domain_pick(pieces: Any, query: str, seed: int, lead: Any = None) -> Optional[ClassicPick]:
    """Запись RTTTL этого же заказа, совпавшая по названию и общественного достояния (реестр ``works``) — источник
    кавера, когда пьеса моста выбрана другая. Живой прогон 13.10: «включи полёт валькирий» — `rideofth` (Вагнер)
    недавно играл, свежесть выбрала `valkyrie` («In Development», не public domain), и кавер из кэша не нашёлся."""
    _ref, candidates, records = pieces.candidates(query)
    for cand in candidates:
        if cand.kind != RTTTL or not cand.title_match or cand.id not in records:
            continue
        record, title = records[cand.id]
        if record_public_domain(record):
            pick = pick_record(record, title, query, seed=seed, lead=lead)
            if pick.found:
                return pick
    return None


class PieceCover:
    """Кавер после моста. ``maker`` — ``gen.cover.CoverMaker``; ``speak(text)`` — реплика робота."""

    def __init__(self, maker: Any, speak: Optional[Callable[[str], Any]] = None,
                 log: Callable[[], Any] = lambda: _LOG) -> None:
        self.maker = maker
        self._speak = speak
        self._log = log

    def source(self, piece: Optional[Mapping[str, Any]], pieces: Any, seed: int) -> Tuple[Optional[CoverWork], Any]:
        """``(произведение кавера, трек-источник мелодии | None — мелодия моста)``: пьеса моста не общественное
        достояние — та же просьба среди записей public domain (:func:`public_domain_pick`)."""
        work = cover_work(piece)
        query = str(((piece or {}).get("played") or {}).get("work") or "")
        if work is None or work.public_domain or not query:
            return work, None
        pick = public_domain_pick(pieces, query, seed, lead_choice(work.instrument))
        if pick is None:
            return work, None
        self._log().info(f"🎼 [cover] мост «{work.title}» не public domain → кавер по «{pick.work}» ({pick.melody_id})")
        return CoverWork(pick.work, True, work.instrument, pick.title), pick.track

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


__all__ = ["PieceCover", "cover_work", "public_domain_pick"]
