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
from typing import Any, Callable, List, Mapping, Optional, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.source import CoverWork, Route, cover_failed_text, cover_ready_text

from .classic import pick_score
from .gen.cover import CoverResult, melody_notes
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


def theme_covers(dj: Any, theme: str, seed: int, log: Any = _LOG) -> List[Tuple[CoverWork, Any]]:
    """Наш узнаваемый материал темы сета для каверов (ADR-0158 В14): партитуры темы (поиск без LLM, как у моста),
    из первых ``SET_COVER_MATERIALS`` — те, где ``score_song`` нашёл мотив RTTTL-эталона темы (порог
    ``CLASSIC_REF_MATCH_MIN``; проба 14.10: «STAR WARS» arr. Hermand, 0.88). Эталонов нет — узнаваемость не доказать,
    каверов нет. ``(произведение, трек с мелодией)``."""
    profile = dj.theme_profile(theme, find=dj.quick_find)
    refs = list(dj._melodies(profile.theme_hooks).values())  # noqa: SLF001 — те же эталоны, что у сета синтезом
    ids = tuple(profile.materials)[:kn.SET_COVER_MATERIALS]
    if not ids or not refs:
        log.info(f"🎼 [cover] тема «{theme}»: каверов нет — материалов {len(ids)}, эталонов RTTTL {len(refs)}")
        return []
    loaded, out, reasons = PlanMaterials(dj._scores, ids, log), [], []  # noqa: SLF001
    for mid in ids:
        pick = pick_score(loaded, [mid], theme, seed=seed, references=refs)
        if pick.found:
            out.append((CoverWork(pick.work, mid, "", pick.title or pick.work), pick.track))
        else:
            reasons.append(pick.reason)
    log.info(f"🎼 [cover] тема «{theme}»: каверы по {[w.material for w, _t in out]}; не узнаётся: {reasons or '—'}")
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


__all__ = ["PieceCover", "cover_work", "theme_covers"]
