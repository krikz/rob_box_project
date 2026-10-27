"""Путь пения хода Личности до основной модели (#4079, ADR-0158 §2.2; ADR-0148): хост ``TurnEngine(sing=…)``.

Решает ``rob_box_music.source.choose_sing``: маркеры слов (``knowledge.SING_ROUTE_MARKERS`` и слова движка) сужают
кандидатов кодом, голое «спой» — чойсер с перевесом живому. Здесь — то, что знает процесс диалога: здоровье генерации
(``ProviderState`` вида ``music_gen`` на доске провайдеров, его издаёт ``mcp_server``), прошлые реплики человека и
играет ли музыка (их даёт ход), и строка лога для харнесса. Баланса KIE процесс диалога не знает: деньги и лимиты
судит тул генерации (``source.gen_block``) и говорит о них фразой кода. Пакет диалога от ``rob_box_music`` не зависит —
решение уходит ему строкой id кандидата (``live:song``, ``generate:robot_song:V6``; «» — не пение).

Тем же вызовом — путь «правка играющего» (#4029): не пение, музыка играет, реплика правит именно её
(``rob_box_music.track_edit.edit_route``: «сделай бит потяжелее», «Робот, ускорь», «добавь баса» — часть реплики
называет играющее или вся реплика — правка; вопрос — не правка) и другой владелец слов правки не назван
(``knowledge.EDIT_OTHER_OWNER_STEMS``: «читай быстрее» — голос, «иди быстрее» — движение) — id ``edit:<вид>+…``:
форма хода закрывает заказ нового и требует ``edit_music``. Иначе («иди быстрее к двери», «продолжай рассказывать»,
«это тот же трек?») пути нет: ход отвечает как обычно.
"""

from __future__ import annotations

from typing import Any, Callable, Sequence

from rob_box_dialog.knowledge import EDIT_OTHER_OWNER_STEMS, PROVIDER_USABLE_HEALTH
from rob_box_dialog.words import has_word
from rob_box_music.choice import Chooser
from rob_box_music.source import SingContext, choose_sing
from rob_box_music.track_edit import edit_route

#: Вид провайдера генерации музыки на доске (``rob_box_dialog.knowledge.PROVIDER_KINDS``).
GEN_KIND = "music_gen"


def gen_health(states: Sequence[Any]) -> str:
    """Здоровье генерации (``ProviderState.health`` вида ``music_gen``); записи нет — «» (неизвестно: генерация не
    исключается, её проверит тул)."""
    return next((str(s.health) for s in states if getattr(s, "kind", "") == GEN_KIND), "")


class SingRouteHost:
    """``(реплика, прошлые реплики, играет ли музыка) -> id пути пения``: контекст процесса диалога → ``choose_sing``
    → строка лога ``🎚️ [source] пение … кандидаты=[…] → <id> (code|chooser:…)``. ``states`` — состояния провайдеров
    (доска ``ProviderStateReporter``), ``chooser`` — тот же выбиратель, что у пути музыки (``chooser_from_env``)."""

    def __init__(self, chooser: Chooser, states: Callable[[], Sequence[Any]], log: Callable[[str], Any]) -> None:
        self._chooser = chooser
        self._states = states
        self._log = log

    def __call__(self, text: str, recent: Sequence[str], playing: bool) -> str:
        health = gen_health(self._states())
        ctx = SingContext(tuple(recent), playing, not health or health in PROVIDER_USABLE_HEALTH, health)
        route = choose_sing(text, ctx, self._chooser)
        if route.id:
            self._log(f"🎚️ {route.line()}")
            return route.id
        edit = "" if has_word(text, EDIT_OTHER_OWNER_STEMS) else edit_route(text, playing)
        if edit:
            self._log(f"🎚️ [edit] путь хода {edit}: играет музыка, реплика её правит — новый трек не заказываем")
        return edit


__all__ = ["GEN_KIND", "SingRouteHost", "gen_health"]
