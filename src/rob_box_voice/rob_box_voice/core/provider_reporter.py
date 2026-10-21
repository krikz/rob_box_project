"""ProviderStateReporter - здоровье провайдеров наружу и одна фраза на смену (ADR-0150 §8, #3693).

Без ROS: узел отдаёт ``publish`` (latched ``/voice/providers/state``, kind=llm), ``speak`` (озвучка) и ``board``
(снимок остальных писателей: tts/stt/music_gen). Решение «говорить ли и что» - в ``rob_box_dialog.degrade``
(код, а не LLM, ADR-0148): класс причины приходит от писателя, текст - из ``phrases/ru.yaml``. Свой вид
переиздаётся КАЖДЫЙ тик (подписчик считает запись без обновлений устаревшей).
"""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, List, Optional, Sequence

from rob_box_dialog.degrade import ProviderAnnouncer, ProviderBoard, ProviderState

#: Классы здоровья, о которых LLM знать нужно (ok/unknown не шумят).
_FACT_HEALTH = ("quota", "network", "timeout", "auth", "down")


class ProviderStateReporter:
    """Опрашивает ``llm.snapshot()`` (``HealthAwareFallbackLLM``), публикует, озвучивает отказ любого вида."""

    def __init__(
        self,
        llm: Any,
        publish: Callable[[List[ProviderState]], None],
        speak: Callable[[str], None],
        *,
        board: Optional[ProviderBoard] = None,
        clock: Callable[[], float] = time.time,
        announcer: Optional[ProviderAnnouncer] = None,
    ) -> None:
        self._llm = llm
        self._publish = publish
        self._speak = speak
        self._board = board or ProviderBoard()
        self._clock = clock
        self._announcer = announcer or ProviderAnnouncer()
        self._priority: List[str] = []

    @property
    def board(self) -> ProviderBoard:
        return self._board

    @property
    def enabled(self) -> bool:
        """Одиночный провайдер (без цепочки) снимка не даёт - свой вид не публикуем."""
        return callable(getattr(self._llm, "snapshot", None))

    def _own(self) -> List[ProviderState]:
        if not self.enabled:
            return []
        snap = self._llm.snapshot()
        self._priority = [s["name"] for s in snap]  # порядок цепочки = приоритет из llm_providers
        return [ProviderState(s["name"], "llm", s["health"], s["since"], s["last_error_code"] or None) for s in snap]

    def all_states(self) -> List[ProviderState]:
        """Свой LLM-вид + чужие виды с доски (устаревшие - ``unknown``)."""
        return self._own() + self._board.states(self._clock(), kinds=("tts", "stt", "music_gen"))

    def tick(self) -> None:
        own = self._own()
        if own:
            self._publish(own)
        states = own + self._board.states(self._clock(), kinds=("tts", "stt", "music_gen"))
        # Порядок запасных - по видам: llm - приоритет из цепочки, остальные - порядок писателя (#3755: список
        # llm и stt в одной куче давал «STT упал -> переключаюсь на deepseek»).
        priority: Dict[str, List[str]] = {"llm": list(self._priority)}
        for s in states:
            if s.kind != "llm":
                priority.setdefault(s.kind, []).append(s.name)
        for text in self._announcer.observe(states, priority, self._clock()):
            self._speak(text)

    def context_lines(self) -> List[str]:
        """Строка ``<providers>`` для системного контекста LLM: только то, что сломано (факт, а не догадка)."""
        return providers_context(self.all_states())


def providers_context(states: Sequence[ProviderState]) -> List[str]:
    """``<providers>`` из недоступных провайдеров; пусто, пока всё в порядке (контекст не раздуваем)."""
    broken = [f"{s.kind}:{s.name}={s.health}" for s in states if s.health in _FACT_HEALTH]
    if not broken:
        return []
    return ["  <providers>" + ", ".join(broken) + " - это факт из здоровья провайдеров; других отказов облака не выдумывай"
            "</providers>"]


def on_provider_states(board: ProviderBoard, clock: Callable[[], float], msg: Any) -> None:
    """Подписка на ``/voice/providers/state``: сообщение писателя -> доска (приём по часам подписчика)."""
    board.update(
        [ProviderState(p.name, p.kind, p.health, p.since.sec + p.since.nanosec / 1e9, p.last_error_code or None)
         for p in msg.providers],
        clock(),
    )


def reporter_context(reporter: Optional[ProviderStateReporter]) -> List[str]:
    """Строки ``<providers>`` для контекста; нет репортёра (стаб теста) - пусто."""
    return reporter.context_lines() if reporter is not None else []
