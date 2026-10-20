"""ProviderStateReporter - здоровье LLM-провайдеров наружу и одна фраза на смену (ADR-0150 §8, #3693).

Без ROS: узел отдаёт ``publish`` (latched ``/voice/providers/state``) и ``speak`` (озвучка). Решение «говорить ли и
что» - в ``rob_box_dialog.degrade`` (код, а не LLM, ADR-0148): класс причины приходит из ``HealthCache``, текст -
из ``phrases/ru.yaml``.
"""

from __future__ import annotations

import time
from typing import Any, Callable, List, Optional

from rob_box_dialog.degrade import ProviderAnnouncer, ProviderState


class ProviderStateReporter:
    """Опрашивает ``llm.snapshot()`` (``HealthAwareFallbackLLM``), публикует изменения, озвучивает отказ."""

    def __init__(
        self,
        llm: Any,
        publish: Callable[[List[ProviderState]], None],
        speak: Callable[[str], None],
        *,
        clock: Callable[[], float] = time.time,
        announcer: Optional[ProviderAnnouncer] = None,
    ) -> None:
        self._llm = llm
        self._publish = publish
        self._speak = speak
        self._clock = clock
        self._announcer = announcer or ProviderAnnouncer()
        self._last: Optional[tuple] = None

    @property
    def enabled(self) -> bool:
        """Одиночный провайдер (без цепочки) снимка не даёт - сообщать нечего."""
        return callable(getattr(self._llm, "snapshot", None))

    def tick(self) -> None:
        if not self.enabled:
            return
        snap = self._llm.snapshot()
        states = [ProviderState(s["name"], "llm", s["health"], s["since"], s["last_error_code"] or None) for s in snap]
        priority = [s["name"] for s in snap]  # порядок цепочки = приоритет из llm_providers
        signature = tuple((s.name, s.health) for s in states)
        if signature != self._last:
            self._last = signature
            self._publish(states)
        for text in self._announcer.observe(states, priority, self._clock()):
            self._speak(text)


def publish_provider_states(pub: Any, states: List[ProviderState]) -> None:
    """Снимок в ``ProviderStateArray`` (since - wall-clock секунды); сообщения импортируются лениво (стабы тестов)."""
    from rob_box_dialog_msgs.msg import ProviderState as Msg  # noqa: PLC0415
    from rob_box_dialog_msgs.msg import ProviderStateArray  # noqa: PLC0415

    arr = ProviderStateArray()
    for st in states:
        m = Msg()
        m.name, m.kind, m.health = st.name, st.kind, st.health
        m.since.sec = int(st.since)
        m.since.nanosec = int((st.since % 1) * 1e9)
        m.last_error_code = st.last_error_code or ""
        arr.providers.append(m)
    pub.publish(arr)
