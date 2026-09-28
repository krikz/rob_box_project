"""Ограничитель частоты кадров per session × topic (issue #3150).

Оператор капитанского мостика может сидеть не в LAN, а в интернете:
SUBSCRIBE несёт необязательный ``max_hz`` — «не чаще N кадров в секунду
на этот поток». Сервер режет лишнее у себя, до ``ws.send_bytes``, чтобы
не забивать чужой канал кадрами, которые клиент всё равно не успеет
показать.

Семантика «последний кадр не теряется» (важно для событийных потоков —
``voice_state``, ``map_2d``, ``robot_status``): ранний кадр не
выбрасывается, а кладётся в единственный слот ожидания. Более свежий
ранний кадр вытесняет старый (старый считается отброшенным). Когда
интервал истёк — слот выталкивается (через ``loop.call_later`` в
ws_server). Так поток с редкими событиями доставит последнее состояние,
а частый видеопоток будет прорежен до ``max_hz``.

Чистая логика, без aiohttp/ROS: ``broadcast_frame`` зовётся из
ROS-потоков, ``flush`` — из aiohttp-loop, поэтому всё под ``Lock``.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Any, Optional

# Выше 120 Гц ограничение бессмысленно (ни один поток мостика так часто
# не публикует) — трактуем как «без лимита», чтобы опечатка клиента
# не превращалась в неожиданное поведение.
MAX_HZ_CEILING = 120.0


def parse_max_hz(raw: Any) -> Optional[float]:
    """Нормализовать ``max_hz`` из SUBSCRIBE-payload.

    Returns: частота (Гц) или ``None`` = без ограничения. ``None`` дают:
    отсутствие поля, bool, не-число, NaN/inf, ``<= 0``, ``> 120``.
    """
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        return None
    value = float(raw)
    if not math.isfinite(value) or value <= 0.0 or value > MAX_HZ_CEILING:
        return None
    return value


@dataclass
class _TopicState:
    interval_s: float
    last_sent: Optional[float] = None
    pending: Any = None
    has_pending: bool = False
    flush_scheduled: bool = False
    sent: int = 0
    dropped: int = 0


@dataclass(frozen=True)
class OfferResult:
    """Решение по кадру.

    * ``send_now`` — отправлять немедленно.
    * ``flush_in_s`` — кадр отложен; вызывающий ДОЛЖЕН запланировать
      :meth:`StreamRateLimiter.flush` через столько секунд (приходит
      только когда flush ещё не запланирован).
    """

    send_now: bool
    flush_in_s: Optional[float] = None


class StreamRateLimiter:
    """Лимитер одной сессии: topic → минимальный интервал между кадрами."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._topics: dict[str, _TopicState] = {}

    def set_limit(self, topic: str, max_hz: Optional[float]) -> None:
        """Задать/снять лимит. ``None`` — без лимита (слот ожидания сброшен)."""
        with self._lock:
            if max_hz is None:
                self._topics.pop(topic, None)
                return
            state = self._topics.get(topic)
            if state is None:
                self._topics[topic] = _TopicState(interval_s=1.0 / max_hz)
            else:
                state.interval_s = 1.0 / max_hz

    def remove(self, topic: str) -> None:
        """UNSUBSCRIBE: забыть topic вместе с отложенным кадром."""
        self.set_limit(topic, None)

    def limit_hz(self, topic: str) -> Optional[float]:
        with self._lock:
            state = self._topics.get(topic)
            return None if state is None else 1.0 / state.interval_s

    def offer(self, topic: str, item: Any, now: float) -> OfferResult:
        """Предложить кадр. Без лимита — всегда ``send_now``."""
        with self._lock:
            state = self._topics.get(topic)
            if state is None:
                return OfferResult(send_now=True)
            wait = self._wait_s(state, now)
            if wait <= 0.0 and not state.has_pending:
                state.last_sent = now
                state.sent += 1
                return OfferResult(send_now=True)
            if state.has_pending:
                state.dropped += 1
            state.pending = item
            state.has_pending = True
            if state.flush_scheduled:
                return OfferResult(send_now=False)
            state.flush_scheduled = True
            return OfferResult(send_now=False, flush_in_s=max(wait, 0.0))

    def flush(self, topic: str, now: float) -> tuple[Any, Optional[float]]:
        """Вытолкнуть отложенный кадр.

        Returns: ``(item, reschedule_in_s)``. ``item`` — кадр к отправке
        (``None`` если слот пуст / topic снят). ``reschedule_in_s`` —
        интервал ещё не истёк (лимит ужесточили): повторить flush позже.
        """
        with self._lock:
            state = self._topics.get(topic)
            if state is None or not state.has_pending:
                if state is not None:
                    state.flush_scheduled = False
                return None, None
            wait = self._wait_s(state, now)
            if wait > 0.0:
                return None, wait
            item = state.pending
            state.pending = None
            state.has_pending = False
            state.flush_scheduled = False
            state.last_sent = now
            state.sent += 1
            return item, None

    def stats(self, topic: str) -> tuple[int, int]:
        """(sent, dropped) по лимитированному topic; (0, 0) если лимита нет."""
        with self._lock:
            state = self._topics.get(topic)
            return (0, 0) if state is None else (state.sent, state.dropped)

    @staticmethod
    def _wait_s(state: _TopicState, now: float) -> float:
        if state.last_sent is None:
            return 0.0
        return state.last_sent + state.interval_s - now
