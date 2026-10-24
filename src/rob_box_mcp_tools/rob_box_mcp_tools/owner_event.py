"""owner_event - ожидание события владельца ресурса по id запроса (ADR-0150 §5.2-5.3, PR-7 ч.7b, #3698).

Тул не знает, вышло ли действие, пока владелец ресурса не ответил: голос - ``tts_node`` (``SpeechEvent
voice_changed``), звук - ``sound_node`` (``SoundEvent``), голос собеседника - ``speaker_id_node`` (``SpeakerResult``
registered/register_error по ``utterance_id``, #3793). Здесь один механизм на все такие тулы: подписка на топик
событий владельца (одна на ноду и топик), запрос с новым ``request_id`` (или с id, который владелец и так вернёт),
ожидание первого подходящего события с тем же id. Нет события за срок - ``None``: тул честно говорит «не дождался»,
а не «готово» (ADR-0148).
"""

from __future__ import annotations

import threading
import uuid
from collections import OrderedDict
from typing import Any, Callable, Dict, List, Optional, Tuple

Publish = Callable[[str], None]


class OwnerEvents:
    """Ожидания событий одного топика владельца: ``request_id`` → первое событие с этим id.

    ``remember`` > 0 - последние события по id хранятся: запрос мог отправить не тул (регистрацию по представлению
    шлёт ``dialogue_node`` до вызова LLM, #3793), и его исход пришёл раньше, чем тул начал ждать.
    """

    def __init__(self, node: Any, msg_type: Any, topic: str, key_of: Callable[[Any], str],
                 accept: Callable[[Any], bool] = lambda _msg: True, depth: int = 20, remember: int = 0) -> None:
        self._key_of, self._accept, self._remember = key_of, accept, remember
        self._lock = threading.Lock()
        self._waiters: Dict[str, Tuple[threading.Event, List[Any]]] = {}
        self._recent: "OrderedDict[str, Any]" = OrderedDict()
        node.create_subscription(msg_type, topic, self._on_event, depth)

    def _on_event(self, msg: Any) -> None:
        if not self._accept(msg):
            return
        key = self._key_of(msg)
        with self._lock:
            if self._remember:
                self._recent[key] = msg
                self._recent.move_to_end(key)
                while len(self._recent) > self._remember:
                    self._recent.popitem(last=False)
            waiter = self._waiters.get(key)
            if waiter is None or waiter[1]:
                return  # чужой запрос или уже есть первое событие
            waiter[1].append(msg)
        waiter[0].set()

    def ask(self, publish: Optional[Publish], wait_s: float,
            request_id: Optional[str] = None) -> Tuple[str, Optional[Any]]:
        """Отправить запрос (``publish(request_id)``) и ждать события; ``(request_id, событие | None)``.

        ``request_id`` - id, который владелец вернёт в событии сам (иначе - новый). ``publish=None`` - запрос уже
        отправил другой: исход берётся из запомненных, если успел прийти, иначе ждём его."""
        rid = request_id or uuid.uuid4().hex
        waiter: Tuple[threading.Event, List[Any]] = (threading.Event(), [])
        with self._lock:
            known = self._recent.pop(rid, None)
            if publish is None and known is not None:
                return rid, known
            self._waiters[rid] = waiter
        try:
            if publish is not None:
                publish(rid)
            waiter[0].wait(wait_s)
        finally:
            with self._lock:
                self._waiters.pop(rid, None)
        return rid, (waiter[1][0] if waiter[1] else None)


def owner_events(node: Any, name: str, make: Callable[[], OwnerEvents]) -> OwnerEvents:
    """Одни ожидания на ноду и вид запроса ``name`` (подписка создаётся при первом запросе, ``make``)."""
    cache = node.__dict__.setdefault("_owner_events", {})
    if name not in cache:
        cache[name] = make()
    return cache[name]
