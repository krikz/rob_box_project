#!/usr/bin/env python3
"""
camera_on_demand.py — подписка на камеру только пока просят кадр.

Постоянная подписка бота на три сжатых потока держала OAK-D и потолочную
камеру включёнными всегда (у драйверов lazy publisher): на Vision Pi это
~90% CPU в oak-d и ~10 МБ/с через zenoh даже когда фото никто не просит.
Теперь подписка создаётся на запрос фото и снимается через ``linger_s``
после последнего запроса.
"""

import logging
import threading
import time
from collections.abc import Callable
from typing import Any

from .camera_cache import CameraCache

logger = logging.getLogger(__name__)


class OnDemandCameraSubscriptions:
    """Лениво создаёт и снимает подписки на топики камер.

    ``request()`` блокирующий — handlers вызывают его через executor,
    ``reap()`` дёргает ROS-таймер ноды.
    """

    def __init__(
        self,
        node: Any,
        cache: CameraCache,
        msg_type: Any,
        qos: Any,
        callback_group: Any = None,
        linger_s: float = 10.0,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._node = node
        self._cache = cache
        self._msg_type = msg_type
        self._qos = qos
        self._callback_group = callback_group
        self._linger_s = linger_s
        self._clock = clock
        self._lock = threading.Lock()
        # topic -> (subscription, время последнего запроса)
        self._subs: dict[str, tuple[Any, float]] = {}

    @property
    def active_topics(self) -> list:
        with self._lock:
            return list(self._subs)

    def request(self, topic: str, timeout_s: float) -> bytes | None:
        """Вернуть свежий кадр, при необходимости подписавшись и подождав."""
        self._ensure(topic)
        frame = self._cache.get(topic)
        if frame is not None:
            return frame
        return self._cache.wait_for(topic, timeout_s)

    def reap(self) -> None:
        """Снять подписки, к которым не обращались дольше ``linger_s``."""
        now = self._clock()
        with self._lock:
            stale = [t for t, (_, last) in self._subs.items() if now - last > self._linger_s]
            subs = [self._subs.pop(t)[0] for t in stale]
        for topic, sub in zip(stale, subs):
            self._node.destroy_subscription(sub)
            logger.info("camera %s: подписка снята (нет запросов %.0f с)", topic, self._linger_s)

    def _ensure(self, topic: str) -> None:
        with self._lock:
            entry = self._subs.get(topic)
            if entry is not None:
                self._subs[topic] = (entry[0], self._clock())
                return
            sub = self._node.create_subscription(
                self._msg_type,
                topic,
                lambda m, t=topic: self._cache.update(t, bytes(m.data)),
                self._qos,
                callback_group=self._callback_group,
            )
            self._subs[topic] = (sub, self._clock())
        logger.info("camera %s: подписка создана по запросу", topic)
