"""ROS-подписки на тяжёлые стримы — только пока их смотрит шлем.

Камеры на Vision Pi публикуются лениво (lazy publisher у OAK-D и
image_transport): пока есть хоть один подписчик, драйвер кодирует кадры.
Постоянная подписка quest_node держала OAK-D и потолочную камеру включёнными
без подключённого шлема — ~90% CPU в oak-d и ~10 МБ/с через zenoh.

``DemandDrivenSubscriptions.tick()`` зовётся ROS-таймером: если хоть одна
WS-сессия подписана на ui-стрим — держим ROS-подписку, если нет дольше
``linger_s`` — снимаем.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import Any

log = logging.getLogger(__name__)


class DemandDrivenSubscriptions:
    def __init__(
        self,
        node: Any,
        factories: dict[str, Callable[[], Any]],
        has_demand: Callable[[str], bool],
        linger_s: float = 5.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._node = node
        self._factories = dict(factories)
        self._has_demand = has_demand
        self._linger_s = linger_s
        self._clock = clock
        self._subs: dict[str, Any] = {}
        self._last_demand: dict[str, float] = {}

    def active(self, ui_name: str) -> Any | None:
        return self._subs.get(ui_name)

    def tick(self) -> None:
        now = self._clock()
        for ui_name, factory in self._factories.items():
            if self._has_demand(ui_name):
                self._last_demand[ui_name] = now
                if ui_name not in self._subs:
                    self._subs[ui_name] = factory()
                    log.info("quest: %s — ROS-подписка создана (есть зритель)", ui_name)
                continue
            sub = self._subs.get(ui_name)
            if sub is None:
                continue
            if now - self._last_demand.get(ui_name, now) > self._linger_s:
                self._node.destroy_subscription(self._subs.pop(ui_name))
                log.info("quest: %s — ROS-подписка снята (зрителей нет)", ui_name)
