"""Тик recovery-BackUp по копии costmap_raw (#5003). Решение — код, не лог.

HIL 22.11, стенд dead_end (``/tmp/hil_5003_1122/nav2.log``): ``Running backup``,
и через 1–20 мс behavior_server сам пишет ``Collision Ahead - Exiting
DriveOnHeading``. Колёса не крутятся. Копия ``local_costmap/costmap_raw``
публикуется на 2 Гц (``publish_frequency: 2.0``), очистка отвечает сразу, так
что тик в десятки миллисекунд после clear читает копию ещё со стеной.

Humble ``BackUp`` наследует ``DriveOnHeading``, но CMake собирает его в свой
``libnav2_back_up_behavior.so`` (``plugins/back_up.cpp`` включает заголовок
``drive_on_heading.hpp``). Правка только этого заголовка попадает в
``libnav2_drive_on_heading_behavior.so`` и BackUp её не видит: в том же логе
``Costmap copy stale`` — 0 строк, ``Collision Ahead`` на BackUp — 4.

Решение живёт здесь и в ``behavior_server`` (одна библиотека на оба плагина,
ADR-0148). Плагин только спрашивает, что делать с тиком.
"""

from __future__ import annotations

# Сколько тиков behavior_server (cycle_frequency 10 Гц) можно пропустить, пока
# копия старше допуска. Три тика покрывают период публикации 0.5 с один раз.
# Четвёртый тик без свежей копии — отказ: мёртвая карта не должна отпускать ход.
MAX_STALE_COSTMAP_SKIPS = 3

CHECK = "check"
SKIP = "skip"
FAIL = "fail"


class CostmapTickGate:
    """Что делать тику: решать столкновение, пропустить или закрыться отказом."""

    def __init__(self, max_skips: int = MAX_STALE_COSTMAP_SKIPS) -> None:
        self.max_skips = max_skips
        self.stale_skips = 0

    def action(self, copy_fresh: bool) -> str:
        """``copy_fresh`` — копия не старше допуска из конфига.

        Свежая копия сбрасывает счётчик. Устаревшая пропускает тик, пока
        счётчик не вышел за ``max_skips``; дальше отказ.
        """
        if copy_fresh:
            self.stale_skips = 0
            return CHECK
        self.stale_skips += 1
        if self.stale_skips <= self.max_skips:
            return SKIP
        return FAIL


def backup_tick(action: str, collision_ahead: bool) -> str:
    """Исход одного тика BackUp. ``collision_ahead`` — ответ проверки по копии.

    ``skip`` не смотрит в копию: устаревшая стена не абортит ход.
    ``fail`` — отказ, даже если копия пустая. ``check`` решает как раньше.
    """
    if action == SKIP:
        return "running"
    if action == FAIL:
        return "failed"
    if collision_ahead:
        return "collision_ahead"
    return "running"
