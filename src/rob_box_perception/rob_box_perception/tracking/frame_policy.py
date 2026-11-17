"""Выбор фрейма трекера: ``map``, если он есть, иначе ``base_link`` (ADR-0161 §1).

Решение Шифу 04.11: «следить относительно себя». Когда RTAB-Map не
публикует ``map`` (у наблюдений ``map_status != ok``), трекер ведёт треки
в ``base_link``; появился ``map`` — переходит в ``map``.

Переход явный, тихой подмены нет: :meth:`FramePolicy.observe` возвращает
новый фрейм ровно на том пакете, где решено переключиться, и вызывающий
обязан вызвать :meth:`Tracker.reset` — все треки старого фрейма удаляются
с событием ``deleted``, новые рождаются в новом фрейме с новыми id.
Пересчёта треков между фреймами нет (почему — ADR-0161 §1).

Без дребезга:

* ``base_link`` → ``map``: ``map_confirm_batches`` пакетов подряд с ``map``;
* ``map`` → ``base_link``: ``map`` нет дольше ``map_loss_sec``. Пока
  потеря короче, фрейм остаётся ``map``, а наблюдения без ``map`` трекеру
  не отдаются (``to_detection`` вернёт причину, счётчик — в узле).

Пустой пакет о ``map`` ничего не говорит — его в политику не подают.
"""

from __future__ import annotations

from typing import Optional

from rob_box_perception.tracking.params import TrackerParams

MAP = 'map'
BASE_LINK = 'base_link'


class FramePolicy:
    """Текущий фрейм трекера и правило его смены."""

    def __init__(self, params: Optional[TrackerParams] = None) -> None:
        """Фрейм ещё не выбран: выберет первый пакет."""
        self.params = params or TrackerParams()
        self.frame: Optional[str] = None
        self._map_streak = 0
        self._map_lost_since: Optional[float] = None

    def observe(self, stamp: float, map_ok: bool) -> Optional[str]:
        """Учесть пакет; вернуть новый фрейм, если надо переключиться.

        ``map_ok`` — у наблюдений пакета точка в ``map`` есть
        (``map_status == ok``). Возврат ``None`` — фрейм прежний.
        Первый пакет всегда возвращает выбранный фрейм.
        """
        self._map_streak = self._map_streak + 1 if map_ok else 0
        if map_ok:
            self._map_lost_since = None
        elif self._map_lost_since is None:
            self._map_lost_since = stamp
        if self.frame is None:
            return self._switch(MAP if map_ok else BASE_LINK)
        if self.frame == BASE_LINK and \
                self._map_streak >= self.params.map_confirm_batches:
            return self._switch(MAP)
        if self.frame == MAP and self._map_lost_since is not None and \
                stamp - self._map_lost_since > self.params.map_loss_sec:
            return self._switch(BASE_LINK)
        return None

    def _switch(self, frame: str) -> str:
        """Запомнить новый фрейм и вернуть его."""
        self.frame = frame
        return frame
