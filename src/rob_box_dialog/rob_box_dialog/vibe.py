"""Вайб лица — катящийся шар (issue #3780 B, ADR-0148).

Шифу 21.10: вайб — шар, который катится от окружения и плавно переходит в другой, или в нейтральный,
если ничего значимого не происходит. Шифу 05.11: решает код по событиям; без новых событий шар сам
возвращается к обычному за 5–15 минут; модель только подсказывает, редко.

Один модуль состояния. Событие толкает шар. Без событий шар за :data:`.knowledge.VIBE_DECAY_S` доезжает
до нейтрали. Числа кривой — только в ``knowledge`` (инерция, скорость дрейфа, время затухания). Пиксели
радуги и сцена сна сюда не входят: это части A и C той же карточки.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Mapping, Optional

from . import knowledge as kn

#: Нейтраль: шар в покое, лицо обычное. Не стиль и не сцена рендера.
NEUTRAL = "neutral"


@dataclass(frozen=True)
class Vibe:
    """Снимок шара: куда катится, насколько сдвинут от нейтрали, с какой скоростью.

    ``pull`` — 0 у нейтрали и 1 у стиля. ``speed`` — доля пути за секунду, со знаком к стилю.
    ``previous`` — стиль до последней смены; ``None`` — смены ещё не было (нет данных).
    ``since`` — монотонное время включения текущего; ``None`` — ещё не включали (нет данных).
    """

    style: str = NEUTRAL
    pull: float = 0.0
    speed: float = 0.0
    previous: Optional[str] = None
    since: Optional[float] = None

    def neutral(self) -> bool:
        """Шар у нейтрали: стиль обычный и сдвига нет."""
        return self.style == NEUTRAL and self.pull == 0.0 and self.speed == 0.0


def _style(name: Optional[str]) -> str:
    """Имя стиля. Пустое — не стиль. ``neutral`` — явный толчок домой, не «ничего»."""
    return (name or "").strip().lower()


def _kick(strength: float) -> float:
    """Сила толчка в долях потолка дрейфа. Чутьё и нечисло — ноль, больше единицы — единица."""
    strength = _finite(strength)
    if strength <= 0.0:
        return 0.0
    return min(1.0, strength) * kn.VIBE_DRIFT_PER_S


def _finite(value: float) -> float:
    return value if math.isfinite(value) else 0.0


def _coast(pull: float, speed: float, dt: float) -> tuple:
    """Где шар через ``dt`` без событий. Дома — ``(0, 0)``.

    Точное решение, не шаг интегратора. Скорость наружу садится экспонентой инерции.
    Остаток тает экспонентой за время затухания: через это время от толчка остаётся
    ``1/e``, порог :data:`.knowledge.VIBE_PULL_REST` уже считает шар нейтралью.
    Один скачок времени и пачка коротких дают одно место. Порог нейтрали не режет толчок, пока скорость наружу.
    """
    if dt <= 0.0:
        return max(0.0, pull), speed
    rate = kn.VIBE_INERTIA_PER_S
    decay = kn.VIBE_DECAY_S
    fade = math.exp(-dt / decay)
    grown = 0.0
    outward = 0.0
    if speed > 0.0 and rate > 0.0 and rate != 1.0 / decay:
        grown = speed * (fade - math.exp(-rate * dt)) / (rate - 1.0 / decay)
        outward = speed * math.exp(-rate * dt)
    landed = pull * fade + grown
    if landed >= 1.0:
        return 1.0, 0.0
    going = outward > kn.VIBE_SPEED_REST
    if landed <= kn.VIBE_PULL_REST and not going:
        return 0.0, 0.0
    if not going:
        return landed, -landed / decay
    return landed, min(outward, kn.VIBE_DRIFT_PER_S)


class VibeBall:
    """Владелец шара. :meth:`push` — событие, :meth:`at` — где шар в момент ``now``.

    Часы снаружи (``clock``): тест гоняет время сам. Между вызовами шар не тикает — положение
    считается из последнего толчка и прошедшего времени.
    """

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._at = clock()
        self._vibe = Vibe()
        #: Имя последнего принятого толчка (включая ``neutral``). Смена — только здесь.
        self._chosen = NEUTRAL
        #: Стиль до последней смены. ``None`` — смены ещё не было.
        self._previous: Optional[str] = None
        #: Монотонное время включения текущего. ``None`` — ещё не включали.
        self._since: Optional[float] = None

    def at(self, now: Optional[float] = None) -> Vibe:
        """Где шар сейчас. Без новых событий за время затухания — нейтраль."""
        moment = self._clock() if now is None else now
        self._roll(max(0.0, moment - self._at))
        self._at = moment
        return self._snapshot()

    def push(
        self, style: Optional[str], strength: float = 1.0, now: Optional[float] = None
    ) -> Vibe:
        """Событие толкает шар к стилю. Сила 0 — толчка нет. «neutral» — толчок обратно к обычному.

        Сила больше 1 режется до 1: явная просьба — сильный толчок, не сцена на несколько секунд.
        Смена стиля (новое имя ≠ текущему chosen, не пустое) запоминает предыдущий и время включения.
        «neutral» как толчок домой — тоже смена, если chosen был не neutral. Сила 0 и пустое — не смена.
        """
        landed = self.at(now)
        kick = _kick(strength)
        if kick == 0.0:
            return landed
        name = _style(style)
        if not name:
            return landed
        self._note_change(name, self._at)
        if name == NEUTRAL:
            self._vibe = Vibe(landed.style, landed.pull, landed.speed - kick)
            return self._snapshot()
        if landed.style not in (NEUTRAL, name):
            landed = Vibe(name, 0.0, 0.0)
        speed = min(kn.VIBE_DRIFT_PER_S, landed.speed + kick)
        self._vibe = Vibe(name, landed.pull, speed)
        return self._snapshot()

    def hint(
        self,
        style: Optional[str],
        now: Optional[float] = None,
        accepted: Optional[Mapping[str, float]] = None,
    ) -> Optional[Vibe]:
        """Подсказка модели. Редко: чаще :data:`.knowledge.VIBE_HINT_MIN_S` — код её не берёт.

        ``accepted`` — книга хоста «когда стиль уже принят» (монотонные секунды). Её ведёт хост, не модель:
        модуль только решает, пора ли. Первый раз стиль проходит. Нет книги — подсказка не толкает.
        """
        name = _style(style)
        if name == NEUTRAL or accepted is None:
            return None
        moment = self._clock() if now is None else now
        last = accepted.get(name)
        if last is not None and moment - last < kn.VIBE_HINT_MIN_S:
            return None
        accepted[name] = moment  # type: ignore[index]
        return self.push(name, kn.VIBE_HINT_STRENGTH, moment)

    def _roll(self, dt: float) -> None:
        vibe = self._vibe
        pull, speed = _coast(vibe.pull, vibe.speed, dt)
        self._vibe = (
            Vibe() if pull == 0.0 and speed == 0.0 else Vibe(vibe.style, pull, speed)
        )

    def _note_change(self, name: str, moment: float) -> None:
        """Если имя реально сменилось — запомнить предыдущий и время включения нового."""
        if name == self._chosen:
            return
        # До первой смены предыдущего не было: не подставляем стартовый neutral.
        if self._since is not None:
            self._previous = self._chosen
        self._chosen = name
        self._since = moment

    def _snapshot(self) -> Vibe:
        """Снимок физики плюс предыдущий стиль и время включения текущего."""
        vibe = self._vibe
        return Vibe(vibe.style, vibe.pull, vibe.speed, self._previous, self._since)


__all__ = ["NEUTRAL", "Vibe", "VibeBall"]
