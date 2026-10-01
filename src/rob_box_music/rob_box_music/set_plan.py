"""План сета ``SetPlan`` и ``seeded_plan(profile, seed)`` без LLM (ADR-0149 §4.4–§4.6; PR-3b).

* **Один темп на сет** (§4.4, решение Шифу В6): ``SetPlan.bpm`` — темп профиля темы в окне жанра
  (club 128–138). У треков своего темпа нет: ``compose`` берёт его из плана.
* **Дуга энергии** (ADR-0147 §3.4, встроена по §4.6): волна ``knowledge.ENERGY_WAVE`` по номеру трека —
  сет открытый, длина заранее не известна, поэтому волна повторяется.
* **Ход тоники** (§8.1: перенос ``dj_set_walk.related_root``): чистая квинта вверх на трек — соседи по кругу
  квинт делят 6 из 7 нот (Camelot +1), за 12 треков все 12 тоник. Старый ``dj_set_walk`` импортирует
  :func:`root_shift` отсюда — одна реализация.
* **Свинг сета** — в окне жанра, от сида; один на весь сет, как грув у диджея.

Уточнение плана от LLM по JSON-схеме (``schema``/``validate_plan``) — PR-10.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Tuple

from . import knowledge as kn
from .theme import ThemeProfile

#: Шаг тоники между соседними треками: чистая квинта вверх, полутонов.
FIFTH = 7
DEFAULT_TRACKS = 10


@dataclass(frozen=True)
class TrackPlan:
    no: int  # с 1
    energy: int  # 1..5
    root_shift: int  # полутонов от тоники сета, 0..11


@dataclass(frozen=True)
class SetPlan:
    set_id: str
    seed: int
    profile: ThemeProfile  # тема, лад, хуки, тоника сета
    bpm: int  # единственный темп сета
    swing: float  # доля восьмой, на которую опаздывают нечётные 16-е хэтов
    tracks: Tuple[TrackPlan, ...]  # первые треки; дальше — :meth:`track`

    def track(self, no: int) -> TrackPlan:
        """План трека ``no`` (с 1) — и за пределами ``tracks``: сет открытый."""
        return track_plan(no)

    def root(self, no: int) -> int:
        """Тоника трека ``no``, pitch class 0..11."""
        return (self.profile.root + self.track(no).root_shift) % 12


def track_energy(no: int) -> int:
    """Энергия трека ``no`` (с 1) по волне ``knowledge.ENERGY_WAVE``."""
    return kn.ENERGY_WAVE[(max(1, no) - 1) % len(kn.ENERGY_WAVE)]


def root_shift(no: int) -> int:
    """Сдвиг тоники трека ``no`` (с 1) от тоники сета: трек 1 — 0, каждый следующий — квинта выше."""
    return FIFTH * (max(1, no) - 1) % 12


def track_plan(no: int) -> TrackPlan:
    return TrackPlan(no, track_energy(no), root_shift(no))


def seeded_plan(profile: ThemeProfile, seed: int, n_tracks: int = DEFAULT_TRACKS, set_id: str = "v2") -> SetPlan:
    """План сета мгновенно, без сети и LLM: детерминирован по ``(profile, seed)``."""
    window = kn.GENRE_WINDOWS[profile.genre]
    lo, hi = window.bpm
    bpm = min(max(profile.bpm, lo), hi)
    s_lo, s_hi = window.swing
    swing = round(s_lo + random.Random(f"plan:{seed}:{profile.theme}").random() * (s_hi - s_lo), 3)
    tracks = tuple(track_plan(no) for no in range(1, max(1, n_tracks) + 1))
    return SetPlan(set_id, seed, profile, bpm, swing, tracks)


__all__ = ["DEFAULT_TRACKS", "FIFTH", "SetPlan", "TrackPlan", "root_shift", "seeded_plan", "track_energy",
           "track_plan"]
