"""План сета ``SetPlan`` и ``seeded_plan(profile, seed)`` без LLM (ADR-0149 §4.4–§4.6; PR-3b).

* **Один темп на сет** (§4.4, решение Шифу В6): ``SetPlan.bpm`` — темп профиля темы в окне стиля
  (``knowledge.STYLES``, club 128–138). У треков своего темпа нет: ``compose`` берёт его из плана.
* **Дуга энергии** (ADR-0147 §3.4, встроена по §4.6): волна ``knowledge.ENERGY_WAVE`` по номеру трека —
  сет открытый, длина заранее не известна, поэтому волна повторяется.
* **Ход тоники** (§8.1: перенос ``dj_set_walk.related_root``): чистая квинта вверх на трек — соседи по кругу
  квинт делят 6 из 7 нот (Camelot +1), за 12 треков все 12 тоник. Старый ``dj_set_walk`` импортирует
  :func:`root_shift` отсюда — одна реализация.
* **Свинг сета** — в окне стиля, от сида; один на весь сет, как грув у диджея.
* **Тоника сета** (PR-3d, ось «тоника» ``music_history``): тоника темы, если её не было в последних
  ``TONIC_MEMORY`` треках истории; иначе — выбор сидом (``diversity.weighted_pick``) среди тоник, которых там не
  было. Темп сета от истории не зависит.

Уточнение плана от LLM по JSON-схеме — ``reasoner`` (PR-10): ``reasoner.apply`` меняет ``tracks``, не темп.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, replace
from typing import Mapping, Sequence, Tuple

from . import knowledge as kn
from .diversity import recent_values, weighted_pick
from .theme import ThemeProfile

#: Шаг тоники между соседними треками: чистая квинта вверх, полутонов.
FIFTH = 7
DEFAULT_TRACKS = 10
#: Сколько последних треков истории не должна повторять тоника нового сета.
TONIC_MEMORY = 4


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
        """План трека ``no`` (с 1): из ``tracks`` (поправка LLM, PR-10), за их пределами — волна: сет открытый."""
        return self.tracks[no - 1] if 1 <= no <= len(self.tracks) else track_plan(no)

    def root(self, no: int) -> int:
        """Тоника трека ``no``, pitch class 0..11."""
        return (self.profile.root + self.track(no).root_shift) % 12

    @property
    def style(self) -> str:
        """Ключ ``knowledge.STYLES`` сета (ADR-0153: один стиль на сет) — стиль профиля темы."""
        return self.profile.style


def track_energy(no: int) -> int:
    """Энергия трека ``no`` (с 1) по волне ``knowledge.ENERGY_WAVE``."""
    return kn.ENERGY_WAVE[(max(1, no) - 1) % len(kn.ENERGY_WAVE)]


def root_shift(no: int) -> int:
    """Сдвиг тоники трека ``no`` (с 1) от тоники сета: трек 1 — 0, каждый следующий — квинта выше."""
    return FIFTH * (max(1, no) - 1) % 12


def track_plan(no: int) -> TrackPlan:
    return TrackPlan(no, track_energy(no), root_shift(no))


def set_root(profile: ThemeProfile, seed: int, history: Sequence[Mapping] = ()) -> int:
    """Тоника сета: тоника темы, если её не было в последних ``TONIC_MEMORY`` треках, иначе — сидом из остальных."""
    recent = recent_values(history[:TONIC_MEMORY], "root")
    if kn.ROOTS[profile.root] not in recent:
        return profile.root
    options = [r for r in kn.ROOTS if r not in recent]
    return kn.ROOTS.index(weighted_pick(options, recent, random.Random(f"root:{seed}:{profile.theme}")))


def seeded_plan(profile: ThemeProfile, seed: int, n_tracks: int = DEFAULT_TRACKS, set_id: str = "v2",
                history: Sequence[Mapping] = ()) -> SetPlan:
    """План сета мгновенно, без сети и LLM: детерминирован по ``(profile, seed, history)``; ``history`` — строки
    ``music_history`` (свежие первыми)."""
    profile = replace(profile, root=set_root(profile, seed, history))
    window = kn.STYLES[profile.style]
    lo, hi = window.bpm
    bpm = min(max(profile.bpm, lo), hi)
    s_lo, s_hi = window.swing
    swing = round(s_lo + random.Random(f"plan:{seed}:{profile.theme}").random() * (s_hi - s_lo), 3)
    tracks = tuple(track_plan(no) for no in range(1, max(1, n_tracks) + 1))
    return SetPlan(set_id, seed, profile, bpm, swing, tracks)


__all__ = ["DEFAULT_TRACKS", "FIFTH", "SetPlan", "TONIC_MEMORY", "TrackPlan", "root_shift", "seeded_plan", "set_root",
           "track_energy", "track_plan"]
