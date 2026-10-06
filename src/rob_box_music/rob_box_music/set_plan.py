"""План сета ``SetPlan`` и ``seeded_plan(profile, seed)`` без LLM (ADR-0149 §4.4–§4.6; PR-3b).

* **Один темп на сет** (§4.4, решение Шифу В6): ``SetPlan.bpm`` — темп профиля темы в окне стиля
  (``knowledge.STYLES``, club 128–138). У треков своего темпа нет: ``compose`` берёт его из плана.
* **Дуга энергии** (ADR-0147 §3.4, встроена по §4.6): волна ``knowledge.ENERGY_WAVE`` по номеру трека —
  сет открытый, длина заранее не известна, поэтому волна повторяется.
* **Ход тоники** (§8.1: перенос ``dj_set_walk.related_root``): чистая квинта вверх на трек — соседи по кругу
  квинт делят 6 из 7 нот (Camelot +1), за 12 треков все 12 тоник. Старый ``dj_set_walk`` импортирует
  :func:`root_shift` отсюда — одна реализация.
* **Жанровое окно** (ADR-0152 §3.5, PR-8, В1): ``SetPlan.genre`` — окно клуба (``Style.genre_windows``: club/deep/
  breaks) выбирается на сет сидом со штрафом за окна прошлых сетов (:func:`pick_genre`); темп сета — в окне
  (:func:`plan_bpm`), пул бочек и пэдов, рисунок бочки дропа — из окна (``knowledge.genre_style``). Внутри сета окно
  не меняется. Окно — не стиль (ADR-0153): стиль — набор таблиц, окно — подвыбор внутри него.
* **Свинг сета** — в окне стиля, от сида; один на весь сет, как грув у диджея.
* **Тоника сета** (PR-3d, ось «тоника» ``music_history``): тоника темы, если её не было в последних
  ``TONIC_MEMORY`` треках истории; иначе — выбор сидом (``diversity.weighted_pick``) среди тоник, которых там не
  было. Темп сета от истории не зависит.

Уточнение плана от LLM по JSON-схеме — ``reasoner`` (PR-10): ``reasoner.apply`` меняет ``tracks``, не темп.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, replace
from typing import Mapping, Optional, Sequence, Tuple

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
    kick: str = ""  # бочка из пула стиля (``knowledge.KICK_SOUNDS``); пусто — ``compose`` выбирает по истории сам
    template: str = ""  # форма трека (ключ ``Style.forms``); пусто — ``compose`` выбирает по энергии и истории сам


@dataclass(frozen=True)
class SetPlan:
    set_id: str
    seed: int
    profile: ThemeProfile  # тема, лад, хуки, тоника сета
    bpm: int  # единственный темп сета
    swing: float  # доля восьмой, на которую опаздывают нечётные 16-е хэтов
    tracks: Tuple[TrackPlan, ...]  # первые треки; дальше — :meth:`track`
    genre: str = kn.DEFAULT_GENRE  # жанровое окно клуба сета (``Style.genre_windows``), не меняется внутри сета

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

    @property
    def table(self) -> kn.Style:
        """Таблицы стиля сета в его жанровом окне: то, что читают ``arrange/*`` (``knowledge.genre_style``)."""
        return kn.genre_style(kn.STYLES[self.style], self.genre)


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


def recent_genres(history: Sequence[Mapping]) -> list:
    """Окна прошлых СЕТОВ, свежие первыми: строки истории (свежие первыми) свёрнуты по ``set_id`` (у сета одно окно);
    строки без окна (записаны до PR-8) пропускаются."""
    out: list = []
    last = object()
    for row in history:
        genre, set_id = row.get("genre"), row.get("set_id")
        if not genre or (out and set_id == last):
            continue
        out.append(genre)
        last = set_id
    return out


def pick_genre(style: kn.Style, history: Sequence[Mapping], rng: random.Random) -> str:
    """Окно сета: сид + штраф за окна прошлых сетов (``weighted_pick``, ось ``genre``); с прошлым сетом подряд одно
    окно не повторяется."""
    recent = recent_genres(history)
    windows = tuple(style.genre_windows)
    options = [g for g in windows if not recent or g != recent[0]] or list(windows)
    return weighted_pick(options, recent, rng)


def plan_bpm(window: kn.GenreWindow, theme_bpm: int, seed: int, theme: str) -> int:
    """Темп сета: темп темы, если он в окне; иначе — сидом внутри окна (один темп на сет, ADR-0149 I7)."""
    lo, hi = window.bpm
    return theme_bpm if lo <= theme_bpm <= hi else random.Random(f"bpm:{seed}:{theme}").randint(lo, hi)


def pick_kick(style: kn.Style, history: Sequence[Mapping], rng: random.Random) -> str:
    """Бочка трека из пула стиля: сид + штраф за недавние в истории (``weighted_pick``, ось ``kick``), с прошлым
    треком подряд не повторяется."""
    recent = recent_values(history, "kick")
    options = [k for k in style.kick_pool if not recent or k != recent[0]] or list(style.kick_pool)
    return weighted_pick(options, recent, rng)


def plan_kicks(style: kn.Style, seed: int, theme: str, n_tracks: int,
               history: Sequence[Mapping] = ()) -> Tuple[str, ...]:
    """Бочки первых ``n_tracks`` треков сета: каждая выбрана со штрафом за прошлые сеты и предыдущие треки плана."""
    recent = [k for k in recent_values(history, "kick") if k]
    kicks = []
    for no in range(1, n_tracks + 1):
        rows = [{"kick": k} for k in kicks[::-1] + recent]
        kicks.append(pick_kick(style, rows, random.Random(f"kick:{seed}:{theme}:{no}")))
    return tuple(kicks)


def pick_template(style: kn.Style, energy: int, history: Sequence[Mapping], rng: random.Random) -> str:
    """Форма трека энергии ``energy`` из ``Style.energy_forms``: сид + штраф за недавние (ось ``template``), с
    прошлым треком подряд не повторяется."""
    recent = recent_values(history, "template")
    allowed = style.energy_forms.get(energy) or tuple(style.forms)
    options = [t for t in allowed if not recent or t != recent[0]] or list(allowed)
    return weighted_pick(options, recent, rng)


def plan_templates(style: kn.Style, seed: int, theme: str, energies: Sequence[int],
                   history: Sequence[Mapping] = ()) -> Tuple[str, ...]:
    """Формы первых треков сета (энергия — по номеру): трек 1 — ``Style.opening_form`` (блэнда на входе нет, тему
    человек ждёт сразу, #3427), остальные — :func:`pick_template` со штрафом за прошлые сеты и предыдущие треки."""
    recent = [t for t in recent_values(history, "template") if t]
    out = []
    for no, energy in enumerate(energies, 1):
        rows = [{"template": t} for t in out[::-1] + recent]
        out.append(style.opening_form if no == 1
                   else pick_template(style, energy, rows, random.Random(f"template:{seed}:{theme}:{no}")))
    return tuple(out)


def seeded_plan(profile: ThemeProfile, seed: int, n_tracks: int = DEFAULT_TRACKS, set_id: str = "v2",
                history: Sequence[Mapping] = (), genre: Optional[str] = None) -> SetPlan:
    """План сета мгновенно, без сети и LLM: детерминирован по ``(profile, seed, history)``; ``history`` — строки
    ``music_history`` (свежие первыми). ``genre`` — окно, заданное явно (тема, оператор);
    ``None`` — :func:`pick_genre`."""
    profile = replace(profile, root=set_root(profile, seed, history))
    base = kn.STYLES[profile.style]
    genre = pick_genre(base, history, random.Random(f"genre:{seed}:{profile.theme}")) if genre is None else genre
    window = kn.genre_style(base, genre)
    bpm = plan_bpm(base.genre_windows[genre], profile.bpm, seed, profile.theme)
    s_lo, s_hi = window.swing
    swing = round(s_lo + random.Random(f"plan:{seed}:{profile.theme}").random() * (s_hi - s_lo), 3)
    n = max(1, n_tracks)
    kicks = plan_kicks(window, seed, profile.theme, n, history)
    plans = [track_plan(no) for no in range(1, n + 1)]
    forms = plan_templates(window, seed, profile.theme, [p.energy for p in plans], history)
    tracks = tuple(replace(p, kick=kicks[p.no - 1], template=forms[p.no - 1]) for p in plans)
    return SetPlan(set_id, seed, profile, bpm, swing, tracks, genre)


__all__ = ["DEFAULT_TRACKS", "FIFTH", "SetPlan", "TONIC_MEMORY", "TrackPlan", "pick_genre", "pick_kick",
           "pick_template", "plan_bpm", "plan_kicks", "plan_templates", "recent_genres", "root_shift", "seeded_plan",
           "set_root", "track_energy", "track_plan"]
