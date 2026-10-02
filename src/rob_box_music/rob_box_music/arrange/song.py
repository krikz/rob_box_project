"""Classic-песня в модели v2: мелодия целиком по куплетам, аккомпанемент — выход ``harmonize`` (ADR-0149 §9 PR-11).

«Поставь Калинку»: хук — ВСЯ мелодия, куплет = один её проход, куплеты отличаются составом
(``knowledge.SONG_VERSES``: тема поверх пэда и баса → входят ударные → полный состав). Темп и тональность — из
мелодии (RTTTL), а не из club-окна. Ноты аккомпанемента здесь не сочиняются: бас, пэд и рисунки ударных — это
:class:`SongMaterial`, который собирает ``rob_box_mcp_tools.engine.classic`` из ``core.harmonize`` (библиотека с
10+ вшитыми фиксами, не переписывается; пакет без ROS её не импортирует). Здесь — только раскладка по форме,
тембры и уровни (``arrange.mix``).
"""

from __future__ import annotations

import hashlib
import random
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..model import (
    BEATS_PER_BAR, STEPS_PER_BAR, Form, Harmony, HistoryKey, Hook, Key, Part, PitchEvent, Section, Track, Transition,
    validate,
)
from . import mix, rhythm

#: Песня не стыкуется в DJ-сете; переход — только чтобы трек прошёл валидатор модели.
SONG_TRANSITION = Transition(8, 4, False)
SONG_ENERGY = 3
#: Ноты партии ``harmonize``: ``(midi | None, доли)``; у пэда вместо midi — кортеж аккорда.
Line = Sequence[Tuple[object, float]]


@dataclass(frozen=True)
class SongMaterial:
    """Мелодия и аккомпанемент одного прохода (куплета) — абсолютные MIDI и доли, как отдаёт ``harmonize``.

    ``drums``/``hats`` — рисунки на такт (16 шагов, ``knowledge.SONG_DRUM_SYMBOLS``), пустая строка — без них.
    ``pad_sus`` — длина удара аккорда пэда в долях; ``None`` — аккорд держится всю свою длительность.
    """

    melody_id: str
    title: str
    bpm: int
    root: int
    mode: str
    lead: Line
    bass: Line
    pad: Line
    pad_sus: Optional[float]
    drums: str
    hats: str

    @property
    def beats(self) -> float:
        return float(sum(d for _n, d in self.lead))


def verse_count(theme_bars: int) -> int:
    """Проходов мелодии в песне: около ``SONG_TARGET_BARS`` тактов, не больше строк ``SONG_VERSES``."""
    most = max(kn.SONG_VERSES)
    return max(1, min(most, round(kn.SONG_TARGET_BARS / max(theme_bars, 1))))


def _events(line: Line, offset: float, sus: Optional[float] = None) -> List[PitchEvent]:
    out: List[PitchEvent] = []
    beat = 0.0
    for notes, dur in line:
        chord = notes if isinstance(notes, tuple) else (notes,)
        length = min(float(sus), float(dur)) if sus else float(dur)
        out += [PitchEvent(int(m), offset + beat, length, 2) for m in chord if m is not None]
        beat += float(dur)
    return out


def _tonal(role: str, synth: str, line: Line, form: Form, theme_beats: float,
           sus: Optional[float] = None) -> Optional[Part]:
    events: List[PitchEvent] = []
    start = 0.0
    for sec in form.sections:
        if role in sec.roles:
            events += _events(line, start, sus)
        start += theme_beats
    if not events:
        return None
    pitches = [e.midi for e in events]
    return Part(role, synth, rhythm.grid(()), tuple(events), 0.0, (min(pitches), max(pitches)))


def _drums(material: SongMaterial) -> Dict[str, Part]:
    """Рисунки ``harmonize`` → сетки ролей (такт); бочка — сэмпл жанра с настоящим низом (``mix.kick_sound``)."""
    steps: Dict[str, List[int]] = {}
    for pattern in (material.drums, material.hats):
        if pattern and len(pattern) != STEPS_PER_BAR:
            raise ValueError(f"рисунок ударных {pattern!r} не 16 шагов")
        for i, symbol in enumerate(pattern):
            role = kn.SONG_DRUM_SYMBOLS.get(symbol)
            if role is not None:
                steps.setdefault(role, []).append(i)
    kick = mix.kick_sound("club")
    return {role: Part(role, kn.PLAY_SYNTH, rhythm.grid(on, accents={0: 3}), None, 0.0, (0, 0),
                       kick.sample if role == "kick" else 0)
            for role, on in steps.items()}


def _form(theme_bars: int, roles: frozenset) -> Form:
    plan = kn.SONG_VERSES[verse_count(theme_bars)]
    return Form(tuple(Section(name, theme_bars, energy, frozenset(sec & roles), False)
                      for name, energy, sec in plan), kind=kn.FORM_SONG)


def song_track(material: SongMaterial, *, seed: int, deck: str = "A") -> Track:
    """Трек-песня по материалу ``harmonize``; ``ValueError``/``TrackError`` — материал не ложится в модель."""
    beats = material.beats
    theme_bars = int(round(beats / BEATS_PER_BAR))
    for name, line in (("bass", material.bass), ("pad", material.pad)):
        if abs(sum(d for _n, d in line) - beats) > 1e-6:
            raise ValueError(f"{name}: длина {sum(d for _n, d in line)} долей ≠ мелодии {beats}")
    if theme_bars * BEATS_PER_BAR != beats:
        raise ValueError(f"мелодия {beats} долей — не целое число тактов")
    synths = {role: random.Random(f"song:{seed}:{role}").choice(kn.SONG_TIMBRES[role]) for role in kn.TONAL_ROLES}
    drums = _drums(material)
    form = _form(theme_bars, frozenset(drums) | set(kn.TONAL_ROLES))
    tonal = {"lead": _tonal("lead", synths["lead"], material.lead, form, beats),
             "bass": _tonal("bass", synths["bass"], material.bass, form, beats),
             "pad": _tonal("pad", synths["pad"], material.pad, form, beats, material.pad_sus)}
    parts, track_mix = mix.mix_parts({**drums, **{r: p for r, p in tonal.items() if p is not None}}, ())
    track_mix = replace(track_mix, duck_depth=0.0, duck_roles=frozenset(), duck_trigger=())  # песня без «качания»
    form = replace(form, sections=tuple(replace(s, roles=frozenset(s.roles & set(parts))) for s in form.sections))
    key = Key(material.root, material.mode)
    hook = Hook(tuple(_events(material.lead, 0.0)), theme_bars, material.melody_id)
    sha = hashlib.sha256(repr((material.melody_id, material.bpm, key, sorted(parts.items()))).encode()).hexdigest()[:8]
    track = Track(
        track_id=f"classic:{material.melody_id}:{deck}:{sha}", seed=seed, bpm=material.bpm, key=key, form=form,
        parts=parts, harmony=Harmony({}), hook=hook, mix=track_mix, energy=SONG_ENERGY,
        transition_in=SONG_TRANSITION, transition_out=SONG_TRANSITION,
        history_key=HistoryKey("classic_v2", "harmonize", material.melody_id, None, key.root),
    )
    validate(track)
    return track


__all__ = ["SONG_TRANSITION", "SongMaterial", "song_track", "verse_count"]
