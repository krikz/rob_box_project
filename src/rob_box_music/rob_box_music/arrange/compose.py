"""Первый club-трек модели v2 (ADR-0149 §9 PR-2). Полный ``compose(plan, track_no, history)`` — PR-3.

Форма 32 такта: intro (бочка, хэт, пэд) → groove (+клэп, бас) → drop (+лид) → outro (без лида).
Бочка 4/4, бас в оффбит, пэд с голосоведением, лид — мотив с паузами. Всё — от ``seed``.
"""

from __future__ import annotations

import hashlib
import random
from typing import Dict, Optional, Tuple

from .. import knowledge as kn
from ..model import (
    BEATS_PER_BAR, Form, Harmony, HistoryKey, Key, Mix, Part, PitchEvent, Section, Track, Transition,
)
from . import bass, harmony, lead, rhythm

_DRUMS = frozenset({"kick", "hats"})
SECTIONS: Tuple[Tuple[str, frozenset], ...] = (
    ("intro", _DRUMS | {"pad"}),
    ("groove", _DRUMS | {"clap", "bass", "pad"}),
    ("drop", _DRUMS | {"clap", "bass", "pad", "lead"}),
    ("outro", _DRUMS | {"bass", "pad"}),
)
SECTION_BARS = 8
CHORD_BARS = 2
#: Уровни ролей, дБ пика (≤ потолков ``knowledge.LEVEL_CEILINGS``; сумма drop ≈ −3.2 дБ).
LEVELS_DB: Dict[str, float] = {"kick": -6.0, "hats": -16.0, "clap": -13.0, "bass": -10.0, "pad": -18.0, "lead": -14.0}
PAN: Dict[str, float] = {"hats": 0.25, "clap": -0.2}
SYNTHS: Dict[str, str] = {"bass": "bass", "pad": "sinepad", "lead": "pluck"}
PAD_GAP = 3  # верх пэда ниже лида на ≥ 3 полутона (ADR-0149 §3.6)


def _bars_with(role: str):
    for i, (_name, roles) in enumerate(SECTIONS):
        if role in roles:
            yield from range(i * SECTION_BARS, (i + 1) * SECTION_BARS)


def _pad(chords, register) -> Part:
    loop = len(chords) * CHORD_BARS
    events = tuple(
        PitchEvent(m, bar * BEATS_PER_BAR, CHORD_BARS * BEATS_PER_BAR - 0.5, 2)
        for bar in _bars_with("pad") if bar % CHORD_BARS == 0
        for m in chords[(bar % loop) // CHORD_BARS].voicing
    )
    grid = rhythm.grid(range(0, loop * 16, CHORD_BARS * 16), loop * 16)
    return Part("pad", SYNTHS["pad"], grid, events, LEVELS_DB["pad"], register)


def _bass(key: Key, chords) -> Part:
    loop = len(chords) * CHORD_BARS
    bars = [(bar, harmony.triad_pcs(key, chords[(bar % loop) // CHORD_BARS].degree)) for bar in _bars_with("bass")]
    events = bass.offbeat_bass(bars, kn.REGISTERS["bass"])
    return Part("bass", SYNTHS["bass"], rhythm.grid(rhythm.OFFBEAT_STEPS), events, LEVELS_DB["bass"],
                kn.REGISTERS["bass"])


def _lead(key: Key, rng: random.Random) -> Part:
    register = kn.REGISTERS["lead"]
    motif = lead.motif(key, rng, register)
    span = lead.MOTIF_BARS * BEATS_PER_BAR
    starts = [bar * BEATS_PER_BAR for bar in _bars_with("lead") if bar % lead.MOTIF_BARS == 0]
    events = tuple(PitchEvent(e.midi, start + e.beat, e.dur_beats, e.accent) for start in starts for e in motif)
    grid = rhythm.grid({int(e.beat * 4) % (span * 4) for e in motif}, span * 4)
    return Part("lead", SYNTHS["lead"], grid, events, LEVELS_DB["lead"], register)


def _drums() -> Dict[str, Part]:
    grids = {"kick": rhythm.kick_grid(kn.GENRE_WINDOWS["club"].kick), "hats": rhythm.hats_grid(),
             "clap": rhythm.clap_grid()}
    return {r: Part(r, kn.PLAY_SYNTH, g, None, LEVELS_DB[r], (0, 0)) for r, g in grids.items()}


def club_track(seed: int, *, set_id: str = "v2", deck: str = "A", bpm: Optional[int] = None,
               root: Optional[int] = None, mode: str = "minor") -> Track:
    """Один club-трек из сида: темп в окне жанра, тоника, прогрессия и мотив лида — от ``seed``."""
    rng = random.Random(seed)
    lo, hi = kn.GENRE_WINDOWS["club"].bpm
    bpm = rng.randint(lo, hi) if bpm is None else bpm
    key = Key(rng.randrange(12) if root is None else root, mode)
    degrees = rng.choice(harmony.PROGRESSIONS)
    lead_part = _lead(key, rng)
    pad_top = min(kn.REGISTERS["pad"][1], min(e.midi for e in lead_part.pitches) - PAD_GAP)
    pad_register = (kn.REGISTERS["pad"][0], pad_top)
    chords = harmony.pad_chords(key, degrees, pad_register)
    parts = {**_drums(), "bass": _bass(key, chords), "pad": _pad(chords, pad_register), "lead": lead_part}
    form = Form(tuple(Section(n, SECTION_BARS, 3 + 2 * i, roles) for i, (n, roles) in enumerate(SECTIONS)))
    prog = "-".join(str(d) for d in degrees)
    sha = hashlib.sha256(repr((bpm, key, sorted(parts.items()), chords)).encode()).hexdigest()[:8]  # модель, не сид
    return Track(
        track_id=f"{set_id}:{seed:02d}:{deck}:{sha}", seed=seed, bpm=bpm, key=key, form=form, parts=parts,
        harmony=Harmony({n: chords for n, _ in SECTIONS}),
        hook=None, mix=Mix({r: p.level_db for r, p in parts.items()}, {r: PAN.get(r, 0.0) for r in parts}, 0.0),
        energy=3, transition_in=Transition(8, 4, True), transition_out=Transition(8, 4, True),
        history_key=HistoryKey("club_v2_first", prog, None, None, key.root),
    )


__all__ = ["LEVELS_DB", "SECTIONS", "club_track"]
