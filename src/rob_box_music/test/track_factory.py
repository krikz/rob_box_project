"""Генератор случайных ВАЛИДНЫХ ``Track`` для тестов (ADR-0149 §9, PR-1)."""

from __future__ import annotations

import math
import random

from rob_box_music import knowledge as kn
from rob_box_music.model import (
    Chord, Form, Grid, Harmony, HistoryKey, Hook, Key, Mix, Part, PitchEvent, Section, Step, Track, Transition,
)

_MIDDLE = ("build", "drop", "break", "drop")


def _sections(rng: random.Random, bars_total: int) -> list:
    middle = (bars_total - 16) // 8
    names = ["intro"] + [rng.choice(_MIDDLE) for _ in range(middle)] + ["outro"]
    sizes = [8] + [8] * middle + [8]
    out = []
    for name, bars in zip(names, sizes):
        roles = {"kick", "hats", "pad"} if name in ("intro", "outro") else {"kick", "hats", "bass", "lead", "pad"}
        if name == "break":
            roles = {"pad", "lead", "hats"}
        if name == "drop":
            roles |= {"clap", "perc"}
        out.append(Section(name, bars, rng.randint(0, 10), frozenset(roles), rng.random() < 0.5))
    return out


def _grid(rng: random.Random) -> Grid:
    n = rng.choice((16, 32, 64))
    return Grid(tuple(Step(rng.random() < 0.5, rng.randint(0, 3), rng.choice((0, 0, 8))) for _ in range(n)))


def _pitches(rng: random.Random, role: str, key: Key, register, limit: float):
    pcs = kn.scale_pitch_classes(key.root, key.mode)
    pool = [m for m in range(register[0], register[1] + 1) if m % 12 in pcs]
    chrom = [m for m in range(register[0], register[1] + 1) if m % 12 not in pcs]
    events = []
    for beat in sorted(rng.sample(range(0, int(limit) - 2), rng.randint(6, 24))):
        if role == "bass" and chrom and rng.random() < 0.2:
            events.append(PitchEvent(rng.choice(chrom), float(beat), 0.25, 1))
        else:
            events.append(PitchEvent(rng.choice(pool), float(beat), rng.choice((0.5, 1.0, 2.0)), rng.randint(0, 3)))
    return tuple(events)


def _part(rng: random.Random, role: str, key: Key, bars_total: int) -> Part:
    limit = float(bars_total * 4)
    register = (0, 0)
    pitches = None
    synth = kn.PLAY_SYNTH if role not in kn.TONAL_ROLES else rng.choice(kn.SYNTH_PALETTE[role])
    if role in kn.TONAL_ROLES:
        c_lo, c_hi = kn.REGISTERS[role]
        lo = rng.randint(c_lo, c_lo + 4)
        register = (lo, rng.randint(lo + 12, c_hi))
        pitches = _pitches(rng, role, key, register, limit)
    level = kn.role_ceiling(role) - rng.uniform(0.0, 6.0)
    return Part(role, synth, _grid(rng), pitches, level, register)


def _fit_levels(parts: dict, sections: list) -> dict:
    limit = kn.LEVEL_CEILINGS["master_peak_db"]
    worst = max(10 * math.log10(sum(10 ** (parts[r].level_db / 10) for r in s.roles)) for s in sections)
    shift = min(0.0, limit - 0.1 - worst)  # 0.1 дБ запаса от плавающей точки на границе
    return {r: Part(p.role, p.synth_or_sample, p.grid, p.pitches, p.level_db + shift, p.register)
            for r, p in parts.items()}


def make_track(seed: int) -> Track:
    rng = random.Random(seed)
    key = Key(rng.randrange(12), rng.choice(kn.GENRE_WINDOWS["club"].scales))
    bars_total = rng.choice((32, 48, 64))
    sections = _sections(rng, bars_total)
    used = sorted({r for s in sections for r in s.roles})
    parts = _fit_levels({r: _part(rng, r, key, bars_total) for r in used}, sections)
    pad_lo, pad_hi = parts["pad"].register
    prog = {s.name: tuple(Chord(rng.randrange(7), (pad_lo, pad_lo + 4, pad_lo + 7)) for _ in range(4))
            for s in sections}
    hook = None
    if rng.random() < 0.7:
        hook = Hook((PitchEvent(rng.randint(60, 80), 0.0, 1.0, 1), PitchEvent(62, 4.0, 2.0, 0)),
                    rng.randint(4, 8), rng.choice((None, "spott_ci")))
    mix = Mix({r: p.level_db for r, p in parts.items()},
              {r: (0.0 if r in ("kick", "bass") else rng.uniform(-0.8, 0.8)) for r in parts},
              rng.random(), {"drop": ("room",)})
    tr = Transition(rng.choice((8, 16, 32)), 0, True)
    return Track(f"set1:{seed:02d}:A:{seed:08x}", seed, rng.randint(*kn.GENRE_WINDOWS["club"].bpm), key,
                 Form(tuple(sections)), parts, Harmony(prog), hook, mix, rng.randint(1, 5), tr, tr,
                 HistoryKey("kit", "prog", None, None, key.root))
