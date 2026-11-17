"""Бит подложки из движка: ноты клубного трека v2 → звук 16 кГц в Python (без scsynth).

Ритм, темп и форма — настоящие: ``arrange.compose.club_track`` (энергия настроения ``groove`` — то, что ставит
``request_music(intent=backing)``, ``backing_deck.DEFAULT_MOOD``) → ``render.renardo.render`` → события
``render.events.program_events`` (семантика ``Players.py``). Тембры — простые модели (бочка — синус со спадом
частоты, хэт и клэп — полосовой шум, бас — синус с октавой); лупы паков DJ_Dave и пэд не звучат. Это не NRT
scsynth: громкость и тембр не те, что на роботе, сетка долей — та же.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np
from scipy.signal import butter, sosfilt

from common import SR

REPO = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPO / "src" / "rob_box_music"), str(REPO / "src" / "rob_box_dialog")]

ROLES = ("kick", "hats", "clap", "bass")
LOOP_BEATS = 16  # четыре такта


def engine_loop(seed: int) -> Tuple[float, List[tuple]]:
    """``(bpm движка, [(доля, роль, midi, sus_долей, amp)])`` — четыре такта трека, где звучат бочка, хэт и клэп."""
    from rob_box_music import knowledge as kn
    from rob_box_music.arrange.compose import club_track
    from rob_box_music.render.events import program_events
    from rob_box_music.render.renardo import render

    track = club_track(seed, track_no=kn.ENERGY_WAVE.index(kn.MOOD_ENERGY["groove"]) + 1)
    program = render(track, "A")
    _, events = program_events(program.code, program.form_beats)
    role_of = {slot: role for role, slot in program.slots.items() if role in ROLES}
    notes = [(e.beat, role_of[e.slot], e.midi, e.sus_beats, e.amp) for e in events if e.slot in role_of]
    best, best_score = 0.0, -1
    for w in np.arange(0, float(program.form_beats) - LOOP_BEATS + 1, LOOP_BEATS):
        roles = [n[1] for n in notes if w <= n[0] < w + LOOP_BEATS]
        score = min(roles.count("kick"), 16) * 10 + ("hats" in roles) * 5 + ("clap" in roles) * 5 + ("bass" in roles)
        if score > best_score:
            best, best_score = w, score
    loop = [(b - best, r, m, s, a) for b, r, m, s, a in notes if best <= b < best + LOOP_BEATS]
    return float(track.bpm), loop


def _noise(n: int, band: Tuple[float, float], rng: np.random.Generator) -> np.ndarray:
    sos = butter(2, band, btype="band", fs=SR, output="sos")
    return sosfilt(sos, rng.standard_normal(n))


def _hit(role: str, midi: float, dur_s: float, rng: np.random.Generator) -> np.ndarray:
    if role == "kick":
        t = np.arange(int(0.3 * SR)) / SR
        f = 50 + 70 * np.exp(-t / 0.03)
        return np.sin(2 * np.pi * np.cumsum(f) / SR) * np.exp(-t / 0.15)
    if role == "hats":
        n = int(0.06 * SR)
        return 0.35 * _noise(n, (5000, 7600), rng) * np.exp(-np.arange(n) / SR / 0.02)
    if role == "clap":
        n = int(0.2 * SR)
        env = np.exp(-np.arange(n) / SR / 0.07)
        return 0.8 * _noise(n, (900, 3200), rng) * env
    n = max(1, int(dur_s * SR))
    t = np.arange(n) / SR
    f = 440.0 * 2 ** ((float(midi or 36) - 69) / 12)
    env = np.minimum(1, t / 0.005) * np.minimum(1, (dur_s - t) / 0.03).clip(0)
    return 0.5 * (np.sin(2 * np.pi * f * t) + 0.3 * np.sin(4 * np.pi * f * t)) * env


def render_beat(loop: List[tuple], bpm: float, seconds: float, seed: int = 0) -> np.ndarray:
    """Петля ``loop`` (доли от 0) на темпе ``bpm`` с доли 0 в t=0, длиной ``seconds``."""
    rng = np.random.default_rng(seed)
    beat_s = 60.0 / bpm
    out = np.zeros(int(seconds * SR) + SR)
    k = 0
    while k * LOOP_BEATS * beat_s < seconds:
        for b, role, midi, sus, amp in loop:
            t = (k * LOOP_BEATS + b) * beat_s
            if t >= seconds:
                continue
            hit = _hit(role, midi, sus * beat_s, rng) * float(amp)
            i = int(round(t * SR))
            out[i:i + len(hit)] += hit[:len(out) - i]
        k += 1
    return out[:int(seconds * SR)].astype(np.float32)
