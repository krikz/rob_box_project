"""Talking: three mouth designs sharing one speech envelope and face acting.

The envelope is synthetic for the lab preview (syllables); live, the mouth
follows ``ticks.ctx.mouth_level`` — the real TTS envelope (ADR-0150 §5.5.4).
"""
from __future__ import annotations

import random
from dataclasses import replace

from .face import (FPS, TEETH, TEETH_HI, Eye, Frame, Mouth, apply_blink,
                   blink_curve, clamp, col, draw_eye, draw_mouth, ease_io, gaze_px)

INTERIOR = col(1.6, 0.35, 0.3)      # dim warm "throat" so an open mouth is not just missing pixels


def frames(T):
    return [i / FPS for i in range(int(round(T * FPS)))]


def overlap(a0, a1, b0, b1):
    return max(0.0, min(a1, b1) - max(a0, b0))


# ------------------------------------------------------------ speech model
def speech(rng, T, phrase_pause=(0.45, 0.8)):
    """Syllables grouped into words and phrases. Returns (events, phrases).
    event = (t, dur, amp, vowel, stressed)."""
    ev, phrases = [], []
    t = 0.25
    while t < T - 0.6:
        p0 = t
        for _ in range(rng.randint(2, 5)):                 # words
            n = rng.randint(1, 3)
            stress = rng.randrange(n)
            for k in range(n):
                dur = rng.uniform(0.12, 0.22)
                amp = rng.uniform(0.55, 0.8) if k != stress else rng.uniform(0.85, 1.0)
                vowel = rng.choices(['A', 'O', 'E'], [0.45, 0.25, 0.30])[0]
                ev.append((t, dur, amp, vowel, k == stress))
                t += dur
            t += rng.uniform(0.04, 0.12)
        phrases.append((p0, t))
        t += rng.uniform(*phrase_pause)
    return ev, phrases


def envelope(ev, T):
    """Openness 0..1 per frame, smoothed like an audio follower."""
    o, out, vowels, stress = 0.0, [], [], []
    cur = 'A'
    for t in frames(T):
        target, st = 0.0, False
        for (s, d, amp, vowel, stressed) in ev:
            if s <= t < s + d:
                p = (t - s) / d
                target = amp if p < 0.65 else amp * 0.3
                cur = vowel
                st = stressed and p < 0.35
                break
        k = 0.6 if target > o else 0.35          # fast attack, slower release at 30 fps
        o += (target - o) * k
        out.append(o)
        vowels.append(cur)
        stress.append(st)
    return out, vowels, stress


# ------------------------------------------------------------ face acting
def acting(t, phrases, stressed, rng_seed):
    """Eyes while talking: look at the listener, glance away at some phrase
    starts (recalling), brows up + nod on stressed syllables, blink at the
    end of every phrase."""
    e = Eye(top=0.9, gx=0, gy=0)
    for i, (p0, p1) in enumerate(phrases):
        r = random.Random(rng_seed * 31 + i)
        if r.random() < 0.55 and p0 <= t < p0 + 0.45:
            k = ease_io((t - p0) / 0.08) if t < p0 + 0.08 else (1 - ease_io((t - p0 - 0.33) / 0.12) if t > p0 + 0.33 else 1)
            side = -1 if r.random() < 0.5 else 1
            e = replace(e, gx=round(1.8 * side * k), gy=round(-1.3 * k))
        if p0 <= t < p1 and r.random() < 0.3:
            # small drift toward one side during the phrase
            e = replace(e, gx=e.gx or (1 if i % 2 else -1) * (1 if (t - p0) > 0.8 else 0))
    if stressed:
        e = replace(e, top=0.0)        # brows up on the stressed syllable
    k = max((blink_curve(t, p1 + 0.04, 0.2) for (_, p1) in phrases), default=0.0)
    return apply_blink(e, k)


# ------------------------------------------------------------ mouth designs
def mouth_jaw(arr, o, vowel):
    """Bender grille stays; a lens-shaped opening grows from the bite line,
    jaw drops more than the upper lip, edges anti-aliased."""
    draw_mouth(arr, Mouth())
    W = {'A': 0.9, 'O': 0.42, 'E': 1.06}[vowel]
    Hmax = {'A': 3.1, 'O': 3.1, 'E': 1.8}[vowel]
    up = {'A': 0.36, 'O': 0.5, 'E': 0.12}[vowel]
    H = o * Hmax
    for x in range(1, 24):
        prof = max(0.0, 1 - ((x - 12) / (11.5 * W)) ** 2) ** 0.6
        h = H * prof
        if h < 0.12:
            continue
        a0 = max(0.5, 1.5 - h * up)          # outer rows stay: they are the lips
        a1 = min(3.5, 2.5 + h * (1 - up))
        glow = min(1.0, h / 1.2)
        for y in range(5):
            c = overlap(y - 0.5, y + 0.5, a0, a1)
            if c <= 0:
                continue
            arr[y, x] = arr[y, x] * (1 - c) + INTERIOR * glow * c


def mouth_rubber(arr, o, vowel):
    """Squash & stretch: closed = wide thin slot, open = narrower and taller.
    The grille stretches with it."""
    Wv = {'A': 19.0, 'O': 11.0, 'E': 23.0}[vowel]
    Hv = {'A': 5.0, 'O': 5.0, 'E': 4.2}[vowel]
    H = 3 + (Hv - 3) * o
    W = 23 - (23 - Wv) * o
    yt, yb = 2 - H / 2, 2 + H / 2
    xl, xr = 12 - W / 2, 12 + W / 2
    divs = [xl + W * i / 6 for i in range(1, 6)]
    arr[:] = 0
    for x in range(25):
        cx = overlap(x - 0.5, x + 0.5, xl, xr)
        if cx <= 0:
            continue
        dd = min(abs(x - d) for d in divs)
        grid = 0.22 + 0.78 * clamp(dd - 0.15, 0, 1) if dd < 1.15 else 1.0
        for y in range(5):
            ct = overlap(y - 0.5, y + 0.5, yt, yt + 1)
            cb = overlap(y - 0.5, y + 0.5, yb - 1, yb)
            ci = overlap(y - 0.5, y + 0.5, yt + 1, yb - 1)
            c = TEETH * (ct + cb) * grid + INTERIOR * ci * min(1.0, o * 2)
            # rounded corners
            corner = (x - 0.5 < xl + 1 or x + 0.5 > xr - 1) and (ct > 0 and y - 0.5 < yt + 0.6 or cb > 0 and y + 0.5 > yb - 0.6)
            arr[y, x] = c * cx * (0.4 if corner else 1.0)


def mouth_voicebox(arr, o, vowel):
    """KITT-style voice box inside the grille: light grows from the centre
    teeth outward and up/down with loudness."""
    draw_mouth(arr, Mouth(bright=0.16))
    amber = col(25, 15, 4)
    for k, (x0, x1) in enumerate([(1, 3), (5, 7), (9, 11), (13, 15), (17, 19), (21, 23)]):
        d = abs(k - 2.5)                      # 0.5, 1.5, 2.5
        lvl = clamp(o * 1.35 - (d - 0.5) * 0.42, 0.0, 1.0)
        h = lvl * 2.6                          # half-height in rows around row 2
        for x in range(x0, x1 + 1):
            for y in range(5):
                c = overlap(y - 0.5, y + 0.5, 2 - h, 2 + h)
                if c <= 0:
                    continue
                hot = clamp(1 - abs(y - 2) / 2.6, 0, 1)
                colr = TEETH_HI * (1 - hot) * 0.7 + amber * hot
                arr[y, x] = arr[y, x] * (1 - c) + colr * c


DESIGNS = {'jaw': mouth_jaw, 'rubber': mouth_rubber, 'voicebox': mouth_voicebox}


def talking(ticks, design='jaw', seed=6, T=10.0, mood='neutral', phrase_pause=(0.45, 0.8)):
    """Live: mouth by ``ctx.mouth_level`` (TTS envelope), tone by ``ctx.mood``,
    eyes keep acting on the synthetic phrase timeline and look at the speaker."""
    rng = random.Random(seed)
    ev, phrases = speech(rng, T, phrase_pause)
    env, vowels, stress = envelope(ev, T)
    draw = DESIGNS[design]
    for t in ticks:
        i = min(int(t * FPS + 0.5), len(env) - 1)
        c = ticks.ctx
        md = mood if c.mood == 'neutral' else c.mood
        e = acting(t, phrases, stress[i], seed)
        if c.gaze_deg is not None:
            e = replace(e, gx=round(gaze_px(c.gaze_deg) + e.gx * 0.5))
        if md == 'happy':
            e = replace(e, top=0.0, bot=6.4, bot_arc=1.4, bot_lid_dark=True, gy=-0.5)
        elif md == 'excited':
            e = replace(e, top=0.0)
        f = Frame()
        draw_eye(f.eye_l, e, 'l')
        draw_eye(f.eye_r, e, 'r')
        o, vowel = (env[i], vowels[i]) if c.mouth_level is None else (c.mouth_level, c.vowel)
        draw(f.mouth, o, vowel)
        if md == 'happy' and o < 0.15 and design == 'jaw':
            draw_mouth(f.mouth, Mouth(curve=1.0))
        yield f
