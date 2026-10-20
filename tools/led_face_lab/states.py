"""Face clips: each returns a list of Frames at FPS. Rear is filled separately
unless the clip owns it (music, driving)."""
from __future__ import annotations

import math
import random

import numpy as np

import rear as R
import talk as TK
import vehicles as V
from face import (DT, EYE, EYE_RIM, FPS, TEETH, TEETH_HI, VOWEL_LEVELS, Eye, Frame,
                  Mouth, Track, apply_blink, blink_curve, clamp, col, draw_eye,
                  draw_mouth, ease_io, gaze_at, lerp, schedule_blinks,
                  schedule_saccades, tooth_pixels, tooth_ring)


def frames(T):
    n = int(round(T * FPS))
    return [i * DT for i in range(n)]


def put_face(f: Frame, el: Eye, er: Eye, m: Mouth, hl=None, tint=None):
    draw_eye(f.eye_l, el, 'l')
    draw_eye(f.eye_r, er, 'r')
    draw_mouth(f.mouth, m, hl, tint)


def blinked(e, t, blinks, dur=0.22):
    k = max((blink_curve(t, b, dur) for b in blinks), default=0.0)
    return apply_blink(e, k)


# ============================================================ IDLE
def idle_stare(seed=1, T=16.0):
    rng = random.Random(seed)
    blinks = schedule_blinks(rng, T)
    sac = schedule_saccades(rng, T)
    look_t = 7.0
    smirk = Track(0.0).hold(10.2).to(10.5, 1.0).hold(11.6).to(12.0, 0.0)
    out = []
    for t in frames(T):
        gx, gy = gaze_at(sac, t)
        # slow look-around: left ... right ... back
        if look_t <= t < look_t + 2.6:
            p = (t - look_t) / 2.6
            gx = -2.6 * math.sin(p * math.pi * 2) if p < 0.5 else 2.6 * math.sin((p - 0.5) * math.pi * 2)
            gx = round(gx)
            gy = 0
        e = Eye(top=1.0, gx=gx, gy=gy)
        s = smirk(t)
        f = Frame()
        put_face(f, blinked(e, t, blinks), blinked(e, t, blinks), Mouth(smirk=1.0 if s > 0.5 else 0.0))
        out.append(f)
    return out


def yawn_shape(y):
    return 'C' if y < 0.2 else ('O1' if y < 0.45 else ('A2' if y < 0.75 else 'A3'))


def idle_bored(seed=2, T=14.0):
    rng = random.Random(seed)
    blinks = [b for b in schedule_blinks(rng, T, 3.5, 7.0, 0.0) if not (5.0 < b < 8.8)]
    drift = Track(0.0).to(2.0, -2.0).hold(4.0).to(5.0, 1.0).hold(9.5).to(11.0, 2.5).hold(13.0).to(14.0, 0.0)
    yawn = Track(0.0).hold(5.4).to(6.4, 1.0).hold(7.4).to(8.2, 0.0)
    out = []
    for t in frames(T):
        y = yawn(t)
        e = Eye(top=lerp(2.8, 4.3, y), gx=round(drift(t)), gy=0.6, bright=0.9)
        m = Mouth(shape=yawn_shape(y), curve=-1.0 if y < 0.1 else 0.0)
        f = Frame()
        put_face(f, blinked(e, t, blinks, 0.4), blinked(e, t, blinks, 0.4), m)
        out.append(f)
    return out


# ============================================================ LISTENING
def listening(seed=3, T=9.0):
    rng = random.Random(seed)
    blinks = schedule_blinks(rng, T, 3.0, 6.0, 0.0)
    perk = Track(1.0).to(0.18, -0.2).to(0.35, 0.0)
    # the speaker moves from viewer-right to viewer-left at t=4.5 (DOA from ReSpeaker)
    target = Track(1.6).hold(4.5).to(4.62, -1.6)
    nods = [2.2, 6.4]
    out = []
    for t in frames(T):
        jitter = 0.5 if (int(t * 1.3) % 3 == 1) else 0.0
        nod = max((math.sin(math.pi * (t - n) / 0.4) for n in nods if n <= t < n + 0.4), default=0.0)
        e = Eye(top=max(0.0, perk(t)) + 1.4 * nod, gx=round(target(t) + jitter), gy=-0.4)
        f = Frame()
        put_face(f, blinked(e, t, blinks), blinked(e, t, blinks), Mouth())
        out.append(f)
    return out


# ============================================================ THINKING
def thinking_teeth(seed=4, T=8.0):
    rng = random.Random(seed)
    blinks = schedule_blinks(rng, T, 2.5, 4.5, 0.0)
    ring = tooth_ring()
    out = []
    for t in frames(T):
        gx = 1.6 if (t % 4.0) < 2.6 else -1.0
        el = Eye(top=1.8, slant=-0.6, gx=gx, gy=-1.6)          # sceptical squint
        er = Eye(top=0.0, gx=gx, gy=-1.6)
        pos = (t / 1.1) * len(ring)
        hl = {}
        for k in range(3):
            idx = int(pos - k) % len(ring)
            which, cell = ring[idx]
            c = [TEETH_HI, col(16, 16, 13), col(10, 10, 8)][k]
            for p in tooth_pixels(which, cell):
                hl[p] = c
        f = Frame()
        put_face(f, blinked(el, t, blinks), blinked(er, t, blinks), Mouth(bright=0.35), hl)
        out.append(f)
    return out


def thinking_orbit(seed=5, T=8.0):
    out = []
    for t in frames(T):
        a = t * math.tau / 1.3
        gx, gy = round(math.cos(a) * 1.6), round(math.sin(a) * 1.4)
        e = Eye(top=0.6, gx=gx, gy=gy)
        tilt = math.sin(t * math.tau / 2.6)
        m = Mouth(tilt=0.9 if tilt > 0.35 else (-0.9 if tilt < -0.35 else 0.0))
        f = Frame()
        put_face(f, e, e, m)
        out.append(f)
    return out


# ============================================================ TALKING
def syllables(rng, T):
    ev = []
    t = 0.15
    while t < T - 0.3:
        words = rng.randint(2, 5)
        for _ in range(words):
            n = rng.randint(1, 3)
            for _ in range(n):
                dur = rng.uniform(0.13, 0.24)
                amp = rng.uniform(0.45, 1.0)
                shape = rng.choices(['A', 'O', 'E'], [0.45, 0.25, 0.30])[0]
                ev.append((t, dur, amp, shape))
                t += dur
            t += rng.uniform(0.05, 0.14)
        t += rng.uniform(0.35, 0.7)
    return ev


def speech_envelope(ev, T):
    """Mouth openness 0..1 per frame: syllable targets smoothed like an
    audio envelope (fast attack, slower release). The real robot can feed
    the same filter from the TTS audio level instead of synthetic syllables."""
    env, shapes = [], []
    o = 0.0
    cur_shape = 'A'
    for t in frames(T):
        target = 0.0
        for (s, d, amp, shape) in ev:
            if s <= t < s + d:
                p = (t - s) / d
                target = amp if p < 0.7 else amp * 0.35
                cur_shape = shape
                break
        k = 0.75 if target > o else 0.45
        o = o + (target - o) * k
        env.append(o)
        shapes.append(cur_shape)
    return env, shapes


def shape_for(o, vowel):
    lvl = 0 if o < 0.16 else (1 if o < 0.42 else (2 if o < 0.7 else 3))
    return VOWEL_LEVELS[vowel][lvl], lvl


def talking(seed=6, T=9.0, mood='neutral'):
    rng = random.Random(seed)
    ev = syllables(rng, T)
    env, vowels = speech_envelope(ev, T)
    blinks = schedule_blinks(rng, T, 3.0, 6.0, 0.1)
    sac = schedule_saccades(rng, T, every=(1.4, 3.2), rx=1.0, ry=0.5)
    out = []
    for i, t in enumerate(frames(T)):
        gx, gy = gaze_at(sac, t)
        o = env[i] * (1.2 if mood == 'excited' else 1.0)
        shape, lvl = shape_for(o, vowels[i])
        if mood == 'happy':
            m = Mouth(curve=1.0 if lvl == 0 else 0.0, shape=shape)
            e = Eye(top=0.0, bot=6.4, bot_arc=1.4, gx=round(gx), gy=-0.5, bot_lid_dark=True)
        elif mood == 'excited':
            m = Mouth(shape=shape)
            e = Eye(top=0.0, gx=round(gx), gy=round(gy))
        else:
            m = Mouth(shape=shape)
            e = Eye(top=0.9 if o < 0.8 else 0.0, gx=round(gx), gy=round(gy))
        f = Frame()
        put_face(f, blinked(e, t, blinks), blinked(e, t, blinks), m)
        out.append(f)
    return out


# ============================================================ EMOTIONS
def happy(seed=7, T=6.0):
    k = Track(0.0).to(0.25, 1.0)
    out = []
    for t in frames(T):
        kk = k(t)
        giggle = t < 2.2 and (t * 5.0) % 1.0 < 0.5
        squeeze = 0.5 + 0.5 * math.sin(t * math.tau * 2.5) if t < 2.2 else 0.0
        e = Eye(top=0.0, bot=lerp(7, 6.2, kk), bot_arc=3.4 * kk + 0.7 * squeeze, gx=0, gy=-1, bot_lid_dark=True)
        m = Mouth(curve=1.0, shape='E1' if giggle else 'C')
        f = Frame()
        put_face(f, e, e, m)
        out.append(f)
    return out


def wink(seed=8, T=5.0):
    w = Track(0.0).hold(0.8).to(0.95, 1.0).hold(1.5).to(1.7, 0.0)
    out = []
    for t in frames(T):
        el = Eye(top=0.5, gx=1, gy=0)
        er = apply_blink(Eye(top=0.5, gx=1, gy=0, bot_arc=1.5 * w(t)), w(t))
        f = Frame()
        s = max(w(t), 1.0 if 0.9 < t < 3.5 else 0.0)
        put_face(f, el, er, Mouth(smirk=1.0 if s > 0.3 else 0.0))
        out.append(f)
    return out


def sad(seed=9, T=7.0):
    blinks = [1.2, 4.6]
    droop = Track(0.0).to(0.6, 1.0)
    out = []
    for t in frames(T):
        d = droop(t)
        e = Eye(top=lerp(1.0, 2.6, d), slant=-2.0 * d, gx=0, gy=round(1.4 * d), bright=lerp(1, 0.62, d))
        f = Frame()
        put_face(f, blinked(e, t, blinks, 0.5), blinked(e, t, blinks, 0.5), Mouth(curve=-2.0 if d > 0.6 else (-1.0 if d > 0.2 else 0.0), bright=lerp(1, 0.75, d)))
        # a single tear from the viewer-left eye, inner corner
        if 2.4 < t < 4.4:
            p = (t - 2.4) / 2.0
            y = 5 + int(p * 4)
            if y < 8:
                f.eye_l[y, 6] = col(3, 9, 25)
        out.append(f)
    return out


def angry(seed=10, T=6.0):
    k = Track(0.0).to(0.2, 1.0)
    red, red_rim = col(25, 5, 0), col(13, 1, 0)
    out = []
    for t in frames(T):
        kk = k(t)
        e = Eye(top=lerp(1.0, 1.6, kk), slant=2.3 * kk, gy=0.6, gx=0,
                color=EYE * (1 - kk) + red * kk, rim=EYE_RIM * (1 - kk) + red_rim * kk)
        grind = (int(t * 12) % 2) if (t < 1.2 or 3.4 < t < 3.9) else 0
        m = Mouth(curve=-1.0 if kk > 0.5 else 0.0, color=TEETH * (1 - kk) + col(25, 25, 22) * kk, x_shift=grind, grid=0.0)
        f = Frame()
        put_face(f, e, e, m)
        out.append(f)
    return out


def surprised(seed=11, T=5.0):
    out = []
    for t in frames(T):
        if t < 0.12:
            e = apply_blink(Eye(top=1.0), 1.0)
            m = Mouth()
        else:
            relax = ease_io((t - 3.2) / 0.8) if t > 3.2 else 0.0
            e = Eye(top=lerp(0.0, 1.0, relax), pupil=1 if relax < 0.5 else 2, gx=0, gy=0)
            o = 1 - relax
            m = Mouth(shape='O2' if o > 0.66 else ('O1' if o > 0.25 else 'C'))
        f = Frame()
        put_face(f, e, e, m)
        out.append(f)
    return out


# ============================================================ SLEEP / WAKE
Z = ["###", ".#.", "###"]


SLEEP_EYE = [(3, 0), (3, 7), (4, 1), (4, 6), (5, 2), (5, 3), (5, 4), (5, 5)]   # "︶"


def sleep(seed=12, T=9.0):
    out = []
    for t in frames(T):
        breath = 0.5 + 0.5 * math.sin(t * math.tau / 3.6)
        f = Frame()
        draw_mouth(f.mouth, Mouth(bright=0.3 + 0.1 * breath, shape='O1' if math.sin(t * math.tau / 3.6) > 0.8 else 'C'))
        for arr in (f.eye_l, f.eye_r):
            for (y, x) in SLEEP_EYE:
                arr[y, x] = col(12, 9, 3) * (0.45 + 0.55 * breath)
        # one z at a time drifts up and to the right above the viewer-right eye
        p = (t / 2.2) % 1.0
        x = int(2 + p * 4)
        y = int(round(1.5 - p * 4))
        k = math.sin(p * math.pi)
        for dy, row in enumerate(Z):
            for dx, ch in enumerate(row):
                yy, xx = y + dy - 1, x + dx
                if ch == '#' and 0 <= yy < 3 and 0 <= xx < 8:
                    f.eye_r[yy, xx] = col(9, 12, 25) * k
        out.append(f)
    return out


def wakeup(seed=13, T=6.0):
    open_ = Track(0.0).hold(0.6).to(0.75, 0.15).hold(1.0).to(1.35, 1.1).to(1.5, 1.0)
    look = Track(0.0).hold(1.8).to(1.9, -2.0).hold(2.5).to(2.6, 2.0).hold(3.1).to(3.2, 0.0)
    yawn = Track(0.0).hold(3.6).to(4.3, 1.0).hold(4.9).to(5.4, 0.0)
    blinks = [3.25, 3.55]
    out = []
    for t in frames(T):
        o = open_(t)
        y = yawn(t)
        e = Eye(top=lerp(3.5, 1.0, o) + 1.6 * y, bot=lerp(3.5, 7.0, min(o, 1.0)), gx=round(look(t)))
        f = Frame()
        m = Mouth(shape=yawn_shape(y))
        put_face(f, blinked(e, t, blinks), blinked(e, t, blinks), m)
        if t < 1.6:   # teeth boot one by one, left to right
            lit = int(t / 1.6 * 7)
            for k in range(6):
                x0, x1 = [(1, 3), (5, 7), (9, 11), (13, 15), (17, 19), (21, 23)][k]
                if k >= lit:
                    f.mouth[:, x0:x1 + 1] = 0
        if t < 0.6:   # CRT line warms up
            f.eye_l[:] = 0; f.eye_r[:] = 0
            k = t / 0.6
            for arr in (f.eye_l, f.eye_r):
                w = int(1 + k * 3.5)
                arr[3:5, 4 - w:4 + w] = col(25, 21, 9) * (0.3 + 0.7 * k)
        out.append(f)
    return out


# ============================================================ MUSIC
def eq_mouth(f, t, bpm, accent, seed=0):
    beat = t * bpm / 60.0
    ph = beat % 1.0
    kick = math.exp(-ph * 4.5)
    order = [2, 1, 0, 0, 1, 2]     # bass in the middle, highs at the corners
    mm = np.zeros((5, 25, 3))
    for k, (x0, x1) in enumerate([(1, 3), (5, 7), (9, 11), (13, 15), (17, 19), (21, 23)]):
        band = order[k]
        r = random.Random(int(t * 9) * 11 + k * 7 + seed)
        if band == 0:
            h = 1.8 + kick * 3.6
        elif band == 1:
            h = 2.0 + 2.6 * (0.5 + 0.5 * math.sin(t * 7 + k)) * (0.6 + 0.4 * r.random()) + kick * 0.8
        else:
            h = 1.5 + 3.4 * r.random() ** 1.2
        h = int(clamp(round(h), 1, 5))
        for i in range(h):
            y = 4 - i
            c = TEETH if i < 3 else accent
            mm[y, x0:x1 + 1] = c * (0.75 if i == 0 else 1.0)
    f.mouth[:] = mm


def music_groove(seed=14, T=8.0, bpm=118):
    accents = [col(25, 3, 12), col(2, 14, 25), col(25, 14, 0), col(4, 22, 8)]
    out = []
    for t in frames(T):
        beat = t * bpm / 60.0
        ph = beat % 1.0
        bar = int(beat // 4)
        side = 1 if int(beat // 2) % 2 == 0 else -1
        e = Eye(top=1.0 + 0.9 * math.exp(-ph * 9), gx=1.6 * side, gy=0)
        f = Frame()
        put_face(f, e, e, Mouth())
        eq_mouth(f, t, bpm, accents[bar % 4], seed)
        f.rear_l, f.rear_r = R.speaker_rings(t, bpm)
        out.append(f)
    return out


def music_shades(seed=15, T=8.0, bpm=118):
    out = []
    for t in frames(T):
        beat = t * bpm / 60.0
        ph = beat % 1.0
        drop = clamp(t / 0.7, 0, 1)
        bar_ph = (beat % 4) / 4
        glint = bar_ph * 16 - 4 if bar_ph < 0.6 else -1
        e = Eye(top=1.0, shades=drop, glint=glint if drop >= 1 else -1)
        m = Mouth(curve=1.0, smirk=1.0, shape='E1' if ph < 0.15 else 'C')
        f = Frame()
        put_face(f, e, e, m)
        f.rear_l, f.rear_r = R.disco(t, bpm)
        out.append(f)
    return out


def music_vibe(seed=16, T=8.0, bpm=96):
    out = []
    for t in frames(T):
        beat = t * bpm / 60.0
        e = Eye(top=0.0, bot=6.2, bot_arc=3.4 + 0.6 * math.exp(-(beat % 2) * 4), gy=-1, bot_lid_dark=True)
        f = Frame()
        put_face(f, e, e, Mouth(curve=1.0))
        # humming: a warm glow rolls along the teeth with the beat
        mm = f.mouth
        pos = (beat * 0.5 % 1.0) * 30 - 3
        for x in range(1, 24):
            k = max(0.0, 1 - abs(x - pos) / 4.0)
            lit = mm[:, x].sum(axis=1) > 0
            mm[lit, x] = mm[lit, x] * (1 - k) + col(25, 20, 8) * k
        f.rear_l, f.rear_r = R.eq_bars(t, bpm)
        out.append(f)
    return out


# ============================================================ SYSTEM
X_EYE = [
    "##....##",
    ".##..##.",
    "..####..",
    "...##...",
    "..####..",
    ".##..##.",
    "##....##",
    "........",
]


def error(seed=17, T=5.0):
    out = []
    for t in frames(T):
        f = Frame()
        on = (t % 1.0) < 0.7
        for arr in (f.eye_l, f.eye_r):
            for y, row in enumerate(X_EYE):
                for x, ch in enumerate(row):
                    if ch == '#' and on:
                        arr[y, x] = col(25, 2, 0)
        draw_mouth(f.mouth, Mouth(bright=0.8))
        if (t % 1.3) < 0.35:      # glitch: rows slide
            g = random.Random(int(t * 20))
            for y in range(5):
                f.mouth[y] = np.roll(f.mouth[y], g.randint(-3, 3), axis=0)
            f.mouth[:, :, 1:] *= 0.4
        out.append(f)
    return out


def battery_mouth(f, level, color, blink_on=True):
    mm = f.mouth
    mm[:] = 0
    frame_c = col(12, 12, 12)
    mm[0, 3:21] = frame_c
    mm[4, 3:21] = frame_c
    mm[1:4, 3] = frame_c
    mm[1:4, 20] = frame_c
    mm[1:4, 21] = 0
    mm[2, 21] = frame_c
    cells = int(round(level * 8))
    for c in range(8):
        x0 = 5 + c * 2
        if c < cells and blink_on:
            mm[1:4, x0] = color


def low_battery(seed=18, T=6.0):
    out = []
    for t in frames(T):
        e = Eye(top=3.0, gy=1, bright=0.7)
        f = Frame()
        put_face(f, blinked(e, t, [1.5, 4.0], 0.6), blinked(e, t, [1.5, 4.0], 0.6), Mouth())
        battery_mouth(f, 1 / 8, col(25, 1, 0), (t % 1.0) < 0.55)
        out.append(f)
    return out


def charging(seed=19, T=6.0):
    out = []
    for t in frames(T):
        e = Eye(top=0.0, bot=6.0, bot_arc=3.0, gy=-1, bright=0.6, bot_lid_dark=True)
        f = Frame()
        put_face(f, e, e, Mouth())
        lvl = 0.375 + ((t % 2.0) / 2.0) * 0.625
        battery_mouth(f, lvl, col(2, 22, 4))
        out.append(f)
    return out


# ============================================================ DRIVING
def turn(side='left', T=4.0):
    out = []
    for t in frames(T):
        # face is drawn in viewer space: the robot's left is the viewer's right
        g = 2.2 if side == 'left' else -2.2
        e = Eye(top=0.8, gx=round(g), gy=0)
        f = Frame()
        put_face(f, e, e, Mouth())
        f.rear_l, f.rear_r = R.turn_signal(t, side)
        out.append(f)
    return out


def brake(T=4.0):
    out = []
    for t in frames(T):
        k = 1.0 if (t % 2.0) < 1.4 else 0.0
        e = Eye(top=1.0 + 0.8 * k, gy=0)
        f = Frame()
        put_face(f, e, e, Mouth(curve=-1.0 * k))
        f.rear_l, f.rear_r = R.taillight(t, brake=k)
        out.append(f)
    return out


# ============================================================ HOLIDAY celebrate
DIGITS = {
    '0': ["###", "#.#", "#.#", "#.#", "###"], '2': ["###", "..#", "###", "#..", "###"],
    '7': ["###", "..#", ".#.", ".#.", ".#."], '1': [".#.", "##.", ".#.", ".#.", "###"],
}


def new_year_eve(seed=20, T=8.0):
    fw = R.Fireworks(seed)
    text = "2027"
    out = []
    for t in frames(T):
        e = Eye(top=0.0, bot=6.2, bot_arc=3.4 + 0.6 * (0.5 + 0.5 * math.sin(t * math.tau * 2.4)), gy=-1, bot_lid_dark=True)
        f = Frame()
        put_face(f, e, e, Mouth(curve=1.0))
        if 2.0 < t < 7.2:     # mouth flips to the year, digits drop in one by one
            f.mouth[:] = 0
            for i, ch in enumerate(text):
                appear = 2.0 + i * 0.35
                if t < appear:
                    continue
                dy = max(0, int((appear + 0.25 - t) * 20))
                x0 = 5 + i * 4
                c = [col(25, 4, 2), col(25, 18, 0), col(4, 22, 6), col(6, 12, 25)][i]
                sparkle = 1.0 if int(t * 6 + i) % 5 else 0.55
                for y, row in enumerate(DIGITS[ch]):
                    for x, chh in enumerate(row):
                        yy = y - dy
                        if chh == '#' and 0 <= yy < 5:
                            f.mouth[yy, x0 + x] = c * sparkle
        f.rear_l, f.rear_r = fw(t)
        out.append(f)
    return out


# name: (fn, group, title, note, legacy manifest it replaces or None)
CLIPS = {
    'idle_stare': (idle_stare, 'idle', 'Покой · «взгляд»', 'моргания 2–6 с, двойные, саккады, осмотр, ухмылка', 'idle'),
    'idle_bored': (idle_bored, 'idle', 'Покой · «скучно»', 'через минуту тишины: веки ниже, зевок', 'idle'),
    'listening': (listening, 'listening', 'Слушает', 'веки поднимаются, зрачки на говорящего (DOA), кивок', None),
    'thinking_teeth': (thinking_teeth, 'thinking', 'Думает · «зубы-загрузка»', 'огонёк бежит по зубам, прищур', 'thinking'),
    'thinking_orbit': (thinking_orbit, 'thinking', 'Думает · «орбита»', 'зрачки по кругу, прикус наискось «хмм»', 'thinking'),
    'talk_jaw': (lambda: TK.talking('jaw'), 'talking', 'Говорит · «челюсть»', 'решётка на месте, проём линзой, челюсть падает сильнее губы, тёплая «глотка»', 'talking'),
    'talk_rubber': (lambda: TK.talking('rubber'), 'talking', 'Говорит · «резина»', 'сжатие-растяжение: закрыт — широкая щель, открыт — уже и выше', 'talking'),
    'talk_voicebox': (lambda: TK.talking('voicebox'), 'talking', 'Говорит · «голос-решётка»', 'зубы светятся от центра наружу по громкости, как у KITT', 'talking'),
    'talk_happy': (lambda: TK.talking('jaw', mood='happy'), 'talking', 'Говорит · радостно', 'глаза ^^, улыбка в паузах', 'talking'),
    'happy': (happy, 'emotion', 'Радость', 'глаза ^^, хихиканье', 'happy'),
    'wink': (wink, 'emotion', 'Подмигнул', 'одним глазом + ухмылка', 'happy'),
    'sad': (sad, 'emotion', 'Грусть', 'веки домиком, зрачки вниз, слеза', 'sad'),
    'angry': (angry, 'emotion', 'Злость', 'веки клином, краснеет, скрежет зубами', 'angry'),
    'surprised': (surprised, 'emotion', 'Удивление', 'моргнул — глаза распахнуты, зрачок-точка, «О»', 'surprised'),
    'victory': (V.victory, 'emotion', 'Победа', 'глаза ^^ прыгают, радуга по зубам, салют и конфетти сзади', 'victory'),
    'music_groove': (music_groove, 'music', 'Музыка · «грув»', 'кивок на долю, эквалайзер в зубах, кольца-динамики', None),
    'music_shades': (music_shades, 'music', 'Музыка · «очки»', 'очки съезжают, блик раз в такт, диско', None),
    'music_vibe': (music_vibe, 'music', 'Музыка · «кайф»', 'глаза ^^, тёплая волна по зубам, эквалайзер сзади', None),
    'police_lights': (V.police, 'emergency', 'Полиция', 'Бендер-патрульный: строгие глаза в отсветах, рот — балка: двойные вспышки, разбег, чередование', 'police_lights'),
    'fire_truck': (V.fire_truck, 'emergency', 'Пожарная', 'глаза — вращающиеся маяки, балка красно-белая, потом «огонь» во рту', 'fire_truck'),
    'ambulance': (V.ambulance, 'emergency', 'Скорая', 'красный крест бьётся в такт пульсу, во рту кардиограмма, синие вспышки', 'ambulance'),
    'road_service': (V.road_service, 'emergency', 'Дорожная служба', 'жёлтые маяки, во рту табло со стрелками, сзади стрелки', 'road_service'),
    'turn_left': (lambda: V.turn('left'), 'driving', 'Поворот налево', 'глаз со стороны поворота мигает жёлтым, бегущая заливка сзади', 'turn_left'),
    'turn_right': (lambda: V.turn('right'), 'driving', 'Поворот направо', 'то же зеркально', 'turn_right'),
    'brake': (brake, 'driving', 'Торможение', 'стоп-сигнал, прищур', 'braking'),
    'accelerating': (V.accelerating, 'driving', 'Разгон', 'прищур, сжатые зубы, дрожь, сзади форсаж', 'accelerating'),
    'sleep': (sleep, 'system', 'Сон', 'веки «︶» дышат, z, похрапывает', 'sleep'),
    'wakeup': (wakeup, 'system', 'Пробуждение', 'ЭЛТ-включение, зубы загружаются, зевок', 'wakeup'),
    'error': (error, 'system', 'Ошибка', 'глаза-крестики, глитч', 'error'),
    'low_battery': (low_battery, 'system', 'Батарея садится', 'сонные веки, батарейка во рту', 'low_battery'),
    'charging': (charging, 'system', 'Зарядка', 'довольные глаза, батарейка наполняется', 'charging'),
    'new_year_eve': (new_year_eve, 'holiday', 'Новогодняя полночь', 'салют сзади, «2027» во рту', None),
}
