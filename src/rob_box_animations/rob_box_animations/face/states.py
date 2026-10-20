"""Face scenes: generators of Frames driven by ``ticks`` (see ``scene.py``).

Every scene is ``fn(ticks, ..., T)``: setup for one pass of ``T`` seconds, then
``for t in ticks: ... yield f``. Live values (speaker direction, mouth level,
beat) come from ``ticks.ctx``. Rear is filled by the scene runner unless the
scene owns it (music, driving)."""
from __future__ import annotations

import math
import random

import numpy as np

from . import rear as R
from . import talk as TK
from . import vehicles as V
from .scene import SceneSpec
from .face import (EYE, EYE_RIM, TEETH, TEETH_HI, Eye, Frame,
                   Mouth, Track, apply_blink, blink_curve, clamp, col, draw_eye,
                   draw_mouth, ease_io, gaze_at, gaze_px, lerp, schedule_blinks,
                   schedule_saccades, tooth_pixels, tooth_ring)


def put_face(f: Frame, el: Eye, er: Eye, m: Mouth, hl=None, tint=None):
    draw_eye(f.eye_l, el, 'l')
    draw_eye(f.eye_r, er, 'r')
    draw_mouth(f.mouth, m, hl, tint)


def blinked(e, t, blinks, dur=0.22):
    k = max((blink_curve(t, b, dur) for b in blinks), default=0.0)
    return apply_blink(e, k)


# ============================================================ IDLE
def idle_stare(ticks, seed=1, T=16.0):
    rng = random.Random(seed)
    blinks = schedule_blinks(rng, T)
    sac = schedule_saccades(rng, T)
    look_t = 7.0
    smirk = Track(0.0).hold(10.2).to(10.5, 1.0).hold(11.6).to(12.0, 0.0)
    for t in ticks:
        gx, gy = gaze_at(sac, t)
        # slow look-around: left ... right ... back
        if look_t <= t < look_t + 2.6:
            p = (t - look_t) / 2.6
            gx = -2.6 * math.sin(p * math.pi * 2) if p < 0.5 else 2.6 * math.sin((p - 0.5) * math.pi * 2)
            gx = round(gx)
            gy = 0
        if ticks.ctx.gaze_deg is not None:      # someone speaks: look at them, keep small saccades
            gx = round(gaze_px(ticks.ctx.gaze_deg) + gx * 0.25)
        e = Eye(top=1.0, gx=gx, gy=gy)
        s = smirk(t)
        f = Frame()
        put_face(f, blinked(e, t, blinks), blinked(e, t, blinks), Mouth(smirk=1.0 if s > 0.5 else 0.0))
        yield f


def yawn_shape(y):
    return 'C' if y < 0.2 else ('O1' if y < 0.45 else ('A2' if y < 0.75 else 'A3'))


def idle_bored(ticks, seed=2, T=14.0):
    rng = random.Random(seed)
    blinks = [b for b in schedule_blinks(rng, T, 3.5, 7.0, 0.0) if not (5.0 < b < 8.8)]
    drift = Track(0.0).to(2.0, -2.0).hold(4.0).to(5.0, 1.0).hold(9.5).to(11.0, 2.5).hold(13.0).to(14.0, 0.0)
    yawn = Track(0.0).hold(5.4).to(6.4, 1.0).hold(7.4).to(8.2, 0.0)
    for t in ticks:
        y = yawn(t)
        e = Eye(top=lerp(2.8, 4.3, y), gx=round(drift(t)), gy=0.6, bright=0.9)
        m = Mouth(shape=yawn_shape(y), curve=-1.0 if y < 0.1 else 0.0)
        f = Frame()
        put_face(f, blinked(e, t, blinks, 0.4), blinked(e, t, blinks, 0.4), m)
        yield f


# ============================================================ LISTENING
def listening(ticks, seed=3, T=9.0):
    rng = random.Random(seed)
    blinks = schedule_blinks(rng, T, 3.0, 6.0, 0.0)
    perk = Track(1.0).to(0.18, -0.2).to(0.35, 0.0)
    # preview: the speaker moves from viewer-right to viewer-left at t=4.5;
    # live, the target is the DOA of the speaker (ticks.ctx.gaze_deg)
    target = Track(1.6).hold(4.5).to(4.62, -1.6)
    nods = [2.2, 6.4]
    for t in ticks:
        jitter = 0.5 if (int(t * 1.3) % 3 == 1) else 0.0
        nod = max((math.sin(math.pi * (t - n) / 0.4) for n in nods if n <= t < n + 0.4), default=0.0)
        gx = target(t) if ticks.ctx.gaze_deg is None else gaze_px(ticks.ctx.gaze_deg)
        e = Eye(top=max(0.0, perk(t)) + 1.4 * nod, gx=round(gx + jitter), gy=-0.4)
        f = Frame()
        put_face(f, blinked(e, t, blinks), blinked(e, t, blinks), Mouth())
        yield f


# ============================================================ THINKING
def thinking_teeth(ticks, seed=4, T=8.0):
    rng = random.Random(seed)
    blinks = schedule_blinks(rng, T, 2.5, 4.5, 0.0)
    ring = tooth_ring()
    for t in ticks:
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
        yield f


def thinking_orbit(ticks, seed=5, T=8.0):
    for t in ticks:
        a = t * math.tau / 1.3
        gx, gy = round(math.cos(a) * 1.6), round(math.sin(a) * 1.4)
        e = Eye(top=0.6, gx=gx, gy=gy)
        tilt = math.sin(t * math.tau / 2.6)
        m = Mouth(tilt=0.9 if tilt > 0.35 else (-0.9 if tilt < -0.35 else 0.0))
        f = Frame()
        put_face(f, e, e, m)
        yield f


# ============================================================ EMOTIONS
def happy(ticks, seed=7, T=6.0):
    k = Track(0.0).to(0.25, 1.0)
    for t in ticks:
        kk = k(t)
        giggle = t < 2.2 and (t * 5.0) % 1.0 < 0.5
        squeeze = 0.5 + 0.5 * math.sin(t * math.tau * 2.5) if t < 2.2 else 0.0
        e = Eye(top=0.0, bot=lerp(7, 6.2, kk), bot_arc=3.4 * kk + 0.7 * squeeze, gx=0, gy=-1, bot_lid_dark=True)
        m = Mouth(curve=1.0, shape='E1' if giggle else 'C')
        f = Frame()
        put_face(f, e, e, m)
        yield f


def wink(ticks, seed=8, T=5.0):
    w = Track(0.0).hold(0.8).to(0.95, 1.0).hold(1.5).to(1.7, 0.0)
    for t in ticks:
        el = Eye(top=0.5, gx=1, gy=0)
        er = apply_blink(Eye(top=0.5, gx=1, gy=0, bot_arc=1.5 * w(t)), w(t))
        f = Frame()
        s = max(w(t), 1.0 if 0.9 < t < 3.5 else 0.0)
        put_face(f, el, er, Mouth(smirk=1.0 if s > 0.3 else 0.0))
        yield f


def sad(ticks, seed=9, T=7.0):
    blinks = [1.2, 4.6]
    droop = Track(0.0).to(0.6, 1.0)
    for t in ticks:
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
        yield f


def angry(ticks, seed=10, T=6.0):
    k = Track(0.0).to(0.2, 1.0)
    red, red_rim = col(25, 5, 0), col(13, 1, 0)
    for t in ticks:
        kk = k(t)
        e = Eye(top=lerp(1.0, 1.6, kk), slant=2.3 * kk, gy=0.6, gx=0,
                color=EYE * (1 - kk) + red * kk, rim=EYE_RIM * (1 - kk) + red_rim * kk)
        grind = (int(t * 12) % 2) if (t < 1.2 or 3.4 < t < 3.9) else 0
        m = Mouth(curve=-1.0 if kk > 0.5 else 0.0, color=TEETH * (1 - kk) + col(25, 25, 22) * kk, x_shift=grind, grid=0.0)
        f = Frame()
        put_face(f, e, e, m)
        yield f


def surprised(ticks, seed=11, T=5.0):
    for t in ticks:
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
        yield f


# ============================================================ SLEEP / WAKE
Z = ["###", ".#.", "###"]


SLEEP_EYE = [(3, 0), (3, 7), (4, 1), (4, 6), (5, 2), (5, 3), (5, 4), (5, 5)]   # "︶"


def sleep(ticks, seed=12, T=9.0):
    for t in ticks:
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
        yield f


def wakeup(ticks, seed=13, T=6.0):
    open_ = Track(0.0).hold(0.6).to(0.75, 0.15).hold(1.0).to(1.35, 1.1).to(1.5, 1.0)
    look = Track(0.0).hold(1.8).to(1.9, -2.0).hold(2.5).to(2.6, 2.0).hold(3.1).to(3.2, 0.0)
    yawn = Track(0.0).hold(3.6).to(4.3, 1.0).hold(4.9).to(5.4, 0.0)
    blinks = [3.25, 3.55]
    for t in ticks:
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
            f.eye_l[:] = 0
            f.eye_r[:] = 0
            k = t / 0.6
            for arr in (f.eye_l, f.eye_r):
                w = int(1 + k * 3.5)
                arr[3:5, 4 - w:4 + w] = col(25, 21, 9) * (0.3 + 0.7 * k)
        yield f


# ============================================================ MUSIC
def beat_time(ticks, t, bpm):
    """(bpm, beat clock in seconds): live tempo and phase of the playing track
    from ``ticks.ctx`` when known, else counted from the scene start (preview)."""
    c = ticks.ctx
    bpm = c.bpm or bpm
    if c.beat_phase is None:
        return bpm, t
    return bpm, c.beat_phase * 60.0 / bpm


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


def music_groove(ticks, seed=14, T=8.0, bpm=118):
    accents = [col(25, 3, 12), col(2, 14, 25), col(25, 14, 0), col(4, 22, 8)]
    for t in ticks:
        bpm_, tb = beat_time(ticks, t, bpm)
        beat = tb * bpm_ / 60.0
        ph = beat % 1.0
        bar = int(beat // 4)
        side = 1 if int(beat // 2) % 2 == 0 else -1
        e = Eye(top=1.0 + 0.9 * math.exp(-ph * 9), gx=1.6 * side, gy=0)
        f = Frame()
        put_face(f, e, e, Mouth())
        eq_mouth(f, tb, bpm_, accents[bar % 4], seed)
        f.rear_l, f.rear_r = R.speaker_rings(tb, bpm_)
        yield f


def music_shades(ticks, seed=15, T=8.0, bpm=118):
    for t in ticks:
        bpm_, tb = beat_time(ticks, t, bpm)
        beat = tb * bpm_ / 60.0
        ph = beat % 1.0
        drop = clamp(t / 0.7, 0, 1)
        bar_ph = (beat % 4) / 4
        glint = bar_ph * 16 - 4 if bar_ph < 0.6 else -1
        e = Eye(top=1.0, shades=drop, glint=glint if drop >= 1 else -1)
        m = Mouth(curve=1.0, smirk=1.0, shape='E1' if ph < 0.15 else 'C')
        f = Frame()
        put_face(f, e, e, m)
        f.rear_l, f.rear_r = R.disco(tb, bpm_)
        yield f


def music_vibe(ticks, seed=16, T=8.0, bpm=96):
    for t in ticks:
        bpm_, tb = beat_time(ticks, t, bpm)
        beat = tb * bpm_ / 60.0
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
        f.rear_l, f.rear_r = R.eq_bars(tb, bpm_)
        yield f


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


def error(ticks, seed=17, T=5.0):
    for t in ticks:
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
        yield f


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


def low_battery(ticks, seed=18, T=6.0):
    for t in ticks:
        e = Eye(top=3.0, gy=1, bright=0.7)
        f = Frame()
        put_face(f, blinked(e, t, [1.5, 4.0], 0.6), blinked(e, t, [1.5, 4.0], 0.6), Mouth())
        battery_mouth(f, 1 / 8, col(25, 1, 0), (t % 1.0) < 0.55)
        yield f


def charging(ticks, seed=19, T=6.0):
    for t in ticks:
        e = Eye(top=0.0, bot=6.0, bot_arc=3.0, gy=-1, bright=0.6, bot_lid_dark=True)
        f = Frame()
        put_face(f, e, e, Mouth())
        lvl = 0.375 + ((t % 2.0) / 2.0) * 0.625
        battery_mouth(f, lvl, col(2, 22, 4))
        yield f


# ============================================================ DRIVING
def brake(ticks, T=4.0):
    for t in ticks:
        k = 1.0 if (t % 2.0) < 1.4 else 0.0
        e = Eye(top=1.0 + 0.8 * k, gy=0)
        f = Frame()
        put_face(f, e, e, Mouth(curve=-1.0 * k))
        f.rear_l, f.rear_r = R.taillight(t, brake=k)
        yield f


# ============================================================ HOLIDAY celebrate
DIGITS = {
    '0': ["###", "#.#", "#.#", "#.#", "###"], '2': ["###", "..#", "###", "#..", "###"],
    '7': ["###", "..#", ".#.", ".#.", ".#."], '1': [".#.", "##.", ".#.", ".#.", "###"],
}


def new_year_eve(ticks, seed=20, T=8.0):
    fw = R.Fireworks(seed)
    text = "2027"
    for t in ticks:
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
        yield f


def _s(fn, period, group, title, note, **kwargs):
    return SceneSpec(fn, period, group, title, note, kwargs)


#: Every scene the face can show: name -> generator, length of one pass, lab metadata.
SCENES = {
    'idle_stare': _s(idle_stare, 16.0, 'idle', 'Покой · «взгляд»', 'моргания 2–6 с, двойные, саккады, осмотр, ухмылка; взгляд на говорящего по DOA'),
    'idle_bored': _s(idle_bored, 14.0, 'idle', 'Покой · «скучно»', 'через минуту тишины: веки ниже, зевок'),
    'listening': _s(listening, 9.0, 'listening', 'Слушает', 'веки поднимаются, зрачки на говорящего (DOA), кивок'),
    'thinking_teeth': _s(thinking_teeth, 8.0, 'thinking', 'Думает · «зубы-загрузка»', 'огонёк бежит по зубам, прищур'),
    'thinking_orbit': _s(thinking_orbit, 8.0, 'thinking', 'Думает · «орбита»', 'зрачки по кругу, прикус наискось «хмм»'),
    'talk_jaw': _s(TK.talking, 10.0, 'talking', 'Говорит · «челюсть»',
                   'решётка на месте, проём линзой, челюсть падает сильнее губы, тёплая «глотка»', design='jaw'),
    'talk_rubber': _s(TK.talking, 10.0, 'talking', 'Говорит · «резина»',
                      'сжатие-растяжение: закрыт — широкая щель, открыт — уже и выше', design='rubber'),
    'talk_voicebox': _s(TK.talking, 10.0, 'talking', 'Говорит · «голос-решётка»',
                        'зубы светятся от центра наружу по громкости, как у KITT', design='voicebox'),
    'talk_happy': _s(TK.talking, 10.0, 'talking', 'Говорит · радостно', 'глаза ^^, улыбка в паузах', design='jaw', mood='happy'),
    'happy': _s(happy, 6.0, 'emotion', 'Радость', 'глаза ^^, хихиканье'),
    'wink': _s(wink, 5.0, 'emotion', 'Подмигнул', 'одним глазом + ухмылка'),
    'sad': _s(sad, 7.0, 'emotion', 'Грусть', 'веки домиком, зрачки вниз, слеза'),
    'angry': _s(angry, 6.0, 'emotion', 'Злость', 'веки клином, краснеет, скрежет зубами'),
    'surprised': _s(surprised, 5.0, 'emotion', 'Удивление', 'моргнул — глаза распахнуты, зрачок-точка, «О»'),
    'victory': _s(V.victory, 6.0, 'emotion', 'Победа', 'глаза ^^ прыгают, радуга по зубам, салют и конфетти сзади'),
    'music_groove': _s(music_groove, 8.0, 'music', 'Музыка · «грув»', 'кивок на долю, эквалайзер в зубах, кольца-динамики'),
    'music_shades': _s(music_shades, 8.0, 'music', 'Музыка · «очки»', 'очки съезжают, блик раз в такт, диско'),
    'music_vibe': _s(music_vibe, 8.0, 'music', 'Музыка · «кайф»', 'глаза ^^, тёплая волна по зубам, эквалайзер сзади'),
    'police_lights': _s(V.police, 7.2, 'emergency', 'Полиция',
                        'Бендер-патрульный: строгие глаза в отсветах, рот — балка: двойные вспышки, разбег, чередование'),
    'fire_truck': _s(V.fire_truck, 7.0, 'emergency', 'Пожарная', 'глаза — вращающиеся маяки, балка красно-белая, потом «огонь» во рту'),
    'ambulance': _s(V.ambulance, 6.0, 'emergency', 'Скорая', 'красный крест бьётся в такт пульсу, во рту кардиограмма, синие вспышки'),
    'road_service': _s(V.road_service, 6.0, 'emergency', 'Дорожная служба', 'жёлтые маяки, во рту табло со стрелками, сзади стрелки'),
    'turn_left': _s(V.turn, 4.0, 'driving', 'Поворот налево', 'глаз со стороны поворота мигает жёлтым, бегущая заливка сзади', side='left'),
    'turn_right': _s(V.turn, 4.0, 'driving', 'Поворот направо', 'то же зеркально', side='right'),
    'brake': _s(brake, 4.0, 'driving', 'Торможение', 'стоп-сигнал, прищур'),
    'accelerating': _s(V.accelerating, 5.0, 'driving', 'Разгон', 'прищур, сжатые зубы, дрожь, сзади форсаж'),
    'sleep': _s(sleep, 9.0, 'system', 'Сон', 'веки «︶» дышат, z, похрапывает'),
    'wakeup': _s(wakeup, 6.0, 'system', 'Пробуждение', 'ЭЛТ-включение, зубы загружаются, зевок'),
    'error': _s(error, 5.0, 'system', 'Ошибка', 'глаза-крестики, глитч'),
    'low_battery': _s(low_battery, 6.0, 'system', 'Батарея садится', 'сонные веки, батарейка во рту'),
    'charging': _s(charging, 6.0, 'system', 'Зарядка', 'довольные глаза, батарейка наполняется'),
    'new_year_eve': _s(new_year_eve, 8.0, 'holiday', 'Новогодняя полночь', 'салют сзади, «2027» во рту'),
}

#: Names of the v1 vocabulary (rob_box_mcp_tools.animations.KNOWN_ANIMATIONS,
#: ``play_animation``) that are not scene names. ``talking``/``thinking``
#: follow the node's ``mouth_design``/``thinking_design`` (decision В13).
LEGACY = {'idle': 'idle_stare', 'braking': 'brake', 'talking': 'talk_{mouth}', 'thinking': '{thinking}'}


def resolve(name, mouth='jaw', thinking='thinking_teeth'):
    """Scene name for a scene or v1 animation name; None when unknown."""
    key = str(name or '').strip().lower()
    key = LEGACY.get(key, key).format(mouth=mouth, thinking=thinking)
    return key if key in SCENES else None
