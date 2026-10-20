"""Rear lights (2x 8x8): taillight, driving signals, holiday themes, music."""
from __future__ import annotations

import math
import random

import numpy as np

from face import col, clamp

_YY, _XX = np.mgrid[0:8, 0:8]
_D = np.hypot(_XX - 3.5, _YY - 3.5)


def sprite(rows, pal):
    out = np.zeros((8, 8, 3))
    for y, row in enumerate(rows):
        for x, ch in enumerate(row):
            if ch in pal:
                out[y, x] = pal[ch]
    return out


# ------------------------------------------------------------- taillight
def taillight(t, brake=0.0):
    ring = (_D > 2.6) & (_D <= 3.95)
    core = _D <= 2.6
    out = np.zeros((8, 8, 3))
    out[ring] = col(12, 0, 0)
    out[core] = col(3, 0, 0) * (1 - brake) + col(25, 1, 0) * brake
    if brake > 0:
        out[ring] = col(12, 0, 0) * (1 - brake) + col(25, 3, 0) * brake
    return out, out[:, ::-1].copy()


def turn_signal(t, side, period=0.75):
    """Sequential amber sweep (Audi-style) from the inner edge outward."""
    p = (t % period) / period
    tail_l, tail_r = taillight(t)
    sweep = np.zeros((8, 8, 3))
    if p < 0.6:
        k = int(p / 0.45 * 8) if p < 0.45 else 8      # columns lit, then hold
        disc = _D <= 3.95
        for x in range(8):
            if 7 - x < k:
                sweep[disc[:, x], x] = col(25, 10, 0)
    if side == 'left':
        lit = sweep.sum(axis=2) > 0
        tail_l = tail_l * 0.35
        tail_l[lit] = sweep[lit]
    else:
        sweep = sweep[:, ::-1]
        lit = sweep.sum(axis=2) > 0
        tail_r = tail_r * 0.35
        tail_r[lit] = sweep[lit]
    return tail_l, tail_r


# ------------------------------------------------------------- new year
TREE = [
    "...SS...",
    "...GG...",
    "..GGGG..",
    "..GGGG..",
    ".GGGGGG.",
    ".GGGGGG.",
    "GGGGGGGG",
    "...TT...",
]
TREE_BULBS = [(2, 2), (3, 5), (4, 2), (5, 4), (5, 6), (6, 1), (6, 3), (6, 6), (4, 5), (2, 5)]
BULB_COLORS = [col(25, 2, 2), col(25, 15, 0), col(4, 6, 25), col(25, 3, 16), col(2, 22, 22)]


def new_year(t, rng_seed=7):
    pal = {'G': col(0, 9, 1), 'T': col(7, 3, 0), 'S': col(25, 18, 0)}
    outs = []
    for side in (0, 1):
        img = sprite(TREE, pal)
        # star twinkle
        s = 0.65 + 0.35 * math.sin(t * 5.3 + side)
        img[0, 3] = img[0, 4] = col(25, 19, 2) * s
        if (int(t * 3) + side) % 7 == 0:
            img[0, 3] = img[0, 4] = col(25, 25, 18)
        # garland: chasing bulbs
        for i, (y, x) in enumerate(TREE_BULBS):
            phase = (t * 2.2 - i * 0.37 + side * 0.5) % 1.0
            c = BULB_COLORS[(i + int(t * 0.8)) % len(BULB_COLORS)]
            k = 1.0 if phase < 0.55 else 0.25
            img[y, x] = c * k
        # snow in the empty corners
        r = random.Random(rng_seed + side)
        for f in range(7):
            x = r.randrange(8)
            speed = r.uniform(1.2, 2.4)
            y = int((t * speed + r.uniform(0, 8)) % 9) - 1
            xx = (x + int(math.sin(t * 1.3 + f) * 0.8 + 0.5)) % 8
            if 0 <= y < 8 and img[y, xx].sum() == 0:
                img[y, xx] = col(9, 10, 12)
        outs.append(img)
    return outs[0], outs[1]


# ------------------------------------------------------------- halloween
PUMPKIN = [
    "...gg...",
    ".OOOOOO.",
    "OYOOOOYO",
    "OYYOOYYO",
    "OOOOOOOO",
    "OYYYYYYO",
    ".OYOOYO.",
    "..OOOO..",
]
PUMPKIN_SQUINT = [
    "...gg...",
    ".OOOOOO.",
    "OOOOOOOO",
    "OYYOOYYO",
    "OOOOOOOO",
    "OYYYYYYO",
    ".OYOOYO.",
    "..OOOO..",
]


def halloween(t, rng_seed=3):
    outs = []
    for side in (0, 1):
        r = random.Random(int(t * 12) * 31 + side)
        flick = 0.6 + 0.4 * (0.5 + 0.5 * math.sin(t * 11 + side * 2)) * r.uniform(0.75, 1.0)
        pal = {'O': col(5, 1, 0), 'g': col(2, 9, 0), 'Y': col(25, 21, 5) * flick}
        squint = ((t + side * 1.7) % 4.3) < 0.2
        img = sprite(PUMPKIN_SQUINT if squint else PUMPKIN, pal)
        # rim darker for a round silhouette
        for (y, x) in ((1, 1), (1, 6), (7, 2), (7, 5), (2, 0), (5, 0), (2, 7), (5, 7)):
            if img[y, x].sum() > 0 and img[y, x][1] < 5:
                img[y, x] = col(3, 0, 0)
        outs.append(img)
    return outs[0], outs[1]


# ------------------------------------------------------------- valentine
HEART_BIG = [
    ".RR..RR.",
    "RRRRRRRR",
    "RRRRRRRR",
    "RRRRRRRR",
    ".RRRRRR.",
    "..RRRR..",
    "...RR...",
    "........",
]
HEART_SMALL = [
    "........",
    "..R..R..",
    ".RRRRRR.",
    ".RRRRRR.",
    "..RRRR..",
    "...RR...",
    "........",
    "........",
]


def valentine(t):
    outs = []
    for side in (0, 1):
        p = (t * 1.25 + side * 0.08) % 1.0     # 75 bpm heartbeat
        big = p < 0.12 or 0.24 < p < 0.36       # lub-dub
        pal = {'R': col(25, 0, 3) if big else col(14, 0, 2)}
        img = sprite(HEART_BIG if big else HEART_SMALL, pal)
        img[1 if big else 2, 1 if big else 2] = col(25, 12, 14)
        outs.append(img)
    return outs[0], outs[1]


# ------------------------------------------------------------- red star (23.02)
STAR = [
    "...RR...",
    "...RR...",
    "RRRRRRRR",
    ".RRRRRR.",
    "..RRRR..",
    ".RRRRRR.",
    ".RR..RR.",
    "RR....RR",
]


def red_star(t):
    outs = []
    for side in (0, 1):
        img = sprite(STAR, {'R': col(20, 0, 0)})
        sweep = ((t * 0.6 + side * 0.15) % 1.6) * 10 - 2   # diagonal shine
        for y in range(8):
            for x in range(8):
                if img[y, x].sum() > 0:
                    d = abs((x + y) - sweep)
                    if d < 0.8:
                        img[y, x] = col(25, 14, 6)
                    elif d < 1.8:
                        img[y, x] = col(25, 4, 1)
        outs.append(img)
    return outs[0], outs[1]


# ------------------------------------------------------------- tulips (08.03)
TULIP = [
    "..P.P...",
    "..PPP...",
    "..PPP...",
    "...G....",
    ".L.G....",
    "..LG.L..",
    "...GL...",
    "...G....",
]


def tulips(t):
    outs = []
    for side, petal in ((0, col(25, 2, 9)), (1, col(25, 9, 0))):
        img = sprite(TULIP, {'P': petal, 'G': col(1, 10, 1), 'L': col(2, 14, 1)})
        sway = math.sin(t * 1.6 + side * 1.4)
        if sway > 0.55:
            img[0:3] = np.roll(img[0:3], 1, axis=1)
        elif sway < -0.55:
            img[0:3] = np.roll(img[0:3], -1, axis=1)
        img = np.roll(img, 2, axis=1) if side == 1 else np.roll(img, 1, axis=1)
        outs.append(img)
    return outs[0], outs[1]


# ------------------------------------------------------------- rocket (12.04)
ROCKET = [            # flying up-right, like the emoji
    "......RR",
    ".....WWR",
    "....WBW.",
    "...WWW..",
    ".RWWW...",
    ".RWW....",
    "..RR....",
    "........",
]


def rocket(t):
    outs = []
    for side in (0, 1):
        img = np.zeros((8, 8, 3))
        r = random.Random(side * 13)
        for i in range(8):                       # stars stream down-left
            sp = r.uniform(3, 7)
            off = r.uniform(0, 16)
            p = (t * sp + off) % 16
            x = int(7 - p + r.randrange(0, 8)) % 8
            y = int(p * 0.5 + r.randrange(0, 8)) % 8
            img[y, x] = col(4, 4, 6) if sp < 5 else col(10, 10, 13)
        rk = sprite(ROCKET, {'W': col(17, 17, 19), 'B': col(2, 10, 25), 'R': col(22, 2, 0)})
        mask = rk.sum(axis=2) > 0
        img[mask] = rk[mask]
        fl = random.Random(int(t * 15) + side)
        flame = [((6, 0), col(25, 7, 0)), ((7, 0), col(18, 2, 0)), ((6, 1), col(25, 20, 4)), ((7, 1), col(25, 9, 0))]
        img[5, 0] = col(25, 20, 4)
        for (y, x), c in flame:
            if fl.random() < 0.8:
                img[y, x] = c
        outs.append(img)
    return outs[0], outs[1]


# ------------------------------------------------------------- fireworks (09.05 / NY eve)
class Fireworks:
    def __init__(self, seed=1):
        self.rng = random.Random(seed)
        self.parts = []
        self.next_t = 0.0
        self.last_t = 0.0

    def __call__(self, t):
        dt = t - self.last_t
        self.last_t = t
        if t >= self.next_t:
            side = self.rng.randrange(2)
            cx = self.rng.uniform(1.5, 5.5)
            cy = self.rng.uniform(1.5, 4.0)
            c = self.rng.choice([col(25, 4, 2), col(25, 18, 0), col(4, 22, 6), col(6, 10, 25), col(25, 6, 22), col(25, 25, 22)])
            for i in range(14):
                a = i / 14 * math.tau + self.rng.uniform(-0.2, 0.2)
                sp = self.rng.uniform(2.6, 4.2)
                self.parts.append([side, cx, cy, math.cos(a) * sp, math.sin(a) * sp, 1.0, c])
            self.next_t = t + self.rng.uniform(0.45, 1.1)
        imgs = [np.zeros((8, 8, 3)), np.zeros((8, 8, 3))]
        alive = []
        for p in self.parts:
            p[1] += p[3] * dt
            p[2] += p[4] * dt
            p[4] += 4.0 * dt          # gravity
            p[3] *= 0.92
            p[4] *= 0.95
            p[5] -= dt * 0.9
            if p[5] <= 0:
                continue
            x, y = int(round(p[1])), int(round(p[2]))
            if 0 <= x < 8 and 0 <= y < 8:
                imgs[p[0]][y, x] = np.maximum(imgs[p[0]][y, x], p[6] * clamp(p[5] * 1.3, 0, 1))
            alive.append(p)
        self.parts = alive
        return imgs[0], imgs[1]


# ------------------------------------------------------------- birthday
CAKE = [
    "........",
    ".c.c.c..",
    ".w.w.w..",
    "PPPPPPP.",
    "CsCCsCC.",
    "CCsCCsC.",
    "CCCCCCC.",
    "........",
]


def birthday(t):
    outs = []
    for side in (0, 1):
        img = sprite(CAKE, {'w': col(18, 18, 18), 'P': col(25, 6, 14), 'C': col(10, 4, 1), 's': col(3, 14, 22)})
        img = np.roll(img, 1 if side else 0, axis=1)
        for i, x in enumerate((1, 3, 5)):
            x += 1 if side else 0
            r = random.Random(int(t * 14) * 7 + i + side * 3)
            img[1, x] = col(25, 14 + r.randrange(-3, 6), 1) * r.uniform(0.6, 1.0)
            if r.random() < 0.3:
                img[0, x] = col(18, 6, 0)
        outs.append(img)
    return outs[0], outs[1]


# ------------------------------------------------------------- music
def speaker_rings(t, bpm, hue_shift=0.0):
    beat = t * bpm / 60.0
    ph = beat % 1.0
    bar = int(beat // 4)
    palette = [col(25, 3, 12), col(2, 14, 25), col(25, 14, 0), col(4, 22, 8)]
    c = palette[bar % 4]
    out = np.zeros((8, 8, 3))
    r = ph * 5.5
    ring = np.abs(_D - r) < 0.7
    out[ring] = c * clamp(1.15 - ph, 0, 1)
    core = _D < 1.5
    out[core] = c * (1.0 if ph < 0.15 else 0.35)
    return out, out[:, ::-1].copy()


def eq_bars(t, bpm, seed=5):
    beat = t * bpm / 60.0
    ph = beat % 1.0
    kick = math.exp(-ph * 5)
    outs = []
    for side in (0, 1):
        img = np.zeros((8, 8, 3))
        for b in range(8):
            band = b if side == 0 else 7 - b
            lo = 1 - band / 7
            r = random.Random(int(t * 10) * 17 + band * 3 + side)
            h = lo * (2 + kick * 6.5) + (1 - lo) * (3 + 4.5 * r.random()) * (0.7 + 0.3 * math.sin(t * 3 + band))
            h = int(clamp(h, 1, 8))
            for k in range(h):
                y = 7 - k
                c = col(2, 18, 6) if k < 4 else (col(25, 16, 0) if k < 6 else col(25, 2, 2))
                img[y, b] = c
        outs.append(img)
    return outs[0], outs[1]


def disco(t, bpm):
    beat = t * bpm / 60.0
    outs = []
    for side in (0, 1):
        img = np.zeros((8, 8, 3))
        ang = beat * math.pi / 2 * (1 if side else -1)
        for y in range(8):
            for x in range(8):
                a = math.atan2(y - 3.5, x - 3.5) + ang
                sector = int(((a % math.tau) / math.tau) * 6)
                d = _D[y, x]
                if d <= 3.95:
                    pal = [col(25, 0, 10), col(25, 12, 0), col(18, 22, 0), col(0, 20, 8), col(0, 9, 25), col(12, 0, 25)]
                    k = 0.9 if (sector + int(beat)) % 2 == 0 else 0.25
                    img[y, x] = pal[sector] * k
        outs.append(img)
    return outs[0], outs[1]


THEMES = {
    'default': lambda t: taillight(t),
    'new_year': new_year,
    'halloween': halloween,
    'valentine': valentine,
    'defender_day': red_star,
    'womens_day': tulips,
    'cosmonautics': rocket,
    'birthday': birthday,
}
