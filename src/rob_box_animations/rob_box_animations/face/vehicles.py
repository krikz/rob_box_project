"""Emergency and driving scenes in the face's visual language.

The grille doubles as a light bar (each tooth cell is a segment), the eyes
either stay Bender's eyes lit by the flashers or turn into rotating beacons.
Beacons, flashers, flash patterns, the fire and the ECG are ros2leds
``led_matrix_effects.signals``; here they are put on the face.
"""
from __future__ import annotations

import math
import random
from dataclasses import replace

import numpy as np

from led_matrix_effects import lamps as R
from led_matrix_effects.signals import (AMBER, BLUE, CHEVRON, CROSS, DISC, ECG, RED, WHITE, YELLOW, beacon, doom_fire,
                                        fire_color, flasher, police_pattern, roadworks_pattern)
from led_matrix_effects.signals import LAMP_R as _D

from .face import EYE, EYE_RIM, Eye, Frame, Mouth, clamp, col, draw_eye, draw_mouth

CELLS = [(1, 3), (5, 7), (9, 11), (13, 15), (17, 19), (21, 23)]


# ------------------------------------------------------------ building blocks
def lightbar(arr, colors, base_dim=0.15):
    """colors: 6 entries (None = dark segment). Keeps the grille readable."""
    draw_mouth(arr, Mouth(bright=base_dim))
    for k, c in enumerate(colors):
        if c is None:
            continue
        x0, x1 = CELLS[k]
        for y in (0, 1, 3, 4):
            for x in range(x0, x1 + 1):
                if x in (1, 23) and y in (0, 4):
                    continue
                arr[y, x] = c
        arr[2, x0:x1 + 1] = c * 0.35


# ------------------------------------------------------------ police
def police(ticks, T=7.2):
    """'Bender on patrol': stern eyes lit red/blue by the light bar mouth."""
    for t in ticks:
        left, right, segs = police_pattern(t)
        f = Frame()
        base = Eye(top=1.9, slant=1.0, gy=0.4, gx=0)
        el = replace(base, color=EYE * 0.3 + RED * 0.7 if left else EYE * 0.8, rim=EYE_RIM * 0.7)
        er = replace(base, color=EYE * 0.25 + BLUE * 0.75 if right else EYE * 0.8, rim=EYE_RIM * 0.7)
        draw_eye(f.eye_l, el, 'l')
        draw_eye(f.eye_r, er, 'r')
        lightbar(f.mouth, segs)
        f.rear_l = flasher(RED, left)
        f.rear_r = flasher(BLUE, right)
        yield f


# ------------------------------------------------------------ fire truck
def fire_truck(ticks, T=7.0):
    rng = random.Random(4)
    heat = [[0.0] * 25 for _ in range(7)]
    for t in ticks:
        f = Frame()
        f.eye_l = beacon(t, RED, rpm=110)
        f.eye_r = beacon(t, RED, rpm=110, phase=math.pi)
        p = t % 7.0
        if p < 4.0:   # red / white alternating light bar
            k = int(p / 0.125) % 2
            segs = [RED if (i % 2 == k) else WHITE for i in range(6)]
            if int(p / 0.5) % 2:
                segs = [s if i in (0, 5, 2, 3) else None for i, s in enumerate(segs)]
            lightbar(f.mouth, segs)
        else:         # "огонь!" — the grille turns into flames
            doom_fire(heat, rng)
            for y in range(5):
                for x in range(25):
                    f.mouth[y, x] = fire_color(heat[y + 1][x] * (1.15 if 1 <= x <= 23 else 0))
        f.rear_l = beacon(t, RED, rpm=110, phase=math.pi / 2)
        f.rear_r = beacon(t, RED, rpm=110, phase=-math.pi / 2)
        yield f


# ------------------------------------------------------------ ambulance
def ambulance(ticks, T=6.0, bpm=84):
    period = 60 / bpm
    for t in ticks:
        f = Frame()
        beat = (t % period) / period
        pulse = 1.0 if beat < 0.08 else (0.75 if 0.16 < beat < 0.24 else 0.5)
        for arr in (f.eye_l, f.eye_r):
            arr[DISC] = col(18, 18, 20) * 0.6
            arr[(_D > 3.0) & DISC] = col(10, 10, 12)
            for (y, x) in CROSS:
                arr[y, x] = col(25, 0, 1) * pulse
        # ECG trace sweeping across the mouth, monitor green
        speed = 22.0
        head = (t * speed) % 25
        for x in range(25):
            age = (head - x) % 25
            if age > 19:
                continue
            tt = (t - age / speed) / period
            idx = int((tt % 1.0) * len(ECG)) % len(ECG)
            v = ECG[idx]
            yv = 2.6 - v * 2.4
            y0 = int(clamp(round(yv), 0, 4))
            k = (1 - age / 19) ** 1.4
            c = col(25, 25, 25) if age < 1 else col(3, 25, 9)
            f.mouth[y0, x] = c * k
            # vertical stroke for the steep QRS so the line stays connected
            prev = ECG[(idx - 1) % len(ECG)]
            yp = int(clamp(round(2.6 - prev * 2.4), 0, 4))
            for yy in range(min(y0, yp), max(y0, yp) + 1):
                f.mouth[yy, x] = np.maximum(f.mouth[yy, x], col(3, 25, 9) * k * 0.8)
        dbl = int((t % 0.8) / 0.08)
        f.rear_l = flasher(BLUE, dbl in (0, 2))
        f.rear_r = flasher(BLUE, dbl in (5, 7))
        yield f


# ------------------------------------------------------------ road service
def road_service(ticks, T=6.0, direction=1):
    for t in ticks:
        f = Frame()
        f.eye_l = beacon(t, AMBER, rpm=80)
        f.eye_r = beacon(t, AMBER, rpm=80, phase=math.pi)
        shift = int(t * 12) % 6
        for x in range(25):
            for y in range(5):
                xx = (x - shift * direction) % 6
                if xx < 4 and CHEVRON[y][xx] == '#':
                    f.mouth[y, x] = AMBER
        for k, arr in enumerate((f.rear_l, f.rear_r)):      # sequential: left board, then right
            if (t % 0.8) < 0.25 + 0.2 * k or (t % 0.8) > 0.7:
                continue
            for y in range(8):
                dx = int(abs(y - 3.5))
                for w in range(2):
                    x = 5 - dx + w                          # '>' pointing the same way as the mouth board
                    x = x if direction > 0 else 7 - x
                    if 0 <= x < 8:
                        arr[y, x] = AMBER
        yield f


# ------------------------------------------------------------ roadworks beacon
def roadworks_beacon(ticks, T=4.8):
    """'Roadworks': amber/yellow strobe beacons instead of the eyes and rear lights, the grille is a yellow-black
    hazard board crawling slowly; the flashing side of the board lights up with its beacon. Proximity alarm (#4365)."""
    for t in ticks:
        left, right = roadworks_pattern(t)
        f = Frame()
        f.eye_l = flasher(AMBER, left)
        f.eye_r = flasher(YELLOW, right)
        shift = int(t * 6)
        for y in range(5):
            for x in range(25):
                if x == 12 or ((x + y + shift) // 3) % 2:
                    continue
                side_on = left if x < 12 else right
                f.mouth[y, x] = (AMBER if x < 12 else YELLOW) * (1.0 if side_on else 0.2)
        f.rear_l = flasher(AMBER, left)
        f.rear_r = flasher(YELLOW, right)
        yield f


# ------------------------------------------------------------ driving
def accelerating(ticks, T=5.0):
    for t in ticks:
        f = Frame()
        e = Eye(top=2.4, slant=0.6, gy=0)
        draw_eye(f.eye_l, e, 'l')
        draw_eye(f.eye_r, e, 'r')
        draw_mouth(f.mouth, Mouth(curve=1.0, grid=0.0, color=col(25, 25, 22)))
        for arr, seed in ((f.rear_l, 1), (f.rear_r, 2)):
            r = random.Random(int(t * 30) * 7 + seed)
            ring = (_D > 3.1) & DISC
            arr[ring] = col(5, 5, 6)
            inner = _D <= 3.1
            for y in range(8):
                for x in range(8):
                    if not inner[y, x]:
                        continue
                    d = _D[y, x] / 3.1 + r.uniform(-0.25, 0.25)
                    if d < 0.35:
                        arr[y, x] = col(14, 18, 25)
                    elif d < 0.7:
                        arr[y, x] = col(25, 14, 3)
                    elif d < 1.0:
                        arr[y, x] = col(22, 4, 0) * r.uniform(0.5, 1.0)
        yield f


def turn(ticks, side='left', T=4.0):
    """Robot's left = viewer's right on the face. The eye on the turning side
    blinks amber in sync with the rear sweep, both look into the turn."""
    for t in ticks:
        f = Frame()
        g = 2.0 if side == 'left' else -2.0
        on = (t % 0.75) < 0.45
        e = Eye(top=0.8, gx=g, gy=0)
        amber_eye = replace(e, color=EYE * 0.3 + AMBER * 0.7, rim=AMBER * 0.5) if on else e
        if side == 'left':
            draw_eye(f.eye_l, e, 'l')
            draw_eye(f.eye_r, amber_eye, 'r')
        else:
            draw_eye(f.eye_l, amber_eye, 'l')
            draw_eye(f.eye_r, e, 'r')
        draw_mouth(f.mouth, Mouth())
        f.rear_l, f.rear_r = R.turn_signal(t, side)
        yield f


def victory(ticks, T=6.0):
    fw = R.Fireworks(5)
    for t in ticks:
        f = Frame()
        bounce = (t * 2.5) % 1.0 < 0.25
        squeeze = 0.5 + 0.5 * math.cos(t * math.tau * 2.5)
        e = Eye(top=0.0, bot=6.2, bot_arc=2.9 + 0.8 * squeeze, gy=-1, bot_lid_dark=True)
        draw_eye(f.eye_l, e, 'l')
        draw_eye(f.eye_r, e, 'r')
        draw_mouth(f.mouth, Mouth(curve=1.0, shape='E1' if bounce else 'C'))
        # party grille: a rainbow wave rolls across the teeth
        for x in range(25):
            h = (x / 25 - t * 0.8) % 1.0
            rgb = np.array([abs(h * 6 - 3) - 1, 2 - abs(h * 6 - 2), 2 - abs(h * 6 - 4)])
            rgb = np.clip(rgb, 0, 1) * 25
            lit = f.mouth[:, x].sum(axis=1) > 3
            f.mouth[lit, x] = f.mouth[lit, x] * 0.25 + rgb * 0.75
        f.rear_l, f.rear_r = fw(t)
        r = random.Random(int(t * 12))
        for arr in (f.rear_l, f.rear_r):           # confetti
            for _ in range(3):
                arr[r.randrange(8), r.randrange(8)] = R.BULB_COLORS[r.randrange(len(R.BULB_COLORS))]
        yield f
