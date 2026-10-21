"""Effects on the face canvas (#3780, part A): backgrounds under the features, one-shot pulses, feature swaps.

One slot of ``knowledge.FACE_SCENE_SLOTS`` per kind, the tables below are the renderers of their values:

* ``bg`` (:data:`BACKGROUNDS`) lasts with the scene and lies UNDER the eyes and mouth: ``bg(t) -> Frame`` on the
  canvas of ``face.CANVAS_AT``, no channel above :data:`BG_LV` levels. It runs through the features too: they keep
  their level and take its hue (:func:`under`), the background next to them is dimmed to :data:`UNDER_FEATURES` —
  the features read and the strip stays in the current budget. The rear boards show it instead of their theme.
* ``fx`` (:data:`PULSES`) is one pulse from the scene start (after :data:`PULSE_LEAD_S`): ``fx(t, f) -> Frame``
  takes the composed frame (``glitch`` tears it, ``boot``/``power_down`` mask it, ``slash`` draws over it); past
  its length the frame is the scene's own, only ``power_down`` stays dark — the face has been switched off.
* ``feat`` (:data:`FEATURES`) swaps the features: ``loading_eyes(t, f)`` draws spinners in place of the eyes.

``t`` is seconds since the scene started. Levels 0..25 as everywhere in ``face``; canvas grids are built once at
import, a frame costs a few numpy operations per panel.
"""
from __future__ import annotations

import math
import random
from typing import Callable, Dict, Mapping, Optional, Tuple

import numpy as np

from .face import BLACK, CANVAS_AT, EYE, EYE_RIM_MASK, FPS, GROUP_OF, RAINBOW_PERIOD_S, Frame, clamp, col
from .sides import MARKS

PANELS: Tuple[str, ...] = tuple(CANVAS_AT)


def _grid(panel: str) -> Tuple[np.ndarray, np.ndarray]:
    h, w = getattr(Frame(), panel).shape[:2]
    x0, y0 = CANVAS_AT[panel]
    yy, xx = np.mgrid[0:h, 0:w]
    return (xx + x0).astype(float), (yy + y0).astype(float)


#: Canvas coordinates (X, Y) of every LED of a panel.
XY: Dict[str, Tuple[np.ndarray, np.ndarray]] = {p: _grid(p) for p in PANELS}
CANVAS_W = int(max(XY[p][0].max() for p in PANELS)) + 1
CANVAS_H = int(max(XY[p][1].max() for p in PANELS)) + 1

#: Brightest channel of a background: 6 of 25 levels (24 %) — the eyes (25), their rim (13) and the teeth (17)
#: stay at least twice as bright.
BG_LV = 6.0
#: Background next to a lit feature pixel (4-neighbours) is dimmed to this share: a dark outline around the features.
UNDER_FEATURES = 0.25
#: A feature pixel is lit from this level; dimmer ones (the metal lids, 2) are background.
LIT_LV = 3.0
#: Share of the background's hue a lit feature takes where the background is at full strength.
THROUGH = 0.85


def _canvas(fn: Callable[[np.ndarray, np.ndarray], np.ndarray]) -> Frame:
    f = Frame()
    for p in PANELS:
        setattr(f, p, fn(*XY[p]))
    return f


# ---------------------------------------------------------------- backgrounds
RAINBOW_WAVE_PX = 26.0        # one hue circle along the diagonal of the canvas
RAINBOW_SLANT = 0.8           # rows count this much as columns: the wave goes slantwise
_HUE_AT = np.array((3.0, 2.0, 4.0))


def _hue(h: np.ndarray) -> np.ndarray:
    """Hue 0..1 (any real, wraps) -> RGB 0..1 at full saturation."""
    d = np.abs((h % 1.0)[..., None] * 6.0 - _HUE_AT)
    rgb = 2.0 - d
    rgb[..., 0] = d[..., 0] - 1.0
    return np.clip(rgb, 0.0, 1.0)


def bg_rainbow(t: float) -> Frame:
    """A rainbow wave passing slantwise through the whole canvas (not the whole face recoloured at once)."""
    shift = t / RAINBOW_PERIOD_S
    return _canvas(lambda X, Y: _hue((X + RAINBOW_SLANT * Y) / RAINBOW_WAVE_PX - shift) * BG_LV)


FIRE_BASE, FIRE_SWING = 4.0, 2.6     # flame height in rows and its sway


def bg_fire(t: float) -> Frame:
    """Tongues of flame from the bottom of the canvas: red at the root, a yellow core, every column sways."""
    def fn(X, Y):
        sway = (np.sin(X * 0.9 + t * 7.0) + np.sin(X * 0.37 - t * 4.3) + 0.6 * np.sin(X * 1.7 + t * 11.0)) / 2.6
        height = FIRE_BASE + FIRE_SWING * sway
        heat = np.clip(1.0 - (CANVAS_H - 1 - Y) / np.maximum(height, 0.5), 0.0, 1.0)
        out = np.zeros(X.shape + (3,))
        out[..., 0] = np.clip(heat * 1.6, 0.0, 1.0)
        out[..., 1] = np.clip(heat * 1.6 - 0.7, 0.0, 1.0) * 0.7
        return out * BG_LV
    return _canvas(fn)


MATRIX_TRAIL = 4.0            # rows of a falling drop
MATRIX_CYCLE = CANVAS_H + 9.0  # rows from one drop of a column to the next
_MRNG = np.random.RandomState(3780)
_M_SPEED = _MRNG.uniform(5.0, 11.0, CANVAS_W)      # rows per second, per canvas column
_M_PHASE = _MRNG.uniform(0.0, MATRIX_CYCLE, CANVAS_W)
_M_COLS = {p: (_M_SPEED[XY[p][0].astype(int)], _M_PHASE[XY[p][0].astype(int)]) for p in PANELS}
MATRIX_GREEN = col(0, 1.0, 0.15)
MATRIX_HEAD = col(0.45, 1.0, 0.45)


def bg_matrix(t: float) -> Frame:
    """Green drops of «code» falling down every canvas column, each column at its own speed; the head is paler."""
    f = Frame()
    for p in PANELS:
        speed, phase = _M_COLS[p]
        d = (t * speed + phase) % MATRIX_CYCLE - XY[p][1]
        k = np.where((d >= 0.0) & (d < MATRIX_TRAIL), 1.0 - d / MATRIX_TRAIL, 0.0)[..., None]
        head = ((d >= 0.0) & (d < 1.0))[..., None]
        setattr(f, p, k * BG_LV * np.where(head, MATRIX_HEAD, MATRIX_GREEN))
    return f


#: Slot ``bg`` -> background. ``none`` — no background.
BACKGROUNDS: Mapping[str, Callable[[float], Frame]] = {'rainbow': bg_rainbow, 'fire': bg_fire, 'matrix': bg_matrix}


def _near(lit: np.ndarray) -> np.ndarray:
    n = lit.copy()
    n[1:] |= lit[:-1]
    n[:-1] |= lit[1:]
    n[:, 1:] |= lit[:, :-1]
    n[:, :-1] |= lit[:, 1:]
    return n


def under(f: Frame, bg: Frame) -> Frame:
    """Background through the canvas, features on top. A lit feature pixel keeps its level (the brightest channel)
    and takes the background's hue by :data:`THROUGH` of the background's strength there — the wave passes through
    the eyes too, they stay the brightest; elsewhere the background, dimmed next to the features."""
    for p in PANELS:
        a, b = getattr(f, p), getattr(bg, p)
        lv = a.max(axis=2, keepdims=True)
        lit = lv[..., 0] >= LIT_LV
        bl = b.max(axis=2, keepdims=True)
        w = np.clip(bl / BG_LV, 0.0, 1.0) * THROUGH
        feat = a / np.maximum(lv, 1e-6) * (1.0 - w) + b / np.maximum(bl, 1e-6) * w
        feat *= lv / np.maximum(feat.max(axis=2, keepdims=True), 1e-6)
        dim = np.where(_near(lit), UNDER_FEATURES, 1.0)[..., None]
        setattr(f, p, np.where(lit[..., None], feat, b * dim))
    return f


def has_bg(slots: Optional[Mapping[str, str]]) -> bool:
    """A background is on: the rear lights show it too, unless the scene sets its own ``rear``."""
    return (slots or {}).get('bg') in BACKGROUNDS


# ---------------------------------------------------------------- pulses
SLASH_S = 0.24                # the cut crosses the whole canvas in this time
SLASH_SLANT = 1.5             # columns per row: the slant of the blade
SLASH_CORE = 2.5              # columns of the bright blade
SLASH_TRAIL = 20.0            # columns of the shadow behind it (> the blade's step per frame: no board is skipped)
SLASH_SHADE = 0.8             # the face right behind the blade is darkened by this share, then comes back
SLASH_C = col(20, 23, 25)     # cold white of steel
_SLASH_SPAN = CANVAS_W + SLASH_SLANT * CANVAS_H + SLASH_TRAIL


def fx_slash(t: float, f: Frame) -> Frame:
    """Katana: a thin bright slanted blade sweeps the canvas left to right, the face behind it darkened a moment."""
    if not 0.0 <= t < SLASH_S:
        return f
    head = t / SLASH_S * _SLASH_SPAN
    for p in PANELS:
        X, Y = XY[p]
        d = (head - (X + SLASH_SLANT * (CANVAS_H - 1 - Y)))[..., None]
        shade = np.where((d >= SLASH_CORE) & (d < SLASH_TRAIL),
                         1.0 - SLASH_SHADE * (1.0 - (d - SLASH_CORE) / (SLASH_TRAIL - SLASH_CORE)), 1.0)
        setattr(f, p, np.where((d >= 0.0) & (d < SLASH_CORE), SLASH_C, getattr(f, p) * shade))
    return f


GLITCH_S = 0.32
GLITCH_DROP = 0.15            # share of frames shown black (a «dropped» frame)
GLITCH_DEAD = (BLACK, col(25, 0, 25), col(0, 25, 25), col(16, 16, 16))


def fx_glitch(t: float, f: Frame) -> Frame:
    """Glitch: boards and rows jerk sideways, red and blue tear apart, a few dead pixels; a new tear each frame."""
    if not 0.0 <= t < GLITCH_S:
        return f
    rng = random.Random(3780 + int(t * FPS))
    if rng.random() < GLITCH_DROP:
        return Frame()
    for p in PANELS:
        a = getattr(f, p)
        h, w = a.shape[:2]
        if rng.random() < 0.35:
            a = np.roll(a, rng.choice((-2, -1, 1, 2)), axis=1)
        out = a.copy()
        for _ in range(rng.randint(1, 2)):
            y = rng.randrange(h)
            out[y] = np.roll(a[y], rng.choice((-3, -2, 2, 3)), axis=0)
        out[..., 0] = np.roll(out[..., 0], 1, axis=1)
        out[..., 2] = np.roll(out[..., 2], -1, axis=1)
        for _ in range(rng.randint(1, 3)):
            out[rng.randrange(h), rng.randrange(w)] = rng.choice(GLITCH_DEAD)
        setattr(f, p, out)
    return f


#: Boards light up in the order of their numbers L1 -> R2 -> mouth -> L3 -> R4 (``sides.MARKS``, the mouth after 2).
BOOT_ORDER: Tuple[str, ...] = tuple(sorted(
    PANELS, key=lambda p: float(MARKS[GROUP_OF[p]][1]) if GROUP_OF[p] in MARKS else 2.5))
BOOT_STEP_S = 0.22            # next board starts after this
BOOT_ROW_S = 0.03             # one row switched on
BOOT_LINE = col(10, 12, 14)   # the scan line that switches the rows on
BOOT_S = BOOT_STEP_S * (len(BOOT_ORDER) - 1) + 8 * BOOT_ROW_S


def fx_boot(t: float, f: Frame) -> Frame:
    """Switching on: board after board, row by row under a scan line; dark boards wait their turn."""
    if t >= BOOT_S:
        return f
    for i, p in enumerate(BOOT_ORDER):
        a = getattr(f, p).copy()
        rows = (t - i * BOOT_STEP_S) / BOOT_ROW_S
        n = int(clamp(math.floor(rows), 0, a.shape[0]))
        a[n:] = 0.0
        if 0.0 <= rows < a.shape[0]:
            a[n] = BOOT_LINE
        setattr(f, p, a)
    return f


POWER_SQUEEZE_S = 0.3         # the picture squeezes into a line
POWER_DOWN_S = 0.6            # ... the line into a dot in the middle of the mouth, then dark
POWER_LINE = col(18, 18, 18)
_POWER_CX = CANVAS_AT['mouth'][0] + 12.0
_POWER_CY = (CANVAS_H - 1) / 2.0


def fx_power_down(t: float, f: Frame) -> Frame:
    """Old TV switched off: the picture squeezes into a white line, the line into a dot, then dark (and stays dark)."""
    if t >= POWER_DOWN_S:
        return Frame()
    p1 = clamp(t / POWER_SQUEEZE_S, 0.0, 1.0)
    p2 = clamp((t - POWER_SQUEEZE_S) / (POWER_DOWN_S - POWER_SQUEEZE_S), 0.0, 1.0)
    half_h = max((1.0 - p1) * CANVAS_H / 2.0, 0.5)
    half_w = max((1.0 - p2) * CANVAS_W, 0.5)
    for p in PANELS:
        X, Y = XY[p]
        band = (np.abs(Y - _POWER_CY) <= half_h) & (np.abs(X - _POWER_CX) <= half_w)
        line = (np.abs(Y - _POWER_CY) <= 0.5)[..., None] * POWER_LINE * p1
        setattr(f, p, np.where(band[..., None], np.maximum(getattr(f, p), line), 0.0))
    return f


#: Slot ``fx`` -> pulse.
PULSES: Mapping[str, Callable[[float, Frame], Frame]] = {
    'slash': fx_slash, 'glitch': fx_glitch, 'boot': fx_boot, 'power_down': fx_power_down}
#: Length of every pulse, seconds (``power_down`` then holds the face dark).
PULSE_S: Mapping[str, float] = {'slash': SLASH_S, 'glitch': GLITCH_S, 'boot': BOOT_S, 'power_down': POWER_DOWN_S}
#: A pulse starts this long after its scene: ``FaceDirector`` crossfades into a new scene for 0.12 s, and a 0.24 s
#: slash would lose half of itself in the fade.
PULSE_LEAD_S = 0.12


# ---------------------------------------------------------------- features
def _ring() -> Tuple[Tuple[int, int], ...]:
    cells = [(y, x) for y in range(8) for x in range(8) if EYE_RIM_MASK[y, x]]
    return tuple(sorted(cells, key=lambda c: math.atan2(c[0] - 3.5, c[1] - 3.5)))


SPIN_RING = _ring()           # rim LEDs of an eye, clockwise from the left
SPIN_HZ = 1.2                 # turns per second
SPIN_TAIL = 6                 # LEDs of the spinner's tail


def loading_eyes(t: float, f: Frame) -> None:
    """Spinners in place of the eyes: a comet runs round the rim, the eyes turn mirror-wise (to the mouth on top)."""
    n = len(SPIN_RING)
    head = int(t * SPIN_HZ * n)
    for p, mirror in (('eye_l', False), ('eye_r', True)):
        a = np.zeros((8, 8, 3))
        for j in range(SPIN_TAIL):
            y, x = SPIN_RING[(head - j) % n]
            a[y, 7 - x if mirror else x] = EYE * (1.0 - j / SPIN_TAIL)
        setattr(f, p, a)


#: Slot ``feat`` -> feature swap (draws into the frame in place).
FEATURES: Mapping[str, Callable[[float, Frame], None]] = {'loading_eyes': loading_eyes}


def features(f: Frame, t: float, slots: Optional[Mapping[str, str]]) -> None:
    """Slot ``feat``: before the tint, so a tinted scene tints the spinners too."""
    fn = FEATURES.get((slots or {}).get('feat') or '')
    if fn is not None:
        fn(t, f)


def layers(f: Frame, t: float, slots: Optional[Mapping[str, str]]) -> Frame:
    """Slots ``bg`` and ``fx`` over a finished (tinted) frame: background under, then the pulse."""
    slots = slots or {}
    bg = BACKGROUNDS.get(slots.get('bg') or '')
    if bg is not None:
        f = under(f, bg(t))
    fx = PULSES.get(slots.get('fx') or '')
    return f if fx is None else fx(t - PULSE_LEAD_S, f)


__all__ = ['BACKGROUNDS', 'CANVAS_H', 'CANVAS_W', 'FEATURES', 'PULSES', 'PULSE_S', 'XY', 'features', 'has_bg',
           'layers', 'under']
