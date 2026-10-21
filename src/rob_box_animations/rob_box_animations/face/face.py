"""Procedural LED face for rob_box: models, palette, layout, current budget.

All colours are in *LED levels* 0..25: the driver runs at brightness 25/255,
so a channel value v becomes v*25//255 on the wire. Designing in levels keeps
every shade we pick actually distinguishable on the robot.

Panels (viewer space, robot faces the viewer):
    eye_l, eye_r : 8x8   (front headlights)
    mouth        : 25x5  (main_display)
    rear_l, rear_r : 8x8 (rear lights, seen from behind)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

FPS = 25
DT = 1.0 / FPS
MAX_LV = 25.0


def col(r, g, b):
    return np.array((r, g, b), dtype=float)


BLACK = col(0, 0, 0)

# ---------------------------------------------------------------- palette
EYE = col(25, 21, 9)        # Bender's warm visor-white
EYE_RIM = col(13, 10, 3)
LID = col(2, 2, 2)          # metal shutter, barely lit
LID_EDGE = col(7, 6, 4)
TEETH = col(17, 17, 13)
TEETH_RIM = col(11, 11, 8)
TEETH_HI = col(25, 25, 21)

# ---------------------------------------------------------------- geometry
_YY, _XX = np.mgrid[0:8, 0:8]
EYE_MASK = np.hypot(_XX - 3.5, _YY - 3.5) <= 3.95


def _rim(mask):
    h, w = mask.shape
    rim = np.zeros_like(mask)
    for y in range(h):
        for x in range(w):
            if not mask[y, x]:
                continue
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                yy, xx = y + dy, x + dx
                if not (0 <= yy < h and 0 <= xx < w) or not mask[yy, xx]:
                    rim[y, x] = True
                    break
    return rim


EYE_RIM_MASK = _rim(EYE_MASK)
DIVIDERS = (4, 8, 12, 16, 20)
TOOTH_CELLS = [(1, 3), (5, 7), (9, 11), (13, 15), (17, 19), (21, 23)]


def lerp(a, b, t):
    return a + (b - a) * t


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ease_io(t):
    t = clamp(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


# ---------------------------------------------------------------- frame
class Frame:
    def __init__(self):
        self.eye_l = np.zeros((8, 8, 3))
        self.eye_r = np.zeros((8, 8, 3))
        self.mouth = np.zeros((5, 25, 3))
        self.rear_l = np.zeros((8, 8, 3))
        self.rear_r = np.zeros((8, 8, 3))

    def panels(self):
        return {
            'eye_l': self.eye_l, 'eye_r': self.eye_r, 'mouth': self.mouth,
            'rear_l': self.rear_l, 'rear_r': self.rear_r,
        }


# ---------------------------------------------------------------- eyes
@dataclass
class Eye:
    top: float = 1.0          # rows above are covered by the upper lid
    slant: float = 0.0        # +: inner corner lower (angry), -: outer lower (sad)
    bot: float = 7.0          # rows below are covered by the lower lid
    bot_arc: float = 0.0      # >0 lifts the lower lid in the middle (happy ^^)
    gx: float = 0.0           # pupil offset, viewer space (+ = viewer's right)
    gy: float = 0.0
    pupil: int = 2            # pupil size in px (1..3)
    color: np.ndarray = field(default_factory=lambda: EYE.copy())
    rim: np.ndarray = field(default_factory=lambda: EYE_RIM.copy())
    bright: float = 1.0
    lid_color: np.ndarray = field(default_factory=lambda: LID.copy())
    shades: float = 0.0       # 0..1 sunglasses drop-in
    glint: float = -1.0       # sunglasses glint position (-1 = none)
    bot_lid_dark: bool = False   # happy ^^: lower lid black so the crescent pops


SHADES_L = [
    "........",
    "........",
    "FFFFFFFF",
    "FLLLLLLF",
    "FLLLLLLF",
    ".FLLLLF.",
    "..FFFF..",
    "........",
]


def draw_eye(arr, e: Eye, side: str):
    """side: 'l' or 'r' in viewer space; inner corner faces the mouth."""
    inner_dir = 1.0 if side == 'l' else -1.0
    out = np.zeros((8, 8, 3))
    cover = np.zeros((8, 8))
    for x in range(8):
        u = (x - 3.5) / 3.5 * inner_dir          # -1 outer .. +1 inner
        top = e.top + e.slant * (u + 1) / 2 * 2.2 - (e.slant * 0.6 if e.slant else 0)
        arc = e.bot_arc * (1 - ((x - 3.5) / 3.5) ** 2)
        bot = e.bot - arc
        for y in range(8):
            if not EYE_MASK[y, x]:
                continue
            # anti-aliased lids: fractional coverage of the boundary row
            ct = clamp(top - y, 0.0, 1.0)
            cb = clamp(y - bot, 0.0, 1.0)
            c = max(ct, cb)
            cover[y, x] = c
            iris = (e.rim if EYE_RIM_MASK[y, x] else e.color) * e.bright
            lid = e.lid_color
            if cb > ct and e.bot_lid_dark:
                lid = BLACK
            if c >= 1.0:
                edge = (ct >= 1 and clamp(top - (y + 1), 0, 1) < 1) or (cb >= 1 and clamp((y - 1) - bot, 0, 1) < 1)
                out[y, x] = LID_EDGE if (edge and not (cb > ct and e.bot_lid_dark)) else lid
            else:
                out[y, x] = iris * (1 - c) + lid * c
    # pupil
    if e.pupil > 0 and e.shades < 0.99:
        p = e.pupil
        cx = 3.5 + e.gx
        cy = 3.5 + e.gy
        x0 = math.floor(cx - p / 2 + 0.5)
        y0 = math.floor(cy - p / 2 + 0.5)
        for _ in range(4):
            cells = [(y, x) for y in range(y0, y0 + p) for x in range(x0, x0 + p)]
            if all(0 <= x < 8 and 0 <= y < 8 and EYE_MASK[y, x] for y, x in cells):
                break
            x0 += 1 if x0 + p / 2 < 4 else -1 if x0 + p / 2 > 4 else 0
            y0 += 1 if y0 + p / 2 < 4 else -1 if y0 + p / 2 > 4 else 0
        for y in range(y0, y0 + p):
            for x in range(x0, x0 + p):
                if 0 <= x < 8 and 0 <= y < 8 and EYE_MASK[y, x]:
                    # only paint pupil on the open part of the eye
                    if cover[y, x] < 0.5:
                        out[y, x] = out[y, x] * cover[y, x]
    # sunglasses ("deal with it"): slide down from above, glint sweeps the lens
    if e.shades > 0:
        spr = SHADES_L if side == 'l' else [r[::-1] for r in SHADES_L]
        drop = int(round((1 - ease_io(e.shades)) * -7))
        out[:] = 0 if e.shades >= 1 else out * (1 - e.shades)
        for y, row in enumerate(spr):
            for x, ch in enumerate(row):
                yy = y + drop
                if not (0 <= yy < 8) or ch == '.':
                    continue
                if ch == 'F':
                    out[yy, x] = col(14, 14, 15)
                else:
                    # mirrored lens: dark blue with a lighter band near the top
                    out[yy, x] = col(3, 3, 9) if y == 3 else col(1, 1, 4)
                    if e.glint >= 0:
                        gx_ = x if side == 'l' else 7 - x
                        d = (gx_ + (y - 3)) - e.glint
                        if abs(d) < 0.6:
                            out[yy, x] = col(25, 25, 25)
                        elif abs(d) < 1.6:
                            out[yy, x] = col(5, 5, 6)
    arr[:] = out


# ---------------------------------------------------------------- mouth
# Bender's grille: 6 cells x (upper, lower) teeth, 1px grid lines between.
# Shapes are hand-drawn per cell as (rows opened above the bite line,
# rows opened below it). Cells: 0..5 left to right.
MOUTH_SHAPES = {
    'C':  [(0, 0)] * 6,
    'A1': [(0, 0), (0, 1), (0, 1), (0, 1), (0, 1), (0, 0)],
    'A2': [(0, 0), (0, 1), (1, 1), (1, 1), (0, 1), (0, 0)],
    'A3': [(0, 1), (1, 1), (1, 1), (1, 1), (1, 1), (0, 1)],
    'O1': [(0, 0), (0, 0), (1, 1), (1, 1), (0, 0), (0, 0)],
    'O2': [(0, 0), (0, 1), (1, 1), (1, 1), (0, 1), (0, 0)],
    'E1': [(0, 1), (0, 1), (0, 1), (0, 1), (0, 1), (0, 1)],
    'E2': [(0, 1), (0, 1), (1, 1), (1, 1), (0, 1), (0, 1)],
}
VOWEL_LEVELS = {'A': ['C', 'A1', 'A2', 'A3'], 'O': ['C', 'O1', 'O1', 'O2'], 'E': ['C', 'E1', 'E1', 'E2']}


def seam_cells(curve=0.0, tilt=0.0, smirk=0.0):
    """Bite line row per cell. curve>0 smiles (corners up); tilt>0 slopes the
    line up to the viewer's right ("hmm"); smirk>0 lifts only the viewer-right
    corner, smirk<0 only the left one."""
    out = []
    for k in range(6):
        u = (k - 2.5) / 2.5
        s = 2 - curve * (u * u - 0.36) - tilt * u * 1.2
        s -= abs(smirk) * max(0.0, u if smirk > 0 else -u) * 1.25
        out.append(int(clamp(math.floor(s + 0.5), 1, 3)))
    return out


@dataclass
class Mouth:
    curve: float = 0.0
    tilt: float = 0.0
    smirk: float = 0.0
    shape: str = 'C'
    open: list = None          # explicit per-cell (up, dn) overrides shape
    bright: float = 1.0
    grid: float = 0.22         # brightness of the grille lines relative to teeth
    color: np.ndarray = field(default_factory=lambda: TEETH.copy())
    x_shift: int = 0


def _cell_of(x):
    for k, (a, b) in enumerate(TOOTH_CELLS):
        if a <= x <= b:
            return k, False
    for k, d in enumerate(DIVIDERS):
        if x == d:
            return k, True      # divider between cell k and k+1
    return None, False


def draw_mouth(arr, m: Mouth, highlight=None, tint=None):
    out = np.zeros((5, 25, 3))
    seams = seam_cells(m.curve, m.tilt, m.smirk)
    opens = m.open or MOUTH_SHAPES[m.shape]
    base = (m.color if tint is None else tint) * m.bright
    for x in range(1, 24):
        k, is_div = _cell_of(x)
        if is_div:
            s = (seams[k] + seams[k + 1]) / 2
            s_lo, s_hi = min(seams[k], seams[k + 1]), max(seams[k], seams[k + 1])
            up = min(opens[k][0], opens[k + 1][0])
            dn = min(opens[k][1], opens[k + 1][1])
            dark = set(range(s_lo - up, s_hi + dn + 1))
            c = base * m.grid
        else:
            s = seams[k]
            up, dn = opens[k]
            dark = set(range(s - up, s + dn + 1))
            c = base
        for y in range(5):
            if y in dark:
                continue
            if x in (1, 23) and y in (0, 4):
                continue        # rounded corners
            out[y, x] = c
    if highlight is not None:
        for (y, x), c in highlight.items():
            if out[y, x].sum() > 0 and x not in DIVIDERS:
                out[y, x] = c
    if m.x_shift:
        out = np.roll(out, m.x_shift, axis=1)
    arr[:] = out


def tooth_ring():
    """12 teeth in a loop: upper row left->right, lower row right->left."""
    ring = []
    for k in range(6):
        ring.append(('u', k))
    for k in reversed(range(6)):
        ring.append(('d', k))
    return ring


def tooth_pixels(which, k, seam=2):
    x0, x1 = TOOTH_CELLS[k]
    rows = range(0, seam) if which == 'u' else range(seam + 1, 5)
    return [(y, x) for y in rows for x in range(x0, x1 + 1)]


# ---------------------------------------------------------------- tweens
class Track:
    """Piecewise eased keyframes: [(t, value)]."""

    def __init__(self, v0):
        self.keys = [(0.0, v0)]

    def to(self, t, v):
        self.keys.append((t, v))
        return self

    def hold(self, t):
        self.keys.append((t, self.keys[-1][1]))
        return self

    def __call__(self, t):
        ks = self.keys
        if t <= ks[0][0]:
            return ks[0][1]
        for (t0, v0), (t1, v1) in zip(ks, ks[1:]):
            if t0 <= t <= t1:
                if t1 == t0:
                    return v1
                return lerp(v0, v1, ease_io((t - t0) / (t1 - t0)))
        return ks[-1][1]


def blink_curve(t, start, dur=0.22):
    """0 -> 1 (closed) -> 0. Fast close, slightly slower open."""
    if t < start or t > start + dur:
        return 0.0
    p = (t - start) / dur
    if p < 0.35:
        return ease_io(p / 0.35)
    if p < 0.5:
        return 1.0
    return 1 - ease_io((p - 0.5) / 0.5)


def schedule_blinks(rng, T, lo=2.2, hi=5.5, double=0.18):
    out = []
    t = rng.uniform(0.6, 1.8)
    while t < T:
        out.append(t)
        if rng.random() < double:
            out.append(t + 0.32)
        t += rng.uniform(lo, hi)
    return out


def apply_blink(e: Eye, k: float):
    if k <= 0:
        return e
    mid = 3.5
    return replace(e, top=lerp(e.top, mid, k), bot=lerp(e.bot - e.bot_arc * 0.5, mid, k), bot_arc=e.bot_arc * (1 - k))


def schedule_saccades(rng, T, every=(0.9, 2.8), rx=2.0, ry=1.0):
    pts = [(0.0, 0.0, 0.0)]
    t = rng.uniform(*every)
    while t < T:
        if rng.random() < 0.35:
            gx, gy = 0.0, 0.0
        else:
            gx = rng.choice([-1, 1]) * rng.uniform(0.6, rx)
            gy = rng.uniform(-ry, ry * 0.6)
        pts.append((t, gx, gy))
        t += rng.uniform(*every)
    return pts


def gaze_at(pts, t, snap=0.07):
    prev = pts[0]
    for p in pts:
        if p[0] <= t:
            prev = p
        else:
            break
    # quick snap between previous and current target (saccades are fast)
    idx = pts.index(prev)
    before = pts[idx - 1] if idx > 0 else prev
    k = ease_io((t - prev[0]) / snap) if snap > 0 else 1
    return lerp(before[1], prev[1], k), lerp(before[2], prev[2], k)


# ---------------------------------------------------------------- layout
#: Panel of the face (drawn for a person facing the robot) -> logical group of
#: the ros2leds compositor. The robot's left headlight (wheel_front_left) is on
#: the viewer's RIGHT, so eye_r goes there. Rear lights are drawn as seen from
#: behind: rear_l is the robot's left board. Checked on the robot by the side
#: test frame (``test_pattern.py``); the test pins this table.
GROUP_OF = {
    'eye_l': 'wheel_front_right',
    'eye_r': 'wheel_front_left',
    'mouth': 'main_display',
    'rear_l': 'wheel_rear_left',
    'rear_r': 'wheel_rear_right',
}
#: (width, height) of every logical group, as the compositor expects them.
GROUP_SIZE = {
    'main_display': (25, 5),
    'wheel_front_left': (8, 8),
    'wheel_front_right': (8, 8),
    'wheel_rear_left': (8, 8),
    'wheel_rear_right': (8, 8),
}


def to_wire(levels):
    """Levels 0..25 -> channel bytes for the driver at brightness 25.

    ``ceil``: the driver sends ``v * 25 // 255``, and level L must come back
    exactly as L (L=1 -> 11 -> 1)."""
    v = np.clip(np.asarray(levels, dtype=float), 0.0, MAX_LV) / MAX_LV * 255.0
    return np.ceil(v - 1e-6).astype(np.uint8)


# ---------------------------------------------------------------- current budget
N_LEDS = 381
MA_PER_CHANNEL = 20.0         # WS2812B, one channel at PWM 255 (datasheet)
IDLE_MA_PER_LED = 1.0
DRIVER_BRIGHTNESS = 25        # led_matrix_driver.yaml; raising it is a separate decision
IDLE_MA = N_LEDS * IDLE_MA_PER_LED
#: Lit current (above the 381 mA idle of the strip) any scene may draw at
#: brightness 25. Total peak of the heaviest scene is then ~1.25 A: x3.6 headroom
#: to the driver limiter (4.5 A), A28.
SCENE_LIT_LIMIT_MA = 1000.0


def frame_ma(frame: Frame, brightness: int = DRIVER_BRIGHTNESS) -> float:
    """Strip current of a frame by the driver's own model (#3674): every channel
    is ``v * brightness // 255`` on the wire and draws 20 mA at 255, plus 1 mA
    idle per LED."""
    lit = sum(int((to_wire(a).astype(np.int64) * brightness // 255).sum()) for a in frame.panels().values())
    return IDLE_MA + lit / 255.0 * MA_PER_CHANNEL


# ---------------------------------------------------------------- tint
#: Слот ``tint`` сцены лица для LLM: множитель RGB глаз и рта (палитра, не свободный цвет). База - тёплый
#: белый ``EYE`` (25, 21, 9): красный = (25, 3, 1), холод = (7, 15, 20). ``rainbow`` - ``rainbow_tint(t)``.
TINTS = {
    'warm': (1.0, 0.8, 0.5),
    'cold': (0.3, 0.7, 2.2),
    'red': (1.0, 0.15, 0.1),
    'green': (0.2, 1.0, 0.3),
    'blue': (0.1, 0.5, 2.5),
    'purple': (1.0, 0.2, 2.0),
}
RAINBOW_PERIOD_S = 4.0


def rainbow_tint(t):
    """Множитель радуги: оттенок обходит круг за ``RAINBOW_PERIOD_S``; синий канал усилен под тёплую базу."""
    h = (t / RAINBOW_PERIOD_S) % 1.0 * 6.0
    x = 1.0 - abs(h % 2.0 - 1.0)
    r, g, b = [(1, x, 0), (x, 1, 0), (0, 1, x), (0, x, 1), (x, 0, 1), (1, 0, x)][int(h) % 6]
    return (r * 1.0, g * 1.2, b * 2.7)


def tint_of(slot, t):
    """Множитель RGB по слоту ``tint`` (None - без оттенка)."""
    if slot is None:
        return None
    return rainbow_tint(t) if slot == 'rainbow' else TINTS[slot]


# ---------------------------------------------------------------- gaze
GAZE_FULL_DEG = 60.0          # speaker this far off the nose -> pupils at the edge
GAZE_MAX_PX = 2.2


def gaze_px(deg: float) -> float:
    """Direction to the speaker (degrees off the robot's nose, + = robot's left)
    -> pupil offset in viewer space: the robot's left is the viewer's right (+)."""
    return clamp(deg / GAZE_FULL_DEG * GAZE_MAX_PX, -GAZE_MAX_PX, GAZE_MAX_PX)
