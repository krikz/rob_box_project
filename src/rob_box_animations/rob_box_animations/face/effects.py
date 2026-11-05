"""Effects on the face canvas (#3780, part A): backgrounds under the features, one-shot pulses, feature swaps.

One slot of ``knowledge.FACE_SCENE_SLOTS`` per kind, the tables below are the renderers of their values:

* ``bg`` (:data:`BACKGROUNDS`) lasts with the scene and lies UNDER the eyes and mouth: ``bg(t) -> Frame`` on the
  canvas of ``layout.CANVAS_AT``, no channel above :data:`BG_LV` levels. It runs through the features too: they keep
  their level and take its hue (:func:`under`), the background next to them is dimmed to :data:`UNDER_FEATURES` —
  the features read and the strip stays in the current budget. The rear boards show it instead of their theme.
* ``fx`` (:data:`PULSES`) is one pulse from the scene start (after :data:`PULSE_LEAD_S`): ``fx(t, f) -> Frame``
  takes the composed frame (``glitch`` tears it, ``boot``/``power_down`` mask it, ``slash`` draws over it); past
  its length the frame is the scene's own, only ``power_down`` stays dark — the face has been switched off.
* ``feat`` (:data:`FEATURES`) swaps the features: ``loading_eyes(t, f)`` draws spinners in place of the eyes.

``t`` is seconds since the scene started. Levels 0..25 as everywhere in ``face``. The renderers themselves live in
ros2leds (``led_matrix_effects``: canvas, backgrounds, pulses); this module binds them to the rover canvas with the
face's settings and keeps what is the face's own (the spinner eyes, the slot tables).
"""
from __future__ import annotations

import math
from typing import Callable, Dict, Mapping, Optional, Tuple

import numpy as np

from led_matrix_effects import Boot, Canvas, Fire, Glitch, MatrixRain, PowerDown, Rainbow, Slash
from led_matrix_effects import under as _under

from .face import EYE, EYE_RIM_MASK, FPS, RAINBOW_PERIOD_S, Frame
from .layout import CANVAS_AT, GROUP_OF, ROVER
from .sides import MARKS

#: The rover canvas (``layout.CANVAS_AT``); effects return frames of the face's own ``Frame``.
CANVAS = Canvas(ROVER, frame=Frame)
PANELS: Tuple[str, ...] = CANVAS.panels
#: Canvas coordinates (X, Y) of every LED of a panel.
XY: Dict[str, Tuple[np.ndarray, np.ndarray]] = CANVAS.xy
CANVAS_W = CANVAS.width
CANVAS_H = CANVAS.height

#: Brightest channel of a background: 6 of 25 levels (24 %) — the eyes (25), their rim (13) and the teeth (17)
#: stay at least twice as bright.
BG_LV = 6.0
#: Background next to a lit feature pixel (4-neighbours) is dimmed to this share: a dark outline around the features.
UNDER_FEATURES = 0.25
#: A feature pixel is lit from this level; dimmer ones (the metal lids, 2) are background.
LIT_LV = 3.0
#: Share of the background's hue a lit feature takes where the background is at full strength.
THROUGH = 0.85
#: Seed of the matrix rain and the glitch (#3780): the same face every run.
SEED = 3780


# ---------------------------------------------------------------- backgrounds
bg_rainbow = Rainbow(CANVAS, period_s=RAINBOW_PERIOD_S, level=BG_LV)
bg_fire = Fire(CANVAS, level=BG_LV)
bg_matrix = MatrixRain(CANVAS, seed=SEED, level=BG_LV)

#: Slot ``bg`` -> background. ``none`` — no background.
BACKGROUNDS: Mapping[str, Callable[[float], Frame]] = {'rainbow': bg_rainbow, 'fire': bg_fire, 'matrix': bg_matrix}


def under(f: Frame, bg: Frame) -> Frame:
    """Background through the canvas, features on top. A lit feature pixel keeps its level (the brightest channel)
    and takes the background's hue by :data:`THROUGH` of the background's strength there — the wave passes through
    the eyes too, they stay the brightest; elsewhere the background, dimmed next to the features."""
    return _under(CANVAS, f, bg, level=BG_LV, dim=UNDER_FEATURES, lit_level=LIT_LV, through=THROUGH)


def has_bg(slots: Optional[Mapping[str, str]]) -> bool:
    """A background is on: the rear lights show it too, unless the scene sets its own ``rear``."""
    return (slots or {}).get('bg') in BACKGROUNDS


# ---------------------------------------------------------------- pulses
fx_slash = Slash(CANVAS)
SLASH_S, SLASH_C = fx_slash.duration, fx_slash.color
fx_glitch = Glitch(CANVAS, fps=FPS, seed=SEED)
GLITCH_S = fx_glitch.duration
#: Boards light up in the order of their numbers L1 -> R2 -> mouth -> L3 -> R4 (``sides.MARKS``, the mouth after 2).
BOOT_ORDER: Tuple[str, ...] = tuple(sorted(
    PANELS, key=lambda p: float(MARKS[GROUP_OF[p]][1]) if GROUP_OF[p] in MARKS else 2.5))
fx_boot = Boot(CANVAS, order=BOOT_ORDER)
BOOT_S = fx_boot.duration
#: The picture collapses to the middle of the mouth.
fx_power_down = PowerDown(CANVAS, centre=(CANVAS_AT['mouth'][0] + 12.0, (CANVAS_H - 1) / 2.0))
POWER_DOWN_S = fx_power_down.duration

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
