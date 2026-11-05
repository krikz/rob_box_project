"""#3733: the fast renderers draw the same picture as the per-pixel loops they replaced, to the last bit.

The references below are the loops of ``draw_eye`` / ``rear.disco`` / ``effects.under`` as they were before the
optimisation; the face code must match them byte for byte (``tobytes``), so a later «speed-up» cannot shift a shade.
"""

from __future__ import annotations

import math
import types
from dataclasses import replace

import numpy as np
import pytest

from rob_box_animations.face import effects as E
from led_matrix_effects import lamps as R
from rob_box_animations.face.face import (BLACK, EYE_MASK, EYE_RIM_MASK, LID_EDGE, Eye, Frame, Mouth, clamp, col,
                                          draw_eye, draw_mouth, frame_ma, to_wire, wire_ma)
from rob_box_animations.face.scene import FaceContext, Scene
from rob_box_animations.face.states import SCENES
from rob_box_animations.face_node import PanelPublisher


def _ref_eye_body(e: Eye, side: str):
    inner_dir = 1.0 if side == 'l' else -1.0
    out = np.zeros((8, 8, 3))
    for x in range(8):
        u = (x - 3.5) / 3.5 * inner_dir
        top = e.top + e.slant * (u + 1) / 2 * 2.2 - (e.slant * 0.6 if e.slant else 0)
        bot = e.bot - e.bot_arc * (1 - ((x - 3.5) / 3.5) ** 2)
        for y in range(8):
            if not EYE_MASK[y, x]:
                continue
            ct = clamp(top - y, 0.0, 1.0)
            cb = clamp(y - bot, 0.0, 1.0)
            c = max(ct, cb)
            iris = (e.rim if EYE_RIM_MASK[y, x] else e.color) * e.bright
            lid = BLACK if (cb > ct and e.bot_lid_dark) else e.lid_color
            if c >= 1.0:
                edge = (ct >= 1 and clamp(top - (y + 1), 0, 1) < 1) or (cb >= 1 and clamp((y - 1) - bot, 0, 1) < 1)
                out[y, x] = LID_EDGE if (edge and not (cb > ct and e.bot_lid_dark)) else lid
            else:
                out[y, x] = iris * (1 - c) + lid * c
    return out


EYES = [
    Eye(),
    Eye(top=1.8, slant=-0.6, gx=1.6, gy=-1.6),
    Eye(top=0.0, bot=6.2, bot_arc=3.4, gy=-1, bot_lid_dark=True),
    Eye(top=2.6, slant=-2.0, gy=1, bright=0.62),
    Eye(top=1.6, slant=2.3, color=col(25, 5, 0), rim=col(13, 1, 0)),
    Eye(top=3.17, bot=4.03, bot_arc=0.4),
    Eye(top=0.0, pupil=1),
]


@pytest.mark.parametrize('side', ['l', 'r'])
@pytest.mark.parametrize('e', EYES)
def test_eye_body_matches_the_per_pixel_loop(e, side):
    arr = np.zeros((8, 8, 3))
    draw_eye(arr, replace(e, pupil=0), side)
    assert arr.tobytes() == _ref_eye_body(e, side).tobytes()


def test_memo_hands_out_copies_a_scene_may_paint_over():
    a = np.zeros((8, 8, 3))
    draw_eye(a, Eye(), 'l')
    clean = a.copy()
    a[5, 6] = col(3, 9, 25)                     # sad: a tear painted over the drawn eye
    b = np.zeros((8, 8, 3))
    draw_eye(b, Eye(), 'l')
    assert b.tobytes() == clean.tobytes()
    m1, m2 = np.zeros((5, 25, 3)), np.zeros((5, 25, 3))
    draw_mouth(m1, Mouth(curve=1.0))
    m1[:] = 0
    draw_mouth(m2, Mouth(curve=1.0))
    assert m2.any()


def _ref_disco(t, bpm):
    beat = t * bpm / 60.0
    outs = []
    pal = [col(25, 0, 10), col(25, 12, 0), col(18, 22, 0), col(0, 20, 8), col(0, 9, 25), col(12, 0, 25)]
    for side in (0, 1):
        img = np.zeros((8, 8, 3))
        ang = beat * math.pi / 2 * (1 if side else -1)
        for y in range(8):
            for x in range(8):
                sector = int((((math.atan2(y - 3.5, x - 3.5) + ang) % math.tau) / math.tau) * 6)
                if math.hypot(x - 3.5, y - 3.5) <= 3.95:
                    img[y, x] = pal[sector] * (0.9 if (sector + int(beat)) % 2 == 0 else 0.25)
        outs.append(img)
    return outs


@pytest.mark.parametrize('t', [0.0, 0.37, 1.0, 2.51, 7.96, 123.4])
def test_disco_matches_the_per_pixel_loop(t):
    got = R.disco(t, 118)
    for a, b in zip(got, _ref_disco(t, 118)):
        assert a.tobytes() == b.tobytes()


def _ref_under(f: Frame, bg: Frame) -> Frame:
    out = Frame()
    for p in E.PANELS:
        a, b = getattr(f, p), getattr(bg, p)
        lv = a.max(axis=2, keepdims=True)
        lit = lv[..., 0] >= E.LIT_LV
        bl = b.max(axis=2, keepdims=True)
        w = np.clip(bl / E.BG_LV, 0.0, 1.0) * E.THROUGH
        feat = a / np.maximum(lv, 1e-6) * (1.0 - w) + b / np.maximum(bl, 1e-6) * w
        feat *= lv / np.maximum(feat.max(axis=2, keepdims=True), 1e-6)
        n = lit.copy()
        n[1:] |= lit[:-1]
        n[:-1] |= lit[1:]
        n[:, 1:] |= lit[:, :-1]
        n[:, :-1] |= lit[:, 1:]
        dim = np.where(n, E.UNDER_FEATURES, 1.0)[..., None]
        setattr(out, p, np.where(lit[..., None], feat, b * dim))
    return out


@pytest.mark.parametrize('bg', sorted(E.BACKGROUNDS))
@pytest.mark.parametrize('t', [0.0, 0.52, 3.3])
def test_background_under_the_face_matches_the_per_panel_code(bg, t):
    face = Scene(SCENES['music_shades']).frame(t, FaceContext(bpm=118))
    want = _ref_under(face, E.BACKGROUNDS[bg](t))
    got = E.under(face, E.BACKGROUNDS[bg](t))
    for p in E.PANELS:
        assert getattr(got, p).shape == getattr(want, p).shape
        assert getattr(got, p).tobytes() == getattr(want, p).tobytes(), p


def test_publisher_counts_the_current_once_and_only_on_a_change():
    try:    # real sensor_msgs/Image (CI) insists on builtin_interfaces/Time
        from builtin_interfaces.msg import Time
    except ImportError:
        Time = type(None)
    node = types.SimpleNamespace(
        create_publisher=lambda *a: types.SimpleNamespace(publish=lambda m: None),
        get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=Time)))
    pub = PanelPublisher(node)
    scene = Scene(SCENES['music_shades'])
    for i in range(30):
        f = scene.frame(i / 25.0, FaceContext(bpm=118))
        assert pub.publish(f, 100.0 + i / 25.0) == frame_ma(f)
    f = scene.frame(0.0)
    assert wire_ma([to_wire(a) for a in f.panels().values()]) == frame_ma(f)
