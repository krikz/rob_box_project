"""Сцена лица из слотов LLM (``FaceScene``, ADR-0150 §5.5.9): enum-слоты -> кадр, ни пикселей, ни кода от модели.

Слоты приходят в ``ctx.face`` (уже проверены ``rob_box_dialog.face_scene``), оттенок и тема фар - в ``ctx.tint`` /
``ctx.rear_theme`` (их кладёт ``FaceDirector``). Не заданное в слоте берётся как у покоя: глаза открыты, моргание
обычное, взгляд на говорящего или осмотр, рот закрыт. Пока робот говорит, рот - по огибающей речи при любом слоте
``mouth`` (#4225). Глаза, рот и очки
рисуют те же функции, что и обычные сцены (``draw_eye``/``draw_mouth``, ``SHADES_L`` в ``face.py``, ``eq_mouth``
в ``states.py``), копий нет. ``left``/``right`` - со стороны человека напротив робота.
"""
from __future__ import annotations

import random
from dataclasses import replace

from . import talk as TK
from .face import (TEETH_HI, Eye, Frame, Mouth, Track, apply_blink, blink_curve, clamp, col, draw_eye,
                   draw_mouth, gaze_at, gaze_px, schedule_blinks, schedule_saccades)
from .scene import SceneSpec
from .states import SLEEP_EYE, X_EYE, eq_mouth

DSL_KEY = 'face_dsl'
#: Темп морганий, саккад и эквалайзера.
TEMPO = {'slow': 0.5, 'normal': 1.0, 'fast': 2.0}
#: Моргание: (от, до) секунд между морганиями.
BLINK_EVERY = {'normal': (2.2, 5.5), 'rapid': (0.25, 0.6)}
EQ_BPM = 118.0
GAZE_FIXED = {'left': (-2.2, 0.0), 'right': (2.2, 0.0), 'up': (0.0, -1.8), 'down': (0.0, 1.8)}
#: Параметры ``Eye`` по слоту ``eyes`` (рисуемые глаза; ``closed``/``x``/``shades`` - ниже).
EYES = {
    'open': dict(top=1.0),
    'half': dict(top=3.4, bright=0.9),
    'squint': dict(top=2.6, bot=5.4, slant=0.5),
    'wide': dict(top=0.0, pupil=1),
}
MOUTHS = {
    'grin': dict(curve=2.0, shape='E1'),
    'frown': dict(curve=-2.0),
    'flat': dict(),
    'o': dict(shape='O2'),
    'teeth': dict(curve=-0.6, shape='E2', color=TEETH_HI, grid=0.0),
}
WINK = Track(0.0).hold(0.4).to(0.55, 1.0).hold(1.1).to(1.3, 0.0)


def _sprite(arr, cells, color):
    for y, x in cells:
        arr[y, x] = color


def _x_cells():
    return [(y, x) for y, row in enumerate(X_EYE) for x, ch in enumerate(row) if ch == '#']


def _eye(arr, side, kind, e, shades_k, glint):
    """Один глаз по слоту ``eyes``: нарисованный, «︶», «×» или очки."""
    if kind == 'closed':
        _sprite(arr, SLEEP_EYE, col(12, 9, 3))
    elif kind == 'x':
        _sprite(arr, _x_cells(), col(25, 2, 0))
    elif kind == 'shades':
        draw_eye(arr, replace(e, shades=shades_k, glint=glint if shades_k >= 1 else -1.0), side)
    else:
        draw_eye(arr, e, side)


def _gaze(slot, ctx, t, sac):
    """Смещение зрачка (gx, gy) по слоту ``gaze``; ``None`` - на говорящего, если он известен, иначе осмотр."""
    if slot in GAZE_FIXED:
        return GAZE_FIXED[slot]
    if slot in (None, 'speaker') and ctx.gaze_deg is not None:
        return float(round(gaze_px(ctx.gaze_deg))), 0.0
    return gaze_at(sac, t)


def _mouth(f, slot, ctx, u, bpm):
    """Рот по слоту ``mouth``; пока робот говорит (``ctx.mouth_level`` есть) - по огибающей речи, какой бы слот ни
    был: сцена команды держится поверх речи (#4225: «Глаза опустил, свечусь синим…» с ``mouth: frown`` - рот
    стоял, а робот говорил), глаза, взгляд и оттенок остаются её, а рот её - после речи."""
    if ctx.mouth_level is not None:
        slot = 'talk'
    elif slot is None:
        slot = 'flat'
    if slot == 'talk':
        TK.mouth_jaw(f.mouth, ctx.mouth_level or 0.0, ctx.vowel)
    elif slot == 'eq':
        draw_mouth(f.mouth, Mouth())
        eq_mouth(f, u, bpm, col(25, 3, 12))
    else:
        draw_mouth(f.mouth, Mouth(**MOUTHS[slot]))


def face_scene(ticks, seed=21, T=8.0):
    """Кадр из ``ticks.ctx.face``: слоты читаются на каждом кадре, расписания морганий пересчитываются за проход."""
    rng = random.Random(seed)
    cache = {}
    for t in ticks:
        slots = ticks.ctx.face or {}
        k = TEMPO[slots.get('tempo') or 'normal']
        key = (slots.get('blink'), k)
        if key not in cache:
            lo, hi = BLINK_EVERY.get(slots.get('blink') or 'normal', BLINK_EVERY['normal'])
            cache = {key: (schedule_blinks(rng, T * k + 1.0, lo, hi, 0.0), schedule_saccades(rng, T * k + 1.0))}
        blinks, sac = cache[key]
        u = t * k
        kind = slots.get('eyes') or 'open'
        gx, gy = _gaze(slots.get('gaze'), ticks.ctx, u, sac)
        e = Eye(**{**EYES.get(kind, EYES['open']), 'gx': gx, 'gy': gy})
        if slots.get('blink') != 'none':
            e = apply_blink(e, max((blink_curve(u, b) for b in blinks), default=0.0))
        el = er = e
        wink = slots.get('blink')
        if wink in ('wink_left', 'wink_right'):
            closed = apply_blink(replace(e, bot_arc=1.5 * WINK(t)), WINK(t))
            el, er = (closed, e) if wink == 'wink_left' else (e, closed)
        f = Frame()
        shades_k, glint = clamp(t / 0.7, 0.0, 1.0), (u % 3.0) / 3.0 * 16 - 4
        _eye(f.eye_l, 'l', kind, el, shades_k, glint)
        _eye(f.eye_r, 'r', kind, er, shades_k, glint)
        _mouth(f, slots.get('mouth'), ticks.ctx, u, EQ_BPM * k)
        yield f


DSL_SPEC = SceneSpec(face_scene, 8.0, 'dsl', 'Сцена из слотов LLM', 'глаза, взгляд, моргание, рот, оттенок, темп (ADR-0150 §5.5.9)')
__all__ = ['DSL_KEY', 'DSL_SPEC', 'face_scene']
