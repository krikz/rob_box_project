"""Эффекты полотна лица (#3780, часть A): фоны, импульсы, черты-спиннеры; размеры, ток, читаемость, порядок."""

from __future__ import annotations

import time

import numpy as np
import pytest

from rob_box_dialog import knowledge as kn
from rob_box_animations.face import effects as E
from rob_box_animations.face.face import (CANVAS_AT, GROUP_OF, GROUP_SIZE, IDLE_MA, SCENE_LIT_LIMIT_MA, FPS, Frame,
                                          frame_ma)
from rob_box_animations.face.scene import IDLE, Scene
from rob_box_animations.face.states import SCENES
from rob_box_animations.face_director import FaceDirector

T0 = 1_800_000_000.0
FRONT = ('eye_l', 'mouth', 'eye_r')
EFFECTS = [('bg', v) for v in E.BACKGROUNDS] + [('fx', v) for v in E.PULSES] + [('feat', v) for v in E.FEATURES]
#: Тяжёлые черты под эффектом: широкие глаза и зубы — худший случай по току.
HEAVY = {'eyes': 'wide', 'mouth': 'teeth'}


def scenes_peak_ma() -> float:
    """Нынешний максимум светящейся части среди всех сцен лица (один проход каждой)."""
    peak = 0.0
    for spec in SCENES.values():
        scene = Scene(spec)
        peak = max(peak, max(frame_ma(scene.frame(i / FPS, IDLE)) - IDLE_MA for i in range(int(spec.period * FPS))))
    return peak


SCENES_PEAK_MA = scenes_peak_ma()


def run(slots, seconds=4.0):
    """Кадры сцены ``slots`` за ``seconds`` с на 25 к/с через ``FaceDirector`` (как на роботе) и время кадра, мс."""
    d = FaceDirector(T0)
    d.voice_seen(T0 + 7.0)
    now = T0 + 8.0
    d.apply_face(slots, 30.0, now)
    frames, ms = [], []
    for i in range(int(seconds * FPS)):
        t = now + i / FPS
        d.voice_seen(t)
        t_start = time.perf_counter()
        frames.append(d.tick(t))
        ms.append((time.perf_counter() - t_start) * 1000.0)
    return frames, float(np.mean(ms))


def level(arr):
    return arr.max(axis=2)


def test_slot_values_are_the_renderers_tables():
    assert set(E.BACKGROUNDS) | {'none'} == set(kn.FACE_SCENE_SLOTS['bg'])
    assert set(E.PULSES) == set(E.PULSE_S) == set(kn.FACE_SCENE_SLOTS['fx'])
    assert set(E.FEATURES) == set(kn.FACE_SCENE_SLOTS['feat'])


def test_canvas_is_one_table_of_all_panels_without_overlaps():
    assert set(CANVAS_AT) == set(GROUP_OF) == set(Frame().panels())
    taken = set()
    for panel, (x0, y0) in CANVAS_AT.items():
        w, h = GROUP_SIZE[GROUP_OF[panel]]
        cells = {(x0 + x, y0 + y) for x in range(w) for y in range(h)}
        assert not cells & taken, panel
        taken |= cells
    assert [p for p, _ in sorted(CANVAS_AT.items(), key=lambda kv: kv[1][0])] == \
        ['eye_l', 'mouth', 'eye_r', 'rear_l', 'rear_r']


@pytest.mark.parametrize('slot, value', EFFECTS)
def test_every_effect_renders_every_group_at_its_size_in_levels(slot, value):
    frames, ms = run({**HEAVY, slot: value}, seconds=2.0)
    for f in frames:
        for panel, arr in f.panels().items():
            w, h = GROUP_SIZE[GROUP_OF[panel]]
            assert arr.shape == (h, w, 3), (value, panel)
            assert np.isfinite(arr).all() and arr.min() >= 0.0 and arr.max() <= 25.0 + 1e-9, (value, panel)
    assert ms < 20.0, (value, ms)         # кадр 40 мс; на ПК ~2–4 мс, рядом со сценами


@pytest.mark.parametrize('slot, value', EFFECTS)
def test_current_stays_under_the_heaviest_scene_pulses_only_peak_briefly(slot, value):
    for base in ({}, HEAVY, {'eyes': 'shades', 'mouth': 'eq', 'rear': 'party'}):
        frames, _ = run({**base, slot: value})
        lit = [frame_ma(f) - IDLE_MA for f in frames]
        if slot == 'fx':
            over = sum(m > SCENES_PEAK_MA for m in lit)
            assert over <= int(E.PULSE_S[value] * FPS) + 1 and max(lit) < SCENE_LIT_LIMIT_MA, (value, base, max(lit))
        else:
            assert max(lit) <= SCENES_PEAK_MA, (value, base, max(lit), SCENES_PEAK_MA)


@pytest.mark.parametrize('bg', list(E.BACKGROUNDS))
def test_features_keep_their_level_over_a_background_that_stays_dim(bg):
    plain, _ = run(HEAVY, seconds=2.0)
    shown, _ = run({**HEAVY, 'bg': bg}, seconds=2.0)
    seen_bg = 0
    for a, b in zip(plain, shown):
        for panel in FRONT:
            lv_a, lv_b = level(getattr(a, panel)), level(getattr(b, panel))
            lit = lv_a >= E.LIT_LV
            assert np.allclose(lv_a[lit], lv_b[lit]), (bg, panel)     # черта светит, как светила
            assert lv_b[~lit].max(initial=0.0) <= E.BG_LV + 1e-9, (bg, panel)
            seen_bg += int((lv_b[~lit] >= 1.0).sum())
            if lit.any() and (lv_b[~lit] >= 1.0).any():
                # яркая часть черт (радужка, ободок, зубы; и при моргании) хотя бы вдвое ярче фона
                assert np.percentile(lv_b[lit], 90) >= 2.0 * lv_b[~lit].max(), (bg, panel)
    assert seen_bg > 0, bg


@pytest.mark.parametrize('bg', list(E.BACKGROUNDS))
def test_background_moves_and_reaches_the_rear_boards(bg):
    frames, _ = run({'bg': bg}, seconds=1.0)
    assert any(f.rear_l.any() for f in frames) and any(f.rear_r.any() for f in frames)
    first, last = frames[0], frames[-1]
    assert not (np.array_equal(first.eye_l, last.eye_l) and np.array_equal(first.mouth, last.mouth))


def test_rainbow_is_a_wave_not_one_colour_for_the_whole_face():
    f = E.bg_rainbow(1.0)
    hues = {tuple(np.round(px / max(px.max(), 1e-6), 1)) for arr in f.panels().values() for px in arr.reshape(-1, 3)}
    assert len(hues) > 12


def test_slash_crosses_all_five_boards_in_the_order_of_the_canvas():
    first = {}
    for i in range(int(E.SLASH_S * 1000)):
        t = i / 1000.0
        f = E.fx_slash(t, Frame())
        for panel, arr in f.panels().items():
            if panel not in first and np.all(arr == E.SLASH_C, axis=2).any():
                first[panel] = t
    assert set(first) == set(CANVAS_AT)
    by_time = sorted(first, key=first.get)
    assert by_time == sorted(CANVAS_AT, key=lambda p: CANVAS_AT[p][0]), first
    assert E.fx_slash(E.SLASH_S, Frame()).mouth.max() == 0.0


def test_slash_at_25_fps_touches_every_board_and_is_over_in_a_quarter_second():
    plain, _ = run({}, seconds=1.0)
    cut, _ = run({'fx': 'slash'}, seconds=1.0)
    changed = [i for i, (a, b) in enumerate(zip(plain, cut)) if any(
        not np.array_equal(getattr(a, p), getattr(b, p)) for p in CANVAS_AT)]
    for panel in CANVAS_AT:
        assert any(not np.array_equal(getattr(a, panel), getattr(b, panel)) for a, b in zip(plain, cut)), panel
    assert (changed[-1] - changed[0] + 1) / FPS <= 0.28, changed


def test_boot_lights_boards_in_number_order_and_starts_dark():
    assert E.BOOT_ORDER == ('eye_r', 'eye_l', 'mouth', 'rear_l', 'rear_r')     # L1 R2 рот L3 R4
    frames, _ = run({'fx': 'boot'}, seconds=2.0)
    on = {p: next(i for i, f in enumerate(frames) if getattr(f, p).any()) for p in CANVAS_AT}
    assert [on[p] for p in E.BOOT_ORDER] == sorted(on[p] for p in E.BOOT_ORDER), on
    assert len(set(on.values())) == len(on), on
    plain, _ = run({}, seconds=2.0)
    assert all(np.array_equal(a.mouth, b.mouth) for a, b in zip(plain[-5:], frames[-5:]))


def test_power_down_squeezes_to_a_dot_and_stays_dark():
    frames, _ = run({'fx': 'power_down'}, seconds=3.0)
    lit_rows = [{y for p in CANVAS_AT for y in np.nonzero(getattr(f, p).any(axis=(1, 2)))[0] + CANVAS_AT[p][1]}
                for f in frames]
    end = int((E.PULSE_LEAD_S + E.POWER_DOWN_S) * FPS) + 1
    squeezed = [rows for rows in lit_rows[:end] if rows and len(rows) <= 2]
    assert squeezed, lit_rows[:end]
    assert all(not any(f.panels()[p].any() for p in CANVAS_AT) for f in frames[end:])


def test_glitch_tears_the_face_then_gives_it_back():
    plain, _ = run({'eyes': 'x'}, seconds=1.5)
    torn, _ = run({'eyes': 'x', 'fx': 'glitch'}, seconds=1.5)
    end = int((E.PULSE_LEAD_S + E.GLITCH_S) * FPS) + 1
    assert sum(not np.array_equal(a.mouth, b.mouth) for a, b in zip(plain[:end], torn[:end])) >= 4
    assert all(np.array_equal(a.mouth, b.mouth) and np.array_equal(a.eye_l, b.eye_l)
               for a, b in zip(plain[end:], torn[end:]))


def test_loading_eyes_are_mirrored_spinners_that_turn():
    f0, f1 = Frame(), Frame()
    E.loading_eyes(0.0, f0)
    E.loading_eyes(0.2, f1)
    for f in (f0, f1):
        assert (level(f.eye_l) > 0).sum() == E.SPIN_TAIL
        assert np.array_equal(f.eye_l, f.eye_r[:, ::-1])
    assert not np.array_equal(f0.eye_l, f1.eye_l)
    frames, _ = run({'feat': 'loading_eyes', 'tint': 'blue'}, seconds=0.5)
    assert (level(frames[-1].eye_l) > 0).sum() == E.SPIN_TAIL


def test_no_effect_slots_change_nothing():
    a, _ = run({'eyes': 'wide'}, seconds=1.0)
    b, _ = run({'eyes': 'wide', 'bg': 'none'}, seconds=1.0)
    assert all(np.array_equal(x.panels()[p], y.panels()[p]) for x, y in zip(a, b) for p in CANVAS_AT)
