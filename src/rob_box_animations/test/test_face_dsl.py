"""Сцена лица из слотов LLM: рендер каждого слота, приоритеты, снятие, ответы лица (ADR-0150 §5.5.9, PR-3c #3691)."""

from __future__ import annotations

import json
import pathlib
import time
import types

import numpy as np
import pytest

from rob_box_dialog import knowledge as kn
from rob_box_animations.face import dsl, rear as R
from rob_box_animations.face.face import SHADES_L, TINTS, tint_of, to_wire
from rob_box_animations.face_director import FaceDirector

T0 = 1_800_000_000.0
CASES = json.loads((pathlib.Path(__file__).parents[2] / 'rob_box_dialog' / 'test' / 'face_scene_cases.json')
                   .read_text(encoding='utf-8'))
#: Значения слотов, что и есть «как у покоя»: от пустой сцены не отличаются.
SAME_AS_BASE = {('eyes', 'open'), ('gaze', 'speaker'), ('gaze', 'wander'), ('blink', 'normal'), ('mouth', 'flat'), ('mouth', 'talk'),
                ('tempo', 'normal'), ('rear', 'markers'), ('bg', 'none')}


def awake() -> FaceDirector:
    d = FaceDirector(T0)
    d.voice_seen(T0 + 7.0)
    return d


def clip(slots, seconds=6.0, speaking=False):
    """Провод кадров (байты всех панелей) сцены ``slots`` за ``seconds`` с на 25 к/с."""
    d = awake()
    now = T0 + 8.0
    d.apply_face(slots, 30.0, now)
    out = []
    for i in range(int(seconds * 25)):
        t = now + i / 25.0
        d.voice_seen(t)
        if speaking:
            d.on_envelope('s1', now, 40, [0.1, 0.9, 0.6, 1.0, 0.2] * 40, [1] * 200, t)
        frame = d.tick(t)
        out.append(b''.join(to_wire(a).tobytes() for a in frame.panels().values()))
    assert d.key == dsl.DSL_KEY
    return out


BASE = clip({})


@pytest.mark.parametrize('slot, value', [(s, v) for s, vs in kn.FACE_SCENE_SLOTS.items() for v in vs])
def test_every_slot_value_changes_the_picture_unless_it_is_the_default(slot, value):
    shown = clip({slot: value})
    assert (shown == BASE) is ((slot, value) in SAME_AS_BASE), (slot, value)


def test_twenty_cases_render_differently_from_the_base_scene():
    for case in CASES:
        slots = {k: v for k, v in case['face'].items() if k != 'duration_s'}
        assert clip(slots) != BASE, case['phrase']


def test_twenty_cases_stay_in_the_cpu_and_current_budget():
    from rob_box_animations.face.face import IDLE_MA, SCENE_LIT_LIMIT_MA, frame_ma
    worst_ms, worst_ma = 0.0, 0.0
    for case in CASES:
        d = awake()
        d.apply_face({k: v for k, v in case['face'].items() if k != 'duration_s'}, 30.0, T0 + 8.0)
        ms = []
        for i in range(150):
            t = T0 + 8.0 + i / 25.0
            d.voice_seen(t)
            t_start = time.perf_counter()
            frame = d.tick(t)
            ms.append((time.perf_counter() - t_start) * 1000.0)
            worst_ma = max(worst_ma, frame_ma(frame) - IDLE_MA)
        worst_ms = max(worst_ms, float(np.mean(ms)))
    assert worst_ms < 20.0, worst_ms          # кадр 40 мс; рядом с остальными сценами (замер в PR)
    assert worst_ma < SCENE_LIT_LIMIT_MA, worst_ma


def test_unset_mouth_follows_the_envelope_while_speaking_and_an_explicit_mouth_does_not():
    assert clip({'eyes': 'wide'}, speaking=True) != clip({'eyes': 'wide'})
    assert clip({'eyes': 'wide', 'mouth': 'grin'}, speaking=True) == clip({'eyes': 'wide', 'mouth': 'grin'})


def test_talk_slot_is_closed_in_silence_and_open_by_the_envelope():
    assert clip({'mouth': 'talk'}) == BASE
    assert clip({'mouth': 'talk'}, speaking=True) != BASE


def test_wink_closes_one_eye_and_the_sides_are_the_viewers():
    frames = {}
    for name in ('wink_left', 'wink_right'):
        d = awake()
        now = T0 + 8.0
        d.apply_face({'blink': name}, 30.0, now)
        best = max((d.tick(now + i / 25.0) for i in range(60)), key=lambda f: abs(f.eye_l.sum() - f.eye_r.sum()))
        frames[name] = (best.eye_l.sum(), best.eye_r.sum())
    assert frames['wink_left'][0] < frames['wink_left'][1]       # зритель видит закрытым свой левый глаз
    assert frames['wink_right'][1] < frames['wink_right'][0]


def test_tint_is_a_palette_not_a_free_colour():
    assert set(TINTS) | {'rainbow'} == set(kn.FACE_SCENE_SLOTS['tint'])
    assert tint_of(None, 0.0) is None
    assert tint_of('rainbow', 0.0) != tint_of('rainbow', 1.5)
    red = clip({'eyes': 'wide', 'tint': 'red'})[10]
    plain = clip({'eyes': 'wide'})[10]
    assert red != plain


def test_shades_use_the_one_sunglasses_sprite_not_a_copy():
    source = pathlib.Path(dsl.__file__).read_text(encoding='utf-8')
    assert 'SHADES_L =' not in source and 'FFFFFFFF' not in source and 'LLLLLL' not in source
    assert SHADES_L[2] == 'FFFFFFFF'
    d = awake()
    d.apply_face({'eyes': 'shades'}, 30.0, T0 + 8.0)
    f = [d.tick(T0 + 8.0 + i / 25.0) for i in range(30)][-1]
    assert (f.eye_l.sum(axis=2) > 0).sum() == sum(ch != '.' for row in SHADES_L for ch in row)


def test_rear_slot_sets_the_rear_theme_for_the_scene_duration_and_names_exist():
    assert set(R.REAR_OF_SLOT) == set(kn.FACE_SCENE_SLOTS['rear'])
    assert set(R.REAR_OF_SLOT.values()) <= set(R.THEMES)
    d = awake()
    now = T0 + 8.0
    d.apply_face({'rear': 'heart'}, 5.0, now)
    assert d.rear_theme_at(now + 1) == 'valentine'
    assert d.rear_theme_at(now + 5.5) != 'valentine'
    d.apply_face({'rear': 'off'}, 5.0, now)
    frame = d.tick(now + 1)
    assert not frame.rear_l.any() and not frame.rear_r.any()


def test_scene_expires_by_duration_and_failed_takes_it_off():
    d = awake()
    now = T0 + 8.0
    d.apply_face({'eyes': 'wide'}, 3.0, now)
    assert d.choose(now + 2.9)[0] == dsl.DSL_KEY
    assert d.choose(now + 3.1)[0] == 'idle_stare'
    d.apply_face({'eyes': 'wide', 'rear': 'party'}, 20.0, now)
    assert d.clear_face() is True and d.clear_face() is False
    assert d.choose(now + 1)[0] == 'idle_stare' and d.rear_theme_at(now + 1) != 'party'


def test_duration_is_cut_to_thirty_seconds_and_sleep_still_wins():
    d = awake()
    now = T0 + 8.0
    d.apply_face({'eyes': 'wide'}, 500.0, now)
    d.voice_seen(now + 29)
    assert d.choose(now + 29)[0] == dsl.DSL_KEY
    d.voice_seen(now + 30.5)
    assert d.choose(now + 30.5)[0] == 'idle_stare'
    d.apply_face({'eyes': 'wide'}, 20.0, now + 31)
    assert d.choose(now + 50)[0] == 'sleep'        # голос пропал на voice_timeout_s - сон главнее сцены


# ---------------------------------------------------------------- узел
def _expression(**fields):
    base = dict(turn_id='t1', epoch=1, agent='personality', emotion='', gesture='', earcon='', ttl_ms=0, face_json='')
    return types.SimpleNamespace(**{**base, **fields})


def _face_node():
    from rclpy.node import Node
    if Node.__name__ != '_Node':
        pytest.skip('real rclpy installed: node wiring needs a ROS context')
    from rob_box_animations.face_node import FaceNode
    return FaceNode()


def test_valid_face_json_starts_the_scene_and_is_answered_started():
    node = _face_node()
    node._on_expression(_expression(face_json=json.dumps({'eyes': 'wide', 'tint': 'red', 'duration_s': 4})))
    (ev,) = node._event_pub.sent
    assert (ev.executor, ev.kind, ev.reason) == ('face', 0, '')
    assert node.director._overlay[0] == {'eyes': 'wide', 'tint': 'red'}


def test_invalid_face_json_is_dropped_the_rest_still_runs_and_skipped_invalid_face_is_sent():
    node = _face_node()
    node._on_expression(_expression(emotion='happy', face_json='{"eyes": "neon"}'))
    kinds = [(e.kind, e.reason) for e in node._event_pub.sent]
    assert kinds == [(1, 'invalid_face'), (0, '')]
    assert node.director._overlay is None and node.director._request[0] == 'happy'


def test_error_and_silence_reflexes_take_the_scene_off_but_a_plain_emotion_does_not():
    node = _face_node()
    node._on_expression(_expression(face_json='{"eyes": "wide"}'))
    node._on_expression(_expression(emotion='happy'))
    assert node.director._overlay is not None
    node._on_expression(_expression(emotion='error'))
    assert node.director._overlay is None
    node._on_expression(_expression(face_json='{"eyes": "wide"}'))
    node._on_expression(_expression(emotion='taped_mouth'))
    assert node.director._overlay is None
