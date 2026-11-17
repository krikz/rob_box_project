"""#4225 (марафон-6, n904): сцена лица команды держится поверх речи, рот - по огибающей речи.

Модель ответила речью «…Глаза опустил, свечусь синим — вот так мне грустно.» и сценой
``{eyes: half, gaze: down, mouth: frown, tint: blue}``. Сцена LLM у ``FaceDirector`` выше «говорит» (талк-рот), но слот
``mouth: frown`` рисовал неподвижный рот, пока робот говорил. Теперь, пока звучит речь, рот сцены идёт по огибающей
(как ``mouth: talk``), а глаза, взгляд и оттенок остаются её; речь кончилась - рот сцены (``frown``) вернулся.
"""

from __future__ import annotations

import numpy as np

from rob_box_animations.face.dsl import DSL_KEY, DSL_SPEC
from rob_box_animations.face.scene import FaceContext, Scene
from rob_box_animations.face_director import FaceDirector

T0 = 1_800_000_000.0
N904 = {'eyes': 'half', 'gaze': 'down', 'mouth': 'frown', 'tint': 'blue', 'tempo': 'slow'}


def _mouth(slots, level=None, t=1.0):
    return Scene(DSL_SPEC).raw(t, FaceContext(face=slots, mouth_level=level)).mouth


def test_while_speaking_the_scene_mouth_follows_the_voice_whatever_the_slot():
    for level in (0.0, 0.4, 0.9):
        talking = _mouth({**N904, 'mouth': 'talk'}, level)
        for slot in ('frown', 'grin', 'flat', 'o', 'teeth', 'eq', None):
            assert np.array_equal(_mouth({**N904, 'mouth': slot}, level), talking), (slot, level)


def test_open_and_shut_voice_give_different_mouths_under_a_frown_scene():
    """Рот правда двигается: огибающая 0.9 и 0.0 - разные кадры рта (а не застывший ``frown``)."""
    assert not np.array_equal(_mouth(N904, 0.9), _mouth(N904, 0.0))


def test_without_speech_the_scene_keeps_its_own_mouth():
    frown = _mouth(N904)
    assert not np.array_equal(frown, _mouth({**N904, 'mouth': 'talk'}))
    assert not np.array_equal(frown, _mouth({**N904, 'mouth': 'flat'}))


def _director_with_scene():
    d = FaceDirector(T0)
    now = T0 + 8.0
    d.voice_seen(now)
    d.apply_face(N904, 15.0, now)
    return d, now


def test_director_keeps_the_llm_scene_over_speech_with_the_voice_mouth():
    """Сцена LLM на лице во время речи - ``face_dsl`` с уровнем рта по огибающей, а не ``talk_jaw`` без эмоции."""
    d, now = _director_with_scene()
    d.on_envelope('s1', now + 0.5, 40, [0.6] * 50, [], now)
    key, ctx = d.choose(now + 1.0)
    assert key == DSL_KEY and ctx.mouth_level is not None and ctx.face == N904 and ctx.tint is not None


def test_after_speech_the_scene_mouth_is_back_until_the_scene_time_ends():
    d, now = _director_with_scene()
    d.on_envelope('s1', now + 0.5, 40, [0.6] * 25, [], now)   # 1 с речи
    key, ctx = d.choose(now + 4.0)                             # речь и её «хвост» кончились, сцена ещё 11 с
    assert key == DSL_KEY and ctx.mouth_level is None
    d.voice_seen(now + 15.0)
    assert d.choose(now + 15.5)[0].startswith('idle_'), "время сцены кончилось - лицо покоя"
