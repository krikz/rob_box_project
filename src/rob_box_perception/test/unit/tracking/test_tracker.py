"""Поведение камерного трекера (ADR-0130 §7 этап 2).

Контракт:
  1. Пересечение двух людей: track_id не меняются местами — ни при
     точном пересечении в одной точке, ни с шумом, ни при любом порядке
     детекций в пакете.
  2. Пропуск кадров короче ``lost_after_sec``: тот же id, трек confirmed,
     положение предсказано по скорости.
  3. Выход и вход в кадр: lost → reacquired с тем же id рядом с местом
     ухода; после ``delete_after_sec`` или далеко — новый id.
  4. Жизненный цикл: одиночная вспышка не становится confirmed и
     удаляется; лицо трек не рождает, но обновляет и несёт candidate_id.
  5. Сближение ближе ``resolution_m`` — событие close_encounter (ADR-0130 §6).
  6. Пакет старше предыдущего отбрасывается со счётчиком; детерминизм —
     два прогона дают одно и то же.

Запуск:
    python -m pytest src/rob_box_perception/test/unit/tracking -v --no-cov
"""

from __future__ import annotations

import random
from typing import Callable, Dict, List, Sequence, Tuple

import pytest

from rob_box_perception.tracking import (
    Detection,
    StepResult,
    Tracker,
    TrackerParams,
)
from rob_box_perception.tracking.tracker import (
    CONFIRMED,
    EVENT_CLOSE_ENCOUNTER,
    EVENT_DELETED,
    EVENT_LOST,
    EVENT_REACQUIRED,
    LOST,
    TENTATIVE,
)

DT = 0.1  # 10 Гц, как кадры OAK-D через Hailo


def person(x: float, y: float, **kw) -> Detection:
    return Detection('person', x, y, source='oak_d', **kw)


def run(tracker: Tracker, frames: Sequence[Tuple[float, List[Detection]]]
        ) -> List[StepResult]:
    return [tracker.step(t, dets) for t, dets in frames]


def walk(start: Tuple[float, float], vel: Tuple[float, float],
         t: float) -> Tuple[float, float]:
    return start[0] + vel[0] * t, start[1] + vel[1] * t


def ids_by_nearest(result: StepResult,
                   truth: Dict[str, Tuple[float, float]]) -> Dict[str, str]:
    """Для каждого настоящего человека — id ближайшего трека."""
    out = {}
    for name, (x, y) in truth.items():
        best = min(result.tracks,
                   key=lambda s: (s.x - x) ** 2 + (s.y - y) ** 2)
        out[name] = best.track_id
    return out


def crossing(noise: float, seed: int, order: Callable[[list], list],
             y_offset: float) -> Tuple[List[Dict[str, str]], Tracker]:
    """A идёт слева направо, B навстречу; встреча в x=2 на t=2 с."""
    rng = random.Random(seed)
    tracker = Tracker()
    a0, av = (0.0, 0.0), (1.0, 0.0)
    b0, bv = (4.0, y_offset), (-1.0, 0.0)
    history = []
    for k in range(41):
        t = 100.0 + k * DT
        pa = walk(a0, av, k * DT)
        pb = walk(b0, bv, k * DT)
        dets = [person(pa[0] + rng.gauss(0, noise), pa[1] + rng.gauss(0, noise)),
                person(pb[0] + rng.gauss(0, noise), pb[1] + rng.gauss(0, noise))]
        result = tracker.step(t, order(dets))
        if k >= 3:  # оба уже confirmed
            history.append(ids_by_nearest(result, {'A': pa, 'B': pb}))
    return history, tracker


# --- 1. пересечение ---------------------------------------------------------

@pytest.mark.parametrize('y_offset', [0.0, 0.25])
@pytest.mark.parametrize('order', [lambda d: d, lambda d: d[::-1]],
                         ids=['AB', 'BA'])
def test_crossing_keeps_identities(y_offset, order):
    history, tracker = crossing(noise=0.0, seed=0, order=order,
                                y_offset=y_offset)
    first = history[0]
    assert first['A'] != first['B']
    # После встречи (t > 2.3 с) ближайший к A трек — всё ещё трек A.
    for ids in history[-10:]:
        assert ids == first
    assert len(tracker.tracks()) == 2
    assert all(s.state == CONFIRMED for s in tracker.tracks())


@pytest.mark.parametrize('seed', range(5))
def test_crossing_with_noise_and_shuffled_order(seed):
    shuffler = random.Random(1000 + seed)

    def shuffled(dets):
        dets = list(dets)
        shuffler.shuffle(dets)
        return dets

    history, tracker = crossing(noise=0.05, seed=seed, order=shuffled,
                                y_offset=0.2)
    first = history[0]
    for ids in history[-10:]:
        assert ids == first
    assert len(tracker.tracks()) == 2


def test_both_people_step_aside_no_duplicate_track():
    """Жадный «ближайший первым» здесь родил бы дубль; оптимум — нет.

    Двое стоят в 0.7 м, оба шагнули влево на ~0.4 м. Ближайшая пара
    (трек 0.0 ↔ детекция 0.3) оставила бы детекцию −0.45 вне ворот
    второго трека.
    """
    tracker = Tracker(TrackerParams(confirm_hits=1))
    for k in range(5):
        tracker.step(k * DT, [person(0.0, 0.0), person(0.7, 0.0)])
    before = {round(s.x, 1): s.track_id for s in tracker.tracks()}
    result = tracker.step(0.5, [person(0.3, 0.0), person(-0.45, 0.0)])
    assert len(result.tracks) == 2
    after = {s.track_id: s.x for s in result.tracks}
    assert after[before[0.0]] < 0.0 < after[before[0.7]]


def test_crossing_raises_close_encounter_once_per_pair():
    tracker = Tracker()
    events = []
    for k in range(41):
        pa = walk((0.0, 0.0), (1.0, 0.0), k * DT)
        pb = walk((4.0, 0.2), (-1.0, 0.0), k * DT)
        events += tracker.step(100.0 + k * DT,
                               [person(*pa), person(*pb)]).events
    close = [e for e in events if e.kind == EVENT_CLOSE_ENCOUNTER]
    assert len(close) == 1
    assert {close[0].track_id, close[0].other_track_id} == {
        s.track_id for s in tracker.tracks()}
    assert all(s.close_encounter_at is not None for s in tracker.tracks())


# --- 2. пропуск кадров ------------------------------------------------------

def test_skipped_frames_keep_id_and_predict_motion():
    tracker = Tracker()
    vel = (0.8, 0.0)
    for k in range(10):
        tracker.step(k * DT, [person(*walk((0, 0), vel, k * DT))])
    (track,) = tracker.tracks()
    tid = track.track_id
    # Кадры 10..15 пустые (0.6 с < lost_after_sec=1.0).
    for k in range(10, 16):
        tracker.step(k * DT, [])
    (track,) = tracker.tracks()
    assert track.state == CONFIRMED
    expected_x = 0.8 * 15 * DT
    assert track.x == pytest.approx(expected_x, abs=0.1)
    # Человек снова виден там, где его предсказали.
    tracker.step(16 * DT, [person(*walk((0, 0), vel, 16 * DT))])
    (track,) = tracker.tracks()
    assert track.track_id == tid
    assert track.state == CONFIRMED


def test_long_gap_while_moving_still_reacquires_near_exit():
    """Ушёл за край кадра на 3 с и вернулся в 0.5 м от места ухода."""
    tracker = Tracker()
    for k in range(10):
        tracker.step(k * DT, [person(1.0 + 0.5 * k * DT, 2.0)])
    tid = tracker.tracks()[0].track_id
    exit_x = tracker.tracks()[0].x
    events = []
    for k in range(10, 40):
        events += tracker.step(k * DT, []).events
    assert [e.kind for e in events] == [EVENT_LOST]
    assert tracker.tracks()[0].state == LOST
    result = tracker.step(40 * DT, [person(exit_x + 0.5, 2.0)])
    assert [e.kind for e in result.events] == [EVENT_REACQUIRED]
    (track,) = result.tracks
    assert (track.track_id, track.state) == (tid, CONFIRMED)


# --- 3. выход и вход в кадр -------------------------------------------------

def _confirmed_then_gone(tracker: Tracker, x: float, y: float,
                         gone_sec: float) -> str:
    for k in range(5):
        tracker.step(k * DT, [person(x, y)])
    tid = tracker.tracks()[0].track_id
    t = 4 * DT
    while t < 4 * DT + gone_sec:
        t += DT
        tracker.step(t, [])
    return tid


def test_reenter_after_delete_window_gets_new_id():
    params = TrackerParams()
    tracker = Tracker(params)
    old = _confirmed_then_gone(tracker, 1.0, 1.0,
                               params.delete_after_sec + 0.5)
    assert tracker.tracks() == ()
    t = 4 * DT + params.delete_after_sec + 1.0
    result = tracker.step(t, [person(1.0, 1.0)])
    (track,) = result.tracks
    assert track.track_id != old
    assert track.state == TENTATIVE


def test_reenter_far_from_exit_is_a_new_track():
    tracker = Tracker()
    old = _confirmed_then_gone(tracker, 1.0, 1.0, 2.0)
    result = tracker.step(10.0, [person(4.0, -2.0)])
    states = {s.track_id: s.state for s in result.tracks}
    assert states[old] == LOST
    (new,) = [tid for tid in states if tid != old]
    assert states[new] == TENTATIVE


def test_two_people_leave_and_return_swapped_sides_do_not_swap_ids():
    """Вернулись каждый к своему месту ухода — каждый свой id."""
    tracker = Tracker()
    for k in range(5):
        tracker.step(k * DT, [person(0.0, 0.0), person(3.0, 0.0)])
    left, right = sorted(tracker.tracks(), key=lambda s: s.x)
    for k in range(5, 30):
        tracker.step(k * DT, [])
    result = tracker.step(3.0, [person(3.1, 0.1), person(0.1, -0.1)])
    by_x = sorted(result.tracks, key=lambda s: s.x)
    assert [s.track_id for s in by_x] == [left.track_id, right.track_id]
    assert all(s.state == CONFIRMED for s in by_x)


# --- 4. жизненный цикл ------------------------------------------------------

def test_single_blip_never_confirms_and_is_deleted():
    tracker = Tracker()
    born = tracker.step(0.0, [person(1.0, 1.0)])
    assert born.tracks[0].state == TENTATIVE
    events = []
    for k in range(1, 10):
        events += tracker.step(k * DT, []).events
    assert [e.kind for e in events] == [EVENT_DELETED]
    assert tracker.tracks() == ()


def test_confirms_after_confirm_hits():
    tracker = Tracker(TrackerParams(confirm_hits=3))
    states = [tracker.step(k * DT, [person(1.0, 1.0)]).tracks[0].state
              for k in range(3)]
    assert states == [TENTATIVE, TENTATIVE, CONFIRMED]


def test_face_does_not_birth_but_updates_person_track():
    tracker = Tracker()
    alone = tracker.step(0.0, [Detection('face', 1.0, 1.0,
                                         candidate_id='f-42')])
    assert alone.tracks == ()
    tracker.step(0.1, [person(2.0, 2.0)])
    result = tracker.step(0.1, [Detection('face', 2.05, 2.0,
                                          candidate_id='f-42',
                                          source='oak_d_face')])
    (track,) = result.tracks
    assert track.class_name == 'person'
    assert track.candidate_id == 'f-42'
    assert track.sources == ('oak_d', 'oak_d_face')
    assert track.hits == 2


def test_candidate_id_breaks_geometric_tie():
    """Детекция ровно посередине — уходит треку с тем же candidate_id."""
    tracker = Tracker(TrackerParams(confirm_hits=1))
    tracker.step(0.0, [person(0.0, 0.0, candidate_id='f-a'),
                       person(0.6, 0.0, candidate_id='f-b')])
    a, b = tracker.tracks()
    result = tracker.step(0.0, [person(0.3, 0.0, candidate_id='f-b')])
    hits = {s.track_id: s.hits for s in result.tracks}
    assert hits == {a.track_id: 1, b.track_id: 2}


def test_unknown_class_is_counted_not_tracked():
    tracker = Tracker()
    result = tracker.step(0.0, [Detection('chair', 1.0, 1.0)])
    assert result.tracks == ()
    assert tracker.stats['unknown_class_dropped'] == 1


def test_max_tracks_cap_refuses_births():
    tracker = Tracker(TrackerParams(max_tracks=2))
    result = tracker.step(0.0, [person(0, 0), person(2, 0), person(4, 0)])
    assert len(result.tracks) == 2
    assert tracker.stats['births_refused_cap'] == 1


# --- 6. время и детерминизм -------------------------------------------------

def test_late_batch_is_dropped_and_counted():
    tracker = Tracker()
    tracker.step(1.0, [person(1.0, 1.0)])
    before = tracker.tracks()
    result = tracker.step(0.9, [person(5.0, 5.0)])
    assert result.tracks == before
    assert result.events == ()
    assert tracker.stats['late_batches_dropped'] == 1


def test_same_input_gives_same_output():
    def once():
        history, tracker = crossing(noise=0.05, seed=7, order=lambda d: d,
                                    y_offset=0.1)
        return history, tracker.tracks()

    assert once() == once()


@pytest.mark.parametrize('kwargs', [
    {'gate_m': 0.0},
    {'confirm_hits': 0},
    {'lost_after_sec': 5.0, 'delete_after_sec': 5.0},
    {'birth_classes': ('dog',)},
])
def test_params_validate(kwargs):
    with pytest.raises(ValueError):
        TrackerParams(**kwargs)
