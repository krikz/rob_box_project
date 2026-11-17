"""Unit-тесты для face_tracker (ADR-0123 §3 «Встреча, а не кадр», issue #2599 PR-B).

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_face_tracker.py -q

Чистый Python + numpy, без ROS/cv2/железа. Время подаётся явно через ``now``
(никаких sleep) — как того требует ``FaceTracker.update``/``expire``.

Что покрываем (см. ADR-0123 §3 и docstring face_tracker.py):
  1. Устойчивое лицо промоутится ровно один раз, после min_track_sec, не раньше.
  2. Лицо мельче min_face_px не промоутится никогда.
  3. Лицо короче min_track_sec исчезает — не промоутится.
  4. Два лица рядом — два независимых трека, две встречи.
  5. Трек переживает короткий разрыв (< max_gap_sec) и НЕ переживает длинный.
  6. IoU-ассоциация держит identity при медленном дрейфе bbox.
  7. Эмбеддинг спасает ассоциацию при быстром прыжке (IoU промахнулся).
  8. Отбор лучших кадров предпочитает более крупный/резкий; best <= best_n.
  9. Эмбеддинг встречи — unit-norm, равен нормированному среднему best.
  10. expire дропает протухшие треки и возвращает id; max_tracks вытесняет старейший.
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import numpy as np
import pytest

# ---------- import target under test ------------------------------------

# `parents[2]` = .../src/rob_box_perception → содержит пакет rob_box_perception/.
_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))
tracker_mod = importlib.import_module('rob_box_perception.face_tracker')

FaceTracker = tracker_mod.FaceTracker
FaceObservation = tracker_mod.FaceObservation
FaceEncounter = tracker_mod.FaceEncounter


# ---------- helpers --------------------------------------------------------

def make_obs(
    ts,
    cx=0.5,
    cy=0.5,
    w=0.15,
    h=0.15,
    confidence=0.9,
    face_px=100.0,
    embedding=None,
    sharpness=500.0,
):
    """Короткая фабрика FaceObservation с разумными дефолтами."""
    return FaceObservation(
        ts=ts,
        bbox_cx=cx,
        bbox_cy=cy,
        bbox_w=w,
        bbox_h=h,
        confidence=confidence,
        face_px=face_px,
        embedding=embedding,
        sharpness=sharpness,
        frame_w=640,
        frame_h=480,
    )


def unit_vec(seed, dim=8):
    """Детерминированный unit-norm вектор для тестов эмбеддингов."""
    rng = np.random.RandomState(seed)
    v = rng.rand(dim).astype(np.float32)
    return v / np.linalg.norm(v)


# ---------- 1. промоушн устойчивого лица -----------------------------------

def test_steady_face_promotes_exactly_once_after_min_track_sec():
    tracker = FaceTracker(min_track_sec=2.0, min_face_px=48.0)

    # Кадры каждые 0.2с, лицо стоит на месте, крупное.
    promotions_before = []
    now = 0.0
    for i in range(9):  # до t=1.6 включительно — ещё меньше min_track_sec
        now = round(i * 0.2, 2)
        promoted = tracker.update([make_obs(now, face_px=100.0)], now)
        promotions_before.extend(promoted)

    assert promotions_before == [], 'не должно промоутиться раньше min_track_sec'

    # t=2.0 — ровно min_track_sec, трек должен промоутиться.
    now = 2.0
    promoted = tracker.update([make_obs(now, face_px=100.0)], now)
    assert len(promoted) == 1
    encounter = promoted[0]
    assert isinstance(encounter, FaceEncounter)
    assert encounter.promoted_at == 2.0
    assert encounter.observation_count == 10

    # Дальнейшие кадры не производят повторного промоушна.
    now = 2.2
    promoted_again = tracker.update([make_obs(now, face_px=100.0)], now)
    assert promoted_again == []
    assert tracker.promoted_track_count() == 1


# ---------- 2. мелкое лицо никогда не промоутится ---------------------------

def test_small_face_never_promotes_no_matter_how_long():
    tracker = FaceTracker(min_track_sec=2.0, min_face_px=48.0)

    promoted_total = []
    for i in range(30):  # 6 секунд удержания
        now = round(i * 0.2, 2)
        promoted = tracker.update([make_obs(now, face_px=30.0)], now)
        promoted_total.extend(promoted)

    assert promoted_total == []
    assert tracker.promoted_track_count() == 0


# ---------- 3. короткая вспышка не промоутится ------------------------------

def test_brief_flicker_shorter_than_min_track_sec_never_promotes():
    tracker = FaceTracker(min_track_sec=2.0, min_face_px=48.0, max_gap_sec=1.5)

    promoted_total = []
    for i in range(3):  # t=0, 0.2, 0.4 — держится 0.4с
        now = round(i * 0.2, 2)
        promoted = tracker.update([make_obs(now, face_px=100.0)], now)
        promoted_total.extend(promoted)

    # Лицо исчезло: трек истекает по max_gap_sec.
    dropped = tracker.expire(now=5.0)
    assert dropped == [1]

    assert promoted_total == []
    assert tracker.promoted_track_count() == 0
    assert tracker.active_track_count() == 0


# ---------- 4. два лица рядом -> два трека, две встречи --------------------

def test_two_faces_side_by_side_produce_two_independent_encounters():
    tracker = FaceTracker(min_track_sec=1.0, min_face_px=48.0)

    all_promoted = []
    for i in range(7):  # t=0..1.2
        now = round(i * 0.2, 2)
        obs = [
            make_obs(now, cx=0.2, cy=0.5, face_px=100.0),
            make_obs(now, cx=0.8, cy=0.5, face_px=100.0),
        ]
        all_promoted.extend(tracker.update(obs, now))

    assert len(all_promoted) == 2
    track_ids = {e.track_id for e in all_promoted}
    assert len(track_ids) == 2
    assert tracker.active_track_count() == 2
    assert tracker.promoted_track_count() == 2


# ---------- 5. разрыв: переживает короткий, не переживает длинный ----------

def test_track_survives_brief_gap_but_splits_after_long_gap():
    tracker = FaceTracker(min_track_sec=100.0, max_gap_sec=1.5, min_face_px=48.0)

    tracker.update([make_obs(0.0, cx=0.5, cy=0.5)], 0.0)
    first_id = next(iter(tracker._tracks))

    # Короткий разрыв: 1.0с < max_gap_sec=1.5с — трек не должен истечь.
    dropped = tracker.expire(now=1.0)
    assert dropped == []
    tracker.update([make_obs(1.0, cx=0.5, cy=0.5)], 1.0)
    assert tracker.active_track_count() == 1
    assert first_id in tracker._tracks

    # Длинный разрыв от последнего кадра (t=1.0): 1.0 + 3.0 = 4.0, gap=3.0 > 1.5.
    dropped = tracker.expire(now=4.0)
    assert dropped == [first_id]
    assert tracker.active_track_count() == 0

    tracker.update([make_obs(4.0, cx=0.5, cy=0.5)], 4.0)
    new_id = next(iter(tracker._tracks))
    assert new_id != first_id, 'после длинного разрыва должен появиться новый трек'


# ---------- 6. IoU держит identity при медленном дрейфе ---------------------

def test_iou_association_keeps_identity_during_slow_drift():
    tracker = FaceTracker(min_track_sec=100.0, iou_threshold=0.3)

    tracker.update([make_obs(0.0, cx=0.50, cy=0.50, w=0.15, h=0.15)], 0.0)
    track_id = next(iter(tracker._tracks))

    # Небольшие сдвиги кадр к кадру — IoU каждый раз выше порога.
    for i, dx in enumerate([0.02, 0.04, 0.06, 0.08], start=1):
        now = float(i)
        tracker.update([make_obs(now, cx=0.50 + dx, cy=0.50, w=0.15, h=0.15)], now)
        assert tracker.active_track_count() == 1
        assert track_id in tracker._tracks

    snap = tracker.snapshot(track_id)
    assert snap is not None
    assert snap.observation_count == 5


# ---------- 7. эмбеддинг спасает ассоциацию при быстром прыжке -------------

def test_embedding_rescue_keeps_one_track_on_fast_jump():
    tracker = FaceTracker(
        min_track_sec=100.0, iou_threshold=0.3, embedding_threshold=0.5
    )
    emb = unit_vec(seed=1)

    tracker.update(
        [make_obs(0.0, cx=0.2, cy=0.2, w=0.1, h=0.1, embedding=emb)], 0.0
    )
    track_id = next(iter(tracker._tracks))
    assert tracker.active_track_count() == 1

    # Прыжок на другой конец кадра: IoU = 0, но эмбеддинг тот же вектор.
    tracker.update(
        [make_obs(0.2, cx=0.85, cy=0.85, w=0.1, h=0.1, embedding=emb)], 0.2
    )

    assert tracker.active_track_count() == 1, 'эмбеддинг должен был спасти ассоциацию'
    assert track_id in tracker._tracks
    assert tracker._tracks[track_id].observation_count == 2


def test_without_embedding_fast_jump_creates_new_track():
    """Контроль к тесту 7: без эмбеддингов такой же прыжок ДОЛЖЕН расщепиться."""
    tracker = FaceTracker(min_track_sec=100.0, iou_threshold=0.3)

    tracker.update([make_obs(0.0, cx=0.2, cy=0.2, w=0.1, h=0.1)], 0.0)
    tracker.update([make_obs(0.2, cx=0.85, cy=0.85, w=0.1, h=0.1)], 0.2)

    assert tracker.active_track_count() == 2


# ---------- 8. отбор лучших кадров ------------------------------------------

def test_best_frame_selection_prefers_bigger_sharper_and_respects_best_n():
    tracker = FaceTracker(min_track_sec=100.0, best_n=2)

    tracker.update(
        [make_obs(0.0, face_px=50.0, sharpness=100.0, confidence=0.6)], 0.0
    )
    tracker.update(
        [make_obs(1.0, face_px=150.0, sharpness=800.0, confidence=0.95)], 1.0
    )
    tracker.update(
        [make_obs(2.0, face_px=60.0, sharpness=120.0, confidence=0.65)], 2.0
    )

    track_id = next(iter(tracker._tracks))
    snap = tracker.snapshot(track_id)

    assert len(snap.best) <= 2
    assert len(snap.best) == 2
    # Лучший (t=1.0, крупнее и резче) должен быть первым.
    assert snap.best[0].ts == 1.0
    # Худший из трёх (t=0.0, самый маленький и мутный) не должен попасть в best.
    kept_ts = {o.ts for o in snap.best}
    assert 0.0 not in kept_ts
    assert 2.0 in kept_ts


def test_best_never_exceeds_best_n_with_many_observations():
    tracker = FaceTracker(min_track_sec=100.0, best_n=3)

    for i in range(20):
        now = float(i)
        tracker.update([make_obs(now, face_px=50.0 + i, sharpness=100.0 + i)], now)

    track_id = next(iter(tracker._tracks))
    snap = tracker.snapshot(track_id)
    assert len(snap.best) == 3


# ---------- 9. эмбеддинг встречи -------------------------------------------

def test_encounter_embedding_is_unit_norm_and_matches_normalized_mean_of_best():
    tracker = FaceTracker(min_track_sec=1.0, min_face_px=10.0, best_n=3)

    e1 = unit_vec(seed=10)
    e2 = unit_vec(seed=11)
    e3 = unit_vec(seed=12)

    obs_list = [
        make_obs(0.0, face_px=100.0, sharpness=500.0, embedding=e1),
        make_obs(0.5, face_px=110.0, sharpness=520.0, embedding=e2),
        make_obs(1.0, face_px=120.0, sharpness=540.0, embedding=e3),
    ]
    promoted = []
    for obs in obs_list:
        promoted.extend(tracker.update([obs], obs.ts))

    assert len(promoted) == 1
    encounter = promoted[0]

    assert encounter.embedding is not None
    norm = float(np.linalg.norm(encounter.embedding))
    assert norm == pytest.approx(1.0, abs=1e-5)

    expected_mean = np.mean(
        np.stack([o.embedding for o in encounter.best], axis=0), axis=0
    )
    expected_mean = expected_mean / np.linalg.norm(expected_mean)
    np.testing.assert_allclose(encounter.embedding, expected_mean, atol=1e-5)


def test_encounter_embedding_is_none_when_no_observation_has_embedding():
    tracker = FaceTracker(min_track_sec=1.0, min_face_px=10.0)

    promoted = []
    for i in range(3):
        now = float(i) * 0.5
        promoted.extend(tracker.update([make_obs(now, face_px=100.0)], now))

    assert len(promoted) == 1
    assert promoted[0].embedding is None


# ---------- 10. expire и max_tracks -----------------------------------------

def test_expire_drops_stale_tracks_and_returns_their_ids():
    tracker = FaceTracker(max_gap_sec=1.0)

    tracker.update([make_obs(0.0, cx=0.2, cy=0.2)], 0.0)
    tracker.update([make_obs(0.0, cx=0.8, cy=0.8)], 0.0)
    assert tracker.active_track_count() == 2

    dropped = tracker.expire(now=5.0)
    assert sorted(dropped) == [1, 2]
    assert tracker.active_track_count() == 0


def test_max_tracks_evicts_least_recently_seen():
    tracker = FaceTracker(max_tracks=2, iou_threshold=0.9)

    # Три непересекающихся лица в одном кадре — превышает max_tracks=2.
    tracker.update(
        [
            make_obs(0.0, cx=0.1, cy=0.1),
            make_obs(0.0, cx=0.5, cy=0.5),
            make_obs(0.0, cx=0.9, cy=0.9),
        ],
        0.0,
    )

    assert tracker.active_track_count() == 2
    # Самый первый (наименее недавно виденный при равенстве last_seen —
    # наименьший track_id создан первым и вытесняется первым при tie).
    assert 1 not in tracker._tracks


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
