"""Сопоставление, фильтр движения и адаптер Observation трекера.

Контракт:
  1. ``assign`` даёт минимальную сумму — сверка с перебором на случайных
     матрицах, включая прямоугольные и с запрещёнными парами.
  2. Запрещённые пары (``None``) в результат не попадают; максимум пар
     важнее суммы.
  3. ``ConstantVelocity2D`` сходится к истинной скорости, ``stop`` её
     обнуляет.
  4. ``to_detection`` берёт точку нужного фрейма, отказ фрейма — причиной.

Запуск:
    python -m pytest src/rob_box_perception/test/unit/tracking -v --no-cov
"""

from __future__ import annotations

import itertools
import math
import random
from types import SimpleNamespace

import pytest

from rob_box_perception.tracking.assignment import assign
from rob_box_perception.tracking.motion import ConstantVelocity2D
from rob_box_perception.tracking.observation_adapter import (
    stamp_seconds,
    to_detection,
)


def brute_force(costs):
    """Лучший (число пар, -сумма) по всем назначениям."""
    n, m = len(costs), len(costs[0])
    best = (0, 0.0)
    for k in range(min(n, m), 0, -1):
        for rows in itertools.combinations(range(n), k):
            for cols in itertools.permutations(range(m), k):
                pairs = list(zip(rows, cols))
                if any(costs[i][j] is None for i, j in pairs):
                    continue
                total = sum(costs[i][j] for i, j in pairs)
                if (k, -total) > best:
                    best = (k, -total)
        if best[0]:
            return best
    return best


@pytest.mark.parametrize('seed', range(40))
def test_assign_matches_brute_force(seed):
    rng = random.Random(seed)
    n, m = rng.randint(1, 4), rng.randint(1, 4)
    costs = [[None if rng.random() < 0.25 else round(rng.uniform(0, 3), 3)
              for _ in range(m)] for _ in range(n)]
    pairs = assign(costs)
    assert len({i for i, _ in pairs}) == len(pairs)
    assert len({j for _, j in pairs}) == len(pairs)
    assert all(costs[i][j] is not None for i, j in pairs)
    count, neg_total = brute_force(costs)
    assert len(pairs) == count
    assert sum(costs[i][j] for i, j in pairs) == pytest.approx(-neg_total)


def test_assign_beats_greedy_on_crossing():
    # Жадный взял бы (0,0)=0.1 и оставил (1,1)=2.0 → 2.1; оптимум 0.6.
    costs = [[0.1, 0.3], [0.3, 2.0]]
    assert assign(costs) == [(0, 1), (1, 0)]


def test_assign_empty_and_all_forbidden():
    assert assign([]) == []
    assert assign([[]]) == []
    assert assign([[None, None], [None, None]]) == []


def test_filter_learns_velocity_and_stop_resets_it():
    filt = ConstantVelocity2D(0.0, 0.0, 0.02, 1.0)
    for k in range(1, 30):
        filt.predict(0.1, 1.0)
        filt.update(0.5 * k * 0.1, -0.2 * k * 0.1, 0.02)
    vx, vy = filt.velocity
    assert vx == pytest.approx(0.5, abs=0.05)
    assert vy == pytest.approx(-0.2, abs=0.05)
    filt.stop(1.0)
    x0, _ = filt.position
    filt.predict(1.0, 1.0)
    assert filt.position[0] == pytest.approx(x0)
    assert filt.velocity == (0.0, 0.0)


def test_filter_variance_grows_without_measurements():
    filt = ConstantVelocity2D(0.0, 0.0, 0.02, 1.0)
    before = filt.position_variance[0]
    filt.predict(0.5, 1.0)
    assert filt.position_variance[0] > before
    filt.predict(0.0, 1.0)
    filt.predict(-1.0, 1.0)  # dt <= 0 — без изменений


def _obs(map_status='ok', base_status='ok', mx=1.0, my=2.0):
    point = SimpleNamespace
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=500_000_000)),
        source='oak_d', class_name='person', confidence=0.9,
        position_stddev_m=0.05, candidate_id='',
        position_map=point(x=mx, y=my, z=0.0), map_status=map_status,
        position_base_link=point(x=3.0, y=4.0, z=1.0),
        base_link_status=base_status)


def test_to_detection_takes_requested_frame():
    det, why = to_detection(_obs(), 'map')
    assert why == 'ok'
    assert (det.x, det.y, det.class_name, det.source) == (1.0, 2.0, 'person',
                                                          'oak_d')
    det, _ = to_detection(_obs(), 'base_link')
    assert (det.x, det.y) == (3.0, 4.0)
    assert stamp_seconds(_obs()) == pytest.approx(12.5)


def test_to_detection_reports_frame_failure():
    det, why = to_detection(_obs(map_status='no_transform'), 'map')
    assert (det, why) == (None, 'no_transform')
    det, why = to_detection(_obs(mx=math.nan), 'map')
    assert (det, why) == (None, 'non_finite')
    with pytest.raises(ValueError):
        to_detection(_obs(), 'odom')
