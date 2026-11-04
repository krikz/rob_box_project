"""Unit-тесты ``face_frontality`` (ADR-0130 §10.4 п.2): фронтальность по
5 landmarks RetinaFace и решение «можно ли завести новую запись»."""

from __future__ import annotations

import importlib
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional

_PKG_ROOT = Path(__file__).resolve().parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

ff = importlib.import_module('rob_box_perception.face_frontality')

W, H = 640, 480

#: Шаблон ArcFace 112x112 — эталонный анфас.
TEMPLATE = (
    (38.2946, 51.6963), (73.5318, 51.5014), (56.0252, 71.7366),
    (41.5493, 92.3655), (70.7299, 92.2041),
)


def _lm(points, x0: float = 200.0, y0: float = 150.0, scale: float = 1.5) -> List[float]:
    """Точки шаблона (px 112) → normalized координаты кадра W x H."""
    out: List[float] = []
    for x, y in points:
        out += [(x0 + scale * x) / W, (y0 + scale * y) / H]
    return out


def _with_nose(dx: float = 0.0, dy: float = 0.0):
    pts = list(TEMPLATE)
    nx, ny = pts[2]
    pts[2] = (nx + dx, ny + dy)
    return pts


def _rotated(points, deg: float):
    a = math.radians(deg)
    cx, cy = 56.0, 72.0
    out = []
    for x, y in points:
        dx, dy = x - cx, y - cy
        out.append((cx + dx * math.cos(a) - dy * math.sin(a),
                    cy + dx * math.sin(a) + dy * math.cos(a)))
    return out


def test_arcface_template_is_frontal():
    pose = ff.estimate_pose(_lm(TEMPLATE), W, H)
    assert pose is not None
    assert abs(pose.yaw_ratio) < 0.05
    assert 0.45 < pose.pitch_ratio < 0.55
    assert pose.frontal


def test_head_roll_does_not_break_frontality():
    assert ff.is_frontal(_lm(_rotated(TEMPLATE, 25.0)), W, H)


def test_turned_head_is_not_frontal():
    # Нос у глаза — голова повёрнута.
    assert not ff.is_frontal(_lm(_with_nose(dx=30.0)), W, H)
    assert not ff.is_frontal(_lm(_with_nose(dx=-30.0)), W, H)


def test_nodding_head_is_not_frontal():
    assert not ff.is_frontal(_lm(_with_nose(dy=-18.0)), W, H)  # нос у глаз
    assert not ff.is_frontal(_lm(_with_nose(dy=18.0)), W, H)   # нос у рта


def test_no_face_in_points_is_not_frontal():
    assert ff.estimate_pose(None, W, H) is None
    assert ff.estimate_pose([0.5] * 10, W, H) is None          # точки слиплись
    assert ff.estimate_pose([0.1] * 9, W, H) is None           # не 5 точек
    assert ff.estimate_pose([float('nan')] * 10, W, H) is None
    upside_down = [(x, 144.0 - y) for x, y in TEMPLATE]        # рот выше глаз
    assert ff.estimate_pose(_lm(upside_down), W, H) is None


@dataclass
class _Obs:
    embedding: Optional[Any]
    landmarks: Optional[List[float]]
    frame_w: int = W
    frame_h: int = H


def test_decision_allows_birth_for_frontal_encounter():
    frames = [_Obs([1.0], _lm(TEMPLATE)), _Obs([1.0], _lm(_with_nose(dx=30.0)))]
    decision = ff.new_record_decision(frames)
    assert decision.allowed
    assert (decision.frontal_frames, decision.considered_frames) == (1, 2)


def test_decision_blocks_birth_for_mostly_profile_encounter():
    frames = [
        _Obs([1.0], _lm(TEMPLATE)),
        _Obs([1.0], _lm(_with_nose(dx=30.0))),
        _Obs([1.0], None),
    ]
    assert not ff.new_record_decision(frames).allowed


def test_decision_blocks_birth_without_landmarks():
    assert not ff.new_record_decision([_Obs([1.0], None)]).allowed


def test_frames_without_embedding_are_ignored():
    # Кадр без эмбеддинга не вошёл в эмбеддинг Встречи — не голосует.
    frames = [_Obs([1.0], _lm(TEMPLATE)), _Obs(None, None), _Obs(None, None)]
    assert ff.new_record_decision(frames).allowed
    assert not ff.new_record_decision([_Obs(None, _lm(TEMPLATE))]).allowed


@dataclass
class _Enc:
    embedding: Optional[Any]
    best: List[_Obs]
    track_id: int = 7
    max_face_px: float = 120.0


def _never_known(_emb):
    return None


def test_gate_skips_encounter_without_embedding():
    gate = ff.encounter_gate(_Enc(None, []), _never_known, 32.0)
    assert not gate.record
    assert gate.suppressed_birth == 0
    assert gate.level == 'warn'


def test_gate_records_frontal_stranger():
    gate = ff.encounter_gate(_Enc([1.0], [_Obs([1.0], _lm(TEMPLATE))]), _never_known, 32.0)
    assert gate.record


def test_gate_suppresses_birth_of_profile_stranger():
    enc = _Enc([1.0], [_Obs([1.0], _lm(_with_nose(dx=30.0)))])
    gate = ff.encounter_gate(enc, _never_known, 32.0)
    assert not gate.record
    assert gate.suppressed_birth == 1
    assert 'не заведена' in gate.reason


def test_gate_lets_known_person_through_in_profile():
    seen = []

    def known(emb):
        seen.append(emb)
        return object()

    enc = _Enc([1.0], [_Obs([1.0], None)])
    assert ff.encounter_gate(enc, known, 32.0).record
    assert seen == [[1.0]]
