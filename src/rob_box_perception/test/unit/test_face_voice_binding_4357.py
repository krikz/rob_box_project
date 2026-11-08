"""Доказательство «говорит это лицо» (issue #4357): чистый модуль ``face_voice_binding`` без камеры и хранилища."""

from __future__ import annotations

import sys
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from rob_box_perception import face_voice_binding as fvb  # noqa: E402


def _lm(mouth_half: float, scale: float = 1.0, dx: float = 0.0):
    """5 точек: глаза на 0.08*scale, рот 2*mouth_half*scale; dx — сдвиг всего лица."""
    c = 0.5 + dx
    return [c - 0.04 * scale, 0.47, c + 0.04 * scale, 0.47, c, 0.5,
            c - mouth_half * scale, 0.54, c + mouth_half * scale, 0.54]


def test_mouth_ratio_ignores_scale_and_shift():
    """Фото, которое приблизилось и сдвинулось вместе с роботом, — то же отношение рта: не «говорит»."""
    near = fvb.mouth_ratio(_lm(0.03, scale=1.5, dx=0.1), 640, 480)
    far = fvb.mouth_ratio(_lm(0.03), 640, 480)
    assert abs(near - far) < 1e-9


def test_mouth_ratio_rejects_missing_or_broken_points():
    assert fvb.mouth_ratio(None, 640, 480) is None
    assert fvb.mouth_ratio([0.1] * 6, 640, 480) is None
    assert fvb.mouth_ratio([0.5] * 10, 640, 480) is None  # глаза слиплись


def test_mouth_spread_drops_single_outlier():
    still = [0.75, 0.75, 0.75, 0.75, 1.5]  # один выброс детектора на фото
    assert fvb.mouth_spread(still) == 0.0
    assert fvb.mouth_spread([0.7]) is None


def test_empty_frames_are_stale_not_no_face():
    b = fvb.VoiceFaceBinding(window_sec=6.0)
    assert b.judge('spk', 100.0).kind == fvb.STALE
    b.note_frame(99.0, [])
    assert b.judge('spk', 100.0).kind == fvb.NO_FACE


def test_unrecognized_single_face_is_no_face():
    b = fvb.VoiceFaceBinding(window_sec=6.0, min_frames=1, confirmations=1)
    for t in range(5):
        b.note_frame(float(t), [None], [0.7 + 0.1 * (t % 2)])
    assert b.judge('spk', 4.0).kind == fvb.NO_FACE


def test_speaking_face_binds_after_confirmations_and_resets_counter():
    b = fvb.VoiceFaceBinding(window_sec=6.0, min_frames=4, confirmations=2)
    kinds = []
    for t0 in (0.0, 10.0):
        for i in range(6):
            b.note_frame(t0 + 0.5 * i, ['p1'], [0.7 if i % 2 else 0.9])
        kinds.append(b.judge('spk', t0 + 2.5).kind)
    assert kinds == [fvb.PENDING, fvb.BIND]
    for i in range(6):
        b.note_frame(20.0 + 0.5 * i, ['p1'], [0.7 if i % 2 else 0.9])
    assert b.judge('spk', 22.5).kind == fvb.PENDING  # после привязки счёт начинается заново


def _judge_ratios(ratios, **judge_kw):
    b = fvb.VoiceFaceBinding(window_sec=6.0)
    for i, r in enumerate(ratios):
        b.note_frame(0.5 * i, ['p1'], [r])
    return b.judge('spk', 2.5, **judge_kw)


def test_mouth_threshold_calibrated_on_robot_4538():
    """Живые числа из #4538: разброс 0.076 (живое лицо) проходит порог, 0.042 (фото) — нет."""
    assert fvb.DEFAULT_MIN_MOUTH_SPREAD == 0.06
    live, photo = [1.0, 1.0, 1.0, 1.076], [1.0, 1.0, 1.0, 1.042]
    assert abs(fvb.mouth_spread(live) - 0.076) < 1e-9
    assert abs(fvb.mouth_spread(photo) - 0.042) < 1e-9
    assert _judge_ratios(live, self_introduced=True).kind == fvb.BIND
    assert _judge_ratios(photo, self_introduced=True).kind == fvb.NOT_SPEAKING


def test_self_introduced_binds_at_once_ordinary_needs_confirmations():
    ratios = [0.7, 0.9, 0.7, 0.9, 0.7, 0.9]
    assert _judge_ratios(ratios, self_introduced=True).kind == fvb.BIND
    assert _judge_ratios(ratios).kind == fvb.PENDING
