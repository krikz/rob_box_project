"""Unit-тесты для модуля «Взгляд» (gaze.py, ADR-0104, issue #2531).

Что покрываем (всё pure-Python, без rclpy):
  1. Frame dataclass: scale/pad_left/pad_top/orig_w/orig_h
     корректно сохраняются и читаются.
  2. Frame.as_letterbox(): RGB ndarray + letterbox → NHWC uint8 +
     метаданные scale/pad_left/pad_top в dataclass.
  3. StubSource: отдаёт один синтетический кадр, source_name='stub'.
  4. make_source('stub'): создаёт StubSource, не дожидается кадра.
  5. GazeSourceUnavailable: сообщение содержит source_name, topic,
     waited_sec (capability-honest, ADR-0018).

Что НЕ покрываем (требует rclpy, проверяется на Vision Pi / e2e):
- OakDSource.frames() — реальная ROS-подписка.
- CeilingCameraSource.frames() — реальная ROS-подписка.
- wait_for_first_frame(timeout) — таймаут против реального ROS.

Запуск:
    PYTHONPATH=src/rob_box_perception \\
        python3 -m pytest \\
        src/rob_box_perception/test/unit/test_gaze_seam.py \\
        -v
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest


# ---------- import target under test ------------------------------------

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]


def _import_module():
    if str(_PKG_ROOT) not in sys.path:
        sys.path.insert(0, str(_PKG_ROOT))
    return importlib.import_module('rob_box_perception.gaze')


gaze_mod = _import_module()


# ============================================================================
# Frame dataclass
# ============================================================================

def _has_numpy() -> bool:
    try:
        import numpy  # noqa: F401
        return True
    except ImportError:
        return False


def _has_cv2() -> bool:
    try:
        import cv2  # noqa: F401
        return True
    except ImportError:
        return False


def _has_numpy_and_cv2() -> bool:
    return _has_numpy() and _has_cv2()


@pytest.mark.skipif(not _has_numpy(), reason='требует numpy для синтетического кадра')
def test_frame_dataclass_carries_metadata():
    """Frame хранит scale/pad_left/pad_top/orig_w/orig_h/source_name."""
    import numpy as np
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    frame = gaze_mod.Frame(
        rgb=rgb,
        scale=1.0,
        pad_left=0,
        pad_top=80,
        original_w=640,
        original_h=480,
        frame_id='oak_rgb_frame',
        stamp=12345.678,
        source_name='oak_d',
    )
    assert frame.scale == 1.0
    assert frame.pad_left == 0
    assert frame.pad_top == 80
    assert frame.original_w == 640
    assert frame.original_h == 480
    assert frame.frame_id == 'oak_rgb_frame'
    assert frame.stamp == 12345.678
    assert frame.source_name == 'oak_d'
    assert frame.rgb.shape == (480, 640, 3)


def test_frame_dataclass_works_without_numpy_rgb():
    """Frame можно сконструировать без numpy (rgb=None)."""
    frame = gaze_mod.Frame(
        rgb=None,
        scale=1.0,
        pad_left=0,
        pad_top=0,
        original_w=0,
        original_h=0,
        frame_id='',
        stamp=0.0,
        source_name='test',
    )
    assert frame.source_name == 'test'
    assert frame.rgb is None


@pytest.mark.skipif(not _has_numpy_and_cv2(), reason='требует numpy + cv2 для as_letterbox')
def test_frame_as_letterbox_produces_nhwc_uint8_with_metadata():
    """Frame.as_letterbox(input_w=640, input_h=640) → NHWC uint8 tensor.

    Метаданные scale/pad_left/pad_top обновляются в dataclass.
    """
    import numpy as np
    # 320×240 RGB → letterbox 640×640: scale=2.0, pad=0,0.
    rgb = np.full((240, 320, 3), 128, dtype=np.uint8)
    frame = gaze_mod.Frame(
        rgb=rgb, scale=1.0, pad_left=0, pad_top=0,
        original_w=320, original_h=240,
        frame_id='test', stamp=0.0, source_name='test',
    )
    tensor = frame.as_letterbox(input_w=640, input_h=640)
    assert tensor.shape == (1, 640, 640, 3)
    assert tensor.dtype == np.uint8
    assert frame.scale == pytest.approx(2.0)
    assert frame.pad_left == 0
    assert frame.pad_top == 0


@pytest.mark.skipif(not _has_numpy_and_cv2(), reason='требует numpy + cv2 для as_letterbox')
def test_frame_as_letterbox_640x480_has_vertical_pad():
    """640×480 → letterbox 640×640: scale=1.0, pad_top=80."""
    import numpy as np
    rgb = np.full((480, 640, 3), 128, dtype=np.uint8)
    frame = gaze_mod.Frame(
        rgb=rgb, scale=1.0, pad_left=0, pad_top=0,
        original_w=640, original_h=480,
        frame_id='test', stamp=0.0, source_name='test',
    )
    tensor = frame.as_letterbox(input_w=640, input_h=640)
    assert tensor.shape == (1, 640, 640, 3)
    assert frame.scale == pytest.approx(1.0)
    assert frame.pad_left == 0
    assert frame.pad_top == 80
    # Top-padding rows должны быть серыми (114 = LETTERBOX_PAD_VALUE).
    assert tensor[0, 0, :, 0].mean() == pytest.approx(114, abs=1)
    assert tensor[0, 79, :, 0].mean() == pytest.approx(114, abs=1)
    # Центральные rows — это исходный кадр (серый 128).
    assert tensor[0, 200, :, 0].mean() == pytest.approx(128, abs=1)


# ============================================================================
# StubSource
# ============================================================================

def test_stub_source_emits_one_synthetic_frame():
    """StubSource(period_sec=0) — один кадр, потом stop."""
    src = gaze_mod.StubSource(period_sec=0.0)
    frames = list(src.frames())
    assert len(frames) == 1
    frame = frames[0]
    assert frame.source_name == 'stub'
    assert frame.frame_id == 'stub_frame'
    assert frame.scale == 1.0
    assert frame.pad_left == 0
    assert frame.pad_top == 0


def test_stub_source_wait_for_first_frame_returns_immediately():
    """wait_for_first_frame() — синхронный, без таймаута."""
    import time
    src = gaze_mod.StubSource()
    start = time.monotonic()
    frame = src.wait_for_first_frame(timeout_sec=10.0)
    elapsed = time.monotonic() - start
    assert elapsed < 0.5, f'stub wait_for_first_frame took {elapsed:.2f}s (slow)'
    assert frame.source_name == 'stub'


# ============================================================================
# make_source factory
# ============================================================================

def test_make_source_stub_returns_stub_source():
    """make_source('stub', node=None) → StubSource, не дожидается кадра."""
    # node=None допустим для stub, потому что StubSource не подписывается.
    src = gaze_mod.make_source(name='stub', node=None, timeout_sec=0.0)
    assert isinstance(src, gaze_mod.StubSource)
    assert src.source_name == 'stub'


def test_make_source_unknown_raises_value_error():
    """make_source('nonexistent') → ValueError."""
    with pytest.raises(ValueError) as exc_info:
        gaze_mod.make_source(name='nonexistent', node=None)
    assert 'Unknown gaze source' in str(exc_info.value)
    # Имя должно быть в сообщении для диагностики.
    assert 'nonexistent' in str(exc_info.value)


# ============================================================================
# GazeSourceUnavailable — capability-honest error
# ============================================================================

def test_gaze_source_unavailable_contains_diagnostic_info():
    """GazeSourceUnavailable должен содержать source_name, topic, waited_sec.

    ADR-0018 capability-honest: оператор должен видеть в логе, КАКОЙ
    источник ждали, КАКОЙ топик, СКОЛЬКО секунд.
    """
    exc = gaze_mod.GazeSourceUnavailable(
        source_name='oak_d',
        topic='/camera/camera/color/image_raw',
        waited_sec=10.0,
    )
    msg = str(exc)
    assert 'oak_d' in msg
    assert '/camera/camera/color/image_raw' in msg
    assert '10.0s' in msg
    assert exc.source_name == 'oak_d'
    assert exc.topic == '/camera/camera/color/image_raw'
    assert exc.waited_sec == 10.0
