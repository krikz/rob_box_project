"""Регресс-тест issue #2602: ``_poll_gaze`` должен быть non-blocking.

Баг: ``vision_hailo_node._poll_gaze`` (колбэк таймера) вызывал блокирующий
итератор ``gaze.frames()``, который внутри крутил ``rclpy.spin_once`` на
той же ноде. Это рекурсивный spin — executor ноды блокировался, таймер
публикации ``_tick`` не срабатывал, и нода не публиковала ни одного
``VisionEvent`` (при живом процессе, чистом логе и низком CPU).

Фикс: ``_poll_gaze`` читает последний кадр через ``gaze.poll_latest()``
(non-blocking, кадр обновляется ROS-колбэком ``_on_msg`` executor'ом).

Тест детерминированный: конструируем ноду через ``__new__`` (минуя
``__init__``) и дёргаем ``_poll_gaze`` напрямую с duck-typed gaze source.
Требует rclpy для импорта модуля ноды → локально без ROS2 skip, в CI
(colcon test) работает.

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_vision_hailo_poll_gaze.py -o addopts=""
"""

from __future__ import annotations

import pytest

pytest.importorskip('rclpy')

from rob_box_perception.vision_hailo_node import VisionHailoNode  # noqa: E402


# ---------- fakes ---------------------------------------------------------

class _FakeFrame:
    """Минимальный заменитель gaze.Frame (нужны только frame_id/stamp)."""

    def __init__(self, frame_id: str = 'cam', stamp: float = 1.5) -> None:
        self.frame_id = frame_id
        self.stamp = stamp


class _FakeGaze:
    """Duck-typed gaze source: запоминает, что именно дёргал узел."""

    def __init__(self, frame: _FakeFrame | None = None) -> None:
        self._frame = frame
        self.frames_called = False
        self.poll_called = False

    def frames(self):
        self.frames_called = True
        return iter(())

    def poll_latest(self):
        self.poll_called = True
        return self._frame


def _make_bare_node(gaze: _FakeGaze, gaze_source: str = 'oak_d'):
    node = VisionHailoNode.__new__(VisionHailoNode)
    node._gaze = gaze
    node.gaze_source_name = gaze_source
    node._has_received_frame = False
    node._latest_frame = None
    node._last_frame_id = None
    node._last_frame_stamp = None
    return node


# ---------- tests ---------------------------------------------------------

def test_poll_gaze_uses_poll_latest_not_frames():
    """_poll_gaze обязан читать кадр через poll_latest(), не frames().

    Это главный регресс-контракт issue #2602: вызов блокирующего
    ``frames()`` из колбэка таймера и был причиной мёртвой ноды.
    """
    frame = _FakeFrame()
    gaze = _FakeGaze(frame=frame)
    node = _make_bare_node(gaze)

    node._poll_gaze()

    assert gaze.frames_called is False, (
        'issue #2602 regression: _poll_gaze вызвал блокирующий frames() '
        '(рекурсивный rclpy.spin_once из колбэка таймера).'
    )
    assert gaze.poll_called is True
    assert node._latest_frame is frame
    assert node._has_received_frame is True
    assert node._last_frame_id == 'cam'
    assert node._last_frame_stamp == 1.5


def test_poll_gaze_no_frame_keeps_state():
    """Нет кадра — _poll_gaze не должен ничего менять и не блокировать."""
    gaze = _FakeGaze(frame=None)
    node = _make_bare_node(gaze)

    node._poll_gaze()

    assert gaze.frames_called is False
    assert gaze.poll_called is True
    assert node._latest_frame is None
    assert node._has_received_frame is False
    assert node._last_frame_id is None


def test_poll_gaze_stub_short_circuits():
    """stub-источник: short-circuit, ни poll_latest, ни frames не нужны."""
    gaze = _FakeGaze(frame=None)
    node = _make_bare_node(gaze, gaze_source='stub')

    node._poll_gaze()

    assert gaze.frames_called is False
    assert gaze.poll_called is False
    assert node._has_received_frame is True
    assert node._last_frame_id == 'stub_frame'
