"""Unit-тесты streams.provider (CameraProvider, FrameSource).

CameraProvider.start() запускает capture-loop в daemon-thread'ах.
Эти тесты НЕ требуют depthai/cv2 — только структуру.
"""

import time

from rob_box_quest.streams.provider import CameraFrame, CameraProvider


class _FakeSource:
    """Имитация FrameSource для тестов без cv2/depthai."""

    def __init__(self, frames: list[bytes] | None = None, open_ok: bool = True) -> None:
        self._frames = frames or [b"frame1", b"frame2", b"frame3"]
        self._idx = 0
        self._opened = open_ok
        self.closed = False

    def open(self) -> bool:
        return self._opened

    def read(self, timeout_s: float) -> CameraFrame | None:
        if self._idx >= len(self._frames):
            time.sleep(min(timeout_s, 0.01))
            return None
        data = self._frames[self._idx]
        self._idx += 1
        return CameraFrame(device_id="test", encoding="jpeg", data=data, ts_ms=int(time.time() * 1000))

    def close(self) -> None:
        self.closed = True


def test_camera_frame_dataclass():
    f = CameraFrame(device_id="oak:color", encoding="jpeg", data=b"abc", ts_ms=123)
    assert f.device_id == "oak:color"
    assert f.encoding == "jpeg"
    assert f.data == b"abc"
    assert f.ts_ms == 123


def test_camera_provider_starts_and_stops_gracefully():
    received: list[bytes] = []
    provider = CameraProvider(cameras=[("test_cam", "test", 50.0)])  # 50 fps чтобы быстро

    # Без переопределения — _make_source вернёт OpenCvUsbSource который
    # не откроется без cv2. Поэтому проверим только lifecycle метода:
    # start без callback не падает (но threads завершаются сразу если source не откроется).
    def cb(frame: CameraFrame) -> None:
        received.append(frame.data)

    provider.set_callback(cb)
    provider.start()
    time.sleep(0.1)
    provider.stop()
    # Не получили frames (потому что _make_source даёт OpenCvUsbSource → open() False → thread exits).
    # Главное — start/stop не крашат.
    assert isinstance(provider._threads, list)
    assert provider._threads == []  # stop очистил


def test_camera_provider_callback_receives_frames(monkeypatch):
    """Если _make_source даёт FakeSource, callback получает все кадры."""
    received: list[bytes] = []
    provider = CameraProvider(cameras=[("test_cam", "test", 200.0)])
    fake = _FakeSource(frames=[b"a", b"b", b"c", b"d", b"e"])

    # Подменяем через monkeypatch:
    monkeypatch.setattr(provider, "_make_source", lambda source_id: fake)

    def cb(frame: CameraFrame) -> None:
        received.append(frame.data)

    provider.set_callback(cb)
    provider.start()
    time.sleep(0.5)  # 200 fps → ~100 frames за 0.5 с, но у нас всего 5.
    provider.stop()
    assert received[:5] == [b"a", b"b", b"c", b"d", b"e"]


def test_camera_provider_idempotent_start_stop():
    provider = CameraProvider(cameras=[("test_cam", "test", 100.0)])
    provider.start()
    provider.start()  # повторный start — no-op
    assert len(provider._threads) == 1
    provider.stop()
    provider.stop()  # повторный stop — no-op
    assert provider._threads == []
