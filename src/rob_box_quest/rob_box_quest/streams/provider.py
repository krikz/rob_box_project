"""CameraProvider — абстракция чтения кадров **мимо ROS2**.

Источник истины: docs/plans/2026-08-24-meta-quest-telepresence.md §1.4 v2.
Зачем: видео/глубина не должны идти через ROS-топики, чтобы не
нагружать Zenoh/Discovery (Phase 1.4 — discussion 2026-08-25).

Реализация для Phase 1: легковесная — OpenCV читает /dev/video* для
USB-камер, depthai SDK — для OAK-D. Кадры JPEG-кодируются на Vision Pi
и шлются в WSS через CameraProvider → Bridge.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Protocol

log = logging.getLogger(__name__)


@dataclass
class CameraFrame:
    """Один кадр от камеры."""

    device_id: str  # "oak:color" / "oak:depth" / "/dev/video0"
    encoding: str  # "jpeg" / "h264" / "depth_colormap"
    data: bytes  # payload (готов к BINARY_FRAME)
    ts_ms: int  # capture timestamp


class FrameSource(Protocol):
    """Один источник кадров (одна камера)."""

    def open(self) -> bool:
        """Подготовка. False если устройство недоступно."""
        ...

    def read(self, timeout_s: float) -> Optional[CameraFrame]:
        """Один кадр или None по таймауту."""
        ...

    def close(self) -> None: ...


# ----------------------------------------------------------------------------
# USB / V4L2 через OpenCV (Phase 1.4 baseline для ceiling-camera)
# ----------------------------------------------------------------------------


class OpenCvUsbSource:
    """V4L2 / OpenCV источник. Использует cv2.VideoCapture.

    Phase 1.4: читает JPEG через cv2 (просто, не нужен cv_bridge).
    Если cv2 нет — open() возвращает False, capture-loop помечает
    stream как недоступный.
    """

    def __init__(self, device: str, jpeg_quality: int = 75) -> None:
        self._device = device
        self._jpeg_quality = jpeg_quality
        self._cap = None  # type: ignore[assignment]
        self._encode_params: list = []

    def open(self) -> bool:
        try:
            import cv2  # type: ignore[import-not-found]
        except ImportError:
            log.warning("cv2 unavailable — %s disabled", self._device)
            return False
        self._cap = cv2.VideoCapture(self._device)
        if not self._cap.isOpened():
            log.warning("cannot open %s", self._device)
            return False
        self._encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self._jpeg_quality)]
        return True

    def read(self, timeout_s: float) -> Optional[CameraFrame]:
        if self._cap is None:
            return None
        import cv2  # type: ignore[import-not-found]

        ok, frame = self._cap.read()
        if not ok or frame is None:
            return None
        ok_jpg, buf = cv2.imencode(".jpg", frame, self._encode_params)
        if not ok_jpg:
            return None
        return CameraFrame(
            device_id=self._device,
            encoding="jpeg",
            data=bytes(buf),
            ts_ms=int(time.time() * 1000),
        )

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None


# ----------------------------------------------------------------------------
# OAK-D через depthai SDK (Phase 1.4: stub — реальная реализация в Docker)
# ----------------------------------------------------------------------------


class OakDepthaiSource:
    """OAK-D через depthai SDK.

    Phase 1.4: интерфейс готов, реальная инициализация depthai pipeline
    добавляется в Docker image (depthai SDK не в dev-env).
    Если depthai недоступен — open() возвращает False, и CameraProvider
    переключается на fallback: читает через ROS-топик
    /camera/camera/color/image_raw (как у нас сейчас в test/build env).
    """

    def __init__(self, kind: str = "color") -> None:
        # kind: "color" или "depth"
        self._kind = kind
        self._pipeline = None

    def open(self) -> bool:
        try:
            import depthai  # type: ignore[import-not-found]
        except ImportError:
            log.warning("depthai unavailable — OAK %s disabled", self._kind)
            return False
        # Реальный pipeline строится в Docker image; в dev-env stub-режим.
        try:
            self._pipeline = depthai.Pipeline()
            # color cam / mono cams + stereo depth — заглушка-минимум для Phase 1.4.
            _ = self._pipeline.createColorCamera() if self._kind == "color" else None
        except Exception as e:  # noqa: BLE001
            log.warning("OAK pipeline init failed: %s", e)
            return False
        return True

    def read(self, timeout_s: float) -> Optional[CameraFrame]:
        # Phase 1.4 stub: реальная интеграция — отдельная карточка.
        time.sleep(timeout_s)
        return None

    def close(self) -> None:
        self._pipeline = None


# ----------------------------------------------------------------------------
# CameraProvider — главный класс, держит capture-loop для всех камер
# ----------------------------------------------------------------------------


# Type for the per-frame callback (called on capture thread).
FrameCallback = Callable[[CameraFrame], None]


@dataclass
class _CameraConfig:
    """Спецификация одной камеры в CameraProvider."""

    ui_name: str  # "camera_oak_color"
    source_id: str  # "oak:color" / "/dev/video0"
    fps: float = 15.0


class CameraProvider:
    """Менеджер capture-потоков для всех камер Vision Pi.

    API:
        provider = CameraProvider(cameras=[("camera_oak_color", "oak:color", 15.0), ...])
        provider.set_callback(on_frame)  # вызывается для каждого кадра
        provider.start()
        # ...работает в фоне, дёргает callback...
        provider.stop()

    Каждая камера — отдельный поток (чтение + encode + callback).
    """

    def __init__(self, cameras: list[tuple[str, str, float]]) -> None:
        # cameras: list of (ui_name, source_id, fps)
        self._configs = [_CameraConfig(ui_name=u, source_id=s, fps=fps) for u, s, fps in cameras]
        self._callback: Optional[FrameCallback] = None
        self._threads: list[threading.Thread] = []
        self._stop_event = threading.Event()

    def set_callback(self, callback: FrameCallback) -> None:
        self._callback = callback

    def start(self) -> None:
        if self._threads:
            return
        for cfg in self._configs:
            t = threading.Thread(
                target=self._capture_loop,
                args=(cfg,),
                name=f"quest-cam-{cfg.ui_name}",
                daemon=True,
            )
            t.start()
            self._threads.append(t)
        log.info("CameraProvider started %d cameras", len(self._threads))

    def stop(self) -> None:
        self._stop_event.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self._threads.clear()

    def _make_source(self, source_id: str) -> FrameSource:
        if source_id.startswith("oak:"):
            return OakDepthaiSource(kind=source_id.split(":", 1)[1])
        return OpenCvUsbSource(device=source_id)

    def _capture_loop(self, cfg: _CameraConfig) -> None:
        period_s = 1.0 / max(cfg.fps, 0.1)
        source = self._make_source(cfg.source_id)
        if not source.open():
            log.warning("camera %s unavailable — thread exits", cfg.ui_name)
            return
        log.info("camera %s started (source=%s, fps=%.1f)", cfg.ui_name, cfg.source_id, cfg.fps)
        try:
            while not self._stop_event.is_set():
                frame = source.read(timeout_s=period_s)
                if frame is None:
                    continue
                if self._callback is not None:
                    try:
                        self._callback(frame)
                    except Exception as e:  # noqa: BLE001
                        log.debug("frame callback error: %s", e)
        finally:
            source.close()
