#!/usr/bin/env python3
"""gaze.py — модуль «Взгляд» (ADR-0104, issue #2531).

Единый шов получения кадра из любого поддерживаемого источника. Скрывает:
- выбор ROS-топика и типа сообщения (CompressedImage vs Image),
- JPEG/PNG декодирование,
- BGR → RGB (YOLOv8n HEF обучен на RGB; cv2.imdecode отдаёт BGR),
- метаданные кадра (frame_id, stamp, source_name).

Архитектура:
    +-------------------+       +-------------------+
    | OakDSource        |       | CeilingCameraSource|
    | (/camera/camera/  |       | (/ceiling_camera/ |
    |  color/image_raw) |       |  image_raw/       |
    | msg=Image         |       |  compressed)      |
    +---------+---------+       +---------+---------+
              |                           |
              +-------------+-------------+
                            v
                    +---------------+
                    |  FrameSource  |   Protocol: frames() -> Iterator[Frame]
                    +-------+-------+
                            v
                    +---------------+
                    | Frame dataclass|
                    |   rgb ndarray  |
                    |   scale,       |
                    |   pad_left,    |
                    |   pad_top,     |
                    |   frame_id,    |
                    |   stamp,       |
                    |   source_name  |
                    +---------------+

Шов обязан быть **честным**: если источник недоступен, ``frames()``
должен либо отдать первый кадр за разумный таймаут, либо бросить
``GazeSourceUnavailable`` с указанием ожидаемого топика и затраченного
времени. Никакого silent "жив-но-слепой" режима (ADR-0018 capability-honest,
issue #2531 acceptance #1).

**Не** зависит от rclpy на уровне модуля — импорт rclpy ленивый
(``gaze.py`` тестируется в CI без colcon-build).

Touchpoints:
- ADR-0104 (этот issue, docs/adr/0104-perception-gaze-seam.md).
- ADR-0089 §3 touchpoint #3 (vision_hailo_node → Gaze).
- ADR-0096 (launch-файл vision_hailo.launch.py — выбор источника).
- Issue #2531 — «Взгляд»: единый шов источника кадра.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Iterator, Optional, Protocol, Tuple

_LOG = logging.getLogger(__name__)


# ============================================================================
# Frame dataclass — то, что возвращает frames()
# ============================================================================

@dataclass
class Frame:
    """Один кадр, готовый к инференсу.

    Attributes:
        rgb: HxWx3 uint8 numpy.ndarray в RGB (НЕ BGR). YOLOv8n HEF обучен
            на RGB; cv2.imdecode отдаёт BGR, поэтому перестановка каналов
            обязательна **до** letterbox, чтобы паддинг заливался уже в
            RGB (фикс issue #2531 acceptance #8).
        scale: коэффициент letterbox — оригинальная ширина/высота кадра,
            умноженная на scale, даёт размер после resize ДО паддинга.
            Используется в bbox-денормализации для обратного
            масштабирования (фикс issue #2531 acceptance #9).
        pad_left / pad_top: пиксели паддинга слева/сверху в letterbox
            (квадратном) тензоре. Нужны для обратной проекции bbox из
            letterbox-space в исходный кадр.
        original_w / original_h: размер исходного кадра до letterbox.
            Полезно для дебага и для VisionEvent-метаданных.
        frame_id: ``header.frame_id`` из ROS-сообщения (например,
            ``oak-d`` или ``ceiling_camera_optical_frame``).
        stamp: ``header.stamp`` как float seconds (ros Time → sec.nanoseconds).
        source_name: имя адаптера (``"oak_d"``, ``"ceiling_camera"``) —
            пробрасывается в VisionEvent.source_camera.
    """

    rgb: Any  # numpy.ndarray — ленивый import numpy, чтобы модуль
              # импортировался в окружениях без numpy.
    scale: float
    pad_left: int
    pad_top: int
    original_w: int
    original_h: int
    frame_id: str
    stamp: float
    source_name: str

    def as_letterbox(self, input_w: int, input_h: int) -> Any:
        """Привести Frame к letterbox-тензору input_w×input_h (NHWC uint8).

        Соглашение о паддинге (issue #2584, зафиксировано явно): **симметричный**
        letterbox, как в стандартном YOLOv8-препроцессинге — паддинг делится
        поровну между противоположными сторонами (``pad // 2`` на одну сторону,
        остаток — на другую), а НЕ top-left (весь паддинг справа/снизу).
        Пример: кадр 320×240 → resize в 640×480 (scale=2.0) → вертикальный
        паддинг 160px делится на pad_top=80 (сверху) и 80 (снизу), а не
        pad_top=0/pad_bottom=160. Это соглашение обязано совпадать с
        ``LetterboxInfo`` / ``_preprocess`` в ``vision_hailo_loader.py`` —
        ``LetterboxInfo.unproject`` вычитает ``pad_left``/``pad_top``, и при
        расхождении соглашений bbox систематически уезжает на величину
        паддинга (регрессия к дефекту, закрытому issue #2531 acceptance #9).

        Args:
            input_w / input_h: целевой размер модели (640×640 для YOLOv8n).

        Returns:
            numpy.ndarray формы (1, input_h, input_w, 3) uint8.
            ``scale``/``pad_left``/``pad_top`` **пересчитываются** — после
            повторного letterbox на уже-RGB кадре они совпадают с теми,
            что записаны в Frame (мы используем их, чтобы не делать
            letterbox дважды — pre-processing делается тут).
        """
        try:
            import cv2  # type: ignore[import-not-found]
            import numpy as np  # type: ignore[import-not-found]
        except ImportError as exc:
            raise ImportError(
                'Frame.as_letterbox() requires opencv-python + numpy. '
                'Установите: pip install opencv-python numpy',
            ) from exc

        h, w = self.rgb.shape[:2]
        scale = min(input_w / w, input_h / h)
        new_w = int(round(w * scale))
        new_h = int(round(h * scale))

        if (w, h) != (new_w, new_h):
            resized = cv2.resize(self.rgb, (new_w, new_h))
        else:
            resized = self.rgb

        pad_w = input_w - new_w
        pad_h = input_h - new_h
        pad_left = pad_w // 2
        pad_top = pad_h // 2
        padded = cv2.copyMakeBorder(
            resized,
            pad_top,
            pad_h - pad_top,
            pad_left,
            pad_w - pad_left,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),  # YOLOv8n letterbox fill (gray).
        )

        if padded.dtype != np.uint8:
            padded = padded.astype(np.uint8)
        tensor = np.expand_dims(padded, axis=0)  # NHWC

        # Обновим метаданные, чтобы вызывающий мог их читать.
        self.scale = scale
        self.pad_left = pad_left
        self.pad_top = pad_top
        return np.ascontiguousarray(tensor)


# ============================================================================
# Ошибки
# ============================================================================

class GazeError(Exception):
    """Базовый класс ошибок модуля Взгляд."""


class GazeSourceUnavailable(GazeError):
    """Источник не отдал кадр за отведённый таймаут (capability-honest).

    Обязательно содержит:
        source_name: имя адаптера.
        topic: ожидаемый ROS-топик.
        waited_sec: сколько секунд реально ждали.
    """

    def __init__(self, source_name: str, topic: str, waited_sec: float):
        self.source_name = source_name
        self.topic = topic
        self.waited_sec = waited_sec
        super().__init__(
            f'Gaze source {source_name!r} не отдал кадр с {topic!r} '
            f'за {waited_sec:.1f}s (capability-honest, ADR-0018).'
        )


# ============================================================================
# Protocol — что отдаёт любой FrameSource
# ============================================================================

class FrameSource(Protocol):
    """Контракт источника кадров.

    Реализации:
        - :class:`OakDSource` — реальная OAK-D камера, msg=Image.
        - :class:`CeilingCameraSource` — потолочная USB-камера, msg=CompressedImage.

    Метод ``frames()`` — **блокирующий** итератор. Возвращает кадры
    по мере поступления. Завершается только при вызове ``stop()``
    (или при исключении).
    """

    source_name: str
    topic: str

    def frames(self) -> Iterator[Frame]: ...
    def wait_for_first_frame(self, timeout_sec: float) -> Frame: ...
    def stop(self) -> None: ...


# ============================================================================
# Helpers — общий код подписки + decode + BGR->RGB
# ============================================================================

def _decode_compressed_to_rgb(data: bytes) -> Optional[Any]:
    """JPEG/PNG → RGB numpy.ndarray (или None если decode упал)."""
    try:
        import cv2  # type: ignore[import-not-found]
        import numpy as np  # type: ignore[import-not-found]
    except ImportError:
        _LOG.warning('cv2/numpy недоступны — декод невозможен')
        return None

    arr = np.frombuffer(data, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _decode_image_msg_to_rgb(msg: Any) -> Optional[Any]:
    """sensor_msgs/Image → RGB numpy.ndarray.

    Если encoding уже rgb8/bgr8/rgba8/bgra8 — поддерживаем напрямую,
    иначе cv2.cvtColor с сообщением ошибки в логе.
    """
    try:
        import cv2  # type: ignore[import-not-found]
        import numpy as np  # type: ignore[import-not-found]
    except ImportError:
        _LOG.warning('cv2/numpy недоступны — декод невозможен')
        return None

    h, w = msg.height, msg.width
    if msg.encoding in ('rgb8', 'bgr8'):
        channels = 3
    elif msg.encoding in ('rgba8', 'bgra8'):
        channels = 4
    elif msg.encoding == 'mono8':
        channels = 1
    else:
        _LOG.warning('Unsupported Image encoding %r; попытка cvtColor', msg.encoding)
        channels = 3  # best-effort

    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    expected = h * w * channels
    if arr.size < expected:
        _LOG.warning('Image msg shorter than expected: %d < %d', arr.size, expected)
        return None
    img = arr.reshape((h, w, channels))
    if msg.encoding == 'bgr8':
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    elif msg.encoding == 'bgra8':
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    elif msg.encoding == 'mono8':
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    return img


def _ros_stamp_to_float(stamp: Any) -> float:
    """rclpy.time.Time → float seconds."""
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except AttributeError:
        # builtin_interfaces.msg.Time — sec + nanosec.
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


# ============================================================================
# OakDSource — реальный адаптер для OAK-D (ros2 topic: /camera/camera/color/image_raw)
# ============================================================================

class OakDSource:
    """Подписка на OAK-D RGB-стрим.

    Topic:   ``/camera/camera/color/image_raw`` (msg=sensor_msgs/Image).
    Это путь, который публикует ``oakd_with_apriltag.launch.py`` с
    ``namespace='camera'``, ``name='camera'`` + ``i_rs_compat: true``
    (rgb → color). См. ``docker/vision/config/oak-d/oak_d_config.yaml``
    и ``docker/vision/config/oak-d/launch/oakd_with_apriltag.launch.py``.

    Если камера выключена или launch не поднял ноду — ``wait_for_first_frame``
    бросает :class:`GazeSourceUnavailable` (capability-honest).
    """

    source_name = 'oak_d'
    topic = '/camera/camera/color/image_raw'

    def __init__(self, node: Any) -> None:
        """Args:
            node: rclpy.node.Node — нужен для ``create_subscription`` и логов.
        """
        self._node = node
        self._latest: Optional[Frame] = None
        self._stopped = False

        from sensor_msgs.msg import Image  # type: ignore
        self._msg_type = Image
        self._sub = self._node.create_subscription(
            Image,
            self.topic,
            self._on_msg,
            10,
        )
        self._node.get_logger().info(
            f'[{self.source_name}] subscribed to {self.topic} (sensor_msgs/Image)'
        )

    def _on_msg(self, msg: Any) -> None:
        if self._stopped:
            return
        rgb = _decode_image_msg_to_rgb(msg)
        if rgb is None:
            return
        self._latest = Frame(
            rgb=rgb,
            scale=1.0,
            pad_left=0,
            pad_top=0,
            original_w=rgb.shape[1],
            original_h=rgb.shape[0],
            frame_id=str(getattr(msg.header, 'frame_id', '') or 'unknown'),
            stamp=_ros_stamp_to_float(msg.header.stamp),
            source_name=self.source_name,
        )

    def frames(self) -> Iterator[Frame]:
        """Блокирующий итератор: spin + yield новейший кадр."""
        import rclpy  # type: ignore
        while not self._stopped:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if self._latest is not None:
                yield self._latest

    def wait_for_first_frame(self, timeout_sec: float) -> Frame:
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            import rclpy  # type: ignore
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._latest is not None:
                return self._latest
        raise GazeSourceUnavailable(
            source_name=self.source_name,
            topic=self.topic,
            waited_sec=time.monotonic() - start,
        )

    def stop(self) -> None:
        self._stopped = True
        try:
            self._node.destroy_subscription(self._sub)
        except Exception:  # noqa: BLE001
            pass


# ============================================================================
# CeilingCameraSource — адаптер для потолочной USB-камеры (msg=CompressedImage)
# ============================================================================

class CeilingCameraSource:
    """Подписка на потолочную USB-камеру.

    Topic:   ``/ceiling_camera/image_raw/compressed`` (msg=sensor_msgs/CompressedImage).
    Используется существующими потребителями (``quest_node.py:2151``,
    ``telegram_node.py:107``), это второй живой сценарий адаптера Взгляда.

    Не используется в production-vision_hailo немедленно — добавляется
    как второй адаптер, чтобы шов «Взгляд» был настоящим (issue #2531
    acceptance #4). Включается через ``gaze_source: ceiling_camera`` в
    launch / YAML.
    """

    source_name = 'ceiling_camera'
    topic = '/ceiling_camera/image_raw/compressed'

    def __init__(self, node: Any) -> None:
        self._node = node
        self._latest: Optional[Frame] = None
        self._stopped = False

        from sensor_msgs.msg import CompressedImage  # type: ignore
        self._msg_type = CompressedImage
        self._sub = self._node.create_subscription(
            CompressedImage,
            self.topic,
            self._on_msg,
            10,
        )
        self._node.get_logger().info(
            f'[{self.source_name}] subscribed to {self.topic} (sensor_msgs/CompressedImage)'
        )

    def _on_msg(self, msg: Any) -> None:
        if self._stopped:
            return
        rgb = _decode_compressed_to_rgb(bytes(msg.data))
        if rgb is None:
            return
        self._latest = Frame(
            rgb=rgb,
            scale=1.0,
            pad_left=0,
            pad_top=0,
            original_w=rgb.shape[1],
            original_h=rgb.shape[0],
            frame_id=str(getattr(msg.header, 'frame_id', '') or 'ceiling_camera'),
            stamp=_ros_stamp_to_float(msg.header.stamp),
            source_name=self.source_name,
        )

    def frames(self) -> Iterator[Frame]:
        import rclpy  # type: ignore
        while not self._stopped:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if self._latest is not None:
                yield self._latest

    def wait_for_first_frame(self, timeout_sec: float) -> Frame:
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            import rclpy  # type: ignore
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._latest is not None:
                return self._latest
        raise GazeSourceUnavailable(
            source_name=self.source_name,
            topic=self.topic,
            waited_sec=time.monotonic() - start,
        )

    def stop(self) -> None:
        self._stopped = True
        try:
            self._node.destroy_subscription(self._sub)
        except Exception:  # noqa: BLE001
            pass


# ============================================================================
# StubSource — для CI/тестов, не требует реальной подписки
# ============================================================================

class StubSource:
    """Заглушка, отдающая синтетический кадр каждые ``period_sec``.

    Используется:
    - в unit-тестах (без rclpy и без cv2 — Frame.rgb может быть пустым).
    - в CI smoke-тестах, когда нет ни OAK-D, ни потолочной камеры.

    Если ``period_sec == 0`` — отдаёт один кадр и завершает итератор.
    """

    source_name = 'stub'
    topic = '<stub>'

    def __init__(self, period_sec: float = 0.0) -> None:
        self._period_sec = period_sec
        self._stopped = False
        self._emitted = 0

    def frames(self) -> Iterator[Frame]:
        import time as _t
        while not self._stopped:
            rgb = _make_synthetic_rgb()  # пустой массив если cv2 нет
            yield Frame(
                rgb=rgb,
                scale=1.0,
                pad_left=0,
                pad_top=0,
                original_w=rgb.shape[1] if rgb is not None else 0,
                original_h=rgb.shape[0] if rgb is not None else 0,
                frame_id='stub_frame',
                stamp=_t.time(),
                source_name=self.source_name,
            )
            self._emitted += 1
            if self._period_sec <= 0:
                self._stopped = True
                return
            _t.sleep(self._period_sec)

    def wait_for_first_frame(self, timeout_sec: float) -> Frame:
        return next(self.frames())

    def stop(self) -> None:
        self._stopped = True


def _make_synthetic_rgb() -> Any:
    """Вернуть 320x240 RGB массив (gray fill) или None без numpy/cv2."""
    try:
        import numpy as np  # type: ignore[import-not-found]
    except ImportError:
        return None
    return np.full((240, 320, 3), 128, dtype=np.uint8)


# ============================================================================
# Factory — выбор адаптера по имени
# ============================================================================

_KNOWN_SOURCES = (
    'oak_d',
    'ceiling_camera',
    'stub',
)


def make_source(
    name: str,
    node: Any,
    timeout_sec: float = 5.0,
) -> FrameSource:
    """Сконструировать FrameSource по имени и дождаться первого кадра.

    Args:
        name: имя адаптера (``oak_d`` / ``ceiling_camera`` / ``stub``).
        node: rclpy.node.Node — нужен адаптерам, использующим ROS.
        timeout_sec: сколько секунд ждать первый кадр от реального
            источника перед тем как бросить :class:`GazeSourceUnavailable`.

    Raises:
        ValueError: если ``name`` не из списка ``_KNOWN_SOURCES``.
        GazeSourceUnavailable: если за ``timeout_sec`` кадр не пришёл.
    """
    if name not in _KNOWN_SOURCES:
        raise ValueError(
            f'Unknown gaze source {name!r}. '
            f'Допустимые: {", ".join(_KNOWN_SOURCES)}.'
        )

    if name == 'oak_d':
        src: FrameSource = OakDSource(node)
    elif name == 'ceiling_camera':
        src = CeilingCameraSource(node)
    else:
        src = StubSource(period_sec=0.0)

    # Для real-источников — дождаться первого кадра с timeout'ом.
    if name in ('oak_d', 'ceiling_camera'):
        first = src.wait_for_first_frame(timeout_sec=timeout_sec)
        node.get_logger().info(
            f'[gaze] first frame from {src.source_name!r}: '
            f'{first.original_w}x{first.original_h} RGB, '
            f'frame_id={first.frame_id!r}'
        )
    return src
