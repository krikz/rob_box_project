#!/usr/bin/env python3
"""gaze.py — модуль «Взгляд» (ADR-0104, issue #2531).

Единый шов получения кадра из любого поддерживаемого источника. Скрывает:
- выбор ROS-топика и типа сообщения (CompressedImage vs Image),
- JPEG/PNG декодирование,
- BGR → RGB (YOLOv8n HEF обучен на RGB; cv2.imdecode отдаёт BGR),
- метаданные кадра (frame_id, stamp, source_name),
- выровненную глубину + intrinsics RGB-камеры, синхронизированные с кадром
  по stamp (ADR-0138, этап 1 ADR-0130). Нет глубины — честный статус в
  ``Frame.depth_status``, а не молчаливые нули.

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
                    |  FrameSource  |   Protocol: poll_latest() -> Optional[Frame]
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

Шов обязан быть **честным**: если источник недоступен, ``wait_for_first_frame``
должен либо отдать первый кадр за разумный таймаут, либо бросить
``GazeSourceUnavailable`` с указанием ожидаемого топика и затраченного
времени. Никакого silent "жив-но-слепой" режима (ADR-0018 capability-honest,
issue #2531 acceptance #1).

**Не** зависит от rclpy на уровне модуля — импорт rclpy ленивый
(``gaze.py`` тестируется в CI без colcon-build).

Touchpoints:
- ADR-0104 (этот issue, docs/adr/0104-perception-gaze-seam.md).
- ADR-0089 §3 touchpoint #3 (vision_hailo_node → Gaze).
- ADR-0110 (launch-файл vision_hailo.launch.py — выбор источника).
- Issue #2531 — «Взгляд»: единый шов источника кадра.
"""

from __future__ import annotations

import collections
import dataclasses
import logging
import time
from dataclasses import dataclass
from typing import Any, Iterator, Optional, Protocol, Sequence, Tuple

from rob_box_perception.observation_geometry import (
    STATUS_CAMERA_INFO_SIZE_MISMATCH,
    STATUS_DEPTH_ENCODING_UNSUPPORTED,
    STATUS_DEPTH_OUT_OF_SYNC,
    STATUS_DEPTH_SIZE_MISMATCH,
    STATUS_NO_CAMERA_INFO,
    STATUS_NO_DEPTH_STREAM,
    STATUS_OK,
)

_LOG = logging.getLogger(__name__)

#: Допуск синхронизации depth ↔ RGB по stamp, с (ADR-0138). При 5 fps
#: соседние кадры идут через 0.2 с — 0.15 с отделяет «тот же кадр» от
#: «соседнего». На роботе не замерено.
DEFAULT_DEPTH_SYNC_TOLERANCE_SEC = 0.15


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
            пробрасывается в Observation.source.
        depth_mm: HxW uint16 numpy.ndarray, глубина в мм (0 = нет данных),
            выровненная по RGB (тот же размер), ближайшая по stamp; или
            None (ADR-0138).
        intrinsics: (fx, fy, cx, cy) RGB-камеры в пикселях этого кадра из
            camera_info; или None.
        depth_status: ``ok`` только когда есть и глубина, и intrinsics,
            согласованные с кадром; иначе причина из
            ``observation_geometry.STATUS_*`` (capability-honest).
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
    depth_mm: Any = None
    intrinsics: Optional[Tuple[float, float, float, float]] = None
    depth_status: str = STATUS_NO_DEPTH_STREAM

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

    Кадр складывается ROS-колбэком ``_on_msg`` в поле ``_latest``, а
    ``poll_latest()`` просто читает это поле **без спина** (non-blocking).
    Это фикс issue #2602: старый ``frames()`` вызывал ``rclpy.spin_once``
    из колбэка таймера ноды — рекурсивный spin блокировал executor, и
    нода переставала публиковать события.
    """

    source_name: str
    topic: str

    def poll_latest(self) -> Optional[Frame]: ...
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


def select_nearest_by_stamp(
    items: Sequence[Tuple[float, Any]],
    stamp: float,
    tolerance_sec: float,
) -> Tuple[Optional[Any], Optional[float]]:
    """Ближайший по stamp элемент ``(stamp, item)`` в пределах допуска.

    Returns:
        (item, |dt|) — если ближайший в допуске; (None, |dt|) — если
        ближайший дальше допуска; (None, None) — если ``items`` пуст.
    """
    best = None
    best_dt = None
    for item_stamp, item in items:
        dt = abs(float(item_stamp) - float(stamp))
        if best_dt is None or dt < best_dt:
            best, best_dt = item, dt
    if best_dt is None:
        return None, None
    if best_dt > tolerance_sec:
        return None, best_dt
    return best, best_dt


def _decode_depth_msg(msg: Any) -> Tuple[Optional[Any], str]:
    """sensor_msgs/Image (16UC1 | mono16, мм) → (HxW uint16 ndarray, статус)."""
    if msg.encoding not in ('16UC1', 'mono16'):
        return None, STATUS_DEPTH_ENCODING_UNSUPPORTED
    import numpy as np  # type: ignore[import-not-found]

    h, w = int(msg.height), int(msg.width)
    step = int(msg.step) or w * 2
    dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
    arr = np.frombuffer(bytes(msg.data), dtype=dtype)
    if step % 2 or arr.size < h * (step // 2):
        return None, STATUS_DEPTH_ENCODING_UNSUPPORTED
    depth = arr[:h * (step // 2)].reshape((h, step // 2))[:, :w]
    return depth.astype(np.uint16, copy=False), STATUS_OK


def _intrinsics_from_camera_info(
    msg: Any,
) -> Optional[Tuple[float, float, float, float, int, int]]:
    """sensor_msgs/CameraInfo → (fx, fy, cx, cy, width, height) или None."""
    k = list(getattr(msg, 'k', None) or [])
    if len(k) != 9 or float(k[0]) <= 0.0 or float(k[4]) <= 0.0:
        return None
    return (
        float(k[0]), float(k[4]), float(k[2]), float(k[5]),
        int(msg.width), int(msg.height),
    )


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

    Глубина (ADR-0138): дополнительно подписывается на выровненную по RGB
    глубину ``depth_topic`` (16UC1, мм; ``i_align_depth: true``) и на
    ``camera_info_topic`` RGB-камеры. К кадру прикладывается depth,
    ближайший по stamp в пределах ``depth_sync_tolerance_sec``; иначе
    ``Frame.depth_status`` говорит почему глубины нет. Depth-кадр может
    прийти позже RGB — тогда связывание повторяется при его приходе.
    TF здесь не используется: наблюдения уезжают в optical frame,
    в base_link/map переводит трекер на Main Pi (ADR-0130 §2.11).
    """

    source_name = 'oak_d'
    topic = '/camera/camera/color/image_raw'
    depth_topic = '/camera/camera/depth/image_rect_raw'
    camera_info_topic = '/camera/camera/color/camera_info'
    #: Сколько последних depth-сообщений держать для поиска по stamp.
    _DEPTH_BUFFER_LEN = 8

    def __init__(
        self,
        node: Any,
        depth_enabled: bool = True,
        depth_sync_tolerance_sec: float = DEFAULT_DEPTH_SYNC_TOLERANCE_SEC,
    ) -> None:
        """Подписаться на RGB и (если ``depth_enabled``) на глубину OAK-D.

        Args:
            node: rclpy.node.Node — нужен для ``create_subscription`` и логов.
            depth_enabled: подписываться ли на depth + camera_info.
                False → все кадры с ``depth_status=no_depth_stream``.
            depth_sync_tolerance_sec: допуск |stamp_rgb − stamp_depth|.
        """
        self._node = node
        self._latest: Optional[Frame] = None
        self._stopped = False
        self._depth_enabled = bool(depth_enabled)
        self._depth_sync_tolerance_sec = float(depth_sync_tolerance_sec)
        self._depth_buffer: Any = collections.deque(maxlen=self._DEPTH_BUFFER_LEN)
        self._camera_info: Optional[Tuple[float, float, float, float, int, int]] = None
        self._depth_sub = None
        self._info_sub = None

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
        if self._depth_enabled:
            from sensor_msgs.msg import CameraInfo  # type: ignore
            self._depth_sub = self._node.create_subscription(
                Image, self.depth_topic, self._on_depth, 10,
            )
            self._info_sub = self._node.create_subscription(
                CameraInfo, self.camera_info_topic, self._on_camera_info, 10,
            )
            self._node.get_logger().info(
                f'[{self.source_name}] depth: {self.depth_topic} + '
                f'{self.camera_info_topic}, sync tolerance '
                f'{self._depth_sync_tolerance_sec:.3f}s'
            )
        else:
            self._node.get_logger().warning(
                f'[{self.source_name}] depth disabled (depth_enabled=false) — '
                f'наблюдения без 3D, position_status={STATUS_NO_DEPTH_STREAM}'
            )

    def _on_msg(self, msg: Any) -> None:
        if self._stopped:
            return
        rgb = _decode_image_msg_to_rgb(msg)
        if rgb is None:
            return
        self._latest = self._attach_depth(Frame(
            rgb=rgb,
            scale=1.0,
            pad_left=0,
            pad_top=0,
            original_w=rgb.shape[1],
            original_h=rgb.shape[0],
            frame_id=str(getattr(msg.header, 'frame_id', '') or 'unknown'),
            stamp=_ros_stamp_to_float(msg.header.stamp),
            source_name=self.source_name,
        ))

    def _on_depth(self, msg: Any) -> None:
        if self._stopped:
            return
        self._depth_buffer.append((_ros_stamp_to_float(msg.header.stamp), msg))
        # Depth мог прийти позже своего RGB-кадра — повторить связывание.
        # Только для «ещё не нашли пару»: остальные статусы новым depth-
        # сообщением не лечатся, а декод 1280x720 стоит CPU.
        latest = self._latest
        if latest is not None and latest.depth_status in (
            STATUS_NO_DEPTH_STREAM, STATUS_DEPTH_OUT_OF_SYNC,
        ):
            self._latest = self._attach_depth(latest)

    def _on_camera_info(self, msg: Any) -> None:
        if self._stopped:
            return
        self._camera_info = _intrinsics_from_camera_info(msg)

    def _attach_depth(self, frame: Frame) -> Frame:
        """Вернуть копию кадра с ближайшей по stamp глубиной и статусом."""
        depth_mm = None
        intrinsics = None
        depth_msg, _dt = select_nearest_by_stamp(
            self._depth_buffer, frame.stamp, self._depth_sync_tolerance_sec,
        )
        if not self._depth_buffer:
            status = STATUS_NO_DEPTH_STREAM
        elif depth_msg is None:
            status = STATUS_DEPTH_OUT_OF_SYNC
        else:
            depth_mm, status = _decode_depth_msg(depth_msg)
            if depth_mm is not None and depth_mm.shape[:2] != (
                frame.original_h, frame.original_w,
            ):
                depth_mm, status = None, STATUS_DEPTH_SIZE_MISMATCH
        if status == STATUS_OK:
            info = self._camera_info
            if info is None:
                status = STATUS_NO_CAMERA_INFO
            elif (info[4], info[5]) != (frame.original_w, frame.original_h):
                status = STATUS_CAMERA_INFO_SIZE_MISMATCH
            else:
                intrinsics = info[:4]
        return dataclasses.replace(
            frame, depth_mm=depth_mm, intrinsics=intrinsics, depth_status=status,
        )

    def poll_latest(self) -> Optional[Frame]:
        """Вернуть последний декодированный кадр **без спина**.

        Кадр обновляется ROS-колбэком ``_on_msg`` (его вызывает executor
        ноды при приходе сообщения); здесь мы только читаем поле. Это
        фикс issue #2602: вызов ``rclpy.spin_once`` отсюда (из колбэка
        таймера) был рекурсивным спином, блокировавшим executor.
        """
        return self._latest

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
        for sub in (self._sub, self._depth_sub, self._info_sub):
            if sub is None:
                continue
            try:
                self._node.destroy_subscription(sub)
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

    Глубины нет: ``Frame.depth_status=no_depth_stream`` (ADR-0138).

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

    def poll_latest(self) -> Optional[Frame]:
        """Вернуть последний декодированный кадр **без спина** (issue #2602)."""
        return self._latest

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

    Глубины нет: ``Frame.depth_status=no_depth_stream`` (ADR-0138).

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

    def poll_latest(self) -> Optional[Frame]:
        """Stub не callback-driven — кадр для инференса не нужен.

        ``vision_hailo_node`` в stub-режиме зовёт ``_loader.infer(image=None)``
        и не читает кадр от источника, поэтому здесь честный ``None``
        (реализация ради контракта :class:`FrameSource`).
        """
        return None

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
    depth_enabled: bool = True,
    depth_sync_tolerance_sec: float = DEFAULT_DEPTH_SYNC_TOLERANCE_SEC,
) -> FrameSource:
    """Сконструировать FrameSource по имени и дождаться первого кадра.

    Args:
        name: имя адаптера (``oak_d`` / ``ceiling_camera`` / ``stub``).
        node: rclpy.node.Node — нужен адаптерам, использующим ROS.
        timeout_sec: сколько секунд ждать первый кадр от реального
            источника перед тем как бросить :class:`GazeSourceUnavailable`.
        depth_enabled / depth_sync_tolerance_sec: глубина для ``oak_d``
            (ADR-0138); остальные источники глубины не имеют.

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
        src: FrameSource = OakDSource(
            node,
            depth_enabled=depth_enabled,
            depth_sync_tolerance_sec=depth_sync_tolerance_sec,
        )
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
