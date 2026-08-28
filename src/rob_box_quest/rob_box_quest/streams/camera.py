"""Camera payload: sensor_msgs/Image → JPEG bytes.

Источник истины: docs/architecture/meta-quest-api.md §4 (camera_rear 0x1001),
docs/plans/2026-08-24-meta-quest-telepresence.md §1.4.

Phase 1 baseline: JPEG через numpy + cv2.imencode (БЕЗ cv_bridge).
H.264 — отдельная карточка Phase 2 если latency > 200 мс (ADR-0027 §4.3).

Почему без cv_bridge: ros-humble-cv-bridge собран под numpy 1.x, а в образе
quest pip подтянул numpy 2.x → `from cv_bridge import CvBridge` падает
(`AttributeError: _ARRAY_API not found`). Прямое numpy→cv2 конвертирование
от cv_bridge не зависит и работает на numpy 2.x.
"""

from __future__ import annotations

from typing import Optional


def image_to_payload(
    msg,
    *,
    quality: int = 75,
) -> bytes:
    """Encode sensor_msgs/Image → JPEG bytes (для BINARY_FRAME.payload).

    Args:
        msg: sensor_msgs/Image (тип проверяется лениво — модуль не импортируется,
            чтобы можно было тестировать без cv2 в dev-env).
        quality: JPEG quality (1-100). Дефолт 75 — баланс bandwidth/качество.

    Returns:
        JPEG bytes.

    Raises:
        ImportError: если numpy или cv2 недоступен (должны быть в Docker image).
        ValueError: если encoding не поддерживается (только rgb8/bgr8/mono8).
    """
    # Ленивый импорт — без numpy / cv2 код читается, но payload не построить.
    import cv2  # type: ignore[import-not-found]
    import numpy as np  # type: ignore[import-not-found]

    if msg.encoding not in ("rgb8", "bgr8", "mono8"):
        raise ValueError(f"unsupported encoding '{msg.encoding}' " "(supported: rgb8, bgr8, mono8)")

    channels = 1 if msg.encoding == "mono8" else 3
    row_bytes = int(msg.width) * channels
    step = int(msg.step) if getattr(msg, "step", 0) > 0 else row_bytes

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if step != row_bytes:
        # Неконтигуозный буфер (stride): вырезаем полезную область построчно.
        raw = np.ascontiguousarray(raw.reshape(int(msg.height), step)[:, :row_bytes])

    if channels == 3:
        img = raw.reshape(int(msg.height), int(msg.width), channels)
    else:
        img = raw.reshape(int(msg.height), int(msg.width))

    if msg.encoding == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("cv2.imencode('.jpg') failed")
    return bytes(buf)


def has_camera_deps() -> bool:
    """Runtime-проверка: numpy + cv2 доступны (для JPEG-кодирования)."""
    try:
        import cv2  # noqa: F401
        import numpy  # noqa: F401

        return True
    except ImportError:
        return False


def quality_to_bandwidth_hint(quality: int) -> Optional[str]:
    """Утилита для логов: ожидаемый bandwidth hint (только для observability).

    Source: ad-hoc эмпирика для 1280x720 JPEG с этого проекта; НЕ source-of-truth.
    """
    if quality <= 40:
        return "low (~30 KB/frame @ 720p)"
    if quality <= 70:
        return "med (~80 KB/frame @ 720p)"
    if quality <= 85:
        return "high (~150 KB/frame @ 720p)"
    return "max (~250 KB/frame @ 720p)"
