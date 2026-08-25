"""Camera payload: sensor_msgs/Image → JPEG bytes.

Источник истины: docs/architecture/meta-quest-api.md §4 (camera_rear 0x1001),
docs/plans/2026-08-24-meta-quest-telepresence.md §1.4.

Phase 1 baseline: JPEG через cv_bridge (не нужен /dev/video, работает на
любом sensor_msgs/Image). H.264 — отдельная карточка Phase 2 если
latency > 200 мс (ADR-0027 §4.3).
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
            чтобы можно было тестировать без cv_bridge в dev-env).
        quality: JPEG quality (1-100). Дефолт 75 — баланс bandwidth/качество.

    Returns:
        JPEG bytes.

    Raises:
        ImportError: если cv_bridge или cv2 недоступен (должен быть в Docker image).
        ValueError: если encoding не поддерживается (только rgb8/bgr8/mono8).
    """
    # Ленивый импорт — без cv_bridge / cv2 код читается, но payload не построить.
    from cv_bridge import CvBridge  # type: ignore[import-not-found]
    import cv2  # type: ignore[import-not-found]

    if msg.encoding not in ("rgb8", "bgr8", "mono8"):
        raise ValueError(f"unsupported encoding '{msg.encoding}' " "(supported: rgb8, bgr8, mono8)")
    bridge = CvBridge()
    if msg.encoding == "mono8":
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    elif msg.encoding == "rgb8":
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    else:  # bgr8
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    ok, buf = cv2.imencode(".jpg", cv_img, encode_params)
    if not ok:
        raise RuntimeError("cv2.imencode('.jpg') failed")
    return bytes(buf)


def has_camera_deps() -> bool:
    """Runtime-проверка: cv_bridge + cv2 доступны."""
    try:
        import cv2  # noqa: F401
        from cv_bridge import CvBridge  # noqa: F401

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
