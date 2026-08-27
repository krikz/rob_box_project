"""Unit-тест streams.camera.image_to_payload (sensor_msgs/Image → JPEG).

Требует cv2 + cv_bridge + numpy + sensor_msgs — только в Docker image.
На dev-env без ROS — skip через importorskip (как test_quest_bridge.py).
"""

import pytest

pytest.importorskip("cv2", reason="image_to_payload требует cv2 (только в Docker image)")
pytest.importorskip("cv_bridge", reason="image_to_payload требует cv_bridge (только в Docker image)")
pytest.importorskip("sensor_msgs", reason="sensor_msgs/Image требует ROS (только в Docker image)")
import numpy as np  # noqa: E402

from sensor_msgs.msg import Image  # noqa: E402

from rob_box_quest.streams.camera import image_to_payload  # noqa: E402


def _make_image(width: int = 32, height: int = 24, encoding: str = "rgb8") -> Image:
    msg = Image()
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.step = width * 3
    msg.data = np.zeros((height, width, 3), dtype=np.uint8).tobytes()
    return msg


def test_image_to_payload_returns_jpeg_bytes() -> None:
    payload = image_to_payload(_make_image())
    assert isinstance(payload, bytes)
    assert len(payload) > 0
    # JPEG начинается с SOI-маркера 0xFFD8.
    assert payload[:2] == b"\xff\xd8"


def test_image_to_payload_rejects_unsupported_encoding() -> None:
    msg = _make_image()
    msg.encoding = "yuv422"
    with pytest.raises(ValueError):
        image_to_payload(msg)
