#!/usr/bin/env python3
"""decoder_node.py — сжатые кадры сцены → сырые Image для узла лица (ADR-0144 §5.2).

В бэге сцены кадры сжаты (JPEG и ``compressedDepth``), а ``gaze.OakDSource``
слушает сырые ``/camera/camera/color/image_raw`` и
``/camera/camera/depth/image_rect_raw`` (``gaze.py:405-407``). Этот узел
живёт ТОЛЬКО в изолированном графе прогона (``replay.py``) и публикует
сырые кадры с ИСХОДНЫМ ``header`` — stamp кадра сцены сохраняется.

Запускается в образе ``vision-hailo`` (там рабочий cv2; в ``voice-assistant``
cv2 собран под NumPy 1.x и не импортируется — ADR-0144 §1.3).

``compressedDepth`` = 12-байтный заголовок (формат + параметры квантования)
+ PNG 16-бит; проверено на пилоте 29.09.2026: 3/3 кадра → uint16 720×1280.
"""

from __future__ import annotations

from typing import Any, Optional

COMPRESSED_DEPTH_HEADER = 12

COLOR_IN = "/camera/camera/color/image_raw/compressed"
DEPTH_IN = "/camera/camera/depth/image_rect_raw/compressedDepth"
COLOR_OUT = "/camera/camera/color/image_raw"
DEPTH_OUT = "/camera/camera/depth/image_rect_raw"


def png_payload(data: bytes) -> bytes:
    """Отрезать заголовок ``compressedDepth``; без PNG-сигнатуры — ValueError."""
    payload = bytes(data[COMPRESSED_DEPTH_HEADER:])
    if not payload.startswith(b"\x89PNG"):
        raise ValueError("compressedDepth: после 12 байт нет PNG-сигнатуры")
    return payload


def _decode(data: bytes, flags: int) -> Optional[Any]:
    import cv2
    import numpy as np

    return cv2.imdecode(np.frombuffer(data, dtype=np.uint8), flags)


def main() -> None:
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image

    class Decoder(Node):
        def __init__(self) -> None:
            super().__init__("scene_replay_decoder")
            self.color_pub = self.create_publisher(Image, COLOR_OUT, 10)
            self.depth_pub = self.create_publisher(Image, DEPTH_OUT, 10)
            self.create_subscription(CompressedImage, COLOR_IN, self.on_color, qos_profile_sensor_data)
            self.create_subscription(CompressedImage, DEPTH_IN, self.on_depth, qos_profile_sensor_data)
            self.counts = {"color": 0, "depth": 0, "errors": 0}
            self.create_timer(10.0, lambda: self.get_logger().info(f"decoded {self.counts}"))
            self.get_logger().info("decoder ready")

        def _publish(self, pub, header, img, encoding: str, bpp: int) -> None:
            msg = Image()
            msg.header = header
            msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
            msg.encoding = encoding
            msg.step = msg.width * bpp
            # НЕ msg.data = bytes: присваивание гонит 2.7 МБ через проверку
            # типа поэлементно — 494 мс на кадр 720p против 4.8 мс у frombytes
            # (замер на Vision Pi 29.09.2026); с присваиванием узел успевал
            # 88 кадров из 376 и прогон шёл на ~1.2 Гц вместо 5.
            msg.data.frombytes(img.tobytes())
            pub.publish(msg)

        def on_color(self, msg) -> None:
            img = _decode(bytes(msg.data), cv2.IMREAD_COLOR)
            if img is None:
                self.counts["errors"] += 1
                return
            self._publish(self.color_pub, msg.header, img, "bgr8", 3)
            self.counts["color"] += 1

        def on_depth(self, msg) -> None:
            try:
                img = _decode(png_payload(bytes(msg.data)), cv2.IMREAD_UNCHANGED)
            except ValueError:
                img = None
            if img is None or img.dtype.itemsize != 2:
                self.counts["errors"] += 1
                return
            self._publish(self.depth_pub, msg.header, img, "16UC1", 2)
            self.counts["depth"] += 1

    rclpy.init()
    node = Decoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
