#!/usr/bin/env python3
"""Кормушка AprilTag потолочной камеры: прореживание + перевод в mono8.

usb_cam отдаёт ``/ceiling_camera/image_raw`` в ``yuv422_yuy2`` 640×480 15 к/с
(замер 04.11.2026). apriltag_node берёт кадр через
``cv_bridge::toCvShare(img, "mono8")``; умеет ли cv_bridge 3.2.1 (Humble)
переводить ``yuv422_yuy2`` в ``mono8``, не проверено. Яркость Y в YUYV уже
лежит в каждом втором байте, так что перевод здесь стоит одного среза numpy,
и вопрос про cv_bridge снимается.

Нода же прореживает поток: детектору хватит ``rate_hz`` (по умолчанию 5 Гц),
а 15 к/с на Vision Pi — лишняя нагрузка.

Выход (одно пространство имён, image_transport ищет camera_info рядом с
картинкой):

* ``/ceiling_camera/apriltag/image``       — sensor_msgs/Image, mono8;
* ``/ceiling_camera/apriltag/camera_info`` — тот же camera_info с меткой
  времени кадра (CameraSubscriber в apriltag_node сводит их по stamp).

Потребитель обоих — apriltag_node ``ceiling_apriltag``
(docker/vision/config/ceiling-camera/launch/ceiling_apriltag.launch.py).
"""

from __future__ import annotations

import sys
from typing import Optional

import numpy as np

#: Кодировки, которые умеем перевести в mono8. Ключ — ``Image.encoding``,
#: значение — (байт на пиксель, смещение байта яркости внутри пары пикселей).
#: YUYV (yuv422_yuy2): Y0 U Y1 V — яркость в чётных байтах.
#: UYVY (yuv422):      U Y0 V Y1 — яркость в нечётных байтах.
_YUV422_LUMA_OFFSET = {"yuv422_yuy2": 0, "yuyv": 0, "yuv422": 1, "uyvy": 1}


class UnsupportedEncoding(ValueError):
    """Кадр в кодировке, которую кормушка не переводит в mono8."""


def to_mono8(encoding: str, data: bytes, width: int, height: int, step: int) -> np.ndarray:
    """Кадр ``sensor_msgs/Image`` → массив ``uint8`` формы ``(height, width)``.

    ``step`` — длина строки в байтах (может быть больше ``width × bpp``).
    """
    buf = np.frombuffer(data, dtype=np.uint8)
    if buf.size < step * height:
        raise ValueError(f"кадр короче заявленного: {buf.size} < {step}×{height}")
    rows = buf[: step * height].reshape(height, step)
    enc = encoding.lower()
    if enc == "mono8":
        return np.ascontiguousarray(rows[:, :width])
    if enc in _YUV422_LUMA_OFFSET:
        off = _YUV422_LUMA_OFFSET[enc]
        return np.ascontiguousarray(rows[:, off : off + 2 * width : 2])
    if enc in ("rgb8", "bgr8"):
        px = rows[:, : 3 * width].reshape(height, width, 3).astype(np.uint16)
        r, b = (px[..., 0], px[..., 2]) if enc == "rgb8" else (px[..., 2], px[..., 0])
        # BT.601, как cv::cvtColor(RGB2GRAY): 0.299 R + 0.587 G + 0.114 B.
        y = (r * 77 + px[..., 1] * 150 + b * 29 + 128) >> 8
        return y.astype(np.uint8)
    raise UnsupportedEncoding(f"кодировка {encoding!r} не поддерживается")


class RateGate:
    """Пропускает не чаще ``rate_hz`` кадров в секунду по времени кадра."""

    def __init__(self, rate_hz: float) -> None:
        if rate_hz <= 0:
            raise ValueError("rate_hz должен быть > 0")
        self._period = 1.0 / rate_hz
        self._last: Optional[float] = None

    def admit(self, t: float) -> bool:
        # Время назад (перезапуск камеры, сим) — начинаем отсчёт заново.
        if self._last is None or t < self._last or t - self._last >= self._period * 0.999:
            self._last = t
            return True
        return False


def main(argv: Optional[list] = None) -> int:  # pragma: no cover — нужен ROS
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    class CeilingTagFeed(Node):
        def __init__(self) -> None:
            super().__init__("ceiling_tag_feed")
            rate = float(self.declare_parameter("rate_hz", 5.0).value)
            in_image = self.declare_parameter("input_image", "/ceiling_camera/image_raw").value
            in_info = self.declare_parameter("input_camera_info", "/ceiling_camera/camera_info").value
            out_ns = self.declare_parameter("output_ns", "/ceiling_camera/apriltag").value
            self._gate = RateGate(rate)
            self._info: Optional[CameraInfo] = None
            self._warned: set = set()
            self._pub_img = self.create_publisher(Image, f"{out_ns}/image", qos_profile_sensor_data)
            self._pub_info = self.create_publisher(CameraInfo, f"{out_ns}/camera_info", qos_profile_sensor_data)
            self.create_subscription(CameraInfo, in_info, self._on_info, qos_profile_sensor_data)
            self.create_subscription(Image, in_image, self._on_image, qos_profile_sensor_data)
            self.get_logger().info(f"{in_image} → {out_ns}/image (mono8, ≤{rate:g} Гц)")

        def _warn_once(self, key: str, text: str) -> None:
            if key not in self._warned:
                self._warned.add(key)
                self.get_logger().warning(text)

        def _on_info(self, msg: CameraInfo) -> None:
            self._info = msg

        def _on_image(self, msg: Image) -> None:
            if self._info is None:
                self._warn_once("no_info", "нет camera_info — кадры не отдаю")
                return
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if not self._gate.admit(t):
                return
            try:
                mono = to_mono8(msg.encoding, bytes(msg.data), msg.width, msg.height, msg.step)
            except ValueError as exc:
                self._warn_once(f"enc:{msg.encoding}", f"кадр пропущен: {exc}")
                return
            info = self._info
            if (info.width, info.height) != (msg.width, msg.height):
                self._warn_once(
                    "size",
                    f"camera_info {info.width}×{info.height} ≠ кадр {msg.width}×{msg.height}: "
                    "поза тегов будет неверной, нужна калибровка "
                    "(docs/guides/ceiling-camera-calibration.md)",
                )
            out = Image()
            out.header = msg.header
            out.height, out.width = mono.shape
            out.encoding = "mono8"
            out.is_bigendian = 0
            out.step = out.width
            out.data = mono.tobytes()
            info.header = msg.header
            self._pub_info.publish(info)
            self._pub_img.publish(out)

    rclpy.init(args=argv)
    node = CeilingTagFeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main(sys.argv))
