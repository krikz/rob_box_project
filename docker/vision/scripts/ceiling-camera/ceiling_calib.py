#!/usr/bin/env python3
"""Калибровка потолочной камеры по шахматке без графики.

Инструкция: docs/guides/ceiling-camera-calibration.md.

Два шага:

``capture``   — в контейнере ceiling-camera (rclpy есть, графики нет):
    сохраняет кадры ``/ceiling_camera/image_raw`` в PGM (mono8) раз в
    ``--every`` секунд. Нужен только numpy.

        docker exec -it ceiling-camera bash -c "source /opt/ros/humble/setup.bash && \\
            python3 /scripts/ceiling_calib.py capture --out /tmp/ceiling_calib --count 40"

``calibrate`` — где угодно с OpenCV (``pip install opencv-python numpy pyyaml``):
    ищет углы шахматки, считает K и дисторсию, пишет YAML в формате
    camera_info_manager (тот же, что ost.yaml у camera_calibration).

        python ceiling_calib.py calibrate --images ./ceiling_calib --cols 8 --rows 6 \\
            --square 0.025 --out ceiling_camera.yaml

``--cols``/``--rows`` — число ВНУТРЕННИХ углов (доска 9×7 клеток → 8×6).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Iterable, Optional, Sequence

import numpy as np

CAMERA_NAME = "ceiling_camera"
#: Минимум кадров с найденной доской. camera_calibration требует сопоставимое
#: число; меньше — K и дисторсия плохо обусловлены.
MIN_VIEWS = 15


# --------------------------------------------------------------------- YAML


def _fmt(values: Iterable[float]) -> str:
    return "[" + ", ".join(f"{float(v):.6f}" for v in values) + "]"


def camera_info_yaml(
    width: int,
    height: int,
    k: Sequence[Sequence[float]],
    d: Sequence[float],
    rms: Optional[float] = None,
    views: Optional[int] = None,
    camera_name: str = CAMERA_NAME,
) -> str:
    """YAML для camera_info_manager (usb_cam ``camera_info_url``).

    Кадр не ректифицируется (apriltag_node берёт сырой mono8), поэтому
    ``R = I`` и ``P = [K | 0]``: apriltag_node считает позу по fx, fy, cx, cy из P.
    """
    k = np.asarray(k, dtype=float).reshape(3, 3)
    d = list(np.asarray(d, dtype=float).ravel())
    if len(d) < 5:
        d += [0.0] * (5 - len(d))
    d = d[:5]
    p = np.hstack([k, np.zeros((3, 1))])
    head = [f"# Калибровка {camera_name}: ceiling_calib.py calibrate, {time.strftime('%Y-%m-%d')}"]
    if rms is not None:
        head.append(f"# RMS ошибка репроекции: {rms:.3f} px, кадров с доской: {views}")
    lines = head + [
        f"image_width: {int(width)}",
        f"image_height: {int(height)}",
        f"camera_name: {camera_name}",
        "camera_matrix:",
        "  rows: 3",
        "  cols: 3",
        f"  data: {_fmt(k.ravel())}",
        "distortion_model: plumb_bob",
        "distortion_coefficients:",
        "  rows: 1",
        "  cols: 5",
        f"  data: {_fmt(d)}",
        "rectification_matrix:",
        "  rows: 3",
        "  cols: 3",
        f"  data: {_fmt(np.eye(3).ravel())}",
        "projection_matrix:",
        "  rows: 3",
        "  cols: 4",
        f"  data: {_fmt(p.ravel())}",
    ]
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------- PGM


def write_pgm(path: Path, mono: np.ndarray) -> None:
    """mono8 → бинарный PGM (P5): читается OpenCV, PIL и любым просмотрщиком."""
    h, w = mono.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(np.ascontiguousarray(mono, dtype=np.uint8).tobytes())


# --------------------------------------------------------------- calibrate


def board_points(cols: int, rows: int, square: float) -> np.ndarray:
    """Координаты внутренних углов доски в её плоскости (Z = 0), метры."""
    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return grid


def calibrate(images: Sequence[Path], cols: int, rows: int, square: float):
    """→ (width, height, K, D, rms, число кадров с доской). Нужен OpenCV."""
    import cv2

    obj = board_points(cols, rows, square)
    obj_pts, img_pts, size = [], [], None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    for path in images:
        img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"  пропуск (не читается): {path.name}")
            continue
        if size is None:
            size = img.shape[::-1]
        elif img.shape[::-1] != size:
            raise SystemExit(f"{path.name}: размер {img.shape[::-1]} ≠ {size} — смешаны режимы камеры")
        ok, corners = cv2.findChessboardCorners(img, (cols, rows))
        print(f"  {'доска' if ok else 'нет  '}: {path.name}")
        if not ok:
            continue
        corners = cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), crit)
        obj_pts.append(obj)
        img_pts.append(corners)
    if len(obj_pts) < MIN_VIEWS:
        raise SystemExit(f"доска найдена на {len(obj_pts)} кадрах, нужно ≥ {MIN_VIEWS}")
    rms, k, d, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    return size[0], size[1], k, d.ravel(), float(rms), len(obj_pts)


# ----------------------------------------------------------------- capture


def capture(out: Path, count: int, every: float, topic: str) -> int:  # pragma: no cover — нужен ROS
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from ceiling_tag_feed import to_mono8

    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("ceiling_calib_capture")
    state = {"n": 0, "last": 0.0}

    def on_image(msg: Image) -> None:
        now = time.monotonic()
        if state["n"] >= count or now - state["last"] < every:
            return
        state["last"] = now
        mono = to_mono8(msg.encoding, bytes(msg.data), msg.width, msg.height, msg.step)
        path = out / f"ceiling_{state['n']:03d}.pgm"
        write_pgm(path, mono)
        state["n"] += 1
        print(f"[{state['n']}/{count}] {path} ({msg.width}×{msg.height}) — смените положение доски", flush=True)

    node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
    print(f"Жду кадры {topic}, сохраняю раз в {every:g} с в {out}", flush=True)
    try:
        while rclpy.ok() and state["n"] < count:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


# -------------------------------------------------------------------- main


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture", help="сохранить кадры с камеры (в контейнере)")
    c.add_argument("--out", type=Path, default=Path("/tmp/ceiling_calib"))
    c.add_argument("--count", type=int, default=40)
    c.add_argument("--every", type=float, default=2.0, help="секунд между кадрами")
    c.add_argument("--topic", default="/ceiling_camera/image_raw")
    k = sub.add_parser("calibrate", help="посчитать калибровку по кадрам (нужен OpenCV)")
    k.add_argument("--images", type=Path, required=True)
    k.add_argument("--cols", type=int, required=True, help="внутренних углов по горизонтали")
    k.add_argument("--rows", type=int, required=True, help="внутренних углов по вертикали")
    k.add_argument("--square", type=float, required=True, help="сторона клетки, м")
    k.add_argument("--out", type=Path, default=Path("ceiling_camera.yaml"))
    a = ap.parse_args(argv)
    if a.cmd == "capture":
        return capture(a.out, a.count, a.every, a.topic)
    files = sorted(p for p in a.images.iterdir() if p.suffix.lower() in (".pgm", ".png", ".jpg", ".jpeg"))
    w, h, kmat, d, rms, views = calibrate(files, a.cols, a.rows, a.square)
    a.out.write_text(camera_info_yaml(w, h, kmat, d, rms, views), encoding="utf-8")
    print(f"\n{w}×{h}: fx={kmat[0][0]:.1f} fy={kmat[1][1]:.1f} cx={kmat[0][2]:.1f} cy={kmat[1][2]:.1f}")
    print(f"D={np.round(d, 4).tolist()}  RMS={rms:.3f} px  кадров с доской: {views}")
    print(f"Записано: {a.out}")
    if rms > 1.0:
        print("ВНИМАНИЕ: RMS > 1 px — переснимите (смаз, доска не плоская, мало наклонов)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
