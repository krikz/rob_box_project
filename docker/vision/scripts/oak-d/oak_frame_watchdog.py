#!/usr/bin/env python3
"""Честный сторож кадров OAK-D (issue #4532, capability-honest, ADR-0018).

Старый healthcheck ``pgrep camera_node`` показывал ``healthy``, когда камеру
переткнули: нода осталась на мёртвом XLink, каждую секунду писала
``X_LINK_ERROR``, кадров не было, а ``restarts=0``. Docker сам по unhealthy
контейнер не перезапускает.

Три режима одним файлом (решение принимает код ``verdict``, не текст в промпте):

* ``run``   — запускается из start_oak_d.sh рядом с ``ros2 launch``: подписка на
              топик кадров (по умолчанию camera_info цветной камеры — публикуется
              вместе с каждым кадром и в сотни раз легче image_raw), на каждый
              кадр обновляет heartbeat-файл. Если кадров нет дольше порога, выходит
              с кодом 1; start_oak_d.sh тогда гасит launch и выходит сам, а
              ``restart: unless-stopped`` поднимает контейнер на свежем устройстве.
* ``check`` — healthcheck контейнера: 0 / 1 по тому же ``verdict``.
* (без режима) — ошибка использования.

ENV: OAK_FRAME_TOPIC, OAK_FRAME_STALE_SEC (30), OAK_START_GRACE_SEC (90),
OAK_HEARTBEAT_PATH, OAK_STARTED_PATH.
"""
from __future__ import annotations

import os
import sys
import time
from typing import Optional

DEFAULT_TOPIC = "/camera/camera/color/camera_info"
DEFAULT_STALE_SEC = 30.0
# Холодный старт драйвера: USB-инициализация DepthAI + прошивка в камеру, 60 с мало на медленной SD.
DEFAULT_GRACE_SEC = 90.0
DEFAULT_HEARTBEAT_PATH = "/tmp/oak_frame_heartbeat"
DEFAULT_STARTED_PATH = "/tmp/oak_watchdog_started"

OK = "ok"
STARTING = "starting"
STALE = "stale"


def verdict(
    now: float,
    last_frame: Optional[float],
    started: Optional[float],
    stale_sec: float = DEFAULT_STALE_SEC,
    grace_sec: float = DEFAULT_GRACE_SEC,
) -> str:
    """``ok`` — кадры свежие; ``starting`` — кадров нет, но идёт льготный срок; ``stale`` — кадров нет дольше порога.

    ``started`` — когда стартовал сторож (None: сторожа нет, значит нечему отвечать за кадры, это stale).
    Часы пошли назад (``now`` раньше отметки) — это не свежесть: считаем stale.
    """
    if started is None:
        return STALE
    if last_frame is not None and last_frame >= started:
        age = now - last_frame
        return OK if 0 <= age <= stale_sec else STALE
    age_since_start = now - started
    return STARTING if 0 <= age_since_start <= grace_sec else STALE


def _mtime(path: str) -> Optional[float]:
    try:
        return os.stat(path).st_mtime
    except OSError:
        return None


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


def _paths() -> tuple[str, str]:
    return (
        os.environ.get("OAK_HEARTBEAT_PATH", DEFAULT_HEARTBEAT_PATH),
        os.environ.get("OAK_STARTED_PATH", DEFAULT_STARTED_PATH),
    )


def check(now: Optional[float] = None) -> int:
    """Код выхода healthcheck: 0 — не stale, 1 — stale."""
    hb_path, st_path = _paths()
    state = verdict(
        time.time() if now is None else now,
        _mtime(hb_path),
        _mtime(st_path),
        _env_float("OAK_FRAME_STALE_SEC", DEFAULT_STALE_SEC),
        _env_float("OAK_START_GRACE_SEC", DEFAULT_GRACE_SEC),
    )
    if state == STALE:
        print(f"[oak_frame_watchdog] FAIL: кадров нет дольше порога ({hb_path})", file=sys.stderr)
        return 1
    return 0


def _touch(path: str) -> None:
    with open(path, "a"):
        os.utime(path, None)


def run() -> int:  # pragma: no cover - нужен rclpy и камера
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo

    hb_path, st_path = _paths()
    stale_sec = _env_float("OAK_FRAME_STALE_SEC", DEFAULT_STALE_SEC)
    grace_sec = _env_float("OAK_START_GRACE_SEC", DEFAULT_GRACE_SEC)
    topic = os.environ.get("OAK_FRAME_TOPIC", DEFAULT_TOPIC)

    # Контейнер при рестарте сохраняет /tmp: старый heartbeat нельзя принять за свежий.
    try:
        os.remove(hb_path)
    except FileNotFoundError:
        pass
    _touch(st_path)

    rclpy.init()
    node = rclpy.create_node("oak_frame_watchdog")
    last_touch = [0.0]

    def on_frame(_msg: CameraInfo) -> None:
        now = time.time()
        if now - last_touch[0] >= 1.0:  # не чаще раза в секунду
            last_touch[0] = now
            _touch(hb_path)

    node.create_subscription(CameraInfo, topic, on_frame, qos_profile_sensor_data)
    node.get_logger().info(f"сторож кадров: {topic}, порог {stale_sec}s, льгота {grace_sec}s")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
            state = verdict(time.time(), _mtime(hb_path), _mtime(st_path), stale_sec, grace_sec)
            if state == STALE:
                node.get_logger().error(
                    f"кадров нет дольше {stale_sec}s (X_LINK_ERROR?) — выходим, контейнер перезапустится"
                )
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main(argv: list[str]) -> int:
    if len(argv) == 2 and argv[1] == "check":
        return check()
    if len(argv) == 2 and argv[1] == "run":
        return run()
    print("usage: oak_frame_watchdog.py run|check", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
