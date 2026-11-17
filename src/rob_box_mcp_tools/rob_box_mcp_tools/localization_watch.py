"""Подписки на сигналы локализации на время одной поездки (#4495).

Решение «верить ли позе» — ``core.localization_check`` (без ROS). Здесь только адаптер: на время
поездки ``NavigateToPose`` нода сервера подписывается на ``/odom`` (ICP-одометрия rtabmap, 10 Гц,
~0.7 КБ) и ``/rtabmap/info`` (1 Гц, ~10 КБ) и снимает подписки сразу после ответа Nav2.
Постоянных подписок нет: как и ``map_pose.OnDemandTf``, сервер не держит поток топиков
между поездками (mcp-server-cpu-loop-2026-08-22).
"""

from __future__ import annotations

from typing import Any, Optional

from .core.localization_check import LIMITS, LocalizationLimits, LocalizationTrack, map_to_odom_from_stats

#: ICP-одометрия rtabmap (``odom_topic:=/odom`` в ``docker/main/docker-compose.yaml``).
ODOM_TOPIC = "/odom"
#: Статистика rtabmap: ``Loop/MapToOdom_*`` — поправка карты ``map → icp_odom``.
INFO_TOPIC = "/rtabmap/info"


class LocalizationWatch:
    """``start()`` → :class:`LocalizationTrack` с живыми подписками, ``stop(track)`` снимает их.

    Нет пакетов сообщений (CI без ROS) — ``start()`` возвращает пустой трек: вердикт ``unchecked``.
    """

    def __init__(self, node: Any, limits: LocalizationLimits = LIMITS) -> None:
        self._node = node
        self._limits = limits
        self._subs = {}

    def start(self) -> LocalizationTrack:
        track = LocalizationTrack(self._limits)
        subs = []
        try:
            from nav_msgs.msg import Odometry  # noqa: PLC0415 — лениво: модуль импортируется в CI без ROS
            from rtabmap_msgs.msg import Info  # noqa: PLC0415

            subs.append(
                self._node.create_subscription(
                    Odometry, ODOM_TOPIC, lambda m: track.on_odom_covariance(float(m.pose.covariance[0])), 10
                )
            )
            subs.append(self._node.create_subscription(Info, INFO_TOPIC, lambda m: _on_info(track, m), 10))
        except Exception as exc:  # noqa: BLE001 — нет подписки = «не проверено», а не падение поездки
            self._node.get_logger().warning(f"#4495 сторож локализации без подписок: {exc}")
        self._subs[id(track)] = subs
        return track

    def stop(self, track: Optional[LocalizationTrack]) -> None:
        for sub in self._subs.pop(id(track), []):
            self._node.destroy_subscription(sub)


def _on_info(track: LocalizationTrack, msg: Any) -> None:
    pose = map_to_odom_from_stats(msg.stats_keys, msg.stats_values)
    if pose is not None:
        track.on_map_to_odom(*pose)
