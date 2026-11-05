"""map_pose — поза робота в кадре карты для точек навигации и «где ты» (#4390).

Баг #4390 (HIL 04.11): ``save_waypoint`` и ``get_current_pose`` брали позу
из ``/odom``, а ``navigate_to_waypoint`` слал цель в кадре ``map``. Дерево
TF робота (``docker/main/docker-compose.yaml``, rtabmap
``odom_frame_id:=icp_odom``; ``controller_manager.yaml``)::

    map ──rtabmap──▶ icp_odom ──icp_odometry──▶ odom ──diff_drive──▶ base_footprint

Поза в ``/odom`` — это ``base_footprint`` в одометрическом кадре, у которого
начало там, где стартовала одометрия; ``map → icp_odom`` — поправка rtabmap,
в общем случае НЕ единичная (в HIL после картографии разошлись ~3 м).
Поэтому поза здесь — трансформ ``map ← base_footprint`` через tf2, а не
снимок ``/odom``.

Нет трансформа — честный отказ со статусом (ADR-0018 capability-honest),
а не одометрия молча и не «последняя известная поза».

Почему слушатель TF только на время запроса. Постоянный
``tf2_ros.TransformListener`` в этой ноде уже был и жёг ~45 % CPU
(подписка на ``/tf`` ~110 Гц, пересборка WaitSet на rmw_zenoh,
mcp-server-cpu-loop-2026-08-22). Поза нужна редко — по команде «запомни
точку» / «где ты», — поэтому :class:`OnDemandTf` заводит буфер и слушатель
на один запрос и снимает подписки сразу после. Сколько занимает первый
трансформ у свежего слушателя через Zenoh, НЕ измерено: таймаут
:data:`DEFAULT_TIMEOUT_SEC` взят меньше потолка FAST-тула (2 с в
``async_executor``), проверка — HIL.

Адаптер tf2 и трансформ — общие с восприятием: ``rob_box_core.tf_lookup``
(PR #4304), второй реализации нет.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Protocol

from rob_box_core.tf_lookup import NoTransform, RigidTransform, Tf2Lookup, TransformStale

#: Кадр, в котором пишутся точки и в который шлётся цель Nav2
#: (``global_frame: map`` в ``docker/main/config/nav2/nav2_params.yaml``).
MAP_FRAME = "map"
#: База робота для Nav2 (``robot_base_frame: base_footprint`` там же).
BASE_FRAME = "base_footprint"
#: Кадр точки, записанной до #4390 (колонки ``frame_id`` не было, NULL).
#: Такие точки и раньше уходили в Nav2 как ``map`` — поведение не меняем,
#: только помечаем (см. PR #4390: записывались из ``/odom``).
LEGACY_WAYPOINT_FRAME = MAP_FRAME

#: Сколько ждать трансформ у свежего слушателя. Не измерено (см. докстринг).
DEFAULT_TIMEOUT_SEC = 1.5

POSE_STATUS_OK = "ok"
#: В дереве TF нет цепочки map ← base_footprint (rtabmap не публикует карту).
POSE_STATUS_NO_TRANSFORM = "no_transform"
#: Цепочка есть, данных не дождались за таймаут.
POSE_STATUS_TRANSFORM_STALE = "transform_stale"
#: tf2 не поднялся (нет tf2_ros, не создался слушатель).
POSE_STATUS_TF_UNAVAILABLE = "tf_unavailable"

#: Причина отказа словами для ``error`` тула: строит код (ADR-0148).
POSE_FAILURE_REASONS: Dict[str, str] = {
    POSE_STATUS_NO_TRANSFORM: f"нет трансформа {MAP_FRAME} → {BASE_FRAME}, карта не публикуется",
    POSE_STATUS_TRANSFORM_STALE: f"трансформ {MAP_FRAME} → {BASE_FRAME} не пришёл вовремя",
    POSE_STATUS_TF_UNAVAILABLE: "tf2 недоступен",
}


@dataclass(frozen=True)
class MapPose:
    """Поза робота в ``frame_id`` или отказ со статусом (координаты тогда None)."""

    status: str
    frame_id: str = MAP_FRAME
    x: Optional[float] = None
    y: Optional[float] = None
    theta: Optional[float] = None
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.status == POSE_STATUS_OK

    @property
    def reason(self) -> str:
        """Причина отказа словами (пусто для ``ok``)."""
        return POSE_FAILURE_REASONS.get(self.status, self.status) if not self.ok else ""

    def as_dict(self) -> Dict[str, Any]:
        """Данные тула: ``{x, y, theta, frame_id}``."""
        return {"x": self.x, "y": self.y, "theta": self.theta, "frame_id": self.frame_id}


class LatestLookup(Protocol):
    """Сеанс поиска TF: последний трансформ с ожиданием, потом ``close``."""

    def latest(self, target: str, source: str, timeout_sec: float) -> RigidTransform: ...

    def close(self) -> None: ...


class MapPoseSource:
    """Вызываемый источник позы: ``source() -> MapPose`` в кадре ``target``.

    ``open_lookup`` открывает сеанс TF на один запрос (прод — :class:`OnDemandTf`
    над нодой, тесты — фейк). Запросы сериализуются: два одновременных тула не
    заводят два слушателя ``/tf``.
    """

    def __init__(
        self,
        open_lookup: Callable[[], LatestLookup],
        *,
        target: str = MAP_FRAME,
        base: str = BASE_FRAME,
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
    ) -> None:
        self._open_lookup = open_lookup
        self._target = target
        self._base = base
        self._timeout_sec = max(0.0, float(timeout_sec))
        self._lock = threading.Lock()

    def __call__(self) -> MapPose:
        with self._lock:
            try:
                session = self._open_lookup()
            except Exception as exc:  # noqa: BLE001 — нет tf2 = честный отказ, не падение тула
                return MapPose(POSE_STATUS_TF_UNAVAILABLE, self._target, detail=str(exc) or type(exc).__name__)
            try:
                transform = session.latest(self._target, self._base, self._timeout_sec)
            except NoTransform as exc:
                return MapPose(POSE_STATUS_NO_TRANSFORM, self._target, detail=str(exc))
            except TransformStale as exc:
                return MapPose(POSE_STATUS_TRANSFORM_STALE, self._target, detail=str(exc))
            finally:
                session.close()
        x, y, _z = transform.translation
        return MapPose(POSE_STATUS_OK, self._target, x=float(x), y=float(y), theta=transform.yaw)


class OnDemandTf:
    """Буфер tf2 и слушатель ``/tf`` на время одного запроса позы (см. докстринг модуля).

    Буфер без ``node``: с нодой ``tf2_ros.Buffer`` заводит сервис ``tf2_frames``
    на каждый запрос. Подписки слушателя — на ноду сервера; их обслуживает
    её ``MultiThreadedExecutor`` в других потоках, пока поток тула ждёт в
    ``lookup_transform``.
    """

    def __init__(self, node: Any) -> None:
        import tf2_ros  # noqa: PLC0415 — лениво: модуль импортируется в CI без ROS

        self._node = node
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, node)
        self._lookup = Tf2Lookup(self._buffer)

    def latest(self, target: str, source: str, timeout_sec: float) -> RigidTransform:
        return self._lookup.latest(target, source, timeout_sec)

    def close(self) -> None:
        """Снять подписки ``/tf`` и ``/tf_static`` слушателя (иначе вернётся CPU-петля)."""
        unregister = getattr(self._listener, "unregister", None)
        if unregister is not None:
            unregister()
            return
        for name in ("tf_sub", "tf_static_sub"):
            sub = getattr(self._listener, name, None)
            if sub is not None:
                self._node.destroy_subscription(sub)
