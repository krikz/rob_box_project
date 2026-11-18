"""Чистое ядро семантического слоя карты (ADR-0169 §2, #4369).

Копит наблюдения названных классов профиля в Предметы карты. Без ROS,
без клички, без стен, без Ожидаемой видимости: это следующий PR.
Копит только при готовой карте и уверенной локализации (решение Шифу В1).
"""

from __future__ import annotations

import math
import uuid
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

from rob_box_core.object_classes import is_static_inanimate
from rob_box_perception.map_layer.params import MapLayerParams
from rob_box_perception.tracking.assignment import assign

CANDIDATE = "candidate"
CONFIRMED = "confirmed"
MAP_STATUS_OK = "ok"


@dataclass(frozen=True)
class MapObservation:
    """Одна детекция в кадре ``map`` (плоскость пола)."""

    class_name: str
    x: float
    y: float
    distance_m: float
    stamp: float
    position_valid: bool = True
    map_status: str = MAP_STATUS_OK
    extent_m: float = 0.5
    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_yaw: float = 0.0


@dataclass(frozen=True)
class MapItem:
    """Снимок Предмета карты для потребителей."""

    item_id: str
    map_id: str
    class_name: str
    state: str
    x: float
    y: float
    extent_m: float
    hits: int
    views: int
    first_seen: float
    last_seen: float


@dataclass
class _LiveItem:
    item_id: str
    map_id: str
    class_name: str
    state: str
    x: float
    y: float
    extent_m: float
    hits: int
    views: int
    first_seen: float
    last_seen: float
    seen_from: List[Tuple[float, float, float]] = field(default_factory=list)

    def snapshot(self) -> MapItem:
        return MapItem(
            item_id=self.item_id,
            map_id=self.map_id,
            class_name=self.class_name,
            state=self.state,
            x=self.x,
            y=self.y,
            extent_m=self.extent_m,
            hits=self.hits,
            views=self.views,
            first_seen=self.first_seen,
            last_seen=self.last_seen,
        )


def _hypot(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def _yaw_delta(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def _can_accumulate(map_id: str, mapping: bool, localization_ok: bool) -> bool:
    """Решение Шифу В1: копим только по готовой карте при уверенной позе."""
    return bool(map_id) and (not mapping) and localization_ok


def _accept(obs: MapObservation, params: MapLayerParams) -> bool:
    if not is_static_inanimate(obs.class_name):
        return False
    if not obs.position_valid or obs.map_status != MAP_STATUS_OK:
        return False
    if not math.isfinite(obs.x) or not math.isfinite(obs.y):
        return False
    return params.min_distance_m <= obs.distance_m <= params.max_distance_m


def _match_cost(item: _LiveItem, obs: MapObservation, params: MapLayerParams) -> Optional[float]:
    if item.class_name != obs.class_name:
        return None
    dist = _hypot(item.x, item.y, obs.x, obs.y)
    gate = max(params.match_min_m, item.extent_m / 2.0 + params.match_extent_extra_m)
    if dist > gate:
        return None
    return dist


def _is_new_view(item: _LiveItem, obs: MapObservation, params: MapLayerParams) -> bool:
    for px, py, pyaw in item.seen_from:
        moved = _hypot(obs.robot_x, obs.robot_y, px, py) >= params.view_min_move_m
        turned = _yaw_delta(obs.robot_yaw, pyaw) >= params.view_min_yaw_rad
        if not moved and not turned:
            return False
    return True


class MapLayer:
    """Накопитель Предметов карты одной активной карты."""

    def __init__(self, params: Optional[MapLayerParams] = None) -> None:
        self.params = params or MapLayerParams()
        self._items: Dict[str, _LiveItem] = {}

    def snapshots(self) -> Tuple[MapItem, ...]:
        return tuple(item.snapshot() for item in self._ordered())

    def ingest(
        self,
        observations: Sequence[MapObservation],
        *,
        map_id: str,
        mapping: bool,
        localization_ok: bool,
    ) -> Tuple[MapItem, ...]:
        """Один пакет наблюдений одного кадра. Пустой снимок, если ворота закрыты."""
        if not _can_accumulate(map_id, mapping, localization_ok):
            return self.snapshots()
        accepted = tuple(obs for obs in observations if _accept(obs, self.params))
        used = self._update_matched(accepted)
        self._birth(map_id, accepted, used)
        self._confirm_or_forget(accepted[0].stamp if accepted else 0.0)
        return self.snapshots()

    def _ordered(self) -> List[_LiveItem]:
        return sorted(self._items.values(), key=lambda item: item.item_id)

    def _update_matched(self, observations: Sequence[MapObservation]) -> Set[int]:
        live = self._ordered()
        if not live or not observations:
            return set()
        costs = [[_match_cost(item, obs, self.params) for obs in observations] for item in live]
        used: Set[int] = set()
        for item_i, obs_i in assign(costs):
            self._hit(live[item_i], observations[obs_i])
            used.add(obs_i)
        return used

    def _hit(self, item: _LiveItem, obs: MapObservation) -> None:
        n = item.hits + 1
        item.x = (item.x * item.hits + obs.x) / n
        item.y = (item.y * item.hits + obs.y) / n
        item.hits = n
        item.last_seen = obs.stamp
        item.extent_m = max(item.extent_m, obs.extent_m)
        if _is_new_view(item, obs, self.params):
            item.views += 1
            pose = (obs.robot_x, obs.robot_y, obs.robot_yaw)
            item.seen_from.append(pose)
            overflow = len(item.seen_from) - self.params.max_seen_from
            if overflow > 0:
                item.seen_from = item.seen_from[overflow:]

    def _birth(
        self,
        map_id: str,
        observations: Sequence[MapObservation],
        used: Iterable[int],
    ) -> None:
        taken = set(used)
        for index, obs in enumerate(observations):
            if index in taken:
                continue
            item_id = str(uuid.uuid4())
            pose = (obs.robot_x, obs.robot_y, obs.robot_yaw)
            self._items[item_id] = _LiveItem(
                item_id=item_id,
                map_id=map_id,
                class_name=obs.class_name,
                state=CANDIDATE,
                x=obs.x,
                y=obs.y,
                extent_m=obs.extent_m,
                hits=1,
                views=1,
                first_seen=obs.stamp,
                last_seen=obs.stamp,
                seen_from=[pose],
            )

    def _confirm_or_forget(self, stamp: float) -> None:
        drop: List[str] = []
        window = self.params.confirm_window_sec
        for item in self._items.values():
            if item.state != CANDIDATE:
                continue
            if item.hits >= self.params.confirm_hits and item.views >= self.params.confirm_views:
                item.state = CONFIRMED
                continue
            if stamp - item.first_seen > window:
                drop.append(item.item_id)
        for item_id in drop:
            del self._items[item_id]
