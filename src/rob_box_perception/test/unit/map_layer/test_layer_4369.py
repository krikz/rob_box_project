"""Слой пишет на карту только названные классы ADR-0169 (§1.3, #4369).

Запуск:
    python3 -m pytest src/rob_box_perception/test/unit/map_layer -v --no-cov
"""

from __future__ import annotations

import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
_PERCEPTION = _HERE.parents[3]
_CORE = _HERE.parents[4] / "rob_box_core"
for _root in (_PERCEPTION, _CORE):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

from rob_box_core.object_classes import STATIC_INANIMATE_CLASSES  # noqa: E402
from rob_box_perception.map_layer import CANDIDATE, CONFIRMED, MapLayer, MapLayerParams, MapObservation  # noqa: E402

MAP_ID = "map-test"


def _obs(class_name: str, x: float, y: float, stamp: float, **kw) -> MapObservation:
    fields = dict(
        class_name=class_name,
        x=x,
        y=y,
        distance_m=2.0,
        stamp=stamp,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
    )
    fields.update(kw)
    return MapObservation(**fields)


def _confirm_chair(layer: MapLayer, x: float = 1.0, y: float = 2.0) -> None:
    """5 попаданий с двух ракурсов — confirmed (ADR-0169 §2.3)."""
    for i in range(3):
        layer.ingest(
            [_obs("chair", x, y, stamp=float(i), robot_x=0.0, robot_y=0.0)],
            map_id=MAP_ID,
            mapping=False,
            localization_ok=True,
        )
    for i in range(3, 5):
        layer.ingest(
            [_obs("chair", x, y, stamp=float(i), robot_x=1.0, robot_y=0.0)],
            map_id=MAP_ID,
            mapping=False,
            localization_ok=True,
        )


def test_named_class_becomes_confirmed_item_on_map():
    layer = MapLayer()
    _confirm_chair(layer)
    items = layer.snapshots()
    assert len(items) == 1
    item = items[0]
    assert item.class_name == "chair"
    assert item.state == CONFIRMED
    assert item.map_id == MAP_ID
    assert item.x == 1.0
    assert item.y == 2.0
    assert item.hits == 5
    assert item.views >= 2
    assert item.item_id


def test_cup_and_invented_wardrobe_never_enter_the_layer():
    layer = MapLayer()
    layer.ingest(
        [
            _obs("cup", 1.0, 1.0, 0.0),
            _obs("wardrobe", 2.0, 2.0, 0.0),
            _obs("person", 3.0, 3.0, 0.0),
            _obs("shelf", 4.0, 4.0, 0.0),
        ],
        map_id=MAP_ID,
        mapping=False,
        localization_ok=True,
    )
    assert layer.snapshots() == ()


def test_mapping_mode_does_not_accumulate():
    layer = MapLayer()
    layer.ingest(
        [_obs("couch", 1.0, 2.0, 0.0)],
        map_id=MAP_ID,
        mapping=True,
        localization_ok=True,
    )
    assert layer.snapshots() == ()


def test_uncertain_localization_does_not_accumulate():
    layer = MapLayer()
    layer.ingest(
        [_obs("couch", 1.0, 2.0, 0.0)],
        map_id=MAP_ID,
        mapping=False,
        localization_ok=False,
    )
    assert layer.snapshots() == ()


def test_missing_map_id_does_not_accumulate():
    layer = MapLayer()
    layer.ingest(
        [_obs("couch", 1.0, 2.0, 0.0)],
        map_id="",
        mapping=False,
        localization_ok=True,
    )
    assert layer.snapshots() == ()


def test_two_named_classes_stay_two_items():
    layer = MapLayer()
    _confirm_chair(layer, x=1.0, y=0.0)
    for i in range(5):
        robot_x = 0.0 if i < 3 else 1.0
        layer.ingest(
            [_obs("couch", 4.0, 0.0, stamp=10.0 + i, robot_x=robot_x)],
            map_id=MAP_ID,
            mapping=False,
            localization_ok=True,
        )
    names = sorted(item.class_name for item in layer.snapshots())
    assert names == ["chair", "couch"]


def test_far_same_class_does_not_merge():
    layer = MapLayer()
    layer.ingest(
        [_obs("chair", 0.0, 0.0, 0.0), _obs("chair", 3.0, 0.0, 0.0)],
        map_id=MAP_ID,
        mapping=False,
        localization_ok=True,
    )
    items = layer.snapshots()
    assert len(items) == 2
    assert all(item.state == CANDIDATE for item in items)


def test_out_of_range_distance_is_ignored():
    layer = MapLayer()
    layer.ingest(
        [
            _obs("tv", 1.0, 1.0, 0.0, distance_m=0.3),
            _obs("tv", 1.0, 1.0, 1.0, distance_m=5.0),
        ],
        map_id=MAP_ID,
        mapping=False,
        localization_ok=True,
    )
    assert layer.snapshots() == ()


def test_candidate_forgotten_after_confirm_window():
    layer = MapLayer(MapLayerParams(confirm_window_sec=10.0))
    layer.ingest(
        [_obs("bench", 1.0, 1.0, 0.0)],
        map_id=MAP_ID,
        mapping=False,
        localization_ok=True,
    )
    assert len(layer.snapshots()) == 1
    layer.ingest(
        [_obs("bench", 1.0, 1.0, 11.0, robot_x=0.0)],
        map_id=MAP_ID,
        mapping=False,
        localization_ok=True,
    )
    # Попадание в 11 с обновило last_seen, но first_seen=0, окно 10 с,
    # hits < 5 → кандидат забывается.
    assert layer.snapshots() == ()


def test_every_named_class_is_written_to_the_layer():
    layer = MapLayer()
    observations = [_obs(name, float(index), 0.0, 0.0) for index, name in enumerate(STATIC_INANIMATE_CLASSES)]
    layer.ingest(observations, map_id=MAP_ID, mapping=False, localization_ok=True)
    names = sorted(item.class_name for item in layer.snapshots())
    assert names == sorted(STATIC_INANIMATE_CLASSES)


def test_invalid_map_status_skipped():
    layer = MapLayer()
    layer.ingest(
        [_obs("bed", 1.0, 1.0, 0.0, map_status="no_transform")],
        map_id=MAP_ID,
        mapping=False,
        localization_ok=True,
    )
    assert layer.snapshots() == ()
