"""Семантический слой карты — чистое ядро без ROS (ADR-0169, #4369).

Нода, хранилище, клички и стены — следующие PR.
"""

from rob_box_perception.map_layer.layer import CANDIDATE, CONFIRMED, MapItem, MapLayer, MapObservation
from rob_box_perception.map_layer.params import MapLayerParams

__all__ = [
    "CANDIDATE",
    "CONFIRMED",
    "MapItem",
    "MapLayer",
    "MapLayerParams",
    "MapObservation",
]
