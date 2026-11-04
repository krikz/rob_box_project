"""Guard: боевой Nav2 описывает робота прямоугольником по замеру, а не кругом.

Шифу 04.11 измерил габарит рулеткой: 0.61 м вдоль хода × 0.605 м поперёк.
Круг robot_radius 0.45 в симе (шаг 6) не пускал DWB в проём 0.9 м: 5/15
против 25/25 с прямоугольником. Проверяется, что запустит Nav2 (параметры
костмапов и критики DWB), а не тексты комментариев.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
NAV2_PARAMS = REPO_ROOT / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"

LENGTH_M = 0.61  # вдоль хода (x base_footprint)
WIDTH_M = 0.605  # поперёк (y)
COSTMAPS = ("local_costmap", "global_costmap")


def _params() -> dict:
    return yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))


def _costmap(cm: str) -> dict:
    return _params()[cm][cm]["ros__parameters"]


def _polygon(cm: str) -> list[list[float]]:
    raw = _costmap(cm)["footprint"]
    assert isinstance(raw, str), "Nav2 читает footprint строкой"
    pts = yaml.safe_load(raw)
    assert len(pts) == 4 and all(len(p) == 2 for p in pts), raw
    return pts


@pytest.mark.parametrize("cm", COSTMAPS)
def test_costmap_uses_measured_rectangle_not_radius(cm):
    assert "robot_radius" not in _costmap(cm), f"{cm}: robot_radius рядом с footprint вводит в заблуждение"
    xs = [p[0] for p in _polygon(cm)]
    ys = [p[1] for p in _polygon(cm)]
    assert max(xs) - min(xs) == pytest.approx(LENGTH_M, abs=1e-6)
    assert max(ys) - min(ys) == pytest.approx(WIDTH_M, abs=1e-6)
    # Центр корпуса — у base_footprint: смещение меша base_link.stl 1.4 мм вперёд.
    assert (max(xs) + min(xs)) / 2 == pytest.approx(0.0014, abs=1e-6)
    assert (max(ys) + min(ys)) / 2 == pytest.approx(0.0, abs=1e-6)


def test_local_and_global_footprint_are_the_same():
    assert _polygon("local_costmap") == _polygon("global_costmap")
    assert _costmap("local_costmap")["footprint_padding"] == _costmap("global_costmap")["footprint_padding"]


@pytest.mark.parametrize("cm", COSTMAPS)
def test_inflation_covers_corners_when_rotating(cm):
    """inflation_radius не меньше описанного радиуса (с padding) + 5 см запаса."""
    pad = _costmap(cm)["footprint_padding"]
    circumscribed = max(math.hypot(abs(x) + pad, abs(y) + pad) for x, y in _polygon(cm))
    inflation = _costmap(cm)["inflation_layer"]["inflation_radius"]
    assert inflation >= circumscribed + 0.05, (inflation, circumscribed)


def test_dwb_checks_whole_footprint():
    """BaseObstacle смотрит клетку под центром; с прямоугольником нужен ObstacleFootprint."""
    fp = _params()["controller_server"]["ros__parameters"]["FollowPath"]
    assert fp["plugin"] == "dwb_core::DWBLocalPlanner"
    assert "ObstacleFootprint" in fp["critics"]
    assert "BaseObstacle" not in fp["critics"]
    assert "ObstacleFootprint.scale" in fp
