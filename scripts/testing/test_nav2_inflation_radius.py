import math
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV2_CONFIG = REPO_ROOT / "docker/main/config/nav2/nav2_params.yaml"


def test_nav2_costmaps_use_reduced_inflation_radius():
    """inflation_radius = описанный радиус габарита с padding + ~0.10 м (как у круга 0.45 + 0.10).

    С 04.11 padding 0.03: описанный 0.473 → 0.57 (было 0.55 при padding 0.01).
    С 05.11 padding 0.02 был временно (HIL, #4365); решением Шифу 05.11 вернули 0.03 —
    безопасность не ослабляем, проём ограничиваем (docs/shifu-decisions.md).
    """
    config = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))

    for cm in ("local_costmap", "global_costmap"):
        params = config[cm][cm]["ros__parameters"]
        pad = params["footprint_padding"]
        corners = yaml.safe_load(params["footprint"])
        circumscribed = max(math.hypot(abs(x) + pad, abs(y) + pad) for x, y in corners)
        inflation = params["inflation_layer"]["inflation_radius"]
        assert inflation == 0.57, (cm, inflation)
        assert round(circumscribed + 0.10, 2) == inflation, (cm, circumscribed)
