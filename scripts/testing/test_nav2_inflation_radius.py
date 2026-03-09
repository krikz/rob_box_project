from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV2_CONFIG = REPO_ROOT / "docker/main/config/nav2/nav2_params.yaml"


def test_nav2_costmaps_use_reduced_inflation_radius():
    config = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))

    local_inflation = config["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]
    global_inflation = config["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]

    assert local_inflation["inflation_radius"] == 0.50
    assert global_inflation["inflation_radius"] == 0.50