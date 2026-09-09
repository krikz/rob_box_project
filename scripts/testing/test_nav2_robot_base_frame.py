from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV2_CONFIG = REPO_ROOT / "docker/main/config/nav2/nav2_params.yaml"


BASE_FRAME_SECTIONS = (
    ("bt_navigator", "ros__parameters"),
    ("local_costmap", "local_costmap", "ros__parameters"),
    ("global_costmap", "global_costmap", "ros__parameters"),
    ("behavior_server", "ros__parameters"),
)


def test_nav2_uses_base_footprint_for_navigation_frames():
    config = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))

    for path in BASE_FRAME_SECTIONS:
        section = config
        for key in path:
            section = section[key]
        assert section["robot_base_frame"] == "base_footprint", path
