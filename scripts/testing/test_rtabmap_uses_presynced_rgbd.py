from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
MAIN_COMPOSE = REPO_ROOT / "docker/main/docker-compose.yaml"


def test_main_rtabmap_uses_lidar_only_runtime_for_stability():
    compose = yaml.safe_load(MAIN_COMPOSE.read_text(encoding="utf-8"))
    command = compose["services"]["rtabmap"]["command"]
    joined = "\n".join(command)

    assert "depth:=false" in joined
    assert "subscribe_rgbd:=false" in joined
    assert "subscribe_rgbd:=true" not in joined
    assert "rgbd_topic:=/rgbd_image" not in joined