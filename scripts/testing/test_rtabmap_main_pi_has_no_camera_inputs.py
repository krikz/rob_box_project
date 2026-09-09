from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
MAIN_COMPOSE = REPO_ROOT / "docker/main/docker-compose.yaml"
RTABMAP_YAML = REPO_ROOT / "docker/main/config/rtabmap/rtabmap.yaml"


def test_main_rtabmap_compose_has_no_camera_runtime_topics():
    compose = yaml.safe_load(MAIN_COMPOSE.read_text(encoding="utf-8"))
    command = "\n".join(compose["services"]["rtabmap"]["command"])

    assert "imu_topic:=/camera/imu/data" in command
    assert "apriltag_topic:=/detections" in command
    assert "subscribe_rgbd:=false" in command
    assert "depth:=false" in command


def test_rtabmap_yaml_is_aligned_with_lidar_only_runtime():
    config = yaml.safe_load(RTABMAP_YAML.read_text(encoding="utf-8"))
    params = config["/rtabmap/rtabmap"]["ros__parameters"]

    assert params["subscribe_scan"] is True
    assert params["subscribe_rgbd"] is False
    assert params["subscribe_depth"] is False
    assert params["Grid/Sensor"] == "0"