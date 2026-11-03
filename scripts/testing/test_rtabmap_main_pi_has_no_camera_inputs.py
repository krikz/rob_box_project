from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
MAIN_COMPOSE = REPO_ROOT / "docker/main/docker-compose.yaml"
RTABMAP_YAML = REPO_ROOT / "docker/main/config/rtabmap/rtabmap.yaml"


def test_main_rtabmap_compose_has_no_camera_runtime_topics():
    compose = yaml.safe_load(MAIN_COMPOSE.read_text(encoding="utf-8"))
    command = "\n".join(compose["services"]["rtabmap"]["command"])

    # IMU выключен по умолчанию: дефолт — мёртвое имя, включение через
    # RTABMAP_IMU_TOPIC (сырой IMU OAK-D имеет фиктивную orientation, #3106).
    assert "imu_topic:=${RTABMAP_IMU_TOPIC:-/camera/imu/data}" in command
    # Аргумент rtabmap.launch.py называется tag_topic; apriltag_topic молча
    # игнорировался — переименован в 91e94a01 (#670, PR #1157).
    assert "tag_topic:=/detections" in command
    assert "apriltag_topic:=" not in command
    assert "subscribe_rgbd:=false" in command
    assert "depth:=false" in command


def test_rtabmap_yaml_is_aligned_with_lidar_only_runtime():
    config = yaml.safe_load(RTABMAP_YAML.read_text(encoding="utf-8"))
    params = config["/rtabmap/rtabmap"]["ros__parameters"]

    assert params["subscribe_scan"] is True
    assert params["subscribe_rgbd"] is False
    assert params["subscribe_depth"] is False
    assert params["Grid/Sensor"] == "0"


def test_rtabmap_imu_enable_value_matches_published_camera_topic():
    """Значение для включения IMU должно совпадать с тем, что реально публикует OAK-D.

    Камера (rs_compat, namespace camera, имя ноды camera) публикует /camera/camera/imu;
    /camera/imu/data не публикует никто.
    """
    camera_cfg = yaml.safe_load(
        (REPO_ROOT / "docker/vision/config/oak-d/oak_d_config.yaml").read_text(encoding="utf-8")
    )
    assert camera_cfg["/camera/camera"]["ros__parameters"]["camera"]["i_enable_imu"] is True
    text = MAIN_COMPOSE.read_text(encoding="utf-8")
    assert "RTABMAP_IMU_TOPIC=/camera/camera/imu" in text
