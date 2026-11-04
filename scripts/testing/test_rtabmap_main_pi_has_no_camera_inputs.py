from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
MAIN_COMPOSE = REPO_ROOT / "docker/main/docker-compose.yaml"
RTABMAP_YAML = REPO_ROOT / "docker/main/config/rtabmap/rtabmap.yaml"


def test_main_rtabmap_compose_has_no_camera_runtime_topics():
    compose = yaml.safe_load(MAIN_COMPOSE.read_text(encoding="utf-8"))
    command = "\n".join(compose["services"]["rtabmap"]["command"])

    # imu_topic:= в compose не задаётся: его дописывает start_rtabmap.sh из
    # RTABMAP_IMU_TOPIC (выход imu_filter_madgwick, решение Шифу 04.11).
    assert "imu_topic:=" not in command
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


def _env(service: dict) -> dict:
    return dict(item.partition("=")[::2] for item in service["environment"])


def test_imu_filter_input_is_published_camera_topic():
    """Вход madgwick — то, что реально публикует OAK-D.

    Камера (rs_compat, namespace camera, имя ноды camera) публикует /camera/camera/imu;
    /camera/imu/data не публикует никто.
    """
    camera_cfg = yaml.safe_load(
        (REPO_ROOT / "docker/vision/config/oak-d/oak_d_config.yaml").read_text(encoding="utf-8")
    )
    assert camera_cfg["/camera/camera"]["ros__parameters"]["camera"]["i_enable_imu"] is True
    services = yaml.safe_load(MAIN_COMPOSE.read_text(encoding="utf-8"))["services"]
    assert _env(services["imu-filter"])["IMU_FILTER_INPUT"] == "${IMU_FILTER_INPUT:-/camera/camera/imu}"


def test_rtabmap_reads_filter_output_and_default_is_imu_off():
    """Выход фильтра = вход rtabmap — одна переменная на оба сервиса.

    ``${VAR:-}``: не задано или пусто → пусто (IMU выключен по умолчанию, решение Шифу
    04.11: «заливаем, но пока не включаем»); включение — RTABMAP_IMU_TOPIC=/imu/data в .env.
    """
    services = yaml.safe_load(MAIN_COMPOSE.read_text(encoding="utf-8"))["services"]
    flag = "${RTABMAP_IMU_TOPIC:-}"
    assert _env(services["rtabmap"])["RTABMAP_IMU_TOPIC"] == flag
    assert _env(services["imu-filter"])["RTABMAP_IMU_TOPIC"] == flag
    assert services["imu-filter"]["image"] == services["rtabmap"]["image"]
    params = yaml.safe_load(
        (REPO_ROOT / "docker/main/config/rtabmap/imu_filter_madgwick.yaml").read_text(encoding="utf-8")
    )["imu_filter_madgwick"]["ros__parameters"]
    # У OAK-D нет магнитометра; TF публикует rtabmap.
    assert params["use_mag"] is False
    assert params["publish_tf"] is False
    assert params["world_frame"] == "enu"
