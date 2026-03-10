from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV2_CONFIG = REPO_ROOT / "docker/main/config/nav2/nav2_params.yaml"
NAV2_START = REPO_ROOT / "docker/main/scripts/nav2/start_nav2_direct.sh"
NAV2_DOCKERFILE = REPO_ROOT / "docker/main/nav2/Dockerfile"


def test_local_costmap_uses_scan_only_not_depth_scan_or_rtabmap_cloud():
    config = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))
    voxel_layer = config["local_costmap"]["local_costmap"]["ros__parameters"]["voxel_layer"]

    assert voxel_layer["observation_sources"] == "scan"
    assert "cloud_obstacles" not in voxel_layer
    assert "depth_scan" not in voxel_layer


def test_nav2_start_script_has_no_camera_or_depth_obstacle_pipeline():
    script = NAV2_START.read_text(encoding="utf-8")

    assert "depthimage_to_laserscan_node" not in script
    assert "/camera/camera/depth/image_rect_raw" not in script
    assert "/camera/depth/scan" not in script


def test_nav2_image_has_no_depthimage_to_laserscan_package():
    dockerfile = NAV2_DOCKERFILE.read_text(encoding="utf-8")

    assert "ros-${ROS_DISTRO}-depthimage-to-laserscan" not in dockerfile
