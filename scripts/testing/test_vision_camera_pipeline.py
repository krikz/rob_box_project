#!/usr/bin/env python3
"""Validate Vision Pi camera pipeline configuration."""

from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
VISION_COMPOSE = PROJECT_ROOT / "docker/vision/docker-compose.yaml"
OAK_D_CONFIG = PROJECT_ROOT / "docker/vision/config/oak-d/oak_d_config.yaml"


def main() -> int:
    compose = yaml.safe_load(VISION_COMPOSE.read_text())
    services = compose.get("services", {})
    assert "rtabmap-sync" not in services, "Legacy rtabmap-sync service should be removed"

    config = yaml.safe_load(OAK_D_CONFIG.read_text())
    ros_params = config["/camera/camera"]["ros__parameters"]
    camera_params = ros_params["camera"]
    color_params = ros_params["color"]

    assert camera_params["i_stereo_width"] == 1280
    assert camera_params["i_stereo_height"] == 720
    assert color_params["i_resolution"] == "1080P"
    assert color_params["i_width"] == 1280
    assert color_params["i_height"] == 720
    assert color_params["i_output_isp"] is True
    assert color_params["i_isp_num"] == 2
    assert color_params["i_isp_den"] == 3

    print("Vision camera pipeline configuration is valid.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())