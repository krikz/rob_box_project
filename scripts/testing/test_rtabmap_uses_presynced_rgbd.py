from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
MAIN_COMPOSE = REPO_ROOT / "docker/main/docker-compose.yaml"


def test_main_rtabmap_uses_presynced_rgbd_stream():
    compose = yaml.safe_load(MAIN_COMPOSE.read_text(encoding="utf-8"))
    command = compose["services"]["rtabmap"]["command"]
    joined = "\n".join(command)

    assert "subscribe_rgbd:=true" in joined
    assert "rgbd_topic:=/rgbd_image" in joined
    assert "subscribe_rgbd:=false" not in joined
    assert "camera_info_topic:=/camera/camera/color/camera_info" not in joined
    assert "rgb_image_transport:=compressed" not in joined
    assert "depth_image_transport:=compressedDepth" not in joined