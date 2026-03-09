from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RTABMAP_START = REPO_ROOT / "docker/main/scripts/rtabmap/start_rtabmap.sh"


def test_rtabmap_start_script_publishes_depth_frame_alias():
    script = RTABMAP_START.read_text(encoding="utf-8")

    assert "camera_stereo_camera_frame" in script
    assert "camera_depth_frame" in script
