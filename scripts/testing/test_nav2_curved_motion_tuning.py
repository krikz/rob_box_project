from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV2_CONFIG = REPO_ROOT / "docker/main/config/nav2/nav2_params.yaml"


def test_dwb_prefers_curved_motion_over_stop_turn_go_behavior():
    config = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))
    follow_path = config["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = config["velocity_smoother"]["ros__parameters"]

    assert follow_path["min_speed_theta"] == 0.0
    assert follow_path["trans_stopped_velocity"] == 0.03
    assert follow_path["RotateToGoal.scale"] == 8.0
    assert follow_path["PathAlign.scale"] == 24.0
    assert follow_path["GoalAlign.scale"] == 12.0
    assert follow_path["max_vel_theta"] == 0.8
    assert smoother["max_velocity"][2] == 0.8
    assert smoother["max_accel"][2] == 1.0