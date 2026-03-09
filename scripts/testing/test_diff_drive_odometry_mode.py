from pathlib import Path

import yaml


CONFIG_PATH = Path(__file__).resolve().parents[2] / "docker/main/config/controllers/controller_manager.yaml"


def load_diff_drive_params() -> dict:
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    return config["diff_drive_controller"]["ros__parameters"]


def test_open_loop_matches_no_position_feedback() -> None:
    params = load_diff_drive_params()

    assert params["position_feedback"] is False
    assert params["open_loop"] is True