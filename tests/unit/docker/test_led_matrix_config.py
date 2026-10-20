"""Guards for the LED matrix volume config (docker/vision/config/led_matrix).

С ros2leds TD-7 композитор читает раскладку панелей из YAML тома, а не из
кода. До этого PR том держал устаревший файл на 253 светодиода без задних
фар и с другим порядком панелей — поднять сабмодуль, не поправив его,
значило перемешать всю ленту на роботе. Раскладка, на которой робот
работал до TD-7 (ros2leds afb263d, зашита в коде), совпадает с YAML пакета;
тест держит том равным ему.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
VOLUME = REPO_ROOT / "docker" / "vision" / "config" / "led_matrix"
PACKAGE_YAML = REPO_ROOT / "src" / "ros2leds" / "led_matrix_compositor" / "config" / "led_matrix_compositor.yaml"

# Раскладка живого робота до TD-7 (ros2leds afb263d, LEDMatrixCompositor.__init__).
LIVE_PANELS = [(5, 5, True)] * 5 + [(8, 8, False)] * 4
LIVE_GROUPS = {
    "main_display": ([0, 1, 2, 3, 4], [5, 1], False, True),
    "wheel_front_left": ([5], [1, 1], False, False),
    "wheel_front_right": ([6], [1, 1], True, True),
    "wheel_rear_left": ([7], [1, 1], False, False),
    "wheel_rear_right": ([8], [1, 1], False, False),
}


def _params(path: Path, node: str) -> dict:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    return raw[node]["ros__parameters"]


@pytest.fixture(scope="module")
def compositor() -> dict:
    return _params(VOLUME / "led_matrix_compositor.yaml", "led_matrix_compositor")


@pytest.fixture(scope="module")
def driver() -> dict:
    return _params(VOLUME / "led_matrix_driver.yaml", "led_matrix_driver")


def test_volume_layout_is_the_live_robot_layout(compositor):
    panels = [(p["width"], p["height"], p["snake_connection"]) for p in compositor["physical_panels"]]
    assert panels == LIVE_PANELS
    groups = {
        g["name"]: (g["physical_indices"], g["arrangement"], g["flip_x"], g["flip_y"])
        for g in compositor["logical_groups"]
    }
    assert groups == LIVE_GROUPS
    assert all(not g["snake_arrangement"] for g in compositor["logical_groups"])
    assert compositor["input_topic"] == "panel_image"
    assert compositor["output_topic"] == "led_matrix/data"


def test_driver_led_count_matches_compositor_chain(compositor, driver):
    total = sum(p["width"] * p["height"] for p in compositor["physical_panels"])
    assert total == 381
    assert driver["num_leds"] == total


def test_current_limit_leaves_headroom_on_the_buck(driver):
    # бак 5 В x 30 Вт = 6 А; лимит должен быть включён и ниже бака
    assert 0 < driver["max_current_ma"] < 6000
    assert driver["queue_depth"] == 1


@pytest.mark.skipif(not PACKAGE_YAML.exists(), reason="сабмодуль src/ros2leds не выкачан")
def test_volume_matches_ros2leds_package_yaml(compositor):
    package = _params(PACKAGE_YAML, "led_matrix_compositor")
    for key in ("physical_panels", "logical_groups", "input_topic", "output_topic", "max_publish_hz"):
        assert compositor[key] == package[key], key
