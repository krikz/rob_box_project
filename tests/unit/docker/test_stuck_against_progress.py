"""Патч #5003: упор в препятствие — свой отказ прогресса, не второй чистый.

Humble SimpleProgressChecker абортит FollowPath только после 30 с без
смещения 0.08 м. У стены DWB смещается вдоль стены, и NavigateRecovery
(ClearThenBackUp, #5000) не тикается. Патч зовёт rob_box_teleop.stuck_against
из controller_server: команда держится, центр стоит, по курсу клетка
препятствия — ``Failed to make progress: stuck against obstacle``.
"""

from __future__ import annotations

import subprocess
import urllib.request
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
NAV2_DIR = REPO / "docker" / "main" / "nav2"
PATCH = NAV2_DIR / "controller_stuck_against.patch"
DOCKERFILE = NAV2_DIR / "Dockerfile"
PINNED_COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"
UPSTREAM_CPP = (
    f"https://raw.githubusercontent.com/ros-navigation/navigation2/"
    f"{PINNED_COMMIT}/nav2_controller/src/controller_server.cpp"
)
UPSTREAM_HPP = (
    f"https://raw.githubusercontent.com/ros-navigation/navigation2/"
    f"{PINNED_COMMIT}/nav2_controller/include/nav2_controller/controller_server.hpp"
)
UPSTREAM_CMAKE = (
    f"https://raw.githubusercontent.com/ros-navigation/navigation2/"
    f"{PINNED_COMMIT}/nav2_controller/CMakeLists.txt"
)

THROW = 'throw nav2_core::PlannerException("Failed to make progress: stuck against obstacle")'


def _fetch(url: str) -> bytes:
    try:
        with urllib.request.urlopen(url, timeout=30) as resp:
            return resp.read()
    except Exception as exc:  # noqa: BLE001 — honest fail if upstream unreachable
        pytest.fail(f"could not fetch pinned Nav2 source: {exc}")


def test_patch_names_the_stuck_failure_and_does_not_touch_thresholds():
    text = PATCH.read_text(encoding="utf-8")
    added = "\n".join(
        line[1:]
        for line in text.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )
    assert THROW in added
    assert "stuckAgainstObstacle" in added
    assert "rob_box_teleop.stuck_against" in added
    assert "reset_controller_state" in added
    # Чужие пороги патч не двигает: радиус и окно остаются в nav2_params.yaml.
    assert "required_movement_radius:" not in added
    assert "movement_time_allowance" not in added
    assert "xy_goal_tolerance" not in added
    assert "terminate_all" not in added


def test_dockerfile_applies_stuck_patch_after_progress_abort_on_pinned_controller():
    df = DOCKERFILE.read_text(encoding="utf-8")
    assert "controller_stuck_against.patch" in df
    assert PINNED_COMMIT in df
    assert df.index("controller_progress_abort.patch") < df.index(
        "controller_stuck_against.patch"
    )
    # Образ подменяет libcontroller_server_core.so — патч должен попасть в эту сборку.
    assert df.index("controller_stuck_against.patch") < df.index(
        "libcontroller_server_core.so"
    )
    assert "python3-dev" in df
    assert "target_link_libraries(${library_name} python3.10)" in PATCH.read_text(
        encoding="utf-8"
    )


def test_patch_applies_after_existing_controller_patches(tmp_path: Path):
    """Все три патча контроллера ложатся на pinned-исходник по порядку (нужна сеть)."""
    src_root = tmp_path / "navigation2"
    cpp = src_root / "nav2_controller" / "src" / "controller_server.cpp"
    hpp = (
        src_root
        / "nav2_controller"
        / "include"
        / "nav2_controller"
        / "controller_server.hpp"
    )
    cmake = src_root / "nav2_controller" / "CMakeLists.txt"
    cpp.parent.mkdir(parents=True)
    hpp.parent.mkdir(parents=True)
    cpp.write_bytes(_fetch(UPSTREAM_CPP))
    hpp.write_bytes(_fetch(UPSTREAM_HPP))
    cmake.write_bytes(_fetch(UPSTREAM_CMAKE))

    order = (
        "controller_goal_tf_fail.patch",
        "controller_progress_abort.patch",
        "controller_stuck_against.patch",
    )
    for name in order:
        result = subprocess.run(
            ["patch", "-p1", "--forward", "--batch"],
            cwd=src_root,
            input=(NAV2_DIR / name).read_text(encoding="utf-8"),
            text=True,
            capture_output=True,
            check=False,
        )
        assert result.returncode == 0, f"{name}: {result.stdout}{result.stderr}"

    after = cpp.read_text(encoding="utf-8")
    assert THROW in after
    assert "stuckAgainstReset();" in after
    assert "have_last_cmd_ = true;" in after
    # Прежний отказ по радиусу патч не убрал.
    assert 'throw nav2_core::PlannerException("Failed to make progress");' in after
    header = hpp.read_text(encoding="utf-8")
    assert "bool stuckAgainstObstacle(" in header
    assert "bool have_last_cmd_{false};" in header
    assert "target_link_libraries(${library_name} python3.10)" in cmake.read_text(
        encoding="utf-8"
    )
