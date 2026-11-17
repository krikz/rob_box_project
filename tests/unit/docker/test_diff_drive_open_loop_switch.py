"""Переключатель источника колёсной одометрии DIFF_DRIVE_OPEN_LOOP.

Проверяется поведение render_controller_config.py (что получит ros2_control_node)
и то, что compose прокидывает переменную с пустым умолчанием, т.е. деплой без
переменной ничего не меняет. Docker и ROS не нужны.
"""

from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
MAIN = REPO_ROOT / "docker" / "main"
SCRIPT = MAIN / "scripts" / "ros2_control" / "render_controller_config.py"
CONFIG = MAIN / "config" / "controllers" / "controller_manager.yaml"


def _load_module():
    spec = importlib.util.spec_from_file_location("render_controller_config", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


rcc = _load_module()


def _ddc(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))["diff_drive_controller"]["ros__parameters"]


def test_repo_default_stays_open_loop():
    """Боевое значение не меняется этим PR: одометрия по командам."""
    assert _ddc(CONFIG)["open_loop"] is True


@pytest.mark.parametrize("raw", [None, "", "  "])
def test_unset_returns_source_without_copy(tmp_path, raw):
    dst = tmp_path / "out.yaml"
    assert rcc.render(CONFIG, dst, raw) == CONFIG
    assert not dst.exists()


@pytest.mark.parametrize(("raw", "expected"), [("false", False), ("FALSE", False), ("true", True), (" True ", True)])
def test_override_sets_only_open_loop(tmp_path, raw, expected):
    dst = tmp_path / "out.yaml"
    assert rcc.render(CONFIG, dst, raw) == dst

    src_cfg = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    out_cfg = yaml.safe_load(dst.read_text(encoding="utf-8"))
    assert out_cfg["diff_drive_controller"]["ros__parameters"]["open_loop"] is expected

    # Всё остальное (колея, радиус, TF, лимиты, другие контроллеры) — без изменений.
    src_cfg["diff_drive_controller"]["ros__parameters"]["open_loop"] = expected
    assert out_cfg == src_cfg


def test_closed_loop_reads_velocity_state_interface(tmp_path):
    """open_loop=false + position_feedback=false → diff_drive берёт state 'velocity' (из eRPM)."""
    dst = tmp_path / "out.yaml"
    params = _ddc(rcc.render(CONFIG, dst, "false"))
    assert params["open_loop"] is False
    assert params["position_feedback"] is False
    assert params["enable_odom_tf"] is True
    assert (params["odom_frame_id"], params["base_frame_id"]) == ("odom", "base_footprint")


@pytest.mark.parametrize("raw", ["0", "no", "closed", "flase"])
def test_bad_value_is_an_error(tmp_path, raw):
    with pytest.raises(rcc.OpenLoopValueError):
        rcc.render(CONFIG, tmp_path / "out.yaml", raw)


def test_missing_controller_section_is_an_error(tmp_path):
    src = tmp_path / "cm.yaml"
    src.write_text("controller_manager:\n  ros__parameters: {}\n", encoding="utf-8")
    with pytest.raises(KeyError):
        rcc.render(src, tmp_path / "out.yaml", "false")


def _run_cli(tmp_path, value):
    env = {k: v for k, v in os.environ.items() if k != rcc.ENV_VAR}
    if value is not None:
        env[rcc.ENV_VAR] = value
    return subprocess.run(
        [sys.executable, str(SCRIPT), str(CONFIG), str(tmp_path / "out.yaml")],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )


def test_cli_prints_path_for_shell(tmp_path):
    unset = _run_cli(tmp_path, None)
    assert unset.returncode == 0, unset.stderr
    assert Path(unset.stdout.strip()) == CONFIG

    closed = _run_cli(tmp_path, "false")
    assert closed.returncode == 0, closed.stderr
    assert _ddc(Path(closed.stdout.strip()))["open_loop"] is False


def test_cli_bad_value_fails_loudly(tmp_path):
    result = _run_cli(tmp_path, "maybe")
    assert result.returncode == 1
    assert rcc.ENV_VAR in result.stderr
    assert result.stdout == ""


def test_compose_passes_switch_with_empty_default():
    compose = yaml.safe_load((MAIN / "docker-compose.yaml").read_text(encoding="utf-8"))
    service = compose["services"]["ros2-control"]
    assert "DIFF_DRIVE_OPEN_LOOP=${DIFF_DRIVE_OPEN_LOOP:-}" in service["environment"]
    # Скрипт попадает в контейнер через этот mount.
    assert "./scripts/ros2_control:/scripts:ro" in service["volumes"]
