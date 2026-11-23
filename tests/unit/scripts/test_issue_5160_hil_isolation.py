"""#5160: цель move_ahead не уходит, пока предохранитель не доказал изоляцию.

На develop ``sc_move_ahead`` звал ``inject`` сразу после ``probe watch``: боевой
``ros2-control`` и командные кадры VESC никто не спрашивал. Предохранитель
(``scripts/hil/checks/isolation.py``) решает до цели. Нет факта — отказ, не PASS.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
CHECKS = ROOT / "scripts" / "hil" / "checks"
_spec = importlib.util.spec_from_file_location("hil_isolation", CHECKS / "isolation.py")
iso = importlib.util.module_from_spec(_spec)
sys.modules["hil_isolation"] = iso
_spec.loader.exec_module(iso)

_STATUS_SAFE = """\
── Main Pi
ros2-control exited
nav2 running
── /controller_manager в графе (сторож в gazebo): 1 (норма HIL — 1)
"""
_STATUS_COMBAT = """\
── Main Pi
ros2-control running
── /controller_manager в графе (сторож в gazebo): 1 (норма HIL — 1)
"""
_STATUS_TWO_CM = """\
── Main Pi
ros2-control exited
── /controller_manager в графе (сторож в gazebo): 2 (норма HIL — 1)
"""


_INSPECT_SAFE = "/ros2-control exited\n/rob-box-hil-gazebo running\n"
_INSPECT_COMBAT = "/ros2-control running\n/rob-box-hil-gazebo running\n"
_INSPECT_NO_GZ = "/ros2-control exited\n/rob-box-hil-gazebo exited\n"
_PREFIX = "rob-box-hil"


def _raw(
    tmp_path: Path,
    status: str,
    commands: int,
    inject: str = "",
    probe: str = "",
    inspect: str = _INSPECT_SAFE,
) -> Path:
    (tmp_path / "hil_status.txt").write_text(status, encoding="utf-8")
    (tmp_path / "inspect.txt").write_text(inspect, encoding="utf-8")
    (tmp_path / "can_watch.json").write_text(
        json.dumps({"commands": commands, "frames": 12}), encoding="utf-8"
    )
    (tmp_path / "inject.txt").write_text(inject, encoding="utf-8")
    (tmp_path / "probe.log").write_text(probe, encoding="utf-8")
    return tmp_path


def _judge(directory: Path):
    return iso.judge(directory, _PREFIX)


@pytest.mark.unit
def test_isolation_passes_only_gazebo_stopped_control_zero_vesc(tmp_path):
    verdict = _judge(_raw(tmp_path, _STATUS_SAFE, 0))
    assert verdict.ok
    assert verdict.line().startswith("ISOLATION PASS")
    assert "командных кадров VESC: 0" in verdict.line()
    assert "ros2-control: exited" in verdict.line()


@pytest.mark.unit
def test_isolation_refuses_when_combat_ros2_control_running(tmp_path):
    verdict = _judge(_raw(tmp_path, _STATUS_COMBAT, 0, inspect=_INSPECT_COMBAT))
    assert not verdict.ok
    assert "ros2-control" in verdict.line()


@pytest.mark.unit
def test_isolation_refuses_when_controller_manager_is_not_one(tmp_path):
    verdict = _judge(_raw(tmp_path, _STATUS_TWO_CM, 0))
    assert not verdict.ok
    assert "/controller_manager" in verdict.line()


@pytest.mark.unit
def test_isolation_refuses_when_vesc_command_frame_seen(tmp_path):
    verdict = _judge(_raw(tmp_path, _STATUS_SAFE, 3))
    assert not verdict.ok
    assert "командных кадров VESC: 3" in verdict.line()


@pytest.mark.unit
def test_isolation_refuses_without_can_raw(tmp_path):
    _raw(tmp_path, _STATUS_SAFE, 0)
    (tmp_path / "can_watch.json").write_text("", encoding="utf-8")
    verdict = _judge(tmp_path)
    assert not verdict.ok
    assert "нет в сырье" in verdict.line()


@pytest.mark.unit
def test_isolation_refuses_when_goal_already_sent(tmp_path):
    spoken = _judge(_raw(tmp_path, _STATUS_SAFE, 0, inject="Робот, вперёд на метр.\n"))
    probed = _judge(_raw(tmp_path, _STATUS_SAFE, 0, probe="NAV_SENT goal=abc\n"))
    assert not spoken.ok and not probed.ok


@pytest.mark.unit
def test_isolation_refuses_when_gazebo_container_down(tmp_path):
    verdict = _judge(_raw(tmp_path, _STATUS_SAFE, 0, inspect=_INSPECT_NO_GZ))
    assert not verdict.ok
    assert "gazebo" in verdict.line()


@pytest.mark.unit
def test_cli_refuses_with_exit_2(tmp_path):
    raw = _raw(tmp_path, _STATUS_COMBAT, 1, inspect=_INSPECT_COMBAT)
    assert iso.main([str(raw), "--gazebo-prefix", _PREFIX]) == iso.REFUSED
    assert iso.REFUSED == 2


@pytest.mark.unit
def test_move_ahead_asks_isolation_before_inject():
    """На develop inject шёл сразу после probe watch — этот тест там падает."""
    text = (CHECKS / "run_checks.sh").read_text(encoding="utf-8")
    body = text.split("sc_move_ahead()", 1)[1].split("\nsc_", 1)[0]
    assert "isolation.py" in text
    calls = [line for line in body.splitlines() if not line.strip().startswith("#")]
    code = "\n".join(calls)
    assert code.index("isolation_ok") < code.index("inject ")
    assert "exit 2" in text.split("isolation_ok()", 1)[1].split("sc_move_ahead", 1)[0]
