"""#4587: блок <hardware> VescSystem не несёт параметров, которые плагин игнорирует.

``vesc_nexus/VescSystemHardwareInterface::on_init`` зовёт
``warnIgnored(hw, scope, {"publish_rate", "wheel_radius"})`` (vesc_nexus@38e5995,
``vesc_system_hardware_interface.cpp:84``) и пишет WARN на каждом старте.
``wheel_radius`` живёт только в ``<joint>``, ``publish_rate`` — в yaml диффпривода.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
FILES = [PKG / "urdf" / "rob_box_ros2_control.xacro", PKG / "test" / "golden" / "rob_box_real.urdf"]
IGNORED_BY_PLUGIN = {"publish_rate", "wheel_radius"}


def _vesc_hardware_params(path: Path) -> set[str]:
    root = ET.parse(path).getroot()
    for rc in root.iter("ros2_control"):
        if rc.get("name") == "VescSystem":
            hw = rc.find("hardware")
            assert hw is not None
            return {p.get("name") for p in hw.findall("param")}
    raise AssertionError(f"VescSystem не найден в {path}")


@pytest.mark.parametrize("path", FILES, ids=lambda p: p.name)
def test_hardware_block_has_no_ignored_params(path):
    assert _vesc_hardware_params(path) & IGNORED_BY_PLUGIN == set()


@pytest.mark.parametrize("path", FILES, ids=lambda p: p.name)
def test_hardware_block_keeps_can_interface(path):
    assert "can_interface" in _vesc_hardware_params(path)


# #4639: на уровне <joint> плагин игнорирует min_erpm и wheel_radius
# (vesc_system_hardware_interface.cpp:132). Радиус колеса живёт в yaml диффпривода.
IGNORED_BY_PLUGIN_JOINT = {"min_erpm", "wheel_radius"}


def _vesc_joint_params(path: Path) -> list[set[str]]:
    root = ET.parse(path).getroot()
    for rc in root.iter("ros2_control"):
        if rc.get("name") == "VescSystem":
            return [{p.get("name") for p in j.findall("param")} for j in rc.findall("joint")]
    raise AssertionError(f"VescSystem не найден в {path}")


@pytest.mark.parametrize("path", FILES, ids=lambda p: p.name)
def test_joints_have_no_ignored_params(path):
    joints = _vesc_joint_params(path)
    assert len(joints) == 4
    for params in joints:
        assert params & IGNORED_BY_PLUGIN_JOINT == set()
        assert {"can_id", "poles", "max_rps"} <= params
