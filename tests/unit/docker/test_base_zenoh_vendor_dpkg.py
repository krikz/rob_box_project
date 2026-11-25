"""Вендорные zenoh .deb ставятся через dpkg, а не apt (#5245).

`apt-get install` локального .deb, имя которого есть в ROS APT, качает пакет
с CDN и падает на Hash Sum mismatch. Ветка RMW_ZENOH_VENDOR=true обязана
ставить файлы из /tmp/rmw-zenoh-vendor через `dpkg -i`.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
DOCKERFILES = (
    REPO / "docker" / "base" / "Dockerfile.depthai",
    REPO / "docker" / "base" / "Dockerfile.ros2-zenoh",
    REPO / "docker" / "base" / "Dockerfile.rtabmap",
)

_VENDOR_DPKG = re.compile(r'dpkg -i "\$vnd_deb" "\$rmw_deb"')
_APT_PATH = re.compile(r'apt-get install[^\n]*"/tmp/rmw-zenoh-vendor/')
_FALSE_PIN = "ros-humble-rmw-zenoh-cpp=0.1.11-1jammy.*"


def _true_branch(text: str) -> str:
    match = re.search(r"true\)(?P<body>.*?)false\)", text, re.DOTALL)
    assert match, "ветка true) ... false) не найдена"
    return match.group("body")


def assert_vendor_debs_via_dpkg(text: str) -> None:
    """Ветка true ставит .deb через dpkg, не аргументами apt-get install."""
    true = _true_branch(text)
    assert _VENDOR_DPKG.search(true), true
    assert true.index("zenoh-cpp-vendor_") < true.index("rmw-zenoh-cpp_"), true
    assert "apt-get install -y -f" in true
    assert not _APT_PATH.search(true), true
    assert 'apt-get install -y "$@"' not in true
    assert "rm -rf /tmp/rmw-zenoh-vendor /var/lib/apt/lists/*" in text
    assert "set -eux" in text


def _false_branch(text: str) -> str:
    match = re.search(r"false\)(?P<body>.*?)\*\)", text, re.DOTALL)
    assert match, "ветка false) ... *) не найдена"
    return match.group("body")


@pytest.mark.parametrize("dockerfile", DOCKERFILES, ids=lambda path: path.name)
def test_true_branch_installs_vendor_debs_with_dpkg(dockerfile: Path) -> None:
    assert_vendor_debs_via_dpkg(dockerfile.read_text(encoding="utf-8"))


@pytest.mark.parametrize("dockerfile", DOCKERFILES, ids=lambda path: path.name)
def test_false_branch_keeps_ros_apt_pins(dockerfile: Path) -> None:
    false = _false_branch(dockerfile.read_text(encoding="utf-8"))
    assert _FALSE_PIN in false
    assert "ros-humble-zenoh-cpp-vendor=0.1.11-1jammy.*" in false
    assert "dpkg -i" not in false
    assert "apt-get install -y" in false


def test_develop_text_fails_the_guard() -> None:
    """Текст origin/develop (apt-get install путей) гард не пропускает."""
    import subprocess

    old = subprocess.check_output(
        ["git", "show", "origin/develop:docker/base/Dockerfile.depthai"],
        cwd=REPO,
        text=True,
    )
    with pytest.raises(AssertionError):
        assert_vendor_debs_via_dpkg(old)
