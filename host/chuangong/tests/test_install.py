"""host/chuangong/install.sh в подменённом корне ФС: что ставит, идемпотентность, таймер без --enable не включается."""

from __future__ import annotations

import getpass
import os
import shutil
import subprocess
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
INSTALL = HERE / "install.sh"

pytestmark = pytest.mark.skipif(os.name != "posix" or shutil.which("bash") is None, reason="нужен bash и POSIX")


def _run(root: Path, *args: str) -> subprocess.CompletedProcess:
    env = dict(
        os.environ,
        CHUANGONG_ROOT=str(root),
        CHUANGONG_SKIP_APT="1",
        CHUANGONG_USER=getpass.getuser(),
        CHUANGONG_SYSTEMCTL="false",  # с подменённым корнем systemctl звать нельзя — false уронил бы установку
    )
    return subprocess.run(["bash", str(INSTALL), *args], capture_output=True, text=True, env=env, timeout=60)


def test_install_layout_and_observe_mode(tmp_path):
    res = _run(tmp_path, "--node", "vision", "--robot-id", "RBXU100001")
    assert res.returncode == 0, res.stdout + res.stderr
    assert (tmp_path / "opt/rob_box/chuangong/chuangong/agent.py").is_file()
    assert (tmp_path / "opt/rob_box/releases").is_dir()
    assert (tmp_path / "var/lib/chuangong").is_dir()
    env = (tmp_path / "etc/chuangong/chuangong.env").read_text()
    assert "CHUANGONG_NODE=vision" in env and "CHUANGONG_MODE=observe" in env and "CHUANGONG_ROBOT_ID=RBXU100001" in env
    for unit in ("chuangong.service", "chuangong.timer"):
        assert (tmp_path / "etc/systemd/system" / unit).read_bytes() == (HERE / "systemd" / unit).read_bytes()
    assert os.access(tmp_path / "usr/local/bin/chuangong", os.X_OK)
    assert "таймер НЕ включён" in res.stdout
    assert "no_pubkey" in res.stdout


def test_second_run_keeps_host_config_and_refreshes_code(tmp_path):
    assert _run(tmp_path, "--node", "sound").returncode == 0
    envf = tmp_path / "etc/chuangong/chuangong.env"
    envf.write_text(envf.read_text().replace("CHUANGONG_RESERVE_GB=2", "CHUANGONG_RESERVE_GB=5"))
    stale = tmp_path / "opt/rob_box/chuangong/chuangong/old_module.py"
    stale.write_text("# лишний файл прошлой версии\n")

    res = _run(tmp_path, "--node", "sound")
    assert res.returncode == 0, res.stderr
    assert "CHUANGONG_RESERVE_GB=5" in envf.read_text(), "правка конфига на хосте не затирается"
    assert not stale.exists(), "код агента заменяется целиком"
    assert "юниты обновлены" not in res.stdout, "неизменные юниты не переставляются"


def test_pubkey_and_enable_flag(tmp_path):
    key = tmp_path / "k.pub"
    key.write_text("untrusted comment: minisign public key X\nRWQ=\n")
    root = tmp_path / "root"
    res = _run(root, "--node", "main", "--enable", "--pubkey", str(key))
    assert res.returncode == 0, res.stderr
    assert (root / "etc/chuangong/release.pub").read_text() == key.read_text()
    assert "таймер chuangong.timer включён" in res.stdout


@pytest.mark.parametrize("args", [["--node", "kitchen"], [], ["--node", "main", "--robot-id", "bad id"]])
def test_bad_arguments(tmp_path, args):
    assert _run(tmp_path, *args).returncode == 2
