"""setup_chuangong в setup_node.sh (решение Шифу): chuangong внедряется штатно и постоянно.

Оператор передаёт только тип узла main|vision|sound — без ROBBOX_ENABLE_CHUANGONG и
CHUANGONG_PUBKEY_FILE: install.sh всегда вызывается с --enable и ключом
host/chuangong/keys/release.pub из репо, success-log не ветвится на старых env.

setup_node.sh подключается с ROBBOX_SETUP_SOURCE_ONLY=1 (только функции), sudo подменён
функцией-заглушкой, которая пишет аргументы в лог. Без root, без systemd.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
SETUP = REPO / "scripts" / "setup" / "setup_node.sh"
pytestmark = pytest.mark.skipif(
    os.name != "posix" or shutil.which("bash") is None, reason="нужен bash и POSIX"
)

HARNESS = r"""
set -e
ROBBOX_SETUP_SOURCE_ONLY=1 source "$SETUP"
# Как настоящий sudo: KEY=value префикс — это env, а не аргументы инсталлятора.
sudo() {
    local out=() a
    for a in "$@"; do
        case "$a" in *=*) ;; *) out+=("$a") ;; esac
    done
    printf '%s\n' "${out[*]}" >> "$SUDO_LOG"
}
NODE_TYPE="$NODE_TYPE_IN"
DOCKER_COMPOSE_DIR="$DC_DIR"
setup_chuangong
"""


def _run(
    tmp_path: Path,
    node_type: str,
    env_extra: dict[str, str] | None = None,
    with_installer: bool = True,
) -> subprocess.CompletedProcess:
    repo_home = tmp_path / "home"
    installer = repo_home / "rob_box_project" / "host" / "chuangong" / "install.sh"
    if with_installer:
        installer.parent.mkdir(parents=True, exist_ok=True)
        installer.write_text(
            "#!/usr/bin/env bash\n# fake installer\n", encoding="utf-8"
        )
    dc_dir = tmp_path / "compose"
    dc_dir.mkdir()
    (dc_dir / ".env").write_text("ROBOT_ID=RBXU100042\n", encoding="utf-8")
    env = {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "HOME": str(repo_home),
        "SETUP": str(SETUP),
        "NODE_TYPE_IN": node_type,
        "DC_DIR": str(dc_dir),
        "SUDO_LOG": str(tmp_path / "sudo.log"),
        **(env_extra or {}),
    }
    return subprocess.run(
        ["bash", "-c", HARNESS],
        env=env,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=60,
    )


@pytest.mark.parametrize("node_type", ["main", "vision", "sound"])
def test_enable_and_repo_pubkey_always_passed(tmp_path, node_type):
    """Без всяких env install.sh получает --enable и --pubkey из репо (постоянное поведение)."""
    res = _run(tmp_path, node_type)
    assert res.returncode == 0, res.stdout + res.stderr
    installer = f"{tmp_path}/home/rob_box_project/host/chuangong/install.sh"
    pubkey = f"{tmp_path}/home/rob_box_project/host/chuangong/keys/release.pub"
    expected = f"bash {installer} --node {node_type} --robot-id RBXU100042 --enable --pubkey {pubkey}"
    assert (tmp_path / "sudo.log").read_text(encoding="utf-8").strip() == expected


def test_old_env_does_not_change_behavior(tmp_path):
    """Старые env не читаются: значение не влияет ни на аргументы, ни на success-log."""
    env = {"ROBBOX_ENABLE_CHUANGONG": "0", "CHUANGONG_PUBKEY_FILE": "/tmp/чужой.pub"}
    res = _run(tmp_path, "main", env)
    assert res.returncode == 0, res.stdout + res.stderr
    pubkey = f"{tmp_path}/home/rob_box_project/host/chuangong/keys/release.pub"
    args = (tmp_path / "sudo.log").read_text(encoding="utf-8")
    assert " --enable " in args
    assert f"--pubkey {pubkey}" in args
    assert "чужой.pub" not in args
    out = res.stdout + res.stderr
    assert "не включён" not in out, "success-log не должен ветвиться на старом env"
    assert "таймер: включён" in out


def test_other_node_type_skips_installer(tmp_path):
    res = _run(tmp_path, "sensor-hub")
    assert res.returncode == 0, res.stdout + res.stderr
    assert not (
        tmp_path / "sudo.log"
    ).exists(), "install.sh не вызывается для чужих типов узлов"


def test_missing_installer_is_warning_not_failure(tmp_path):
    res = _run(tmp_path, "main", with_installer=False)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "не найден" in res.stdout + res.stderr
    assert not (tmp_path / "sudo.log").exists()
