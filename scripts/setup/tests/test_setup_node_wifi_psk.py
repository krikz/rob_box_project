"""Пароль Wi-Fi в setup_node.sh (решение Шифу 09.11): не в репо, ввод без эха или переменная на один запуск.

setup_node.sh подключается с ROBBOX_SETUP_SOURCE_ONLY=1 (только функции), sudo и netplan подменены функциями,
netplan пишется во временный файл (ROBBOX_NETPLAN_FILE). Без сети, без root.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
SETUP = REPO / "scripts" / "setup" / "setup_node.sh"
pytestmark = pytest.mark.skipif(os.name != "posix" or shutil.which("bash") is None, reason="нужен bash и POSIX")

HARNESS = r"""
set -e
ROBBOX_SETUP_SOURCE_ONLY=1 source "$SETUP"
sudo() { "$@"; }
netplan() { echo "netplan $*" >> "$NETPLAN_LOG"; }
setup_sound_network
"""


def _run(tmp_path: Path, env_extra: dict[str, str], stdin=subprocess.DEVNULL) -> subprocess.CompletedProcess:
    env = {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "HOME": str(tmp_path),
        "SETUP": str(SETUP),
        "ROBBOX_NETPLAN_FILE": str(tmp_path / "01-netcfg.yaml"),
        "NETPLAN_LOG": str(tmp_path / "netplan.log"),
        **env_extra,
    }
    return subprocess.run(["bash", "-c", HARNESS], env=env, stdin=stdin, capture_output=True, text=True, timeout=60)


def test_no_password_in_repo_file():
    text = SETUP.read_text(encoding="utf-8")
    assert "1234567890" not in text
    assert not re.search(r'password:\s*"[^"$]', text), "пароль литералом в netplan-шаблоне"


def test_without_tty_and_env_is_honest_error(tmp_path):
    res = _run(tmp_path, {})
    assert res.returncode != 0
    out = res.stdout + res.stderr
    assert "ROBBOX_WIFI_PSK=" in out and "терминала нет" in out
    assert not (tmp_path / "01-netcfg.yaml").exists()


def test_env_password_goes_to_netplan_only(tmp_path):
    psk = 'te"st\\psk-4375'
    res = _run(tmp_path, {"ROBBOX_WIFI_PSK": psk, "ROBBOX_WIFI_SSID": "rbx-100001"})
    assert res.returncode == 0, res.stdout + res.stderr
    netplan = tmp_path / "01-netcfg.yaml"
    body = netplan.read_text(encoding="utf-8")
    assert '"rbx-100001":' in body
    assert 'password: "te\\"st\\\\psk-4375"' in body
    assert psk not in res.stdout + res.stderr, "пароль не печатается в лог"
    assert oct(netplan.stat().st_mode & 0o777) == "0o600"
    assert "netplan apply" in (tmp_path / "netplan.log").read_text()
    yaml = pytest.importorskip("yaml")
    parsed = yaml.safe_load(body)
    assert parsed["network"]["wifis"]["wlan0"]["access-points"]["rbx-100001"]["password"] == psk


def test_interactive_password_is_read_without_echo(tmp_path):
    """С терминалом (pty) пароль спрашивается и не отображается."""
    import pty
    import select
    import time

    master, slave = pty.openpty()
    env = {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "HOME": str(tmp_path),
        "SETUP": str(SETUP),
        "ROBBOX_NETPLAN_FILE": str(tmp_path / "01-netcfg.yaml"),
        "NETPLAN_LOG": str(tmp_path / "netplan.log"),
    }
    proc = subprocess.Popen(["bash", "-c", HARNESS], env=env, stdin=slave, stdout=slave, stderr=slave)
    os.close(slave)
    seen, sent, deadline = b"", False, time.time() + 30
    while time.time() < deadline:
        ready, _, _ = select.select([master], [], [], 0.5)
        if ready:
            try:
                chunk = os.read(master, 4096)
            except OSError:
                break
            if not chunk:
                break
            seen += chunk
        if not sent and "ввод не отображается".encode() in seen:
            os.write(master, b"pty-secret-4375\n")
            sent = True
        if proc.poll() is not None and not ready:
            break
    proc.wait(timeout=10)
    os.close(master)
    assert sent, seen.decode(errors="replace")
    assert proc.returncode == 0, seen.decode(errors="replace")
    assert b"pty-secret-4375" not in seen, "пароль не должен отображаться"
    assert 'password: "pty-secret-4375"' in (tmp_path / "01-netcfg.yaml").read_text(encoding="utf-8")


def test_existing_netplan_is_kept_without_asking(tmp_path):
    netplan = tmp_path / "01-netcfg.yaml"
    netplan.write_text("      dhcp4-overrides:\n        route-metric: 100\n")
    res = _run(tmp_path, {})
    assert res.returncode == 0, res.stdout + res.stderr
    assert "пропускаю" in res.stdout
