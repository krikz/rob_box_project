"""Юнит-тесты для scripts/setup/ensure_hailo_driver.sh (issue #3090).

Root cause (docs/reports/HAILO_DKMS_ROOT_CAUSE_2026-09-27.md): unattended-upgrades
ставит новый kernel, а metapackage ``linux-headers-raspi`` не установлен →
DKMS не может пересобрать ``hailo_pci.ko`` → после ребута ``/dev/hailo0`` нет.

Тесты герметичны: ``apt-get`` / ``dpkg-query`` / ``lsmod`` / ``uname`` —
shell-фейки в PATH, устройство — файл в tmp_path (``HAILO_DEVICE``). Ни
root, ни apt, ни Hailo на машине не нужны. Покрывается только ветка
«устройство и модуль уже есть» (боевой путь деплоя в штатном состоянии) —
DKMS-recovery ветка здесь НЕ исполняется.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "setup" / "ensure_hailo_driver.sh"


@pytest.fixture()
def env(tmp_path: Path) -> dict:
    fake_bin = tmp_path / "fakebin"
    fake_bin.mkdir()
    apt_log = tmp_path / "apt.log"
    installed = tmp_path / "installed"  # один пакет на строку
    installed.write_text("")

    def w(name: str, body: str) -> None:
        p = fake_bin / name
        p.write_text("#!/bin/bash\n" + body)
        p.chmod(0o755)

    w("uname", "echo 6.8.0-1065-raspi\n")
    w("lsmod", "printf 'Module Size Used\\nhailo_pci 126976 6\\n'\n")
    # Страховка: recovery-ветка НИКОГДА не должна дойти до реальных
    # dkms/depmod/modprobe хоста.
    for tool in ("dkms", "depmod", "modprobe", "udevadm", "lspci", "dpkg"):
        w(tool, "exit 1\n")
    # dpkg-query -W -f='${Status}' <pkg>
    w(
        "dpkg-query",
        'pkg="${!#}"\n'
        'if grep -qx "$pkg" "$FAKE_INSTALLED"; then printf "install ok installed"; exit 0; fi\n'
        "exit 1\n",
    )
    # apt-get: логируем вызов; install успешен, если FAKE_APT_FAIL не задан.
    w(
        "apt-get",
        'echo "apt-get $*" >> "$FAKE_APT_LOG"\n'
        'if [ "$1" = install ] && [ -z "${FAKE_APT_FAIL:-}" ]; then\n'
        '  echo "${!#}" >> "$FAKE_INSTALLED"\n'
        "fi\n"
        'if [ -n "${FAKE_APT_FAIL:-}" ] && [ "$1" = install ]; then exit 100; fi\n'
        "exit 0\n",
    )

    device = tmp_path / "hailo0"
    device.touch()

    e = os.environ.copy()
    e.update(
        PATH=f"{fake_bin}:/usr/bin:/bin",
        HAILO_DEVICE=str(device),
        FAKE_APT_LOG=str(apt_log),
        FAKE_INSTALLED=str(installed),
    )
    e.pop("FAKE_APT_FAIL", None)
    return {"env": e, "apt_log": apt_log, "installed": installed, "device": device}


def run(env: dict) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", str(SCRIPT)],
        capture_output=True,
        text=True,
        env=env["env"],
        timeout=20,
    )


def apt_calls(env: dict) -> list[str]:
    p = env["apt_log"]
    return p.read_text().splitlines() if p.exists() else []


def test_script_syntax() -> None:
    res = subprocess.run(["bash", "-n", str(SCRIPT)], capture_output=True, text=True)
    assert res.returncode == 0, res.stderr


def test_installs_headers_metapackage_when_missing_on_raspi(env: dict) -> None:
    """raspi kernel, headers-мета нет → ставим linux-headers-raspi (#3090)."""
    env["installed"].write_text("linux-image-raspi\n")
    res = run(env)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "apt-get install -y linux-headers-raspi" in apt_calls(env)
    assert "installed linux-headers-raspi" in res.stdout
    assert "already ready" in res.stdout


def test_headers_metapackage_step_is_idempotent(env: dict) -> None:
    """Повторный запуск: мета уже стоит → apt-get не вызывается вовсе."""
    env["installed"].write_text("linux-image-raspi\n")
    first = run(env)
    assert first.returncode == 0, first.stdout + first.stderr
    env["apt_log"].unlink()
    second = run(env)
    assert second.returncode == 0, second.stdout + second.stderr
    assert apt_calls(env) == []
    assert "linux-headers-raspi is installed" in second.stdout


def test_skips_headers_metapackage_on_other_kernel_flavour(env: dict) -> None:
    """Нет linux-image-raspi (не Pi / другой flavour) → мету не трогаем."""
    res = run(env)
    assert res.returncode == 0, res.stdout + res.stderr
    assert apt_calls(env) == []
    assert "skipping linux-headers-raspi" in res.stdout


def test_headers_install_failure_is_warning_not_fatal(env: dict) -> None:
    """apt упал (сеть / lock) → WARNING, но рабочий драйвер не валит деплой."""
    env["installed"].write_text("linux-image-raspi\n")
    env["env"]["FAKE_APT_FAIL"] = "1"
    res = run(env)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "WARNING: failed to install linux-headers-raspi" in res.stdout
    # install → update → повторный install
    assert apt_calls(env) == [
        "apt-get install -y linux-headers-raspi",
        "apt-get update -qq",
        "apt-get install -y linux-headers-raspi",
    ]


def test_device_path_is_injectable(env: dict) -> None:
    """HAILO_DEVICE не существует → скрипт уходит в recovery (не 'already ready')."""
    env["device"].unlink()
    res = run(env)
    assert "already ready" not in res.stdout
    assert "attempting DKMS recovery" in res.stdout
    # Фейковый dkms падает → скрипт честно выходит с ERROR, реальный dkms не зовётся.
    assert res.returncode == 1
    assert "ERROR: dkms autoinstall failed" in res.stdout


def test_default_device_is_dev_hailo0() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert 'HAILO_DEVICE="${HAILO_DEVICE:-/dev/hailo0}"' in text
    assert 'HEADERS_META="${HAILO_HEADERS_META:-linux-headers-raspi}"' in text
