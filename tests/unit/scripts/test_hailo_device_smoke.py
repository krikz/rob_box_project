"""Регресс-тесты для docker/vision/scripts/vision-hailo/hailo_device_smoke.sh.

Issue #3090, деплой-лог run 36687135688 (08:09:55):
``[start_vision_hailo] WARN: hailortcli не установлен, но HAILO_ENABLED=true``.

Образ vision-hailo ставит только wheel hailo_platform + libhailort.so
(ADR-0099 §2.2), hailortcli в нём нет. Прежняя проверка в
start_vision_hailo.sh поэтому всегда уходила в WARN и устройство не
проверяла. Теперь smoke сканирует через hailo_platform.Device.scan(),
если hailortcli нет.

Тесты не зависят от машины: PATH содержит только fake-bin + системные
утилиты, ``hailo_platform`` подменяется fake-модулем через PYTHONPATH.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
HAILO_SCRIPTS = REPO_ROOT / "docker" / "vision" / "scripts" / "vision-hailo"
SMOKE = HAILO_SCRIPTS / "hailo_device_smoke.sh"
START = HAILO_SCRIPTS / "start_vision_hailo.sh"

OLD_WARN = "hailortcli не установлен, но HAILO_ENABLED=true"


def _fake_hailo_platform(root: Path, scan_body: str) -> Path:
    pkg = root / "pylib" / "hailo_platform"
    pkg.mkdir(parents=True)
    (pkg / "__init__.py").write_text(
        "class Device:\n"
        "    @staticmethod\n"
        "    def scan():\n"
        f"        {scan_body}\n",
        encoding="utf-8",
    )
    return pkg.parent


def _run(tmp_path: Path, *, hailortcli: str | None = None,
         pythonpath: Path | None = None) -> subprocess.CompletedProcess[str]:
    fake_bin = tmp_path / "fakebin"
    fake_bin.mkdir(exist_ok=True)
    if hailortcli is not None:
        cli = fake_bin / "hailortcli"
        cli.write_text(f"#!/bin/sh\n{hailortcli}\n", encoding="utf-8")
        cli.chmod(0o755)
    env = {
        # /usr/bin:/bin для bash/dirname; реального hailortcli на CI нет,
        # и мы это проверяем ниже, чтобы тест не зависел от хоста.
        "PATH": f"{fake_bin}:/usr/bin:/bin",
        "HAILO_SMOKE_PYTHON": sys.executable,
        "HOME": str(tmp_path),
    }
    if pythonpath is not None:
        env["PYTHONPATH"] = str(pythonpath)
    return subprocess.run(
        ["bash", str(SMOKE)], capture_output=True, text=True, env=env,
        timeout=30,
    )


@pytest.fixture(autouse=True)
def _no_host_hailortcli() -> None:
    for d in ("/usr/bin", "/bin"):
        if Path(d, "hailortcli").exists():
            pytest.skip("host has a real hailortcli in /usr/bin or /bin")


def test_no_cli_binding_sees_device_exit0(tmp_path: Path) -> None:
    pp = _fake_hailo_platform(tmp_path, "return ['0000:01:00.0']")
    proc = _run(tmp_path, pythonpath=pp)
    assert proc.returncode == 0, proc.stderr
    assert "Hailo device(s) visible via hailo_platform: 0000:01:00.0" in proc.stdout
    assert OLD_WARN not in proc.stderr


def test_no_cli_binding_sees_nothing_exit1(tmp_path: Path) -> None:
    """Драйвер пропал после ребута (#3090) → контейнер не стартует молча."""
    pp = _fake_hailo_platform(tmp_path, "return []")
    proc = _run(tmp_path, pythonpath=pp)
    assert proc.returncode == 1
    assert "Device.scan() не нашёл устройств" in proc.stderr


def test_no_cli_scan_raises_is_warn_not_fatal(tmp_path: Path) -> None:
    pp = _fake_hailo_platform(tmp_path, "raise RuntimeError('api drift')")
    proc = _run(tmp_path, pythonpath=pp)
    assert proc.returncode == 0
    assert "WARN" in proc.stderr and "RuntimeError: api drift" in proc.stderr


def test_no_cli_no_binding_is_warn(tmp_path: Path) -> None:
    empty = tmp_path / "emptylib"
    empty.mkdir()
    proc = _run(tmp_path, pythonpath=empty)
    assert proc.returncode == 0
    assert "нет ни hailortcli, ни hailo_platform" in proc.stderr


def test_cli_present_scan_ok_uses_cli(tmp_path: Path) -> None:
    proc = _run(tmp_path, hailortcli="echo 'Hailo Devices: [-] Device: 0000:01:00.0'")
    assert proc.returncode == 0, proc.stderr
    assert "running hailortcli scan" in proc.stdout
    assert "Device.scan()" not in proc.stdout


def test_cli_present_scan_fails_exit1(tmp_path: Path) -> None:
    proc = _run(tmp_path, hailortcli="echo 'Hailo devices not found' >&2; exit 1")
    assert proc.returncode == 1
    assert "hailortcli scan failed" in proc.stderr


def test_start_script_delegates_to_smoke() -> None:
    text = START.read_text(encoding="utf-8")
    assert OLD_WARN not in text
    assert 'hailo_device_smoke.sh" || exit 1' in text


@pytest.mark.skipif(shutil.which("shellcheck") is None, reason="shellcheck not installed")
def test_shellcheck_clean() -> None:
    proc = subprocess.run(
        ["shellcheck", "-S", "warning", str(SMOKE)], capture_output=True, text=True,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr


def test_smoke_is_executable() -> None:
    assert os.access(SMOKE, os.X_OK)
