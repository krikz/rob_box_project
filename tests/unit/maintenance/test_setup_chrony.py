"""test_setup_chrony.py — контракт ``scripts/maintenance/setup_chrony.sh`` (ADR-0137 §2.1).

Без root и без изменения системы: проверяются синтаксис (bash -n, shellcheck),
генерация конфига (``--print-config``) и логика ``--check`` на поддельном
``chronyc`` (переменная CHRONYC), печатающем заранее заданный CSV.
Реальный chrony на Pi здесь НЕ проверяется — это ручной прогон (ADR-0137 §6).

Run:
  python -m pytest tests/unit/maintenance/test_setup_chrony.py -v -p no:cacheprovider --no-cov -o addopts=""
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "maintenance" / "setup_chrony.sh"
SYNC_TIME = REPO_ROOT / "scripts" / "maintenance" / "sync_time.sh"


def _run(args, env_extra=None):
    env = dict(os.environ)
    env.update(env_extra or {})
    return subprocess.run(
        ["bash", str(SCRIPT), *args], capture_output=True, text=True, timeout=30, env=env
    )


@pytest.mark.parametrize("script", [SCRIPT, SYNC_TIME], ids=lambda p: p.name)
def test_bash_syntax_clean(script):
    res = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True, timeout=10)
    assert res.returncode == 0, res.stderr


@pytest.mark.parametrize("script", [SCRIPT, SYNC_TIME], ids=lambda p: p.name)
def test_shellcheck_clean(script):
    shellcheck = shutil.which("shellcheck")
    if shellcheck is None:
        pytest.skip("shellcheck not installed locally")
    res = subprocess.run([shellcheck, str(script)], capture_output=True, text=True, timeout=30)
    assert res.returncode == 0, res.stdout + res.stderr


def test_sync_time_delegates_to_chrony_when_active():
    text = SYNC_TIME.read_text(encoding="utf-8")
    assert "systemctl is-active --quiet chrony" in text
    assert 'setup_chrony.sh" --check' in text


def test_print_config_vision_only_main_pi():
    res = _run(["--print-config", "--role", "vision"])
    assert res.returncode == 0, res.stderr
    lines = [ln for ln in res.stdout.splitlines() if ln and not ln.startswith("#")]
    assert "server 10.1.1.10 iburst prefer minpoll 2 maxpoll 4" in lines
    assert not any(ln.startswith("pool ") for ln in lines), "Vision Pi не должен ходить во внешний NTP"
    assert not any(ln.startswith("allow ") for ln in lines)
    assert "# role: vision" in res.stdout


def test_print_config_vision_custom_server():
    res = _run(["--print-config", "--role", "vision", "--server", "10.1.1.20"])
    assert "server 10.1.1.20 iburst prefer minpoll 2 maxpoll 4" in res.stdout


def test_print_config_main_serves_lan():
    res = _run(["--print-config", "--role", "main"])
    assert res.returncode == 0, res.stderr
    assert "allow 10.1.1.0/24" in res.stdout
    assert "local stratum 10" in res.stdout
    assert "pool ru.pool.ntp.org iburst maxsources 2" in res.stdout
    assert "\nserver " not in res.stdout


def test_bad_role_rejected():
    assert _run(["--print-config", "--role", "katana"]).returncode == 1
    assert _run(["--print-config"]).returncode == 1


# ── --check на поддельном chronyc ────────────────────────────────────────────

def _fake_chronyc(tmp_path: Path, sources_csv: str) -> str:
    fake = tmp_path / "fake_chronyc"
    fake.write_text(
        "#!/bin/bash\n"
        'case "$*" in\n'
        f"  *-c\\ sources*) printf '%s\\n' '{sources_csv}' ;;\n"
        "  *tracking*) echo 'Reference ID    : 0A01010A (10.1.1.10)' ;;\n"
        "  *sources*) echo '^* 10.1.1.10 (fake)' ;;\n"
        "esac\n",
        encoding="utf-8",
    )
    fake.chmod(0o755)
    return str(fake)


def _conf(tmp_path: Path, role: str, server: str = "10.1.1.10") -> str:
    res = _run(["--print-config", "--role", role, "--server", server])
    conf = tmp_path / f"chrony_{role}.conf"
    conf.write_text(res.stdout, encoding="utf-8")
    return str(conf)


@pytest.mark.parametrize(
    "sources_csv, expected_rc, expected_verdict",
    [
        ("^,*,10.1.1.10,10,2,377,1,0.000850,0.000900,0.000200", 0, "verdict=PASS"),
        ("^,*,10.1.1.10,10,2,377,1,-0.004900,-0.004900,0.000200", 0, "verdict=PASS"),
        ("^,*,10.1.1.10,10,2,377,1,0.008000,0.008000,0.000200", 2, "verdict=FAIL"),
        ("^,*,10.1.1.10,10,2,377,1,-0.012000,-0.012000,0.000200", 2, "verdict=FAIL"),
        ("^,*,91.206.16.3,2,6,377,1,0.000100,0.000100,0.010000", 2, "verdict=FAIL"),
        ("^,?,10.1.1.10,0,2,0,-,0,0,0", 2, "source=none"),
    ],
    ids=["0.85ms", "-4.9ms", "8ms", "-12ms", "external-source", "unreachable"],
)
def test_check_vision(tmp_path, sources_csv, expected_rc, expected_verdict):
    env = {"CHRONYC": _fake_chronyc(tmp_path, sources_csv), "CHRONY_CONF": _conf(tmp_path, "vision")}
    res = _run(["--check"], env)  # роль и сервер — из маркера конфига
    assert res.returncode == expected_rc, res.stdout + res.stderr
    assert expected_verdict in res.stdout
    assert "chrony_check role=vision" in res.stdout


def test_check_main_without_external_source_is_warn_not_fail(tmp_path):
    env = {
        "CHRONYC": _fake_chronyc(tmp_path, "^,?,91.206.16.3,0,6,0,-,0,0,0"),
        "CHRONY_CONF": _conf(tmp_path, "main"),
    }
    res = _run(["--check"], env)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "local stratum 10" in res.stdout + res.stderr
    assert "role=main source=none" in res.stdout


def test_check_chronyd_not_answering_is_exit_3(tmp_path):
    fake = tmp_path / "dead_chronyc"
    fake.write_text("#!/bin/bash\necho '506 Cannot talk to daemon'\nexit 1\n", encoding="utf-8")
    fake.chmod(0o755)
    res = _run(["--check", "--role", "vision"], {"CHRONYC": str(fake), "CHRONY_CONF": "/nonexistent"})
    assert res.returncode == 3


def test_apply_requires_root_when_not_root():
    if os.geteuid() == 0:
        pytest.skip("запущено от root — ветка 'нужны права root' недостижима")
    res = _run(["--role", "vision"])
    assert res.returncode == 1
    assert "root" in res.stderr
