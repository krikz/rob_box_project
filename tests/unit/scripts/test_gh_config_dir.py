"""Резолв GH_CONFIG_DIR для scripts/dev/*-via-gh-api.sh.

Сеть и ``gh auth token`` не вызываются: проверяется только выбор каталога
и то, что скрипты больше не падают на отсутствии /home/builder/.config/gh.
"""

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
RESOLVE = REPO / "scripts" / "dev" / "gh_config_dir.sh"
SCRIPTS = (
    REPO / "scripts" / "dev" / "gh-pr-create-via-gh-api.sh",
    REPO / "scripts" / "dev" / "push-via-gh-api.sh",
)


def _bash(script: str, env: dict) -> subprocess.CompletedProcess:
    # XDG раннера не должен протекать: тест задаёт его сам, если проверяет XDG.
    drop = {"GH_CONFIG_DIR", "XDG_CONFIG_HOME"}
    merged = {k: v for k, v in os.environ.items() if k not in drop}
    merged.update(env)
    return subprocess.run(
        ["bash", "-c", script],
        capture_output=True,
        text=True,
        env=merged,
        timeout=20,
        check=False,
    )


def test_scripts_pass_bash_n():
    for path in (RESOLVE, *SCRIPTS):
        subprocess.run(["bash", "-n", str(path)], check=True)


def test_explicit_dir_wins_even_if_missing(tmp_path):
    missing = tmp_path / "no-such-gh"
    got = _bash(
        f'source "{RESOLVE}"; gh_config_dir_resolve',
        {
            "GH_CONFIG_DIR": str(missing),
            "HOME": str(tmp_path),
            "XDG_CONFIG_HOME": str(tmp_path / "xdg"),
        },
    )
    assert got.returncode == 0, got.stderr
    assert got.stdout.strip() == str(missing)


def test_user_config_when_env_unset(tmp_path):
    home = tmp_path / "home"
    (home / ".config" / "gh").mkdir(parents=True)
    got = _bash(
        f'source "{RESOLVE}"; gh_config_dir_resolve',
        {"HOME": str(home)},
    )
    assert got.returncode == 0, got.stderr
    assert got.stdout.strip() == str(home / ".config" / "gh")


def test_xdg_without_gh_falls_through_to_home(tmp_path):
    """CI задаёт XDG_CONFIG_HOME без подкаталога gh — домашний конфиг всё равно находится."""
    home = tmp_path / "home"
    (home / ".config" / "gh").mkdir(parents=True)
    xdg = tmp_path / "xdg-without-gh"
    xdg.mkdir()
    got = _bash(
        f'source "{RESOLVE}"; gh_config_dir_resolve',
        {"HOME": str(home), "XDG_CONFIG_HOME": str(xdg)},
    )
    assert got.returncode == 0, got.stderr
    assert got.stdout.strip() == str(home / ".config" / "gh")


def test_xdg_config_home_beats_home(tmp_path):
    xdg = tmp_path / "xdg"
    (xdg / "gh").mkdir(parents=True)
    (tmp_path / "home" / ".config" / "gh").mkdir(parents=True)
    got = _bash(
        f'source "{RESOLVE}"; gh_config_dir_resolve',
        {"HOME": str(tmp_path / "home"), "XDG_CONFIG_HOME": str(xdg)},
    )
    assert got.returncode == 0, got.stderr
    assert got.stdout.strip() == str(xdg / "gh")


def test_empty_when_no_config_exists(tmp_path):
    """Нет конфига пользователя — пусто, кроме живого каталога билдера."""
    got = _bash(
        f'source "{RESOLVE}"; gh_config_dir_resolve',
        {"HOME": str(tmp_path / "nobody")},
    )
    assert got.returncode == 0, got.stderr
    builder = Path("/home/builder/.config/gh")
    expected = str(builder) if builder.is_dir() else ""
    assert got.stdout.strip() == expected


@pytest.mark.parametrize("script", SCRIPTS, ids=lambda p: p.name)
def test_scripts_do_not_die_on_missing_builder_dir(script, tmp_path):
    """Раньше скрипт падал FATAL до аргументов без каталога билдера."""
    got = _bash(f'bash "{script}"', {"HOME": str(tmp_path / "nobody")})
    assert "GH_CONFIG_DIR=/home/builder/.config/gh не существует" not in got.stderr
    assert got.returncode == 2  # usage: аргументов нет, до токена и сети не дошло
