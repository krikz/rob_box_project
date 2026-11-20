"""Код выхода обёртки маршрута HIL (scripts/hil/video/runvideo.sh) без стенда.

Раньше ``docker exec route.py | grep`` отдавал код grep (0), хотя маршрут
падал до первой цели. Координатор смотрит код обёртки: нет GOAL_START —
``ERROR before GOAL_START`` и ненулевой код.

docker подменён фейком в PATH. Настоящий контейнер и ROS не нужны.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
RUNVIDEO = ROOT / "scripts" / "hil" / "video" / "runvideo.sh"

FAKE_DOCKER = r"""#!/usr/bin/env bash
# Фейк docker: пишет вызов и на exec python3 /route.py печатает сценарий из FAKE_ROUTE_*.
echo "docker $*" >> "${FAKE_DOCKER_LOG:?}"
case "$1" in
    cp) exit 0 ;;
    exec)
        joined="$*"
        case "$joined" in
            *"/route.py"*)
                printf '%s' "${FAKE_ROUTE_STDOUT-}"
                if [ -n "${FAKE_ROUTE_STDERR-}" ]; then
                    printf '%s' "$FAKE_ROUTE_STDERR" >&2
                fi
                exit "${FAKE_ROUTE_RC:-0}"
                ;;
        esac
        exit 0
        ;;
esac
exit 0
"""


def _bash_works() -> bool:
    if shutil.which("bash") is None:
        return False
    try:
        return subprocess.run(["bash", "-c", "true"], capture_output=True, timeout=20).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


pytestmark = pytest.mark.skipif(not _bash_works(), reason="нет рабочего bash")


def _run(
    tmp_path: Path, stdout: str, rc: int, stderr: str = "", route_log: Path | None = None
) -> subprocess.CompletedProcess[str]:
    video = tmp_path / "video"
    (video / "out").mkdir(parents=True)
    (video / "route.py").write_text("# stub\n", encoding="utf-8")
    bindir = tmp_path / "bin"
    bindir.mkdir()
    docker = bindir / "docker"
    docker.write_text(FAKE_DOCKER, encoding="utf-8")
    docker.chmod(0o755)
    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{bindir}{os.pathsep}{env.get('PATH', '')}",
            "VIDEO_DIR": str(video),
            "CAMERAS": "0",
            "SETTLE_S": "0",
            "STOP_S": "0",
            "FAKE_DOCKER_LOG": str(tmp_path / "docker.log"),
            "FAKE_ROUTE_STDOUT": stdout,
            "FAKE_ROUTE_STDERR": stderr,
            "FAKE_ROUTE_RC": str(rc),
        }
    )
    if route_log is not None:
        env["ROUTE_LOG"] = str(route_log)
    return subprocess.run(
        ["bash", str(RUNVIDEO), "M1"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        env=env,
        timeout=30,
        cwd=tmp_path,
    )


def _log(tmp_path: Path) -> str:
    return (tmp_path / "video" / "out" / "route.log").read_text(encoding="utf-8")


def test_success_with_goal_start_exits_zero(tmp_path: Path) -> None:
    """route.py дошёл до первой цели и вышел 0 — обёртка тоже 0, ошибки нет."""
    proc = _run(tmp_path, "12:00:00.000 GOAL_START M1 target=(1,2,0) sim=1.00 frame=-1\n", 0)
    assert proc.returncode == 0, proc.stderr
    assert "ERROR before GOAL_START" not in proc.stderr
    assert "GOAL_START M1" in _log(tmp_path)
    assert proc.stdout.strip().splitlines()[-1] == "done"


def test_error_before_goal_start_is_visible(tmp_path: Path) -> None:
    """Ошибка в логе до GOAL_START: координатор видит строку и ненулевой код route.py."""
    proc = _run(
        tmp_path,
        "AssertionError: нет /navigate_to_pose\n",
        1,
    )
    assert proc.returncode == 1, proc.stderr
    assert "ERROR before GOAL_START:" in proc.stderr
    assert "нет /navigate_to_pose" in proc.stderr
    assert "GOAL_START" not in _log(tmp_path)


def test_route_crash_exit_code_is_not_swallowed(tmp_path: Path) -> None:
    """Падение route.py (rc=2, traceback, zenoh-шум): код обёртки = код route.py, не 0 от grep."""
    proc = _run(
        tmp_path,
        "zenoh noise line\n\nTraceback (most recent call last):\n"
        '  File "/route.py", line 1\nRuntimeError: rclpy.init failed\n',
        2,
    )
    assert proc.returncode == 2, proc.stderr
    assert "ERROR before GOAL_START:" in proc.stderr
    assert "RuntimeError: rclpy.init failed" in proc.stderr
    log = _log(tmp_path)
    assert "zenoh" not in log.lower()
    assert "RuntimeError" in log
    assert "GOAL_START" not in log


def test_empty_goals_before_goal_start_exits_nonzero(tmp_path: Path) -> None:
    """Пустой список целей: route.py вышел 0, но GOAL_START нет — обёртка не врёт успехом."""
    proc = _run(tmp_path, "", 0)
    assert proc.returncode == 1, proc.stderr
    assert "ERROR before GOAL_START:" in proc.stderr
    assert "rc=0" in proc.stderr
    assert _log(tmp_path) == ""


def test_route_log_gets_the_same_goal_start_check(tmp_path: Path) -> None:
    """ROUTE_LOG (файл run_id) и общий out/route.log — один лог; сбой до цели виден и там."""
    route_log = tmp_path / "run_id" / "route.log"
    proc = _run(tmp_path, "нет целей ['nope']\n", 1, route_log=route_log)
    assert proc.returncode == 1, proc.stderr
    assert "ERROR before GOAL_START:" in proc.stderr
    assert route_log.read_text(encoding="utf-8") == _log(tmp_path)
    assert "нет целей" in route_log.read_text(encoding="utf-8")
