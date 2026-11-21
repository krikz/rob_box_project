"""#5043: healthcheck nav2 спрашивает lifecycle, а не pgrep bt_navigator.

pgrep зеленел, когда controller_server умер (SIGILL) или завис на configure:
bt_navigator запускается отдельной командой и переживает контроллер. Гард
держит compose, юнит гоняет скрипт с подставным ros2 в PATH.
"""

from __future__ import annotations

import os
import stat
import subprocess
import textwrap
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
COMPOSE = REPO / "docker" / "main" / "docker-compose.yaml"
SCRIPT = REPO / "docker" / "main" / "scripts" / "nav2" / "healthcheck.sh"

NODES = ("/controller_server", "/planner_server", "/bt_navigator")


def _nav2_healthcheck() -> dict:
    compose = yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))
    hc = compose["services"]["nav2"]["healthcheck"]
    assert isinstance(hc, dict), hc
    return hc


def _test_command(hc: dict) -> str:
    test = hc["test"]
    assert test[0] == "CMD-SHELL", test
    assert len(test) == 2, test
    return test[1]


def test_script_defaults_to_humble_setup():
    text = SCRIPT.read_text(encoding="utf-8")
    assert 'ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"' in text
    assert 'source "$ROS_SETUP"' in text


def test_compose_healthcheck_is_lifecycle_script_not_pgrep():
    hc = _nav2_healthcheck()
    command = _test_command(hc)
    assert "pgrep" not in command, command
    assert "/scripts/healthcheck.sh" in command, command
    assert hc["interval"] == "15s"
    assert hc["timeout"] == "10s"
    assert hc["retries"] == 3
    # Зависание configure наблюдали 176 с; 60 с start_period кончался раньше.
    period = hc["start_period"]
    assert period.endswith("s"), period
    assert int(period[:-1]) >= 90


def test_script_is_executable_in_git():
    out = subprocess.run(
        ["git", "ls-files", "-s", "docker/main/scripts/nav2/healthcheck.sh"],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    assert out.strip(), "healthcheck.sh не в git"
    mode = out.split()[0]
    assert mode == "100755", f"git mode {mode}, нужен 100755 (Permission denied на роботе)"
    assert SCRIPT.stat().st_mode & stat.S_IXUSR


def _fake_ros2(bin_dir: Path, answers: dict[str, str], hang: set[str] | None = None) -> None:
    """Подставной ros2: lifecycle get печатает ответ или молчит."""
    hang = hang or set()
    cases = []
    for node in NODES:
        if node in hang:
            body = "sleep 30"
        else:
            body = f"printf '%s\\n' {answers[node]!r}"
        cases.append(f'"{node}") {body} ;;')
    script = textwrap.dedent(
        f"""\
        #!/bin/bash
        if [ "$1" != "lifecycle" ] || [ "$2" != "get" ]; then
            echo "fake ros2: unexpected args: $*" >&2
            exit 2
        fi
        case "$3" in
        {chr(10).join(cases)}
          *) echo "fake ros2: unknown node $3" >&2; exit 2 ;;
        esac
        """
    )
    path = bin_dir / "ros2"
    path.write_text(script, encoding="utf-8")
    path.chmod(0o755)


def _setup(tmp_path: Path) -> Path:
    opt = tmp_path / "opt" / "ros" / "humble"
    opt.mkdir(parents=True)
    (opt / "setup.bash").write_text("# fake humble setup\n", encoding="utf-8")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    return bin_dir


def _run(tmp_path: Path, bin_dir: Path) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}:{env.get('PATH', '')}"
    env["ROS_SETUP"] = str(tmp_path / "opt" / "ros" / "humble" / "setup.bash")
    return subprocess.run(
        ["bash", str(SCRIPT)],
        capture_output=True,
        text=True,
        env=env,
        timeout=40,
    )


def test_all_active_exits_zero(tmp_path):
    bin_dir = _setup(tmp_path)
    _fake_ros2(bin_dir, {node: "active [3]" for node in NODES})
    proc = _run(tmp_path, bin_dir)
    assert proc.returncode == 0, proc.stderr
    assert proc.stderr == ""


def test_one_inactive_exits_one(tmp_path):
    bin_dir = _setup(tmp_path)
    answers = {node: "active [3]" for node in NODES}
    answers["/controller_server"] = "inactive [2]"
    _fake_ros2(bin_dir, answers)
    proc = _run(tmp_path, bin_dir)
    assert proc.returncode == 1, proc.stderr
    assert "/controller_server" in proc.stderr
    assert "inactive" in proc.stderr


def test_no_answer_exits_one(tmp_path):
    bin_dir = _setup(tmp_path)
    answers = {node: "active [3]" for node in NODES}
    _fake_ros2(bin_dir, answers)
    # ros2 для planner_server завершается с ошибкой и пустым stdout.
    ros2 = bin_dir / "ros2"
    text = ros2.read_text(encoding="utf-8")
    text = text.replace(
        "\"/planner_server\") printf '%s\\n' 'active [3]' ;;",
        '"/planner_server") echo "service not available" >&2; exit 1 ;;',
    )
    ros2.write_text(text, encoding="utf-8")
    proc = _run(tmp_path, bin_dir)
    assert proc.returncode == 1, proc.stderr
    assert "/planner_server" in proc.stderr


def test_hung_node_exits_one_on_timeout(tmp_path):
    bin_dir = _setup(tmp_path)
    answers = {node: "active [3]" for node in NODES}
    _fake_ros2(bin_dir, answers, hang={"/controller_server"})
    proc = _run(tmp_path, bin_dir)
    assert proc.returncode == 1, proc.stderr
    assert "/controller_server" in proc.stderr
    assert "no answer" in proc.stderr
