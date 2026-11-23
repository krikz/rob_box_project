"""#5043: healthcheck nav2 спрашивает lifecycle, а не pgrep bt_navigator.
#5060: префикс zenoh в stdout не должен красить active в unhealthy.
#5150: три CLI подряд не влезают в docker Timeout 10 с и жгут ядра. Проба —
процесс rclpy на три узла, общий дедлайн короче bond-ориентира 4 с. Timeout
решением Шифу 23.11 не поднимается.

Гард держит compose, юнит гоняет скрипт с подменным rclpy в PYTHONPATH.
Живой nav2 здесь не спрашивается: `ros2 lifecycle get` на роботе грузит хост.
"""

from __future__ import annotations

import os
import stat
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
COMPOSE = REPO / "docker" / "main" / "docker-compose.yaml"
SCRIPT = REPO / "docker" / "main" / "scripts" / "nav2" / "healthcheck.sh"
PROBE_DIR = REPO / "docker" / "main" / "scripts" / "nav2"
DOCKERFILE = REPO / "docker" / "main" / "nav2" / "Dockerfile"
WORKFLOW = REPO / ".github" / "workflows" / "L-Deploy and Verify.yml"

NODES = ("controller_server", "planner_server", "bt_navigator")

sys.path.insert(0, str(PROBE_DIR))
from healthcheck_budget import (  # noqa: E402
    BOND_TIMEOUT_S,
    DEPLOY_CONFIRM_INTERVAL_S,
    DOCKER_INTERVAL_S,
    DOCKER_RETRIES,
    DOCKER_START_PERIOD_S,
    DOCKER_TIMEOUT_S,
    PROBE_DEADLINE_S,
    budget_holds,
)
from lifecycle_probe import collect_states, verdict  # noqa: E402


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
    # #5150: числа из healthcheck_budget.py. Timeout Шифу не поднимает.
    assert hc["interval"] == f"{DOCKER_INTERVAL_S}s"
    assert hc["timeout"] == f"{DOCKER_TIMEOUT_S}s"
    assert hc["retries"] == DOCKER_RETRIES
    # Зависание configure наблюдали 176 с; 60 с start_period кончался раньше.
    period = hc["start_period"]
    assert period == f"{DOCKER_START_PERIOD_S}s"


def test_dockerfile_healthcheck_matches_budget():
    # Образ собирается редко, но HEALTHCHECK не должен расходиться с compose:
    # пока compose-блок жив, образный не исполняется, после удаления — да.
    text = DOCKERFILE.read_text(encoding="utf-8")
    assert f"--interval={DOCKER_INTERVAL_S}s" in text
    assert f"--timeout={DOCKER_TIMEOUT_S}s" in text
    assert f"--start-period={DOCKER_START_PERIOD_S}s" in text
    assert "bash /scripts/healthcheck.sh" in text
    assert "node list" not in text


def test_deploy_gate_waits_budget_interval():
    # Пауза гейта Main Pi равна Interval: Timeout не поднимаем, ждать дольше
    # одной пробы гейту незачем. Число — из модуля, не из текста workflow.
    text = WORKFLOW.read_text(encoding="utf-8")
    assert f"CONFIRM_INTERVAL={DEPLOY_CONFIRM_INTERVAL_S} \\" in text
    assert DEPLOY_CONFIRM_INTERVAL_S == DOCKER_INTERVAL_S


def test_budget_numbers_come_from_measurement():
    # 3.0 < 4.0 (ориентир bond) и 3.0 < 10 (Timeout, который не поднимаем).
    # 1.5 с замера одного процесса укладывается в дедлайн с запасом.
    assert budget_holds()
    assert PROBE_DEADLINE_S < BOND_TIMEOUT_S
    assert PROBE_DEADLINE_S < DOCKER_TIMEOUT_S == 10
    assert BOND_TIMEOUT_S == 4.0


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


def test_probe_imports_only_stdlib_and_ros():
    # /scripts — bind-mount с репо. Пакет вне каталога до робота не доедет.
    text = (PROBE_DIR / "lifecycle_probe.py").read_text(encoding="utf-8")
    for forbidden in ("rob_box_", "import yaml", "import requests"):
        assert forbidden not in text, forbidden


# --- подменный rclpy: один процесс, три сервиса, без живого nav2 -----------


def _fake_rclpy(
    directory: Path,
    answers: dict[str, str | None],
    hang: set[str] | None = None,
) -> None:
    """Пакет ``rclpy`` и ``lifecycle_msgs``, которые читает probe.

    ``answers[node] is None`` — сервис не появляется (узел недоступен).
    ``node in hang`` — сервис виден, но ответ не приходит никогда.
    """
    hang = hang or set()
    encoded = ", ".join(repr(node) + ": " + repr(state) for node, state in answers.items())
    hanging = ", ".join(repr(node) for node in hang)
    pkg = directory / "rclpy"
    pkg.mkdir()
    (pkg / "__init__.py").write_text(
        textwrap.dedent(
            f"""\
            ANSWERS = {{{encoded}}}
            HANG = {{{hanging}}}
            CALLS = []

            def init(*_a, **_k):
                return None

            def shutdown(*_a, **_k):
                return None

            def create_node(_name):
                return _Node()

            def spin_once(_node, timeout_sec=0.0):
                return None

            def spin_until_future_complete(_node, future, timeout_sec=0.0):
                CALLS.append(timeout_sec)
                if future.node_name in HANG:
                    return None
                future.done_ = True

            class _Node:
                def create_client(self, _srv, service):
                    return _Client(service)

                def destroy_node(self):
                    return None

            class _Client:
                def __init__(self, service):
                    self.node_name = service.split("/")[1]

                def service_is_ready(self):
                    return ANSWERS.get(self.node_name) is not None

                def call_async(self, _request):
                    return _Future(self.node_name)

            class _Future:
                def __init__(self, node_name):
                    self.node_name = node_name
                    self.done_ = False

                def done(self):
                    return self.done_

                def result(self):
                    label = ANSWERS[self.node_name]
                    return _Result(label)

            class _Result:
                def __init__(self, label):
                    self.current_state = _State(label)

            class _State:
                def __init__(self, label):
                    self.label = label
            """
        ),
        encoding="utf-8",
    )
    lifecycle = directory / "lifecycle_msgs"
    lifecycle.mkdir()
    (lifecycle / "__init__.py").write_text("", encoding="utf-8")
    (lifecycle / "srv.py").write_text(
        "class GetState:\n    class Request:\n        pass\n",
        encoding="utf-8",
    )


def _setup(tmp_path: Path) -> Path:
    opt = tmp_path / "opt" / "ros" / "humble"
    opt.mkdir(parents=True)
    (opt / "setup.bash").write_text("# fake humble setup\n", encoding="utf-8")
    fake = tmp_path / "fake_ros"
    fake.mkdir()
    return fake


def _run(tmp_path: Path, fake: Path) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(fake)
    env["ROS_SETUP"] = str(tmp_path / "opt" / "ros" / "humble" / "setup.bash")
    # Дедлайн 3 с на подмене не нужен: ответы мгновенные. Бюджет wall-clock
    # проверяет отдельный тест на collect_states.
    return subprocess.run(
        ["bash", str(SCRIPT)],
        capture_output=True,
        text=True,
        env=env,
        timeout=20,
    )


def test_all_active_exits_zero(tmp_path):
    # Приёмка, случай 1 (mock, не живой робот): три active → 0, stderr пуст.
    fake = _setup(tmp_path)
    _fake_rclpy(fake, {node: "active" for node in NODES})
    proc = _run(tmp_path, fake)
    assert proc.returncode == 0, proc.stderr
    assert proc.stderr == ""


def test_one_inactive_exits_one(tmp_path):
    # Приёмка, случай 2: один узел не active → не 0 и внятная причина.
    fake = _setup(tmp_path)
    answers = {node: "active" for node in NODES}
    answers["controller_server"] = "inactive"
    _fake_rclpy(fake, answers)
    proc = _run(tmp_path, fake)
    assert proc.returncode == 1, proc.stderr
    assert "/controller_server" in proc.stderr
    assert "inactive" in proc.stderr
    assert "want active" in proc.stderr


def test_unavailable_node_exits_one(tmp_path):
    # Узел недоступен: сервис не появляется → не 0, причина «no answer».
    fake = _setup(tmp_path)
    answers: dict[str, str | None] = {node: "active" for node in NODES}
    answers["planner_server"] = None
    _fake_rclpy(fake, answers)
    proc = _run(tmp_path, fake)
    assert proc.returncode == 1, proc.stderr
    assert "/planner_server" in proc.stderr
    assert "no answer" in proc.stderr


def test_hung_call_exits_one_within_budget(tmp_path):
    # Зависший ответ не выходит за дедлайн пробы. Подмена отвечает мгновенно
    # «не готов», поэтому wall-clock здесь — накладные скрипта, не дедлайн.
    # Сам дедлайн проверяет test_collect_states_stops_at_deadline.
    fake = _setup(tmp_path)
    answers = {node: "active" for node in NODES}
    _fake_rclpy(fake, answers, hang={"bt_navigator"})
    started = time.monotonic()
    proc = _run(tmp_path, fake)
    elapsed = time.monotonic() - started
    assert proc.returncode == 1, proc.stderr
    assert "/bt_navigator" in proc.stderr
    assert "no answer" in proc.stderr
    assert elapsed < DOCKER_TIMEOUT_S, elapsed


def test_setup_with_unset_var_still_reaches_probe(tmp_path):
    # ament setup.bash читает $AMENT_TRACE_SETUP_FILES без проверки. Под set -u
    # это убило бы healthcheck навсегда. Стаб кончается без exit — как реальный
    # setup.sh, который кончается на return, а не на exit.
    fake = _setup(tmp_path)
    setup = tmp_path / "opt" / "ros" / "humble" / "setup.bash"
    setup.write_text(
        'echo "$NAV2_HC_UNSET_VAR"\nexport AMENT_PREFIX_PATH=/opt/ros/humble\n',
        encoding="utf-8",
    )
    _fake_rclpy(fake, {node: "active" for node in NODES})
    proc = _run(tmp_path, fake)
    assert proc.returncode == 0, proc.stderr
    assert proc.stderr == ""


def test_setup_exit_is_reported(tmp_path):
    fake = _setup(tmp_path)
    setup = tmp_path / "opt" / "ros" / "humble" / "setup.bash"
    setup.write_text("exit 7\n", encoding="utf-8")
    _fake_rclpy(fake, {node: "active" for node in NODES})
    proc = _run(tmp_path, fake)
    assert proc.returncode == 1, proc.stderr
    assert f"source {setup} failed rc=7" in proc.stderr


def test_probe_failure_is_reported(tmp_path):
    # Импорт rclpy упал — причина в stderr, не трейс и не молчаливый ноль.
    fake = _setup(tmp_path)
    proc = _run(tmp_path, fake)
    assert proc.returncode == 1, proc.stderr
    assert "probe failed" in proc.stderr


# --- чистые функции: дедлайн и вердикт, без подмены ROS ---------------------


def test_verdict_names_every_failure():
    code, reason = verdict(
        {
            "controller_server": "active",
            "planner_server": "inactive",
            "bt_navigator": None,
        }
    )
    assert code == 1
    assert "/planner_server" in reason and "inactive" in reason
    assert "/bt_navigator" in reason and "no answer" in reason
    assert "/controller_server" not in reason


def test_collect_states_stops_at_deadline():
    # Зависший первый вызов не выходит за общий дедлайн и не съедает остальных:
    # каждый получает долю остатка, сумма не больше дедлайна.
    clock = {"t": 0.0}

    def now() -> float:
        return clock["t"]

    def spin_once(_timeout: float) -> None:
        clock["t"] += 0.01

    asked: list[tuple[str, float]] = []

    def call_state(name: str, timeout_s: float) -> str | None:
        asked.append((name, timeout_s))
        clock["t"] += timeout_s
        if name == "controller_server":
            return None
        return "active"

    states = collect_states(NODES, PROBE_DEADLINE_S, now, spin_once, lambda _name: True, call_state)
    assert states["controller_server"] is None
    assert states["planner_server"] == "active"
    assert states["bt_navigator"] == "active"
    assert clock["t"] <= PROBE_DEADLINE_S + 0.02
    assert sum(timeout for _name, timeout in asked) <= PROBE_DEADLINE_S


def test_collect_states_reports_node_that_never_appears():
    clock = {"t": 0.0}

    def now() -> float:
        return clock["t"]

    def spin_once(timeout_s: float) -> None:
        clock["t"] += timeout_s

    states = collect_states(
        NODES,
        0.2,
        now,
        spin_once,
        lambda name: name != "planner_server",
        lambda name, _timeout: "active",
    )
    assert states["planner_server"] is None
    assert states["controller_server"] == "active"
    assert clock["t"] <= 0.2 + 0.01
