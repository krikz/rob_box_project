"""Переключатель ``USE_SIM_TIME`` в боевых стартовых скриптах Main Pi (сим, шаг 2).

Профиль ``docker/sim`` поднимает в Gazebo те же узлы навигации, что на роботе,
теми же скриптами и с теми же конфигами: robot_state_publisher, twist_mux,
rtabmap, Nav2. Отличаются две переменные окружения контейнера:

* ``USE_SIM_TIME=true`` — узлы берут время из ``/clock`` (Gazebo);
* ``URDF_SIM=true`` — robot_state_publisher строит URDF с ``sim:=true``.

Главная проверка — **без флагов робот получает ровно ту же команду, что до
шага 2** (эталоны ниже сняты с develop ``662b7f6cd`` этим же стендом).

Как устроен стенд: настоящий скрипт исполняется в ``bash``, но ``exec``,
``ros2``, ``xacro``, ``python3``, ``sleep``, ``wait`` и ``source /opt/ros/...``
подменены функциями, которые печатают, что бы запустилось. ``exec`` ещё
печатает содержимое params-файла из ``/tmp``, если он передан. ROS и Docker
не нужны.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
MAIN_SCRIPTS = REPO_ROOT / "docker" / "main" / "scripts"
COMMON = REPO_ROOT / "docker" / "common"

BASH = shutil.which("bash")
pytestmark = pytest.mark.skipif(BASH is None, reason="нет bash в PATH")

PRELUDE = r"""
source() {
    case "$1" in
        /opt/ros/*) return 0 ;;
        /ros_scripts/*) builtin source "$REPO_COMMON/scripts/${1#/ros_scripts/}" ;;
        *) builtin source "$@" ;;
    esac
}
# Конфиги в /config на стенде нет — считаем, что они есть.
[() {
    if [[ "$1" == "!" && "$2" == "-f" && "$3" == /config/* ]]; then return 1; fi
    builtin [ "$@"
}
exec() {
    echo "EXEC: $*"
    local a
    for a in "$@"; do
        if [[ "$a" == /tmp/*.yaml && -f "$a" ]]; then cat "$a"; fi
    done
    exit 0
}
ros2() {
    echo "RUN: ros2 $*"
    if [[ "$1 $2" == "topic list" ]]; then echo /odom; fi
}
xacro() { echo "XACRO: $*" >&2; echo '<robot name="stub"/>' ; }
python3() { echo "RUN: python3 $*"; }
sleep() { :; }
wait() { :; }
. "$SCRIPT" "$@"
"""

SIM_SWITCHES = ("USE_SIM_TIME", "URDF_SIM", "NAV2_PARAMS_OVERLAYS", "NAV2_SMOOTH_CMD_VEL")

RTABMAP_ARGS = ["ros2", "launch", "rtabmap_launch", "rtabmap.launch.py", "use_sim_time:=false"]


def run_script(rel: str, env_extra: dict[str, str] | None = None, args: list[str] | None = None):
    env = {k: v for k, v in os.environ.items() if k not in SIM_SWITCHES}
    env.update(
        {
            "SCRIPT": (MAIN_SCRIPTS / rel).as_posix(),
            "REPO_COMMON": COMMON.as_posix(),
            "ROS_DISTRO": "humble",
        }
    )
    env.update(env_extra or {})
    return subprocess.run(
        [BASH, "-c", PRELUDE, "bash", *(args or [])],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        encoding="utf-8",
        text=True,
        timeout=60,
    )


def commands(out: str) -> list[str]:
    """Строки EXEC/RUN/XACRO; порядок фоновых ``ros2 run &`` не важен — сортируем."""
    return sorted(
        line for line in out.splitlines() if line.startswith(("EXEC: ", "RUN: ", "XACRO: "))
    )


def yaml_tail(out: str) -> list[str]:
    """Строки params-файла robot_state_publisher после строки EXEC."""
    lines = out.splitlines()
    idx = next(i for i, line in enumerate(lines) if line.startswith("EXEC: "))
    return lines[idx + 1:]


# ── Эталоны «как на роботе» (develop 662b7f6cd, до шага 2) ──────────────────

NAV2_NODES = [
    ("nav2_controller controller_server", ""),
    ("nav2_planner planner_server", "-r __node:=planner_server "),
    ("nav2_bt_navigator bt_navigator", "-r __node:=bt_navigator "),
    ("nav2_behaviors behavior_server", "-r __node:=behavior_server "),
    ("nav2_waypoint_follower waypoint_follower", "-r __node:=waypoint_follower "),
    ("nav2_velocity_smoother velocity_smoother", "-r __node:=velocity_smoother "),
    ("nav2_smoother smoother_server", "-r __node:=smoother_server "),
]
PARAMS = "/config/nav2/nav2_params.yaml"


def nav2_expected(sim_args: str) -> list[str]:
    lines = [
        "RUN: ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args "
        f"--params-file {PARAMS} {sim_args}-r __node:=lifecycle_manager_navigation",
    ]
    for exe, node_remap in NAV2_NODES:
        lines.append(
            f"RUN: ros2 run {exe} --ros-args --params-file {PARAMS} {sim_args}"
            f"{node_remap}-r /tf:=tf -r /tf_static:=tf_static"
        )
    return sorted(lines)


TWIST_MUX_EXPECTED = [
    "EXEC: ros2 run twist_mux twist_mux --ros-args --params-file "
    "/config/twist_mux/twist_mux.yaml -r cmd_vel_out:=/diff_drive_controller/cmd_vel_unstamped "
    "-p use_sim_time:=false"
]

RTABMAP_EXPECTED = sorted(
    [
        "RUN: python3 /scripts/patch_rtabmap_launch.py",
        "EXEC: ros2 launch rtabmap_launch rtabmap.launch.py use_sim_time:=false localization:=true",
    ]
)

RSP_EXPECTED = sorted(
    [
        "XACRO: /workspace/src/rob_box_description/urdf/rob_box.xacro",
        "EXEC: ros2 run robot_state_publisher robot_state_publisher --ros-args "
        "--params-file /tmp/robot_state_publisher_params.yaml",
    ]
)
RSP_YAML_EXPECTED = [
    "robot_state_publisher:",
    "  ros__parameters:",
    "    robot_description: |",
    '      <robot name="stub"/>',
    "    use_sim_time: false",
    "    publish_frequency: 30.0",
]


# ── Без флага — как на роботе ────────────────────────────────────────────────


def test_nav2_without_flag_is_unchanged():
    r = run_script("nav2/start_nav2_direct.sh")
    assert r.returncode == 0, r.stdout
    assert commands(r.stdout) == nav2_expected("")


def test_twist_mux_without_flag_is_unchanged():
    r = run_script("twist_mux/start_twist_mux.sh")
    assert r.returncode == 0, r.stdout
    assert commands(r.stdout) == TWIST_MUX_EXPECTED


def test_rtabmap_without_flag_is_unchanged():
    r = run_script("rtabmap/start_rtabmap.sh", args=RTABMAP_ARGS)
    assert r.returncode == 0, r.stdout
    assert commands(r.stdout) == RTABMAP_EXPECTED


def test_robot_state_publisher_without_flag_is_unchanged():
    r = run_script("robot_state_publisher/start_robot_state_publisher.sh")
    assert r.returncode == 0, r.stdout
    assert commands(r.stdout) == RSP_EXPECTED
    assert yaml_tail(r.stdout) == RSP_YAML_EXPECTED


# Явное USE_SIM_TIME=false — тоже как на роботе.


@pytest.mark.parametrize(
    "rel,args,expected",
    [
        ("nav2/start_nav2_direct.sh", None, nav2_expected("")),
        ("twist_mux/start_twist_mux.sh", None, TWIST_MUX_EXPECTED),
        ("rtabmap/start_rtabmap.sh", RTABMAP_ARGS, RTABMAP_EXPECTED),
        ("robot_state_publisher/start_robot_state_publisher.sh", None, RSP_EXPECTED),
    ],
)
def test_explicit_false_is_same_as_robot(rel, args, expected):
    r = run_script(rel, {"USE_SIM_TIME": "false", "URDF_SIM": "false"}, args)
    assert r.returncode == 0, r.stdout
    assert commands(r.stdout) == expected


# ── USE_SIM_TIME=true — профиль docker/sim ───────────────────────────────────


def test_nav2_sim_time_overrides_every_node():
    r = run_script("nav2/start_nav2_direct.sh", {"USE_SIM_TIME": "true"})
    assert r.returncode == 0, r.stdout
    assert commands(r.stdout) == nav2_expected("-p use_sim_time:=true ")


def test_twist_mux_sim_time():
    r = run_script("twist_mux/start_twist_mux.sh", {"USE_SIM_TIME": "true"})
    assert r.returncode == 0, r.stdout
    assert commands(r.stdout) == [
        TWIST_MUX_EXPECTED[0].replace("use_sim_time:=false", "use_sim_time:=true")
    ]


def test_rtabmap_sim_time_goes_after_compose_args():
    r = run_script("rtabmap/start_rtabmap.sh", {"USE_SIM_TIME": "true"}, RTABMAP_ARGS)
    assert r.returncode == 0, r.stdout
    assert commands(r.stdout) == sorted(
        [
            "RUN: python3 /scripts/patch_rtabmap_launch.py",
            "EXEC: ros2 launch rtabmap_launch rtabmap.launch.py use_sim_time:=false "
            "localization:=true use_sim_time:=true",
        ]
    )


def test_robot_state_publisher_sim_time_and_sim_urdf():
    r = run_script(
        "robot_state_publisher/start_robot_state_publisher.sh",
        {"USE_SIM_TIME": "true", "URDF_SIM": "true"},
    )
    assert r.returncode == 0, r.stdout
    xacro_line = "XACRO: /workspace/src/rob_box_description/urdf/rob_box.xacro sim:=true"
    assert xacro_line in commands(r.stdout)
    assert yaml_tail(r.stdout) == [
        line.replace("use_sim_time: false", "use_sim_time: true") for line in RSP_YAML_EXPECTED
    ]


# ── Опечатка — громкое падение, а не тихий режим ─────────────────────────────


@pytest.mark.parametrize(
    "rel,args",
    [
        ("nav2/start_nav2_direct.sh", None),
        ("twist_mux/start_twist_mux.sh", None),
        ("rtabmap/start_rtabmap.sh", RTABMAP_ARGS),
        ("robot_state_publisher/start_robot_state_publisher.sh", None),
    ],
)
def test_bad_use_sim_time_fails_loudly(rel, args):
    r = run_script(rel, {"USE_SIM_TIME": "yes"}, args)
    assert r.returncode != 0
    assert "USE_SIM_TIME='yes'" in r.stdout
    assert not any(line.startswith("EXEC: ") for line in r.stdout.splitlines())


def test_bad_urdf_sim_fails_loudly():
    r = run_script("robot_state_publisher/start_robot_state_publisher.sh", {"URDF_SIM": "1"})
    assert r.returncode != 0
    assert "URDF_SIM='1'" in r.stdout


# ── Шаг 6 плана Gazebo: оверлеи параметров и цепочка velocity_smoother ───────
# Обе переменные ставит только docker/sim/compose.yaml; без них команды выше
# (test_nav2_without_flag_is_unchanged) — ровно как на роботе.

SIM_NAV2 = REPO_ROOT / "src" / "rob_box_sim" / "config" / "nav2"


def _nav2_cmd(out: str, exe: str) -> str:
    return next(line for line in commands(out) if f"ros2 run {exe} " in line)


def test_nav2_overlays_go_after_robot_params_on_every_node():
    a, b = (SIM_NAV2 / "sim.yaml").as_posix(), (SIM_NAV2 / "controller_rpp.yaml").as_posix()
    r = run_script("nav2/start_nav2_direct.sh", {"NAV2_PARAMS_OVERLAYS": f"{a} {b}"})
    assert r.returncode == 0, r.stdout
    lines = commands(r.stdout)
    assert len(lines) == len(NAV2_NODES) + 1
    for line in lines:
        assert f"--params-file {PARAMS} --params-file {a} --params-file {b} " in line, line


def test_nav2_missing_overlay_fails_loudly():
    r = run_script("nav2/start_nav2_direct.sh", {"NAV2_PARAMS_OVERLAYS": "/nonexistent/x.yaml"})
    assert r.returncode != 0
    assert "/nonexistent/x.yaml" in r.stdout
    assert not any(line.startswith("RUN: ros2 run") for line in r.stdout.splitlines())


def test_nav2_smoother_chain_remaps_controller_and_smoother_only():
    r = run_script("nav2/start_nav2_direct.sh", {"NAV2_SMOOTH_CMD_VEL": "true"})
    assert r.returncode == 0, r.stdout
    assert "-r cmd_vel:=cmd_vel_nav -r /tf:=tf" in _nav2_cmd(r.stdout, "nav2_controller controller_server")
    assert "-r cmd_vel:=cmd_vel_nav -r cmd_vel_smoothed:=cmd_vel -r __node:=velocity_smoother" in _nav2_cmd(
        r.stdout, "nav2_velocity_smoother velocity_smoother"
    )
    others = [
        line for line in commands(r.stdout)
        if "controller_server" not in line and "velocity_smoother" not in line
    ]
    assert others and not any("cmd_vel" in line for line in others)


def test_nav2_bad_smoother_switch_fails_loudly():
    r = run_script("nav2/start_nav2_direct.sh", {"NAV2_SMOOTH_CMD_VEL": "yes"})
    assert r.returncode != 0
    assert "NAV2_SMOOTH_CMD_VEL='yes'" in r.stdout
