"""Повторный M1 (#4694): список топиков bag и фиксация условий до цели.

Живой HIL не нужен. Проверяется код scripts/hil/repeat/m1_topics.py и то, что
сценарий вызывает гейт мира как есть и копирует bag на хост до hil off.
"""

from __future__ import annotations

import importlib.util
import math
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
REPO = ROOT
REPEAT = ROOT / "scripts" / "hil" / "repeat"
_spec = importlib.util.spec_from_file_location("hil_m1_topics", REPEAT / "m1_topics.py")
topics = importlib.util.module_from_spec(_spec)
sys.modules["hil_m1_topics"] = topics
_spec.loader.exec_module(topics)

REQUIRED = (
    "/cmd_vel_yaw",
    "/cmd_vel_nav",
    "/global_costmap/costmap",
    "/global_costmap/costmap_raw",
    "/rosout",
    "/evaluation",
    "/trajectories",
    "/local_plan",
)


def test_repeat_topics_add_yaw_costmap_and_controller_before_mux():
    names = topics.repeat_bag_topics()
    for name in REQUIRED:
        assert name in names, name
    # База прошлого bag (/tmp/hil_m1_stall) не выкинута.
    for name in (
        "/cmd_vel",
        "/plan",
        "/sim/ground_truth/odom",
        "/odom",
        "/local_costmap/costmap_raw",
    ):
        assert name in names
    assert topics.DWB_ODOM_TOPIC == "/odom"
    assert topics.DWB_ODOM_TOPIC != "/sim/ground_truth/odom"
    assert names.count("/cmd_vel_yaw") == 1
    assert topics.CONTROLLER_BEFORE_MUX == "/cmd_vel_nav"
    assert "/cmd_vel_nav" != "/cmd_vel"


def test_bag_qos_is_reliable_so_evaluation_is_not_dropped():
    text = topics.bag_qos_overrides_yaml()
    assert "/evaluation:" in text
    assert "reliability: reliable" in text
    assert "best_effort" not in text
    script = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    assert "--qos-profile-overrides-path" in script


def test_record_argv_refuses_list_without_required():
    argv = topics.record_argv("/tmp/bag")
    assert argv[:5] == ["ros2", "bag", "record", "-o", "/tmp/bag"]
    assert "/cmd_vel_yaw" in argv
    with pytest.raises(ValueError, match="cmd_vel_yaw"):
        topics.record_argv("/tmp/bag", topics.BASE_BAG_TOPICS)


def test_start_map_gt_does_not_invent_zero():
    text = topics.format_start_map_gt(None, {"x": 0.0, "y": 1.0})
    assert text == "map none\ngt none\n"
    full = topics.format_start_map_gt(
        {"x": -0.001, "y": 0.002, "yaw": 0.01},
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    assert full.startswith("map x=-0.001000 y=0.002000 yaw=0.010000\n")
    assert "gt x=0.000000" in full


def test_parse_tf_and_ground_truth_raw():
    tf = (
        "At time 1.0\n"
        "- Translation: [0.010000, -0.020000, 0.000000]\n"
        "- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]\n"
        "- Rotation: in RPY (radian) [0.000, 0.000, 0.100]\n"
    )
    gt = (
        "pose:\n"
        "  pose:\n"
        "    position:\n"
        "      x: -1.3e-05\n"
        "      y: 6.0e-10\n"
        "      z: 0.0\n"
        "    orientation:\n"
        "      x: 0.0\n"
        "      y: 0.0\n"
        "      z: 0.0\n"
        "      w: 1.0\n"
    )
    parsed_tf = topics.parse_tf2_echo(tf)
    assert parsed_tf["x"] == pytest.approx(0.01)
    assert parsed_tf["yaw"] == pytest.approx(0.1)
    parsed_gt = topics.parse_ground_truth_echo(gt)
    assert parsed_gt["x"] == pytest.approx(-1.3e-05)
    assert parsed_gt["yaw"] == pytest.approx(0.0)
    assert topics.parse_tf2_echo("no frame") is None
    assert topics.parse_ground_truth_echo("position:\n  x: 1\n") is None


def test_write_start_and_assert_ready(tmp_path):
    body = topics.write_start_map_gt(tmp_path, "garbage", "garbage")
    assert body == "map none\ngt none\n"
    assert (tmp_path / "start_map_gt.txt").read_text(encoding="utf-8") == body
    for name in topics.BEFORE_GOAL_FILES:
        if name == "start_map_gt.txt":
            continue
        (tmp_path / name).write_text("x\n", encoding="utf-8")
    topics.assert_ready(tmp_path, "before_goal")
    (tmp_path / "sdf.sha256").write_text("", encoding="utf-8")
    with pytest.raises(FileNotFoundError, match="sdf.sha256"):
        topics.assert_ready(tmp_path, "before_goal")


def test_yaw_isolate_detach_drops_input_priority_keeps_it():
    combat = (ROOT / "docker/main/config/twist_mux/twist_mux.yaml").read_text(
        encoding="utf-8"
    )
    assert topics.yaw_isolate_from_env(None) == "off"
    assert topics.yaw_isolate_from_env("1") == "detach"
    assert topics.yaw_isolate_from_env("priority") == "priority"
    detached = topics.twist_mux_config_for_isolate(combat, "detach")
    assert "cmd_vel_yaw" not in detached
    assert "joystick_lock" in detached
    assert "cmd_vel_emergency" in detached
    assert "topic: cmd_vel\n" in detached
    lowered = topics.twist_mux_config_for_isolate(combat, "priority")
    assert "topic: cmd_vel_yaw" in lowered
    assert "priority: 5" in lowered
    assert (
        "priority: 15" not in lowered.split("yaw_at_goal:")[1].split("navigation:")[0]
    )
    assert topics.twist_mux_config_for_isolate(combat, "off") == combat
    with pytest.raises(ValueError, match="M1_YAWATGOAL_ISOLATE"):
        topics.yaw_isolate_from_env("2")
    script = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    assert "YAW_MUX_STILL_SUBSCRIBED" in script
    assert "TWIST_MUX_CONFIG" in script


def test_compose_transfer_does_not_use_main_stdin(tmp_path):
    """main() делает ssh </dev/null. Передача compose через неё пишет 0 байт (Шифу, 19.11)."""
    script = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    code = "\n".join(
        line for line in script.splitlines() if not line.lstrip().startswith("#")
    )
    assert "main() { ssh" in code
    assert "</dev/null" in code
    assert 'main "tr -d' not in code
    assert 'scp -q -o BatchMode=yes "$ART/nav2tune.compose.yaml"' in code
    # Локальная модель: функция с </dev/null выбрасывает файл, scp-путь (cp) — нет.
    src = tmp_path / "compose.yaml"
    src.write_text(
        "services:\n  nav2:\n    environment:\n      - X=1\n", encoding="utf-8"
    )
    swallowed = tmp_path / "via_main.yaml"
    kept = tmp_path / "via_scp.yaml"
    harness = tmp_path / "t.sh"
    harness.write_text(
        "set -uo pipefail\n"
        'main() { bash -c \'cat > "$1"\' _ "$1" </dev/null; }\n'
        f"main {swallowed} < {src} || true\n"
        f"cp {src} {kept}\n"
        f"wc -c {swallowed} {kept}\n",
        encoding="utf-8",
    )
    result = subprocess.run(
        ["bash", str(harness)], capture_output=True, text=True, check=True
    )
    assert swallowed.stat().st_size == 0
    assert kept.read_text(encoding="utf-8") == src.read_text(encoding="utf-8")
    assert "0 " in result.stdout


def test_param_get_timeout_is_missing_not_a_hang():
    """Пустой ответ за таймаут — missing. Скрипт не вызывает param get без timeout 15."""
    assert topics.parse_param_value("") is None
    assert (
        topics.format_named_param("FollowPath.max_vel_x", "")
        == "FollowPath.max_vel_x=missing"
    )
    script = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    code = "\n".join(
        line for line in script.splitlines() if not line.lstrip().startswith("#")
    )
    assert "timeout 15 ros2 param get" not in code or "ros2_in_nav2" in code
    assert 'timeout "$ROS2_CLI_TIMEOUT"' in code
    assert "ros2 param get" in code
    # Две попытки, не цикл до бесконечности.
    assert "for attempt in 1 2" in script


def test_unreachable_ros2_cli_exits_37_not_hangs(tmp_path):
    """run2 висел на ros2 lifecycle get без timeout. Недоступный CLI — код 37, не 30 минут.

    Подмена ssh спит дольше таймаута. Функция готова вернуть 37, цикл не крутится.
    """
    art = tmp_path / "art"
    art.mkdir()
    sleeper = tmp_path / "ssh"
    sleeper.write_text("#!/bin/sh\nexec sleep 30\n", encoding="utf-8")
    sleeper.chmod(0o755)
    text = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    start = text.index("ros2_in_nav2() {")
    rest = text[start:]
    lines = rest.splitlines(keepends=True)
    fn = [lines[0]]
    for line in lines[1:]:
        if line and not line.startswith((" ", "\t", "\n")) and line.endswith("() {\n"):
            break
        fn.append(line)
    harness = tmp_path / "ready.sh"
    # Та же функция, что в скрипте: timeout вернул 124 → exit 37, sleep 5 не вызывается.
    harness.write_text(
        "set -uo pipefail\n"
        f"PATH={tmp_path}:$PATH\n"
        "ART=" + str(art) + "\n"
        "SUMMARY=$ART/verdict.log\n"
        ': >"$SUMMARY"\n'
        'log() { printf \'%s\\n\' "$*" | tee -a "$SUMMARY"; }\n'
        "ROS2_CLI_TIMEOUT=1\n"
        "SSHO=()\n"
        "MAIN=unused\n"
        "NAV2_CONTAINER=nav2\n"
        + "".join(fn)
        + "\ncli_rc=0\n"
        + "state=$(ros2_in_nav2 'ros2 lifecycle get /bt_navigator' 2>/dev/null) || cli_rc=$?\n"
        + 'printf "rc=%s\\n" "$cli_rc"\n'
        + 'if [ "$cli_rc" -eq 124 ]; then log "NAV_CLI_TIMEOUT ros2 lifecycle get /bt_navigator"; exit 37; fi\n'
        + "exit 0\n",
        encoding="utf-8",
        newline="\n",
    )
    started = time.monotonic()
    result = subprocess.run(
        ["bash", str(harness)], capture_output=True, text=True, cwd=tmp_path, timeout=8
    )
    elapsed = time.monotonic() - started
    assert result.returncode == 37, result.stdout + result.stderr
    assert elapsed < 8
    assert "NAV_CLI_TIMEOUT" in (art / "verdict.log").read_text(encoding="utf-8")
    script = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    code = "\n".join(
        line for line in script.splitlines() if not line.lstrip().startswith("#")
    )
    assert "ros2 lifecycle get /bt_navigator" in code
    assert code.count("exit 37") >= 2
    # Голый вызов без ros2_in_nav2 больше не стоит в цикле готовности.
    assert "ros2 lifecycle get /bt_navigator'" not in code or "ros2_in_nav2" in code


def test_banner_without_value_is_missing_not_equals_signs():
    """Ночной прогон 19.11: баннер ros_with_namespace.sh без «value is» стал значением."""
    banner = (
        "==================================================\n"
        "  ROS Node с Zenoh namespace\n"
        "🚀 Запуск: ros2 param get /controller_server FollowPath.max_vel_x\n"
        "==================================================\n"
    )
    assert topics.parse_param_value(banner) is None
    assert (
        topics.format_named_param("FollowPath.max_vel_x", banner)
        == "FollowPath.max_vel_x=missing"
    )
    mixed = banner + "Double value is: 0.15\n"
    assert topics.parse_param_value(mixed) == "0.15"
    # Повтор: первый ответ баннер, второй — число. Берётся число, баннер не приклеивается.
    assert topics.parse_param_value("Boolean value is: True") == "True"


def test_params_file_names_each_value():
    text = topics.format_params_file(
        [
            ("FollowPath.max_vel_x", "Double value is: 0.15"),
            ("FollowPath.PathAlign.forward_point_distance", "Double value is: 0.2"),
            ("progress_checker.movement_time_allowance", "Double value is: 30.0"),
            ("FollowPath.min_speed_xy", "Double value is: 0.05"),
            ("FollowPath.trans_stopped_velocity", "Double value is: 0.03"),
            ("general_goal_checker.xy_goal_tolerance", "Double value is: 0.25"),
            ("FollowPath.xy_goal_tolerance", "Double value is: 0.25"),
            ("FollowPath.RotateToGoal.scale", "Double value is: 8.0"),
            ("FollowPath.RotateToGoal.lookahead_time", "Double value is: 1.0"),
            ("FollowPath.Oscillation.scale", "Double value is: 0.5"),
            ("FollowPath.Oscillation.oscillation_reset_dist", "Double value is: 0.05"),
            ("FollowPath.Oscillation.oscillation_reset_angle", "Double value is: 0.2"),
            ("FollowPath.Oscillation.oscillation_reset_time", "Double value is: -1.0"),
        ]
    )
    assert "FollowPath.max_vel_x=0.15" in text
    assert "FollowPath.PathAlign.forward_point_distance=0.2" in text
    assert "progress_checker.movement_time_allowance=30.0" in text
    assert "FollowPath.min_speed_xy=0.05" in text
    assert "Double value is" not in text
    assert topics.params_missing_names(text) == []
    # Безымянный ответ без вызова param get — имя missing, ноль не подставляем.
    bare = topics.format_params_file([])
    assert "FollowPath.max_vel_x=missing" in bare
    assert topics.params_missing_names(bare) == list(topics.NAMED_PARAMS)
    assert (
        topics.format_named_param("FollowPath.max_vel_x", "")
        == "FollowPath.max_vel_x=missing"
    )
    assert math.isclose(topics.MAX_VEL_X, 0.15)
    assert math.isclose(topics.PATHALIGN_FORWARD_M, 0.20)


def test_recovery_hits_name_the_marker_not_invent_a_type():
    log = "Failed to make progress\nClearing global costmap\n"
    assert topics.recovery_hits(log) == ["progress_fail", "clear_global"]
    assert topics.recovery_hits("") == []
    assert "spin" not in topics.recovery_hits(log)


def test_xy_goal_tol_is_flat_and_does_not_touch_checker():
    """Число пишет FollowPath.xy_goal_tolerance плоско. Checker и пустой env — нет."""
    diag = topics.dwb_publish_overlay_yaml(xy_goal_tolerance=0.10)
    assert "      xy_goal_tolerance: 0.1" in diag
    # Ключи плагина прямо под FollowPath: второй ros__parameters узел не читает
    # (пара m1repeat_20261119T084941Z: yaml false вложенно, param get True).
    assert "FollowPath:\n      ros__parameters:" not in diag
    assert "general_goal_checker" not in diag
    assert "xy_goal_tolerance" not in topics.dwb_publish_overlay_yaml()
    assert topics.xy_goal_tol_from_env(None) is None
    assert topics.xy_goal_tol_from_env("0.10") == 0.10
    with pytest.raises(ValueError, match="M1_REPEAT_XY_GOAL_TOL"):
        topics.xy_goal_tol_from_env("0")
    script = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    assert "XY_GOAL_TOL_NOT_APPLIED" in script
    assert "general_goal_checker" not in diag


def test_trans_stopped_overlay_is_opt_in():
    """0.005 пишет строку; пусто — нет (боевой 0.03); 0 и нечисло — ошибка."""
    diag = topics.dwb_publish_overlay_yaml(short_circuit=False, trans_stopped=0.005)
    assert "trans_stopped_velocity: 0.005" in diag
    assert "short_circuit_trajectory_evaluation: false" in diag
    assert "max_vel_x" not in diag
    combat = topics.dwb_publish_overlay_yaml()
    assert "trans_stopped_velocity" not in combat
    assert topics.trans_stopped_from_env(None) is None
    assert topics.trans_stopped_from_env("") is None
    assert topics.trans_stopped_from_env("0.005") == 0.005
    with pytest.raises(ValueError, match="M1_REPEAT_TRANS_STOPPED"):
        topics.trans_stopped_from_env("0")
    with pytest.raises(ValueError, match="M1_REPEAT_TRANS_STOPPED"):
        topics.trans_stopped_from_env("nope")


def test_overlay_compose_is_on_the_command_before_configure():
    """Пара 19.11: recreate после on, param остался True. on должен видеть -f до старта nav2.

    Пустой HIL_NAV2TUNE_COMPOSE — три файла, как боевой hil.sh on. Заданный — четвёртый -f
    и NAV2TUNE_OVERLAYS в той же команде, до up. off этот -f не добавляет.
    """
    hil = (REPO / "scripts/hil/hil.sh").read_text(encoding="utf-8")
    fn = hil[hil.index("main_sim_up() {") :]
    body = fn[: fn.index("\n}\n") + 3]
    assert "HIL_NAV2TUNE_COMPOSE" in body
    assert "NAV2TUNE_OVERLAYS" in body
    assert body.index("HIL_NAV2TUNE_COMPOSE") < body.index("docker compose")
    repeat = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    code = "\n".join(
        line for line in repeat.splitlines() if not line.lstrip().startswith("#")
    )
    on_at = code.index('bash "$HIL_CMD" on')
    assert code.index("export HIL_NAV2TUNE_COMPOSE=") < on_at
    assert code.index("export NAV2TUNE_OVERLAYS=") < on_at
    assert "OVERLAY_BEFORE_ON" in code
    # Recreate nav2 после HIL_ON_OK больше нет: второй старт не чинит уже прошедший configure.
    after_on = code.split("HIL_ON_OK", 1)[1].split("capture_before_goal", 1)[0]
    assert "--force-recreate nav2" not in after_on
    assert "OVERLAY_ENV_MISSING" in after_on
    off = hil[hil.index("cmd_off() {") : hil.index("cmd_status() {")]
    assert "HIL_NAV2TUNE_COMPOSE" not in off

    # Модель main_sim_up: пусто — три -f; задано — четвёртый до up, значение без лишних кавычек.
    model = body
    harness_src = (
        "set -uo pipefail\n"
        'q() { printf "\'%s\'" "$1"; }\n'
        "rsh() { printf '%s\\n' \"$2\"; }\n"
        "main_rm_created_sim() { :; }\n"
        "HIL_MAIN_HOST=main\nHIL_MAIN_DIR=/d\nHIL_MAPS_DIR=/m\nHIL_RTABMAP_MODE=mapping\n"
        "HIL_GUARD_SOUND=0\nHIL_REMOTE_DIR=/tmp/hil\n"
        + model
        + "\nunset HIL_NAV2TUNE_COMPOSE NAV2TUNE_OVERLAYS\n"
        + "main_sim_up nav2 > /tmp/m1_plain.txt\n"
        + "HIL_NAV2TUNE_COMPOSE=/tmp/hil/nav2tune.compose.yaml\n"
        + "NAV2TUNE_OVERLAYS=/nav2tune/dwb_publish.yaml\n"
        + "main_sim_up nav2 > /tmp/m1_overlay.txt\n"
    )
    # Пишем во временный файл рядом с тестом через subprocess и bash -c нельзя без файла:
    # harness ниже собирается в tmp_path вызывающим тестом. Здесь — свой прогон.
    import tempfile

    with tempfile.TemporaryDirectory() as raw:
        root = Path(raw)
        script = root / "up.sh"
        plain = root / "plain.txt"
        over = root / "over.txt"
        script.write_text(
            harness_src.replace("/tmp/m1_plain.txt", str(plain)).replace(
                "/tmp/m1_overlay.txt", str(over)
            ),
            encoding="utf-8",
        )
        subprocess.run(["bash", str(script)], check=True, cwd=root)
        plain_txt = plain.read_text(encoding="utf-8")
        over_txt = over.read_text(encoding="utf-8")
    assert plain_txt.count("-f ") == 3
    assert "nav2tune" not in plain_txt
    assert "-f '/tmp/hil/nav2tune.compose.yaml'" in over_txt
    assert "NAV2TUNE_OVERLAYS='/nav2tune/dwb_publish.yaml'" in over_txt
    assert over_txt.index("-f '/tmp/hil/nav2tune.compose.yaml'") < over_txt.index(
        " up "
    )


def test_evaluation_holes_split_recovery_from_recording_drop():
    """Пара 19.11: 15.359 с — recovery (spin/wait), не дыра bag. 2.9 с — пачка cmd_vel на краю.

    recording_drop только если /local_plan внутри окна есть, а /evaluation нет > 1 с.
    На стенде этого не гоняли. Модель не должна звать такие окна потерей записи.
    """
    recovery = topics.classify_evaluation_gap(
        15.359, cmd_vel_inside=104, local_plan_inside=0, recovery_in_window=True
    )
    assert recovery == "controller_silent"
    burst = topics.classify_evaluation_gap(
        2.903, cmd_vel_inside=17, local_plan_inside=0, recovery_in_window=False
    )
    assert burst == "boundary_burst"
    diag = topics.classify_evaluation_gap(
        3.09, cmd_vel_inside=33, local_plan_inside=0, recovery_in_window=False
    )
    assert diag == "boundary_burst"
    short = topics.classify_evaluation_gap(
        0.4, cmd_vel_inside=0, local_plan_inside=0, recovery_in_window=False
    )
    assert short == "ok"
    drop = topics.classify_evaluation_gap(
        1.2, cmd_vel_inside=4, local_plan_inside=3, recovery_in_window=False
    )
    assert drop == "recording_drop"
    gaps = [
        {
            "gap_s": 15.359,
            "cmd_vel_inside": 104,
            "local_plan_inside": 0,
            "recovery_in_window": True,
        },
        {
            "gap_s": 2.903,
            "cmd_vel_inside": 17,
            "local_plan_inside": 0,
            "recovery_in_window": False,
        },
        {
            "gap_s": 1.2,
            "cmd_vel_inside": 4,
            "local_plan_inside": 3,
            "recovery_in_window": False,
        },
    ]
    found = topics.recording_drops(gaps)
    assert len(found) == 1
    assert found[0]["gap_s"] == 1.2
    qos = topics.bag_qos_overrides_yaml(["/evaluation"])
    assert "reliability: reliable" in qos
    assert "durability: volatile" in qos
    assert "deadline:" in qos


def test_short_circuit_overlay_is_opt_in():
    """false добавляет строку; true и default — нет. Боевой yaml не открывается."""
    full = topics.dwb_publish_overlay_yaml(short_circuit=False)
    assert "short_circuit_trajectory_evaluation: false" in full
    # Вложенность боевого yaml: FollowPath.ros__parameters, не плоский ключ.
    # Ключи плагина прямо под FollowPath, без второго ros__parameters.
    assert "    FollowPath:\n      publish_evaluation: true" in full
    assert "      short_circuit_trajectory_evaluation: false" in full
    assert "FollowPath:\n      ros__parameters:" not in full
    assert "publish_evaluation: true" in full
    assert "max_vel_x" not in full
    combat = topics.dwb_publish_overlay_yaml(short_circuit=True)
    assert "short_circuit_trajectory_evaluation" not in combat
    assert (
        "short_circuit_trajectory_evaluation" not in topics.dwb_publish_overlay_yaml()
    )
    assert topics.short_circuit_from_env(None) is True
    assert topics.short_circuit_from_env("true") is True
    assert topics.short_circuit_from_env("false") is False
    with pytest.raises(ValueError, match="M1_REPEAT_SHORT_CIRCUIT"):
        topics.short_circuit_from_env("maybe")


def test_rotate_latch_markers_and_dwb_overlay_do_not_touch_speed():
    log = "IllegalTrajectoryException: Nonrotation command near goal\nNot slowing down near goal\n"
    hits = topics.recovery_hits(log)
    assert hits == ["rotate_nonrotation", "rotate_not_slowing"]
    overlay = topics.dwb_publish_overlay_yaml()
    assert "publish_evaluation: true" in overlay
    assert "publish_trajectories: true" in overlay
    assert "publish_local_plan: true" in overlay
    assert "max_vel_x" not in overlay
    assert "lookahead_time" not in overlay
    assert topics.EVALUATION_TYPE == "dwb_msgs/msg/LocalPlanEvaluation"
    assert topics.TRAJECTORIES_TYPE == "visualization_msgs/msg/MarkerArray"
    assert "/evaluation" in topics.repeat_bag_topics()
    assert "/local_plan" in topics.repeat_bag_topics()


def test_script_uses_world_gate_and_pulls_before_off():
    script = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    assert "HIL_EXPECT_WORLD=depot" in script
    assert "world_check.py" in script or "WORLD_CHECK" in script
    # Артефакты на хост до off: pull вызывается из cleanup раньше hil off.
    pull_at = script.index("pull_before_off || art_rc=1")
    off_at = script.index('bash "$HIL_CMD" off', pull_at)
    assert pull_at < off_at
    assert "ros2 bag record" in script
    assert "topics_line" in script
    assert "FollowPath.min_speed_xy" in script
    assert "movement_time_allowance" in script
    assert "nav2.log" in script
    assert "recovery.txt" in script
    assert "dwb_publish.yaml" in script
    assert "nav2tune.compose.yaml" in script
    # main() глотает stdin (</dev/null). Compose передаётся scp, не main < файл.
    assert 'main "tr -d' not in script
    assert "OVERLAY_EMPTY_ON_MAIN" in script
    assert "OVERLAY_NOT_IN_CONTAINER" in script
    assert "publish_evaluation" in script
    assert "M1_REPEAT_SHORT_CIRCUIT" in script
    assert "M1_REPEAT_TRANS_STOPPED" in script
    assert "capture_params_after_ready" in script
    assert "ros2_in_nav2" in script
    assert "ros2 param get" in script
    ready_at = script.index("for _ in $(seq 1 $((NAV_READY_TIMEOUT / 5)))")
    sampler_at = script.index("start_resource_samplers || log")
    assert sampler_at < ready_at
    assert "sandbox_resources.log" in script
    assert "main_resources.log" in script
    code_only = "\n".join(
        line for line in script.splitlines() if not line.lstrip().startswith("#")
    )
    assert "nvidia-smi" not in code_only
    assert "CONTROLLER_NOT_READY" in script
    assert "/odom" in script or "DWB_ODOM_TOPIC" in "\n".join(
        (REPEAT / "m1_topics.py").read_text(encoding="utf-8").splitlines()
    )
    assert "trans_stopped_from_env" in script
    assert "short_circuit_from_env" in script
    code = "\n".join(
        line for line in script.splitlines() if not line.lstrip().startswith("#")
    )
    assert "nav2_params.yaml" not in code
    # OpenSSH 9 SFTP отвергает filename «.» — прошлый прогон, exit 50.
    scp_lines = [line for line in code.splitlines() if "scp " in line]
    assert scp_lines
    assert all("bag/." not in line for line in scp_lines)
    assert "pull_host_is_local" in code
    assert "OFF_OK" in code
    assert "OFF_FAILED" in code
    assert "return 51" in code
    assert "return 52" in code
    assert "CAMERAS=0 ROUTE_LOG=" in script and "./runvideo.sh '$GOAL'" in script
    assert "hil.conf" in script
    assert 'printf \'%q\' "$HIL_LOCK"' in script
    assert "NAV2_PARAMS_OVERLAYS" in script
    # Гейт не переписываем: сценарий только вызывает существующий world_check.
    assert "Depot_vid" not in script


@pytest.mark.skipif(
    subprocess.call(["bash", "-n", str(REPEAT / "run_m1_repeat.sh")]) != 0,
    reason="bash -n",
)
def test_resource_sample_parses_synthetic_proc(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "hil_resource_sample", REPEAT / "resource_sample.py"
    )
    rs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rs)
    root = tmp_path / "root"
    (root / "proc/pressure").mkdir(parents=True)
    (root / "proc/stat").write_text(
        "cpu  10 0 5 80 1 0 0 0 0 0\ncpu0 5 0 2 40 0 0 0\ncpu1 5 0 3 40 1 0 0\n",
        encoding="utf-8",
    )
    (root / "proc/meminfo").write_text(
        "MemTotal:       8000000 kB\nMemAvailable:   1000000 kB\n", encoding="utf-8"
    )
    (root / "proc/pressure/cpu").write_text(
        "some avg10=1.00 avg60=51.21 avg300=2.00 total=9\n", encoding="utf-8"
    )
    (root / "proc/diskstats").write_text(
        "   8       0 sda 1 0 2 3 4 0 5 0 0 6 7 8 0 0 0 0\n",
        encoding="utf-8",
    )
    line = rs.sample_host(root, "sandbox", "sda", None)
    assert line.startswith("utc=")
    assert "host=sandbox" in line
    assert "cpu=cpu:10,0,5,80,1,0,0;cpu0:" in line
    assert "mem_available_kb=1000000" in line
    assert "psi_cpu=avg10=1.00,avg60=51.21,avg300=2.00,total=9" in line
    assert "disk=sda:r=1" in line
    assert "gpu=missing" in line
    empty = rs.sample_host(tmp_path / "no", "main", "mmcblk0", None)
    assert "cpu=missing" in empty
    assert "mem_total_kb=missing" in empty
    assert "psi_cpu=missing" in empty
    assert "disk=missing" in empty


def test_loop_flag_writes_a_line(tmp_path, monkeypatch):
    """--loop не первым аргументом пишет строку. Пара run1: unrecognized arguments --loop."""
    spec = importlib.util.spec_from_file_location(
        "hil_resource_sample_loop", REPEAT / "resource_sample.py"
    )
    rs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rs)
    root = tmp_path / "root"
    (root / "proc").mkdir(parents=True)
    (root / "proc/stat").write_text("cpu  1 0 0 1 0 0 0\n", encoding="utf-8")
    (root / "proc/meminfo").write_text(
        "MemTotal: 1 kB\nMemAvailable: 1 kB\n", encoding="utf-8"
    )
    out = tmp_path / "sandbox_resources.log"

    def _once(*_a, **_k):
        raise SystemExit(0)

    monkeypatch.setattr(rs.time, "sleep", _once)
    with pytest.raises(SystemExit) as caught:
        rs.main(
            [
                "--loop",
                "--host",
                "sandbox",
                "--out",
                str(out),
                "--root",
                str(root),
                "--disk",
                "sda",
            ]
        )
    assert caught.value.code == 0
    assert "host=sandbox" in out.read_text(encoding="utf-8")
    assert "unrecognized arguments" not in out.read_text(encoding="utf-8")


def test_loop_flag_writes_from_command_line(tmp_path):
    log = tmp_path / "sampler.log"
    with subprocess.Popen(
        [sys.executable, str(REPEAT / "resource_sample.py"), "--loop", "--host", "test", "--out", str(log)],
        stderr=subprocess.PIPE,
        text=True,
    ) as process:
        try:
            for _ in range(40):
                if log.is_file() or process.poll() is not None:
                    break
                time.sleep(0.05)
        finally:
            if process.poll() is None:
                process.terminate()
            _, error = process.communicate(timeout=5)
    assert log.is_file(), error
    assert "host=test" in log.read_text(encoding="utf-8")


def test_early_exit_does_not_keep_foreign_goal_start(tmp_path):
    """Рядом старый route.log с GOAL_START. Ранний выход нового прогона его не копирует."""
    old = tmp_path / "shared" / "route.log"
    old.parent.mkdir()
    old.write_text("07:28:09 GOAL_START M1 target=(-3.7,4.0,0)\n", encoding="utf-8")
    art = tmp_path / "new_run"
    art.mkdir()
    # Как pull после M1_NOT_SENT: своего route нет — чужой не подставляем.
    own = art / "route.log"
    assert not own.exists()
    assert "GOAL_START" in old.read_text(encoding="utf-8")
    assert "GOAL_START" not in (own.read_text(encoding="utf-8") if own.exists() else "")
    script = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    assert "ROUTE_LOG=" in script
    assert "rm -f '$SANDBOX_REPO/out/route.log'" not in script


def test_stale_route_is_not_the_shared_out_log():
    """route.log прогона — файл run_id, не общий out/route.log прошлого заезда."""
    script = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    code = "\n".join(
        line for line in script.splitlines() if not line.lstrip().startswith("#")
    )
    assert (
        "cp '$SANDBOX_REPO/out/route.log' '$REMOTE_OUT/route.log'"
        not in code.split("GOAL_SEND")[0]
    )
    # После runvideo общий лог забираем, но только перезаписанный этим прогоном (mtime ≥ старта).
    after = code.split("GOAL_SEND")[1]
    assert "cp -f '$SANDBOX_REPO/out/route.log' '$REMOTE_OUT/route.log'" in after
    assert "stat -c %Y" in after
    assert "ROUTE_LOG_NOT_COPIED" in after
    assert "ROUTE_LOG=" in code
    assert "hil_on_attempt1.log" in code
    assert "hil_on_attempt2.log" in code
    assert "before_lock" in code
    assert "NAV_NOT_READY tf_odom_missing" in code
    assert "HIL_ON_TRY_" in code


def test_goal_env_selects_route_point():
    """M1_REPEAT_GOAL=R7: то же имя в runvideo.sh, GOAL_SEND и в grep GOAL_END."""
    script = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    assert 'GOAL="${M1_REPEAT_GOAL:-M1}"' in script
    assert "./runvideo.sh '$GOAL'" in script
    assert 'log "GOAL_SEND run_id=$RUN_ID goal=$GOAL' in script
    assert 'GOAL_END $GOAL result=' in script
    assert "^[MR][0-9]{1,2}$" in script


def test_shell_syntax_ok():
    subprocess.run(["bash", "-n", str(REPEAT / "run_m1_repeat.sh")], check=True)


def _fn_body(name: str) -> str:
    text = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    start = text.index(f"{name}() {{")
    rest = text[start:]
    # следующая функция верхнего уровня — строка без отступа, оканчивающаяся на ().
    lines = rest.splitlines(keepends=True)
    out = [lines[0]]
    for line in lines[1:]:
        if line and not line.startswith((" ", "\t", "\n")) and line.endswith("() {\n"):
            break
        out.append(line)
    return "".join(out)


def test_local_pull_copies_bag_without_scp_dot(tmp_path):
    """Локальная песочница: cp, не scp. bag/. в scp нет — OpenSSH 9 отвергает «.»."""
    remote = tmp_path / "remote" / "bag"
    remote.mkdir(parents=True)
    (remote / "metadata.yaml").write_text("rosbag2\n", encoding="utf-8")
    (remote / "bag_0.db3").write_bytes(b"bag")
    (tmp_path / "remote" / "route.log").write_text("GOAL_END\n", encoding="utf-8")
    art = tmp_path / "art"
    art.mkdir()
    harness = tmp_path / "copy.sh"
    harness.write_text(
        "set -uo pipefail\n"
        f"ART={art}\n"
        f"REMOTE_OUT={tmp_path / 'remote'}\n"
        "SANDBOX=ros2@127.0.0.1\n"
        + _fn_body("copy_bag_to_host")
        + _fn_body("pull_host_is_local")
        + "\ncopy_bag_to_host\nexit $?\n",
        encoding="utf-8",
        newline="\n",
    )
    result = subprocess.run(
        ["bash", str(harness)], capture_output=True, text=True, cwd=tmp_path
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert (art / "bag" / "metadata.yaml").read_text(encoding="utf-8") == "rosbag2\n"
    assert (art / "bag" / "bag_0.db3").read_bytes() == b"bag"
    assert "GOAL_END" in (art / "route.log").read_text(encoding="utf-8")
    assert "unexpected filename" not in (result.stderr + result.stdout)
    assert (
        not (art / "artifact_scp.err").exists()
        or (art / "artifact_scp.err").stat().st_size == 0
    )


def _cleanup_harness(
    tmp_path: Path, pull_rc: int, off_rc: int
) -> subprocess.CompletedProcess:
    """cleanup с подменой pull и hil off. Живой HIL не нужен."""
    art = tmp_path / "art"
    art.mkdir()
    off_stub = art / "off_stub.sh"
    off_stub.write_text(f"#!/bin/sh\nexit {off_rc}\n", encoding="utf-8")
    off_stub.chmod(0o755)
    harness = tmp_path / "cleanup.sh"
    harness.write_text(
        "set -uo pipefail\n"
        f"ART={art}\n"
        f"HIL_CMD={off_stub}\n"
        "SUMMARY=$ART/verdict.log\n"
        ': >"$SUMMARY"\n'
        'log() { printf \'%s\\n\' "$*" | tee -a "$SUMMARY"; }\n'
        "main() { return 0; }\n"
        "lock_is_ours() { return 0; }\n"
        "stop_route() { :; }\n"
        "stop_resource_samplers() { :; }\n"
        f"pull_before_off() {{ return {pull_rc}; }}\n"
        "CLEANED=0\n"
        "HIL_LOCK=/tmp/hil.lock\n"
        "ON_ATTEMPTED=1\n"
        "LOCK_OWNED=1\n" + _fn_body("cleanup") + "\ncleanup\nexit $?\n",
        encoding="utf-8",
        newline="\n",
    )
    return subprocess.run(
        ["bash", str(harness)], capture_output=True, text=True, cwd=tmp_path
    )


@pytest.mark.parametrize(
    ("found", "expected"),
    [
        ("HIL on since now token=0123456789abcdef0123456789abcdef maps=/tmp/run", 0),
        ("HIL on since now token=abcdef0123456789abcdef0123456789 maps=/tmp/run", 1),
        ("HIL on since now token=0123456789abcdef0123456789abcdef maps=/tmp/run_other", 1),
        ("HIL on since now maps=/tmp/run", 1),
        ("unrelated log mentions maps=/tmp/run", 1),
    ],
)
def test_hil_lock_ownership_requires_exact_token_and_map_path(found, expected):
    harness = (
        "set -uo pipefail\nMAPS_DIR=/tmp/run\n"
        "HIL_LOCK='/tmp/custom lock'\nHIL_RUN_TOKEN=0123456789abcdef0123456789abcdef\n"
        "main() { [[ \"$1\" == 'cat /tmp/custom\\ lock 2>/dev/null' ]] || return 9; printf '%s\\n' \"$LOCK_TEXT\"; }\n"
        + _fn_body("lock_is_ours")
        + "\nlock_is_ours\n"
    )
    result = subprocess.run(["bash", "-c", f'LOCK_TEXT="{found}"\n' + harness], capture_output=True, text=True)
    assert result.returncode == expected, result.stderr


@pytest.mark.parametrize(
    ("logging_status", "route_status", "expected"),
    [
        (1, "TIMEOUT", "GOAL_CHECK_LOGS_UNREADABLE"),
        (0, "STATUS_SUCCEEDED", "GOAL_CHECK_TRACE_MISSING"),
        (0, "OUTSIDE_TOLERANCE", "GOAL_CHECK_TRACE_MISSING"),
    ],
)
def test_goal_probe_incomplete_without_readable_trace(logging_status, route_status, expected, tmp_path):
    (tmp_path / "route.log").write_text(f"GOAL_END M1 result={route_status} sim=2.0\n", encoding="utf-8")
    harness = (
        "set -uo pipefail\n"
        'ART="$PWD"\nBAG_CLOSE_WAIT=1\nREMOTE_OUT=/test\nCONTAINER_OUT=/test\n'
        "G=gazebo\nNAV2_CONTAINER=nav2\nGOAL_PROBE_DIR=/test\nGOAL=M1\n"
        "sandbox() { return 0; }\ncopy_bag_to_host() { return 0; }\n"
        f'main() {{ printf "%s\\n" "controller log"; return {logging_status}; }}\n'
        'log() { printf "%s\\n" "$*"; }\n'
        + _fn_body("pull_before_off")
        + "\npull_before_off\n"
    )
    result = subprocess.run(["bash", "-c", harness], capture_output=True, text=True, cwd=tmp_path)
    assert result.returncode == 1, result.stderr
    assert expected in result.stdout


def test_goal_end_of_other_point_is_not_this_goals_reach(tmp_path):
    """GOAL=R7, а в route.log GOAL_END M1: чужой вердикт не выдаём за достижение цели."""
    (tmp_path / "route.log").write_text(
        "GOAL_END M1 result=STATUS_SUCCEEDED sim=2.0\n", encoding="utf-8"
    )
    harness = (
        "set -uo pipefail\n"
        'ART="$PWD"\nBAG_CLOSE_WAIT=1\nREMOTE_OUT=/test\nCONTAINER_OUT=/test\n'
        "G=gazebo\nNAV2_CONTAINER=nav2\nGOAL_PROBE_DIR=/test\nGOAL=R7\nHERE=/test\n"
        "sandbox() { return 0; }\ncopy_bag_to_host() { return 0; }\n"
        'main() { printf "%s\\n" "controller log"; return 0; }\n'
        'log() { printf "%s\\n" "$*"; }\n'
        + _fn_body("pull_before_off")
        + "\npull_before_off\n"
    )
    result = subprocess.run(["bash", "-c", harness], capture_output=True, text=True, cwd=tmp_path)
    assert result.returncode == 1, result.stderr
    assert "GOAL_CHECK_TRACE_NO_REACH" in result.stdout
    assert "GOAL_CHECK_TRACE_MISSING" not in result.stdout


def test_off_ok_and_missing_bag_are_different_codes(tmp_path):
    """off прошёл, bag не доехал — 52, не 50 и не маскарад CLEANUP_FAILED."""
    result = _cleanup_harness(tmp_path, pull_rc=1, off_rc=0)
    assert result.returncode == 52, result.stdout + result.stderr
    verdict = (tmp_path / "art" / "verdict.log").read_text(encoding="utf-8")
    assert "OFF_OK" in verdict
    assert "ARTIFACTS_INCOMPLETE" in verdict
    assert "OFF_FAILED" not in verdict
    assert "CLEANUP_FAILED" not in verdict


def test_off_failed_is_not_artifacts_code(tmp_path):
    result = _cleanup_harness(tmp_path, pull_rc=0, off_rc=1)
    assert result.returncode == 51, result.stdout + result.stderr
    verdict = (tmp_path / "art" / "verdict.log").read_text(encoding="utf-8")
    assert "OFF_FAILED rc=1" in verdict
    assert "ARTIFACTS_OK" in verdict
