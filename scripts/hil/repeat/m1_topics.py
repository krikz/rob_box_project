#!/usr/bin/env python3
"""Список топиков и фиксация условий повторного одиночного M1 (#4694).

Прогон 18.11 (/tmp/hil_m1_stall) писал bag без ``/cmd_vel_yaw`` и без global
costmap: yaw был только тиком в route.log, выход контроллера до mux не
отличить от ``/cmd_vel``. Этот модуль — единственный список топиков и проверка
файлов фиксации. Живой HIL отсюда не стартует.

Имена топиков — из конфига, не выдумка:
  * ``/cmd_vel_yaw`` — twist_mux yaw_at_goal (docker/main/config/twist_mux/twist_mux.yaml);
  * ``/cmd_vel_nav`` — выход controller_server до velocity_smoother и до mux
    (docker/main/scripts/nav2/start_nav2_direct.sh, NAV2_SMOOTH_CMD_VEL=true);
  * ``/global_costmap/costmap_raw`` — сырой global costmap (nav2_costmap_2d,
    always_send_full_costmap); ``/global_costmap/costmap`` — тот же слой после
    inflation (src/rob_box_sim/rob_box_sim/nav_bench.py).
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Iterable, List, Mapping, Sequence, Tuple

# База прошлого прогона: то, что уже было в bag_0.db3 /tmp/hil_m1_stall.
BASE_BAG_TOPICS: Tuple[str, ...] = (
    "/cmd_vel",
    "/diff_drive_controller/cmd_vel_unstamped",
    "/local_costmap/costmap_raw",
    "/local_plan",
    "/plan",
    "/scan",
    "/odom",
    "/sim/ground_truth/odom",
    "/tf",
    "/tf_static",
)

# Чего не хватило разбору срыва у цели (#4694, комментарии Тигра #4682).
# /rosout — «Failed to make progress», тип recovery, защёлка RotateToGoal.
# /evaluation — dwb_msgs/LocalPlanEvaluation (внутри CriticScore[]), не голый CriticScore.
# /trajectories — MarkerArray образцов DWB (create_publisher "marker" → /trajectories).
# /local_plan уже в базе; писать каждый тик, не прореживать.
REPEAT_EXTRA_TOPICS: Tuple[str, ...] = (
    "/cmd_vel_yaw",
    "/cmd_vel_nav",
    "/global_costmap/costmap",
    "/global_costmap/costmap_raw",
    "/rosout",
    "/evaluation",
    "/trajectories",
)

# Типы — из dwb_core 1.1.20 publisher.cpp и dwb_msgs. Не выдумка.
EVALUATION_TOPIC = "/evaluation"
EVALUATION_TYPE = "dwb_msgs/msg/LocalPlanEvaluation"
TRAJECTORIES_TOPIC = "/trajectories"
TRAJECTORIES_TYPE = "visualization_msgs/msg/MarkerArray"
LOCAL_PLAN_TOPIC = "/local_plan"

# HIL-оверлей только включает публикацию оценок. Скорости, critics, lookahead не трогает.
# Боевой nav2_params.yaml этих ключей не задаёт; дефолт DWB 1.1.20 — true, но повтор
# фиксирует факт параметра, а не надеется на дефолт. Публикация оценок выбор траектории
# не меняет: scoreTrajectory считается до publishEvaluation.
DWB_PUBLISH_OVERLAY_KEYS: Tuple[str, ...] = (
    "FollowPath.publish_evaluation",
    "FollowPath.publish_trajectories",
    "FollowPath.publish_local_plan",
)

# Выход контроллера до mux: cmd_vel_nav (до smoother), не /cmd_vel (вход mux).
CONTROLLER_BEFORE_MUX = "/cmd_vel_nav"
YAW_TOPIC = "/cmd_vel_yaw"
GLOBAL_COSTMAP_TOPICS: Tuple[str, ...] = (
    "/global_costmap/costmap",
    "/global_costmap/costmap_raw",
)

# Сопоставимость с прогоном 18.11 (бриф Шифу 18.11 16:42 UTC).
M1_GOAL = (-3.7, 4.0, 0.0)  # x, y, yaw град — route.py GOALS M1
SPAWN = (0.0, 0.0, 0.0)
EXPECT_WORLD = "depot"
MAX_VEL_X = 0.15
PATHALIGN_FORWARD_M = 0.20
WORLD_ARG = "world:=/sim_video/world.sdf"

# Файлы, которые сценарий обязан положить на хост ДО отправки цели.
BEFORE_GOAL_FILES: Tuple[str, ...] = (
    "sdf.sha256",
    "images.txt",
    "params_overlays.txt",
    "start_map_gt.txt",
    "gazebo_args.txt",
    "gazebo_mounts.txt",
    "world_check.txt",
)

# ros2 param get /controller_server <name>. Имя в файле обязательно (Тигр: безымянные
# «Double value is» не доказывают, какой параметр есть какой).
NAMED_PARAMS: Tuple[str, ...] = (
    "FollowPath.max_vel_x",
    "FollowPath.PathAlign.forward_point_distance",
    "progress_checker.movement_time_allowance",
    "FollowPath.min_speed_xy",
    "FollowPath.trans_stopped_velocity",
    "general_goal_checker.xy_goal_tolerance",
    "FollowPath.xy_goal_tolerance",
    "FollowPath.RotateToGoal.scale",
    "FollowPath.RotateToGoal.lookahead_time",
    "FollowPath.Oscillation.scale",
    "FollowPath.Oscillation.oscillation_reset_dist",
    "FollowPath.Oscillation.oscillation_reset_angle",
    "FollowPath.Oscillation.oscillation_reset_time",
)

# Вход twist DWB: controller_server своего odom_topic не задаёт, читает /odom
# (bt_navigator.odom_topic: /odom, nav2_params.yaml:10). Не /sim/ground_truth/odom.
DWB_ODOM_TOPIC = "/odom"

# Файлы, которые должны быть на хосте ДО hil off (bag закрыт, route скопирован).
BEFORE_OFF_FILES: Tuple[str, ...] = BEFORE_GOAL_FILES + (
    "route.log",
    "bag_info.txt",
)


def repeat_bag_topics(extra: Sequence[str] | None = None) -> List[str]:
    """Топики ``ros2 bag record`` повторного M1: база прошлого bag плюс недостающие.

    Порядок стабильный, дубликаты выкидываются. ``extra`` — если координатор
    добавит топик в ту же запись, не правя список руками в shell.
    """
    seen = set()
    out: List[str] = []
    for name in (*BASE_BAG_TOPICS, *REPEAT_EXTRA_TOPICS, *(extra or ())):
        if name not in seen:
            seen.add(name)
            out.append(name)
    return out


def bag_qos_overrides_yaml(topics: Sequence[str] | None = None) -> str:
    """QoS подписки bag. DWB публикует /evaluation depth 1, reliable.

    ``ros2 bag record`` без оверлея подписывается best-effort и теряет тики
    (пара run1: 6 с дыры, защёлка встала в дырке). reliability 1 = reliable.
    """
    names = list(topics) if topics is not None else repeat_bag_topics()
    lines = []
    for name in names:
        lines.append(f"{name}:")
        lines.append("  reliability: reliable")
        lines.append("  history: keep_last")
        lines.append("  depth: 10")
    return "\n".join(lines) + "\n"


def record_argv(out_dir: str, topics: Sequence[str] | None = None) -> List[str]:
    """Аргументы ``ros2 bag record -o <out> <topics...>``. Без живого ros2."""
    names = list(topics) if topics is not None else repeat_bag_topics()
    missing = [t for t in REPEAT_EXTRA_TOPICS if t not in names]
    if missing:
        raise ValueError("repeat bag missing required topics: " + " ".join(missing))
    return ["ros2", "bag", "record", "-o", out_dir, *names]


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _pose_line(label: str, pose: Mapping[str, float] | None) -> str:
    """``label x=… y=… yaw=…`` или ``label none``. Нет ключа — none, ноль не подставляем."""
    if pose is None:
        return f"{label} none"
    try:
        return f"{label} x={float(pose['x']):.6f} y={float(pose['y']):.6f} yaw={float(pose['yaw']):.6f}"
    except (KeyError, TypeError, ValueError):
        return f"{label} none"


def format_start_map_gt(map_pose: Mapping[str, float] | None, gt_pose: Mapping[str, float] | None) -> str:
    """Две строки старта map+GT до цели. Нет измерения — ``none``, не выдуманный ноль."""
    return _pose_line("map", map_pose) + "\n" + _pose_line("gt", gt_pose) + "\n"


_FLOAT_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def parse_tf2_echo(text: str) -> dict | None:
    """Первый кадр ``tf2_echo``: Translation и yaw из RPY. Нет полного кадра — None."""
    import re

    tr = re.search(
        rf"Translation:\s*\[({_FLOAT_RE}),\s*({_FLOAT_RE}),\s*{_FLOAT_RE}\]",
        text or "",
    )
    rpy = re.search(
        rf"in RPY \(radian\)\s*\[{_FLOAT_RE},\s*{_FLOAT_RE},\s*({_FLOAT_RE})\]",
        text or "",
    )
    if not tr or not rpy:
        return None
    return {"x": float(tr.group(1)), "y": float(tr.group(2)), "yaw": float(rpy.group(1))}


def parse_ground_truth_echo(text: str) -> dict | None:
    """Первый ``ros2 topic echo /sim/ground_truth/odom``: position + quaternion → yaw рад."""
    import math
    import re

    pos = re.search(
        rf"position:\s*\n\s*x:\s*({_FLOAT_RE})\s*\n\s*y:\s*({_FLOAT_RE})\s*\n\s*z:\s*{_FLOAT_RE}",
        text or "",
    )
    quat = {}
    block = re.search(r"orientation:\s*\n((?:\s*[xyzw]:\s*" + _FLOAT_RE + r"\s*\n){4})", text or "")
    if pos is None or block is None:
        return None
    for key, value in re.findall(rf"([xyzw]):\s*({_FLOAT_RE})", block.group(1)):
        quat[key] = float(value)
    if set(quat) != {"x", "y", "z", "w"}:
        return None
    yaw = math.atan2(
        2 * (quat["w"] * quat["z"] + quat["x"] * quat["y"]),
        1 - 2 * (quat["y"] ** 2 + quat["z"] ** 2),
    )
    return {"x": float(pos.group(1)), "y": float(pos.group(2)), "yaw": yaw}


def write_start_map_gt(directory: str | Path, map_text: str, gt_text: str) -> str:
    """Разобрать сырьё tf2_echo и GT echo в ``start_map_gt.txt``. Возвращает текст файла."""
    body = format_start_map_gt(parse_tf2_echo(map_text), parse_ground_truth_echo(gt_text))
    path = Path(directory) / "start_map_gt.txt"
    path.write_text(body, encoding="utf-8")
    return body


_VALUE_PREFIXES = (
    "Double value is: ",
    "Integer value is: ",
    "String value is: ",
    "Boolean value is: ",
)


def parse_param_value(raw: str) -> str | None:
    """Число или bool из ответа ``ros2 param get``. Баннер без ``value is:`` — None.

    Ночной прогон 19.11: stdout ``ros_with_namespace.sh`` (строки ``====``) попал в
    значение, а ``Double value is`` следующего вызова приклеился к чужому имени.
    Берём только строку ``value is:``, не последнюю строку баннера.
    """
    for line in (raw or "").splitlines():
        text = line.strip()
        for prefix in _VALUE_PREFIXES:
            if text.startswith(prefix):
                value = text[len(prefix):].strip()
                return value or None
    return None


def format_named_param(name: str, raw: str) -> str:
    """``<имя>=<значение>`` или ``<имя>=missing``. Баннер без числа — missing, не строка ``====``."""
    value = parse_param_value(raw)
    if value is None:
        return f"{name}=missing"
    return f"{name}={value}"


def format_params_file(named: Sequence[Tuple[str, str]], overlays: Iterable[str] = ()) -> str:
    """params.txt: каждая строка с именем параметра. Нет ответа — ``name=missing``."""
    lines = [format_named_param(name, raw) for name, raw in named]
    for required in NAMED_PARAMS:
        if not any(line.startswith(required + "=") for line in lines):
            lines.append(f"{required}=missing")
    lines.extend(
        [
            f"goal=({M1_GOAL[0]},{M1_GOAL[1]},{M1_GOAL[2]})",
            f"spawn=({SPAWN[0]},{SPAWN[1]},{SPAWN[2]})",
            f"expect_world={EXPECT_WORLD}",
            f"world_arg={WORLD_ARG}",
        ]
    )
    ov = [o for o in overlays if o]
    lines.append("overlays=" + (" ".join(ov) if ov else "none"))
    return "\n".join(lines) + "\n"


def params_missing_names(text: str) -> List[str]:
    """Имена из NAMED_PARAMS, которых нет в params.txt или значение пустое/missing."""
    found = {}
    for line in (text or "").splitlines():
        if "=" not in line:
            continue
        name, _, value = line.partition("=")
        found[name.strip()] = value.strip()
    missing = []
    for name in NAMED_PARAMS:
        value = found.get(name, "")
        if not value or value == "missing":
            missing.append(name)
    return missing


# Строки Nav2, по которым повтор различает тип recovery (Тигр: clear/spin/wait/back_up).
RECOVERY_MARKERS: Tuple[Tuple[str, str], ...] = (
    ("progress_fail", "Failed to make progress"),
    ("clear_global", "Clearing global costmap"),
    ("clear_local", "Clearing local costmap"),
    ("clear_costmap", "ClearEntireCostmap"),
    ("spin", "spin"),
    ("wait", "wait"),
    ("back_up", "Backing up"),
    ("backup", "backup"),
    # Защёлка RotateToGoal (rotate_to_goal.cpp:116 и :109). Момент — по /rosout, не по догадке.
    ("rotate_nonrotation", "Nonrotation command near goal"),
    ("rotate_not_slowing", "Not slowing down near goal"),
)


def trans_stopped_from_env(value: str | None) -> float | None:
    """``M1_REPEAT_TRANS_STOPPED``: пусто — боевой 0.03, строку не пишем."""
    return positive_float_from_env("M1_REPEAT_TRANS_STOPPED", value)


def xy_goal_tol_from_env(value: str | None) -> float | None:
    """``M1_REPEAT_XY_GOAL_TOL``: пусто — боевой 0.25, строку не пишем.

    Число > 0 — только ``FollowPath.xy_goal_tolerance`` (окно DWB), не checker.
    """
    return positive_float_from_env("M1_REPEAT_XY_GOAL_TOL", value)


def positive_float_from_env(name: str, value: str | None) -> float | None:
    """Пусто — параметр не трогаем. Число > 0 — в оверлей. Иначе ошибка, не боевой дефолт."""
    if value is None or not str(value).strip():
        return None
    try:
        number = float(str(value).strip())
    except ValueError as exc:
        raise ValueError(f"{name}={value!r} — нужно число > 0") from exc
    if not number > 0:
        raise ValueError(f"{name}={value!r} — нужно число > 0")
    return number


def dwb_publish_overlay_yaml(
    short_circuit: bool = True,
    trans_stopped: float | None = None,
    xy_goal_tolerance: float | None = None,
) -> str:
    """Оверлей Nav2 только для HIL-повтора. Боевой файл не меняет, скорости не задаёт.

    Публикация ``/evaluation`` не входит в выбор траектории. ``short_circuit=False``
    добавляет ``FollowPath.short_circuit_trajectory_evaluation: false`` — DWB дописывает
    оценки всех критиков, лучшую траекторию не перевыбирает (обрыв только прячет хвост
    scores). Боевое поведение — ``True``: строку не пишем, остаётся yaml (true).
    """
    lines = [
        "# HIL M1-repeat only. Не боевой nav2_params.yaml.",
        "# Публикация оценок DWB. Скорость, critics, lookahead не трогает.",
    ]
    if not short_circuit:
        lines.append("# short_circuit false: полный список оценок, выбор траектории тот же.")
    lines.extend(
        [
            # Вложенность как в nav2_params.yaml: controller_server.ros__parameters.FollowPath.
            # Плоский FollowPath: под controller_server не перебивает вложенный ключ
            # (пара run1: yaml false, param get True).
            "controller_server:",
            "  ros__parameters:",
            "    FollowPath:",
            "      ros__parameters:",
            "        publish_evaluation: true",
            "        publish_trajectories: true",
            "        publish_local_plan: true",
        ]
    )
    if not short_circuit:
        lines.append("        short_circuit_trajectory_evaluation: false")
    if trans_stopped is not None:
        # Не боевой 0.03: порог, ниже которого RotateToGoal считает корпус стоящим.
        lines.append(f"        trans_stopped_velocity: {trans_stopped}")
    if xy_goal_tolerance is not None:
        # Только внутреннее окно DWB (RotateToGoal.in_window_). general_goal_checker
        # и GT-приёмка route.py остаются 0.25.
        lines.append(f"        xy_goal_tolerance: {xy_goal_tolerance}")
    return "\n".join(lines) + "\n"


def short_circuit_from_env(value: str | None) -> bool:
    """``M1_REPEAT_SHORT_CIRCUIT``: пусто и true — боевой обрыв; false — полный список оценок.

    Иное значение — ошибка, не молчаливый true: диагностический прогон нельзя перепутать.
    """
    text = (value if value is not None else "true").strip().lower()
    if text in ("", "true", "1", "yes"):
        return True
    if text in ("false", "0", "no"):
        return False
    raise ValueError(f"M1_REPEAT_SHORT_CIRCUIT={value!r} — только true или false")


def recovery_hits(log_text: str) -> List[str]:
    """Какие маркеры recovery встретились в логе nav2. Нет строки — маркера нет, не выдумываем тип."""
    text = log_text or ""
    hits = []
    lowered = text.lower()
    for key, needle in RECOVERY_MARKERS:
        if needle.lower() in lowered:
            hits.append(key)
    return hits


def missing_artifacts(directory: str | Path, required: Sequence[str]) -> List[str]:
    """Имена обязательных файлов, которых нет или они пустые."""
    root = Path(directory)
    missing: List[str] = []
    for name in required:
        path = root / name
        if not path.is_file() or path.stat().st_size == 0:
            missing.append(name)
    return missing


def assert_ready(directory: str | Path, phase: str) -> None:
    """Бросить, если фиксация фазы неполная. phase: before_goal | before_off."""
    required = {"before_goal": BEFORE_GOAL_FILES, "before_off": BEFORE_OFF_FILES}.get(phase)
    if required is None:
        raise ValueError(f"unknown phase {phase!r}")
    missing = missing_artifacts(directory, required)
    if missing:
        raise FileNotFoundError(f"{phase}: missing or empty " + " ".join(missing))
