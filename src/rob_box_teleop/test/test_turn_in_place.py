"""Разворот до свободного курса перед FollowPath (#4475). Поведение кода, не текст.

Факты HIL — docs/research/2026-11-05-nav2-hil-tuning.md §8 и §10, не новый замер:
робот лицом в перегородку, путь есть, прогресс 0; в тупике 1.1 м Spin на 180° падает
«Collision Ahead», потому что оборот вокруг задней оси не влезает. Зазор курса —
spin_stop_m защиты (0.10 м): нового порога тест не вводит.
"""

import math

import yaml

from rob_box_teleop import proximity as px
from rob_box_teleop.turn_in_place import REAR_AXLE_X_M, _swept_pose, facing_wall, free_spin
from rob_box_teleop.turn_then_follow import FOLLOW, SPIN, next_bt_node

BODY = px.Body(-0.3036, 0.3064, -0.3025, 0.3025)
CLEARANCE_M = px.Thresholds().spin_stop_m  # 0.10, решение Шифу 05.11
# Ближе порога — «нос в стене». Дальше — курс уже свободный, Spin не нужен.
WALL_AHEAD_M = 0.05
OPEN_AHEAD_M = 0.40
# §10: боковые стены тупика 1.1 м на ±0.55. Угол корпуса ~0.43 м, оборот задевает стену.
DEAD_END_HALF_M = 0.55


def _ahead(gap_m, samples=4):
    """Перегородка в коридоре хода: уже корпуса, чтобы разворот уводил нос мимо неё.

    Стена на всю ширину корпуса пересекается углом на любом повороте — это не проём,
    а торец без щели. Проём 0.90 м оставляет щель сбоку; здесь щель — сама геометрия
    узкой перегородки перед носом.
    """
    span = 0.05
    return tuple(
        (BODY.x_max + gap_m, -span + 2.0 * span * i / (samples - 1)) for i in range(samples)
    )


def _sides(half_m=DEAD_END_HALF_M):
    xs = (-0.3, -0.1, 0.1, 0.3)
    return tuple((x, y) for x in xs for y in (half_m, -half_m))


def test_facing_partition_with_path_spins_before_follow():
    """Путь есть, прогресс 0, нос в перегородке, вбок свободно — сначала Spin на курс пути."""
    pose = (0.0, 0.0, 0.0)
    scan = _ahead(WALL_AHEAD_M)
    path = [(0.0, 0.0), (0.2, 0.6), (1.2, 0.6)]
    assert facing_wall(BODY, pose, scan, CLEARANCE_M)
    node, spin = next_bt_node(BODY, pose, scan, path, CLEARANCE_M, progress_m=0.0)
    assert node == SPIN
    assert spin == free_spin(BODY, pose, scan, path, CLEARANCE_M)
    assert spin is not None and 0.0 < abs(spin) < math.pi
    assert not facing_wall(BODY, _swept_pose(pose, spin, REAR_AXLE_X_M), scan, CLEARANCE_M)


def test_already_clear_heading_does_not_spin():
    """Нос не в стене (подъезд по оси, §8) — сразу FollowPath, даже если путь ещё впереди."""
    pose = (0.0, 0.0, 0.0)
    scan = _ahead(OPEN_AHEAD_M)
    path = [(0.5, 0.0), (1.5, 0.0)]
    assert not facing_wall(BODY, pose, scan, CLEARANCE_M)
    assert next_bt_node(BODY, pose, scan, path, CLEARANCE_M, progress_m=0.0) == (FOLLOW, None)


def test_moving_robot_is_not_preempted_by_spin():
    """Прогресс есть или корпус уже едет — разворот поверх FollowPath не вставляется."""
    pose = (0.0, 0.0, 0.0)
    scan = _ahead(WALL_AHEAD_M)
    path = [(0.2, 0.6), (1.2, 0.6)]
    assert next_bt_node(BODY, pose, scan, path, CLEARANCE_M, progress_m=0.08) == (FOLLOW, None)
    assert next_bt_node(BODY, pose, scan, path, CLEARANCE_M, progress_m=0.0, vx=0.05) == (FOLLOW, None)
    assert next_bt_node(BODY, pose, scan, path, CLEARANCE_M, progress_m=0.0, wz=0.2) == (FOLLOW, None)


def test_no_path_does_not_insert_spin():
    """Пути нет — отказ планировщика, не повод крутиться у стены."""
    pose = (0.0, 0.0, 0.0)
    node, spin = next_bt_node(BODY, pose, _ahead(WALL_AHEAD_M), path=[], clearance_m=CLEARANCE_M, progress_m=0.0)
    assert (node, spin) == (FOLLOW, None)


def test_dead_end_turn_that_hits_wall_is_not_commanded():
    """§10: дуга до курса пути задевает боковую стену тупика — Spin не шлётся.

    Фиксированные 1.57 рад recovery как раз так и падали «Collision Ahead»: угол корпуса
    от базы ~0.43 м, стена на 0.55 м, оборот её пересекает. Код возвращает None, а не
    этот угол. Точек на стене много: редкая выборка оставляла щель, которой в тупике нет.
    """
    pose = (0.0, 0.0, 0.0)
    # Торец на всю ширину тупика и боковые стены: щели, в которую можно докрутить, нет.
    front = tuple((BODY.x_max + WALL_AHEAD_M, -DEAD_END_HALF_M + 1.1 * i / 40) for i in range(41))
    xs = [i * 0.05 for i in range(-20, 8)]
    sides = tuple((x, y) for x in xs for y in (DEAD_END_HALF_M, -DEAD_END_HALF_M))
    scan = front + sides
    path = [(0.15, 0.7)]  # щель планировщика сбоку, по корпусу её нет
    assert facing_wall(BODY, pose, scan, CLEARANCE_M)
    assert free_spin(BODY, pose, scan, path, CLEARANCE_M, pivot_x=REAR_AXLE_X_M) is None
    assert next_bt_node(BODY, pose, scan, path, CLEARANCE_M, progress_m=0.0) == (FOLLOW, None)


def test_short_turn_out_of_dead_end_when_arc_stays_clear():
    """Выезд: путь уходит в свободный бок, контур стену не пересекает — короткий Spin, не 180°."""
    pose = (0.0, 0.0, 0.0)
    scan = _ahead(WALL_AHEAD_M) + _sides(half_m=1.5)  # стены далеко, короткая дуга свободна
    path = [(0.0, 0.0), (0.15, 0.8), (1.0, 0.8)]
    node, spin = next_bt_node(BODY, pose, scan, path, CLEARANCE_M, progress_m=0.0)
    assert node == SPIN
    assert spin is not None and 0.0 < abs(spin) < math.pi / 2


def test_section8_opening_facing_partition_spins_toward_the_gap():
    """§8: робот в 0.76 м от проёма 0.90 лицом в перегородку, путь уходит в щель.

    Числа — из отчёта, не новый замер: проём 0.90 м (косяки на ±0.45), зазор носа
    до перегородки меньше spin_stop_m. Код заказывает короткий Spin в сторону пути
    и не заказывает фиксированные 1.57 рад.
    """
    pose = (0.0, 0.0, 0.0)
    # Перегородка ближе порога и только в коридоре хода: щель проёма — сбоку от носа.
    partition = tuple((BODY.x_max + 0.05, -0.08 + 0.16 * i / 5) for i in range(6))
    pose_path = [(0.2, 0.55), (1.1, 0.0)]
    assert facing_wall(BODY, pose, partition, CLEARANCE_M)
    spin = free_spin(BODY, pose, partition, pose_path, CLEARANCE_M, pivot_x=REAR_AXLE_X_M)
    assert spin is not None and 0.0 < spin < math.pi / 2
    assert abs(spin - 1.57) > 0.2
    node, commanded = next_bt_node(BODY, pose, partition, pose_path, CLEARANCE_M, progress_m=0.0)
    assert (node, commanded) == (SPIN, spin)


def test_one_ghost_return_does_not_count_as_a_wall():
    """Один луч-призрак в коридоре хода — не стена (тот же приём, что min_points защиты)."""
    pose = (0.0, 0.0, 0.0)
    ghost = ((BODY.x_max + WALL_AHEAD_M, 0.0),)
    assert not facing_wall(BODY, pose, ghost, CLEARANCE_M)
    assert free_spin(BODY, pose, ghost, [(0.2, 0.6)], CLEARANCE_M) is None


def test_base_frame_scan_uses_zero_yaw_not_map_heading():
    """Скан в base_footprint — поза (0,0,0). Map yaw π прячет стену за спину."""
    scan = _ahead(WALL_AHEAD_M)
    path = [(0.2, 0.6), (1.2, 0.6)]
    assert facing_wall(BODY, (0.0, 0.0, 0.0), scan, CLEARANCE_M)
    assert not facing_wall(BODY, (0.0, 0.0, math.pi), scan, CLEARANCE_M)
    assert free_spin(BODY, (0.0, 0.0, 0.0), scan, path, CLEARANCE_M) is not None
    assert free_spin(BODY, (0.0, 0.0, math.pi), scan, path, CLEARANCE_M) is None


def test_clearance_is_the_guard_spin_stop_not_a_new_threshold():
    """Порог зазора — spin_stop_m защиты. Боевой yaml не ослабляет габарит и не трогает PathAlign."""
    from pathlib import Path

    repo = Path(__file__).resolve().parents[3]
    doc = yaml.safe_load((repo / "docker/main/config/nav2/nav2_params.yaml").read_text(encoding="utf-8"))
    local = doc["local_costmap"]["local_costmap"]["ros__parameters"]
    assert local["footprint_padding"] == 0.03
    follow = doc["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["PathAlign.forward_point_distance"] == 0.20
    planner = doc["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["allow_unknown"] is True
    assert CLEARANCE_M == 0.10
