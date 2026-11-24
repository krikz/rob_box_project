"""Критерий отказа «упёрся в препятствие» (#5003).

Старое поведение SimpleProgressChecker (только радиус 0.08 м за 30 с) на этих
входах отказ не ставит: центр за окно сместился на 0.02 м, а вдоль стены —
на 0.9 м. Новый критерий обязан отказать только в первом случае.
"""

import math

import pytest

from rob_box_teleop.proximity import Body
from rob_box_teleop.stuck_against import (
    BLOCK_GAP_M,
    CMD_EPS,
    CMD_HOLD_S,
    INSCRIBED_COST,
    LETHAL_COST,
    PROGRESS_RADIUS_M,
    STUCK_WINDOW_S,
    StuckAgainst,
    command_is_motion,
    course_blocked,
)

# Габарит РОББОКСа (замер 04.11, nav2_params.yaml local_costmap.footprint).
BODY = Body(-0.3036, 0.3064, -0.3025, 0.3025)
# Локальная карта: 6 м, клетка 0.05 м, робот в центре. Как local_costmap.
RES = 0.05
SIZE = 120
ORIGIN = -3.0

# Измерено Gate 2 21.11: вдоль стены Δy ≈ +0.85 / +0.95 м за окно чистки.
WALL_SLIDE_M = 0.90


def _grid(cells: dict[tuple[int, int], int] | None = None) -> list[int]:
    costs = [0] * (SIZE * SIZE)
    for (mx, my), cost in (cells or {}).items():
        costs[my * SIZE + mx] = cost
    return costs


def _cell(x: float, y: float) -> tuple[int, int]:
    return math.floor((x - ORIGIN) / RES), math.floor((y - ORIGIN) / RES)


def _tick(
    checker: StuckAgainst,
    seconds: float,
    pose: tuple[float, float, float],
    commanded: bool,
) -> bool:
    """Тики по 0.05 с (20 Гц контроллера) до ``seconds`` включительно."""
    refused = False
    steps = int(round(seconds / 0.05))
    for i in range(steps + 1):
        refused = checker.update(i * 0.05, pose, commanded)
    return refused


def test_old_radius_alone_does_not_refuse_a_two_centimetre_creep():
    """Без нового признака 0.02 м < 0.08 м за 2 с — это ещё не отказ радиуса.

    Отказ радиуса Humble копится 30 с. Здесь окно 2 с, так что один радиус
    отказ не ставит: нужен признак «упёрся».
    """
    assert PROGRESS_RADIUS_M == pytest.approx(0.08)
    assert math.hypot(0.02, 0.0) < PROGRESS_RADIUS_M
    assert STUCK_WINDOW_S < 30.0


def test_command_without_motion_against_wall_is_a_progress_failure():
    """Команда есть, смещения нет, впереди стена → отказ прогресса сразу."""
    checker = StuckAgainst()
    assert _tick(checker, STUCK_WINDOW_S, (0.0, 0.0, 0.0), commanded=True)

    nose = BODY.x_max + BLOCK_GAP_M / 2
    wall = _cell(nose, 0.0)
    assert course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({wall: LETHAL_COST}), vx=0.1)
    assert checker.note_blocked(True)


def test_slide_along_wall_is_progress_not_a_failure():
    """Команда есть, Δy = 0.90 м за окно, стена сбоку → прогресс, не отказ.

    0.90 м — измеренный порядок Gate 2 21.11 (Δy ≈ +0.85 / +0.95 м).
    """
    checker = StuckAgainst()
    steps = int(round(STUCK_WINDOW_S / 0.05))
    refused = False
    for i in range(steps + 1):
        y = WALL_SLIDE_M * i / steps
        refused = checker.update(i * 0.05, (0.0, y, 0.0), commanded=True)
    assert not refused
    assert WALL_SLIDE_M > PROGRESS_RADIUS_M

    side = _cell(0.0, -(BODY.y_min - BLOCK_GAP_M / 2))
    # Стена слева по борту, не по курсу: курс вперёд пуст.
    assert not course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({side: LETHAL_COST}), vx=0.1)


def test_irregular_ticks_refuse_as_soon_as_the_full_window_is_observed():
    checker = StuckAgainst()
    elapsed_ms = 0
    for tick in range(80):
        refused = checker.update((145013 + elapsed_ms) / 1000, (0.0, 0.0, 0.0), commanded=True)
        assert refused == (elapsed_ms >= 2000), elapsed_ms
        elapsed_ms += (47, 61)[tick % 2]


def test_gap_in_observations_starts_a_new_window():
    checker = StuckAgainst()
    pose = (0.0, 0.0, 0.0)
    _tick(checker, 1.0, pose, commanded=True)
    for tick in range(41):
        assert checker.update(2.0 + tick / 20, pose, commanded=True) == (tick == 40)


def test_held_command_just_before_cutoff_is_only_a_pose_anchor():
    checker = StuckAgainst()
    pose = (0.0, 0.0, 0.0)
    for tick in range(49):
        checker.update(tick / 20, pose, commanded=tick <= 8)
    assert checker.note_blocked(True)  # Last held command at 0.4, cutoff still 0.4.
    assert not checker.update(2.41, pose, commanded=False)


def test_no_command_against_wall_is_not_a_failure():
    """Стена впереди, но команды не было — это стоянка, не упор."""
    checker = StuckAgainst()
    assert not _tick(checker, STUCK_WINDOW_S, (0.0, 0.0, 0.0), commanded=False)
    assert not checker.note_blocked(True)


def test_command_without_obstacle_is_not_a_failure():
    """Команда есть, смещения нет, но по курсу пусто — отказ не ставим.

    Иначе нулевой ход на ровном месте (колёса ещё не стронулись) дал бы
    ложный BackUp.
    """
    checker = StuckAgainst()
    assert _tick(checker, STUCK_WINDOW_S, (0.0, 0.0, 0.0), commanded=True)
    assert not course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid(), vx=0.1)
    assert not checker.note_blocked(False)


def test_creep_above_radius_is_progress_even_against_a_wall():
    """Сдвиг больше радиуса за окно — прогресс, стена впереди не отказывает."""
    checker = StuckAgainst()
    steps = int(round(STUCK_WINDOW_S / 0.05))
    refused = False
    for i in range(steps + 1):
        x = (PROGRESS_RADIUS_M + 0.01) * i / steps
        refused = checker.update(i * 0.05, (x, 0.0, 0.0), commanded=True)
    assert not refused


def test_short_command_blip_does_not_count_as_command():
    """Команда короче cmd_hold — не «команда была»: одиночный тик не упор."""
    checker = StuckAgainst()
    refused = False
    steps = int(round(STUCK_WINDOW_S / 0.05))
    blip = int(round((CMD_HOLD_S / 2) / 0.05))
    for i in range(steps + 1):
        refused = checker.update(i * 0.05, (0.0, 0.0, 0.0), commanded=i < blip)
    assert not refused


def test_window_shorter_than_allowance_does_not_refuse():
    checker = StuckAgainst()
    assert not _tick(checker, STUCK_WINDOW_S / 2, (0.0, 0.0, 0.0), commanded=True)


def test_course_blocked_sees_wall_ahead_and_behind():
    ahead = _cell(BODY.x_max + 0.04, 0.0)
    assert course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({ahead: INSCRIBED_COST}), vx=0.1)
    behind = _cell(BODY.x_min - 0.04, 0.0)
    assert course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({behind: LETHAL_COST}), vx=-0.1)
    # Вперёд стена сзади не мешает, и наоборот.
    assert not course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({behind: LETHAL_COST}), vx=0.1)


def test_inflation_beyond_gap_is_not_a_block():
    """Клетка дальше порога касания — не упор, даже если lethal."""
    far = _cell(BODY.x_max + BLOCK_GAP_M + RES, 0.0)
    assert not course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({far: LETHAL_COST}), vx=0.1)


def test_free_cell_and_zero_command_are_not_a_block():
    cell = _cell(BODY.x_max + 0.04, 0.0)
    assert not course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({cell: 200}), vx=0.1)
    assert not course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({cell: LETHAL_COST}), vx=0.0)


def test_command_is_motion_uses_guard_epsilon():
    assert CMD_EPS == pytest.approx(0.03)
    assert not command_is_motion(0.0, 0.0)
    assert not command_is_motion(CMD_EPS, 0.0)
    assert command_is_motion(CMD_EPS + 0.001, 0.0)
    assert command_is_motion(0.0, CMD_EPS + 0.001)


def test_reset_drops_the_window():
    checker = StuckAgainst()
    _tick(checker, STUCK_WINDOW_S, (0.0, 0.0, 0.0), commanded=True)
    assert checker.refused()
    checker.reset()
    assert not checker.refused()


def test_dwb_stops_commanding_at_a_head_on_wall_and_still_refuses():
    """Лобовой упор: DWB перестал командовать, смещения нет, стена по курсу.

    По коду Humble (nav2_dwb_controller dwb_local_planner.cpp, coreScoringAlgorithm):
    если все траектории illegal, DWB бросает NoLegalTrajectoriesException и
    twist не публикует. controller_server в catch пишет нулевой twist, пока
    не вышел failure_tolerance (0.3 с), и только потом абортит сам. Если
    требовать команду на этом тике, отказ не засчитается в целевом случае.
    Здесь команда > 0.03 была, затем ноль, центр стоит, впереди lethal.
    """
    from rob_box_teleop.stuck_against import controller_stuck, reset_controller_state

    reset_controller_state()
    wall = _cell(BODY.x_max + BLOCK_GAP_M / 2, 0.0)
    grid = _grid({wall: LETHAL_COST})
    steps = int(round(STUCK_WINDOW_S / 0.05))
    # Команда держится дольше cmd_hold, потом DWB замолкает: дальше ноль.
    commanded_until = int(round(CMD_HOLD_S / 0.05)) + 2
    refused = []
    for i in range(steps + 1):
        vx = 0.1 if i <= commanded_until else 0.0
        refused.append(controller_stuck(i * 0.05, 0.0, 0.0, vx, 0.0, ORIGIN, ORIGIN, RES, SIZE, SIZE, grid))
    assert refused[-1] is True
    assert not any(refused[: commanded_until + 1])


def test_zero_command_with_no_recent_motion_is_not_a_failure():
    """Ноль всё окно и стена впереди — стоянка, не упор."""
    from rob_box_teleop.stuck_against import controller_stuck, reset_controller_state

    reset_controller_state()
    wall = _cell(BODY.x_max + BLOCK_GAP_M / 2, 0.0)
    grid = _grid({wall: LETHAL_COST})
    steps = int(round(STUCK_WINDOW_S / 0.05))
    refused = [
        controller_stuck(i * 0.05, 0.0, 0.0, 0.0, 0.0, ORIGIN, ORIGIN, RES, SIZE, SIZE, grid) for i in range(steps + 1)
    ]
    assert not any(refused)


def test_turning_in_place_drops_the_old_linear_course():
    from rob_box_teleop.stuck_against import controller_stuck

    state = StuckAgainst()
    wall = _cell(BODY.x_max + BLOCK_GAP_M / 2, 0.0)
    grid = _grid({wall: LETHAL_COST})
    for tick in range(81):
        # Only the first tick commanded translation. The angular command persists.
        vx = 0.1 if tick == 0 else 0.0
        refused = controller_stuck(tick / 20, 0.0, 0.0, vx, 0.3, ORIGIN, ORIGIN, RES, SIZE, SIZE, grid, state=state)
        assert refused == (tick == 40), tick  # Course is exactly at the cutoff on tick 40.


def test_controller_tick_refuses_when_stuck_and_not_when_sliding():
    """Связка, которую зовёт controller_server: окно копится между тиками.

    Упор: команда 0.1 м/с, поза стоит, впереди lethal → отказ ровно на границе
    окна. Вдоль стены: та же команда и та же стена сбоку, центр ушёл на 0.90 м
    → отказа нет.
    """
    from rob_box_teleop.stuck_against import controller_stuck, reset_controller_state

    reset_controller_state()
    wall = _cell(BODY.x_max + BLOCK_GAP_M / 2, 0.0)
    grid = _grid({wall: LETHAL_COST})
    steps = int(round(STUCK_WINDOW_S / 0.05))
    refused = [
        controller_stuck(i * 0.05, 0.0, 0.0, 0.1, 0.0, ORIGIN, ORIGIN, RES, SIZE, SIZE, grid) for i in range(steps + 1)
    ]
    assert refused[-1] is True
    assert not any(refused[:-1])

    reset_controller_state()
    side = _cell(0.0, BODY.y_min - BLOCK_GAP_M / 2)
    slide = _grid({side: LETHAL_COST})
    refused = [
        controller_stuck(
            i * 0.05,
            0.0,
            WALL_SLIDE_M * i / steps,
            0.1,
            0.0,
            ORIGIN,
            ORIGIN,
            RES,
            SIZE,
            SIZE,
            slide,
        )
        for i in range(steps + 1)
    ]
    assert not any(refused)


def _yaw(deg: float) -> float:
    return math.radians(deg)


def test_wall_ahead_in_odom_frame_is_a_block_world_lookup_is_not():
    """Стена впереди корпуса в кадре odom — блок. Поиск у начала карты — нет.

    Gate 2 21.11: local_costmap.global_frame = odom, корпус на метрах.
    Старый поиск брал мировые координаты корпуса как координаты клетки и
    полосу 0.08 м искал у начала карты. Стена впереди корпуса туда не
    попадала, отказ не ставился, BackUp не тикался.
    """
    from rob_box_teleop.stuck_against import cells_ahead_in_map

    robot = (1.20, -0.80, _yaw(90.0))
    # Вперёд по курсу 90° — это +y мира. Стена на 0.04 м за носом.
    ahead_x = robot[0]
    ahead_y = robot[1] + BODY.x_max + 0.04
    wall = _cell(ahead_x, ahead_y)
    grid = _grid({wall: LETHAL_COST})
    assert course_blocked(
        BODY,
        ORIGIN,
        ORIGIN,
        RES,
        SIZE,
        SIZE,
        grid,
        vx=0.1,
        robot_x=robot[0],
        robot_y=robot[1],
        robot_yaw=robot[2],
    )
    # Тот же вызов без позы (старый смысл) стену на метрах не видит.
    assert not course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, grid, vx=0.1)
    cells = list(cells_ahead_in_map(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, 0.1, BLOCK_GAP_M, *robot))
    assert wall in cells


def test_controller_refuses_when_odom_wall_is_ahead_of_the_body():
    """Команда есть, смещения нет, стена впереди корпуса в odom → отказ."""
    from rob_box_teleop.stuck_against import controller_stuck, reset_controller_state

    reset_controller_state()
    robot = (1.20, -0.80, _yaw(90.0))
    wall = _cell(robot[0], robot[1] + BODY.x_max + 0.04)
    grid = _grid({wall: LETHAL_COST})
    steps = int(round(STUCK_WINDOW_S / 0.05))
    refused = [
        controller_stuck(
            i * 0.05,
            robot[0],
            robot[1],
            0.1,
            0.0,
            ORIGIN,
            ORIGIN,
            RES,
            SIZE,
            SIZE,
            grid,
            yaw=robot[2],
        )
        for i in range(steps + 1)
    ]
    assert refused[-1] is True
    assert not any(refused[:-1])


def test_slide_along_wall_in_odom_is_not_a_failure():
    """Команда есть, центр ушёл на 0.90 м вдоль стены в odom → прогресс есть.

    Стена стоит сбоку от курса, не по нему. Ложного отказа быть не должно:
    иначе ровный проезд вдоль перегородки тикал бы BackUp.
    """
    from rob_box_teleop.stuck_against import controller_stuck, reset_controller_state

    reset_controller_state()
    yaw = _yaw(90.0)
    # Курс +y. Стена слева по борту: в мире это -x от корпуса.
    base = (1.20, -0.80)
    side = _cell(base[0] + BODY.y_min - BLOCK_GAP_M / 2, base[1])
    grid = _grid({side: LETHAL_COST})
    steps = int(round(STUCK_WINDOW_S / 0.05))
    refused = [
        controller_stuck(
            i * 0.05,
            base[0],
            base[1] + WALL_SLIDE_M * i / steps,
            0.1,
            0.0,
            ORIGIN,
            ORIGIN,
            RES,
            SIZE,
            SIZE,
            grid,
            yaw=yaw,
        )
        for i in range(steps + 1)
    ]
    assert not any(refused)
    assert WALL_SLIDE_M > PROGRESS_RADIUS_M


def test_guard_corridor_wall_is_a_failure_side_touch_strip_is_not():
    """Команда есть, смещения нет, стена в коридоре защиты, но дальше 0.08 м.

    Защита держит стоп, пока команда толкает в препятствие ближе stop_m
    (0.20 м, proximity.py). Полоса касания борта (side_stop_m, 0.08 м) перед
    носом при этом пуста: на develop course_blocked молчит, и отказ не
    ставится. Стена на 0.12 м — внутри коридора и дальше касания.
    """
    from rob_box_teleop.proximity import Thresholds
    from rob_box_teleop.stuck_against import controller_stuck, reset_controller_state

    stop_m = Thresholds().stop_m
    assert BLOCK_GAP_M == pytest.approx(stop_m)
    assert stop_m > Thresholds().side_stop_m

    dist = (Thresholds().side_stop_m + stop_m) / 2  # 0.14 м: дальше 0.08, ближе 0.20
    wall = _cell(BODY.x_max + dist, 0.0)
    grid = _grid({wall: LETHAL_COST})
    assert course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, grid, vx=0.1)
    # Дальше коридора — не упор: серая инфляция на ровном месте не стена.
    beyond = _cell(BODY.x_max + stop_m + RES, 0.0)
    assert not course_blocked(BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({beyond: LETHAL_COST}), vx=0.1)

    reset_controller_state()
    steps = int(round(STUCK_WINDOW_S / 0.05))
    commanded_until = int(round(CMD_HOLD_S / 0.05)) + 2
    refused = []
    for i in range(steps + 1):
        vx = 0.1 if i <= commanded_until else 0.0
        refused.append(controller_stuck(i * 0.05, 0.0, 0.0, vx, 0.0, ORIGIN, ORIGIN, RES, SIZE, SIZE, grid))
    assert refused[-1] is True
    assert not any(refused[: commanded_until + 1])
