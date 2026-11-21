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
    assert course_blocked(
        BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({wall: LETHAL_COST}), vx=0.1
    )
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
    assert not course_blocked(
        BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({side: LETHAL_COST}), vx=0.1
    )


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
    assert course_blocked(
        BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({ahead: INSCRIBED_COST}), vx=0.1
    )
    behind = _cell(BODY.x_min - 0.04, 0.0)
    assert course_blocked(
        BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({behind: LETHAL_COST}), vx=-0.1
    )
    # Вперёд стена сзади не мешает, и наоборот.
    assert not course_blocked(
        BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({behind: LETHAL_COST}), vx=0.1
    )


def test_inflation_beyond_gap_is_not_a_block():
    """Клетка дальше порога касания — не упор, даже если lethal."""
    far = _cell(BODY.x_max + BLOCK_GAP_M + RES, 0.0)
    assert not course_blocked(
        BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({far: LETHAL_COST}), vx=0.1
    )


def test_free_cell_and_zero_command_are_not_a_block():
    cell = _cell(BODY.x_max + 0.04, 0.0)
    assert not course_blocked(
        BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({cell: 200}), vx=0.1
    )
    assert not course_blocked(
        BODY, ORIGIN, ORIGIN, RES, SIZE, SIZE, _grid({cell: LETHAL_COST}), vx=0.0
    )


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
        refused.append(
            controller_stuck(
                i * 0.05, 0.0, 0.0, vx, 0.0, ORIGIN, ORIGIN, RES, SIZE, SIZE, grid
            )
        )
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
        controller_stuck(
            i * 0.05, 0.0, 0.0, 0.0, 0.0, ORIGIN, ORIGIN, RES, SIZE, SIZE, grid
        )
        for i in range(steps + 1)
    ]
    assert not any(refused)


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
        controller_stuck(
            i * 0.05, 0.0, 0.0, 0.1, 0.0, ORIGIN, ORIGIN, RES, SIZE, SIZE, grid
        )
        for i in range(steps + 1)
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
