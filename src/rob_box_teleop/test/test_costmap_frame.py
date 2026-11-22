"""#5101: проверка столкновения BackUp только по свежему кадру costmap.

Отказ «Collision Ahead» на первом тике BackUp шёл по копии карты, которую
очистка ещё не опубликовала. Решение — код: чужой кадр пропускает тик, а не
отменяет ход, но не больше MAX_STALE_SKIPS подряд.
"""

from rob_box_teleop.costmap_frame import (
    MAX_STALE_SKIPS,
    collision_check_action,
    costmap_frame_fresh,
)

RESOLUTION = 0.05
TOLERANCE_S = 0.5


def test_fresh_frame_is_checked():
    fresh = costmap_frame_fresh(
        costmap_stamp_s=10.0,
        now_s=10.1,
        tolerance_s=TOLERANCE_S,
        origin_x=1.00,
        origin_y=-0.50,
        pose_x=1.02,
        pose_y=-0.49,
        resolution=RESOLUTION,
    )
    assert fresh
    assert collision_check_action(True, skips_in_a_row=0) == "check"
    # Свежий кадр проверяется даже после серии пропусков.
    assert collision_check_action(True, skips_in_a_row=MAX_STALE_SKIPS) == "check"


def test_stale_stamp_skips_then_fails_at_the_limit():
    stale = costmap_frame_fresh(
        costmap_stamp_s=10.0,
        now_s=10.0 + TOLERANCE_S + 0.01,
        tolerance_s=TOLERANCE_S,
        origin_x=1.0,
        origin_y=-0.5,
        pose_x=1.0,
        pose_y=-0.5,
        resolution=RESOLUTION,
    )
    assert not stale
    for skips in range(MAX_STALE_SKIPS):
        assert collision_check_action(False, skips_in_a_row=skips) == "skip"
    assert collision_check_action(False, skips_in_a_row=MAX_STALE_SKIPS) == "fail"


def test_origin_a_cell_away_is_a_foreign_frame():
    """Rolling costmap двигает origin на целую клетку: это уже чужой кадр."""
    shifted = costmap_frame_fresh(
        costmap_stamp_s=10.0,
        now_s=10.05,
        tolerance_s=TOLERANCE_S,
        origin_x=1.0 + RESOLUTION,
        origin_y=-0.5,
        pose_x=1.0,
        pose_y=-0.5,
        resolution=RESOLUTION,
    )
    assert not shifted
    assert collision_check_action(False, skips_in_a_row=0) == "skip"


def test_zero_resolution_is_not_a_frame():
    assert not costmap_frame_fresh(10.0, 10.0, TOLERANCE_S, 0.0, 0.0, 0.0, 0.0, 0.0)
