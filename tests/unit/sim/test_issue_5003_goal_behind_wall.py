"""Геометрия сценария «цель за стеной» (#5003). Офлайн, Gazebo не запускается.

Карман 0.05 м сюда не годится: BackUp 0.025 м/с на simulate_ahead_time 2 с
проецирует 0.05 м назад и упирается в свою стену. Штатный dead_end оставляет
назад 4.5 м, а боковой зазор корпуса на спавне выше stop_m защиты (0.20 м)
и выше этой проекции. Цель сразу за гранью торца — внутри стены, не в коридоре
и не в 3 м пустоты, которую clearance не считает (search_radius).
"""

from rob_box_sim.geometry2d import ROBOT_FOOTPRINT, Pose2D, clearance
from rob_box_sim.worlds_catalog import DEAD_END_FRONT, DEAD_END_LENGTH, dead_end

# BackUp backup_speed 0.025 * behavior_server.simulate_ahead_time 2.0.
BACKUP_PROJECTION_M = 0.05
# proximity.Thresholds.stop_m, тот же коридор, что полоса упора.
GUARD_STOP_M = 0.20
# Сразу за гранью торца (нос + DEAD_END_FRONT + толщина/2). (1, 0) дальше
# search_radius clearance (3 м) и зазором не считается.
GOAL_X = 0.75


def test_dead_end_has_rear_room_and_side_gap_above_the_guard_stop():
    world = dead_end()
    spawn = Pose2D(0.0, 0.0, 0.0)
    gap = clearance(spawn, world.boxes, ROBOT_FOOTPRINT)
    assert gap == 0.2475 or abs(gap - 0.2475) < 1e-9
    assert gap > GUARD_STOP_M
    assert gap > BACKUP_PROJECTION_M
    assert DEAD_END_LENGTH > BACKUP_PROJECTION_M
    # Торец впереди носа, цель ещё дальше торца — за стеной, не в коридоре.
    end_face = ROBOT_FOOTPRINT.x_max + DEAD_END_FRONT
    assert GOAL_X > end_face
    assert clearance(Pose2D(GOAL_X, 0.0, 0.0), world.boxes, ROBOT_FOOTPRINT) < 0.0
