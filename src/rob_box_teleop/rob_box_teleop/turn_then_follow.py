"""Порядок BT, когда путь есть, а нос упёрт в стену (#4475).

Дерево NavigateToPose само разворот не вставляет: recovery `Spin` крутит фиксированные
1.57 рад уже после провала FollowPath. Этот модуль — тот шаг, который дерево должно
сделать до FollowPath. Код выбирает угол (`turn_in_place.free_spin`) и говорит, какой
узел дерева идёт следующим. LLM и текст промпта здесь не участвуют.
"""

from __future__ import annotations

from typing import Optional, Sequence, Tuple

from rob_box_teleop.proximity import Body
from rob_box_teleop.turn_in_place import REAR_AXLE_X_M, free_spin

Point = Tuple[float, float]
Pose = Tuple[float, float, float]

#: Имена узлов — те же, что в navigate_to_pose_w_replanning.xml.
SPIN = "Spin"
FOLLOW = "FollowPath"
#: Робот «стоит»: пороги движения защиты (proximity.Thresholds), не новый допуск.
STILL_V_MPS = 0.03
STILL_W_RAD_S = 0.10


def next_bt_node(
    body: Body,
    pose: Pose,
    points: Sequence[Point],
    path: Sequence[Point],
    clearance_m: float,
    progress_m: float,
    *,
    vx: float = 0.0,
    wz: float = 0.0,
    pivot_x: float = REAR_AXLE_X_M,
) -> Tuple[str, Optional[float]]:
    """Следующий узел дерева и угол Spin.

    Путь есть, сдвиг с начала цели не больше нуля и нос упёрт в стену, а короткий
    свободный курс есть — ``("Spin", угол)``. Иначе ``("FollowPath", None)``: едем
    как раньше, фиксированный Spin не вставляется. Отрицательный прогресс (откат) —
    тот же ноль: вперёд не продвинулись.
    """
    if not path or progress_m > 0.0 or abs(vx) > STILL_V_MPS or abs(wz) > STILL_W_RAD_S:
        return FOLLOW, None
    spin = free_spin(body, pose, points, path, clearance_m, pivot_x=pivot_x)
    if spin is None:
        return FOLLOW, None
    return SPIN, spin
