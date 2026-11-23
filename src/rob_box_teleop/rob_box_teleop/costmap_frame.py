"""Сверка кадра costmap с позой перед проверкой столкновения (#5101).

behavior_server читает ``local_costmap/costmap_raw`` и решает «Collision Ahead»
по контуру footprint (Humble ``drive_on_heading.hpp``). Карта публикуется
только в цикле обновления (5 Гц, ``costmap_2d_ros.cpp``), а очистка
``clearEntirely`` сбрасывает её в память и ответ сервиса отдаёт сразу, не
дожидаясь публикации. Поэтому на тике BackUp (10 Гц), который идёт через
3–40 мс после очистки, behavior_server читает последнюю опубликованную копию —
до 0.2 с старую, ещё со стеной, в которую робот упёрся. Контур заднего края
ложится на lethal, и BackUp отменяется на первом тике, колёс не крутя.

Решение — код, не порог (ADR-0148): проверку столкновения делать только по
сетке, чей кадр совпадает с позой. Кадр совпадает, когда метка времени карты
не старше допуска и origin карты в той же клетке, что поза. Иначе тик
пропускается, а не считается отказом. Подряд пропусков не больше
``MAX_STALE_SKIPS``: дальше прежний отказ, чтобы при мёртвой карте робот не
поехал вслепую.
"""

from __future__ import annotations

# behavior_server.cycle_frequency 10 Гц (nav2_params.yaml): 3 тика ≈ 0.3 с.
MAX_STALE_SKIPS = 3


def costmap_frame_fresh(
    costmap_stamp_s: float,
    now_s: float,
    tolerance_s: float,
    origin_x: float,
    origin_y: float,
    pose_x: float,
    pose_y: float,
    resolution: float,
) -> bool:
    """Кадр карты годится для проверки столкновения.

    Свежий — метка не старше ``tolerance_s`` и origin в одной клетке с позой
    (rolling costmap двигает origin к сетке на каждом обновлении, клетка —
    ``resolution``). Чужой кадр — пропуск тика, не отказ.
    """
    if resolution <= 0.0:
        return False
    if now_s - costmap_stamp_s > tolerance_s:
        return False
    cell = resolution
    return abs(origin_x - pose_x) < cell and abs(origin_y - pose_y) < cell


def collision_check_action(frame_fresh: bool, skips_in_a_row: int) -> str:
    """Что делать с тиком проверки столкновения.

    ``"check"`` — сетка свежая, решать по контуру как раньше.
    ``"skip"`` — сетка чужая и подряд пропусков меньше предела: тик не отказ.
    ``"fail"`` — сетка чужая, но предел пропусков выбран: прежний отказ,
    чтобы не ехать вслепую при мёртвой карте.
    """
    if frame_fresh:
        return "check"
    if skips_in_a_row < MAX_STALE_SKIPS:
        return "skip"
    return "fail"
