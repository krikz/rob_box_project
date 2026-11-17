"""nav_path payload: глобальный план Nav2 (nav_msgs/Path) → MessagePack.

Источник истины: docs/architecture/meta-quest-api.md §4 (topic_id 0x1104),
issue #3151 (Captain Bridge волна 2).

Откуда путь. ``planner_server`` публикует глобальный план в ``plan``
(относительное имя без namespace → ``/plan``) на каждом ComputePathToPose.
В BT ``navigate_to_pose_w_replanning.xml`` пересчёт идёт через
``RateController hz=1.0`` — то есть ~1 Гц, пока цель активна, и тишина,
когда цели нет.

Почему прорежаем. NavFn на решётке 5 см выдаёт точку на каждую клетку:
путь через комнату — сотни точек, через этаж — тысячи. Клиенту на полу
нужна линия, а не клетки: ``NAV_PATH_MAX_POINTS`` равномерно взятых точек
(первая и последняя — всегда) рисуют тот же изгиб, а кадр остаётся в
единицах килобайт.

Почему кадр ``map`` и никаких пересчётов тут. Мостик эгоцентричный — всё
рисуется в ``base_link`` по последней позе робота, и эту позу клиент уже
получает в map_2d. Пересчитывать на сервере значит пересчитывать по ДРУГОЙ
позе (снимок сервера ≠ снимок клиента), и путь «плыл» бы относительно карты
на полу. Поэтому сервер отдаёт точки как есть в ``map``, а клиент кладёт
их тем же преобразованием, что и карту.

Формат (MessagePack map)::

    {frame: str, n: int, xy: bin, ts_ms: int}

``xy`` — ``n`` пар little-endian float32 ``[x0, y0, x1, y1, …]`` (метры,
кадр ``frame``). Пустой путь (``n = 0``) — явный сигнал «пути нет»: сервер
шлёт его, когда цель завершилась, чтобы клиент погасил линию, а не держал
последний план вечно.
"""

from __future__ import annotations

import struct
from typing import Any, Optional, Sequence

#: Максимум точек в кадре (после прорежения).
NAV_PATH_MAX_POINTS: int = 200
#: Не чаще одного кадра за столько секунд (≤ 2 Гц).
NAV_PATH_MIN_PERIOD_S: float = 0.5
#: Кадр, в котором клиент умеет рисовать путь. Остальные — отбрасываем.
NAV_PATH_FRAME: str = "map"

Point = tuple[float, float]


def decimate_path(points: Sequence[Point], max_points: int = NAV_PATH_MAX_POINTS) -> list[Point]:
    """Равномерно прорядить путь до ``max_points`` точек.

    Первая и последняя точки сохраняются всегда: начало — это робот, конец
    — цель, и именно их оператор сверяет с пином цели на полу.
    """
    n = len(points)
    if max_points < 2:
        raise ValueError(f"max_points must be >= 2, got {max_points}")
    if n <= max_points:
        return [(float(x), float(y)) for x, y in points]
    step = (n - 1) / (max_points - 1)
    return [
        (float(points[round(i * step)][0]), float(points[round(i * step)][1]))
        for i in range(max_points)
    ]


def encode_nav_path(*, frame: str, points: Sequence[Point], ts_ms: int) -> bytes:
    """Собрать nav_path payload (MessagePack, см. шапку модуля)."""
    import msgpack  # локальный импорт — как в streams/occupancy.py

    flat: list[float] = []
    for x, y in points:
        flat.append(float(x))
        flat.append(float(y))
    return msgpack.packb(
        {
            "frame": str(frame),
            "n": len(points),
            "xy": struct.pack(f"<{len(flat)}f", *flat),
            "ts_ms": int(ts_ms),
        },
        use_bin_type=True,
    )


def path_msg_points(msg: Any) -> tuple[str, list[Point]]:
    """nav_msgs/Path → (frame_id, [(x, y), …]).

    ``frame_id`` берётся из заголовка пути, а не из позы: Nav2 кладёт один
    и тот же кадр во все позы плана.
    """
    frame = str(getattr(getattr(msg, "header", None), "frame_id", "") or "")
    pts: list[Point] = []
    for stamped in getattr(msg, "poses", []) or []:
        pos = stamped.pose.position
        pts.append((float(pos.x), float(pos.y)))
    return frame, pts


class NavPathThrottle:
    """Дроссель кадров nav_path: не чаще ``min_period_s``.

    ``force=True`` пропускает кадр без оглядки на период — так уходит
    пустой кадр «путь погас» по завершении цели: терять его из-за того, что
    план пришёл 100 мс назад, нельзя, иначе линия осталась бы на полу.
    """

    def __init__(self, min_period_s: float = NAV_PATH_MIN_PERIOD_S) -> None:
        self._min_period_s = float(min_period_s)
        self._last_ts: Optional[float] = None

    def admit(self, now: float, *, force: bool = False) -> bool:
        if not force and self._last_ts is not None and now - self._last_ts < self._min_period_s:
            return False
        self._last_ts = now
        return True
