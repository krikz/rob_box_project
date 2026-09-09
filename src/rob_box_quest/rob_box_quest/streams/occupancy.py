"""map_2d payload: nav_msgs/OccupancyGrid → RGBA PNG + поза робота.

Источник истины: docs/architecture/meta-quest-api.md §4 (topic_id 0x1103).

Зачем PNG, а не сырая решётка. Карта rtabmap на роботе — 958×744 клеток
(47×37 м при 0.05 м/клетку), это 713 KB сырых байт на кадр. PNG той же
решётки — единицы килобайт (решётка почти целиком «unknown» и заливки),
и браузер декодирует его сам, без пиксельного цикла на JS. Тот же путь,
что у камер: сервер отдаёт готовый к текстуре байт-блоб.

Почему палитра живёт на сервере. Раскраска (unknown → прозрачный,
свободно → холодная плита, занято → яркая кромка) вшита в PNG, чтобы
клиент не гонял getImageData по 713 K пикселей на каждое обновление карты.

Почему карта голубая, а не зелёная. Зелёный на мостике уже занят лидаром
(lidar_overlay.ts: близко красный → далеко зелёный), а лидар и карта лежат
на полу рядом и в одном масштабе. Один цвет на двоих значил бы, что
оператор не отличает «что робот видит прямо сейчас» от «что он запомнил
раньше»: свежее препятствие потерялось бы в старой решётке. Поэтому карта
— голубая (`holo` из палитры мостика, 0x44ddff), лидар остаётся
красно-зелёным. Проверено в сцене: на тёмной палубе (#14181f) обе читаются
и не сливаются.

Значения свободной плиты подбирались там же, глазами: первая версия
(floorPanel 0x1a2230, alpha 110) была неотличима от плит палубы — карта
формально рисовалась, а на полу не читалась вообще.

Кадр приходит в двух видах (см. encode_map_2d):
  - полный: с `png` — клиент пересобирает текстуру (карта изменилась);
  - лёгкий: без `png` — только поза робота, клиент двигает уже готовую
    плоскость. Карта меняется редко, поза — постоянно, гонять из-за неё
    PNG незачем.
"""

from __future__ import annotations

from typing import Optional, Sequence

# Палитра (RGB), см. шапку модуля. Сцена: bridge_scene_meta.json.
FREE_RGB = (0x1B, 0x4A, 0x63)  # холодная плита «здесь просканировано»
OCCUPIED_RGB = (0x44, 0xDD, 0xFF)  # holo: стена/препятствие на карте
FREE_ALPHA = 105  # плита просвечивает — под ней декор палубы
OCCUPIED_ALPHA = 255

# Порог «занято» в шкале OccupancyGrid (0..100, -1 = unknown).
OCCUPIED_THRESHOLD = 50


def grid_to_png(data: Sequence[int], width: int, height: int) -> bytes:
    """OccupancyGrid.data → RGBA PNG bytes.

    Строки переворачиваются (``flipud``): в OccupancyGrid ``data[0]`` — клетка
    у origin, то есть нижняя строка по оси Y карты, а нулевая строка PNG —
    верхняя. Клиент вешает текстуру на плоскость с ``flipY`` по умолчанию,
    так что после переворота «выше по Y карты» = «выше по текстуре».

    Raises:
        ImportError: нет numpy/cv2 (в Docker-образе они есть — тот же путь,
            что у streams/camera.py).
        ValueError: длина data не совпадает с width×height.
    """
    import cv2  # type: ignore[import-not-found]
    import numpy as np  # type: ignore[import-not-found]

    if width <= 0 or height <= 0:
        raise ValueError(f"bad grid size {width}×{height}")
    expected = width * height
    if len(data) != expected:
        raise ValueError(f"grid data length {len(data)} != {width}×{height} = {expected}")

    # msg.data приходит как array.array('b') — np.asarray берёт его без копии
    # поэлементно; int16 нужен, чтобы сравнения не переполнялись на int8.
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)

    known = grid >= 0
    occupied = grid >= OCCUPIED_THRESHOLD

    # OpenCV пишет PNG из BGRA, поэтому каналы кладём в порядке B, G, R, A.
    bgra = np.zeros((height, width, 4), dtype=np.uint8)
    for ch, (free_v, occ_v) in enumerate(
        zip(FREE_RGB[::-1], OCCUPIED_RGB[::-1])  # RGB → BGR
    ):
        plane = bgra[:, :, ch]
        plane[known] = free_v
        plane[occupied] = occ_v
    alpha = bgra[:, :, 3]
    alpha[known] = FREE_ALPHA
    alpha[occupied] = OCCUPIED_ALPHA

    ok, buf = cv2.imencode(".png", np.flipud(bgra))
    if not ok:
        raise RuntimeError("cv2.imencode('.png') failed for occupancy grid")
    return bytes(buf)


def encode_map_2d(
    *,
    resolution: float,
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    robot_x: Optional[float],
    robot_y: Optional[float],
    robot_yaw: Optional[float],
    ts_ms: int,
    png: Optional[bytes] = None,
) -> bytes:
    """Собрать map_2d payload (MessagePack).

    ``png=None`` — лёгкий кадр «только поза»: клиент оставляет текущую
    текстуру и лишь пересчитывает трансформ плоскости.

    ``robot_*`` = ``None``, когда tf ``map → base_link`` ещё не доступен:
    клиент в этом случае не знает, куда класть карту, и прячет её. Явный
    ``None`` (а не 0.0) нужен, чтобы «позы нет» отличалось от «робот в
    начале карты».
    """
    import msgpack  # локальный импорт — как в protocol/topics.py

    payload: dict[str, object] = {
        "resolution": float(resolution),
        "width": int(width),
        "height": int(height),
        "origin_x": float(origin_x),
        "origin_y": float(origin_y),
        "robot_x": float(robot_x) if robot_x is not None else None,
        "robot_y": float(robot_y) if robot_y is not None else None,
        "robot_yaw": float(robot_yaw) if robot_yaw is not None else None,
        "ts_ms": int(ts_ms),
    }
    if png is not None:
        payload["png"] = png
    return msgpack.packb(payload, use_bin_type=True)
