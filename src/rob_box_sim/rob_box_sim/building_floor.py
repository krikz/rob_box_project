"""Этаж учебного корпуса для HIL (#4468): план — данные, стены, двери и точки-кабинеты — код.

Готовые миры не взяли (таблица разведки — в PR шага 1 карточки #4468): AWS RoboMaker Hospital (MIT-0, collision-меши
есть) написан под Gazebo classic 7/9 и тянет ~40 моделей Fuel; OSRF First Office (CC0) — 26×18 м,
collision = визуальный меш, ширина дверей не известна. Свой этаж из коробок: лицензия наша,
collision у каждой стены, двери не уже ``MIN_DOOR`` (решение Шифу 05.11: минимум 0.90 м,
берём с запасом), и есть то, что нужно проверять, — похожие участки коридора.

План, вид сверху (x вдоль длинной стороны, y поперёк; S — спавн робота (0, 0) лицом по +X;
двери — в стенах коридора, точное место — ``ROOMS``)::

    y=+7.5   ┌──────────┬────────┬──────┬──────┬────┬────┬────┬────┬────┬────┬────┐
             │          │ауд. 101│ауд102│кафедр│лест│201 │202 │203 │204 │205 │206 │
    y=+1.15  │   холл   ├────────┴──────┴──────┴────┴────┴────┴────┴────┴────┴────┤
         ┌───┘    S →     коридор 2.2 м в свету                                   │
         │тамбур (вход)                                                         конец
         └───┐                                                                    │
    y=-1.15  │          ├───────────┬─────┬─────┬────┬────┬────┬────┬────┬────┬────┤
             │  вахта   │ столовая  │декан│библ.│сан.│207 │208 │209 │210 │211 │212 │
    y=-7.5   └──────────┴───────────┴─────┴─────┴────┴────┴────┴────┴────┴────┴────┘
          x=-8  -6      4           14    19    24   28   32   36   40   44   48   52

Этаж 58 × 15 м (x −6…52, y ±7.5) и тамбур 2 × 3 м у входа (x −8…−6). Двадцать кабинетов:
западное крыло разное (аудитории, кафедра, столовая, деканат, библиотека), середина — лестничная
клетка и санузел, восточное крыло — двенадцать **одинаковых** кабинетов по 4 м, дверь на одном и том
же месте, одна и та же мебель. Из середины восточного крыла (x 38…42) лидар (10 м) видит только этот
повторяющийся участок: скан почти одинаков каждые 4 м — проверка «похожих мест» (ложные узнавания
rtabmap, скачки позы в ``map``).

Мебель — коробки не ниже ``FURNITURE_MIN_H``: выше плоскости лидара (0.59 м над полом), лидар её
видит. Препятствия ниже лидара проверяет мир ``low_obstacle``, здесь их нет.

Система координат. Робот спавнится в (0, 0, 0) мира. rtabmap на пустой карте кладёт начало ``map``
в стартовую позу (runbook hil-video §1 п. 6), поэтому при спавне в (0, 0, 0) ``map`` = мир и точки
из ``building.goals.yaml`` годятся как цели в ``map`` без пересчёта. Карта, снятая с этого спавна, хранит
ту же систему: верная локализация по ней после рестарта даёт ``map`` = мир (это и проверяем). Если робот стартует
картографию из другой позы ``spawn`` (тест глобальной локализации, «похищенный робот» — до того, как
он себя нашёл), цель в ``map`` новой карты = ``world_to_map(точка, spawn)``.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

from rob_box_sim.geometry2d import Box, Pose2D

WALL_T = 0.1  # как у остальных миров (worlds_catalog.WALL_T), тест сверяет
WALL_H = 1.0
FURNITURE_MIN_H = 0.8

X_WEST = -6.0
X_EAST = 52.0
Y_OUT = 7.5
CORRIDOR_AXIS = 1.15  # оси стен коридора y = ±1.15 → в свету 2.2 м
CORRIDOR_X0 = 4.0  # коридор начинается от холла
ENTRANCE_WIDTH = 1.8  # проём из холла в тамбур
VESTIBULE_X0 = -8.0
VESTIBULE_HALF = 1.5  # оси боковых стен тамбура

# Двери этажа не уже 0.95 м: минимальный проём робота 0.90 м (решение Шифу 05.11),
# кабинетные двери в брифе #4468 — 0.95–1.2 м.
MIN_DOOR = 0.95
# Точка кабинета — напротив первой двери, в GOAL_DEPTH от лица стены коридора внутрь комнаты.
GOAL_DEPTH = 1.4


@dataclass(frozen=True)
class Item:
    """Мебель в системе комнаты: u — вдоль коридора от оси западной стены комнаты, v — вглубь от оси стены коридора."""

    name: str
    u: float
    v: float
    su: float
    sv: float
    height: float = FURNITURE_MIN_H


@dataclass(frozen=True)
class Room:
    id: str  # латиница: имя цели в логах route.py / nav_bench
    label: str  # как кабинет называют голосом
    side: str  # "north" (y > 0) | "south"
    x0: float
    x1: float
    doors: tuple[tuple[float, float], ...]  # (от оси западной стены до края двери, ширина)
    items: tuple[Item, ...] = ()


@dataclass(frozen=True)
class Door:
    """Проём в стене: стена вдоль ``axis`` на линии ``at``, проём ``lo…hi`` вдоль стены."""

    name: str
    axis: str  # "x" — стена вдоль X (y = at), "y" — стена вдоль Y (x = at)
    at: float
    lo: float
    hi: float

    @property
    def width(self) -> float:
        return round(self.hi - self.lo, 6)

    @property
    def center(self) -> float:
        return (self.lo + self.hi) / 2


@dataclass(frozen=True)
class NamedPoint:
    name: str
    label: str
    x: float
    y: float
    yaw: float


def _east_rooms() -> tuple[Room, ...]:
    """Двенадцать одинаковых кабинетов по 4 м: дверь и мебель на одних и тех же местах."""
    items = (Item("desk", 2.5, 3.0, 1.4, 0.7), Item("cabinet", 2.0, 5.95, 2.0, 0.5, 1.8))
    rooms = []
    for i in range(6):
        x0 = 28.0 + 4.0 * i
        rooms.append(Room(f"room_{201 + i}", f"аудитория {201 + i}", "north", x0, x0 + 4.0, ((0.5, 1.0),), items))
        rooms.append(Room(f"room_{207 + i}", f"аудитория {207 + i}", "south", x0, x0 + 4.0, ((0.5, 1.0),), items))
    return tuple(rooms)


_DESK = 0.6
ROOMS: tuple[Room, ...] = (
    Room(
        "aud_101",
        "аудитория 101",
        "north",
        4.0,
        12.0,
        ((1.0, 1.2), (6.0, 1.2)),
        (Item("desks_1", 4.0, 2.8, 5.0, _DESK), Item("desks_2", 4.0, 4.4, 5.0, _DESK)),
    ),
    Room(
        "aud_102",
        "аудитория 102",
        "north",
        12.0,
        18.0,
        ((0.8, 1.1),),
        (Item("desks_1", 3.5, 3.0, 4.0, _DESK), Item("desks_2", 3.5, 4.6, 4.0, _DESK)),
    ),
    Room(
        "dept_103",
        "кафедра",
        "north",
        18.0,
        24.0,
        ((0.6, 0.95),),
        (Item("cabinet", 3.5, 5.95, 4.0, 0.5, 1.8), Item("desk", 3.8, 3.0, 1.4, 0.7)),
    ),
    Room(
        "stairs",
        "лестница",
        "north",
        24.0,
        28.0,
        ((1.4, 1.2),),
        (Item("flight", 2.0, 4.25, 3.5, 4.0, WALL_H),),
    ),
    Room(
        "canteen",
        "столовая",
        "south",
        4.0,
        14.0,
        ((1.4, 1.2), (7.4, 1.2)),
        tuple(Item(f"table_{u:.0f}_{v:.0f}", u, v, 1.2, 0.8) for u in (3.0, 5.0, 7.0) for v in (3.0, 4.8))
        + (Item("counter", 5.0, 5.95, 6.0, 0.5, 1.0),),
    ),
    Room(
        "dean",
        "деканат",
        "south",
        14.0,
        19.0,
        ((0.6, 1.0),),
        (Item("desk", 3.2, 3.6, 1.6, 0.8), Item("cabinet", 4.6, 3.0, 0.5, 2.0, 1.8)),
    ),
    Room(
        "library",
        "библиотека",
        "south",
        19.0,
        24.0,
        ((0.6, 1.1),),
        (Item("shelf_1", 2.6, 3.6, 0.4, 3.5, 1.8), Item("shelf_2", 4.0, 3.6, 0.4, 3.5, 1.8)),
    ),
    Room(
        "wc",
        "санузел",
        "south",
        24.0,
        28.0,
        ((0.6, 0.95),),
        tuple(Item(f"stall_{k}", 1.0 + k, 5.45, 0.9, 1.5, 1.8) for k in range(3))
        + (Item("sinks", 3.4, 2.0, 1.0, 0.5, 0.9),),
    ),
) + _east_rooms()

# Холл: колонны, вахта, стенд — опоры для лидара в большом открытом месте (координаты мира).
HALL_ITEMS: tuple[Box, ...] = (
    Box("hall_column_nw", -3.5, 4.0, 0.4, 0.4, WALL_H),
    Box("hall_column_ne", 1.5, 4.0, 0.4, 0.4, WALL_H),
    Box("hall_column_sw", -3.5, -4.0, 0.4, 0.4, WALL_H),
    Box("hall_column_se", 1.5, -4.0, 0.4, 0.4, WALL_H),
    Box("hall_reception", -2.0, -5.6, 3.0, 0.8, 1.1),
    Box("hall_stand", 2.0, 6.2, 1.5, 0.3, 1.8),
)

# Точки вне кабинетов (координаты мира, yaw в радианах).
EXTRA_POINTS: tuple[NamedPoint, ...] = (
    NamedPoint("hall", "холл", 0.0, 0.0, 0.0),
    NamedPoint("entrance", "вход", -7.0, 0.0, math.pi),
    NamedPoint("reception", "вахта", -2.0, -4.2, -math.pi / 2),
    NamedPoint("corridor_end", "конец коридора", 50.5, 0.0, 0.0),
)


def _on_cm(v: float) -> bool:
    """Край двери на сантиметровой сетке: центры коробок тогда точны в миллиметрах SDF (world_sdf._f)."""
    return abs(v * 100 - round(v * 100)) < 1e-6


def _sign(room: Room) -> float:
    return 1.0 if room.side == "north" else -1.0


def room_doors(room: Room) -> list[Door]:
    """Двери комнаты в стене коридора; ширина и место проверяются здесь, а не на глаз."""
    if room.side not in ("north", "south"):
        raise ValueError(f"{room.id}: side {room.side!r}")
    out = []
    for k, (off, width) in enumerate(room.doors):
        lo, hi = round(room.x0 + off, 6), round(room.x0 + off + width, 6)
        if width < MIN_DOOR - 1e-9:
            raise ValueError(f"{room.id}: дверь {width} м уже {MIN_DOOR}")
        if not (_on_cm(lo) and _on_cm(hi)):
            raise ValueError(f"{room.id}: края двери {lo}…{hi} не на сантиметровой сетке")
        if lo < room.x0 + 0.15 or hi > room.x1 - 0.15:
            raise ValueError(f"{room.id}: дверь {lo}…{hi} упирается в перегородку")
        out.append(Door(f"{room.id}_door{k + 1}", "x", _sign(room) * CORRIDOR_AXIS, lo, hi))
    return out


def all_doors() -> list[Door]:
    doors = [Door("entrance", "y", X_WEST, -ENTRANCE_WIDTH / 2, ENTRANCE_WIDTH / 2)]
    for room in ROOMS:
        doors += room_doors(room)
    return doors


def _solid(lo: float, hi: float, gaps: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Отрезок lo…hi без проёмов ``gaps`` (проёмы не пересекаются)."""
    out, cur = [], lo
    for g0, g1 in sorted(gaps):
        if g0 > cur:
            out.append((cur, g0))
        cur = max(cur, g1)
    if hi > cur:
        out.append((cur, hi))
    return out


def _wall(name: str, axis: str, at: float, lo: float, hi: float, gaps=()) -> list[Box]:
    boxes = []
    for i, (a, b) in enumerate(_solid(lo, hi, list(gaps))):
        n = name if len(gaps) == 0 else f"{name}_{i}"
        c, s = round((a + b) / 2, 6), round(b - a, 6)
        boxes.append(Box(n, c, at, s, WALL_T, WALL_H) if axis == "x" else Box(n, at, c, WALL_T, s, WALL_H))
    return boxes


def _shell() -> list[Box]:
    """Наружные стены и тамбур. Концы стен заходят на полтолщины — углы закрыты."""
    t2 = WALL_T / 2
    return (
        _wall("outer_north", "x", Y_OUT, X_WEST - t2, X_EAST + t2)
        + _wall("outer_south", "x", -Y_OUT, X_WEST - t2, X_EAST + t2)
        + _wall("outer_east", "y", X_EAST, -Y_OUT, Y_OUT)
        + _wall("outer_west", "y", X_WEST, -Y_OUT, Y_OUT, [(-ENTRANCE_WIDTH / 2, ENTRANCE_WIDTH / 2)])
        + _wall("vestibule_north", "x", VESTIBULE_HALF, VESTIBULE_X0 - t2, X_WEST)
        + _wall("vestibule_south", "x", -VESTIBULE_HALF, VESTIBULE_X0 - t2, X_WEST)
        + _wall("vestibule_west", "y", VESTIBULE_X0, -VESTIBULE_HALF, VESTIBULE_HALF)
    )


def _corridor_walls() -> list[Box]:
    t2 = WALL_T / 2
    boxes = []
    for side, sign in (("north", 1.0), ("south", -1.0)):
        gaps = [(d.lo, d.hi) for r in ROOMS if r.side == side for d in room_doors(r)]
        boxes += _wall(f"corridor_{side}", "x", sign * CORRIDOR_AXIS, CORRIDOR_X0 - t2, X_EAST, gaps)
    return boxes


def _partitions() -> list[Box]:
    """Перегородки между комнатами (и стена холла x = 4) — от стены коридора до наружной."""
    t2 = WALL_T / 2
    boxes = []
    for side, sign in (("north", 1.0), ("south", -1.0)):
        xs = sorted({r.x0 for r in ROOMS if r.side == side})
        for x in xs:
            lo, hi = sorted((sign * (CORRIDOR_AXIS - t2), sign * (Y_OUT + t2)))
            boxes += _wall(f"partition_{side}_{x:g}".replace(".", "_"), "y", x, lo, hi)
    return boxes


def _items(room: Room) -> list[Box]:
    s = _sign(room)
    return [
        Box(f"{room.id}_{it.name}", room.x0 + it.u, s * (CORRIDOR_AXIS + it.v), it.su, it.sv, it.height)
        for it in room.items
    ]


def floor_boxes() -> list[Box]:
    boxes = _shell() + _corridor_walls() + _partitions() + list(HALL_ITEMS)
    for room in ROOMS:
        boxes += _items(room)
    return boxes


def room_point(room: Room) -> NamedPoint:
    """Точка кабинета: напротив первой двери, в GOAL_DEPTH от лица стены коридора, лицом внутрь."""
    door = room_doors(room)[0]
    s = _sign(room)
    y = s * (CORRIDOR_AXIS + WALL_T / 2 + GOAL_DEPTH)
    return NamedPoint(room.id, room.label, round(door.center, 3), round(y, 3), s * math.pi / 2)


def named_points() -> list[NamedPoint]:
    """Порядок — объезд этажа: холл и вход, западное крыло, середина, восток, обратно в холл не входит."""
    hall, entrance, reception, end = EXTRA_POINTS
    rooms = sorted((room_point(r) for r in ROOMS), key=lambda p: (p.x, -p.y))
    return [hall, entrance, reception] + rooms + [end]


def world_to_map(point: NamedPoint, spawn: Pose2D) -> Pose2D:
    """Точка мира в ``map`` карты, начатой из позы ``spawn`` (начало ``map`` = стартовая поза)."""
    return spawn.inverse().compose(Pose2D(point.x, point.y, point.yaw))


def goals_yaml(world: str, points: list[NamedPoint]) -> str:
    """Файл точек рядом с миром: формат ``goals_from`` стенда nav_bench, плюс ``label`` для голоса."""
    lines = [
        f"# Точки-кабинеты мира {world} (rob_box_sim.building_floor). Сгенерирован:",
        "#   python3 -m rob_box_sim.worlds_catalog src/rob_box_sim/worlds  — руками не править.",
        "# Координаты — мир Gazebo, yaw в радианах. Спавн робота (0, 0, 0): на пустой карте",
        "# rtabmap кладёт начало map в спавн, поэтому map = мир. Из другой позы спавна:",
        "# building_floor.world_to_map(точка, spawn).",
        f"world: {world}",
        "frame: world",
        "spawn: {x: 0.0, y: 0.0, yaw: 0.0}",
        "goals:",
    ]
    for p in points:
        label = json.dumps(p.label, ensure_ascii=False)
        lines.append(f"  - {{name: {p.name}, label: {label}, x: {p.x:g}, y: {p.y:g}, yaw: {round(p.yaw, 4):g}}}")
    return "\n".join(lines) + "\n"


def description() -> str:
    n_rooms = len(ROOMS)
    return (
        f"  Этаж учебного корпуса (#4468): {X_EAST - X_WEST:g} × {2 * Y_OUT:g} м (x {X_WEST:g}..{X_EAST:g}, "
        f"y ±{Y_OUT:g}) + тамбур у входа x {VESTIBULE_X0:g}..{X_WEST:g}.\n"
        f"  Коридор {2 * CORRIDOR_AXIS - WALL_T:.1f} м в свету вдоль X, {n_rooms} кабинетов, двери ≥ {MIN_DOOR} м.\n"
        "  Восточное крыло (x 28..52) — двенадцать одинаковых кабинетов по 4 м: похожие места.\n"
        "  Стены и мебель — коробки с collision, мебель выше плоскости лидара.\n"
        "  План и точки: rob_box_sim.building_floor, точки — building.goals.yaml.\n"
        "  Робот спавнится в (0, 0) в холле, смотрит по +X вдоль коридора."
    )
