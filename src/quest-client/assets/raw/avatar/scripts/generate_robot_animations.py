"""Генератор robot_animations.glb — анимации колёс робота Rob Box.

Создаёт 4 анимации для использования в Meta Quest WebXR-клиенте
(Phase 2+, ADR-0028 §1.2, ADR-0032 §3.2):

  - idle:        колёса стоят, корпус делает лёгкий breathing-sway
                 (±2 см по Z, период 3 сек). Имитация "живого" робота.
  - forward:     все 4 колеса вращаются вперёд (отрицательный угол вокруг Y,
                 ось согласована с ROS convention). 1 оборот / сек.
  - turn_left:   дифференциальное — левые колёса едут назад, правые вперёд
                 (или все медленнее на одной стороне). Короткий поворот.
  - turn_right:  симметрично turn_left.

Целевой размер каждой анимации: ≤ 25 KB (общий файл ≤ 100 KB).
Используется LINEAR interpolation, keyframes через 0.1 сек, минимальный
квантайз — для delta-encoding через gltf-transform (Phase 2.0).

Технические детали (см. docs/avatar_pipeline.md):
  - glTF 2.0 spec, binary container (.glb).
  - Каждая анимация — отдельный glTF animation с rotation tracks для
    колёс + translation tracks для breathing-sway (idle).
  - Все wheel joints имеют ось вращения [0, 1, 0] (поперечная ось робота),
    положительное вращение = "колесо крутится вперёд" (по ROS REP-103).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any

import numpy as np

# Импортируем SCENE_GRAPH из generate_robot_body для согласованности узлов.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_robot_body import SCENE_GRAPH  # noqa: E402

# Параметры анимаций.
ANIMATION_DURATION_SEC = 1.0  # все 4 анимации — 1 секунда (loop)
KEYFRAME_INTERVAL_SEC = 0.1  # 10 fps для keyframes (минимально)
TARGET_WHEEL_RPM = 30.0  # скорость вращения колёс в idle/forward
# Угловая скорость рад/сек для forward: 30 RPM = π рад/сек.
OMEGA_FORWARD = TARGET_WHEEL_RPM * 2 * math.pi / 60  # ≈ π рад/с
# Breathing sway: ±2 см вертикальное смещение корпуса в idle.
BREATHING_AMPLITUDE_M = 0.02
BREATHING_PERIOD_SEC = 3.0

WHEEL_NAMES = [
    "left_front_wheel",
    "right_front_wheel",
    "left_rear_wheel",
    "right_rear_wheel",
]


# Quaternion от rotation вокруг оси (x, y, z) на угол theta.
def quat_from_axis_angle(axis: tuple[float, float, float], theta: float) -> list[float]:
    """Вернуть quaternion [x, y, z, w] (glTF convention)."""
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n == 0:
        return [0.0, 0.0, 0.0, 1.0]
    x, y, z = x / n, y / n, z / n
    s = math.sin(theta / 2)
    c = math.cos(theta / 2)
    return [x * s, y * s, z * s, c]


def quat_identity() -> list[float]:
    return [0.0, 0.0, 0.0, 1.0]


def build_animations() -> dict[str, Any]:
    """Собрать структуру glTF 2.0 (в памяти) с 4 анимациями.

    Возвращает dict, который затем сереализуется в .glb.
    Используем pygltflib для надёжности (gltf-transform CLI отдельным шагом).
    """
    # Структура keyframes:
    #   input[]: time (sec), float32
    #   output[]: rotation quaternion [x, y, z, w] для каждого keyframe
    #   интерполяция: LINEAR (по умолчанию в glTF)
    times = np.arange(0.0, ANIMATION_DURATION_SEC + KEYFRAME_INTERVAL_SEC / 2, KEYFRAME_INTERVAL_SEC, dtype=np.float32)
    if times[-1] > ANIMATION_DURATION_SEC:
        times = times[:-1]

    animations = []

    # 1) IDLE — колёса неподвижны (rotation = identity), корпус sway.
    # Чтобы не плодить idle-канал для колёс (всё равно нули), выдаём
    # только sway-канал для base_link (корпус едет вверх-вниз).
    idle_channels = []
    idle_samplers = []
    sway_t = np.linspace(
        0, BREATHING_PERIOD_SEC, max(2, int(BREATHING_PERIOD_SEC / KEYFRAME_INTERVAL_SEC)), dtype=np.float32
    )
    # base_link translation: только Z компонент дышит, X и Y = 0.
    sway_z = (BREATHING_AMPLITUDE_M * np.sin(2 * math.pi * sway_t / BREATHING_PERIOD_SEC)).astype(np.float32)
    sway_xyz = np.zeros((len(sway_t), 3), dtype=np.float32)
    sway_xyz[:, 2] = sway_z
    # base_link сдвинут на wheel_radius — анимация перебивает его translation.
    # Поэтому передаём полный translation (X=0, Y=0, Z=0.1143 + sway).
    base_z = 0.1143
    base_xyz = np.zeros((len(sway_t), 3), dtype=np.float32)
    base_xyz[:, 2] = base_z + sway_z
    idle_channels.append(
        {
            "sampler": 0,
            "target": {"node": "base_link", "path": "translation"},
        }
    )
    idle_samplers.append(
        {
            "input": sway_t,
            "output": base_xyz,
            "interpolation": "LINEAR",
        }
    )
    animations.append(
        {
            "name": "idle",
            "channels": idle_channels,
            "samplers": idle_samplers,
        }
    )

    # 2) FORWARD — все 4 колеса вращаются вокруг Y.
    # В ROS REP-103 X=forward, Y=left. Ось вращения колеса — Y (поперечная).
    # Положительный rotation вокруг Y (по правилу правой руки, thumb = +Y)
    # означает вращение "колесо крутится по часовой стрелке если смотреть
    # слева" = робот едет **назад**. Поэтому для forward rotation negative.
    fwd_channels = []
    fwd_samplers = []
    for i, wheel in enumerate(WHEEL_NAMES):
        angles = -OMEGA_FORWARD * times  # negative для forward
        quats = np.array([quat_from_axis_angle((0, 1, 0), float(a)) for a in angles], dtype=np.float32)
        fwd_channels.append(
            {
                "sampler": i,
                "target": {"node": wheel, "path": "rotation"},
            }
        )
        fwd_samplers.append(
            {
                "input": times,
                "output": quats,
                "interpolation": "LINEAR",
            }
        )
    animations.append(
        {
            "name": "forward",
            "channels": fwd_channels,
            "samplers": fwd_samplers,
        }
    )

    # 3) TURN_LEFT — differential drive.
    # Левые колёса: negative (назад), правые: positive (вперёд).
    # Уменьшаем скорость (×0.5) для in-place разворота (но не совсем in-place).
    LEFT_WHEELS = {"left_front_wheel", "left_rear_wheel"}
    RIGHT_WHEELS = {"right_front_wheel", "right_rear_wheel"}
    turn_speed = OMEGA_FORWARD * 0.5
    tl_channels = []
    tl_samplers = []
    for i, wheel in enumerate(WHEEL_NAMES):
        sign = -1.0 if wheel in LEFT_WHEELS else 1.0  # left back, right forward
        angles = sign * turn_speed * times
        quats = np.array([quat_from_axis_angle((0, 1, 0), float(a)) for a in angles], dtype=np.float32)
        tl_channels.append(
            {
                "sampler": i,
                "target": {"node": wheel, "path": "rotation"},
            }
        )
        tl_samplers.append(
            {
                "input": times,
                "output": quats,
                "interpolation": "LINEAR",
            }
        )
    animations.append(
        {
            "name": "turn_left",
            "channels": tl_channels,
            "samplers": tl_samplers,
        }
    )

    # 4) TURN_RIGHT — симметрично.
    tr_channels = []
    tr_samplers = []
    for i, wheel in enumerate(WHEEL_NAMES):
        sign = 1.0 if wheel in LEFT_WHEELS else -1.0
        angles = sign * turn_speed * times
        quats = np.array([quat_from_axis_angle((0, 1, 0), float(a)) for a in angles], dtype=np.float32)
        tr_channels.append(
            {
                "sampler": i,
                "target": {"node": wheel, "path": "rotation"},
            }
        )
        tr_samplers.append(
            {
                "input": times,
                "output": quats,
                "interpolation": "LINEAR",
            }
        )
    animations.append(
        {
            "name": "turn_right",
            "channels": tr_channels,
            "samplers": tr_samplers,
        }
    )

    return {"animations": animations, "nodes": SCENE_GRAPH["nodes"]}


def write_glb_with_animations(out_path: Path, body_glb_path: Path, verbose: bool = True) -> dict[str, Any]:
    """Сгенерировать robot_animations.glb.

    Стратегия: загрузить body_glb как "пустую" сцену (только ноды, без
    геометрии), добавить animations. Геометрия не нужна для воспроизведения
    анимации — Three.js загружает animations отдельно через AnimationMixer.

    Но для удобства тестирования (single-file) положим сюда и геометрию
    (просто копию body_glb + animations). Это даст +93 KB overhead, что
    выходит за 100 KB бюджет. Поэтому:
      - Если body_glb существует → делаем "animations only" glb (без
        геометрии, только nodes skeleton + animations).
      - Если body_glb не существует → собираем полную сцену с нуля.
    """
    # Способ 1: генерируем полную сцену с нуля — это правильнее и
    # согласованнее. body_glb уже есть, мы просто пересоберём его
    # с добавлением animations.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    # Сначала сгенерируем body scene (как раньше), но в persistent tmp-файле.
    import trimesh
    from generate_robot_body import DECIMATE_FACE_TARGET, MESH_MATERIAL, build_scene

    tmp_path = Path("/tmp/_robot_body_tmp.glb")
    tmp_path.parent.mkdir(parents=True, exist_ok=True)
    build_scene(tmp_path, body_glb_path.parent / "meshes", verbose=False)
    body_scene = trimesh.load(str(tmp_path))

    # Теперь добавляем animations через pygltflib (низкоуровневый доступ).
    from pygltflib import (
        GLTF2,
        Accessor,
        Animation,
        AnimationChannel,
        AnimationSampler,
        Asset,
        Buffer,
        BufferView,
    )

    g = GLTF2.load(str(tmp_path))

    # Получаем mapping имя_ноды → node_index.
    node_index_by_name: dict[str, int] = {}
    if g.nodes:
        for i, n in enumerate(g.nodes):
            if n.name:
                node_index_by_name[n.name] = i

    # Соберём буфер для всех animation data.
    buffers = []  # list of (componentType, count, data_bytes)
    accessors = []  # list of Accessor
    samplers = []  # list of AnimationSampler
    channels = []  # list of AnimationChannel
    animations = []

    ACCESSOR_COMPONENT = {
        "float32": (5126, 4),
    }
    TYPE_COUNT = {
        "SCALAR": 1,
        "VEC2": 2,
        "VEC3": 3,
        "VEC4": 4,
        "MAT4": 16,
    }

    def add_accessor(component_type: str, atype: str, data: np.ndarray, min_val=None, max_val=None) -> int:
        """Добавить accessor (и buffer) в наш временный список."""
        ct_code, ct_size = ACCESSOR_COMPONENT[component_type]
        count = len(data)
        flat = data.astype(np.float32).tobytes()
        buffers.append(flat)
        accessor_index = len(accessors)
        if min_val is None:
            min_val = data.min(axis=0).tolist() if data.ndim > 1 else [float(data.min())]
            max_val = data.max(axis=0).tolist() if data.ndim > 1 else [float(data.max())]
        acc = Accessor(
            bufferView=len(buffers) - 1,  # 1 buffer per accessor (over-simplified)
            componentType=ct_code,
            count=count,
            type=atype,
            min=min_val,
            max=max_val,
        )
        accessors.append(acc)
        return accessor_index

    # Генерируем animations через build_animations().
    anim_data = build_animations()

    for anim_def in anim_data["animations"]:
        anim_samplers = []
        for sampler_def in anim_def["samplers"]:
            # Input accessor (time, scalar float32).
            in_idx = add_accessor(
                "float32",
                "SCALAR",
                sampler_def["input"],
                min_val=[float(sampler_def["input"].min())],
                max_val=[float(sampler_def["input"].max())],
            )
            out_data = sampler_def["output"]
            # Rotation quaternion — VEC4. Translation — VEC3.
            target_path = anim_def["channels"][len(anim_samplers)]["target"]["path"]
            atype = "VEC4" if target_path == "rotation" else "VEC3"
            out_idx = add_accessor("float32", atype, out_data)
            anim_samplers.append(
                AnimationSampler(
                    input=in_idx,
                    output=out_idx,
                    interpolation=sampler_def["interpolation"],
                )
            )
        # Каналы: каждый ссылается на свой sampler по индексу.
        anim_channels = []
        for ch_i, ch in enumerate(anim_def["channels"]):
            if ch["target"]["node"] not in node_index_by_name:
                raise KeyError(
                    f"Animation target node '{ch['target']['node']}' не найден в body glb. "
                    f"Есть: {sorted(node_index_by_name.keys())}"
                )
            anim_channels.append(
                AnimationChannel(
                    sampler=ch_i,
                    target={"node": node_index_by_name[ch["target"]["node"]], "path": ch["target"]["path"]},
                )
            )
        animations.append(
            Animation(
                name=anim_def["name"],
                channels=anim_channels,
                samplers=anim_samplers,
            )
        )

    # Соберём новый буфер из всех animation buffers.
    if buffers:
        all_bin = b"".join(buffers)
        # Создаём BufferView для каждого accessor с правильным offset.
        # Т.к. мы создали по 1 buffer на accessor, byteOffset каждого = 0,
        # byteLength = длина его данных.
        # Пересоздадим структуру: 1 общий buffer + несколько BufferView.
        total_len = sum(len(b) for b in buffers)
        g.buffers = [Buffer(byteLength=total_len)]
        g.bufferViews = []
        g.accessors = []
        offset = 0
        for acc_i, acc in enumerate(accessors):
            length = len(buffers[acc_i])
            bv_index = len(g.bufferViews)
            g.bufferViews.append(
                BufferView(
                    buffer=0,
                    byteOffset=offset,
                    byteLength=length,
                )
            )
            offset += length
            acc_new = Accessor(
                bufferView=bv_index,
                byteOffset=0,
                componentType=acc.componentType,
                count=acc.count,
                type=acc.type,
                min=acc.min,
                max=acc.max,
            )
            g.accessors.append(acc_new)
        # Бинарный блок при экспорте.
        g.set_binary_blob(all_bin)
        # Заменим animations.
        g.animations = animations

    g.save(str(out_path))

    file_size = out_path.stat().st_size
    stats = {
        "animations": [a.name for a in animations] if animations else [],
        "file_size_bytes": file_size,
        "file_size_kb": round(file_size / 1024, 1),
    }
    if verbose:
        print(f"  ✓ {out_path.name}: {len(animations)} animations, {stats['file_size_kb']} KB")
        for a in animations:
            print(f"    - {a.name}: {len(a.channels)} channels, {len(a.samplers)} samplers")
    return stats


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument(
        "--body",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "robot_body.glb",
        help="Путь к robot_body.glb (source for nodes)",
    )
    p.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "robot_animations.glb",
        help="Путь к robot_animations.glb",
    )
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args()

    if not args.body.exists():
        print(
            f"ERROR: robot_body.glb не найден: {args.body}. " f"Сначала запустите generate_robot_body.py",
            file=sys.stderr,
        )
        return 1

    if not args.quiet:
        print(f"Building {args.out} from {args.body}")
    stats = write_glb_with_animations(args.out, args.body, verbose=not args.quiet)
    if not args.quiet:
        print(json.dumps(stats, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
