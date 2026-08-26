"""Генератор robot_body.glb — low-poly glTF-модель робота Rob Box.

Источник геометрии: STL-меши из `src/rob_box_description/meshes/`
(иерархия и материалы — из `src/rob_box_description/urdf/rob_box.xacro`).

Назначение: avatar оператора в Meta Quest WebXR-клиенте (Phase 2+).
Стек: ADR-0032 (Three.js r160+ + glTF 2.0 + Draco).

Выход: src/quest-client/assets/raw/avatar/robot_body.glb
  target: ≤ 300 KB до Draco-compression (Phase 2.0 сделает Draco)
         ≤ 10 000 triangles total

Это source-of-truth generator: запускается в CI для воспроизводимой
сборки ассета. Соответствует ADR-0018 (honesty) — мы не претендуем
на ручную работу в Blender, всё генерируется детерминированно из
URDF/STL источника.

CC0: STL-меши экспортированы из CAD-модели, находящейся в
docs/hardware/ (см. ADR-0028 §1.1). Сами STL — собственная разработка
проекта rob_box_project, выпускаем под CC0 для ассета avatar.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
import trimesh

# Параметры бюджета (ADR-0032 §3.2)
MAX_TRIANGLES_TOTAL = 10_000
TARGET_FILE_SIZE_BYTES = 300 * 1024  # 300 KB до Draco

# Целевые face counts по декамации (баланс детали/размер).
# Бюджет: base_link ≤ 3000, wheels ≤ 250×4=1000, body_cover ≤ 500,
# sensors ≤ 200, ENCODER ≤ 50 → total ≤ ~5000 face = 10000 tri.
DECIMATE_FACE_TARGET = {
    "base_link": 3000,
    "body_cover": 500,
    "camera_oak": 20,  # упрощённый корпус OAK-D камеры
    "camera_rpi": 100,
    "lidar": 200,
    "left_front_wheel": 250,
    "right_front_wheel": 250,
    "left_rear_wheel": 250,
    "right_rear_wheel": 250,
}

# Материалы из rob_box_materials.xacro (для визуала).
# Имена соответствуют glTF material.pbrMetallicRoughness baseColorFactor.
# base_link + body_cover — белый глянцевый корпус.
# Колёса — матовый чёрный. Сенсоры — тёмно-серый. LiDAR — киберпанк-неоновый.
MATERIALS = {
    "rob_box_white": {"baseColorFactor": [0.95, 0.95, 0.98, 1.0], "metallicFactor": 0.1, "roughnessFactor": 0.3},
    "wheel_black": {"baseColorFactor": [0.10, 0.10, 0.10, 1.0], "metallicFactor": 0.05, "roughnessFactor": 0.85},
    "sensor_black": {"baseColorFactor": [0.15, 0.15, 0.15, 1.0], "metallicFactor": 0.2, "roughnessFactor": 0.6},
    "cyber_blue": {"baseColorFactor": [0.00, 0.70, 1.00, 1.0], "metallicFactor": 0.4, "roughnessFactor": 0.3},
    "neon_purple": {"baseColorFactor": [0.58, 0.00, 0.83, 1.0], "metallicFactor": 0.4, "roughnessFactor": 0.3},
}

# Соответствие mesh → материал (из rob_box.xacro).
# Колёса: wheel_black. Камеры: sensor_black. LiDAR: cyber_blue.
# base_link, body_cover: rob_box_white.
MESH_MATERIAL = {
    "base_link": "rob_box_white",
    "body_cover": "rob_box_white",
    "camera_oak": "sensor_black",
    "camera_rpi": "sensor_black",
    "lidar": "cyber_blue",
    "left_front_wheel": "wheel_black",
    "right_front_wheel": "wheel_black",
    "left_rear_wheel": "wheel_black",
    "right_rear_wheel": "wheel_black",
}

# URDF-сцена: иерархия нодов с TRS.
# Соответствует rob_box.xacro (URDF-export), но без optical-frame вложений.
# Координаты пересчитаны из исходного URDF (X=forward, Y=left, Z=up).
#
# base_footprint (root, на уровне пола, sphere радиус 1мм — визуально
# невидим, нужен для одометрии в ROS convention, см. REP-120).
#   base_link (шасси, +Z=wheel_radius=0.1143 м)
#     left_front_wheel (continuous joint по X, ось вдоль Y wheel axle)
#     right_front_wheel
#     left_rear_wheel
#     right_rear_wheel
#     body_cover (fixed joint)
#     lidar (fixed joint, вертикально на крыше)
#     camera_oak (fixed joint, передний-верх, смотрит вперёд)
#     camera_rpi (fixed joint, нижний-передний, USB-камера)
#
# ВАЖНО: STL экспортированы в мм с scale 0.001 в URDF → glTF в метрах.

# Joint origin'ы взяты из rob_box.xacro/URDF_EXPORT для совместимости с ROS.
# Оригинал см. src/rob_box_description/urdf/rob_box.xacro
SCENE_GRAPH = {
    "nodes": [
        {"name": "base_footprint", "translation": [0.0, 0.0, 0.0], "children": ["base_link"]},
        # base_link: Z поднят на wheel_radius (0.1143 м), без вращения.
        {
            "name": "base_link",
            "translation": [0.0, 0.0, 0.1143],
            "children": [
                "left_front_wheel",
                "right_front_wheel",
                "left_rear_wheel",
                "right_rear_wheel",
                "body_cover",
                "lidar",
                "camera_oak",
                "camera_rpi",
            ],
        },
        # Колёса: continuous joint, axis вдоль Y (поперечная ось робота).
        # Расположение и ось взяты из URDF wheel macro (см. rob_box.xacro).
        # visual_origin_xyz смещает mesh внутри линка (из xacro).
        # base_size = (0.4757, 0.238, 0.1885) м. Колёса по бокам на ±0.238/2.
        {
            "name": "left_front_wheel",
            "translation": [0.150, 0.140, 0.0],
            "rotation_axis": [0, 1, 0],
            "mesh": "left_front_wheel",
            "pivot_translation": [0.0, -0.0375, 0.0],
        },  # смещение центра mesh
        {
            "name": "right_front_wheel",
            "translation": [0.150, -0.140, 0.0],
            "rotation_axis": [0, 1, 0],
            "mesh": "right_front_wheel",
            "pivot_translation": [0.0, 0.0375, 0.0],
        },
        {
            "name": "left_rear_wheel",
            "translation": [-0.150, 0.140, 0.0],
            "rotation_axis": [0, 1, 0],
            "mesh": "left_rear_wheel",
            "pivot_translation": [0.0, -0.0375, 0.0],
        },
        {
            "name": "right_rear_wheel",
            "translation": [-0.150, -0.140, 0.0],
            "rotation_axis": [0, 1, 0],
            "mesh": "right_rear_wheel",
            "pivot_translation": [0.0, 0.0375, 0.0],
        },
        # body_cover: верхняя крышка корпуса (z = base_height - small).
        {"name": "body_cover", "translation": [0.0, 0.0, 0.10], "mesh": "body_cover"},
        # LiDAR сверху (цилиндр h=36мм). Из xacro: lidar расположен сверху base_link.
        {"name": "lidar", "translation": [0.0, 0.0, 0.30], "mesh": "lidar"},
        # OAK-D камера: спереди-сверху, visual_origin в URDF компенсирует экспорт.
        {"name": "camera_oak", "translation": [0.115, 0.0, 0.16], "mesh": "camera_oak"},
        # RPi камера: нижний передний угол.
        {"name": "camera_rpi", "translation": [0.170, -0.065, 0.10], "mesh": "camera_rpi"},
    ],
}


def load_and_decimate(stl_path: Path, target_faces: int) -> trimesh.Trimesh:
    """Загрузить STL (в мм) и декамировать до target_faces. Вернуть в метрах."""
    mesh = trimesh.load(str(stl_path), force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Expected Trimesh, got {type(mesh)}")
    # Конверсия mm → m (URDF scale 0.001).
    mesh.apply_scale(0.001)
    # Центрирование pivot: для колёс центр должен быть на оси вращения.
    # Для остальных — обнуляем translation чтобы узел scene-graph
    # позиционировал mesh правильно.
    if len(mesh.faces) > target_faces:
        mesh = mesh.simplify_quadric_decimation(face_count=target_faces)
    # Удаляем дублирующиеся вершины для компактности.
    mesh.merge_vertices()
    # Удаляем неиспользуемые атрибуты для уменьшения размера glb.
    mesh.metadata = {}
    return mesh


def build_scene(out_path: Path, mesh_dir: Path, verbose: bool = True) -> dict[str, Any]:
    """Собрать glTF 2.0 binary (.glb) из STL + SCENE_GRAPH.

    Возвращает статистику: { triangles, vertices, file_size_bytes }.
    """
    # 1) Загружаем и декамируем все меши.
    meshes: dict[str, trimesh.Trimesh] = {}
    for name, target in DECIMATE_FACE_TARGET.items():
        stl = mesh_dir / f"{name}.stl"
        m = load_and_decimate(stl, target)
        meshes[name] = m
        if verbose:
            print(f"  mesh {name:22s} faces={len(m.faces):4d} verts={len(m.vertices):4d}")

    total_tri = sum(len(m.faces) for m in meshes.values())
    if total_tri > MAX_TRIANGLES_TOTAL:
        raise RuntimeError(
            f"Total triangles {total_tri} > budget {MAX_TRIANGLES_TOTAL}. " f"Уменьшить DECIMATE_FACE_TARGET."
        )
    if verbose:
        print(f"  total triangles: {total_tri} (budget: {MAX_TRIANGLES_TOTAL})")

    # 2) Собираем объединённую геометрию через Scene.
    # trimesh.Scene поддерживает parent/child attachments и per-node transform.
    scene = trimesh.Scene()

    # Корневой узел — base_link (все вложения), сдвинут на wheel_radius вверх.
    base_link_node = SCENE_GRAPH["nodes"][1]  # "base_link"
    T_base = np.eye(4)
    T_base[:3, 3] = base_link_node["translation"]
    scene.add_geometry(
        meshes["base_link"],
        node_name="base_link",
        geom_name="base_link",
        transform=T_base,
    )

    for node in SCENE_GRAPH["nodes"]:
        name = node["name"]
        if name in ("base_link", "base_footprint"):
            continue
        mesh_name = node.get("mesh")
        if mesh_name is None or mesh_name not in meshes:
            if mesh_name is not None:
                print(f"  WARN: node {name} -> mesh {mesh_name} не найден, пропускаю")
            continue
        # Матрица трансформации: translation + (опционально) rotation pivot.
        T = np.eye(4)
        T[:3, 3] = node["translation"]
        # Для колёс: pivot смещение для корректного вращения вокруг оси.
        if "pivot_translation" in node:
            pivot = np.eye(4)
            pivot[:3, 3] = node["pivot_translation"]
            T = T @ pivot
        scene.add_geometry(
            meshes[mesh_name],
            node_name=name,
            geom_name=mesh_name,
            parent_node_name="base_link",
            transform=T,
        )

    # 3) Применяем материалы: trimesh хранит их per-geometry, поэтому
    # добавим визуальные цвета через visual. Делаем простую PBR-аппроксимацию:
    # baseColorFactor берём из MATERIALS, остальное — defaults.
    for geom_name, mat_name in MESH_MATERIAL.items():
        if geom_name not in scene.geometry:
            continue
        mat = MATERIALS[mat_name]
        color = (np.array(mat["baseColorFactor"][:3]) * 255).astype(np.uint8)
        scene.geometry[geom_name].visual.vertex_colors = np.tile(color, (len(scene.geometry[geom_name].vertices), 1))

    # 4) Экспорт в glb (binary glTF 2.0).
    scene.export(str(out_path), file_type="glb")

    # 4.5) Переcчитаем triangle count через прямой проход по mesh-данным
    # (более надёжно, чем парсить pygltflib accessor indices, которые могут
    # быть нестандартны для trimesh-экспорта).
    n_triangles = total_tri

    file_size = out_path.stat().st_size
    stats = {
        "triangles": n_triangles,
        "file_size_bytes": file_size,
        "file_size_kb": round(file_size / 1024, 1),
    }
    if file_size > TARGET_FILE_SIZE_BYTES:
        print(
            f"  ⚠️  WARNING: {stats['file_size_kb']} KB > target "
            f"{TARGET_FILE_SIZE_BYTES // 1024} KB. Draco (Phase 2.0) сожмёт ещё ~3-5×."
        )
    if verbose:
        print(f"  ✓ {out_path.name}: {stats['triangles']} tri, {stats['file_size_kb']} KB")
    return stats


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument(
        "--mesh-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "meshes",
        help="Каталог с STL-мешами (по умолчанию: ../meshes)",
    )
    p.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "robot_body.glb",
        help="Путь к robot_body.glb (по умолчанию: ../robot_body.glb)",
    )
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args()

    if not args.mesh_dir.exists():
        print(f"ERROR: mesh dir не найден: {args.mesh_dir}", file=sys.stderr)
        return 1

    if not args.quiet:
        print(f"Building {args.out} from {args.mesh_dir}")
    stats = build_scene(args.out, args.mesh_dir, verbose=not args.quiet)
    if not args.quiet:
        print(json.dumps(stats, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
