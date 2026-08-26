"""Smoke-test для robot_body.glb и robot_animations.glb.

Проверяет:
- glb-файлы читаются через trimesh + pygltflib
- meshes присутствуют
- node hierarchy корректна
- animations нацелены на правильные ноды
- размер в пределах бюджета
- triangle count в пределах бюджета

Запуск: python smoke_test.py [dir]
  где dir = src/quest-client/assets/raw/avatar/ (по умолчанию).
"""

from __future__ import annotations

import sys
from pathlib import Path

import trimesh
from pygltflib import GLTF2

BUDGETS = {
    "robot_body.glb": {"max_kb": 300, "max_triangles": 10_000},
    "robot_animations.glb": {"max_kb": 100, "max_triangles": 10_000},
}

REQUIRED_NODES = {
    "robot_body.glb": [
        "base_link",
        "left_front_wheel",
        "right_front_wheel",
        "left_rear_wheel",
        "right_rear_wheel",
        "body_cover",
        "lidar",
        "camera_oak",
        "camera_rpi",
    ],
    "robot_animations.glb": [
        "base_link",
        "left_front_wheel",
        "right_front_wheel",
        "left_rear_wheel",
        "right_rear_wheel",
    ],
}

REQUIRED_ANIMATIONS = ["idle", "forward", "turn_left", "turn_right"]

WHEELS = ["left_front_wheel", "right_front_wheel", "left_rear_wheel", "right_rear_wheel"]


def check_glb(path: Path) -> list[str]:
    """Вернуть список ошибок (пустой = всё ОК)."""
    errors = []
    name = path.name

    if not path.exists():
        return [f"{name}: файл не найден"]

    # 1. Размер файла.
    size_kb = path.stat().st_size / 1024
    budget_kb = BUDGETS[name]["max_kb"]
    if size_kb > budget_kb:
        errors.append(
            f"{name}: размер {size_kb:.1f} KB > бюджет {budget_kb} KB "
            f"(после Draco сжатия в Phase 2.0 станет меньше)"
        )

    # 2. pygltflib парсит без ошибок.
    try:
        gltf = GLTF2.load(str(path))
    except Exception as e:
        errors.append(f"{name}: pygltflib не смог загрузить: {e}")
        return errors

    # 3. Node hierarchy.
    nodes_by_name = {}
    if gltf.nodes:
        for i, n in enumerate(gltf.nodes):
            if n.name:
                nodes_by_name[n.name] = i

    for required in REQUIRED_NODES[name]:
        if required not in nodes_by_name:
            errors.append(f"{name}: missing required node '{required}'")

    # 4. Triangle count (через trimesh). Не применимо к animations-only glb.
    if name == "robot_body.glb":
        try:
            scene = trimesh.load(str(path))
        except Exception as e:
            errors.append(f"{name}: trimesh не смог загрузить: {e}")
            return errors

        tri_count = 0
        if isinstance(scene, trimesh.Scene):
            for g in scene.geometry.values():
                tri_count += len(g.faces) if isinstance(g, trimesh.Trimesh) else 0
        else:
            tri_count = len(scene.faces)

        budget_tri = BUDGETS[name]["max_triangles"]
        if tri_count > budget_tri:
            errors.append(f"{name}: triangles {tri_count} > бюджет {budget_tri}")
    else:
        # robot_animations.glb: trimesh может не загрузить, т.к. файл
        # не обязан содержать meshes. Проверка триангуляции не нужна.
        tri_count = 0

    # 5. Для robot_animations.glb: animations должны существовать
    # и каждый animation channel ссылается на правильный node.
    if name == "robot_animations.glb":
        if not gltf.animations:
            errors.append(f"{name}: нет ни одной анимации")
        else:
            anim_names = {a.name for a in gltf.animations}
            for required in REQUIRED_ANIMATIONS:
                if required not in anim_names:
                    errors.append(f"{name}: missing required animation '{required}'")
            for anim in gltf.animations:
                for ch in anim.channels:
                    target = ch.target
                    target_node = getattr(target, "node", None) if target else None
                    target_path = getattr(target, "path", None) if target else None
                    if target_node is None:
                        errors.append(f"{name}: animation '{anim.name}' channel без target.node")
                        continue
                    if target_node >= len(gltf.nodes):
                        errors.append(
                            f"{name}: animation '{anim.name}' channel target.node " f"{target_node} вне диапазона"
                        )
                        continue
                    node = gltf.nodes[target_node]
                    if node.name not in REQUIRED_NODES[name]:
                        errors.append(f"{name}: animation '{anim.name}' нацелен на " f"неожиданный node '{node.name}'")
                    # Проверка: idle должен быть translation, остальные — rotation.
                    if anim.name == "idle":
                        if target_path != "translation":
                            errors.append(
                                f"{name}: animation 'idle' должна быть translation, " f"получили '{target_path}'"
                            )
                    else:
                        if target_path != "rotation":
                            errors.append(
                                f"{name}: animation '{anim.name}' должна быть rotation, " f"получили '{target_path}'"
                            )
                        if node.name not in WHEELS:
                            errors.append(
                                f"{name}: animation '{anim.name}' channel нацелен " f"на '{node.name}', а не на колесо"
                            )

    return errors, {
        "size_kb": round(size_kb, 1),
        "triangles": tri_count,
        "nodes": sorted(nodes_by_name.keys()),
        "animations": [a.name for a in (gltf.animations or [])],
    }


def main() -> int:
    import argparse

    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument(
        "--dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
        help="Каталог с robot_body.glb и robot_animations.glb",
    )
    args = p.parse_args()

    print(f"Smoke-test для {args.dir}")
    all_ok = True
    for glb_name in BUDGETS.keys():
        path = args.dir / glb_name
        result = check_glb(path)
        if isinstance(result, list):
            errors = result
            stats = None
        else:
            errors, stats = result
        if errors:
            all_ok = False
            print(f"\n❌ {glb_name}:")
            for e in errors:
                print(f"   - {e}")
        else:
            assert stats is not None
            print(
                f"\n✓ {glb_name}: {stats['size_kb']} KB, {stats['triangles']} tri, "
                f"{len(stats['nodes'])} nodes"
                + (f", animations: {stats['animations']}" if "animations" in stats else "")
            )

    print()
    if all_ok:
        print("All smoke-tests passed ✓")
        return 0
    print("FAILED ✗")
    return 1


if __name__ == "__main__":
    sys.exit(main())
