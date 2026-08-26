# Avatar Robot — Rob Box Meta Quest WebXR client

Low-poly glTF 2.0 binary (`.glb`) модель 4-колёсной робо-платформы **Rob Box** и 4 анимации (`idle`, `forward`, `turn_left`, `turn_right`) для использования в **Meta Quest** WebXR-клиенте.

Phase 2.1 Captain Bridge (см. [ADR-0032](../../../../adr/0032-meta-quest-webxr-stack-and-assets.md), [research 2026-08-26](../../../../research/2026-08-26-meta-quest-webxr-best-practices.md) §1.2, [Avatar Supervisor ADR-0028 §1.2](../../../../adr/0028-avatar-supervisor.md)).

## Файлы

| Файл | Размер | Назначение |
|---|---|---|
| `robot_body.glb` | ~94 KB | Геометрия + материалы + node hierarchy |
| `robot_animations.glb` | ~12 KB | 4 анимации (rotation/translation tracks) |
| `viewer_test.html` | — | Three.js smoke-test (desktop preview) |
| `scripts/generate_robot_body.py` | — | Source-of-truth generator (STL → glb) |
| `scripts/generate_robot_animations.py` | — | Animations generator |
| `scripts/smoke_test.py` | — | glb validation: size, triangles, node hierarchy, animation targets |
| `scripts/generate_robot_assets.sh` | — | Orchestrator (all / body / anim / test) |
| `robot.blend.json` | — | Source-of-truth manifest (аналог .blend без Blender) |
| `robot.blend.schema.json` | — | JSON Schema для манифеста |
| `meshes/*.stl` | ~3 MB | STL-меши из `rob_box_description` (CAD-export реального робота) |
| `LICENSE` | — | CC0-1.0 |
| `docs/avatar_pipeline.md` | — | Описание pipeline (как это сделано, как пересобрать) |

## Бюджет (ADR-0032 §3.2)

| Категория | Бюджет | Факт | Запас |
|---|---|---|---|
| Avatar `robot_body.glb` | ≤ 500 KB | 93.8 KB | × 5.3 |
| Avatar `robot_animations.glb` | ≤ 100 KB | 11.7 KB | × 8.5 |
| **Avatar total** | ≤ 500 KB | **~106 KB** | × 4.7 |
| Body triangles | ≤ 10 000 | 4 976 | × 2 |

После Phase 2.0 (gltf-transform + Draco + Meshopt) размер упадёт ещё в 3-5×, что даёт огромный запас для будущих доработок (IK, hand-tracking-attachment, particle-effects).

## Источники

### Геометрия
- **STL-меши** — экспорт из CAD (Fusion 360) реального робота Rob Box. Скопированы из `src/rob_box_description/meshes/`. Лицензия: **CC0-1.0** (собственная разработка проекта rob_box_project, выпускается открыто).
- **URDF-иерархия и размеры** — `src/rob_box_description/urdf/rob_box.xacro` (макросы `wheel`, `lidar`, `oak_d_camera`, `usb_camera`). Согласовано с ROS REP-120 (base_footprint — reference frame на полу).
- **Материалы** — `src/rob_box_description/urdf/materials/rob_box_materials.xacro`.

### Анимации
Собственная разработка, CC0-1.0:
- `idle` — breathing sway (±2 см по Z, период 3 сек).
- `forward` — 4 колеса вращаются вокруг локальной оси Y, 30 RPM (π рад/с).
- `turn_left`/`turn_right` — differential drive (левые колёса назад, правые вперёд, скорость ×0.5).

## Генерация (reproducible build)

```bash
cd src/quest-client/assets/raw/avatar

# Полный pipeline (body → animations → smoke-test)
./scripts/generate_robot_assets.sh

# Или по шагам
./scripts/generate_robot_assets.sh body
./scripts/generate_robot_assets.sh anim
./scripts/generate_robot_assets.sh test
```

Требования: Python 3.10+ с `trimesh`, `pygltflib`, `numpy`, `fast-simplification`.

```bash
python3 -m venv .venv
.venv/bin/pip install trimesh pygltflib numpy fast-simplification
```

## Просмотр (Three.js desktop test)

Откройте `viewer_test.html` через любой статический HTTP-сервер (Three.js использует ES modules + CORS):

```bash
cd src/quest-client/assets/raw/avatar
python3 -m http.server 8080
# Откройте http://localhost:8080/viewer_test.html
```

В браузере увидите: 4-колёсного робота, orbit-controls, кнопки переключения анимаций.

## Почему не .blend

`.blend` (бинарный формат Blender) **не генерируется** в headless CI (Blender недоступен, и его установка ~250 МБ). Вместо `.blend` используется:

- **`robot.blend.json`** — декларативный source-of-truth. Описывает всю сцену (узлы, материалы, анимации, lights).
- **Python-генератор** — детерминированный pipeline из STL + манифеста в glTF. Запускается в CI для воспроизводимой сборки.

Если в будущем появится Blender в CI (или ручной workflow), `.blend` можно сгенерировать из `robot.blend.json` через `bpy`-скрипт — структура совместима с `bpy.data.collections`.

## Связанные документы

- [ADR-0027 — Meta Quest AR control](../../../../adr/0027-meta-quest-ar-control.md)
- [ADR-0028 — Avatar Supervisor](../../../../adr/0028-avatar-supervisor.md)
- [ADR-0032 — Meta Quest WebXR client — финальный стек и asset pipeline](../../../../adr/0032-meta-quest-webxr-stack-and-assets.md)
- [Research 2026-08-26 — Meta Quest WebXR best practices](../../../../research/2026-08-26-meta-quest-webxr-best-practices.md)
- Phase 1.5 plan: `docs/plans/2026-08-24-meta-quest-telepresence.md`

## Out of scope (Phase 2.1)

- ❌ Draco / Meshopt compression (Phase 2.0 — gltf-transform pipeline)
- ❌ KTX2 textures (Phase 2.1+ для environment, не avatar — у нас flat color)
- ❌ Runtime binding к WSS cmd_vel (отдельная карточка)
- ❌ IK / foot-placement (Phase 3+)
- ❌ hand-tracking attachment points (Phase 2+)
- ❌ Skeletal animation (упрощённый mesh transform — для робота не нужен skeleton)
