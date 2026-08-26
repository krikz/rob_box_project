# Avatar Pipeline — как это сделано

Этот документ объясняет, как из STL-мешей и URDF получается `robot_body.glb` + `robot_animations.glb` для Meta Quest WebXR-клиента. Целевая аудитория — следующий разработчик, которому нужно поправить модель или добавить новые анимации.

## 1. Источники

| Что | Откуда | Зачем |
|---|---|---|
| STL-меши | `src/rob_box_description/meshes/*.stl` (CAD-export) | Геометрия реального робота |
| URDF-иерархия | `src/rob_box_description/urdf/rob_box.xacro` | Размеры, joint origin'ы, типы joints |
| Материалы | `src/rob_box_description/urdf/materials/rob_box_materials.xacro` | Цвета |
| Анимации | Собственная разработка (этот PR) | idle sway + differential drive |

STL-меши в репо `rob_box_project` экспортированы из Fusion 360 в мм. URDF применяет `scale="0.001 0.001 0.001"` для конверсии в метры. Наш генератор делает то же самое через `mesh.apply_scale(0.001)`.

## 2. Decimation (low-poly)

Полные STL-меши слишком тяжёлые для Quest:

| Mesh | Original faces | Target faces | Compression |
|---|---|---|---|
| base_link | 32 286 | 3 000 | × 10.7 |
| left_front_wheel | 6 584 | 250 | × 26.3 |
| right_front_wheel | 6 584 | 250 | × 26.3 |
| left_rear_wheel | 6 584 | 250 | × 26.3 |
| right_rear_wheel | 6 584 | 250 | × 26.3 |
| body_cover | 2 432 | 500 | × 4.9 |
| camera_rpi | 436 | 100 | × 4.4 |
| camera_oak | 12 | 20 | × 0.6 (уже low-poly) |
| lidar | 396 | 200 | × 2.0 |
| **Total** | **~59 500** | **~4 800** | **× 12.4** |

Decimation выполняется через `trimesh.simplify_quadric_decimation()` (использует `fast-simplification` C-библиотеку для скорости). Quadric-метод лучше vertex-cluster для сохранения формы (важно для колёс — нужна круглая форма).

## 3. Node hierarchy

Соответствует URDF, но без optical-frame вложений (они не нужны для визуала):

```
base_footprint (root, REP-120 reference frame, sphere r=0.001)
└── base_link (шасси, Z поднят на wheel_radius=0.1143 м)
    ├── left_front_wheel   (continuous joint, axis=Y)
    ├── right_front_wheel  (continuous joint, axis=Y)
    ├── left_rear_wheel    (continuous joint, axis=Y)
    ├── right_rear_wheel   (continuous joint, axis=Y)
    ├── body_cover         (fixed joint, верхняя крышка)
    ├── lidar              (fixed joint, сверху по центру)
    ├── camera_oak         (fixed joint, спереди-сверху)
    └── camera_rpi         (fixed joint, снизу-спереди)
```

**base_footprint** намеренно не визуализирован (пустышка) — это ROS reference frame, не mesh.

### Pivot для колёс

Колёса в URDF используют `continuous` joint с осью `(0, 1, 0)`. Чтобы вращение в glTF-анимациях выглядело корректно (вокруг центра колеса, а не вокруг его mesh-origin), в `SCENE_GRAPH` для каждого колеса задан `pivot_translation`, который смещает mesh вглубь оси вращения.

## 4. Анимации

4 анимации, все loop'ятся:

- **idle** (3 сек) — `base_link.translation.z` дышит ±2 см (синусоида). Колёса неподвижны. Имитация живого робота.
- **forward** (1 сек) — все 4 колеса вращаются вокруг Y. Угловая скорость: 30 RPM = π рад/с, знак «−» (по правилу правой руки, thumb=+Y → положительное вращение = «назад», нам нужно «вперёд»).
- **turn_left** (1 сек) — дифференциальный привод: левые колёса назад (−π/2 рад/с), правые вперёд (+π/2 рад/с). Уменьшенная скорость для in-place разворота.
- **turn_right** — симметрично.

Каждая анимация использует **LINEAR** интерполяцию (по умолчанию в glTF 2.0). Для колёс линейная интерполяция quaternion'ов визуально неотличима от SLERP (при малых dt), но даёт меньший размер файла.

## 5. Формат файлов

`robot_body.glb` — один binary glTF 2.0 со всей геометрией, материалами и node hierarchy. **Без animations** (вынесены отдельно, чтобы можно было перезагружать анимации без перезагрузки мешей).

`robot_animations.glb` — glTF 2.0 binary с 4 animation clips. Содержит **только** node skeleton (для target references) и animations data. Без geometry (это даёт 12 KB вместо 106 KB).

> Примечание: Three.js loader загружает `robot_animations.glb` для `AnimationMixer.clipAction(clip)`, при этом mesh-нода берётся из уже загруженного `robot_body.glb`. Node names должны совпадать в обоих файлах.

## 6. Почему НЕ Blender

`.blend` (Blender binary) **не генерируется** в headless CI:
1. Blender ~250 МБ, тяжело ставить в CI-образ.
2. Детерминированная сборка из JSON+STL даёт reproducible builds, что важнее для CI/CD.
3. Наш pipeline — это код, а не клики мышкой; его можно code-review'ить.

**`robot.blend.json`** — декларативный source-of-truth, описывающий всю сцену. Когда Blender появится в CI (или в ручном workflow дизайнера), `.blend` можно сгенерировать из этого JSON через `bpy`-скрипт (`bpy.data.collections[...]`, `bpy.data.materials[...]`).

## 7. Размер и производительность (ADR-0032 §3.2, §3.3)

| Метрика | Budget | Факт | Запас |
|---|---|---|---|
| `robot_body.glb` размер | ≤ 300 KB (до Draco) | 93.8 KB | × 3.2 |
| `robot_animations.glb` размер | ≤ 100 KB | 11.7 KB | × 8.5 |
| Avatar total | ≤ 500 KB | 105.5 KB | × 4.7 |
| Body triangles | ≤ 10 000 | 4 976 | × 2 |
| Body draw calls | ≤ 1500 (Quest 3) | 9 (одна нода = один mesh) | × 167 |
| Cold-start time | ≤ 3 s | (нужно измерить на устройстве) | — |

После Phase 2.0 (Draco + Meshopt) размер `robot_body.glb` упадёт до ~20-30 KB (сжатие geometry ~3-5×). Это оставит огромный запас для будущих фич (IK, hand-tracking attachments).

## 8. Как добавить новую анимацию

Пример: добавить `wave` (робот машет "рукой", но у нас нет рук — добавим sway body в стороны).

1. Открыть `scripts/generate_robot_animations.py`.
2. В `build_animations()` добавить новый блок:
   ```python
   wave_channels = []
   wave_samplers = []
   sway_t = np.linspace(0, 2.0, 21, dtype=np.float32)
   sway_y = 0.05 * np.sin(2 * np.pi * sway_t / 2.0)
   base_xyz = np.zeros((len(sway_t), 3), dtype=np.float32)
   base_xyz[:, 1] = sway_y  # Y axis sway
   base_xyz[:, 2] = 0.1143
   wave_channels.append({"sampler": 0, "target": {"node": "base_link", "path": "translation"}})
   wave_samplers.append({"input": sway_t, "output": base_xyz, "interpolation": "LINEAR"})
   animations.append({"name": "wave", "channels": wave_channels, "samplers": wave_samplers})
   ```
3. Запустить `./scripts/generate_robot_assets.sh`.
4. Добавить кнопку в `viewer_test.html`.

## 9. Как поменять геометрию

Если в CAD обновился STL:

1. Скопировать новый STL в `meshes/`.
2. Обновить `DECIMATE_FACE_TARGET` в `generate_robot_body.py` если нужна другая детализация.
3. Обновить `target_faces` в `robot.blend.json` (для согласованности).
4. Запустить `./scripts/generate_robot_assets.sh`.

## 10. Тестирование

`scripts/smoke_test.py` проверяет:

- Файлы читаются через `pygltflib` (валидный JSON+bin).
- Размер в пределах бюджета.
- Все требуемые nodes присутствуют (`base_link`, 4 wheels, body_cover, lidar, cameras).
- Body triangle count в пределах бюджета.
- Animations: 4 штуки, каналы нацелены на правильные ноды, paths соответствуют типу анимации (idle = translation, остальные = rotation).

Smoke-test запускается автоматически в `./scripts/generate_robot_assets.sh` и вручную через `./scripts/generate_robot_assets.sh test`.

Полноценный browser-test (Three.js loader + рендер) — через `viewer_test.html`. Это не CI-test, а developer-tool: разработчик открывает его после изменений, чтобы убедиться, что анимации играют и модель выглядит правильно.

## 11. Файлы в репо

```
src/quest-client/
└── assets/
    └── raw/
        └── avatar/
            ├── README.md
            ├── LICENSE
            ├── robot_body.glb         ← артефакт (committed)
            ├── robot_animations.glb   ← артефакт (committed)
            ├── robot.blend.json       ← source-of-truth (committed)
            ├── robot.blend.schema.json
            ├── viewer_test.html       ← developer preview tool
            ├── meshes/                ← STL из rob_box_description
            │   ├── base_link.stl
            │   ├── body_cover.stl
            │   ├── camera_oak.stl
            │   ├── camera_rpi.stl
            │   ├── lidar.stl
            │   ├── left_front_wheel.stl
            │   ├── right_front_wheel.stl
            │   ├── left_rear_wheel.stl
            │   └── right_rear_wheel.stl
            ├── scripts/
            │   ├── generate_robot_body.py
            │   ├── generate_robot_animations.py
            │   ├── smoke_test.py
            │   └── generate_robot_assets.sh
            └── docs/
                ├── rob_box.xacro           ← reference URDF
                ├── rob_box_materials.xacro ← reference materials
                └── avatar_pipeline.md       ← этот файл
```

## 12. Известные ограничения

- **Нет `.blend`** — при ручной доработке дизайнеру нужно использовать Blender локально и экспортировать в glb через "File → Export → glTF 2.0". Source-of-truth в JSON может быть переписан, чтобы отразить ручные изменения.
- **Materials через vertex colors**, не через KTX2-текстуры. После Phase 2.0 можно добавить текстуры (Quaternius PBR textures для робота) — сейчас flat color даёт меньший размер и проще pipeline.
- **Нет hand-tracking attachment points** — будут добавлены в Phase 2.2+ когда появится Avatar Supervisor binding.
- **Нет shadow casting** — оптимизация для Quest. Phase 2+ может добавить, если тени нужны для UX (например, для VR-passability tests).
