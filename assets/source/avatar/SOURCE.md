# Avatar — Source Asset (raw glTF 2.0)

Этот каталог содержит **исходную (raw) glTF-модель аватара** робота —
4-колёсную платформу, на которой построен весь Meta Quest WebXR Captain Bridge
(ADR-0032 §3.2, kanban t_cdb60035, issue #1677).

Оптимизированный артефакт, который грузится в Quest-клиент, лежит в
`src/rob_box_quest/webxr_client/public/models/avatar/avatar.optimized.glb`
(проходит через Phase 2.0 glTF-asset pipeline — Draco + Meshopt + KHR_mesh_quantization).
**Этот raw-исходник — входная точка для пайплайна, его правка = правка аватара.**

---

## Файл

| Поле | Значение |
|---|---|
| Имя | `avatar.glb` |
| Формат | glTF 2.0 binary (`.glb`) |
| Размер | ~38–40 KB (raw budget ≤ 100 KB, проверено build-avatar.mjs) |
| Геометрия | 9 нод, 9 мешей, 6 материалов |
| Анимации | 5 клипов: `idle`, `drive_forward`, `drive_backward`, `turn_left`, `turn_right` |

---

## Лицензия

**CC0 1.0 Universal — Public Domain Dedication**
<https://creativecommons.org/publicdomain/zero/1.0/>

Вся геометрия — **программно синтезированный код** (Three.js примитивы:
BoxGeometry, CylinderGeometry, SphereGeometry), собранный через
`@gltf-transform/core`. Никакие внешние меши, текстуры или модели
не импортируются. Модель — оригинальный код проекта rob_box_quest,
выпущенный в public domain.

Соответствует CREDITS.md оптимизированного артефакта
(`src/rob_box_quest/webxr_client/public/models/avatar/CREDITS.md`).

---

## Источник / автор

| Поле | Значение |
|---|---|
| Автор | rob_box_quest contributors (товарищ Шифу и команда rob_box) |
| Тип источника | **programmatic synthesis** (генерация кодом) |
| URL скрипта-генератора | `src/rob_box_quest/webxr_client/scripts/build-avatar.mjs` |
| Канонический upstream | https://github.com/krikz/rob_box_project/blob/develop/src/rob_box_quest/webxr_client/scripts/build-avatar.mjs |
| Quaternius Ultimate Character Pack | **НЕ используется** — задача t_cdb60035 первично рассматривала CC0-pакет Quaternius как fallback, но архивы Шифы уже содержат процедурную 4-колёсную платформу (соответствует нашему продукту, ADR-0032 §5.2); сторонний пакет не понадобился |
| Дата генерации (этого файла) | 2026-08-28 |
| Kanban-карточка | t_cdb60035 |
| Issue | https://github.com/krikz/rob_box_project/issues/1677 |

---

## Структура модели (4-колёсная платформа)

| Нода | Геометрия | Назначение |
|---|---|---|
| `chassis` | Box 0.6×0.18×0.8 | корпус, PBR grey-blue |
| `wheel_fl` / `wheel_fr` / `wheel_rl` / `wheel_rr` | Cylinder R=0.16, h=0.12 | 4 колеса, вращаются вокруг X |
| `head` | Box 0.28×0.18×0.18 | камера-сенсор, PBR teal |
| `antenna_mast` | Cylinder R=0.012, h=0.22 | мачта антенны |
| `antenna_tip` | Sphere R=0.025 | emissive LED-маркер |

**Примечание про кости / skinned mesh.** Модель использует **node-level
TRS-анимации** (анимируются transform-нод, на которых висят меши). Классического
skinned mesh (joints + skin weights → vertex deformation под позвоночник/конечности)
**нет** — это не humanoid. Это и соответствует дизайну продукта (4-колёсная
платформа, а не антропоморф). ADR-0032 §3.2 явно относит модель к категории
«avatar» с бюджетом ≤ 500 KB (после оптимизации) и node-анимациями для колёс
и сенсорной головы — этого достаточно для teleop-UX в Quest.

Если в будущем потребуется именно skinned humanoid (например, для Phase 3+ —
антропоморфный робот-аватар), то за основу будет взят **Quaternius Ultimate
Animated Character Pack (CC0)**, как зафиксировано в теле задачи t_cdb60035.

---

## Регенерация raw-исходника

Из корня `src/rob_box_quest/webxr_client`:

```bash
# 1. Сгенерировать _raw/avatar.glb
node scripts/build-avatar.mjs

# 2. Оптимизировать → Draco + Meshopt → _raw/avatar.optimized.glb
npm run gltf:optimize

# 3. Переместить оптимизированный в public/models/avatar/, удалить _raw
node scripts/build-avatar.mjs --publish

# 4. Скопировать НОВЫЙ raw в assets/source/avatar/avatar.glb
mkdir -p ../../../assets/source/avatar
cp public/models/avatar/_raw/avatar.glb ../../../assets/source/avatar/avatar.glb
```

> ⚠️ Шаг 4 — ручной, выполняется воркером после генерации (build-avatar.mjs
> пишет только в `_raw/` под webxr_client). Это намеренно: `assets/source/`
> хранит сырьё **вне** клиентского дерева, чтобы не путать с runtime-артефактами.

---

## Верификация (raw asset)

- [x] Файл `avatar.glb` существует, размер ≤ 100 KB (raw budget).
- [x] glTF 2.0 binary, читается через `@gltf-transform/core` NodeIO без ошибок.
- [x] 9 нод, 9 мешей, 6 материалов.
- [x] 5 ожидаемых анимаций: `idle`, `drive_forward`, `drive_backward`, `turn_left`, `turn_right`.
- [x] Каждая анимация имеет ненулевую длительность (нет пустых клипов).
- [x] CC0 явно указан в этом SOURCE.md.
- [x] Источник (programmatic synthesis) и путь к скрипту-генератору зафиксированы.

Тест-проверка — `src/rob_box_quest/webxr_client/tests/avatar_source.test.ts`
(добавлен в этой же ветке).

---

## Ссылки

- ADR-0027 — Meta Quest / WebXR-аватар (Phase 1 PoC).
- ADR-0028 — Avatar Supervisor (Phase 2 / 3).
- **ADR-0032** — Meta Quest WebXR Stack and Assets (avatar category, performance budget).
- `docs/plans/2026-08-24-avatar-decomposition.md` — эпик-декомпозиция Avatar.
- `src/rob_box_quest/webxr_client/public/models/avatar/CREDITS.md` — атрибуция оптимизированного артефакта.
- `src/rob_box_quest/webxr_client/scripts/build-avatar.mjs` — генератор raw .glb.
- `src/rob_box_quest/webxr_client/scripts/gltf-optimize.mjs` — Phase 2.0 optimization (Draco + Meshopt).
- Kanban t_cdb60035 — эта карточка (raw source).
- Kanban t_1fa6e505 — Phase 2.1 (оптимизированный артефакт).
- Kanban t_770f3299 — следующая фаза после raw source.