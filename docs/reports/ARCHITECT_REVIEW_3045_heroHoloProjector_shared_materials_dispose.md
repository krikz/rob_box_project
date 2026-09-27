# Архитектурный ревью #3045 — потенциальный двойной dispose shared materials у heroHoloProjector/heroHoloRight

**Ревьюер:** architect
**Дата:** 2026-09-27
**Issue:** krikz/rob_box_project#3045
**Kanban:** t_405d193d
**Retro-key:** component-review-src-rob_box_quest-2026-W39
**Severity:** MEDIUM (latent — не падает сейчас, но инвариант ownership нарушен)

## 1. Резюме (TL;DR)

Находка **подтверждается** в `origin/develop` (commit `b73de1fa6` «feat(quest): place Tripo3D hero props in Captain Bridge scene»):

- `captain_bridge.ts:706` — `heroHoloRight = g.heroHoloProjector.clone(true)` создаёт копию группы, **но материалы остаются shared** (это документированное поведение `THREE.Object3D.clone(recursive)` в three.js ≥ r144 — параметр `cloneMaterials` для `Object3D` отсутствует).
- `bridge_assets.ts:345-353` — `dispose()` обходит `Object.values(groups)`, и поскольку `groups.heroHoloProjector` (строка 280) указывает на **тот же** `THREE.Group`, чьи материалы проецируются на `heroHoloRight`, первый `mat.dispose()` уже освобождает GPU-ресурс.
- `captain_bridge.ts:1279-1286` — повторный `traverse + dispose geometry/material` для `heroHoloRight` вызывает `mat.dispose()` на уже освобождённых материалах (no-op в three.js, но leak-warning в WebGL Inspector и потенциальный runtime-error при попытке отрендерить меш после dispose).

**Решение (рекомендация архитектора):** Вариант A — глубокий клон материалов сразу после `clone(true)`. Вариант B отвергаем как более рискованный (см. §4).

## 2. Верификация (raw-evidence)

### 2.1 Где живёт код (на момент ревью)

```bash
$ git rev-parse origin/develop
b946500fece1973df01c79c4e67728b40f027902
$ git merge-base --is-ancestor b73de1fa origin/develop
exit: 0   # коммит в develop
```

### 2.2 captain_bridge.ts:706 (origin/develop)

```ts
// bridge_assets.ts:280 → groups.heroHoloProjector = heroHoloProjector (тот же THREE.Group)
// captain_bridge.ts:704-709
if (g.heroHoloProjector) {
  placeOnFloor(g.heroHoloProjector, -1, -1.8, undefined, 0.9);
  heroHoloRight = g.heroHoloProjector.clone(true);   // ← СТРОКА 706, materials SHARED
  heroHoloRight.name = "bridge_holo_projector_right";
  placeOnFloor(heroHoloRight, 1, -1.8, undefined, 0.9);
  scene.add(heroHoloRight);
}
```

### 2.3 bridge_assets.ts:345-353 (origin/develop)

```ts
dispose(): void {
  for (const g of Object.values(groups)) {            // ← groups содержит heroHoloProjector
    if (!g) continue;
    g.traverse((obj) => {
      const mesh = obj as THREE.Mesh;
      mesh.geometry?.dispose?.();
      const mat = mesh.material as THREE.Material | undefined;
      if (mat && "dispose" in mat && typeof mat.dispose === "function") mat.dispose();
    });
  }
  // ... navGroup traverse, envMap?.dispose(), draco.dispose()
}
```

### 2.4 captain_bridge.ts:1279-1286 (origin/develop)

```ts
environment?.dispose();
if (heroHoloRight) {                                  // ← клон живёт отдельно от groups
  heroHoloRight.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    mesh.geometry?.dispose?.();
    const mat = mesh.material as THREE.Material | undefined;
    if (mat && "dispose" in mat && typeof mat.dispose === "function") mat.dispose();
  });
}
```

### 2.5 Доказательство «shared materials» (three.js семантика)

- `THREE.Object3D.clone(recursive=true)` — three.js r170 (наша зависимость `"three": "^0.170.0"` в `webxr_client/package.json`).
- Поведение: клонируются `position/quaternion/scale/visible/name`, дочерние объекты — рекурсивно. **Материалы НЕ клонируются** — это поведение по умолчанию; для глубокого клона нужен явный `mesh.material = mesh.material.clone()` после `traverse`.
- Источник: three.js docs `Object3D.copy(source, recursive)`, `SkinnedMesh/Mesh` constructor — параметр `cloneMaterials` для Object3D отсутствует (добавлен только в SkinnedMesh/Mesh через internal `clone`).
- Симптом в runtime: WebGL Inspector → «Material is already disposed», Chrome DevTools → disposed-in-use warning.

### 2.6 Тестовое покрытие (что нужно добавить воркеру)

В `src/rob_box_quest/webxr_client/tests/` нет теста на `placeHeroProps` / dispose-order. Существующий `bridge_environment.test.ts` покрывает только GLB-файлы, meta и CREDITS.md (после `b73de1fa` — 8 файлов вместо 5). Новый тест должен проверять:

1. После `placeHeroProps(env)` материалы в `g.heroHoloProjector.traverse()` и `heroHoloRight.traverse()` — **разные объекты** (deep-clone).
2. `heroHoloRight.parent === scene` (не groups).
3. После `dispose()` — `heroHoloRight` остаётся в `scene` до явного `scene.remove(heroHoloRight) + heroHoloRight.traverse(dispose)` (см. §3 — нужна правка порядка).

## 3. Рекомендуемый фикс (Вариант A — дополненный)

```ts
// captain_bridge.ts, в placeHeroProps(), после clone(true):
if (g.heroHoloProjector) {
  placeOnFloor(g.heroHoloProjector, -1, -1.8, undefined, 0.9);
  heroHoloRight = g.heroHoloProjector.clone(true);
  // Deep-clone materials: Object3D.clone(recursive) does NOT clone materials
  // by default. Without this, environment.dispose() (which iterates
  // groups.heroHoloProjector) disposes the shared materials, then the
  // later heroHoloRight traverse calls mat.dispose() on already-disposed
  // materials (no-op but generates WebGL Inspector warnings and risks
  // runtime errors if the mesh is rendered after the first dispose).
  heroHoloRight.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (mesh.material) {
      const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
      mesh.material = mats.map((m) => m.clone()) as THREE.Material | THREE.Material[];
    }
  });
  heroHoloRight.name = "bridge_holo_projector_right";
  placeOnFloor(heroHoloRight, 1, -1.8, undefined, 0.9);
  scene.add(heroHoloRight);
}
```

### 3.1 Дополнительно — порядок dispose в `captain_bridge.ts:1263+`

Текущая последовательность:

```ts
environment?.dispose();        // ← dispose'ит и heroHoloProjector, и heroHoloRight (после фикса — нет, материалы разные)
if (heroHoloRight) { ... }    // ← после фикса — безопасно, но heroHoloRight остаётся в scene
```

Рекомендация: **вынести `scene.remove(heroHoloRight)` перед traverse + dispose**, иначе клон остаётся в графе сцены до следующего GC:

```ts
if (heroHoloRight) {
  scene.remove(heroHoloRight);   // ← отвязать от графа ДО dispose
  heroHoloRight.traverse((obj) => { /* dispose */ });
}
```

### 3.2 Почему не тривиальный `mesh.material.clone()` без traverse

- `THREE.Group.clone(true)` создаёт **новые** дочерние `Mesh` объекты, но **сохраняет ссылки** на исходные `material`/`geometry`. Это by design — клонировать ресурсы GPU по умолчанию небезопасно (если не знаешь, что они не shared).
- `traverse` гарантирует обход всех вложенных мешей (включая GLB-вложенные группы).

## 4. Почему не Вариант B

Вариант B предлагал: «пусть `heroHoloRight` добавляется в `groups` bridge_assets, тогда общий `environment.dispose()` обходит обе копии один раз».

**Аргументы против:**

1. **Нарушает ownership-границу.** Сейчас `bridge_assets.ts` отвечает за «что загружено из GLB», а `captain_bridge.ts` — за «как расставлено в сцене». Размещение `heroHoloRight` (transform по bounding box, clone, scene.add) — это семантика `captain_bridge`, а не loader'а. Перенос в `bridge_assets` размывает эту границу.
2. **Двойное позиционирование в loader'е неудобно.** `bridge_assets.ts` не знает про оператора (origin (0,0,0)) и расстановку — это runtime-параметры из `CaptainBridgeOptions`. Придётся тащить их через opts, что уже есть в `captain_bridge.ts`.
3. **Удваивает количество GLB-объектов в `groups`** без выгоды (loader и так не освобождает материалы отдельно — только через общий `dispose()`).
4. **Не решает root cause.** Root cause — «два независимых owner'а делают `dispose()` на shared материалах». Вариант B **переносит** проблему в loader, но не устраняет (если кто-то добавит третий клон, баг вернётся).

Вариант A — **фикс на уровне ownership у клона**, что правильнее: клон должен владеть своими ресурсами.

## 5. Альтернатива, рассмотренная и отвергнутая

**Альтернатива X — флаг «dispose через WeakSet»:** завести `Set<Material> disposedMaterials` в `bridge_assets.ts` и проверять в traverse. **Отвергнуто**: stateful workaround вместо fix'а ownership'а; усложняет dispose-логику; не помогает, если кто-то добавит новый источник материалов.

## 6. Acceptance для воркера

Воркер (профиль `frontend` или `backend`) берёт карточку после `kanban_complete` от архитектора. Минимальный чек-лист:

- [ ] Изменён `placeHeroProps()` в `captain_bridge.ts` — добавлен deep-clone materials (см. §3).
- [ ] Добавлен `scene.remove(heroHoloRight)` перед traverse + dispose в `dispose()` `captain_bridge.ts`.
- [ ] Добавлен vitest-тест в `src/rob_box_quest/webxr_client/tests/` (имя: `place_hero_props_dispose.test.ts` или расширение существующего `bridge_environment.test.ts`):
  - [ ] `heroHoloRight.traverse()` после `placeHeroProps()` возвращает материалы, **отличные** от `g.heroHoloProjector.traverse()`.
  - [ ] После `environment.dispose() + heroHoloRight.traverse(dispose)` — обход не падает, материалы disposed (проверка через `material.disposed` или stub-spy).
  - [ ] `scene.children` содержит `heroHoloRight` до `scene.remove(heroHoloRight)`, и **не содержит** после.
- [ ] `npm run typecheck` (tsc --noEmit) — exit 0.
- [ ] `npm run test` (vitest run) — все тесты зелёные.
- [ ] PR base = `develop`, branch = `z-{role}/3045-hero-holo-dispose-fix` (или подобное).
- [ ] PR description содержит: `Closes #3045` + ссылку на этот документ + raw-вывод `vitest run` и `tsc --noEmit`.
- [ ] **e2e:** если воркер считает, что нужно e2e на Quest — добавить `## e2e` блок в PR body. Иначе достаточно unit-теста + ручной проверки в Quest-эмуляторе через `npm run dev`.

## 7. Что НЕ нужно делать

- **НЕ менять GLB-контент** (assets уже в `public/models/environment/`).
- **НЕ менять `bridge_assets.ts` dispose-логику** для остальных groups (floor/walls/props/nav/occluders) — там нет клонов, проблемы нет.
- **НЕ менять `groups` interface** (можно оставить `heroHoloProjector?` опциональным — null-guard уже есть в `placeHeroProps`).
- **НЕ переименовывать `bridge_holo_projector_right`** — это user-facing `name` для DevTools-инспекции.

## 8. Метаданные ревью

- **Конфликт-hotspot:** `captain_bridge.ts` (1284 строки на момент ревью, ~1335 на develop) — большой файл, частые слияния. Если воркер увидит конфликт в районе строк 700-720 (placeHeroProps) или 1263-1290 (dispose) — это норма для этого файла, не сигнал проблемы.
- **Связанные issues:** #3045 (этот), #3049 (CREDITS.md stale size — недавно закрыт), Phase 2.1 environment из #1677.
- **Дополнительный context для воркера:** commits `27800a3d1` «gltf pipeline — best-effort texture compression, skip _raw in verify», `ecfc454ba` «add Captain Bridge Tripo3D hero props (pedestal, screen, holo projector)», `b73de1fa6` «place Tripo3D hero props in Captain Bridge scene». Между ними 2 коммита, всё в develop.
- **Архитектурное замечание для будущего:** GLB-loader `bridge_assets.ts` имеет 5 групп в `groups` + 3 hero-group'а (всего 8) — это уже на грани читаемости. Если добавится ещё один набор props — выделить hero-loader в отдельный `bridge_hero_assets.ts` (по симметрии с `bridge_assets.ts`). **Не блокер для #3045**, но фиксируем в backlog.

---

*Архитектор не правит код по правилу «не делай руками» (ADR-0014, AGENTS.md). Ревью — proposal, реализация — воркеру.*
