# Captain Bridge — ассеты для Tripo3D (Qwen → Tripo3D)

> **Дата:** 2026-09-26
> **Статус:** только ассеты, без интеграции в сцену.
> **Принцип:** минимум уникальных моделей — один экран на все экраны сцены, пол/стены процедурные.
> **Ссылки:** `docs/architecture/captain-bridge.md`, ADR-0027/0032/0074/0117,
> `src/rob_box_quest/webxr_client/scripts/build_bridge_assets.mjs`.

---

## 1. Контекст сцены

- Мостик = **RVIZ-в-VR**: оператор **стоит** в центре room-scale-сцены
  (спавн `(0,0,0)`, высота глаз 1.6 м), вокруг — floating-панели.
- **Кресла нет, сидячей консоли нет** — оператор стоит и ходит по safe-walk-area.
- Комната 11.6 × 9.12 × 3 м, потолок открыт (`ROOM_W/ROOM_D/ROOM_H`).
- Стиль: тёмный graphite/brushed gunmetal + cyan `#2ec27e` + holo `#44ddff`.

---

## 2. Главный принцип — не плодить дубли

- **Экран — одна модель.** Главный экран, крылья TARS, потолочный экран и
  floating-панели — это **один и тот же** 16:9 экран с разным масштабом/
  поворотом/позицией (задаётся в коде при расстановке). Отдельная «модель
  TARS-крыла» НЕ нужна: отгиб на 50° — это `rotation.y`, а не другой меш.
- **Пол и стены — процедурные** (`build_bridge_assets.mjs`), уже есть. Tripo3D
  для них не нужен — 11-метровую стену с одной 2K-текстурой не вытянуть без
  тайлинга, а процедурная версия уже выглядит нормально.
- **Уникальных Tripo3D-моделей — три**: подиум, экран, голо-проектор.
  Остальное — опционально.

---

## 3. Workflow (на каждую модель)

1. **Qwen** — изображение по промту (раздел 6).
2. **Tripo3D** — Retopo (Topology Quad, Polygon Count **1000–2000**) →
   Smart UV → Texture (**2K**, **Remove Lighting: ON**) → экспорт **GLB**.
3. **Проверка** — `node scripts/gltf-inspect.mjs <file.glb>` (в `webxr_client`).
   Критерии: треугольники ≤ 5 000, файл ≤ 1 MB.
4. **Складываем** в `public/models/environment/_raw/` (gitignored) с именем из
   раздела 7. Пока **без** `gltf:optimize` и без подключения в сцену.

> **Референс (готов):** подиум — 1700 tris / 0.51 MB; экран — 1874 tris /
> 0.26 MB. Это эталон качества и размера.

---

## 4. Единый стиль (префикс)

Вставляйте в начало каждого промта:

```
PBR game asset, futuristic sci-fi command bridge, dark graphite and brushed gunmetal
metal with cyan (#2ec27e) emissive edge accents, clean high-tech hard-surface design,
single isolated object on plain neutral gray studio background, centered, full object
in frame, soft even studio lighting, no text, no logos, no watermark, no background shadow.
```

---

## 5. Список ассетов

| # | Ассет | Кол-во | Статус |
|---|---|---|---|
| 1 | Центральный подиум (hex) | 1 | ✅ готов |
| 2 | Экран 16:9 (универсальная рамка) | 1 модель, N экз. | ✅ готов |
| 3 | Голо-проектор | 2 | ✅ готов |
| 4 | Стоячая консоль-подиум | 0–1 | опционально |
| 5 | Холо-кольцо | 0–1 | опционально |

---

## 6. Промты

### 6.1 Центральный подиум (hex) — ✅ готов

```
Top-down orthographic view of a raised hexagonal platform, dark brushed gunmetal
metal, thin solid cyan (#2ec27e) emissive edge trim around the top perimeter,
recessed center inlay with subtle circuit etching, moderate slab thickness with
beveled side edges, sleek futuristic command bridge asset, PBR, isolated on plain
light-gray background, no shadow, no text, no watermark.
```

### 6.2 Экран 16:9 — ✅ готов

```
{СТИЛЬ} A large widescreen 16:9 landscape display monitor, front orthographic view,
the entire front face is a dark glossy black screen surface, surrounded only by a very
thin rectangular gunmetal bezel with a thin glowing cyan trim along the inner edge,
plain rectangle shape, no rounded corners, no notches, no cutouts, no buttons.
```

**Переиспользование (для этапа интеграции):**

| Экземпляр | Размер | Позиция | Поворот |
|---|---|---|---|
| Главный экран | 4.8×2.7 | (0, 1.5, −3.9) | 0 |
| TARS левый | 4.8×2.7 | левое крыло | +50° (`rotation.y`) |
| TARS правый | 4.8×2.7 | правое крыло | −50° |
| Потолочный | меньше | над головой | наклон (`rotation.x`) |
| Floating (боковые) | 2.5–3 м | по бокам | к оператору |

### 6.3 Голо-проектор (к генерации)

- **Main:** `{СТИЛЬ} A holographic projector, cylindrical dark metal base, small
  glowing cyan emitter on top, translucent cyan holographic cone floating above it,
  three-quarter front view, centered.`
- **Side:** `{СТИЛЬ} Same holographic projector, side orthographic profile, cone
  visible in profile.`
- **Back:** `{СТИЛЬ} Same holographic projector from behind, base and emitter with
  translucent cone.`
- **Top:** `{СТИЛЬ} Same holographic projector, top-down orthographic view, circular
  base and cone from above.`

### 6.4 Стоячая консоль-подиум (опционально)

- **Main:** `{СТИЛЬ} A slim standing command podium, waist-high, slanted dark control
  surface with glowing cyan edge strips and a recessed dark screen on top, front
  orthographic view.`

### 6.5 Холо-кольцо (опционально)

- **Main:** `{СТИЛЬ} A floating holographic ring emitter, thin torus with glowing cyan
  rim and a faint holographic disc inside, front orthographic view.`

---

## 7. Куда складывать

Все GLB кладём в `src/rob_box_quest/webxr_client/public/models/environment/_raw/`
(gitignored — сырые файлы не коммитятся, контракт из `CREDITS.md`).

| # | Ассет | Файл |
|---|---|---|
| 1 | Подиум | `bridge_platform.glb` |
| 2 | Экран | `bridge_screen.glb` |
| 3 | Голо-проектор | `bridge_holo_projector.glb` |
| 4 | Консоль-подиум | `bridge_console_podium.glb` |
| 5 | Холо-кольцо | `bridge_holo_ring.glb` |

---

## 8. Расстановка в сцене (план)

> Координаты — из `captain_bridge.ts` / `build_bridge_assets.mjs`. Оператор стоит
> в `(0,0,0)`, глаза на 1.6 м, смотрит в `-Z` (на главный экран). Единицы — метры.

### 8.1 Подиум (1 шт)

- Позиция: `(0, 0, 0)` — прямо под оператором, на полу.
- Масштаб: диаметр ~2 м (подогнать под модель), высота — как у модели (~0.3 м).
- Роль: центральный hero-объект, «пятак», на котором стоит оператор.

### 8.2 Экран (один меш, переиспользуется)

| Экземпляр | Позиция | Размер | Поворот |
|---|---|---|---|
| Главный экран (front camera) | `(0, 1.5, −3.9)` | 4.8×2.7 | `rotation.y = 0` |
| TARS левый | `(−3.94, 1.5, −2.06)` | 4.8×2.7 | `rotation.y = +50°` |
| TARS правый | `(3.94, 1.5, −2.06)` | 4.8×2.7 | `rotation.y = −50°` |
| Потолочный (ceiling camera) | `(0, 2.85, 0)` | 3.2×2.4 | `rotation.x` вниз (`ceilingScreenPitchRad()`) |
| Floating бок. левый | слева от оператора | ~2.5×1.4 | к оператору (~+75°) |
| Floating бок. правый | справа от оператора | ~2.5×1.4 | к оператору (~−75°) |

> Потолочный экран сейчас — плоский VideoPanel 3.2×2.4 (4:3), рамка-ассет для него
> опциональна: безель 16:9 при таком масштабе чуть исказится, но он тонкий — незаметно.

### 8.3 Голо-проектор (2 шт)

- Позиции: `(−1.0, 0, −1.8)` и `(1.0, 0, −1.8)` — как у процедурных аналогов.
- Высота ~0.9 м; конус-голограмма — процедурный (у модели сверху только эмиттер-диск).

### 8.4 Что НЕ трогаем (процедурное, уже в сцене)

- Пол (6×6 hex-плиток) — остаётся под подиумом.
- Стены + вьюпорты.
- LiDAR-overlay, SLAM-карта на полу, HUD-спрайты (ARM / Status / Voice).
- Панели голосового пайплайна и TTS-picker.

## 9. Что будет позже (отдельный этап, сейчас НЕ делаем)

1. `npm run gltf:optimize` → Draco + Meshopt + WebP → `*.optimized.glb`.
2. Расстановка по §8 + emissive-подсветка cyan-кромок
   (`emissive: 0x2ec27e` — запечённый в текстуру цвет сам не светится).
3. Интеграция в `captain_bridge.ts`.
