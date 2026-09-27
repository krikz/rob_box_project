# AI-generated asset handling — operational procedure

> Процедура работы с AI-generated assets (mesh/texture/audio) в репозитории.
> Обязательная в дополнение к ADR-0032 §1.2 R9 (CC0-or-own).
> Architectural basis: ADR-0136.

## Scope

AI-generated asset — это любой бинарный ассет в репозитории, произведённый
генеративной моделью:

| Тип | Примеры генераторов |
|---|---|
| 3D mesh | Tripo3D, Meshy, Luma Genie, text-to-3D |
| 2D texture / sprite | MiniMax image-01, SDXL, Midjourney |
| Audio / voice | MiniMax TTS, voice cloning (RVC, So-VITS) |
| HDR / environment | Генеративные HDR-генераторы (Poly Haven НЕ входит — Poly Haven ручной capture) |

Не входит:

- Procedural synthesis из primitives (ADR-0032 §3.2 Captain Bridge pattern)
  — это собственный код, не AI-generated.
- CC0 / CC-BY ассеты скачанные из публичных источников — покрыты §"Adding
  non-CC0 assets" в CREDITS.md (existing).

## Default policy

**Запрет** на публичную редiстрибуцию AI-generated ассетов, пока их
юридический статус не settled (что соответствует ADR-0032 R9 буквально).

**Exception** через явный sign-off (см. ниже). Разработка (сборка,
тестирование, deploy в private/internal контурах) — НЕ запрещена.

## Когда вы генерируете AI-asset

### Шаг 1. ДО генерации

Спросите себя: «Можно ли получить тот же результат через procedural
synthesis (ADR-0032 §3.2) или CC0 source?» Если да — не используйте
AI-generated. Если нет (например, нужен уникальный hero prop и нет
времени/возможности на ручное моделирование) — продолжайте.

### Шаг 2. Зафиксировать provenance

Для каждого AI-generated ассета запишите:

- **Генератор** (Tripo3D v3, MiniMax image-01, etc.)
- **Версия модели / дата** (если известно — иначе «latest as of YYYY-MM-DD»)
- **Reference source** (text prompt, reference image, voice sample)
- **Дата генерации** (commit date или дата прогона)
- **Размер финального ассета** (после `npm run gltf:optimize` или аналога)
- **Путь в репо** после commit

### Шаг 3. ДО `git commit` — обновить tracking issue

В tracking issue (для текущего area — issue #3051 для `rob_box_quest`)
добавить checkbox-строку по шаблону:

```markdown
- [ ] **<generator> <short-description>** — `<path/to/asset.ext>`
      (committed YYYY-MM-DD in commits `<sha1>` + `<sha2>`)
  - Reference: <reference source — be specific>
  - License status: **pending** (legal review by товарищ Шифу)
  - Resolution target: <milestone or "before public release">
```

Если tracking issue для area ещё не существует — создать по шаблону
из ADR-0136 §3.2 с labels `legal` + `ai-asset` + `area:<component>` + `process`.

### Шаг 4. ДО `git commit` — добавить trailer в commit body

```bash
git commit --trailer '<!-- ai-asset-policy: pending -->' ...
```

Возможные значения:

| Значение | Семантика |
|---|---|
| `yes` | Settled license (явное решение товарища Шифу зафиксировано в tracking issue) — публичная редiстрибуция ОК |
| `pending` | Юридический статус unsettled — публичная редiстрибуция ЗАПРЕЩЕНА |
| `removed` | Ассет удалён из репо (но commit остаётся в истории) |

### Шаг 5. ДО `git commit` — обновить CREDITS.md

В per-area `CREDITS.md` (или в `README.md` если CREDITS.md не существует
для area) добавить строку по шаблону:

```markdown
### <short-description> (<generator>, YYYY-MM-DD)

- Path: `public/models/environment/<asset>.optimized.glb`
- Generator: Tripo3D v3 (text-to-3D), reference: <Qwen image>
- License status: **pending** — see meta-issue #<N>
- Owner: товарищ Шифу
- Resolution target: <milestone>

<!-- Per ADR-0136. Do NOT publicly redistribute until 'yes'. -->
```

### Шаг 6. После merge — обновить tracking issue

В tracking issue изменить checkbox статус с `pending` на `yes` (с датой
решения товарища Шифу и явной записью принятого риска/лицензии) **или**
на `removed` (если ассет заменён/удалён).

## Release checklist (для каждого public release tag)

Прежде чем выпустить публичный tag / GitHub release / Docker Hub push:

```bash
# 1. Найти все open AI-asset issues
gh issue list --label ai-asset --state open

# 2. Убедиться что каждая запись имеет license status != 'pending'
#    ИЛИ явный sign-off override от товарища Шифу

# 3. Если есть pending — STOP. Либо:
#    - товарищ Шифу явно подтверждает carry-forward (новый milestone);
#    - ассет заменяется/удаляется до release.
```

Эта проверка — **manual** до момента, когда CI guard (ADR-0136 §6
future work) будет реализован.

## Когда ассет больше не AI-generated

Если вы **заменили** AI-generated ассет на procedural или CC0 source:

1. В tracking issue изменить статус на `removed`.
2. В CREDITS.md изменить секцию (заменить на новый источник).
3. В коммите замены НЕ использовать trailer `ai-asset-policy` (это уже не
   AI-generated).

Старые коммиты с `ai-asset-policy: pending` остаются в истории git — это
нормально. Audit trail — желателен.

## Ответственность

- **Товарищ Шифу**: legal owner. Единственный, кто может подписать
  license status `yes` (или явно принять риск carry-forward).
- **Воркер** (любой профиль): обязан следовать этой процедуре. Если
  процедура не выполнена — это protocol violation (см. AGENTS.md), не
  cosmetic issue.
- **Architect**: ревью при появлении новых case'ов (например, voice
  cloning, video generation). ADR-0136 пересматривается при
  структурных изменениях (например, новый тип генератора без явного
  license).

## Changelog этой процедуры

- 2026-09-27 — initial version (kanban t_2d0f27f7, ADR-0136, issue #3051).
