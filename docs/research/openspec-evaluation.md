# OpenSpec Evaluation — research for rob_box_project

**Дата:** 2026-08-31
**Автор:** architect (Hermes Agent)
**Issue:** #1789
**Severity:** MEDIUM (research, не migration)
**Pilot commit:** см. `docs/research/openspec-pilot/` в ветке `z-{agent}/1789-research-docs-openspec-spec-driven-frame`

## TL;DR

**Рекомендация: HYBRID (постепенное внедрение).**

- OpenSpec — **зрелый, brownfield-friendly** spec-driven framework с CLI
  валидацией, 30+ AI-интеграциями (включая `hermes`), MIT-лицензией,
  без API keys. Установка заняла ~16 секунд.
- Pilot change folder `fix-memory-speaker-id` прошёл
  `openspec validate --strict` (1 passed / 0 failed), все 4 артефакта
  (proposal + specs + design + tasks) валидны.
- **Не рекомендую полный перенос 35+ ADR** в `openspec/` — формат ADR
  устоявшийся, перенос создаст шум без выигрыша.
- **Рекомендую** новое поведение (changes, specs, design, tasks) писать
  в `openspec/changes/`, legacy ADR оставить как есть в `docs/adr/`.
- ADR-0030 (нумерация) совместима: `openspec/changes/<name>/` не
  конкурирует с `docs/adr/NNNN-*.md` numbering.
- **Ничего не ломаем в текущей работе** — это только research.

## 1. Что мы изучили

### 1.1 Установка

```bash
$ npm install -g @fission-ai/openspec@latest
added 80 packages in 16s
$ openspec --version
1.11.0
```

Окружение: Node.js v22.23.1 (требует ≥20.19.0), npm 10.9.8. Установка
clean, без warnings. CLI написан на Node + TypeScript, дистрибутив
~10 MB.

### 1.2 Инициализация в проекте

```bash
$ cd docs/research/openspec-pilot
$ openspec init --tools hermes,claude,codex,github-copilot --force
...
Created: Hermes Agent, Claude Code, Codex, GitHub Copilot
6 skills and 6 commands in .hermes, .claude, .agents, .github/
```

Созданные артефакты:
- `openspec/config.yaml` (schema: spec-driven, поддерживает
  `context`, `rules`, `operations`)
- `openspec/changes/` (активные изменения)
- `openspec/specs/` (canonical source of truth, после `archive`)
- `openspec/changes/archive/` (архив завершённых)
- `.hermes/skills/openspec-{propose,explore,apply-change,archive-change,update-change,sync-specs}/SKILL.md`
- `.claude/commands/opsx/{explore,propose,apply,archive,update,sync}.md`
- `.claude/skills/openspec-*/SKILL.md`
- `.agents/skills/openspec-*/SKILL.md` (для Codex)
- `.github/prompts/opsx-*.prompt.md` (для GitHub Copilot)

**Важно**: Hermes требует manual step для подхвата skills:

> Setup required for Hermes Agent: Hermes only loads skills from
> `~/.hermes/skills` by default. Add this project's `.hermes/skills`
> directory to `skills.external_dirs` in `~/.hermes/config.yaml`.

Это значит: для работы OpenSpec-скиллов в Hermes-агентах нужно либо
скопировать их в `~/.hermes/skills/`, либо добавить
`skills.external_dirs: [".hermes/skills"]` в глобальный config.

### 1.3 Прототип change folder

Создал `openspec/changes/fix-memory-speaker-id/` с 4 артефактами:

| Артефакт | Файл | Размер | Что внутри |
|---|---|---|---|
| proposal | `proposal.md` | ~60 строк | Why / What Changes / Capabilities / Impact |
| spec | `specs/voice-speaker-recognition/spec.md` | ~70 строк | 4 Requirements × 2-3 Scenarios (WHEN/THEN) |
| design | `design.md` | ~60 строк | Context / Goals / Decisions / Risks / Alternatives |
| tasks | `tasks.md` | ~30 строк | 5 групп × 2-3 чекбоксов |

### 1.4 Валидация

```bash
$ openspec validate fix-memory-speaker-id --strict
Change 'fix-memory-speaker-id' is valid

$ openspec validate --all --strict --json
{"items": [...], "summary": {"totals": {"items": 1, "passed": 1, "failed": 0}}}

$ openspec status --change fix-memory-speaker-id
Progress: 4/4 artifacts complete
[x] proposal [x] specs [x] design [x] tasks
All planning artifacts complete!

$ openspec doctor
Root: ok / References: (none declared)
```

Validator проверяет:
- Наличие всех обязательных артефактов
- Наличие хотя бы одной capability (или явный `skip_specs: true`)
- Корректность WHEN/THEN scenarios в spec.md
- `.openspec.yaml` метаданные (created, schema, goal)

**Время валидации одного change folder: 24 ms.** Ничтожно.

### 1.5 Схемы (schemas)

```
openspec schemas
Available schemas:
  spec-driven (default) — proposal → specs → design → tasks
```

Только одна схема в OpenSpec 1.11.0. Это **специфический формат
"spec-driven"**, не кастомные schemas. Альтернатив нет (в roadmap
есть `schema` команда как experimental). Для brownfield-проекта
подходит — формат строгий, но простой.

## 2. Что мы выиграем (PROS)

| Плюс | Конкретика |
|---|---|
| **AI-агентам читать легче** | specs в формате "Requirement / WHEN / THEN" — стандарт, который любой LLM понимает с первого взгляда |
| **Brownfield-friendly** | "fluid not rigid" — можно начать с одного change folder, остальное не трогать |
| **30+ AI tools поддержка** | hermes, claude, codex, github-copilot, cursor, kiro, continue, gemini, и т.д. — каждый получает slash-command или skill |
| **Validate в CI** | `openspec validate --all --strict` за 24 ms — можно воткнуть в merge-gate как pre-merge hint |
| **WHEN/THEN scenarios** | Spec формат приближен к Gherkin — будущие e2e-тесты могут читать scenarios напрямую |
| **No API keys** | Всё локально, не зависит от SaaS-сервиса |
| **Schema-driven** | Любой новый change folder имеет одинаковую структуру — предсказуемо |
| **Archive flow** | После реализации → `openspec archive` → перенос в `openspec/specs/<capability>/spec.md` (canonical). Решает проблему "что было до этого изменения" |
| **Hermes skills готовы** | `.hermes/skills/openspec-propose/` и т.д. — можно подключить через `skills.external_dirs` |

## 3. Что мы потеряем / риски (CONS)

| Минус / риск | Severity | Митигация |
|---|---|---|
| **Два места правды** для docs: `docs/adr/` (legacy, 35+ файлов) и `openspec/specs/` (новое) | MEDIUM | HYBRID: новое → openspec/, legacy → docs/adr/. Cross-ref через "See ADR-NNNN" в proposal.md |
| **Hermes требует external_dirs в config** — ещё один шаг setup'а | LOW | Один раз настроить в `~/.hermes/config.yaml`, документировать в AGENTS.md |
| **Specs в формате Gherkin WHEN/THEN требуют дисциплины** — легко написать расплывчато | LOW | Валидатор ловит базовые ошибки (отсутствие WHEN/THEN), качество — на ревьюере |
| **Полная миграция 35+ ADR = ~1 неделя dedicated работы** | MEDIUM | НЕ рекомендую. HYBRID обходит это полностью |
| **Нет единой timeline-вьюхи** для всех changes | LOW | `openspec list` показывает активные; архив — отдельно. Достаточно для нашего масштаба |
| **Двойной ввод при фиксе**: правишь spec.md И потом code** | MEDIUM | OpenSpec явно говорит: "specs описывают behavior, не implementation". Spec — single source of truth, code — производная |
| **Привычка команды к ADR-формату** | LOW | ADR ≠ openSpec spec. ADR — это "почему мы решили так", openSpec spec — "что система должна делать". Они дополняют друг друга |
| **Recovery-card contract (ADR-0026) не покрывает OpenSpec flow** | MEDIUM | Если OpenSpec внедряется — добавить новую секцию в CONTRIBUTING.md про "OpenSpec changes" |
| **OpenSpec не умеет ссылаться на `docs/adr/NNNN-*.md` напрямую** — нужно вручную | LOW | Proposal.md может содержать текстовую ссылку "См. ADR-0026 (recovery card)" |

## 4. Совместимость с нашим стеком

### 4.1 Agent-flow / Hermes (Q20–Q24)

| Аспект | Совместимость | Детали |
|---|---|---|
| **Slash-commands** | ✅ | `/openspec-propose`, `/openspec-apply-change`, `/openspec-archive-change` готовы |
| **Hermes skills** | ⚠️ | Skills сгенерированы в `.hermes/skills/`, но Hermes их НЕ подхватит без `external_dirs` в config |
| **Branching Q20** (`z-{agent}/<id>-slug`) | ✅ | Никак не пересекается с `openspec/changes/<name>/` — это разные сущности |
| **Recovery-card (ADR-0026)** | ⚠️ | Нужно дополнить — добавить, что OpenSpec change folder имеет свой lifecycle (`propose → apply → archive`), и recovery-worker должен проверить его статус |
| **Merge-gate (pre-merge)** | ✅ | `openspec validate --all --strict --json` готов как pre-merge gate (24 ms, можно блокером для PR с `openspec/changes/*`) |
| **E2E voice test** | ✅ | Spec-driven scenarios (WHEN/THEN) могут стать источником для e2e harness'а — но это уже далеко за пределами research |

### 4.2 GitHub Actions / CI

```yaml
- name: OpenSpec validate
  run: openspec validate --all --strict --json
```

Работает, ~24 ms. Можно добавить как отдельный job. **Зависимость:**
`@fission-ai/openspec` нужно ставить в CI runner
(`npm install -g @fission-ai/openspec@latest`), либо
`npx @fission-ai/openspec validate ...` через `package.json` (но
`npx` для global-пакета медленнее).

### 4.3 ADR-нумерация (ADR-0030)

**Не конфликтует.** OpenSpec использует `openspec/changes/<kebab-name>/`
как папки, ADR используют `docs/adr/NNNN-<slug>.md` как файлы.
Разные namespace, разные правила нумерации (kebab-case для change,
NNN для ADR). Pre-merge guard ADR-0030 продолжает работать
независимо.

### 4.4 CONTRIBUTING.md / AGENTS.md

Потребуется дополнить:
- В `CONTRIBUTING.md` секция "OpenSpec changes" (когда использовать,
  как мержить с ADR)
- В `AGENTS.md` строка о том, что Hermes должен подхватить skills
  из `.hermes/skills/` через `external_dirs`

### 4.5 Multi-monorepo vs single repo

rob_box_project — single repo (мульти-Docker-image, но один Git).
OpenSpec "built for brownfield" — подходит. **Если** в будущем
выделим `rob_box_voice/` в отдельный репо — OpenSpec переедет
вместе с ним без проблем (per-repo openspec/).

## 5. Migration plan

### 5.1 HYBRID (рекомендую)

**Правило: новое поведение пишется в `openspec/`, legacy ADR остаются как есть.**

| Категория | Где живёт | Кто пишет | Кто валидирует |
|---|---|---|---|
| **Архитектурное решение (WHY)** | `docs/adr/NNNN-<slug>.md` | architect / senior | merge-gate (ADR-0030) |
| **Новое поведение / требование** | `openspec/changes/<name>/specs/<cap>/spec.md` | worker / agent-flow | `openspec validate --strict` |
| **Дизайн новой фичи** | `openspec/changes/<name>/design.md` | worker / agent-flow | `openspec validate --strict` |
| **Чеклист имплементации** | `openspec/changes/<name>/tasks.md` | worker / agent-flow | `openspec validate --strict` (для pre-apply), `openspec archive` (после) |
| **После реализации** | `openspec/specs/<cap>/spec.md` (canonical) | (auto через `archive`) | same validator |

**Конвенция в Proposal.md** — ссылка на ADR, если он есть:

```markdown
## Why

См. [ADR-0026 (recovery card)](../../../docs/adr/0026-recovery-card-contract.md)
— этот change реализует contract worker'а для recovery-карточек.

[собственно Why текст...]
```

Это сохраняет cross-ref между двумя системами.

### 5.2 Полный перенос (отвергнут, но для полноты)

| Шаг | Объём | Риски |
|---|---|---|
| 1. `openspec init` в корне | 1 команда | — |
| 2. Переписать каждый из 35+ ADR в proposal+spec+design+tasks | ~5-7 дней | потеря контекста ADR; некоторые ADR — про решения, а не про behavior (например, ADR-0026) |
| 3. Обновить все cross-ref в коде и docs | ~2 дня | grep-замены, regression risk |
| 4. Удалить `docs/adr/` | 1 команда | необратимо; сломает любую ссылку "см. ADR-NNNN" в issue/PR/комментариях |

**Не рекомендую** — слишком большой blast radius за research-задачу.

### 5.3 NO-GO (отвергнут)

Если решите не внедрять — **ничего не делаем**. Текущий ADR-формат
работает, ADR-0030 формализовал нумерацию, проблема "нет формата
proposal → specs → design → tasks" не критична (recovery-card уже
пробрасывает contract, остальные ADR'ы описывают решения по мере
надобности).

## 6. Решение (GO / NO-GO / HYBRID)

**HYBRID** — рекомендую.

Предпосылки:
- ✅ OpenSpec установился, прототип валиден (1/1 passed).
- ✅ Hermes adapter есть (но требует `external_dirs` — дешёвая правка).
- ✅ Совместим с agent-flow / merge-gate / e2e.
- ✅ НЕ ломает текущие 35+ ADR (HYBRID = новое в openspec, старое в adr).
- ✅ Полная миграция не нужна (избегаем 5-7 дней работы).

Действия для внедрения (если Шифу подтвердит HYBRID):

1. **Создать ADR-XXXX "Adopt OpenSpec (hybrid)"** с migration plan
   из §5.1 и обоснованием "почему HYBRID, не full".
2. **Добавить `openspec/` в корне репо** (`openspec init --tools hermes,claude,codex`).
3. **Дополнить `CONTRIBUTING.md`** секцией "OpenSpec changes" —
   когда использовать, как мержить с ADR.
4. **Дополнить `AGENTS.md`** — что Hermes должен подхватить
   skills из `.hermes/skills/` через `external_dirs`.
5. **Добавить pre-merge gate** в `scripts/agent_flow/agent-flow-merge-gate.sh`:
   `openspec validate --all --strict --json` для PR с `openspec/changes/*`.
6. **Дополнить recovery-card contract (ADR-0026)** — recovery-worker
   должен проверить состояние OpenSpec change folder.
7. **Пилотная имплементация**: первый worker, который получит карточку
   на новую фичу, пишет её через `openspec-propose` skill. Несколько
   PR'ов — проверить flow на реальной задаче.
8. **Ретроспектива через 2-4 недели** — пересмотреть решение, если
   OpenSpec не оправдал ожиданий.

## 7. Команды для Шифу

```bash
# 1. Установить (если ещё не)
npm install -g @fission-ai/openspec@latest

# 2. Инициализировать в корне репо (после решения о HYBRID)
cd /home/builder/hermes-share/rob_box_project
openspec init --tools hermes,claude,codex --force

# 3. Проверить, что pilot change folder живёт
openspec list
openspec validate --all --strict

# 4. Попробовать slash-command / skill через Hermes / Claude / Codex
/openspec-propose "новая фича"

# 5. Когда change готов к реализации — apply
/openspec-apply-change <name>

# 6. После реализации и PR-merge — archive
openspec archive <name>
# (это переносит specs в openspec/specs/ canonical)
```

## 8. Compatibility check (итого)

| Слой | Совместимо | Усилие |
|---|---|---|
| agent-flow / Hermes | ✅ | external_dirs в `~/.hermes/config.yaml` (1 строка) |
| merge-gate pre-merge | ✅ | ~5 строк в `agent-flow-merge-gate.sh` |
| 30+ AI tools | ✅ | `--tools all` при init |
| GitHub Actions CI | ✅ | один новый step в `.github/workflows/` |
| ADR-0030 нумерация | ✅ | нет пересечения |
| Recovery-card (ADR-0026) | ⚠️ | дополнить CONTRACT для openspec changes |
| CONTRIBUTING.md | ⚠️ | добавить секцию "OpenSpec changes" |
| AGENTS.md | ⚠️ | добавить инструкцию про `external_dirs` |
| Существующие 35+ ADR | ✅ | НЕ трогаем (HYBRID) |
| E2E voice test | ✅ | Spec WHEN/THEN может стать источником тестов (будущее) |
| Multi-monitor deploy | ✅ | n/a |
| Docker images / CI builds | ✅ | n/a |

## 9. Вопросы Шифу

Из acceptance карточки:

1. **Полный vs постепенный переход?** — рекомендую HYBRID
   (только новые changes в openspec/, legacy ADR остаются).
2. **Удалить ли `docs/adr/`?** — НЕТ, оставить как legacy.
3. **Кто владеет правкой specs — юзер, воркер, оба?** —
   рекомендую оба (юзер для архитектурных, воркер для фиксов и
   мелких features), с обязательным PR review.
4. **Hermes skill integration — настраивать skill / slash command?**
   — рекомендую настроить `external_dirs` в `~/.hermes/config.yaml`
   (минимум) и slash-commands через `.claude/commands/opsx/`
   (для будущего Claude-Code worker'а, если он появится).
5. **`openspec/changes/` — отдельно или внутри monorepo?** —
   внутри monorepo, в корне (`/openspec/`), как и рекомендует
   upstream Fission-AI/OpenSpec.

## 10. Что НЕ делаем

- Не пишем `openspec/changes/` в существующий `docs/adr/` namespace —
  это разные системы с разными правилами.
- Не удаляем `docs/adr/` — legacy ADR'ы остаются как reference.
- Не форсим перевод старых ADR'ов — OpenSpec для новых изменений.
- Не модифицируем существующие PR/issue-комментарии со ссылками
  на `docs/adr/NNNN-*.md` — они продолжают работать.

## 11. Связанные

- ADR-0018 (agent-honesty-culture) — culture of raw-evidence применима
  к OpenSpec: каждый proposal должен ссылаться на конкретный issue
  или требование.
- ADR-0026 (recovery-card-contract) — нужно дополнить для OpenSpec
  changes (recovery-worker должен проверить change folder).
- ADR-0030 (adr-numbering-sot) — продолжает действовать для
  legacy `docs/adr/`, OpenSpec не конфликтует.
- `docs/design/AGENT_FLOW_PROPOSAL.md` — OpenSpec встраивается в
  agent-flow как опциональный шаг "spec phase" перед "tasks phase".

## 12. Verification

- [x] OpenSpec 1.11.0 установлен, `openspec --version` отдаёт 1.11.0
- [x] `openspec init` создан в `docs/research/openspec-pilot/` —
      6 skills + config.yaml + пустые changes/specs/archive dirs
- [x] Прототип `fix-memory-speaker-id` создан (4/4 артефакта)
- [x] `openspec validate --strict` — `Change is valid` (1/1 passed, 24 ms)
- [x] `openspec status` — `Progress: 4/4 artifacts complete`
- [x] `openspec doctor` — `Root: ok`
- [ ] ADR "Adopt OpenSpec (hybrid)" — НЕ написан (ждёт решения Шифу)
- [ ] Решение Шифу — HYBRID принят / NO-GO / полный переход

## 13. Сырые выводы (raw evidence для ADR-0018)

См. коммит в ветке `z-{agent}/1789-research-docs-openspec-spec-driven-frame`:
файлы `docs/research/openspec-pilot/openspec/changes/fix-memory-speaker-id/`
содержат 4 заполненных артефакта, прошедших `openspec validate --strict`.

```
$ openspec validate fix-memory-speaker-id --strict
Change 'fix-memory-speaker-id' is valid

$ openspec validate --all --strict --json | jq '.summary'
{
  "totals": { "items": 1, "passed": 1, "failed": 0 },
  "byType": {
    "change": { "items": 1, "passed": 1, "failed": 0 },
    "spec":   { "items": 0, "passed": 0, "failed": 0 }
  }
}

$ openspec status --change fix-memory-speaker-id
Progress: 4/4 artifacts complete
[x] proposal [x] specs [x] design [x] tasks
All planning artifacts complete!
```
