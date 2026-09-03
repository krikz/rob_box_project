# ADR-0038: Adopt OpenSpec (HYBRID + bulk-import legacy ADR)

- **Status:** Accepted
- **Date:** 2026-08-31
- **Deciders:** Товарищ Шифу (владелец репо)
- **Supersedes:** none
- **Related:** ADR-0030 (нумерация), ADR-0026 (recovery card), PR #1798 (research), issue #1813

## Context

В рамках PR #1798 был проведён research по OpenSpec — spec-driven framework для AI-агентов.
Рекомендация research: **HYBRID** — новое поведение писать через `openspec/changes/`,
legacy 35+ ADR оставить как есть в `docs/adr/`.

Товарищ Шифу решил расширить рекомендацию: дополнительно сделать **bulk-import всех
существующих ADR** в `openspec/changes/adr-NNNN-*/` через автоматический скрипт.

## Decision

1. **HYBRID**: новые изменения пишем через `openspec/changes/<name>/` (4 артефакта:
   proposal, spec, design, tasks). Старые ADR в `docs/adr/` остаются как есть.

2. **Bulk-import legacy ADR** через `scripts/import_adr_to_openspec.py`:
   - 45 файлов `docs/adr/*.md` → 45 папок `openspec/changes/adr-NNNN-name/`
   - Создаём ТОЛЬКО `proposal.md` (skeleton + ссылка на оригинал)
   - НЕ создаём `design.md` / `tasks.md` / `spec.md` — это воркер дописывает когда
     change реально понадобится (delta-first подход OpenSpec)

3. **Tooling**: `scripts/import_adr_to_openspec.py` — Python, argparse, idempotent,
   `--dry-run` для безопасного прогона.

## Consequences

### Положительные

- Единый источник правды для будущих изменений (OpenSpec как authoritative)
- 45 импортированных спеков дают AI-агентам контекст об истории решений
- Совместимо с ADR-0030 (нумерация: `docs/adr/NNNN-*.md` не конкурирует с
  `openspec/changes/adr-NNNN-name/`)
- Скрипт идемпотентный — повторный прогон безопасен

### Отрицательные

- ⚠️ **Stale risk**: импортированные proposal.md могут расходиться с реальностью
  (например, статус ADR мог измениться после написания). Фикс: при первом изменении
  в соответствующей области воркер дописывает spec/design/tasks и обновляет proposal
- ⚠️ **45 proposal.md** без design/tasks/spec выглядят как «полуфабрикат» в `openspec list`
- ⚠️ **Trade-off против рекомендации research**: автор PR #1798 явно советовал НЕ
  импортировать legacy. Это решение Шифу — осознанное отклонение от рекомендации.

### Нейтральные

- `openspec/` живёт внутри `docs/research/openspec-pilot/` (pilot scope) — не на
  корне репо. Когда Шифу решит масштабировать — `openspec/` поднимется на уровень
  выше. Это **сознательный шаг**: сначала доказать подход, потом масштабировать.
- Ветки воркеров продолжают использовать pattern `z-{agent}/NNNN-*` (см. существующие
  ветки в remote).

## Alternatives considered

- **Ничего не делать (только HYBRID без импорта)** — рекомендация research PR #1798.
  Отклонено решением Шифу: bulk-import даёт лучший контекст AI-агентам.
- **Полная миграция в `openspec/` с удалением `docs/adr/`** — отклонено: формат ADR
  устоявшийся, 35+ файлов используются в `CONTRIBUTING.md`, AGENTS.md, других ADR.
- **Конвертация через AI в цикле `/opsx:explore` × N** — отклонено: слишком дорого
  (~35 × 20 мин), результат не лучше чем skeleton из скрипта.

## Implementation

См. PR #1799 (issue #1813). Шаги:

1. Скрипт `scripts/import_adr_to_openspec.py` (argparse, --dry-run, idempotent)
2. `openspec/openspec/changes/adr-*-*/proposal.md` × 45 (skeleton)
3. `openspec validate --all --strict` — PASS

## Rollback

Если Шифу решит откатить: `git revert` коммита + `rm -rf openspec/changes/adr-*`.
Старые ADR в `docs/adr/` не затронуты (никаких мутаций).