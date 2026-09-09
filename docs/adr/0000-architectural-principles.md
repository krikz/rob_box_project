# ADR-0000 — Architectural Principles

**Дата:** 2026-09-09 (initial; будет расти)
**Статус:** Accepted (meta-ADR)
**Тип:** principle (cross-cutting policy)

## Назначение

Этот файл — meta-ADR, который собирает **архитектурные принципы и политики
высокого уровня** для всего проекта rob_box_project. Конкретные решения
живут в ADR-XXXX-*.md; здесь только то, что должно влиять на **каждый
новый PR**.

Принципы не отменяют конкретные ADR — они указывают направление, в котором
новые решения должны идти.

## Принципы

### P1. Cron-скрипты ОБЯЗАНЫ писать tick-summary в stdout

> **«silent = bug»**. Каждый cron-скрипт (mode=`no-agent`, deliver=local)
> ОБЯЗАН иметь tick-start и tick-end markers в stdout. `Empty stdout = silent`
> в `hermes_cli.subcommands.cron` — это недопустимое состояние для
> prod-скриптов.

**Обоснование:** Ретро t_8fba04b9 (issue #1977): 50+ тиков merge-gate
имели `silent (empty output)` при exit=0. Невозможно диагностировать без ssh.

**Контракт:** см. [ADR-0088](0088-cron-tick-summary-policy.md).

**Применяется к:** любой `scripts/agent_flow/*-cron*.sh`,
`scripts/agent_flow/*-watchdog*.sh`, `scripts/agent_flow/*-process*.sh`,
`scripts/agent_flow/*-gate*.sh`. Примеры: `agent-flow-merge-gate.sh`,
`agent-flow-e2e-process.sh`, `agent-flow-blocked-watchdog.sh`.

**Тест:** `tests/agent_flow/test_merge_gate_logging.sh` (regression).

### P2. Cron-скрипты ОБЯЗАНЫ использовать trap EXIT для tick-end

> Аварийный exit через `set -euo pipefail` НЕ должен оставлять cron
> в silent-состоянии. `trap 'tick_end_marker 2>/dev/null || true' EXIT`
> гарантирует marker даже при crash.

**Применяется к:** всем скриптам из P1.

### P3. Per-day log-файлы для диагностики

> Каждый agent-flow cron-скрипт пишет в
> `~/.hermes/profiles/<profile>/logs/<script>/YYYY-MM-DD.log` для
> post-mortem диагностики без ssh. Best-effort — отсутствие файла
> не считается багом, но наличие обязательно при нормальной работе.

**Контракт:** см. helper `out()` в любом из обновлённых скриптов.

## Когда обновлять

- Добавлять новый принцип сюда, только если он cross-cutting (применяется
  к **нескольким** компонентам или к новым скриптам по умолчанию).
- Если принцип относится к одному компоненту — делать отдельный ADR
  и **ссылаться** на него отсюда (не дублировать содержание).
- Принципы ревьюит Шифу (владелец репо) + architect.

## Ссылки

- ADR-0088 — реализация P1, P2, P3 для cron-скриптов
- ADR-0018 — честность воркеров (cross-cutting principle)
- ADR-0014 — agent-flow этика (process rules)