# Runbook: stale-candidate triage & escalation

_Owner: pm + devops. Цель — единая процедура реакции на метку `stale-candidate`
от `agent-flow-triage.sh` (см. ADR-0022 §2 GATE-2). Этот runbook обязателен для
triage-cron + PM-rotation + on-call инженеров._

## 0. TL;DR

1. `stale-candidate` ставится автоматически на issue без process-меток старше
   `STALE_HOURS_2h` (по умолчанию 2 ч) — см. `scripts/agent_flow/agent-flow-triage.sh:scan_stale_candidates`.
2. Если `stale-candidate` висит > **2 ч** (high) / **8 ч** (medium) / **24 ч** (low)
   без появления `agent:*` метки — это **эскалация**, не норма. SLA: median
   должен быть ≤ 2 ч для high-priority.
3. **Шаги эскалации зависят от меток.** См. дерево в §3.

## 1. Что делает triage-cron

```text
agent-flow-triage.sh (cron каждые 1 мин, см. .github/workflows/agent-flow-triage.yml)
  ├─ Phase 1: scan_stale_candidates
  │    → add-label stale-candidate (если OPEN + нет process-меток + age > STALE_HOURS_2h)
  ├─ Phase 2: scan_needs_plan
  │    → add-label needs-plan для issues с needs-triage и agent:* без PR
  ├─ Phase 3: bug-orphans
  │    → add-label needs-triage для bug без agent:*
  └─ Phase 4: FORCE_TRIAGE (с 5e90c2f, DRY-RUN по умолчанию)
       → для priority:high AND (bug|voice|operator) без agent:* — логирует
         [FORCE-TRIAGE] candidates (с FORCE_TRIAGE_APPLY=true — apply needs-triage +
         agent:backend + guidance comment с дедупом 60 мин).
```

## 2. Как понять, что есть P0

Триггеры (любой из):

| Триггер | Где смотреть | Ссылка |
|---|---|---|
| `priority:high` ИЛИ `priority:critical` | `gh issue list --label "priority:high,priority:critical" --state open` | — |
| `bug` AND (`voice` OR `operator` OR `quest`) | `gh issue list --label "bug" --label "voice,operator,quest"` | — |
| Issue из P0-листа Шифу (см. `docs/p0-voice-bugs-decisions.md` §2) | прямые ссылки на #2131/#2132/#2136/#2137/#2143 | этот документ |

Если хотя бы один из триггеров сработал И висит `stale-candidate` ≥ 1 ч →
**это P0-эскалация**, иди в §3.

## 3. Escalation: high-priority voice/operator-bug без agent:* метки

Пошаговая инструкция (выполняется тем, кто первый заметил — PM, devops on-call,
или сам `agent-flow-triage.sh` через Phase 4):

1. **Подтверди эскалацию** (≤ 5 мин).
   ```bash
   gh issue list --label "stale-candidate" --label "priority:high" --state open \
     --json number,title,labels,createdAt,updatedAt
   ```
   Если в выдаче есть `bug` ИЛИ `voice` ИЛИ `operator` ИЛИ `quest` —
   это наш случай. Переходи к шагу 2. Иначе — обычный stale-handling.

2. **Подними процесс-метки вручную**, если Phase 4 ещё в DRY-RUN:
   ```bash
   # FORCE_TRIAGE_APPLY=true переключает Phase 4 в apply-режим (см. ADR-0022 §3.4)
   FORCE_TRIAGE_APPLY=true /opt/hermes/bin/agent-flow-triage.sh --phase 4 --apply-force
   # либо ручной apply для одного issue:
   gh issue edit <N> --add-label "agent:backend,needs-triage"
   ```

3. **Пингани людей в правильном канале**:
   - **Slack/Telegram `#devops`** — пост вида:
     ```
     🚨 P0 voice/operator-bug orphan: #<N> <title>
     — stale-candidate уже Xч, нет agent:* метки.
     — @krikz @<profile:backend-on-call> @shifu прошу подтвердить owner.
     ```
     Где Xч — реальное время от createdAt (или от updatedAt с stale-candidate).
   - **В issue** — комментарий с шаблоном:
     ```markdown
     ⚠️ @shifu @krikz stale-candidate висит > 1ч, нет agent:* метки. P0-лист от 11.09
     (см. `docs/p0-voice-bugs-decisions.md`). Прошу подтвердить workaround/reopen.
     refs: ADR-0022 GATE-2, orphan-stale-no-agent-assign retro.
     ```
   - **PM-channel** — упомяни карточку в Kanban (например t_cf60f006 → comment "эскалация <N>").

4. **Отметь в issue, что эскалация сделана**:
   - Добавь label `escalated-to-shifu` (если есть; если нет — создай через
     `gh label create escalated-to-shifu --color "B60205" --description "Эскалировано Шифу"`).
   - В issue напишу комментарий «escalated at <UTC>, channel #devops, pinging @shifu».

5. **Отслеживай реакцию ≤ 4 ч**:
   - Если за 4 ч нет реакции (комментарий/метка/закрытие) — второй пинг в #devops,
     эскалация на владельца репо (krikz) в личку.
   - Если Шифу ответил «workaround» — обнови `docs/p0-voice-bugs-decisions.md` §2
     (статус с «pending» → «workaround-accepted»).
   - Если Шифу ответил «hotfix в PR #X» — обнови §2 на «hotfix-merged, PR #X».

## 4. Когда эскалация НЕ нужна

- Issue имеет `agent:*` + `needs-plan` + открытый PR → нормальный workflow,
  не трогай.
- Issue из категории `type:tech-debt` без `bug` → SLA = 24 ч, не high.
- Issue уже на Шифу-контроле (комментарий от GOODWORKRINKZ в последние 24 ч) —
  не двойной пинг.

## 5. Связанные документы

- ADR-0022: `docs/adr/0022-process-e2e-done-gates.md` — GATE-2, процесс-метки.
- ADR-0031: `docs/adr/0031-gsd-orphan-triage.md` — orphan-triage Phase 1–3.
- ADR-0041: `docs/adr/0041-unknown-assignee-silent-drop-guard.md` — fallback.
- `docs/p0-voice-bugs-decisions.md` — таблица решений по P0-листу Шифу.
- `docs/retros/2026-09-14-t_cf60f006-orphan-stale-no-agent-assign.md` —
  ретро-файл этого инцидента.
- Скрипт: `scripts/agent_flow/agent-flow-triage.sh` — реализация.

## 6. Метрики для дашборда (TODO: добавить в Grafana)

- `stale_candidate_age_high_p95` — 95-й перцентиль возраста stale-candidate для
  high-priority. Должен быть ≤ 2 ч (SLA). Текущий outlier: 80 ч (t_19342112).
- `orphan_high_count` — кол-во high-priority issues без `agent:*` в текущий момент.
  Должен быть 0.
- `escalation_to_shifu_count_per_day` — частота эскалаций. Норма: ≤ 1/неделю.
  Всплеск → ретро-аналитика.