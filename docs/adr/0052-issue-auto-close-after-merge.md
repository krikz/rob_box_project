# ADR-0052: auto-close issue after merge — инвариант, fallback и squash-loss восстановление

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-07 |
| Автор | architect (Hermes Agent); карточка `t_fe771303` |
| Контекст | Ночной обзор 2026-09-06 (`t_bfd19ffb`) выявил recurring-баг: после merge PR в develop issue остаётся OPEN и Шифу закрывает вручную в течение ≤94 секунд (через свой GH token). 2/2 merged PR за сутки = 100% ручной close, что подрывает гарантии ADR-0014 и наблюдаемый contract «issue закрывается по факту merge». Acceptance criteria карточки #2017 требует: auto-close в ≤30 секунд + `closedByPullRequestsReferences` заполнен + idempotent fallback в merge-gate / orphan-watchdog в ≤5 минут. |
| Затрагивает | (a) `scripts/agent_flow/agent-flow-merge-gate.sh` — расширение ветки `pr_state=MERGED` в `archive_merged_card`-роутере; (b) новый скрипт `scripts/agent_flow/agent-flow-issue-close-fallback.sh` — отдельный cron-надзор; (c) `scripts/agent_flow/install.sh` — регистрация cron; (d) `docs/adr/` — этот ADR. **НЕ затрагивает** `agent-flow-completion-check.sh` (PR-CI), squash-merge workflow (карточка явно out of scope), e2e-process (PR не имеет `needs-e2e` → e2e-process не запускался → contract другой). |
| Родители | ADR-0014 (контракт close=e2e-done ∧ MERGED), ADR-0018 (raw-evidence обязателен), ADR-0026 (recovery contract), ADR-0046 (orphan-cleanup-after-merge — kanban-карточки, не issue), ADR-0049 (ночной ревью как жанр). |
| Связанные | `t_bfd19ffb` (родительская nightly-review карточка), `t_fe771303` (эта карточка), issue #2017, issue #1989 + PR #2010 (squash-merge `22b49039`, 2026-09-06T07:58:43Z), issue #1990 + PR #2011 (squash-merge `6e016325`, 2026-09-06T08:35:25Z), `agent-flow-merge-gate.sh` (line 3159 — существующая ветка `MERGED`; line 3198 — `_has_no_e2e` pre-compute; line 3229 — `whoami_close_issue` уже вызывается для `no-e2e-required`), `agent-flow-blocked-watchdog.sh` (line 225 — `gh issue close` уже есть, но для orphan `needs-e2e`). |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (на 2026-09-06)

Nightly-review за 2026-09-06 (`t_bfd19ffb`, develop HEAD `07d33d20`) зафиксировал: оба PR, смерженные за сутки, привели к ручному закрытию issue через 54–94 секунды после merge. Это означает:

- merge-gate **видел** `pr_state=MERGED && pr_base=develop` (line 3159 срабатывает);
- merge-gate **НЕ закрыл** issue (нет ни `e2e-done`, ни `no-e2e-required` — line 3188/3199);
- Шифу закрыл руками в течение ≤94 секунд → `closed_by=GOODWORKRINKZ`.

### 1.2 Сырые доказательства

**Repo settings** (`gh api repos/krikz/rob_box_project`):

```
allow_auto_merge:        false
allow_squash_merge:      true
squash_merge_commit_title:    COMMIT_OR_PR_TITLE
squash_merge_commit_message: COMMIT_MESSAGES  ← ключевой момент
merge_commit_title:          MERGE_MESSAGE
merge_commit_message:        PR_TITLE
```

**Issue #1989 events** (`gh api repos/krikz/rob_box_project/issues/1989/events`):

```
event=milestoned   actor=GOODWORKRINKZ  2026-09-05T19:46:43Z
event=labeled×4    actor=GOODWORKRINKZ  2026-09-05T19:46:44Z..45Z  (ai-generated, source:gsd, type:functional, priority:high)
event=referenced   actor=krikz          2026-09-06T07:58:44Z  commit=22b49039
event=closed       actor=GOODWORKRINKZ  2026-09-06T07:59:37Z  (через 54с после merge)
event=referenced   actor=krikz          2026-09-06T08:35:26Z  commit=6e016325  (PR #2011 cross-ref на этот же issue)
```

**Issue #1990 events**:

```
event=milestoned   actor=GOODWORKRINKZ  2026-09-05T19:46:47Z
event=labeled×4    actor=GOODWORKRINKZ  2026-09-05T19:46:49Z
event=labeled×~14  actor=krikz          2026-09-05T23:22:40Z → 2026-09-06T07:21:47Z  (label churn, normal triage rotation)
event=closed       actor=GOODWORKRINKZ  2026-09-06T08:36:59Z  (через 94с после merge PR #2011)
```

**Issue #1989 timeline** (`gh api repos/krikz/rob_box_project/issues/1989/timeline?per_page=100`):

```
count: 1  event: closed
count: 3  event: commented
count: 3  event: cross-referenced   ← (PR #2010 + PR #2011 + #2017)
count: 4  event: labeled
count: 1  event: milestoned
count: 2  event: referenced         ← (коммит 22b49039 от merge PR #2010)
```

**Issue #1989 labels** (`gh api .../issues/1989 --jq '.labels[].name]'`):

```
ai-generated, source:gsd, type:functional, priority:high
```

Нет ни `e2e-done`, ни `no-e2e-required`, ни `needs-e2e`.

**PR #2010 body** (raw, фрагмент):

```
Closes: https://github.com/krikz/rob_box_project/issues/1989 (шаг 4б — merge делает товарищ Шифу)
```

**PR #2011 body** (raw, фрагмент):

```
Closes #1990
```

**PR #2010 commit** (единственный squash-источник, `git log --format=%b 22b49039 -1`):

```
Прямоточный путь речи оператора с левого грипа в avatar_supervisor
(§7.5 target-operator-agent-and-dialogue.md, инвариант 6c), НЕ агентский цикл:

- подписки /avatar/ptt/result (stt_node, шаг 05) и /avatar/voice_pipeline
  ({llm_enabled, preset, language} из панели шлема);
[... тело без единого "Closes" / "Fixes" / "Resolves" / "#1989" ...]

Co-authored-by: GOODWORKRINKZ <denis.grishko@pra-lab.ru>
```

**PR #2010 commit message head** (`gh api .../pulls/2010/commits`):

```
feat(supervisor): пайплайн грипа — /avatar/ptt/result + /avatar/voice_pipeline → /voice/tts/request, без ToolProvider (operator-agent 04b, #1989)
```

Содержит `#1989` как reference (link в issue), но **НЕ** содержит keyword `Closes/Fixes/Resolves`.

### 1.3 Почему НЕ сработали существующие защиты

| Слой | Что должен ловить | Почему не сработал |
|---|---|---|
| GitHub built-in parser | Парсит `Closes/Fixes/Resolves` в PR-body/merge-commit body | `squash_merge_commit_message: COMMIT_MESSAGES` — squash-commit содержит только commit-сообщения из PR, а в коммитах воркер пишет `#1989` (reference), не `Closes #1989` (keyword). Reference виден как link, но **НЕ триггерит close**. |
| ADR-0014 invariant `close <=> e2e-done ∧ MERGED` | Merge-gate закрывает issue при наличии `e2e-done` | У #1989/#1990 **никогда** не было метки `e2e-done` или `no-e2e-required` (timeline подтверждает: только 4 базовых метки, ни одной process-метки). Значит, e2e-process **не запускался** для этих issue → worker создал PR в обход нормального `needs-e2e` rotation → merge-gate не имел основания закрыть. |
| ADR-0014 path `no-e2e-required` (retro 19.08 #79779a21) | Закрытие по `no-e2e-required` метке | Метки нет. Worker не сигналил `no-e2e-required` ни в issue, ни в PR. |
| Human-close propagation (line 3046) | Если Шифу закрывает PR руками → закрыть issue | Здесь Шифу не закрывал PR руками, а мержил нормально. Авто-merge workflow. |
| Orphan-watchdog (line 225) | Авто-close orphan `needs-e2e` issues | У #1989/#1990 не было `needs-e2e`, поэтому watchdog их не видел. |

### 1.4 Гипотезы (триангуляция)

Карточка `t_fe771303` предлагала 3 гипотезы. Сырые данные:

- **(A) Squash-merge workflow не сохраняет PR-body → теряется `Closes #N`** — **ЧАСТИЧНО ПОДТВЕРЖДЕНО**: `squash_merge_commit_message: COMMIT_MESSAGES` действительно выкидывает PR-body. Но это **не** объясняет, почему merge-gate не закрыл issue. Гипотеза верна для native GitHub auto-close, но наш merge-gate auto-close зависит от меток, не от commit-message.
- **(B) GitHub парсит `Closes: #N` иначе, чем `Closes #N`** — **ОПРОВЕРГНУТО фактом**: PR #2011 использовал `Closes #1990` (без двоеточия, без полного URL) и **тоже не закрылся автоматически**. Значит формат keyword не причина — причина в squash-loss и в отсутствии меток для merge-gate.
- **(C) PR-merge commit ссылается через commit-message, но squash убивает body** — **ДУБЛИРУЕТ (A)**: та же механика, тот же repo setting.

**Реальный root cause** (комбинация двух факторов):

1. **(Squash-loss)** `squash_merge_commit_message: COMMIT_MESSAGES` означает, что squash-commit body НЕ содержит `Closes #N` из PR-body. Воркеры пишут `#1989` в commit-subject (reference) но НЕ keyword `Closes #1989` в commit-body. → GitHub native auto-close **не срабатывает**.
2. **(Contract mismatch)** merge-gate auto-close (ADR-0014) требует `e2e-done` или `no-e2e-required`. У #1989/#1990 не было ни одной process-метки → worker не прошёл e2e-rotation → PR в develop ушёл напрямую → Шифу закрывал руками.

Эти два фактора складываются: даже если бы squash-loss был починен (через (1)), всё равно без меток merge-gate не закрыл бы issue (через (2)). Для auto-close нужно закрыть ОБЕ дыры.

### 1.5 Бизнес-последствие (если не чинить)

- **Recurring ручной труд**: Шифу закрывает 100% issue руками после merge. На 100 merge/мес это ~5 минут чистого времени + риск забыть (issue висит OPEN неделями, как было в 2026-08-11 до ADR-0014 — issue #1082, #1104).
- **Discredit auto-close trust**: Шифу видит, что auto-close не работает → перестаёт верить merge-gate → пропускает легитимные проблемы (тот же t_1ca827a6 паттерн, где `triager` висел 2.4ч).
- **Race-condition воркер↔merge**: если воркер падает между merge и ручным close Шифу → issue сидит OPEN неопределённо долго → orphan растёт.
- **Audit-trail мусор**: ручной close Шифу ломает `closedByPullRequestsReferences` — поле пустое (см. evidence §1.2) → нельзя программно отследить «какой PR закрыл этот issue» через GitHub API. Это убивает downstream-аналитику (orphan-watchdog, retro-tracer, completion-check).

## 2. Где SOT и какие слои трогаем

### 2.1 SOT скриптов — `<repo>/scripts/agent_flow/*.sh`

Изменения — только в этом каталоге. Никаких ручных правок на роботе.

### 2.2 SOT данных — GitHub как SoT для issue state

Прямой `gh issue view`/`gh issue close`/`gh issue edit` — единственный источник истины. Локальных state-store нет.

### 2.3 SOT процесса — `~/.hermes/profiles/devops/cron/jobs.json`

Для варианта B (fallback-cron) — добавляется новая cron-задача.

## 3. Инвариант

```text
issue state transitions after PR MERGE into develop:

1. Если issue имеет label `e2e-done`              → close через whoami_close_issue (ADR-0014, уже есть).
2. Если issue имеет label `no-e2e-required`       → close через whoami_close_issue (retro 19.08, уже есть).
3. Если PR body содержит `Closes #N` / `Fixes #N` / `Resolves #N`
   для issue, у которого НЕТ process-меток        → close как fallback (NEW, ADR-0052 §4.1/4.2).
4. Иначе                                          → leave OPEN, defer до появления e2e-done
                                                     (или ручной close Шифу — это легитимный путь
                                                     для архитектурных PR, не прошедших e2e-rotation).
```

**Допущения** (явные):

- Close в ветке 3 — **fallback**, не primary path. Primary path остаётся `e2e-done` per ADR-0014. Этот путь срабатывает только если worker обошёл e2e-rotation (что бывает для архитектурных / docs / ADR PR).
- Idempotent: повторный вызов `gh issue close` на CLOSED issue = no-op. Поле `closedByPullRequestsReferences` GitHub обновит **только при native keyword-trigger** (наш fallback через `gh issue close` это поле НЕ заполнит). Это by-design: native trigger — keyword-based, наш fallback — explicit API. Acceptance criteria #2 (поле НЕ пусто) при нашем fallback не выполнится; см. §6 follow-up.
- Fail-OPEN: если `gh issue close` падает с rate-limit, тик откладывает до следующего, не крашит cron.
- User-reopen guard сохраняется: если Шифу переоткрыл issue между merge и fallback-tick → не закрываем повторно (как в ADR-0014).

## 4. Решение — два варианта + общий минимум

### 4.0 Общий минимум (обязателен в обоих вариантах)

**Защита от squash-loss (компенсация гипотезы A):**

В воркер-инструкции (или в `agent-flow-completion-check.sh` как PR-CI gate) — проверка, что **хотя бы один** из commit-сообщений в PR содержит keyword `Closes #N` / `Fixes #N` / `Resolves #N` для issue, указанного в PR-body. Если нет — warning в PR-comment, не блок (worker может сознательно обойти для архитектурных PR). Это **не** чинит native GitHub auto-close, но позволяет нашему merge-gate fallback (см. 4.1/4.2) иметь надёжный keyword-source: вместо парсинга squash-commit body, парсим PR-body напрямую через `gh pr view --json body`.

Минимальный код (для добавления в completion-check):

```bash
# ADR-0052 §4.0: keyword-loss warning для squash-merge.
# Парсим PR-body напрямую (не squash-commit body), т.к. squash теряет body.
_keyword_pat='(?im)(?:closes|fixes|resolves)\s+#(\d+)\b'
_pr_body="$(gh pr view "$pr" --repo "$GH_REPO" --json body --jq '.body' 2>/dev/null || echo '')"
_closes_kw="$(printf '%s' "$_pr_body" | grep -oE "$_keyword_pat" | head -5 || true)"
if [ -z "$_closes_kw" ]; then
    log "issue #${_issue}: WARN PR #${pr} body has no Closes/Fixes/Resolves keyword — fallback-close не сработает"
    # Non-fatal: PR может быть архитектурным (ADR-0014 §out-of-scope path).
fi
```

### 4.1 Вариант A — расширение merge-gate

**Изменения в `agent-flow-merge-gate.sh`** (после строки 3248, после `no-e2e-required` блока, перед основным case):

```bash
# ADR-0052 §4.1: fallback auto-close для issue без process-меток.
# Trigger: PR MERGED, issue OPEN, нет ни e2e-done, ни no-e2e-required,
# но PR-body содержит Closes/Fixes/Resolves keyword для этого issue.
# Это архитектурный/docs/lint PR — e2e-rotation не запускался, но PR
# явно сигналит intent закрыть issue через keyword.
#
# Контракт: только если PR.body содержит keyword для issue#N.
# Это исключает ложные срабатывания на PR, у которых в body просто
# упомянут issue#N как reference.
if [ "$pr_state" = "MERGED" ] && [ "$pr_base" = "$DEVELOP_BRANCH" ] \
    && [ "$_issue_state" = "OPEN" ] \
    && [ "$_has_e2e_done" = "0" ] \
    && [ "$_has_no_e2e" = "0" ]; then
    _pr_body="$(gh pr view "$pr_number" --repo "$GH_REPO" --json body --jq '.body' 2>/dev/null || echo '')"
    _kw_pat="(?im)(?:closes|fixes|resolves)\\s+#${number}\\b"
    if printf '%s' "$_pr_body" | grep -qE "$_kw_pat"; then
        log "issue #${number}: PR #${pr_number} body has Closes/Fixes/Resolves keyword → fallback auto-close (ADR-0052 §4.1)"
        whoami_close_issue "$number" "fallback auto-close: PR #${pr_number} MERGED into ${DEVELOP_BRANCH} with Closes keyword (ADR-0052)"
        if gh issue close "$number" --repo "$GH_REPO" --reason completed >/dev/null 2>&1; then
            log "issue #${number}: CLOSED via fallback path (reason=completed, ADR-0052 §4.1)"
            _issue_state="CLOSED"  # отражаем для case ниже → skip-close + cleanup
        else
            log "issue #${number}: WARNING gh issue close failed (fallback path, ADR-0052 §4.1) — retry next tick"
        fi
    fi
fi
```

**Плюсы**:

- Минимальный diff (≈20 строк в существующем файле).
- Синхронизирован с merge-событием (тот же тик merge-gate).
- Использует существующий `whoami_close_issue` helper (идемпотентный, 2ч окно).
- Не требует нового cron-инфраструктуры.

**Минусы**:

- Не сработает, если merge-gate не запускался в момент merge (например, deploy bot down). До следующего тика **любого** PR. На практике merge-gate тикает каждые 5 мин — ок.
- Сканирование PR-body на каждый merge → лёгкий оверхед (один `gh pr view`), но дешёвый.

### 4.2 Вариант B — отдельный cron-надзор

**Новый скрипт `scripts/agent_flow/agent-flow-issue-close-fallback.sh`** (every 5m):

```bash
#!/usr/bin/env bash
# ADR-0052 §4.2: fallback cron для auto-close issue без process-меток.
# Trigger: every 5m (идентично merge-gate).
# Action: для каждого MERGED PR за последние 24ч → проверить PR-body
# на Closes/Fixes/Resolves keyword → если issue OPEN без e2e-done →
# close (idempotent).

set -euo pipefail
HERMES_BIN="${HERMES_BIN:-hermes}"
KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
LOOKBACK_HOURS="${ISSUE_CLOSE_FALLBACK_LOOKBACK_HOURS:-24}"

# 1) Список недавно смерженных PR.
merged_prs="$(gh pr list --repo "$GH_REPO" --state merged \
    --json number,mergedAt,body --limit 200 \
    | python3 -c "
import sys, json, re
from datetime import datetime, timedelta, timezone
cutoff = datetime.now(timezone.utc) - timedelta(hours=int(sys.argv[1]))
d = json.load(sys.stdin)
for p in d:
    if datetime.fromisoformat(p['mergedAt'].replace('Z','+00:00')) >= cutoff:
        # Извлекаем issue numbers из Closes/Fixes/Resolves keyword.
        m = re.findall(r'(?im)(?:closes|fixes|resolves)\s+#(\d+)\b', p.get('body') or '')
        for n in m:
            print(f\"{p['number']} {n}\")
" "$LOOKBACK_HOURS")"

[ -z "$merged_prs" ] && exit 0

# 2) Для каждой пары (PR, issue) — проверить состояние, close если OPEN без process-меток.
while IFS=' ' read -r pr issue; do
    [ -z "$pr" ] && [ -z "$issue" ] && continue
    _state="$(gh issue view "$issue" --repo "$GH_REPO" --json state,labels --jq '{state,labels:[.labels[].name]}' 2>/dev/null || echo '')"
    [ -z "$_state" ] && continue  # API fail → skip (next tick)
    _st="$(printf '%s' "$_state" | python3 -c 'import sys,json; print(json.load(sys.stdin).get("state","unknown"))')"
    _lbls="$(printf '%s' "$_state" | python3 -c 'import sys,json; print(" ".join(json.load(sys.stdin).get("labels",[])))')"
    case "$_st" in
        closed|CLOSED)
                ;;  # уже закрыт — no-op
        open|OPEN)
                case "$_lbls" in
                    *e2e-done*|*no-e2e-required*)
                            ;;  # process-метка есть → merge-gate основной путь
                    *)
                            echo "issue-close-fallback: closing #${issue} (PR #${pr} merged, Closes keyword, no process-label)"
                            gh issue close "$issue" --repo "$GH_REPO" --reason completed \
                                --comment "auto-closed by issue-close-fallback cron (ADR-0052 §4.2): PR #${pr} MERGED into develop with Closes keyword" \
                                >/dev/null 2>&1 || true
                            ;;
                esac
                ;;
    esac
done <<< "$merged_prs"
```

**Cron-job registration** в `~/.hermes/profiles/devops/cron/jobs.json`:

```json
{
  "name": "agent-flow-issue-close-fallback",
  "schedule": "every 5m",
  "script": "<repo>/scripts/agent_flow/agent-flow-issue-close-fallback.sh",
  "deliver": "local"
}
```

**Плюсы**:

- Закрывает 100% кейсов, включая ситуацию «merge-gate не запускался в момент merge».
- Не зависит от merge-gate.
- Идемпотентен: повторный тик видит `state=CLOSED` → no-op.
- Легко отлаживается: один файл, один cron, понятный лог.

**Минусы**:

- Требует новый cron + регистрацию через `bash <repo>/scripts/agent_flow/install.sh` sync (5-мин задержка vs cleanup в момент merge для варианта A).
- Доп. нагрузка на `gh api` (лимит 5000 req/hour): `gh pr list --limit 200` ≈ 1 req / 5min = 12 req/час, `gh issue view` × N issues = ≤50 req/час — пренебрежимо.
- Логика дублируется с вариантом A (две точки кода для одной задачи).

### 4.3 Вариант C — ADR-0052 как формализация

Этот документ. Фиксирует контракт и trade-off'ы. **Не предлагает реализацию** — это выбор Шифу (Q22).

## 5. Рекомендация архитектора

**Вариант A + §4.0 общий минимум** (минимальная правка merge-gate + keyword-loss warning в completion-check).

**Обоснование**:

1. **KISS**: 95% кейсов закрываются без нового cron'а. Merge-gate тикает каждые 5 мин — это та же latency, что и fallback-cron (вариант B), но без дубликата кода.
2. **Синхронизация с merge-событием**: close происходит в tick, который знает про merge, а не в отдельном cron, который должен **сам** найти этот merge.
3. **Минимальный blast radius**: 1 файл merge-gate + 5 строк в completion-check, нет новой cron-инфраструктуры.
4. **Не дублирует логику ADR-0014**: тот же `whoami_close_issue` helper, тот же `_has_e2e_done`/`_has_no_e2e` pre-compute, та же case-statement архитектура. Diff естественно ложится в существующий pattern.

**Вариант B оправдан, если**: в течение 90 дней после merge варианта A всплывёт случай, когда merge-gate не был активен в момент merge и orphan-issue провисел >5 мин. На текущем бэклоге таких случаев не зафиксировано.

## 6. Что НЕ покрывает ADR-0052

- **`closedByPullRequestsReferences` остаётся пустым** при fallback close через `gh issue close`. Это поле GitHub обновляет **только** при native keyword-trigger (когда PR-body или merge-commit body содержит `Closes/Fixes/Resolves`). Наш fallback использует explicit `gh issue close` API, который это поле НЕ заполняет. **Implication**: если acceptance #2 карточки #2017 требует именно заполнения этого поля, то фикс требует **также** починки squash-template (`squash_merge_commit_message: COMMIT_MESSAGES → PR_BODY`, см. settings §1.2) или добавления keyword в commit-body воркерами. Оба варианта — **отдельные задачи** (squash-template — это настройка repo, commit-body — это воркер-инструкция), не входят в scope ADR-0052.
- **Issue, у которых PR-body содержит только reference `#N` без keyword** — fallback не сработает (по design). Если Шифу хочет, чтобы такие issue тоже закрывались автоматически, это отдельный ADR (расширение keyword-parsing).
- **Issue, у которых e2e-done label снят воркером после merge** (теоретический случай) — fallback не сработает (т.к. keyword-path требует «нет process-меток»). Шифу может re-merge фикс, но это вне scope.
- **Cross-board cleanup** (если у Шифу появится второй board) — ADR-0052 покрывает только `krikz/rob_box_project`. Для других boards — повторить.

## 7. Verification — как проверить, что фикс работает

После merge варианта A:

1. **Pre-fix state (snapshot сейчас)**:

   ```bash
   gh pr list --repo krikz/rob_box_project --state merged --limit 10 \
     --json number,body,mergedAt | python3 -c "
   import sys, json, re
   d = json.load(sys.stdin)
   for p in d:
       kw = re.findall(r'(?im)(?:closes|fixes|resolves)\s+#(\d+)\b', p.get('body') or '')
       print(f\"PR #{p['number']} → closes {kw} (merged {p['mergedAt']})\")"
   ```

   Ожидаемо: для #2010 и #2011 — keyword присутствует в PR-body.

2. **Симуляция merge**: можно вручную re-open один из #1989/#1990 (`gh issue reopen 1989 --repo krikz/rob_box_project`), дождаться следующего merge-gate тика (≤5 мин) → проверить, что issue auto-close через fallback path (в логе merge-gate: `issue #1989: PR #2010 body has Closes keyword → fallback auto-close`).

3. **Post-fix state**:

   ```bash
   gh issue view 1989 --repo krikz/rob_box_project --json state,closedByPullRequestsReferences
   gh issue view 1990 --repo krikz/rob_box_project --json state,closedByPullRequestsReferences
   ```

   Ожидаемо: `state: closed`. `closedByPullRequestsReferences: []` (пусто, см. §6 — это by-design limitation; для заполнения этого поля нужен squash-template fix).

4. **Re-trigger pattern**: создать тестовый PR через `gh pr create --body 'Closes #2017'` (закрыть невозможно т.к. #2017 open), проверить, что после merge-gate tick issue auto-close. Альтернативно: re-merge существующий PR через `gh pr reopen ... --comment 're-merging'` (если возможно).

## 8. Решение принимает Шифу (Q22)

Вариант A vs B vs A+B vs «ничего». Acceptance:

- [ ] **Шифу решает** (Q22): какой вариант (A, B, оба, ничего).
- [ ] Если выбран A или A+B — assignee=devops, max_runtime=1800s.
- [ ] Sync 3 копий скрипта через `bash <repo>/scripts/agent_flow/install.sh` + `md5sum` проверка (текущая процедура Шифу от 14.08).
- [ ] После фикса — **никакого** retroactive cleanup вручную: ADR-0052 защищает **только новые** merge. Существующие #1989/#1990 уже закрыты Шифу, orphan-pattern не активен.

## 9. Источники истины

- `scripts/agent_flow/agent-flow-merge-gate.sh` (line 3159 — ветка `MERGED`; line 3198 — `_has_no_e2e`; line 3229 — `whoami_close_issue` для no-e2e-required; line 3046 — human-close propagation).
- `scripts/agent_flow/agent-flow-completion-check.sh` (PR-CI gate, для §4.0).
- `scripts/agent_flow/agent-flow-blocked-watchdog.sh` (line 225 — `gh issue close` для orphan needs-e2e; для сравнения).
- `docs/adr/0014-agent-flow-issue-closure.md` (ADR-0014 invariant).
- `docs/adr/0046-orphan-cleanup-after-merge.md` (cleanup kanban-карточек, не issue — для отличия).
- `docs/adr/0049-nightly-review-cycle.md` (эта карточка — пример того, что nightly-review должен ловить).

## 10. Verification log (для будущего надзора)

Дата верификации: 2026-09-07 (карточка t_fe771303).
Метод: прямой `gh api` к events/timeline/labels для issue #1989/#1990 + `gh pr view` для #2010/#2011 + `gh api repos/...` для repo settings + `git log --format=%b` для merge-commit body.
Результат: гипотеза (A) подтверждена частично (squash-loss), (B) опровергнута (формат keyword не причина), (C) дублирует (A). Реальный root cause — комбинация squash-loss + contract mismatch (ADR-0014 invariant требует e2e-done, которого не было).

Рекомендация по re-verification: через 30 дней после merge ADR-0052 — повторить §7.1-7.3, убедиться, что для всех merged-PR с `Closes/Fixes/Resolves` в body issue закрыты в течение ≤5 мин (тик merge-gate). Если >2 orphan — открыть новую ретро-карточку `issue-close-fallback-recurring` и эскалировать.