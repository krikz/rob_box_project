# FS-001 — fail-streak auto-issue (watchdog → GitHub)

**Дата:** 2026-09-15
**Автор:** architect worker (kanban t_3adf8c8c, ишшу #2486)
**Контекст:** `scripts/agent_flow/agent-flow-e2e-fail-streak-watchdog.sh`
**Статус:** accepted (формализует ранее внедрённое поведение из ретро t_401e52de)
**Связанные тикеты:** t_401e52de (ретро-карточка источник), t_3adf8c8c (формализация)
**Связанные артефакты:** PR #2374 (внедрение watchdog), PR #2417 (документация/README)

## Контекст

E2E watchdog (`agent-flow-e2e-fail-streak-watchdog.sh`) обнаруживает цепочку
подряд падающих `L: E2E Voice Test` прогонов. Ретро t_401e52de (август 2026)
показало: **8 fail-прогонов подряд прошли молча без issue** — process-gap.
Люди узнавали о регрессии только когда тишина становилась слишком длинной.

Внедрённая (PR #2374) и задокументированная (PR #2417) реакция watchdog'а —
авто-создание ОДНОГО issue с лейблом `e2e-fail-streak` при достижении порога.
Эта карточка **формализует** поведение как ADR, чтобы у читателя было
каноническое место для ответа на «почему именно так», а не разбросанные
комментарии в README/shell-скрипте/CHANGELOG.

До этой формализации шесть ссылок на `ADR-FS-001` (README L706, L713;
watchdog L38, L276; test L4; CHANGELOG L490) вели в пустоту — phantom ADR.
Issue #2486 исправляет это: создаётся `docs/adr/FS-001-...md`.

## Решение

При fail-streak ≥ `E2E_FAIL_STREAK_ISSUE_THRESHOLD` (default 5) watchdog
**создаёт ровно один** issue с лейблом `e2e-fail-streak`, который несёт в себе
полную диагностику:

- Timeline последних 8 failed runs (`id / conclusion / createdAt / headSha[7] / headBranch`).
- `origin/develop` HEAD SHA — для контекста «какой baseline был на момент первого fail».
- Релевантные merged PR за последние 5 дней, спарсенные из
  `git log origin/develop --merges` (ищутся `#NNNN` в заголовках merge-коммитов).
- Гипотеза `music-fix regression` со ссылками на известные корреляции
  (#2246 / #2347) и на issue-ссылки из последних PR.

Тело issue собирается в bash (без markdown-темплейтов) и передаётся через
`gh issue create --body-file`.

## Гарантии идемпотентности

Два **независимых** guard'а защищают от шторма issues:

### (a) Local rate-limit — `ISSUE_COOLDOWN_FILE`

Путь: `$HERMES_HOME/state/agent-flow-e2e-fail-streak-last-issue` (default
`~/.hermes/state/...`). Содержит epoch последнего успешного создания issue.

Если файл моложе `E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS` (default 4ч) →
skip. Это защищает **между тиками** в одном HA-окружении.

### (b) GitHub-truth — `gh issue list --label e2e-fail-streak`

Если уже есть **хотя бы один ОТКРЫТЫЙ** issue с этим лейблом → skip.
Это защищает от дублей при **потере state-файла** (перезапуск контейнера,
сбой диска, ручное удаление). Локальный state и удалённая правда
дивергируют — GitHub-truth побеждает.

Связка «local mtime + GitHub-truth» даёт два уровня дедупликации,
которые покрывают разные сценарии отказа.

## Auto-pause (отдельный sentinel, не часть этого ADR)

При streak ≥ `E2E_FAIL_STREAK_PAUSE` (default 20) watchdog создаёт
sentinel-файл `$HERMES_HOME/state/agent-flow-e2e-fail-streak-pause`, который
`agent-flow-e2e-process.sh` читает в начале каждого tick и пропускает round
creation. Manual override — удаление файла. Логически это **соседний
механизм** (auto-pause → защита от runaway), а не часть auto-issue.
Здесь упомянут для полноты картины; если понадобится отдельный ADR для
pause-sentinel — создадим его явно (сейчас не phantom: pause описан в
README L725-735).

## ENV-контракт (SOT для порогов и путей)

```text
GH_REPO                                       — owner/repo (default krikz/rob_box_project)
FAIL_STREAK_DRY_RUN=true                      — log only, без API writes
E2E_FAIL_STREAK_WARN=5                        — порог алерта (issue comment)
E2E_FAIL_STREAK_PAUSE=20                      — порог auto-pause (sentinel)
E2E_FAIL_STREAK_LIMIT=30                      — сколько последних run'ов смотрим
E2E_FAIL_STREAK_DEDUP_HOURS=6                 — дедупликация алерт-комментариев
E2E_FAIL_STREAK_ISSUE_THRESHOLD=5             — порог для auto-create issue
E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS=4      — дедупликация создания issue
E2E_FAIL_STREAK_ISSUE_LABEL=e2e-fail-streak   — лейбл нового issue
E2E_FAIL_STREAK_ISSUE_ASSIGNEE=               — assignee (пусто = без assignee)
HERMES_HOME                                   — для sentinel path (default ~/.hermes)
REPO_DIR                                      — путь к локальному clone репо
                                               (для `git -C` develop HEAD + merges)
LOCK_FILE                                     — flock guard
```

Любая правка дефолта = PR в `agent-flow-e2e-fail-streak-watchdog.sh` +
комментарий «env-контракт изменился» в этом ADR. Сейчас дефолты
**согласованы с README L715-735**.

## Альтернативы (рассмотрены и отвергнуты)

| Альтернатива                       | Почему нет                                                       |
| ---------------------------------- | ---------------------------------------------------------------- |
| Комментарий-алерт вместо issue     | Не остаётся в triage'е; Шифу видит только когда смотрит issue    |
| Каждый tick создаёт issue          | Шторм из 100+ issues за ночь (8 ticks × 12 fail-rounds)          |
| Только GitHub-truth без local      | Потеря state = шторм при первом же restart                        |
| Только local mtime без GitHub-truth | Потеря state-файла = шторм при первом же restart                  |
| Email/Slack алерт                  | Вне scope cron-инфраструктуры, требует секретов в env крона      |

## Pitfalls

- `gh run list --workflow` принимает **filename** на DEFAULT branch, не display
  name. Workflow display name «L: E2E Voice Test» на main == filename
  «L-E2E Voice Test.yml». Используем filename.
- Без `--branch`: берём все round-ветки (`z-{e2e}/test-round-*`) +
  develop fallback — это intentional, чтобы видеть cross-branch regressions.
- Тест `tests/test_e2e_fail_streak_auto_issue.sh` использует PATH-hijack
  mock-gh / mock-git (без сети), покрывает 5 сценариев:
  S1 streak < threshold, S2 DRY-RUN, S3 cooldown active, S4 existing issue,
  S5 label-mismatch (open issue с другим лейблом → всё равно создаём).
- `ISSUE_COOLDOWN_FILE` живёт в `$HERMES_HOME/state/` — не в `/tmp`,
  чтобы пережить перезапуск контейнера.

## Тест-контракт

Регресс-тест `scripts/agent_flow/tests/test_e2e_fail_streak_auto_issue.sh`
обязан запускаться в CI и зелёный после любой правки watchdog'а.
Сценарии S1-S5 покрывают **все ветки guard'ов**.

## Почему отдельный FS- префикс (а не 0096 в общей нумерации)

Существующая нумерация ADR — `NNNN-` (общие архитектурные решения) и
`AF-NNNN-` (agent-flow конкретные процессы). ADR-FS-001 вводит третий
префикс `FS-NNNN-` для **fail-streak**-семейства watchdog'ов. Это сделано
намеренно: fail-streak может породить соседние ADR'ы (pause-sentinel,
rate-limit tuning, threshold tuning), которые логически группируются
отдельно от общих agent-flow решений.

Если Шифу/команде это неудобно — переименуем в `AF-0066-...`, но
`FS-` честнее отражает scope.

## Связанные артефакты

- kanban: t_401e52de (ретро), t_3adf8c8c (формализация)
- коммиты: `a9b0498` ([t_401e52de]), `70b3b5c0` (docs)
- PR: #2374 (внедрение), #2417 (README), #2486 (этот phantom-fix)
- issue: #2486 (tracker)