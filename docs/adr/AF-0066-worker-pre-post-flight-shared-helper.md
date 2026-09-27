# ADR-AF-0066: Общий хелпер для worker_pre_flight / worker_post_flight — ликвидация дублирования rebase-протокола

**Статус:** предложен (черновик архитектора для issue #2476)
**Репортёр:** architect (kanban t_27126ab1)
**Severity:** MEDIUM — текущая боль (PR #2443 → дубли в двух скриптах), ночной alert-спам (ADR-0014-cite-context-ext), а не hard FAIL
**Дата:** 2026-09-15

## Контекст

Read-only ревью компонента `scripts/agent_flow` за 2026-09-14 (kanban `t_b969454d`,
окно `a026f9ed → 9b6ac0b`, 14 коммитов, +5342/-78 LOC) зафиксировало два NEW-скрипта:
**worker_pre_flight.sh** (issue #2438, PR #2443) и **worker_post_flight.sh** (там же),
реализующих протокол «fetch → behind-count → warn → auto-rebase → push» независимо
друг от друга. Карточка `t_27126ab1` (= этот документ) — issue-репорт по результатам
ревью.

### Проблема D-1: полное дублирование тела

`worker_pre_flight.sh` и `worker_post_flight.sh` — byte-identical (или near-identical)
в четырёх крупных блоках:

| Блок                                          | worker_pre_flight.sh | worker_post_flight.sh |
|-----------------------------------------------|----------------------|------------------------|
| args/env/guard                                | 47-82 (36 строк)     | 43-79 (37 строк)       |
| `log()`, `post_comment()`                     | 84-113 (30 строк)    | 79-105 (27 строк)      |
| fetch + behind-count                          | 115-148 (34 строки)  | 124-138 (15 строк)     |
| stash + rebase + pop + push                   | 183-227 (45 строк)   | 183-236 (54 строки)    |

`diff <(sed -n '47,113p' worker_pre_flight.sh | sed 's/worker_pre_flight/X/g') \
      <(sed -n '43,105p' worker_post_flight.sh | sed 's/worker_post_flight/X/g')`
показывает только разницу в SKIP-переменных и имени лог-префикса. Всё остальное —
копипаста с уже начавшимся drift (post_flight добавил `push-via-gh-api.sh` для
force-push, pre_flight — нет; pre_flight рапортует behind/AHEAD, post_flight —
только behind).

### Проблема D-2: обход `post_whoami_comment` / `comment_recently_posted`

Оба скрипта делают **прямой** `gh issue comment --body-file`:

```bash
# worker_pre_flight.sh:105-106
if GH_CONFIG_DIR="$GH_CONFIG_DIR" gh issue comment "$ISSUE_NUM" \
        --repo "$GITHUB_REPO" --body-file "$tmp" >/dev/null 2>&1; then
```

Это нарушает ADR-0014-cite-context-ext (`hermes_github.sh:1-84`):

1. **Нет self-id-marker'а.** В журнале событий GitHub actor = владелец токена
   (krikz), без `[agent:devops]` / `script=...`. Невозможно отличить
   «это cron-алерт от worker_pre_flight» от «товарищ Шифу руками».
2. **Нет idempotency через `comment_recently_posted`.** Если воркер упал посередине
   (timeout, OOM, docker restart), тот же alert будет спамиться при каждом
   post-flight re-try. Сценарий: воркер застрял в rebase-merge state → 4 раза
   retry → 4 одинаковых «🚨 rebase CONFLICT» комментария в issue.

Дополнительно: `post_whoami_comment` (`hermes_github.sh:192`) уже умеет всё нужное
(self-id prefix, dedup-через-`_whoami_already_posted`, `kind/number/action/reason`),
но требует семантически более узкого контракта (action-таксономия), что не подходит
для **ad-hoc informational-комментариев** воркера. Значит нужен **не отдельный
вызов post_whoami_comment, а общий хелпер, который использует тот же backend
(`comment_recently_posted`)**.

## Решение

Двухслойный рефакторинг, минимальный по touch и совместимый с install.sh:

### Слой 1: выделить общие части в `scripts/agent_flow/_worker_flight_common.sh`

Sourced-библиотека (аналогично `lib_agent_flow_common.sh` и `hermes_github.sh`),
содержащая:

- `_wfc_init <flight_name> <task_id> <branch_name> [issue_num]`
  — парсит args/env/guard (SETUP общих переменных: `WFC_TASK_ID`,
  `WFC_FLIGHT_NAME`, `WFC_ISSUE_NUM`, `WFC_MAX_BEHIND`, `WFC_SKIP_VAR`,
  `WFC_GH_REPO`, `WFC_GH_CONFIG_DIR`, `WFC_BASE_REF`, `WFC_CURRENT_BRANCH`).
- `_wfc_log <msg...>` — единый лог с `[$WFC_FLIGHT_NAME $WFC_TASK_ID]` prefix
  (заменяет две копии `log()`).
- `_wfc_post_comment <body>` — комментарий **через** `comment_recently_posted`
  (kind=issue, marker = префикс `[WFC_FLIGHT_NAME]` в body, window=300s по
  умолчанию → дедупликация при re-try). Не использует `post_whoami_comment`,
  потому что тот заточен под action-таксономию (closing/reopening/…); здесь
  informational-комментарий воркера, не привязанный к side-effect.
- `_wfc_fetch_origin` — fetch + явный refspec (см. ретро `t_730ea7b1`).
- `_wfc_drift_count` → `WFC_BEHIND`, `WFC_AHEAD` (через `git rev-list`).
- `_wfc_stash_and_rebase <timeout_seconds>` → stash name + rebase rc
  (0 = ok, 1 = conflict, 2 = stash pop conflict).
- `_wfc_stash_pop` — поп pop после rebase.

### Слой 2: привести оба скрипта к тонкой обёртке над хелпером

`worker_pre_flight.sh` сводится к:

```bash
. "$(dirname "$0")/_worker_flight_common.sh"
_wfc_init pre_flight "$@"
[ "${SKIP_PRE_FLIGHT:-}" = "true" ] && exit 0

# step 1: fetch + drift
_wfc_fetch_origin || { _wfc_post_comment "⚠️ fetch failed"; exit 0; }
_wfc_drift_count

# step 2: no-op if up-to-date
[ "$WFC_BEHIND" -le 0 ] && { _wfc_log "up-to-date"; exit 0; }

# step 3: warn if drift ≤ MAX, no rebase
[ "$WFC_BEHIND" -le "$WFC_MAX_BEHIND" ] && { _wfc_log "small drift"; exit 0; }

# step 4: heavy drift → auto-rebase
_wfc_post_comment "⚠️ branch is $WFC_BEHIND behind; auto-rebase started"
_wfc_stash_and_rebase 300
case $? in
    0)  _wfc_post_comment "✅ rebase OK"; exit 0 ;;
    1)  _wfc_post_comment "🚨 rebase CONFLICT"; exit 1 ;;
    2)  _wfc_post_comment "⚠️ stash pop conflict"; exit 1 ;;
esac
```

`worker_post_flight.sh` — аналогично, плюс вставка scope-check (он уже отдельной
библиотекой, не часть общего протокола) и push через `push-via-gh-api.sh`.

### Контракт `comment_recently_posted` для воркеров

Префикс маркера = `[worker_pre_flight]` / `[worker_post_flight]` (flight_name в
квадратных скобках в начале body). Window = `WFC_DEDUP_WINDOW_SECONDS` env
(default 300s = 5 минут; больше, чем timeout воркера, но достаточно, чтобы
re-try не дублировал). Если marker встречается в issue-за последние 5 минут →
silent skip (worker_logged "dedup: skipped"), exit 0.

Это решает обе проблемы ADR-0014-cite-context-ext **для воркеров**:
- **actor**: остаётся владелец токена (как у всех gh-вызовов от cron), но
  **body содержит `[worker_pre_flight]` / `[worker_post_flight]`** — это self-id
  в формате, который согласован с `[agent:devops]` (читается grep'ом).
- **dedup**: через существующий backend, без новой логики.

### Регистрация

`_worker_flight_common.sh` добавляется в EXPECTED-список `install.sh` (как
`hermes_github.sh` и `lib_agent_flow_common.sh`). Drift-detect будет контролировать,
по аналогии с `worker_pre_flight.sh` / `worker_post_flight.sh`.

## Trade-offs

- **Новый файл vs добавить в `lib_agent_flow_common.sh`.** Решили **новый файл**
  (`_worker_flight_common.sh`). Причины:
  1. `lib_agent_flow_common.sh` — общий код agent-flow процессов (cron);
     `_worker_flight_common.sh` — общий код worker'ов (вручную запускаемые).
     Разные источники вызова, разные ENV-ожидания.
  2. Размер: lib уже 974 строки, новые 80-120 строк rebase-протокола — лишний
     noise для cron-скриптов, которые никогда не делают rebase.
  3. Если в будущем протокол разрастётся (например, добавим pre-push hook,
     bisect-fallback), `_worker_flight_common.sh` останется изолированным.
  Минус: ещё один файл в EXPECTED install.sh. Плюс: чистое разделение
  ответственности.

- **Использовать `post_whoami_comment` vs `comment_recently_posted` напрямую.**
  Решили **`comment_recently_posted`** (а не `post_whoami_comment`). Причины:
  1. `post_whoami_comment` заточен под **side-effect → перед ним комментарий**:
     action ∈ {closing, reopening, adding-label, …}. У воркера informational-
     комментарий «branch behind N commits» — НЕ привязан к side-effect.
     Натягивание семантики «worker doing X» на action-таксономию — лишний шум.
  2. `comment_recently_posted` уже имеет нужный backend (см. `hermes_github.sh:298-389`):
     `kind/number/marker/window_seconds/mode` — воркер передаёт
     `kind=issue`, `marker="[worker_pre_flight]"`, `window=300s`.
  3. `post_whoami_comment` имеет встроенный self-id-prefix (`🤖 [agent:devops]`) —
     **избыточен** для воркеров, у которых уже есть свой `[worker_pre_flight]`-
     prefix в body.
  Минус: не получаем автоматический «🤖 [agent:devops] script=…» — воркеры должны
  сами включать flight_name в body (мы уже это делаем). Плюс: переиспользуем
  проверенный dedup-backend.

- **Window 300s vs 7200s (default `HERMES_WHOAMI_WINDOW_SECONDS`).** Решили
  **300s**. Причины:
  1. Воркер — это одна сессия (один dispatch), не recurring cron. Если алерт уже
     был запощен — значит воркер на нём завершился/застрял. Повторный re-try
     через >5 минут — другая попытка, новый комментарий оправдан.
  2. 7200s приведёт к тому, что при real rebase-CONFLICT воркер НЕ сможет
     повторно напомнить о конфликте 2 часа — теряем observability.
  Override через `WFC_DEDUP_WINDOW_SECONDS` если потребуется.

- **Сводить scope-check в общий хелпер или оставить отдельным.** Решили
  **оставить отдельным** (`worker_scope_check.sh` уже выделен, только post_flight
  его зовёт). Причины: scope-check относится только к post_flight (после работы),
  pre_flight ещё нечего проверять (воркер только начал).

## Альтернативы (рассмотренные и отклонённые)

- **Объединить pre_flight и post_flight в один `worker_flight.sh` с подкомандой.**
  Минус: усложняет вызов из воркер-протокола (две строки vs одна), ломает
  существующие тесты и PR #2443 / #2478 (BASE_REF fail-fast). KISS: оставляем
  два скрипта-обёртки.

- **Вынести в `lib_agent_flow_common.sh`.** Минус: см. trade-off #1 — смешение
  cron-кода и worker-кода.

- **Использовать готовый `post_whoami_comment`.** Минус: семантика action-таксономии
  не подходит для informational-комментов воркера.

- **Не делать ничего (оставить как есть).** Минус: ADR-0014-cite-context-ext
  нарушается уже сейчас; при следующем drift баг будет чиниться в двух местах.
  Plus, если завтра появится `worker_mid_flight.sh` (для long-running воркеров)
  — снова три копии.

## Acceptance (что считается «готово» для реализации)

- [ ] `scripts/agent_flow/_worker_flight_common.sh` создан (~80-120 строк, sourced)
- [ ] `worker_pre_flight.sh` сводится к вызовам `_wfc_*` (≤60 строк working code)
- [ ] `worker_post_flight.sh` сводится к вызовам `_wfc_*` (≤80 строк working code)
- [ ] `_wfc_post_comment` использует `comment_recently_posted` из `hermes_github.sh`
      (kind=issue, marker=`[worker_<flight>]`, mode=prefix)
- [ ] `_worker_flight_common.sh` зарегистрирован в `EXPECTED` `install.sh`
- [ ] `tests/test_worker_pre_flight.sh` зелёный (8/8 сценариев)
- [ ] `tests/test_worker_post_flight.sh` зелёный (8/8 сценариев)
- [ ] `tests/test_hermes_github.sh` зелёный (dedup логика не сломана)
- [ ] Diff ≤ +250/-300 LOC net (большая часть — удаление копипаста)
- [ ] Manual smoke: запустить `bash worker_pre_flight.sh t_test1234 z-test/branch`
      в фикстуре из test 7 (behind=66) → exit 0, в issue появляется комментарий
      с `[worker_pre_flight]`-prefix (raw-evidence: `gh issue view <N> --comments`)

## Где это уже записано (после PR)

- `scripts/agent_flow/_worker_flight_common.sh` — shared helper (новый).
- `scripts/agent_flow/worker_pre_flight.sh` — thin wrapper (≤60 строк).
- `scripts/agent_flow/worker_post_flight.sh` — thin wrapper (≤80 строк).
- `scripts/agent_flow/install.sh` — `_worker_flight_common.sh` в EXPECTED.
- `docs/adr/AF-0066-worker-pre-post-flight-shared-helper.md` — этот документ.
- `docs/adr/AF-0066-worker-pre-post-flight-shared-helper-design.md` — design note
  с API-спецификацией `_wfc_*`.

## Связанные артефакты

- kanban `t_b969454d` — read-only review, 4 subagent reports on disk
- kanban `t_27126ab1` — эта карточка (issue-репорт + ADR-черновик)
- commit `1dc4c8af`, issue #2438, PR #2443 (worker_pre_flight + worker_post_flight)
- commit `563c4b62f` (PR #2478) — BASE_REF honor + fail-fast on branch mismatch
  (touch worker_pre_flight.sh — должен быть совместим с новым helper'ом)
- ADR-0014-cite-context-ext (`hermes_github.sh:1-84`) — обоснование self-id
- `lib_agent_flow_common.sh:148-176` (`af_flock_guard_or_exit`) — пример
  существующего shared helper'а
- `hermes_github.sh:192` (`post_whoami_comment`) — отвергнутая альтернатива
- `hermes_github.sh:298-389` (`comment_recently_posted`) — выбранный backend