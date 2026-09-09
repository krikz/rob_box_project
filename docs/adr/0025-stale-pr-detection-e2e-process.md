# ADR-0025: Stale-PR detection в e2e-process — pre-dispatch gate для test-branch

| Поле | Значение |
|---|---|
| Статус | Proposed |
| Дата | 2026-08-22 |
| Автор | devops (Hermes Agent), kanban t_a2cd5753 |
| Контекст | Fail-streak round-166/167/168/170/178 (5 из 7 за 24ч) — `e2e-process` создавал `test-branch` от develop, потом мержил PR tip, унаследовавший устаревший сценарий/harness |
| Затрагивает | `scripts/agent_flow/agent-flow-e2e-process.sh`, `agent-flow-e2e-process` cron (every 1h), формат issue-комментариев |
| Родители | ADR-0014 (issue-closure on merge), ADR-0015 (e2e verdict SOT), ADR-0022 (process e2e done gates) |
| Связанные | issue/verdict `t_fb037ed1` (W1 root cause), `t_562a8682` (merge-gate stale-rebase watchdog — другой кейс: уже-влитая ветка), PR #1517 (stale tip 7d36af73, отстаёт от db84ff59) |

## 1. Контекст и бизнес-проблема

22.08 в 18:00–19:00 произошёл fail-streak: 5 из 7 e2e-раундов упали на одном и том же PR
(#1517, ветка `z-{agent}/1506-task-voice-e2e-command-gate-new-session-`, tip `7d36af73`).
Расследование архитектора (`t_fb037ed1/verdict-fail-streak-2026-08-22.md`, §W1)
установило:

- PR #1517 открыт ДО того, как commit `db84ff59` (GATE-1 empty `--since`, drop
  dead `generate_music`, voice-agnostic mv01/mv02) был влит в develop.
- e2e-process создал `z-{e2e}/test-round-178` от актуального develop
  (с `db84ff59`), но при мерже PR tip в round — старый `7d36af73` принёс с собой:
  - `.github/e2e/scenarios/voice_core_suite_v1.json` со старыми `patterns = ["alena"]`
  - `.github/workflows/scripts/e2e_voice_test.sh` БЕЗ `E2E_RUN_BEFORE`
- Round-178 fail: `mv01/mv02 PATTERN_MISS: alena` + `dj02 expected tool calls not
  invoked: stop_music` + `GATE-1 ❌ expected tool calls not invoked during run:
  set_voice, execute_music_code, stop_music`.
- Round-182 (после рекомбинации PR tip с develop) — SUCCESS. Регрессий нет.

**Это НЕ баг кода, это архитектурный долг:** `e2e-process` не умеет детектить
«PR tip отстаёт от develop HEAD → диспатчить опасно». Существующий
`stale_branch_scan_all` в `agent-flow-merge-gate.sh` (ретро 12.08 `t_d3aeaa9b` /
14.08 `t_28afb585`) проверяет **другой** кейс — ветка уже влита через ДРУГОЙ PR
(переиспользование). Наш случай — ветка НЕ влита, просто старый tip.

## 2. Решение

Pre-dispatch gate в `agent-flow-e2e-process.sh`:

1. **Переменная окружения** `E2E_STALE_BRANCH_THRESHOLD` (default `10`) — допустимое
   отставание PR tip от develop HEAD в коммитах.
2. **Функция `stale_branch_check`** (после `find_open_pr_by_issue`):
   - `git fetch origin develop <branch>` (свежие refs)
   - `git rev-list --count origin/<branch>..origin/develop` — коммиты в develop,
     которых нет в PR tip.
   - Если `rev-list <= threshold` → return 0 (OK).
   - Если `rev-list > threshold` → return 1 (BLOCKED) + идемпотентный
     issue-коммент с инструкцией `git fetch origin develop && git rebase origin/develop`.
3. **Два вызова:**
   - **Pre-round guard** (после `active_round_with_issue` dedup, перед
     `live_candidates++`) — если stale → `continue`, не считать кандидатом.
     Если ВСЕ кандидаты stale → `live_candidates=0` → round НЕ создаётся (логика
     `t_4212e8ad`).
   - **Pre-merge re-check** (перед `git merge --no-ff` в основном цикле) — на
     случай, если PR был fresh при issue-list snapshot, но develop убежал между
     snapshot и merge.

## 3. Почему именно threshold=10, а не абсолютный запрет

- Типичный PR workflow: rebase раз в 2-5 дней. threshold=10 даёт ~3-4 rebase'а
  «запаса», прежде чем нас начнут беспокоить.
- Аддитивные PR (docs/ci, fixes без удаления) задевают 0-3 коммита develop.
  threshold=10 их не задевает.
- Кейс «PR открыт ДО крупного фикса в harness/scenario» (наш fail-streak) —
  10+ коммитов develop обычно успевают пройти.

При желании override через env: `E2E_STALE_BRANCH_THRESHOLD=30 bash agent-flow-e2e-process.sh`.

## 4. Альтернативы, которые отвергли

### 4.1 Snapshot develop в test-branch (вариант A из S1 вердикта)

Делать `git checkout origin/develop -- .github/e2e/scenarios/voice_core_suite_v1.json
.github/workflows/scripts/e2e_voice_test.sh` после merge PR tip. Брать suite/harness
**всегда** из develop, остальное — из PR.

**Почему нет:** сценарий про voice, но PR может нести фиксы в `dialogue_node`,
которые **тоже** должны попасть в e2e. Если «снапшотить» только scenario/harness —
потеряем часть тестируемого кода. Неполное.

### 4.2 Запрет stale-PR в merge-gate (перед `needs-e2e`)

`stale_branch_scan_all` в `merge-gate` тоже отказывается ставить `needs-e2e`
на stale PR. Тогда e2e-process вообще никогда не увидит этот PR.

**Почему нет:** `merge-gate` ставит `needs-e2e` не на ВСЕ PR с
`agent:*` метками, а только когда `CI зелёный`. Если у PR conflict с develop
(самый частый случай stale-PR) — CI **красный** → `needs-e2e` не ставится →
запрет не сработает в нужный момент. Stale-PR может появиться **после** того,
как `needs-e2e` уже поставлен (между `merge-gate` тик и `e2e-process` тик).
Поэтому защита нужна в обоих местах, **но** `e2e-process` — последний рубеж
(он реально создаёт test-branch).

### 4.3 Force rebase перед merge в test-branch

`git rebase origin/develop` для PR tip перед merge в test-round-N.

**Почему нет:** меняет PR tip на remote (через `--force-with-lease`).
Это **неожиданное** действие автоматики — Шифу прямо просил «не плодить
ветки» и «rebase в той же ветке, руками». Автоматический rebase чужих PR —
out of scope процесса.

## 5. Acceptance criteria

- [x] Pre-dispatch check добавлен в `agent-flow-e2e-process.sh` (~30 строк bash).
- [x] Pre-merge re-check добавлен (защита от re-stale в том же тике).
- [x] Идемпотентный issue-коммент (24h окно).
- [x] Юнит-тесты: 6 сценариев (`scripts/agent_flow/tests/test_e2e_process_stale_branch.sh`).
- [x] Round НЕ создаётся если ВСЕ кандидаты stale (через `live_candidates=0`).
- [x] Счётчик раундов не инкрементируется при всех stale.

## 6. Backwards compatibility / failure modes

- **Fetch fail (offline / 403):** `stale_branch_check` → return 0 (fail-safe, льём
  в round как обычно). Логирует warning.
- **Rev-list fail / sha empty:** то же, fail-safe. Безопаснее дать e2e
  прогнаться на устаревшей базе, чем сломать ротацию.
- **gh api fail при dedup (деградация API):** `|| echo 0` → `_existing=0` →
  пишем новый comment. Возможно лёгкий спам раз в 24h, не критично.
- **`E2E_STALE_BRANCH_THRESHOLD=0`:** любой PR tip с `rev-list > 0` → BLOCKED.
  Полезно для тестов и при «особо важных» раундах. Не default.

## 7. Когда поднимать threshold / отключать

- После успешного merge крупного рефакторинга в develop (где PR'ы будут
  естественно отставать) — увеличить threshold через cron env на 24-48ч.
- При обнаружении ложного срабатывания (PR tip fresh, но rev-list > threshold
  из-за неполной выборки develop в локальном git — например, force-push PR'а
  без fast-forward). Логирование: `stale-branch: 🛑 BLOCKED` всегда
  сопровождается sha обоих refs — для дебага достаточно.

## 8. Связь с существующими механизмами

- **`merge-gate` `stale_branch_scan_all`** (ретро 12.08/14.08): проверяет уже
  влитую ветку → снимает `needs-review`, не ставит `needs-e2e`. **Другой**
  класс проблемы (ветка переиспользована).
- **`merge-gate` stale-rebase watchdog `t_562a8682`**: постит bot-коммент
  «stale-branch detected» в PR. **Другой** уровень — это в PR-time, не в
  e2e-dispatch-time. Дополняет, не дублирует.
- **Pre-round live-candidate guard `t_4212e8ad`**: считает кандидатов перед
  `round_ensure`. Наш stale-check встроен в эту же логику (после dedup).
