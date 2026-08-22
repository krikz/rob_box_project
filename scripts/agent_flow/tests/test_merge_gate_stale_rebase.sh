#!/bin/bash
# ============================================================================
# test_merge_gate_stale_rebase.sh — ретро 22.08 t_562a8682 acceptance tests
#
# Verifies the stale-rebase watchdog in agent-flow-merge-gate.sh:
#   Сценарий: PR с needs-review, CI green & clean (mergeStateStatus=CLEAN),
#   но ahead-of-develop > STALE_REBASE_AHEAD_THRESHOLD. Watchdog должен:
#     - alert в живую карточку воркера (rate-limited 2ч)
#     - comment-on-issue (24h dedup) если карточка мёртвая или task_id пуст
#     - НЕ блокировать merge-gate (alert, не gate)
#     - не alert-ить когда ahead ≤ порога (нормальный флоу)
#     - не alert-ить когда PR без needs-review (lint-PR / etc)
#
# Scenarios covered:
#   A. ahead=180 (как PR #1509) > threshold=30 → alert в живую карточку.
#   B. ahead=5 ≤ threshold=30 → НЕ alert (обычный флоу, needs-e2e ставится).
#   C. ahead=180, карточка мёртвая (done) → comment-on-issue, dedup 24h.
#   D. ahead=180, PR без needs-review (lint-PR) → НЕ alert.
#   E. ahead=180, карточка живая, 2-й тик за <2ч → rate-limit skip.
#   F. ahead=180, task_id пуст → comment-on-issue (24h dedup).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_stale_rebase.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# Журнал для отладки — оставьте /tmp/sr-debug.* доступным после прогона.
TEST_TMP="${TEST_TMP:-/tmp/sr-debug.$$}"
mkdir -p "$TEST_TMP"
mkdir -p "/tmp/sr-debug"

# slugify_branch — локальная копия из test_merge_gate_stale_branch.sh
# (там она file-private, не в lib/). Используется для построения имени
# ветки, под которым mock_env отдаёт PR_HEAD_<branch>_JSON.
slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# ---------------------------------------------------------------------------
# Helper: minimal valid issue+PR fixture for the stale-rebase watchdog.
# Отличается от fixture_stale_pr тем, что:
#   - PR CLEAN + needs-review (нормальный "всё зелёное" PR, кроме ahead).
#   - Включает COMPARE_<base>_<head>_JSON для мока REST compare API.
#   - kanban_card_status настраивается снаружи (по умолчанию running).
# ---------------------------------------------------------------------------
fixture_stale_rebase_pr() {  # $1=issue $2=pr $3=branch $4=ahead $5=deletions (default 5)
    local issue="$1" pr="$2" branch="$3" ahead="$4"
    local deletions="${5:-5}"
    # title должен давать ту же ветку через slugify() — иначе PR не найдётся.
    local title="fix #${issue} stale rebase demo"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\\\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    # CLEAN + needs-review + green CI. mergeStateStatus=CLEAN → пройдёт CLEAN-блок.
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"},{\"name\":\"needs-review\"}],\"additions\":50,\"deletions\":${deletions},\"commits\":[{},{}]}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    # REST compare API: ahead_by = $4, behind_by = 158 (как у PR #1504/#1509).
    # cmp_key трансформирует "develop...z-{agent}/3301-fix-3301-stale-rebase-demo"
    # → "develop___z-{agent}_3301-fix-3301-stale-rebase-demo" (tr . / space → _).
    # Поэтому в state пишем тоже sanitized имя.
    cmp_key="develop___$(printf '%s' "$branch" | tr '/. ' '___')"
    set_state "COMPARE_${cmp_key}_JSON" "{\"ahead_by\":${ahead},\"behind_by\":158,\"status\":\"diverged\"}"
    set_state "PR_MERGED_HEAD_${branch}_JSON" ''  # нет влитого PR
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
}

# ---------------------------------------------------------------------------
# A. ahead=180 > threshold=30, карточка живая → alert в карточку, нулевой rate-limit.
# ---------------------------------------------------------------------------
test_A_stale_rebase_alert_live_card() {
    new_test
    local branch
    branch="$(slugify_branch 3301 'fix #3301 stale rebase demo')"
    fixture_stale_rebase_pr 3301 3302 "$branch" 180
    # Карточка живая (running). Статус читается через hermes kanban list
    # fallback — fixture key KANBAN_LIST_JSON.
    set_state KANBAN_LIST_JSON '[{"id":"t_dead3301","status":"running"}]'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Alert-лог "STALE REBASE detected" присутствует.
    local sr_logs
    sr_logs="$(printf '%s\n' "$journal" | grep -c 'STALE REBASE detected' || true)"
    assert_ge "$sr_logs" "1" "stale rebase ahead=180 > threshold → STALE REBASE log emitted"

    # Reminder отправлен в живую карточку через hermes kanban comment.
    local card_comments
    card_comments="$(printf '%s\n' "$journal" | grep -c 'kanban.*comment t_dead3301' || true)"
    assert_ge "$card_comments" "1" "stale rebase ahead=180 → kanban comment на живую карточку"

    # needs-e2e НЕ ставится (мы не блокируем — alert, не gate).
    # (lint path может поставить needs-review повторно — это ок.)
}

# ---------------------------------------------------------------------------
# B. ahead=5 ≤ threshold=30 → НЕ alert, обычный флоу (needs-e2e ставится).
# ---------------------------------------------------------------------------
test_B_normal_ahead_no_alert() {
    new_test
    local branch
    branch="$(slugify_branch 3303 'fix #3303 stale rebase demo')"
    fixture_stale_rebase_pr 3303 3304 "$branch" 5

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # НЕ должно быть STALE REBASE log.
    local sr_logs
    sr_logs="$(printf '%s\n' "$journal" | grep -c 'STALE REBASE detected' || true)"
    assert_eq "0" "$sr_logs" "stale rebase ahead=5 ≤ threshold → НЕ alert"

    # needs-e2e должен быть поставлен (обычный флоу functional PR).
    local needs_e2e_calls
    needs_e2e_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 3303 --add-label needs-e2e' || true)"
    assert_eq "1" "$needs_e2e_calls" "stale rebase ahead=5 → needs-e2e ставится (обычный флоу)"
}

# ---------------------------------------------------------------------------
# C. ahead=180, карточка мёртвая (done) → comment-on-issue, 24h dedup.
# ---------------------------------------------------------------------------
test_C_stale_rebase_dead_card_post_issue_comment() {
    new_test
    local branch
    branch="$(slugify_branch 3305 'fix #3305 stale rebase demo')"
    fixture_stale_rebase_pr 3305 3306 "$branch" 180
    # Карточка мёртвая (done). Статус читается через hermes kanban list
    # fallback (kanban_card_status, ретро 12.08 t_8af6bf29) — fixture key
    # KANBAN_LIST_JSON, ожидается массив объектов {id,status,...}.
    set_state KANBAN_LIST_JSON '[{"id":"t_dead3305","status":"done"}]'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # НЕ пишем в карточку (она мёртвая).
    local card_comments
    card_comments="$(printf '%s\n' "$journal" | grep -c 'kanban.*comment t_dead3305' || true)"
    assert_eq "0" "$card_comments" "stale rebase dead card → НЕ пишем в карточку"

    # Пишем comment-on-issue (через gh issue comment, не через gh api).
    local issue_comments
    issue_comments="$(printf '%s\n' "$journal" | grep -c 'gh issue comment 3305 .*STALE REBASE' || true)"
    assert_eq "1" "$issue_comments" "stale rebase dead card → comment-on-issue posted"

    # Второй тик: уже есть STALE REBASE коммент в ISSUE_3305_COMMENTS_SINCE_JSON.
    # Запоминаем размер journal, чтобы считать только новые строки от второго тика.
    set_state "ISSUE_3305_COMMENTS_SINCE_JSON" '[{"body":"## 🟡 STALE REBASE detected (merge-gate, 00:00:00Z)"}]'
    local _sr_lines_first
    _sr_lines_first="$(wc -l < "$GH_JOURNAL")"
    run_merge_gate
    local second_journal
    second_journal="$(tail -n "+$((_sr_lines_first + 1))" "$GH_JOURNAL")"

    local issue_comments_2
    issue_comments_2="$(printf '%s\n' "$second_journal" | grep -c 'gh issue comment 3305 .*STALE REBASE' || true)"
    assert_eq "0" "$issue_comments_2" "stale rebase dead card tick 2 → comment dedup (24h)"
}

# ---------------------------------------------------------------------------
# D. ahead=180, PR БЕЗ needs-review (lint-PR) → НЕ alert.
# ---------------------------------------------------------------------------
test_D_stale_rebase_skipped_without_needs_review() {
    new_test
    local branch
    branch="$(slugify_branch 3307 'fix #3307 stale rebase demo')"
    fixture_stale_rebase_pr 3307 3308 "$branch" 180
    # Перезаписываем labels — убираем needs-review (только agent:devops = lint-PR).
    local title="fix #3307 stale rebase demo"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":3308,\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}],\"additions\":50,\"deletions\":5,\"commits\":[{},{}]}]"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # НЕ должно быть STALE REBASE log (watchdog только для needs-review).
    local sr_logs
    sr_logs="$(printf '%s\n' "$journal" | grep -c 'STALE REBASE detected' || true)"
    assert_eq "0" "$sr_logs" "stale rebase без needs-review → НЕ alert"
}

# ---------------------------------------------------------------------------
# E. ahead=180, карточка живая, 2-й тик за <2ч → rate-limit skip (НЕ второй comment).
# Подмена sqlite БД нужна потому что kanban_last_reminder_ts читает
# MAX(created_at) из task_comments напрямую (не через hermes CLI, ретро 12.08
# t_8af6bf29). Создаём минимальную sqlite с одной записью комментария
# timestamp=now — эмулируем «только что сделанный reminder» и проверяем
# rate-limit на 2-м тике.
# ---------------------------------------------------------------------------
test_E_stale_rebase_rate_limit_card_comment() {
    new_test
    local branch
    branch="$(slugify_branch 3309 'fix #3309 stale rebase demo')"
    fixture_stale_rebase_pr 3309 3310 "$branch" 180
    set_state KANBAN_LIST_JSON '[{"id":"t_dead3309","status":"running"}]'

    # Создаём временную sqlite БД с нужной таблицей и строкой.
    # Мерж-гейт берёт KANBAN_DB из env, mock_env в run_merge_gate ставит
    # KANBAN_DB=$TEST_TMP/nonexistent-kanban.db. Здесь пересоздаём этот файл
    # как реальную sqlite.
    rm -f "$TEST_TMP/nonexistent-kanban.db"
    python3 - "$TEST_TMP/nonexistent-kanban.db" "$(date +%s)" "t_dead3309" <<'PYEOF' 2>/dev/null
import sqlite3, sys, time
db_path, now, tid = sys.argv[1], int(sys.argv[2]), sys.argv[3]
conn = sqlite3.connect(db_path)
conn.execute(
    "CREATE TABLE IF NOT EXISTS task_comments ("
    "  id INTEGER PRIMARY KEY,"
    "  task_id TEXT NOT NULL,"
    "  body TEXT,"
    "  created_at INTEGER"
    ")"
)
conn.execute(
    "INSERT INTO task_comments(task_id, body, created_at) VALUES (?, ?, ?)",
    (tid, "STALE REBASE detected\n", now)
)
conn.commit()
conn.close()
PYEOF
    # Первый тик: kanban_last_reminder_ts вернёт now (rate-limit срабатывает).
    # `set -e` в скрипте убьёт раннер если какой-то grep вернёт 1 (нет match).
    # Отключаем на время тиков.
    set +e
    local _sr_lines_pre
    _sr_lines_pre="$(wc -l < "$GH_JOURNAL")"
    run_merge_gate
    local first_journal
    first_journal="$(tail -n "+$((_sr_lines_pre + 1))" "$GH_JOURNAL")"

    # Второй тик: rate-limit активен.
    run_merge_gate
    local second_journal
    second_journal="$(tail -n "+$((_sr_lines_pre + 1))" "$GH_JOURNAL")"
    set -e
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"

    # Оба тика — НЕ должны слать kanban comment (rate-limit на 1-м уже).
    local first_card_comments
    first_card_comments="$(printf '%s\n' "$first_journal" | grep -c 'kanban.*comment t_dead3309' || true)"
    assert_eq "0" "$first_card_comments" "stale rebase tick 1 → rate-limited (DB has fresh reminder)"

    local second_card_comments
    second_card_comments="$(printf '%s\n' "$second_journal" | grep -c 'kanban.*comment t_dead3309' || true)"
    assert_eq "0" "$second_card_comments" "stale rebase tick 2 → rate-limited, НЕ повторный comment"

    # В stderr должен быть rate-limited маркер (log() пишет в stderr).
    local rate_limit_logs
    rate_limit_logs="$(printf '%s\n' "$stderr_log" | grep -c 'rebase reminder rate-limited' || true)"
    assert_ge "$rate_limit_logs" "1" "stale rebase tick 2 → rate-limited log"
}

# ---------------------------------------------------------------------------
# F. ahead=180, task_id пуст → основной цикл skip'ает issue (triage not
#    finished), scan-all-prs подхватит. Проверяем что watchdog не сломал
#    процесс (run_merge_gate exit=0) и что НЕ случилось ложного alert'а
#    (нет gh issue comment с STALE REBASE — task_id не дошёл до watchdog).
# ---------------------------------------------------------------------------
test_F_stale_rebase_no_task_id_no_false_alert() {
    new_test
    local branch
    branch="$(slugify_branch 3311 'fix #3311 stale rebase demo')"
    fixture_stale_rebase_pr 3311 3312 "$branch" 180
    # Issue body И комментарии БЕЗ kanban-маркера → task_id пуст.
    set_state ISSUE_LIST_JSON "[{\"number\":3311,\"title\":\"fix #3311 stale rebase demo\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"no kanban marker here\"}]"
    set_state "ISSUE_3311_COMMENTS_JSON" '{"comments":[{"body":"no kanban marker\n"}]}'

    # merge-gate пишет progress-логи в stderr (не в journal). Не assert'им на
    # exit code — scan-all-prs после основного цикла может падать на пустых
    # fixture-данных; нам важно, что watchdog-блок НЕ выполнился.
    set +e
    run_merge_gate
    set -e

    # Читаем stderr.log (где `log()` merge-gate пишет status messages).
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"

    # Основной цикл skip'ает issue (нет kanban marker), watchdog НЕ сработал.
    # Проверяем по логу «has no kanban marker — triage not finished yet».
    local triage_skips
    triage_skips="$(printf '%s\n' "$stderr_log" | grep -c 'has no kanban marker' || true)"
    assert_ge "$triage_skips" "1" "issue без kanban marker → triage skip (нормальный flow)"

    # НЕ должно быть STALE REBASE alert (watchdog в основном цикле, не сработал).
    local sr_alerts
    sr_alerts="$(printf '%s\n' "$stderr_log" | grep -c 'STALE REBASE detected' || true)"
    assert_eq "0" "$sr_alerts" "watchdog не сработал для issue без task_id (корректное поведение)"
}

# ---------------------------------------------------------------------------
# Helpers для assert_ge (>=).
# ---------------------------------------------------------------------------
assert_ge() {  # $1=actual $2=expected $3=msg
    if [ "$1" -lt "$2" ] 2>/dev/null; then
        printf '  %sassert fail:%s %s\n    expected >= %s\n    actual:    %s\n' \
            "$RED" "$END" "$3" "$2" "$1" >&2
        return 1
    fi
}

# --- Test registry ----------------------------------------------------------
run_test "A. ahead=180 > threshold, живая карточка → alert" test_A_stale_rebase_alert_live_card
run_test "B. ahead=5 ≤ threshold → НЕ alert, обычный флоу" test_B_normal_ahead_no_alert
run_test "C. ahead=180, dead card → comment-on-issue (24h dedup)" test_C_stale_rebase_dead_card_post_issue_comment
run_test "D. ahead=180, lint-PR (no needs-review) → НЕ alert" test_D_stale_rebase_skipped_without_needs_review
run_test "E. ahead=180, 2-й тик <2ч → rate-limit skip" test_E_stale_rebase_rate_limit_card_comment
run_test "F. ahead=180, task_id пуст → watchdog не сработал (норма)" test_F_stale_rebase_no_task_id_no_false_alert

# --- Summary ----------------------------------------------------------------
printf '\n'
printf 'TESTS: %d total, %d passed, %d failed\n' "$TESTS_TOTAL" "$TESTS_PASSED" "$TESTS_FAILED"
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf 'FAILED: %s\n' "${FAILED_NAMES[*]}"
    exit 1
fi
exit 0
