#!/bin/bash
# ============================================================================
# test_merge_gate_stale_after_upstream_fix.sh
#
# Ретро 31.08 t_9d375e3e / ADR-0035: merge-gate UNSTABLE-block создаёт
# diagnostic-карточки (PR #1743 / retro t_e00f448d), но они "вечно живые":
# после upstream-фикса в develop или фикса в самом PR — backend-воркер
# тратит max_runtime на чтение "уже-зелёных" тестов, либо рискует
# false-FIX (правит поверх уже-починенного).
#
# Реализация: helper-функция stale_after_upstream_fix_scan_all() +
# маркеры `<!-- diag-* -->` в body diagnostic-карточек. Scan вызывается
# рядом со stale_branch_scan_all / stale_conflicting_scan_all /
# duplicate_file_scan_all / pr_without_marker_scan_all /
# deploy_issue_reconcile_all (основной путь + no-issues путь).
#
# Scenarios (ADR-0035 §5.2):
#   D1.  diagnostic + upstream-фикс в develop по сигнатуре (strat B) →
#         auto-block + body содержит `### ✅ Upstream-фикс` + reason с sha
#   D2.  diagnostic + PR head SHA ancestor of origin/develop (strat A) →
#         auto-block, reason с PR SHA
#   D3.  diagnostic + PR CLOSED → auto-block, reason "PR CLOSED", sha пуст
#   D4.  diagnostic + фикс в самом PR + CI SUCCESS (strat C) → auto-block,
#         reason "фикс в самом PR", sha пуст
#   D5.  diagnostic без маркеров (legacy) → skip с логом "no diag-pr marker"
#   D6.  DRY_RUN=true → no real `hermes kanban block`, только лог DRY-RUN
#   D7.  Rate-limit: 2й тик подряд, тот же PR → skip "rate-limited"
#   D8.  diagnostic в `done` / `archived` → skip (терминальные статусы)
#   D9.  REPO_DIR пуст (REST compare fallback) → strategy A через
#         gh api compare, B+C skip с логом
#   D10. diag-sig + diag-tests оба пустые → только strategy A (PR-merge)
#   M1.  маркеры корректно записываются в body при создании
#         diagnostic-карточки (UNSTABLE-блок основного цикла).
#         ЗАГЛУШКА: реализация живёт в PR #1743 (ретро t_e00f448d),
#         который ещё не влит в develop. После merge PR #1743 — перенести
#         тест в test_merge_gate_unstable_diagnostic.sh. Сейчас test_M1
#         не запускается (см. run_test ниже).
#   B1.  backfill_diag_markers.sh: legacy diagnostic без маркеров →
#         маркеры добавляются (через HERMES body show/add)
#   B2.  backfill: diagnostic с маркерами → НЕ дублирует (idempotent)
#   B3.  backfill: DRY_RUN=true → файл body не изменяется
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_stale_after_upstream_fix.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# Resolve repo root so merge-gate picks up the fixture (mock_env.sh sets
# REPO_DIR fallback only if it sees the env var; tests don't need a real
# repo since `git merge-base` / `git log -S` are mocked below).
# REPO_ROOT (from mock_env.sh) points to scripts/agent_flow/; backfill
# script lives at the same level.
BACKFILL_SCRIPT="$TEST_LIB_DIR/../../backfill_diag_markers.sh"

# ============================================================================
# Fixture: diagnostic-карточка с маркерами `<!-- diag-* -->`.
#   $1=card_id $2=status $3=pr_num $4=pr_sha $5=pr_base $6=sig_csv
#   $7=tests_csv $8=classification $9=created_ts
# ============================================================================
fixture_diag_card() {
    local cid="$1" status="$2" pr_num="$3" pr_sha="$4" pr_base="$5"
    local sig_csv="$6" tests_csv="$7" classification="$8" created_ts="$9"

    # Build marker block (однострочно для совместимости с mock_env.sh,
    # который читает key=value построчно через grep -E ... | head -n1).
    # JSON body is single-line (raw newlines break json.loads).
    local markers
    markers="<!-- diag-pr: ${pr_num} --> <!-- diag-pr-sha: ${pr_sha} --> <!-- diag-pr-base: ${pr_base} --> <!-- diag-sig: ${sig_csv} --> <!-- diag-tests: ${tests_csv} --> <!-- diag-classification: ${classification} --> <!-- diag-created-ts: ${created_ts} -->"

    local body="## 🐛 CI UNSTABLE: real regression в PR ${markers} Body content for card ${cid}."

    # KANBAN_LIST_JSON: scan выбирает все non-terminal diagnostic-карточки.
    local card_json
    card_json="{\"id\":\"${cid}\",\"title\":\"🐛 CI UNSTABLE DIAGNOSTIC #${pr_num} — wts/branch\",\"status\":\"${status}\",\"body\":\"${body}\"}"
    set_state KANBAN_LIST_JSON "[${card_json}]"

    # KANBAN_SHOW_<cid>_JSON: scan достаёт body для парсинга маркеров.
    set_state "KANBAN_SHOW_${cid}_JSON" "{\"task\":{\"id\":\"${cid}\",\"status\":\"${status}\",\"body\":\"${body}\"}}"

    # PR_<pr>_VIEW_JSON: gh pr view N --json state
    set_state "PR_${pr_num}_VIEW_JSON" '{"state":"OPEN","headRefOid":"'"${pr_sha}"'","headRefName":"wts/branch","mergeable":"MERGEABLE","mergeStateStatus":"UNSTABLE"}'

    # gh pr checks N (statusCheckRollup): for strategy C (all SUCCESS).
    # Default = SUCCESS so D1/D2/D3 (where strat C isn't expected) still
    # don't trip on it. Tests that need strat C to fire also need
    # PR_<pr>_FILES_JSON containing all failing-test files.
    set_state "PR_${pr_num}_ROLLUP_JSON" '[{"name":"Unit Tests (ROS2 Humble)","conclusion":"SUCCESS","status":"COMPLETED"}]'

    # Empty open-PR list: needed so gh pr list --state open doesn't error
    # in stale_branch_scan_all (использует read -r, который падает на
    # пустом stdin без этой заглушки).
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_ALL_OPEN_REST_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
}

# ============================================================================
# Strategy-B fixture: simulate `git log origin/develop -S <attr>` hit.
# Sets STALE_DIAG_GIT_LOG_HIT_<attr>=<sha> for the mock to return.
# ============================================================================
fixture_git_log_hit_attr() {
    local attr="$1" sha="$2"
    # Mock uses simple key: ATTR_HIT_<sanitized_attr>=<sha>.
    set_state "STALE_DIAG_ATTR_HIT_${attr}" "$sha"
}

# ============================================================================
# Strategy-A fixture: simulate `git merge-base --is-ancestor <sha> origin/dev`.
# ============================================================================
fixture_git_merge_base_ancestor() {
    local sha="$1"
    set_state "STALE_DIAG_ANCESTOR_${sha}" "1"
}

# ============================================================================
# Strategy-C fixture: failing-test files listed in PR_<pr>_FILES_JSON.
# ============================================================================
fixture_pr_files() {
    local pr_num="$1" csv="$2"
    local files_json
    # Mock apply_jq regex "[.files[].path]" expects top-level dict with
    # field "files" (matches real `gh pr view --json files` output, not
    # raw REST array — see test_completion_check.sh fixtures).
    files_json="$(printf '%s' "$csv" | tr ',' '\n' | sed 's|.*|{"path":"&","filename":"&"}|' | paste -sd, -)"
    set_state "PR_${pr_num}_FILES_JSON" "{\"files\":[${files_json}]}"
}

# ============================================================================
# D1. diagnostic + upstream-fix in develop via signature (strat B).
# ============================================================================
test_D1_upstream_fix_via_signature() {
    new_test
    # REPO_DIR нужен для git log -S (стратегия B). Мок git (lib/mock_env.sh)
    # поддерживает merge-base/log только когда REPO_DIR экспортирован и
    # существует; см. ADR-0035 §2.2 стратегия B.
    REPO_DIR="$TEST_TMP"
    export REPO_DIR
    local cid="t_diag_d1" pr=1740 sha="f924ad6c47bcf7deb66d2080dcee067c66cf5792"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "_track_mode_music_active" \
        "src/rob_box_voice/test/unit/node/test_barge_in_policy.py" \
        "unit_lint" "1756598400"
    # Strategy B: git log -S _track_mode_music_active → hit at upstream sha.
    local upstream_sha="06b83b01b6a8c1de76c32bf90d809f0cfa809ffc"
    fixture_git_log_hit_attr "_track_mode_music_active" "$upstream_sha"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) `hermes kanban --board X block --kind transient t_diag_d1 ...` issued.
    # Pattern matches `hermes ... block ... t_diag_d1` (the --kind transient
    # flag sits between `block` and `<id>` in the actual kanban command).
    local blocks
    blocks="$(printf '%s\n' "$journal" | grep -cE "hermes .* block.* ${cid} " || true)"
    assert_eq "1" "$blocks" "kanban block called for diagnostic card" || return 1

    # 2) block reason содержит upstream SHA short и "stale-after-upstream-fix".
    local block_line
    block_line="$(printf '%s\n' "$journal" | grep -E "hermes .* block.* ${cid}" | head -1)"
    assert_contains "stale-after-upstream-fix" "$block_line" "reason contains stale-after-upstream-fix marker" || return 1
    assert_contains "${upstream_sha:0:8}" "$block_line" "reason contains upstream sha prefix" || return 1

    # 3) comment patch — содержит ссылку на upstream-коммит.
    # Note: the comment body spans multiple lines, so we use two separate
    # greps (ERE `.*` doesn't span newlines). Match `hermes ... comment <id>`
    # on one line and `Upstream-фикс` anywhere in the journal.
    local comment_lines up_lines
    comment_lines="$(printf '%s\n' "$journal" | grep -cE "hermes .* comment.* ${cid} " || true)"
    up_lines="$(printf '%s\n' "$journal" | grep -cE "Upstream-фикс" || true)"
    if [ "$comment_lines" -ge 1 ] && [ "$up_lines" -ge 1 ]; then
        patches=1
    else
        patches=0
    fi
    assert_eq "1" "$patches" "body patch comment with upstream fix" || return 1

    # 4) stderr содержит лог успеха.
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    assert_contains "auto-blocked" "$stderr_log" "stderr logs auto-block success" || return 1
}

# ============================================================================
# D2. diagnostic + PR head SHA ancestor of origin/develop (strat A).
# ============================================================================
test_D2_pr_sha_ancestor_of_origin_develop() {
    new_test
    REPO_DIR="$TEST_TMP"
    export REPO_DIR
    local cid="t_diag_d2" pr=1741 sha="aaaa1111aaaa1111aaaa1111aaaa1111aaaa1111"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "_code_speech_retry_used" "" "unit_lint" "1756598400"
    # No diag-tests, so strategy B tries attr → no hit.
    # No diag-tests → strategy C skipped. Strategy A wins.
    fixture_git_merge_base_ancestor "$sha"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local blocks
    blocks="$(printf '%s\n' "$journal" | grep -cE "hermes .* block.* ${cid} " || true)"
    assert_eq "1" "$blocks" "kanban block called (strat A)" || return 1

    local block_line
    block_line="$(printf '%s\n' "$journal" | grep -E "hermes .* block.* ${cid}" | head -1)"
    assert_contains "${sha:0:8}" "$block_line" "reason contains PR SHA prefix" || return 1
}

# ============================================================================
# D3. diagnostic + PR CLOSED → auto-block, reason "PR CLOSED", sha empty.
# ============================================================================
test_D3_pr_closed() {
    new_test
    local cid="t_diag_d3" pr=1742 sha="bbbb2222bbbb2222bbbb2222bbbb2222bbbb2222"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "" "" "unit_lint" "1756598400"
    # Override PR_<pr>_VIEW_JSON — state=CLOSED.
    set_state "PR_${pr}_VIEW_JSON" '{"state":"CLOSED","headRefOid":"'"${sha}"'","headRefName":"wts/branch","mergeable":"MERGEABLE","mergeStateStatus":"UNKNOWN"}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local blocks
    blocks="$(printf '%s\n' "$journal" | grep -cE "hermes .* block.* ${cid} " || true)"
    assert_eq "1" "$blocks" "kanban block called (PR CLOSED)" || return 1

    local block_line
    block_line="$(printf '%s\n' "$journal" | grep -E "hermes .* block.* ${cid}" | head -1)"
    assert_contains "PR #${pr} CLOSED" "$block_line" "reason mentions PR CLOSED" || return 1
}

# ============================================================================
# D4. diagnostic + fix in same PR + CI SUCCESS (strat C) → auto-block.
# Strategy C: failing-tests files all present in PR diff + all checks SUCCESS.
# ============================================================================
test_D4_fix_in_same_pr_via_strat_c() {
    new_test
    local cid="t_diag_d4" pr=1743 sha="cccc3333cccc3333cccc3333cccc3333cccc3333"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "_barge_in_attr" \
        "src/rob_box_voice/test/unit/node/test_barge_in_policy.py,src/rob_box_voice/test/unit/node/test_issue_1195_tg_source.py" \
        "unit_lint" "1756598400"
    # No git history hits (strat A/B fail).
    # But: failing-test files all in PR diff + CI SUCCESS → strat C fires.
    fixture_pr_files "$pr" \
        "src/rob_box_voice/test/unit/node/test_barge_in_policy.py,src/rob_box_voice/test/unit/node/test_issue_1195_tg_source.py,src/foo.py"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local blocks
    blocks="$(printf '%s\n' "$journal" | grep -cE "hermes .* block.* ${cid} " || true)"
    assert_eq "1" "$blocks" "kanban block called (strat C)" || return 1

    local block_line
    block_line="$(printf '%s\n' "$journal" | grep -E "hermes .* block.* ${cid}" | head -1)"
    assert_contains "фикс уже в самом PR" "$block_line" "reason mentions fix in same PR" || return 1
}

# ============================================================================
# D5. diagnostic без маркеров (legacy) → skip, не блокируется.
# ============================================================================
test_D5_legacy_no_markers() {
    new_test
    local cid="t_diag_d5" pr=1744
    # Build card WITHOUT `<!-- diag-pr: ... -->` markers.
    # Body is single-line (raw newlines break json.loads).
    local body="## 🐛 CI UNSTABLE: real regression в PR — Legacy body without markers."
    set_state KANBAN_LIST_JSON "[{\"id\":\"${cid}\",\"title\":\"🐛 CI UNSTABLE DIAGNOSTIC #${pr} — wts/branch\",\"status\":\"ready\",\"body\":\"${body}\"}]"
    set_state "KANBAN_SHOW_${cid}_JSON" "{\"task\":{\"id\":\"${cid}\",\"status\":\"ready\",\"body\":\"${body}\"}}"
    # Empty open-PR list: needed so gh pr list --state open doesn't error
    # in stale_branch_scan_all (см. fixture_diag_card).
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_ALL_OPEN_REST_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # No block for legacy card.
    local blocks
    blocks="$(printf '%s\n' "$journal" | grep -cE "hermes .* block.* ${cid} " || true)"
    assert_eq "0" "$blocks" "no block for legacy card (no markers)" || return 1

    # stderr logs "no diag-pr marker, skip (legacy)".
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    assert_contains "no diag-pr marker" "$stderr_log" "stderr logs legacy skip" || return 1
}

# ============================================================================
# D6. DRY_RUN=true → no real `hermes kanban block`, only DRY-RUN log.
# ============================================================================
test_D6_dry_run() {
    new_test
    REPO_DIR="$TEST_TMP"
    export REPO_DIR
    local cid="t_diag_d6" pr=1745 sha="dddd4444dddd4444dddd4444dddd4444dddd4444"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "_d6_attr" "" "unit_lint" "1756598400"
    fixture_git_merge_base_ancestor "$sha"

    DRY_RUN=true run_merge_gate
    local journal stderr_log
    journal="$(cat "$GH_JOURNAL")"
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"

    # No real block issued (block is silent no-op in mock anyway, but the
    # marker line should be DRY-RUN, not actual block).
    # ADR-0035: detector logs DRY-RUN через log() → stderr (как D1/D2),
    # а не в journal. Проверяем stderr.
    local dry_run_marks
    dry_run_marks="$(printf '%s\n' "$stderr_log" | grep -cE "DRY-RUN would: block ${cid}" || true)"
    # ADR-0035: stderr содержит set -x trace + реальную log-строку → 3+ матча.
    # Проверяем ">= 1" чтобы быть устойчивым к set -x в скрипте.
    if [ "$dry_run_marks" -ge 1 ]; then
        :
    else
        printf '  %sassert fail:%s %s\n    expected: >=1, got: %s\n' \
            "$RED" "$END" "DRY-RUN log present (stderr)" "$dry_run_marks" >&2
        return 1
    fi

    # И никакого реального block в journal тоже нет.
    local blocks
    blocks="$(printf '%s\n' "$journal" | grep -cE "hermes .* block.* ${cid} " || true)"
    assert_eq "0" "$blocks" "no real block issued under DRY_RUN" || return 1
}

# ============================================================================
# D7. Rate-limit: 2й тик подряд, тот же PR → skip "rate-limited".
#
# ADR-0035 / task t_d83c9430: rate-limit переехал с DB-комментариев на
# state-файл $STATE_DIR/auto-block-rate.json. State-файл SOT, DB-fallback
# только для backward compatibility со старыми тиками (не тестируется
# здесь — это regression для unit-теста R2, см. test_merge_gate_auto_block_rate_limit.sh).
#
# Сценарий: pre-seed state-файла с last_block_ts = NOW - 60s для карточки
# `t_diag_d7` → 2-й тик scan_all видит свежую запись → skip + warn.
# ============================================================================
test_D7_rate_limit() {
    new_test
    local cid="t_diag_d7" pr=1746 sha="eeee5555eeee5555eeee5555eeee5555eeee5555"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "_d7_attr" "" "unit_lint" "1756598400"
    fixture_git_merge_base_ancestor "$sha"
    # ADR-0035 / t_d83c9430: state-файл изолирован в $TEST_TMP/stale-auto-block-state/
    # (run_merge_gate делает это автоматически). Pre-seed last_block_ts = NOW - 60s
    # (внутри 14400s cooldown). Записываем JSON в state-файл напрямую.
    local state_file="$TEST_TMP/stale-auto-block-state/auto-block-rate.json"
    mkdir -p "$(dirname "$state_file")"
    local now_ts recent_ts
    now_ts="$(date +%s)"
    recent_ts=$((now_ts - 60))
    printf '{"%s":%d}' "$cid" "$recent_ts" > "$state_file"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 0 новых block-команд (state заблокировал).
    local new_blocks
    new_blocks="$(printf '%s\n' "$journal" | grep -cE "hermes .* block.* ${cid} " || true)"
    assert_eq "0" "$new_blocks" "no new block (rate-limited via state-file)" || return 1

    # stderr содержит rate-limit лог.
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    assert_contains "rate-limited" "$stderr_log" "stderr logs rate-limit skip" || return 1
}

# ============================================================================
# D8. diagnostic в done → skip (терминальные статусы).
# ============================================================================
test_D8_done_status_skipped() {
    new_test
    local cid="t_diag_d8" pr=1747 sha="ffff6666ffff6666ffff6666ffff6666ffff6666"
    fixture_diag_card "$cid" "done" "$pr" "$sha" "develop" \
        "_d8_attr" "" "unit_lint" "1756598400"
    fixture_git_merge_base_ancestor "$sha"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local blocks
    blocks="$(printf '%s\n' "$journal" | grep -cE "hermes .* block.* ${cid} " || true)"
    assert_eq "0" "$blocks" "no block for done card" || return 1
}

# ============================================================================
# D9. REPO_DIR пуст (REST mergedAt fallback) → strategy A через gh pr view.
# Мы не можем очистить REPO_DIR внутри мока merge-gate (он внутри скрипта),
# поэтому моделируем: PR head SHA НЕ ancestor (стратегия A по git упадёт),
# но `gh pr view N --json mergedAt` вернёт timestamp (REST fallback A).
# ============================================================================
test_D9_rest_compare_fallback() {
    new_test
    local cid="t_diag_d9" pr=1748 sha="aaaa7777aaaa7777aaaa7777aaaa7777aaaa7777"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "" "" "unit_lint" "1756598400"
    # NO git history hits. REST fallback via PR_1748_MERGEDAT_JSON → strategy A.
    set_state "PR_${pr}_MERGEDAT_JSON" "2026-08-15T12:34:56Z"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local blocks
    blocks="$(printf '%s\n' "$journal" | grep -cE "hermes .* block.* ${cid} " || true)"
    assert_eq "1" "$blocks" "kanban block called (REST fallback)" || return 1

    # gh pr view --json mergedAt was called. Note: merge-gate calls
    # `gh pr view N --repo X --json mergedAt --jq '...'` — after `N`
    # there's "--repo X" before "--json", so regex uses `.*` between
    # `N` and `--json` to cover both forms (with or without --repo).
    # NB: `.* --json` does NOT match `N --json` (no second space), so
    # we use `.*--json` (no leading space before --json).
    local mergedat_calls
    mergedat_calls="$(printf '%s\n' "$journal" | grep -cE "gh pr view ${pr} .*--json mergedAt" || true)"
    assert_contains "1" "$mergedat_calls" "gh pr view --json mergedAt was called" || return 1

    # stderr содержит "REST fallback" — лог strat A через gh pr view mergedAt.
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    assert_contains "REST fallback" "$stderr_log" "stderr mentions REST mergedAt fallback" || return 1
}

# ============================================================================
# D10. diag-sig + diag-tests оба пустые → только strategy A (PR-merge).
# ============================================================================
test_D10_no_sig_no_tests_only_strat_a() {
    new_test
    local cid="t_diag_d10" pr=1749 sha="aaaa8888aaaa8888aaaa8888aaaa8888aaaa8888"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "" "" "unit_lint" "1756598400"
    fixture_git_merge_base_ancestor "$sha"
    # mergedAt пусто — REST fallback не должен сработать (strat A по git уже
    # сработал). Фикстура COMPARE_* больше не нужна (REST fallback переехал
    # на gh pr view --json mergedAt).

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local blocks
    blocks="$(printf '%s\n' "$journal" | grep -cE "hermes .* block.* ${cid} " || true)"
    assert_eq "1" "$blocks" "block via strategy A (no sig/tests)" || return 1

    # Verify that strategy B was attempted (git log -S) — should be no-op since
    # sig list is empty. We assert this implicitly: no crash.
}

# ============================================================================
# M1. NOT TESTED IN THIS PR.
#   Маркеры `<!-- diag-* -->` пишутся в body diagnostic-карточки в
#   UNSTABLE-блоке основного цикла. Этот блок живёт в PR #1743 (retro
#   t_e00f448d, классификация diagnostic vs rebase). PR #1743 ещё не влит
#   в develop на момент написания этой реализации (Этап 2 ADR-0035, см.
#   §5.4 «Rollout: маркеры → detector → backfill»). Поэтому маркеры
#   появятся в карточках сразу после merge PR #1743, и тогда M1 станет
#   реальным тестом (можно будет добавить в test_merge_gate_unstable_*.sh,
#   который живёт в PR #1743).
# ============================================================================

# ============================================================================
# B1. backfill_diag_markers.sh: legacy diagnostic без маркеров → маркеры
# добавляются. Backfill вызывает hermes kanban show + comment; mock hermes
# должен подтвердить, что comment был вызван.
# ============================================================================
test_B1_backfill_adds_markers() {
    new_test
    # Existing legacy card without markers. Body is single-line
    # (raw newlines break json.loads).
    local cid="t_legacy_diag" pr=1750 sha="ffff9999ffff9999ffff9999ffff9999ffff9999"
    local body="## 🐛 CI UNSTABLE: real regression — Legacy body without markers."
    set_state KANBAN_LIST_JSON "[{\"id\":\"${cid}\",\"title\":\"🐛 CI UNSTABLE DIAGNOSTIC #${pr} — branch\",\"status\":\"ready\",\"body\":\"${body}\"}]"
    set_state "KANBAN_SHOW_${cid}_JSON" "{\"task\":{\"id\":\"${cid}\",\"status\":\"ready\",\"body\":\"${body}\"}}"
    set_state "PR_${pr}_VIEW_JSON" '{"state":"OPEN","headRefOid":"'"${sha}"'","headRefName":"branch","mergeable":"MERGEABLE","mergeStateStatus":"UNSTABLE"}'

    # Run backfill script with mock PATH.
    (
        export PATH="$TEST_TMP/bin:$PATH"
        export GH_STATE="$GH_STATE"
        export GH_JOURNAL="$GH_JOURNAL"
        bash "$BACKFILL_SCRIPT" 2>>"$TEST_TMP/stderr.log"
    )

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # backfill should `comment` to add the markers.
    # ADR-0035: comment body многострочный (markers на отдельной строке от
    # "hermes ... comment ..."), так что grep по одной строке не работает.
    # Проверяем двумя шагами: (1) comment line есть в journal, (2) markers
    # в любой строке journal.
    local comment_lines marker_lines
    comment_lines="$(printf '%s\n' "$journal" | grep -cE "hermes .* comment.* ${cid} " || true)"
    marker_lines="$(printf '%s\n' "$journal" | grep -cE "diag-pr: ${pr}" || true)"
    assert_contains "1" "$comment_lines" "backfill posted comment for legacy card" || return 1
    assert_contains "1" "$marker_lines" "backfill posted comment with diag-pr marker" || return 1
}

# ============================================================================
# B2. backfill: diagnostic с маркерами → НЕ дублирует (idempotent).
# ============================================================================
test_B2_backfill_idempotent() {
    new_test
    local cid="t_diag_with_markers" pr=1751 sha="aaaabbbbccccddddeeeeffffaaaabbbb"
    # Markers on a single line to keep the JSON fixture valid
    # (raw newlines inside JSON strings break json.loads).
    local markers="<!-- diag-pr: ${pr} --> <!-- diag-pr-sha: ${sha} --> <!-- diag-pr-base: develop --> <!-- diag-sig:  --> <!-- diag-tests:  --> <!-- diag-classification: unit_lint --> <!-- diag-created-ts: 1756598400 -->"
    local body="## 🐛 CI UNSTABLE ${markers} Body."
    set_state KANBAN_LIST_JSON "[{\"id\":\"${cid}\",\"title\":\"🐛 CI UNSTABLE DIAGNOSTIC #${pr} — branch\",\"status\":\"ready\",\"body\":\"${body}\"}]"
    set_state "KANBAN_SHOW_${cid}_JSON" "{\"task\":{\"id\":\"${cid}\",\"status\":\"ready\",\"body\":\"${body}\"}}"
    set_state "PR_${pr}_VIEW_JSON" '{"state":"OPEN","headRefOid":"'"${sha}"'","headRefName":"branch","mergeable":"MERGEABLE","mergeStateStatus":"UNSTABLE"}'

    (
        export PATH="$TEST_TMP/bin:$PATH"
        export GH_STATE="$GH_STATE"
        export GH_JOURNAL="$GH_JOURNAL"
        bash "$BACKFILL_SCRIPT" 2>>"$TEST_TMP/stderr.log"
    )

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # No comment posted (idempotent skip).
    local cmts
    cmts="$(printf '%s\n' "$journal" | grep -cE "hermes .* comment ${cid}" || true)"
    assert_eq "0" "$cmts" "no comment for already-marked card" || return 1
}

# ============================================================================
# B3. backfill: DRY_RUN=true → comment не вызывается.
# ============================================================================
test_B3_backfill_dry_run() {
    new_test
    local cid="t_legacy_for_dry" pr=1752 sha="ddddeeeeffffaaaabbbbccccddddeeee"
    local body="## 🐛 CI UNSTABLE — Legacy body without markers (single-line for JSON)."
    set_state KANBAN_LIST_JSON "[{\"id\":\"${cid}\",\"title\":\"🐛 CI UNSTABLE DIAGNOSTIC #${pr} — branch\",\"status\":\"ready\",\"body\":\"${body}\"}]"
    set_state "KANBAN_SHOW_${cid}_JSON" "{\"task\":{\"id\":\"${cid}\",\"status\":\"ready\",\"body\":\"${body}\"}}"
    set_state "PR_${pr}_VIEW_JSON" '{"state":"OPEN","headRefOid":"'"${sha}"'","headRefName":"branch","mergeable":"MERGEABLE","mergeStateStatus":"UNSTABLE"}'

    (
        export PATH="$TEST_TMP/bin:$PATH"
        export GH_STATE="$GH_STATE"
        export GH_JOURNAL="$GH_JOURNAL"
        export DRY_RUN=true
        bash "$BACKFILL_SCRIPT" 2>>"$TEST_TMP/stderr.log"
    )

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local cmts
    cmts="$(printf '%s\n' "$journal" | grep -cE "hermes .* comment ${cid}" || true)"
    assert_eq "0" "$cmts" "DRY_RUN: no comment posted" || return 1

    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    assert_contains "DRY-RUN" "$stderr_log" "DRY-RUN log present" || return 1
}

# ============================================================================
# Run
# ============================================================================
run_test "D1.  upstream-fix via signature (strat B)"          test_D1_upstream_fix_via_signature
run_test "D2.  PR SHA ancestor of origin/develop (strat A)"   test_D2_pr_sha_ancestor_of_origin_develop
run_test "D3.  PR CLOSED"                                     test_D3_pr_closed
run_test "D4.  fix in same PR + CI SUCCESS (strat C)"         test_D4_fix_in_same_pr_via_strat_c
run_test "D5.  legacy no markers → skip"                      test_D5_legacy_no_markers
run_test "D6.  DRY_RUN=true → no real block"                    test_D6_dry_run
run_test "D7.  rate-limit (2nd tick same PR)"                test_D7_rate_limit
run_test "D8.  done status → skip"                           test_D8_done_status_skipped
run_test "D9.  REST compare fallback (REPO_DIR empty)"       test_D9_rest_compare_fallback
run_test "D10. no sig + no tests → strat A only"             test_D10_no_sig_no_tests_only_strat_a
# M1: covered in PR #1743 (UNSTABLE diagnostic-klassifikator), не в этом PR.
# После merge PR #1743 можно будет перенести test_M1_markers_in_diag_body
# в test_merge_gate_unstable_diagnostic.sh (как ADR-0035 §5.4 Этап 1).
run_test "B1.  backfill adds markers to legacy"              test_B1_backfill_adds_markers
run_test "B2.  backfill idempotent"                          test_B2_backfill_idempotent
run_test "B3.  backfill DRY_RUN"                             test_B3_backfill_dry_run

summary