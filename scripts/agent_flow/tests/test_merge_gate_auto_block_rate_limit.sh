#!/bin/bash
# ============================================================================
# test_merge_gate_auto_block_rate_limit.sh
#
# Ретро 31.08 t_9d375e3e / ADR-0035 / task t_d83c9430: ветка auto-block в
# merge-gate должна:
#   1. Rate-limit через state-файл $STATE_DIR/auto-block-rate.json
#      с last_block_ts по card_id (НЕ через DB-комментарии, как в D7).
#   2. Cooldown 14400s (4ч) — НЕ 7200s (2ч) как в D7.
#   3. Reason формата `stale-after-upstream-fix: <sha> <commit_message_short>` —
#      добавить короткий commit message (первая строка `git log -1 --pretty=%s`).
#   4. Body patch содержит:
#      - URL upstream-коммита (https://github.com/<GH_REPO>/commit/<sha>);
#      - diff `git log --oneline <create_time>..origin/develop | grep <sig>`
#        (только при strat B / C — там есть sig или tests).
#
# Сценарии:
#   R1. Первый auto-block → state-файл создан с card_id → epoch.
#   R2. Второй auto-block в течение < 14400s → skip + warn.
#   R3. Второй auto-block после 14400s+ → разрешён (state перезаписан).
#   R4. reason содержит `<sha> <commit_message_short>` (где short — заголовок
#       upstream-коммита из `git log -1 <sha> --pretty=%s`).
#   R5. body comment содержит URL upstream + diff из `git log --oneline
#       <create_time>..origin/develop | grep <sig>` (для strat B).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_auto_block_rate_limit.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ============================================================================
# Fixture: короткий subject upstream-коммита (для R4).
# Mock для `git log -1 <sha> --pretty=%s` будет читать state-ключ
# STALE_DIAG_COMMIT_SUBJECT_<sha>=<subject> и подставлять в reason.
# ============================================================================
fixture_commit_subject() {
    local sha="$1" subject="$2"
    set_state "STALE_DIAG_COMMIT_SUBJECT_${sha}" "$subject"
}

# ============================================================================
# R1. Первый auto-block → state-файл создан с card_id → epoch.
# ============================================================================
test_R1_state_file_created_on_first_block() {
    new_test
    REPO_DIR="$TEST_TMP"
    export REPO_DIR
    # Изолируем state-файл от production-каталога, чтобы не повредить
    # реальный rate-limit (tests не должны вмешиваться в cron state).
    export STALE_AUTO_BLOCK_STATE_DIR="$TEST_TMP/auto-block-state"
    mkdir -p "$STALE_AUTO_BLOCK_STATE_DIR"

    local cid="t_r1" pr=1901 sha="aaaa1111aaaa1111aaaa1111aaaa1111aaaa1111"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "_r1_attr" "" "unit_lint" "1756598400"
    fixture_git_merge_base_ancestor "$sha"
    fixture_commit_subject "$sha" "fix(voice): track_mode_music_active race"

    run_merge_gate

    # state-файл создан, содержит JSON-объект с card_id → epoch.
    local state_file="$STALE_AUTO_BLOCK_STATE_DIR/auto-block-rate.json"
    if [ ! -f "$state_file" ]; then
        printf '  %sassert fail:%s state-файл %s не создан после первого block\n' \
            "$RED" "$END" "$state_file" >&2
        return 1
    fi
    # JSON содержит ключ карточки и числовой epoch.
    local val
    val="$(python3 -c "
import json
try:
    d = json.load(open('$state_file'))
    v = d.get('$cid')
    print('OK' if isinstance(v, int) and v > 0 else 'MISSING')
except Exception as e:
    print('PARSE_ERR: ' + str(e))
")"
    assert_eq "OK" "$val" "state-файл содержит $cid → epoch" || return 1

    # Журнал содержит block-команду.
    local blocks
    blocks="$(grep -cE "hermes .* block.* ${cid} " "$GH_JOURNAL" || true)"
    assert_eq "1" "$blocks" "kanban block вызван" || return 1
}

# ============================================================================
# R2. Второй auto-block в течение < 14400s → skip + warn.
# Используем тот же fixture, но state-файл предзаполнен недавним ts.
# ============================================================================
test_R2_rate_limit_skips_within_4h() {
    new_test
    REPO_DIR="$TEST_TMP"
    export REPO_DIR
    export STALE_AUTO_BLOCK_STATE_DIR="$TEST_TMP/auto-block-state"
    mkdir -p "$STALE_AUTO_BLOCK_STATE_DIR"

    local cid="t_r2" pr=1902 sha="bbbb2222bbbb2222bbbb2222bbbb2222bbbb2222"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "_r2_attr" "" "unit_lint" "1756598400"
    fixture_git_merge_base_ancestor "$sha"
    fixture_commit_subject "$sha" "fix(voice): rate-limit second probe"

    # Pre-seed state-файл: last_block_ts = NOW - 60s (внутри 14400s cooldown).
    local now_ts
    now_ts="$(date +%s)"
    local recent_ts=$((now_ts - 60))
    python3 -c "
import json
d = {'$cid': $recent_ts}
print(json.dumps(d))
" > "$STALE_AUTO_BLOCK_STATE_DIR/auto-block-rate.json"

    run_merge_gate

    # 0 block-команд (state заблокировал).
    local blocks
    blocks="$(grep -cE "hermes .* block.* ${cid} " "$GH_JOURNAL" || true)"
    assert_eq "0" "$blocks" "rate-limit: 0 block-команд в пределах 4ч" || return 1

    # stderr содержит warn про rate-limit.
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    assert_contains "rate-limited" "$stderr_log" "stderr логирует warn про rate-limit" || return 1

    # state-файл НЕ перезаписан (old ts сохранился).
    local stored_ts
    stored_ts="$(python3 -c "
import json
d = json.load(open('$STALE_AUTO_BLOCK_STATE_DIR/auto-block-rate.json'))
print(d.get('$cid', 'MISSING'))
")"
    assert_eq "$recent_ts" "$stored_ts" "state-файл НЕ перезаписан при rate-limit skip" || return 1
}

# ============================================================================
# R3. Второй auto-block после > 14400s → разрешён, state перезаписан.
# ============================================================================
test_R3_rate_limit_allows_after_4h() {
    new_test
    REPO_DIR="$TEST_TMP"
    export REPO_DIR
    export STALE_AUTO_BLOCK_STATE_DIR="$TEST_TMP/auto-block-state"
    mkdir -p "$STALE_AUTO_BLOCK_STATE_DIR"

    local cid="t_r3" pr=1903 sha="cccc3333cccc3333cccc3333cccc3333cccc3333"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "_r3_attr" "" "unit_lint" "1756598400"
    fixture_git_merge_base_ancestor "$sha"
    fixture_commit_subject "$sha" "fix(voice): cold-cache rate-limit probe"

    # Pre-seed state-файл с ts = NOW - 15000s (> 14400s cooldown).
    local now_ts
    now_ts="$(date +%s)"
    local old_ts=$((now_ts - 15000))
    python3 -c "
import json
d = {'$cid': $old_ts}
print(json.dumps(d))
" > "$STALE_AUTO_BLOCK_STATE_DIR/auto-block-rate.json"

    run_merge_gate

    # 1 block-команда (cooldown истёк).
    local blocks
    blocks="$(grep -cE "hermes .* block.* ${cid} " "$GH_JOURNAL" || true)"
    assert_eq "1" "$blocks" "rate-limit: 1 block после 14400s+" || return 1

    # state-файл перезаписан (новый ts > старого).
    local stored_ts
    stored_ts="$(python3 -c "
import json
d = json.load(open('$STALE_AUTO_BLOCK_STATE_DIR/auto-block-rate.json'))
print(d.get('$cid', 'MISSING'))
")"
    if ! [ "$stored_ts" -gt "$old_ts" ]; then
        printf '  %sassert fail:%s state-файл не обновлён: stored=%s > old=%s ?\n' \
            "$RED" "$END" "$stored_ts" "$old_ts" >&2
        return 1
    fi
}

# ============================================================================
# R4. Reason содержит `<sha> <commit_message_short>` (после первого пробела
#     после upstream_sha). commit_message_short = первая строка
#     `git log -1 <sha> --pretty=%s` (subject upstream-коммита).
# ============================================================================
test_R4_reason_includes_commit_subject() {
    new_test
    REPO_DIR="$TEST_TMP"
    export REPO_DIR
    export STALE_AUTO_BLOCK_STATE_DIR="$TEST_TMP/auto-block-state"
    mkdir -p "$STALE_AUTO_BLOCK_STATE_DIR"

    local cid="t_r4" pr=1904 sha="dddd4444dddd4444dddd4444dddd4444dddd4444"
    local subject="fix(voice): barge-in race in track_mode_music_active"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "_r4_attr" "" "unit_lint" "1756598400"
    fixture_git_merge_base_ancestor "$sha"
    fixture_commit_subject "$sha" "$subject"

    run_merge_gate

    local block_line
    block_line="$(grep -E "hermes .* block.* ${cid}" "$GH_JOURNAL" | head -1)"

    # reason содержит sha-prefix.
    assert_contains "${sha:0:8}" "$block_line" "reason содержит sha-prefix" || return 1
    # reason содержит subject (commit_message_short).
    # subject может содержать спецсимволы, проверяем как substring.
    if printf '%s' "$block_line" | grep -qF "$subject"; then
        :
    else
        printf '  %sassert fail:%s reason не содержит commit_subject "%s"\n  line=%s\n' \
            "$RED" "$END" "$subject" "$block_line" >&2
        return 1
    fi
}

# ============================================================================
# R5. body comment содержит URL upstream + diff из `git log --oneline
#     <create_time>..origin/develop | grep <sig>`. Для strat B (sig есть).
# ============================================================================
test_R5_body_patch_contains_url_and_diff() {
    new_test
    REPO_DIR="$TEST_TMP"
    export REPO_DIR
    export STALE_AUTO_BLOCK_STATE_DIR="$TEST_TMP/auto-block-state"
    mkdir -p "$STALE_AUTO_BLOCK_STATE_DIR"

    local cid="t_r5" pr=1905 sha="eeee5555eeee5555eeee5555eeee5555eeee5555"
    local upstream_sha="06b83b01b6a8c1de76c32bf90d809f0cfa809ffc"
    local attr="_r5_attr"
    fixture_diag_card "$cid" "ready" "$pr" "$sha" "develop" \
        "$attr" "" "unit_lint" "1756598400"
    # Strategy B (sig match) — detector возвращает upstream_sha через git log -S.
    fixture_git_log_hit_attr "$attr" "$upstream_sha"
    fixture_commit_subject "$upstream_sha" "fix(voice): upstream diag fix"

    # Для diff в body: моделируем `git log --oneline origin/develop
    # <create_time>.. --grep <attr>` через фикстуру
    # STALE_DIAG_LOG_ONELINE_<attr>=<lines...>.
    set_state "STALE_DIAG_LOG_ONELINE_${attr}" "${upstream_sha:0:8} fix(voice): upstream diag fix (${attr})"

    run_merge_gate

    # 1) comment в журнале для cid.
    local comment_count
    comment_count="$(grep -cE "hermes .* comment.* ${cid} " "$GH_JOURNAL" || true)"
    if [ "$comment_count" -lt 1 ]; then
        printf '  %sassert fail:%s body patch comment не найден в журнале\n' \
            "$RED" "$END" >&2
        return 1
    fi

    # 2) URL upstream-коммита присутствует в одной из comment-строк.
    #    Mock hermes компилирует многострочный comment в одну JSON-строку,
    #    поэтому ищем через grep -F по всему журналу.
    local expected_url="https://github.com/${GH_REPO:-krikz/test-repo}/commit/${upstream_sha}"
    if grep -qF "$expected_url" "$GH_JOURNAL"; then
        :
    else
        printf '  %sassert fail:%s URL upstream-коммита не найден в body patch:\n  expected: %s\n' \
            "$RED" "$END" "$expected_url" >&2
        return 1
    fi

    # 3) upstream-sha-prefix есть в body patch (подтверждает diff line).
    if grep -qF "${upstream_sha:0:8}" "$GH_JOURNAL"; then
        :
    else
        printf '  %sassert fail:%s upstream-sha-prefix "%s" не найден в body patch\n' \
            "$RED" "$END" "${upstream_sha:0:8}" >&2
        return 1
    fi

    # 4) diff-секция присутствует в body patch (ADR-0035 / t_d83c9430).
    if grep -qF "Upstream-фикс (diff):" "$GH_JOURNAL"; then
        :
    else
        printf '  %sassert fail:%s diff-секция "Upstream-фикс (diff):" не найдена в body patch\n' \
            "$RED" "$END" >&2
        return 1
    fi
}

# ============================================================================
# Запуск (pattern как в test_merge_gate_stale_after_upstream_fix.sh).
# ============================================================================
run_test "R1.  state-файл создан при первом block"        test_R1_state_file_created_on_first_block
run_test "R2.  rate-limit skip в пределах 4ч"            test_R2_rate_limit_skips_within_4h
run_test "R3.  rate-limit allow после 4ч"               test_R3_rate_limit_allows_after_4h
run_test "R4.  reason содержит commit_subject"          test_R4_reason_includes_commit_subject
run_test "R5.  body patch содержит URL + diff"          test_R5_body_patch_contains_url_and_diff

summary
