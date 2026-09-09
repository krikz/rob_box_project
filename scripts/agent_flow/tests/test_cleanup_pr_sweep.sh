#!/bin/bash
# ============================================================================
# test_cleanup_pr_sweep.sh — ретро 14.08 t_3cfb3b5b
#
# Регресс-гард для sweep_stale_pr_branches() в agent-flow-cleanup-249.sh:
# удаление stale remote-веток по состоянию их PR (merged > MERGED_STALE_HOURS,
# closed > CLOSED_STALE_HOURS) с guard'ами: OPEN PR, защищённые ветки,
# round-ветки, fork-PR, переиспользование ветки после merge/close,
# dry-run, ветка уже удалена.
#
# Стратегия (как в test_install_ensure_cleanup_cron.sh): функция извлекается
# из скрипта через awk, подключается в тестовом окружении с мок-gh, который
# читает фикстуры из env (GH_*_TSV) и пишет журнал DELETE-вызовов.
#
# Run:
#   bash scripts/agent_flow/tests/test_cleanup_pr_sweep.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLEANUP_SH="$TEST_DIR/../agent-flow-cleanup-249.sh"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# Явный дефолт: не даём «унаследованному» DRY_RUN=1 из окружения оператора
# превратить сценарии в dry-run (см. сценарий I, где DRY_RUN=1 осознанно).
DRY_RUN=0

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "PASS: $*"; }

# --- извлекаем sweep_stale_pr_branches() из cleanup-249 ----------------------
awk '/^sweep_stale_pr_branches\(\) \{/{f=1} f{print} f && /^}$/{exit}' \
    "$CLEANUP_SH" > "$WORK/sweep.sh"
[ -s "$WORK/sweep.sh" ] || fail "sweep_stale_pr_branches() not found in $CLEANUP_SH"

# --- мок gh: фикстуры из env (уже TSV — --jq «применён»), DELETE в журнал ----
mkdir -p "$WORK/bin"
cat > "$WORK/bin/gh" <<'GH_MOCK_EOF'
#!/bin/bash
journal="${GH_JOURNAL:-/dev/null}"

subcmd="${1:-}"; shift || true

case "$subcmd" in
    auth) exit 0 ;;
    repo)
        # gh repo view <repo> --json defaultBranchRef --jq .defaultBranchRef.name
        printf '%s\n' "${GH_DEFAULT_BRANCH:-main}"
        exit 0
        ;;
    pr)
        action="${1:-}"; shift || true
        case "$action" in
            list)
                if printf '%s' "$*" | grep -q -- '--state open'; then
                    printf '%s\n' "${GH_OPEN_TSV:-}"
                elif printf '%s' "$*" | grep -q -- '--state merged'; then
                    printf '%s\n' "${GH_MERGED_TSV:-}"
                elif printf '%s' "$*" | grep -q -- '--state closed'; then
                    printf '%s\n' "${GH_CLOSED_TSV:-}"
                fi
                exit 0
                ;;
            *) exit 0 ;;
        esac
        ;;
    api)
        if printf '%s' "$*" | grep -q -- '-X DELETE'; then
            # gh api -X DELETE repos/<repo>/git/refs/heads/<enc>
            # (decod через sed: ${var//%7D/}} в bash парсится как «пустая замена + }»)
            enc="$(printf '%s' "$*" | sed -nE 's#.*git/refs/heads/([^ ]+).*#\1#p')"
            branch="$(printf '%s' "$enc" | sed -e 's/%7B/{/g' -e 's/%7D/}/g')"
            printf '%s\tDELETE %s raw=%s\n' "$(date -Iseconds 2>/dev/null || date)" "$branch" "$enc" >>"$journal"
            exit 0
        fi
        if printf '%s' "$*" | grep -q -- 'branches?per_page=100'; then
            printf '%s\n' "${GH_BRANCHES_TSV:-}"
            exit 0
        fi
        if printf '%s' "$*" | grep -q -- 'commits/'; then
            # gh api repos/<repo>/commits/<sha> --jq .commit.committer.date
            sha="$(printf '%s' "$*" | sed -nE 's#.*commits/([0-9a-f]+).*#\1#p')"
            if [ -n "${GH_COMMIT_DATES:-}" ]; then
                line="$(printf '%s\n' "$GH_COMMIT_DATES" | grep "^${sha}=" | head -n1 || true)"
                [ -n "$line" ] && printf '%s\n' "${line#*=}"
            fi
            exit 0
        fi
        exit 0
        ;;
    *) exit 0 ;;
esac
GH_MOCK_EOF
chmod +x "$WORK/bin/gh"

# --- прогон одного сценария --------------------------------------------------
# Фикстуры берутся из текущих переменных GH_*_TSV / GH_COMMIT_DATES / DRY_RUN.
run_scenario() {
    local name="$1"
    local journal="$WORK/journal.$name"
    : > "$journal"
    env \
        GH_REPO=krikz/rob_box_project \
        GH_JOURNAL="$journal" \
        GH_DEFAULT_BRANCH="${GH_DEFAULT_BRANCH:-main}" \
        GH_OPEN_TSV="${GH_OPEN_TSV:-}" \
        GH_MERGED_TSV="${GH_MERGED_TSV:-}" \
        GH_CLOSED_TSV="${GH_CLOSED_TSV:-}" \
        GH_BRANCHES_TSV="${GH_BRANCHES_TSV:-}" \
        GH_COMMIT_DATES="${GH_COMMIT_DATES:-}" \
        DRY_RUN="${DRY_RUN:-0}" \
        PATH="$WORK/bin:$PATH" \
        bash -c '
            set -euo pipefail
            LOG_PREFIX="[cleanup-249]"
            log() { printf "%s %s %s\n" "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }
            MERGED_STALE_HOURS="${MERGED_STALE_HOURS:-2}"
            CLOSED_STALE_HOURS="${CLOSED_STALE_HOURS:-24}"
            source "$1"
            sweep_stale_pr_branches
        ' _ "$WORK/sweep.sh" 2>/dev/null || fail "$name: sweep crashed"
}

assert_no_delete() { # $1=journal $2=branch $3=msg
    if grep -q "DELETE $2" "$1" 2>/dev/null; then
        fail "$3 (DELETE $2 найден)"
    fi
}

# ============================================================================
echo "=== A: merged PR > 2ч, ветка существует, не переиспользована → DELETE"
GH_MERGED_TSV="100	z-devops/t_a-merged-stale	krikz	2026-08-01T10:00:00Z"
GH_CLOSED_TSV=""; GH_OPEN_TSV=""
GH_BRANCHES_TSV="z-devops/t_a-merged-stale	1111111111111111111111111111111111111111"
GH_COMMIT_DATES="1111111111111111111111111111111111111111=2026-08-01T09:00:00Z"
run_scenario A
grep -q "DELETE z-devops/t_a-merged-stale" "$WORK/journal.A" || fail "A: ожидался DELETE"
pass "A: stale merged → DELETE"

echo "=== B: merged PR < 2ч → keep"
GH_MERGED_TSV="101	z-devops/t_b-fresh-merged	krikz	$(date -u -d '1 hour ago' +%Y-%m-%dT%H:%M:%SZ)"
GH_CLOSED_TSV=""; GH_OPEN_TSV=""
GH_BRANCHES_TSV="z-devops/t_b-fresh-merged	2222222222222222222222222222222222222222"
GH_COMMIT_DATES="2222222222222222222222222222222222222222=2026-08-14T00:00:00Z"
run_scenario B
assert_no_delete "$WORK/journal.B" "z-devops/t_b-fresh-merged" "B: свежий merged не должен удаляться"
pass "B: свежий merged → keep"

echo "=== C: closed PR > 24ч, ветка с { } в имени → DELETE c URL-encoding"
GH_MERGED_TSV=""
GH_CLOSED_TSV="102	z-{agent}/680-deployment-critical-test-main-nav2-criti	krikz	2026-08-12T18:00:00Z	null"
GH_OPEN_TSV=""
GH_BRANCHES_TSV="z-{agent}/680-deployment-critical-test-main-nav2-criti	3333333333333333333333333333333333333333"
GH_COMMIT_DATES="3333333333333333333333333333333333333333=2026-08-12T15:00:00Z"
run_scenario C
grep -q "DELETE z-{agent}/680-deployment-critical-test-main-nav2-criti" "$WORK/journal.C" || fail "C: ожидался DELETE (журнал: $(cat "$WORK/journal.C"))"
grep -q "%7Bagent%7D" "$WORK/journal.C" || fail "C: URL-encoding { } в DELETE"
pass "C: stale closed + { } → DELETE c encoding"

echo "=== D: closed PR < 24ч → keep"
GH_MERGED_TSV=""
GH_CLOSED_TSV="103	z-devops/t_c-fresh-closed	krikz	$(date -u -d '5 hours ago' +%Y-%m-%dT%H:%M:%SZ)	null"
GH_OPEN_TSV=""
GH_BRANCHES_TSV="z-devops/t_c-fresh-closed	4444444444444444444444444444444444444444"
GH_COMMIT_DATES="4444444444444444444444444444444444444444=2026-08-14T00:00:00Z"
run_scenario D
assert_no_delete "$WORK/journal.D" "z-devops/t_c-fresh-closed" "D: свежий closed не должен удаляться"
pass "D: свежий closed → keep"

echo "=== E: OPEN PR guard — stale merged, но есть OPEN PR → keep"
GH_MERGED_TSV="104	z-devops/t_d-open-pr	krikz	2026-08-01T10:00:00Z"
GH_CLOSED_TSV=""
GH_OPEN_TSV="z-devops/t_d-open-pr	krikz"
GH_BRANCHES_TSV="z-devops/t_d-open-pr	5555555555555555555555555555555555555555"
GH_COMMIT_DATES="5555555555555555555555555555555555555555=2026-08-01T09:00:00Z"
run_scenario E
assert_no_delete "$WORK/journal.E" "z-devops/t_d-open-pr" "E: OPEN PR guard"
pass "E: OPEN PR → keep"

echo "=== F: защищённые ветки (develop/main) → keep"
GH_MERGED_TSV="105	develop	krikz	2026-08-01T10:00:00Z
106	main	krikz	2026-08-01T10:00:00Z"
GH_CLOSED_TSV=""; GH_OPEN_TSV=""
GH_BRANCHES_TSV="develop	6666666666666666666666666666666666666666
main	7777777777777777777777777777777777777777"
GH_COMMIT_DATES="6666666666666666666666666666666666666666=2026-08-01T09:00:00Z
7777777777777777777777777777777777777777=2026-08-01T09:00:00Z"
run_scenario F
assert_no_delete "$WORK/journal.F" "develop" "F: develop защищена"
assert_no_delete "$WORK/journal.F" "main" "F: main защищена"
pass "F: develop/main → keep"

echo "=== G: fork-PR guard → keep (owner != repo owner)"
GH_MERGED_TSV="107	z-devops/t_f-fork	other-user	2026-08-01T10:00:00Z"
GH_CLOSED_TSV=""; GH_OPEN_TSV=""
GH_BRANCHES_TSV="z-devops/t_f-fork	8888888888888888888888888888888888888888"
GH_COMMIT_DATES="8888888888888888888888888888888888888888=2026-08-01T09:00:00Z"
run_scenario G
assert_no_delete "$WORK/journal.G" "z-devops/t_f-fork" "G: fork guard"
pass "G: fork PR → keep"

echo "=== H: переиспользование — HEAD новее closedAt → keep"
GH_MERGED_TSV=""
GH_CLOSED_TSV="108	z-devops/t_g-reused	krikz	2026-08-12T18:00:00Z	null"
GH_OPEN_TSV=""
GH_BRANCHES_TSV="z-devops/t_g-reused	9999999999999999999999999999999999999999"
GH_COMMIT_DATES="9999999999999999999999999999999999999999=2026-08-13T10:00:00Z"  # новее closedAt
run_scenario H
assert_no_delete "$WORK/journal.H" "z-devops/t_g-reused" "H: reuse guard"
pass "H: ветка переиспользована → keep"

echo "=== I: dry-run → DELETE не вызывается"
GH_MERGED_TSV="109	z-devops/t_i-dry	krikz	2026-08-01T10:00:00Z"
GH_CLOSED_TSV=""; GH_OPEN_TSV=""
GH_BRANCHES_TSV="z-devops/t_i-dry	aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
GH_COMMIT_DATES="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa=2026-08-01T09:00:00Z"
DRY_RUN=1
run_scenario I
assert_no_delete "$WORK/journal.I" "z-devops/t_i-dry" "I: dry-run не удаляет"
DRY_RUN=0
pass "I: dry-run → no DELETE"

echo "=== J: round-ветка z-{e2e}/test-round-* → keep (секция 4)"
GH_MERGED_TSV="110	z-{e2e}/test-round-77	krikz	2026-08-01T10:00:00Z"
GH_CLOSED_TSV=""; GH_OPEN_TSV=""
GH_BRANCHES_TSV="z-{e2e}/test-round-77	bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
GH_COMMIT_DATES="bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb=2026-08-01T09:00:00Z"
run_scenario J
assert_no_delete "$WORK/journal.J" "z-{e2e}/test-round-77" "J: round-ветка"
pass "J: round-ветка → keep"

echo "=== K: ветки нет на remote (нет в branches-карте) → skip, без DELETE"
GH_MERGED_TSV="111	z-devops/t_k-gone	krikz	2026-08-01T10:00:00Z"
GH_CLOSED_TSV=""; GH_OPEN_TSV=""
GH_BRANCHES_TSV=""; GH_COMMIT_DATES=""
run_scenario K
assert_no_delete "$WORK/journal.K" "z-devops/t_k-gone" "K: уже удалена"
pass "K: ветка уже удалена → skip"

echo "=== L: merged PR только в closed-списке (mergedAt set) → merged-правило"
GH_MERGED_TSV=""
GH_CLOSED_TSV="112	z-devops/t_l-merged-in-closed	krikz	2026-08-14T20:00:00Z	2026-08-01T10:00:00Z"
GH_OPEN_TSV=""
GH_BRANCHES_TSV="z-devops/t_l-merged-in-closed	cccccccccccccccccccccccccccccccccccccccc"
GH_COMMIT_DATES="cccccccccccccccccccccccccccccccccccccccc=2026-08-01T09:00:00Z"
run_scenario L
grep -q "DELETE z-devops/t_l-merged-in-closed" "$WORK/journal.L" || fail "L: merged-правило в closed-проходе (журнал: $(cat "$WORK/journal.L"))"
pass "L: merged в closed-списке → merged-правило"

echo
echo "ALL SCENARIOS PASSED"
