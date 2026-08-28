#!/bin/bash
# ============================================================================
# test_cleanup_blocked_retry.sh — ретро 22.08 t_deba66ef
#
# Регресс-гард для retry_unblock_freed_cards() в agent-flow-cleanup-249.sh:
# auto-unblock blocked-карточек с branch_name, если worktree для ветки свободен.
#
# Стратегия (как в test_cleanup_pr_sweep.sh):
#   - функция извлекается из скрипта через awk, подключается в тестовом окружении;
#   - мок-gh (в данном случае мок-hermes) читает фикстуры из env (BLOCKED_JSON)
#     и пишет журнал unblock-вызовов;
#   - мок-git читает фикстуру BRANCHES_OCCUPIED (список занятых веток) и для
#     каждой возвращает "already checked out" в stderr.
#
# Сценарии:
#   A. branch_name есть, workspace_kind=worktree, ветка свободна → UNBLOCK
#   B. branch_name есть, но ветка занята (другой worktree) → skip, no UNBLOCK
#   C. branch_name пуст (retro/PR-orphan) → skip
#   D. workspace_kind=scratch → skip
#   E. BLOCKED_MIN_BLOCKED_HOURS > 0 и карточка моложе → skip
#   F. .worktrees/<basename> уже существует (другая карточка владеет) → skip
#   G. DRY-RUN → UNBLOCK не вызывается
#
# Run:
#   bash scripts/agent_flow/tests/test_cleanup_blocked_retry.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLEANUP_SH="$TEST_DIR/../agent-flow-cleanup-249.sh"

WORK=$(mktemp -d)
# DEBUG_KEEP_WORK=1 — оставить WORK после прогона для инспекции (по умолчанию off)
if [ -n "${DEBUG_KEEP_WORK:-}" ]; then
    echo "DEBUG: WORK=$WORK"
    trap 'echo "WORK=$WORK (kept for inspection)" >&2' EXIT
else
    trap 'rm -rf "$WORK"' EXIT
fi

# Явный дефолт: не даём «унаследованному» DRY_RUN=1 из окружения превратить
# сценарии в dry-run (см. сценарий G, где DRY_RUN=1 осознанно).
DRY_RUN=0

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "PASS: $*"; }

# --- извлекаем retry_unblock_freed_cards() из cleanup-249 --------------------
awk '/^retry_unblock_freed_cards\(\) \{/{f=1} f{print} f && /^}$/{exit}' \
    "$CLEANUP_SH" > "$WORK/retry.sh"
[ -s "$WORK/retry.sh" ] || fail "retry_unblock_freed_cards() not found in $CLEANUP_SH"

# --- мок-hermes: фикстуры из env, UNBLOCK в журнал ----------------------------
mkdir -p "$WORK/bin"
cat > "$WORK/bin/hermes" <<'HERMES_MOCK_EOF'
#!/bin/bash
journal="${UNBLOCK_JOURNAL:-/dev/null}"
# echo "DEBUG hermes $*" >&2  # раскомментируйте для трассировки

# Разбираем только наш целевой канал: kanban list|unblock
# Игнорируем остальные hermes-вызовы (auth, project, etc.) → молча exit 0.
case "$*" in
    *kanban*list*"--status"*"blocked"*"--json"*)
        printf '%s\n' "${BLOCKED_JSON:-[]}"
        exit 0
        ;;
    *kanban*unblock*)
        # hermes --profile devops kanban --board X unblock --reason '...' <id>
        # Извлекаем последний positional id и reason
        reason="$(printf '%s' "$*" | sed -nE "s/.*--reason '([^']*)'.*/\\1/p")"
        id="$(printf '%s' "$*" | awk '{print $NF}')"
        printf '%s\tUNBLOCK %s reason=%s\n' "$(date -Iseconds 2>/dev/null || date)" "$id" "$reason" >> "$journal"
        exit 0
        ;;
    *) exit 0 ;;
esac
HERMES_MOCK_EOF
chmod +x "$WORK/bin/hermes"

# --- мок-git: BRANCHES_OCCUPIED (список занятых веток) -----------------------
# `git worktree add` для каждой занятой ветки возвращает 128 + stderr.
# Для свободных — успех (0), создаёт каталог, потом его сносят (worktree remove).
cat > "$WORK/bin/git" <<'GIT_MOCK_EOF'
#!/bin/bash
# Поддерживаем только: worktree add <dir> <branch>, worktree remove --force <dir>,
# остальное молча пропускаем (real-git в PATH не дёргаем — иначе тест ляжет на
# реальном репо).
journal="${WORKTREE_JOURNAL:-/dev/null}"
case "$1" in
    worktree)
        sub="$2"
        case "$sub" in
            add)
                # worktree add <dir> <branch>
                target_dir="$3"; branch="$4"
                occupied="${BRANCHES_OCCUPIED:-}"
                # branch может содержать {} — проверим точное вхождение через case
                hit=0
                if [ -n "$occupied" ]; then
                    while IFS= read -r b; do
                        [ -z "$b" ] && continue
                        if [ "$b" = "$branch" ]; then hit=1; break; fi
                    done <<< "$occupied"
                fi
                if [ "$hit" = "1" ]; then
                    printf '%s\tADD %s → already checked out\n' "$(date -Iseconds 2>/dev/null || date)" "$branch" >> "$journal"
                    echo "fatal: '${branch}' is already checked out at '/home/builder/rob_box_project/.worktrees/_fake_'" >&2
                    exit 128
                fi
                # успех — создаём каталог-маркер (для последующего remove)
                mkdir -p "$target_dir"
                printf '%s\tADD %s → OK (probe created)\n' "$(date -Iseconds 2>/dev/null || date)" "$branch" >> "$journal"
                exit 0
                ;;
            remove)
                # worktree remove --force <dir> (или без --force)
                shift  # remove
                case "$2" in --force|"") ;; *) shift ;; esac
                dir="$2"
                rm -rf "$dir" 2>/dev/null || true
                printf '%s\tREMOVE %s\n' "$(date -Iseconds 2>/dev/null || date)" "$dir" >> "$journal"
                exit 0
                ;;
            *) exit 0 ;;
        esac
        ;;
    *) exit 0 ;;
esac
GIT_MOCK_EOF
chmod +x "$WORK/bin/git"

# --- прогон одного сценария --------------------------------------------------
# Фикстуры берутся из текущих переменных: BLOCKED_JSON, BRANCHES_OCCUPIED,
# EXISTING_WORKTREES (через env в скрипте), DRY_RUN, BLOCKED_MIN_BLOCKED_HOURS.
run_scenario() {
    local name="$1"
    local journal="$WORK/journal.$name"
    : > "$journal"
    # Гарантируем существование fake_repo — иначе guard по KANBAN_REPO сработает
    # раньше, чем наш mock-hermes/mock-git успеют что-то зафиксировать.
    mkdir -p "$WORK/fake_repo"
    env \
        KANBAN_BOARD="${KANBAN_BOARD:-robbox}" \
        KANBAN_REPO="$WORK/fake_repo" \
        BLOCKED_JSON="${BLOCKED_JSON:-[]}" \
        BRANCHES_OCCUPIED="${BRANCHES_OCCUPIED:-}" \
        UNBLOCK_JOURNAL="$journal" \
        WORKTREE_JOURNAL="$WORK/wt-journal.$name" \
        DRY_RUN="${DRY_RUN:-0}" \
        BLOCKED_MIN_BLOCKED_HOURS="${BLOCKED_MIN_BLOCKED_HOURS:-0}" \
        PATH="$WORK/bin:$PATH" \
        bash -c '
            set -euo pipefail
            LOG_PREFIX="[cleanup-249]"
            log() { printf "%s %s %s\n" "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }
            DRY_RUN="${DRY_RUN:-0}"
            source "$1"
            retry_unblock_freed_cards
        ' _ "$WORK/retry.sh" 2>"$WORK/stderr.$name" || {
            echo "--- stderr ---"; cat "$WORK/stderr.$name"
            fail "$name: retry_unblock_freed_cards crashed"
        }
}

assert_unblocked() { # $1=journal $2=id $3=msg
    if ! grep -q "UNBLOCK $2 " "$1"; then
        fail "$3 (UNBLOCK $2 не найден)"
    fi
}
assert_not_unblocked() { # $1=journal $2=id $3=msg
    if grep -q "UNBLOCK $2 " "$1"; then
        fail "$3 (UNBLOCK $2 найден)"
    fi
}

# --- фикстуры карточек --------------------------------------------------------
# Используем NOW (текущий epoch в секундах) для started_at, чтобы карточки были
# «старыми» (даже при BLOCKED_MIN_BLOCKED_HOURS > 0). Минус — это позволяет
# нам контролировать возраст независимо от того, когда запустят тест.
NOW="$(date +%s)"
TWO_HOURS_AGO=$(( NOW - 7200 ))   # 2ч назад
TEN_MIN_AGO=$(( NOW - 600 ))       # 10 мин назад

# ============================================================================
echo "=== A: branch_name есть, workspace_kind=worktree, ветка свободна → UNBLOCK"
BLOCKED_JSON=$(cat <<EOF
[
  {"id":"t_blocked_A","branch_name":"z-devops/test-A","workspace_kind":"worktree","workspace_path":"/home/builder/rob_box_project/.worktrees/t_blocked_A","started_at":${TWO_HOURS_AGO},"created_at":${TWO_HOURS_AGO}}
]
EOF
)
BRANCHES_OCCUPIED=""
run_scenario A
assert_unblocked "$WORK/journal.A" "t_blocked_A" "A: ожидался UNBLOCK"
grep -q "ADD z-devops/test-A → OK (probe created)" "$WORK/wt-journal.A" || fail "A: не было dry-run add"
pass "A: worktree freed → UNBLOCK"

echo "=== B: branch_name есть, но ветка занята другим worktree → skip"
BLOCKED_JSON=$(cat <<EOF
[
  {"id":"t_blocked_B","branch_name":"z-devops/test-B","workspace_kind":"worktree","workspace_path":"/home/builder/rob_box_project/.worktrees/t_blocked_B","started_at":${TWO_HOURS_AGO},"created_at":${TWO_HOURS_AGO}}
]
EOF
)
BRANCHES_OCCUPIED="z-devops/test-B"
run_scenario B
assert_not_unblocked "$WORK/journal.B" "t_blocked_B" "B: UNBLOCK не должен сработать"
grep -q "ADD z-devops/test-B → already checked out" "$WORK/wt-journal.B" || fail "B: dry-run add не зафиксирован"
pass "B: worktree занят → skip, no UNBLOCK"

echo "=== C: branch_name пуст (retro/PR-orphan) → skip"
BLOCKED_JSON=$(cat <<EOF
[
  {"id":"t_blocked_C","branch_name":null,"workspace_kind":"worktree","workspace_path":"/home/builder/rob_box_project/.worktrees/t_blocked_C","started_at":${TWO_HOURS_AGO},"created_at":${TWO_HOURS_AGO}}
]
EOF
)
BRANCHES_OCCUPIED=""
run_scenario C
assert_not_unblocked "$WORK/journal.C" "t_blocked_C" "C: branch_name пуст → skip"
pass "C: branch_name пуст → skip"

echo "=== D: workspace_kind=scratch → skip"
BLOCKED_JSON=$(cat <<EOF
[
  {"id":"t_blocked_D","branch_name":"z-devops/test-D","workspace_kind":"scratch","workspace_path":"/tmp/some-scratch","started_at":${TWO_HOURS_AGO},"created_at":${TWO_HOURS_AGO}}
]
EOF
)
BRANCHES_OCCUPIED=""
run_scenario D
assert_not_unblocked "$WORK/journal.D" "t_blocked_D" "D: workspace_kind=scratch → skip"
pass "D: workspace_kind=scratch → skip"

echo "=== E: BLOCKED_MIN_BLOCKED_HOURS=1, карточка 10 мин назад → skip"
BLOCKED_JSON=$(cat <<EOF
[
  {"id":"t_blocked_E","branch_name":"z-devops/test-E","workspace_kind":"worktree","workspace_path":"/home/builder/rob_box_project/.worktrees/t_blocked_E","started_at":${TEN_MIN_AGO},"created_at":${TEN_MIN_AGO}}
]
EOF
)
BRANCHES_OCCUPIED=""
BLOCKED_MIN_BLOCKED_HOURS=1
run_scenario E
assert_not_unblocked "$WORK/journal.E" "t_blocked_E" "E: свежая blocked не должна unblock'нуться"
BLOCKED_MIN_BLOCKED_HOURS=0
pass "E: BLOCKED_MIN_BLOCKED_HOURS=1 + свежая карточка → skip"

echo "=== F: .worktrees/<basename> уже существует (другая карточка владеет) → skip"
# Создаём worktree-маркер в фейковом KANBAN_REPO (WORK/fake_repo/.worktrees/...).
# Но KANBAN_REPO=fake_repo — каталог не существует. Нужно создать.
mkdir -p "$WORK/fake_repo/.worktrees/t_blocked_F"
BLOCKED_JSON=$(cat <<EOF
[
  {"id":"t_blocked_F","branch_name":"z-devops/test-F","workspace_kind":"worktree","workspace_path":"/home/builder/rob_box_project/.worktrees/t_blocked_F","started_at":${TWO_HOURS_AGO},"created_at":${TWO_HOURS_AGO}}
]
EOF
)
BRANCHES_OCCUPIED=""
run_scenario F
assert_not_unblocked "$WORK/journal.F" "t_blocked_F" "F: workspace_path существует → skip"
pass "F: workspace_path существует → skip"

echo "=== G: DRY-RUN=1 → UNBLOCK не вызывается"
BLOCKED_JSON=$(cat <<EOF
[
  {"id":"t_blocked_G","branch_name":"z-devops/test-G","workspace_kind":"worktree","workspace_path":"/home/builder/rob_box_project/.worktrees/t_blocked_G","started_at":${TWO_HOURS_AGO},"created_at":${TWO_HOURS_AGO}}
]
EOF
)
BRANCHES_OCCUPIED=""
DRY_RUN=1
run_scenario G
assert_not_unblocked "$WORK/journal.G" "t_blocked_G" "G: dry-run не должен unblock'нуть"
grep -q "DRY-RUN: unblock'нул бы t_blocked_G" "$WORK/stderr.G" || fail "G: dry-run не зафиксирован в логе"
DRY_RUN=0
pass "G: DRY-RUN → no UNBLOCK, лог содержит DRY-RUN строку"

echo "=== H: несколько карточек одновременно — смешанный вердикт"
BLOCKED_JSON=$(cat <<EOF
[
  {"id":"t_blocked_H1","branch_name":"z-devops/test-H1","workspace_kind":"worktree","workspace_path":"/home/builder/rob_box_project/.worktrees/t_blocked_H1","started_at":${TWO_HOURS_AGO},"created_at":${TWO_HOURS_AGO}},
  {"id":"t_blocked_H2","branch_name":"z-devops/test-H2","workspace_kind":"worktree","workspace_path":"/home/builder/rob_box_project/.worktrees/t_blocked_H2","started_at":${TWO_HOURS_AGO},"created_at":${TWO_HOURS_AGO}},
  {"id":"t_blocked_H3","branch_name":null,"workspace_kind":"worktree","workspace_path":"/home/builder/rob_box_project/.worktrees/t_blocked_H3","started_at":${TWO_HOURS_AGO},"created_at":${TWO_HOURS_AGO}},
  {"id":"t_blocked_H4","branch_name":"z-devops/test-H4","workspace_kind":"worktree","workspace_path":"/home/builder/rob_box_project/.worktrees/t_blocked_H4","started_at":${TWO_HOURS_AGO},"created_at":${TWO_HOURS_AGO}}
]
EOF
)
BRANCHES_OCCUPIED="z-devops/test-H4"
run_scenario H
assert_unblocked "$WORK/journal.H" "t_blocked_H1" "H: H1 → UNBLOCK"
assert_unblocked "$WORK/journal.H" "t_blocked_H2" "H: H2 → UNBLOCK"
assert_not_unblocked "$WORK/journal.H" "t_blocked_H3" "H: H3 (branch=null) → skip"
assert_not_unblocked "$WORK/journal.H" "t_blocked_H4" "H: H4 (ветка занята) → skip"
pass "H: смешанный пакет — правильные вердикты"

echo "=== I: hermes CLI не найден → пропуск (cleanup продолжится)"
# Подменяем PATH так, чтобы hermes не нашёлся
BLOCKED_JSON='[]'
BRANCHES_OCCUPIED=""
env -i HOME="$HOME" PATH="/usr/bin:/bin" \
    KANBAN_BOARD="robbox" \
    KANBAN_REPO="$WORK/fake_repo" \
    BLOCKED_JSON="$BLOCKED_JSON" \
    BRANCHES_OCCUPIED="$BRANCHES_OCCUPIED" \
    DRY_RUN="0" BLOCKED_MIN_BLOCKED_HOURS="0" \
    LOG_PREFIX="[cleanup-249]" \
    bash -c '
        set -euo pipefail
        log() { printf "%s %s %s\n" "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }
        source "$1"
        retry_unblock_freed_cards
    ' _ "$WORK/retry.sh" 2>"$WORK/stderr.I" >/dev/null || fail "I: должно быть мягко, не crash"
grep -q "hermes CLI не найден" "$WORK/stderr.I" || fail "I: ожидалось 'hermes CLI не найден' в логе"
pass "I: hermes missing → graceful skip"

echo "=== J: KANBAN_REPO не существует → пропуск"
# Подменяем PATH так, чтобы git тоже не нашёлся → функция вернётся раньше,
# на проверке каталога KANBAN_REPO.
rm -rf "$WORK/fake_repo"
BLOCKED_JSON='[{"id":"t_blocked_J","branch_name":"z-devops/test-J","workspace_kind":"worktree","workspace_path":"/tmp/wp","started_at":'${TWO_HOURS_AGO}',"created_at":'${TWO_HOURS_AGO}'}]'
env -i HOME="$HOME" PATH="$WORK/bin:/usr/bin:/bin" \
    KANBAN_BOARD="robbox" \
    KANBAN_REPO="$WORK/fake_repo" \
    BLOCKED_JSON="$BLOCKED_JSON" \
    DRY_RUN="0" BLOCKED_MIN_BLOCKED_HOURS="0" \
    UNBLOCK_JOURNAL="$WORK/journal.J" \
    LOG_PREFIX="[cleanup-249]" \
    bash -c '
        set -euo pipefail
        log() { printf "%s %s %s\n" "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }
        source "$1"
        retry_unblock_freed_cards
    ' _ "$WORK/retry.sh" 2>"$WORK/stderr.J" >/dev/null || fail "J: должно быть мягко, не crash"
grep -q "KANBAN_REPO=.* не существует" "$WORK/stderr.J" || fail "J: ожидалось 'KANBAN_REPO не существует' в логе"
pass "J: KANBAN_REPO missing → graceful skip"

echo
echo "ALL SCENARIOS PASSED"