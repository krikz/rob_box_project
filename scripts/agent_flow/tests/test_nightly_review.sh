#!/bin/bash
# ============================================================================
# test_nightly_review.sh — acceptance-тесты ночного ревью
# scripts/agent_flow/agent-flow-nightly-review.sh (ADR-0049).
#
# Стратегия (как в test_kanban_retro_create.sh): мокаем `hermes` шелл-скриптом,
# который читает фикстуру доски из $TEST_TMP/kanban_list.json и пишет каждый
# вызов в journal. `gh` НЕ мокаем по умолчанию — проверяем деградацию
# «НЕТ ДАННЫХ». Git — настоящий: поднимаем локальный репо с ref
# refs/remotes/origin/develop и коммитами внутри окна.
#
# Проверяемые гарантии:
#   A. Вне ночного окна → skip, ни одной карточки, exit 0.
#   B. В окне → ровно ОДНА карточка `nightly-review-<date>` на architect.
#   C. Компонентные карточки создаются на top-N по churn, cap
#      COMPONENT_REVIEW_MAX соблюдается.
#   D. Компонент с числом файлов < COMPONENT_REVIEW_MIN_FILES пропускается.
#   E. Кулдаун: живая карточка с маркером `ретро-key: component-review-<slug>-`
#      → на этот компонент карточка НЕ создаётся, на другой — создаётся.
#   F. Sentinel: повторный тик той же ночью → skip (карточки не пересоздаются).
#   G. Деградация: нет gh → секции печатают «НЕТ ДАННЫХ», тик не падает.
#   H. DRY_RUN → ни одного `kanban create`, дайджест на stdout есть.
#   I. Исключения: docs/ в компонентную таблицу не попадает.
#
# Invocation:
#   bash scripts/agent_flow/tests/test_nightly_review.sh
# Возвращает 0 при всех pass, 1 при первом fail.
# ============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/.." && pwd)"
NIGHTLY="$REPO_ROOT/agent-flow-nightly-review.sh"

TEST_TMP="${TEST_TMP:-/tmp/agent-flow-nightly-review-tests.$$}"
rm -rf "$TEST_TMP"
mkdir -p "$TEST_TMP/bin" "$TEST_TMP/state"

KANBAN_JOURNAL="$TEST_TMP/journal"
KANBAN_LIST_FILE="$TEST_TMP/kanban_list.json"
export KANBAN_JOURNAL KANBAN_LIST_FILE

# --- mock hermes ------------------------------------------------------------
cat > "$TEST_TMP/bin/hermes" <<'HERMES_MOCK_EOF'
#!/bin/bash
# Mock hermes: `kanban --board X list --json` отдаёт фикстуру,
# `kanban --board X create ...` пишет вызов в journal и отдаёт JSON карточки.
journal="${KANBAN_JOURNAL:-/dev/null}"
sub="${4:-}"
case "$sub" in
    list)
        cat "${KANBAN_LIST_FILE:-/dev/null}"
        ;;
    create)
        printf 'create\t%s\n' "$*" >> "$journal"
        echo '{"id": "t_mock01", "status": "ready"}'
        ;;
    *)
        echo "mock: unexpected kanban subcommand: $sub" >&2
        exit 2
        ;;
esac
exit 0
HERMES_MOCK_EOF
chmod +x "$TEST_TMP/bin/hermes"

# --- portability shims (только если инструмента реально нет) -----------------
# На Linux-хосте конвейера flock/python3 есть всегда и шимы НЕ создаются.
# В git-bash под Windows (dev-машина Шифу) flock отсутствует, а `python3` —
# Store-заглушка; без шимов тест там не запускается вообще.
if ! command -v flock >/dev/null 2>&1; then
    printf '#!/bin/bash\nexit 0\n' > "$TEST_TMP/bin/flock"
    chmod +x "$TEST_TMP/bin/flock"
    echo "[shim] flock отсутствует — подставлен no-op (лок в тесте не проверяется)"
fi
if ! python3 -c 'pass' >/dev/null 2>&1; then
    printf '#!/bin/bash\nexec python "$@"\n' > "$TEST_TMP/bin/python3"
    chmod +x "$TEST_TMP/bin/python3"
    echo "[shim] python3 недоступен — проксируем на python"
fi

# --- fixture git repo -------------------------------------------------------
# Три компонента с разным churn:
#   src/rob_box_voice   — 3 файла  (top-1)
#   scripts/agent_flow  — 2 файла  (top-2)
#   src/rob_box_llm     — 1 файл   (ниже MIN_FILES=2 → пропуск)
#   docs/adr            — 1 файл   (в EXCLUDE_RE → не компонент вообще)
FIXTURE_REPO="$TEST_TMP/repo"
setup_repo() {
    rm -rf "$FIXTURE_REPO"
    mkdir -p "$FIXTURE_REPO"
    (
        cd "$FIXTURE_REPO" || exit 1
        git init -q .
        git config core.autocrlf false
        git config user.email t@t.t
        git config user.name tester
        mkdir -p src/rob_box_voice scripts/agent_flow src/rob_box_llm docs/adr
        for f in src/rob_box_voice/a.py src/rob_box_voice/b.py src/rob_box_voice/c.py \
                 scripts/agent_flow/x.sh scripts/agent_flow/y.sh \
                 src/rob_box_llm/z.py docs/adr/0099-test.md; do
            printf 'line1\nline2\n' > "$f"
        done
        git add -A
        git commit -qm "feat(voice): фикстура ночного ревью"
        printf 'line3\n' >> src/rob_box_voice/a.py
        git add -A
        git commit -qm "fix(voice): второй коммит фикстуры"
        git update-ref refs/remotes/origin/develop HEAD
    )
}
setup_repo

# --- registry ---------------------------------------------------------------
TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

run_test() {  # $1=name $2=function
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    printf '[ RUN     ] %s\n' "$name"
    if "$fn"; then
        TESTS_PASSED=$((TESTS_PASSED + 1))
        printf '[   PASS  ] %s\n' "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED + 1))
        FAILED_NAMES+=("$name")
        printf '[   FAIL  ] %s\n' "$name"
    fi
}

assert_eq() {  # $1=expected $2=actual $3=msg
    if [ "$1" != "$2" ]; then
        printf '  assert fail: %s\n    expected: %q\n    actual:   %q\n' "$3" "$1" "$2" >&2
        return 1
    fi
}

assert_contains() {  # $1=needle $2=haystack $3=msg
    case "$2" in
        *"$1"*) return 0 ;;
        *) printf '  assert fail: %s\n    needle: %q\n' "$3" "$1" >&2; return 1 ;;
    esac
}

assert_not_contains() {  # $1=needle $2=haystack $3=msg
    case "$2" in
        *"$1"*) printf '  assert fail: %s\n    needle should NOT appear: %q\n' "$3" "$1" >&2; return 1 ;;
        *) return 0 ;;
    esac
}

# --- runner -----------------------------------------------------------------
# Запускает скрипт с чистым journal/sentinel-каталогом. Все env — через
# окружение вызова; PATH подменён так, что `hermes` — мок, а `gh` отсутствует
# (если не задан GH_MOCK=1).
STDOUT_FILE="$TEST_TMP/stdout"
STDERR_FILE="$TEST_TMP/stderr"

run_nightly() {  # $@ = дополнительные VAR=value
    : > "$KANBAN_JOURNAL"
    # GH_REPO пустой — значит `gh` не зовётся вообще (см. _gh_json), и секции
    # GitHub деградируют в «НЕТ ДАННЫХ». Тест НЕ ходит в сеть.
    env \
        HOME="$TEST_TMP" \
        PATH="$TEST_TMP/bin:$PATH" \
        GH_REPO= \
        KANBAN_JOURNAL="$KANBAN_JOURNAL" \
        KANBAN_LIST_FILE="$KANBAN_LIST_FILE" \
        NIGHTLY_REVIEW_TEST_MODE=1 \
        NIGHTLY_REVIEW_DATE="$REVIEW_DATE" \
        NIGHTLY_REVIEW_STATE_DIR="$TEST_TMP/state" \
        LOCK_FILE="$TEST_TMP/nightly.lock" \
        REPO_DIR="$FIXTURE_REPO" \
        KANBAN_BOARD=robbox \
        HERMES_HOME="$TEST_TMP/hermes-home" \
        "$@" \
        bash "$NIGHTLY" > "$STDOUT_FILE" 2> "$STDERR_FILE"
    echo $?
}

REVIEW_DATE="$(date -d 'yesterday' +%F 2>/dev/null || date +%F)"
echo '[]' > "$KANBAN_LIST_FILE"

journal_creates() {  # печатает число `kanban create` вызовов
    # ВНИМАНИЕ: `grep -c` при нуле совпадений печатает 0 И возвращает rc=1,
    # поэтому `|| echo 0` дал бы две строки вместо одной — считаем через
    # переменную.
    local n
    n="$(grep -c '^create' "$KANBAN_JOURNAL" 2>/dev/null || true)"
    printf '%s' "${n:-0}"
}

reset_state() {
    rm -f "$TEST_TMP"/state/*.done 2>/dev/null || true
    echo '[]' > "$KANBAN_LIST_FILE"
}

# ---------------------------------------------------------------------------
# A. Вне ночного окна → skip.
# ---------------------------------------------------------------------------
test_A_outside_window() {
    reset_state
    local rc
    # Окно [HOUR, HOUR+1) специально выставляем на час, который сейчас НЕ идёт.
    local now_h other_h
    now_h="$(date +%-H)"
    other_h=$(( (now_h + 5) % 20 ))
    rc="$(run_nightly NIGHTLY_REVIEW_HOUR="$other_h" NIGHTLY_REVIEW_WINDOW_HOURS=1)"
    assert_eq "0" "$rc" "A: вне окна exit 0" || return 1
    assert_eq "0" "$(journal_creates)" "A: карточки не создаются" || return 1
    assert_contains "вне ночного окна" "$(cat "$STDERR_FILE")" "A: в логе причина skip" || return 1
}

# ---------------------------------------------------------------------------
# B. В окне → nightly-карточка на architect с key nightly-review-<date>.
# ---------------------------------------------------------------------------
test_B_nightly_card() {
    reset_state
    local rc journal
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "B: exit 0" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "🌙 ночной ревью ${REVIEW_DATE}" "$journal" "B: заголовок карточки" || return 1
    assert_contains "retro:nightly-review-${REVIEW_DATE}" "$journal" "B: idempotency-key" || return 1
    assert_contains "--assignee architect" "$journal" "B: assignee=architect" || return 1
    assert_eq "1" "$(journal_creates)" "B: ровно одна карточка (COMPONENT_REVIEW_MAX=0)" || return 1
}

# ---------------------------------------------------------------------------
# C. Компонентные карточки: top-N по churn, cap соблюдается.
# ---------------------------------------------------------------------------
test_C_component_cards() {
    reset_state
    local rc journal
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=1)"
    assert_eq "0" "$rc" "C: exit 0" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_eq "2" "$(journal_creates)" "C: nightly + ровно одна компонентная (cap=1)" || return 1
    assert_contains "ревью компонента: src/rob_box_voice" "$journal" "C: top-1 по churn = voice" || return 1
    assert_contains "component-review-src-rob_box_voice-${REVIEW_DATE}" "$journal" "C: key компонента" || return 1
    assert_contains "--assignee analyst" "$journal" "C: компонентная на analyst" || return 1
}

# ---------------------------------------------------------------------------
# D. Компонент ниже MIN_FILES не получает карточку.
# ---------------------------------------------------------------------------
test_D_min_files() {
    reset_state
    local rc journal
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=9 COMPONENT_REVIEW_MIN_FILES=2)"
    assert_eq "0" "$rc" "D: exit 0" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "ревью компонента: src/rob_box_voice" "$journal" "D: voice (3 файла) есть" || return 1
    assert_contains "ревью компонента: scripts/agent_flow" "$journal" "D: agent_flow (2 файла) есть" || return 1
    assert_not_contains "ревью компонента: src/rob_box_llm" "$journal" "D: llm (1 файл) пропущен" || return 1
}

# ---------------------------------------------------------------------------
# E. Кулдаун по маркеру в body живой карточки.
# ---------------------------------------------------------------------------
test_E_cooldown() {
    reset_state
    cat > "$KANBAN_LIST_FILE" <<'FIXTURE_EOF'
[{"id":"t_old01","title":"🔍 ревью компонента: src/rob_box_voice (позавчера)",
  "status":"ready","assignee":"analyst",
  "body":"старая карточка\nретро-key: component-review-src-rob_box_voice-2026-09-01"}]
FIXTURE_EOF
    local rc journal
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=9)"
    assert_eq "0" "$rc" "E: exit 0" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_not_contains "ревью компонента: src/rob_box_voice" "$journal" "E: voice на кулдауне" || return 1
    assert_contains "ревью компонента: scripts/agent_flow" "$journal" "E: agent_flow не на кулдауне" || return 1
    assert_contains "на кулдауне" "$(cat "$STDERR_FILE")" "E: причина в логе" || return 1
}

# ---------------------------------------------------------------------------
# F. Sentinel: второй тик той же ночью не пересоздаёт карточки.
# ---------------------------------------------------------------------------
test_F_sentinel() {
    reset_state
    local rc
    # Первый тик — в окне (FORCE), но FORCE игнорирует sentinel, поэтому
    # первый прогон делаем через явное окно на текущий час.
    local now_h
    now_h="$(date +%-H)"
    rc="$(run_nightly NIGHTLY_REVIEW_HOUR="$now_h" NIGHTLY_REVIEW_WINDOW_HOURS=1 COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "F: первый тик exit 0" || return 1
    assert_eq "1" "$(journal_creates)" "F: первый тик создал карточку" || return 1
    [ -f "$TEST_TMP/state/agent-flow-nightly-review.${REVIEW_DATE}.done" ] || {
        printf '  assert fail: F: sentinel не записан\n' >&2; return 1; }
    rc="$(run_nightly NIGHTLY_REVIEW_HOUR="$now_h" NIGHTLY_REVIEW_WINDOW_HOURS=1 COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "F: второй тик exit 0" || return 1
    assert_eq "0" "$(journal_creates)" "F: второй тик карточек не создаёт" || return 1
    assert_contains "уже создано" "$(cat "$STDERR_FILE")" "F: причина skip в логе" || return 1
}

# ---------------------------------------------------------------------------
# G. Деградация без gh: секции печатают «НЕТ ДАННЫХ», тик не падает.
# ---------------------------------------------------------------------------
test_G_no_gh_degradation() {
    reset_state
    local rc out
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "G: exit 0 без gh" || return 1
    out="$(cat "$STDOUT_FILE")"
    assert_contains "НЕТ ДАННЫХ (gh pr list недоступен" "$out" "G: PR-секция честно пустая" || return 1
    assert_contains "НЕТ ДАННЫХ (gh issue list недоступен)" "$out" "G: issues-секция" || return 1
    assert_contains "Коммитов в origin/develop: **2**" "$out" "G: git-секция посчитана" || return 1
}

# ---------------------------------------------------------------------------
# H. DRY_RUN: карточек нет, дайджест есть.
# ---------------------------------------------------------------------------
test_H_dry_run() {
    reset_state
    local rc
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true NIGHTLY_REVIEW_DRY_RUN=true COMPONENT_REVIEW_MAX=9)"
    assert_eq "0" "$rc" "H: exit 0" || return 1
    assert_eq "0" "$(journal_creates)" "H: ни одного kanban create" || return 1
    assert_contains "DRY-RUN" "$(cat "$STDERR_FILE")" "H: лог говорит DRY-RUN" || return 1
    assert_contains "Ночной ревью за" "$(cat "$STDOUT_FILE")" "H: дайджест на stdout" || return 1
    [ -f "$TEST_TMP/state/agent-flow-nightly-review.${REVIEW_DATE}.done" ] && {
        printf '  assert fail: H: dry-run не должен писать sentinel\n' >&2; return 1; }
    return 0
}

# ---------------------------------------------------------------------------
# I. EXCLUDE_RE: docs/ не попадает ни в таблицу компонентов, ни в карточки.
# ---------------------------------------------------------------------------
test_I_exclude_docs() {
    reset_state
    local rc out journal
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=9 COMPONENT_REVIEW_MIN_FILES=1)"
    assert_eq "0" "$rc" "I: exit 0" || return 1
    out="$(cat "$STDOUT_FILE")"
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_not_contains "| \`docs/adr\` |" "$out" "I: docs/adr не в таблице компонентов" || return 1
    assert_not_contains "ревью компонента: docs/adr" "$journal" "I: карточки на docs нет" || return 1
    assert_contains "| \`src/rob_box_voice\` |" "$out" "I: кодовый компонент в таблице" || return 1
}

run_test "A: вне ночного окна → skip"                     test_A_outside_window
run_test "B: ночная карточка (key + assignee)"            test_B_nightly_card
run_test "C: компонентные карточки, cap соблюдается"      test_C_component_cards
run_test "D: порог MIN_FILES"                             test_D_min_files
run_test "E: кулдаун по маркеру"                          test_E_cooldown
run_test "F: sentinel — второй тик за ночь"               test_F_sentinel
run_test "G: деградация без gh"                           test_G_no_gh_degradation
run_test "H: dry-run"                                     test_H_dry_run
run_test "I: EXCLUDE_RE (docs/)"                          test_I_exclude_docs

printf '\n[==========] %d tests, %d passed, %d failed\n' \
    "$TESTS_TOTAL" "$TESTS_PASSED" "$TESTS_FAILED"
if [ "$TESTS_FAILED" -ne 0 ]; then
    printf '[  FAILED  ] %s\n' "${FAILED_NAMES[@]}"
    printf 'artifacts: %s\n' "$TEST_TMP"
    exit 1
fi
rm -rf "$TEST_TMP"
exit 0
