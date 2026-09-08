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
    local rc journal iso_week
    iso_week="$(date -d "$REVIEW_DATE" +%G-W%V 2>/dev/null || date +%Y-W%V)"
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "B: exit 0" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "🌙 ночной ревью ${REVIEW_DATE}" "$journal" "B: заголовок карточки" || return 1
    # ADR-0049 follow-up: ключ = ISO-неделя (не голая дата — issue #2159).
    assert_contains "retro:nightly-review-${iso_week}" "$journal" "B: idempotency-key=ISO-неделя" || return 1
    assert_contains "--assignee architect" "$journal" "B: assignee=architect" || return 1
    assert_eq "1" "$(journal_creates)" "B: ровно одна карточка (COMPONENT_REVIEW_MAX=0)" || return 1
}

# ---------------------------------------------------------------------------
# C. Компонентные карточки: top-N по churn, cap соблюдается.
# ---------------------------------------------------------------------------
test_C_component_cards() {
    reset_state
    local rc journal iso_week
    iso_week="$(date -d "$REVIEW_DATE" +%G-W%V 2>/dev/null || date +%Y-W%V)"
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=1)"
    assert_eq "0" "$rc" "C: exit 0" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_eq "2" "$(journal_creates)" "C: nightly + ровно одна компонентная (cap=1)" || return 1
    assert_contains "ревью компонента: src/rob_box_voice" "$journal" "C: top-1 по churn = voice" || return 1
    assert_contains "component-review-src-rob_box_voice-${iso_week}" "$journal" "C: ключ компонента = ISO-неделя" || return 1
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

# ---------------------------------------------------------------------------
# J. ADR-0049 follow-up: dedup-ключ БЕЗ даты → ISO-неделя (фикс #2159).
#
#    До: ключ содержал `nightly-review-2026-09-08`. Две параллельные тики
#    в одном окне гонки (component-review-*-2026-09-08 vs nightly-review-2026-09-08)
#    имели РАЗНЫЕ idempotency-keys → слой 2 молчал.
#
#    После: ключ включает ISO-неделю `nightly-review-<YYYY-WW>`. Внутри одной
#    недели повторный тик → тот же key → idempotency-key срабатывает.
# ---------------------------------------------------------------------------
test_J_iso_week_key() {
    reset_state
    local rc journal
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "J: exit 0" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    # ISO-week: %G-%V → 2026-W36 на 2026-09-08 (вторник ISO-недели 36).
    local iso_week
    iso_week="$(date -d "$REVIEW_DATE" +%G-W%V 2>/dev/null || date +%Y-W%V)"
    assert_contains "retro:nightly-review-${iso_week}" "$journal" "J: idempotency-key содержит ISO-неделю, не дату" || return 1
    assert_not_contains "retro:nightly-review-${REVIEW_DATE}" "$journal" "J: ключ НЕ содержит голую дату" || return 1
}

# ---------------------------------------------------------------------------
# K. outcome=no-real-defect → карточка НЕ создаётся, JSONL записан.
#
#    Воркер-ревьюер имеет право сказать «находок нет» (контракт §3.2 ADR-0049).
#    Сейчас механический скрипт ВСЕГДА создаёт карточку; это впустую жжёт
#    токены и плодит архив. После follow-up: карточка создаётся только при
#    outcome=open-issue-N или outcome=duplicate-suppressed-с-новыми-находками.
# ---------------------------------------------------------------------------
test_K_no_real_defect_no_card() {
    reset_state
    local rc journal jsonl_file
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true NIGHTLY_REVIEW_OUTCOME=no-real-defect \
        NIGHTLY_REVIEW_JSONL="$TEST_TMP/state/nightly-review.jsonl" \
        COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "K: exit 0 при no-real-defect" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_eq "0" "$(journal_creates)" "K: kanban-карточка НЕ создана (no-real-defect)" || return 1
    # Sentinel всё равно пишется (одна ночь = один тик).
    [ -f "$TEST_TMP/state/agent-flow-nightly-review.${REVIEW_DATE}.done" ] || {
        printf '  assert fail: K: sentinel записан даже без карточки\n' >&2; return 1; }
}

# ---------------------------------------------------------------------------
# L. JSONL append-only: каждая строка валидна, формат стабильный.
#
#    Файл <docs-root>/reports/nightly-review/<YYYY-MM-DD>.jsonl
#    Каждая строка — JSON: ts, task_id, component, files_changed,
#    findings[{type, severity, fingerprint, raw}], outcome.
# ---------------------------------------------------------------------------
test_L_jsonl_valid() {
    reset_state
    local rc jsonl_file
    jsonl_file="$TEST_TMP/state/nightly-review.jsonl"
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true \
        NIGHTLY_REVIEW_OUTCOME=open-issue-9999 \
        NIGHTLY_REVIEW_JSONL="$jsonl_file" \
        COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "L: exit 0" || return 1
    [ -f "$jsonl_file" ] || { printf '  assert fail: L: JSONL не создан\n' >&2; return 1; }
    # Каждая строка — валидный JSON с обязательными полями.
    python3 - "$jsonl_file" <<'PY'
import json, sys
path = sys.argv[1]
required = {"ts", "task_id", "component", "files_changed", "findings", "outcome"}
with open(path) as f:
    for i, line in enumerate(f, 1):
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except Exception as e:
            print(f"  L: строка {i} не JSON: {e}"); sys.exit(1)
        missing = required - set(rec.keys())
        if missing:
            print(f"  L: строка {i} без полей {missing}"); sys.exit(1)
PY
    local py_rc=$?
    assert_eq "0" "$py_rc" "L: каждая строка JSONL валидна и имеет обязательные поля" || return 1
}

# ---------------------------------------------------------------------------
# M. Fingerprint dedup: находка с тем же fingerprint за ту же неделю
#     → подавляется (outcome=duplicate-suppressed), карточка НЕ создаётся.
# ---------------------------------------------------------------------------
test_M_fingerprint_dedup() {
    reset_state
    local rc journal fingerprint
    fingerprint="a3f4b9c0d1e2-test-dedup"
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true \
        NIGHTLY_REVIEW_OUTCOME="duplicate-suppressed:${fingerprint}" \
        COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "M: exit 0 при duplicate-suppressed" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_eq "0" "$(journal_creates)" "M: kanban-карточка НЕ создана (dedup)" || return 1
    assert_contains "duplicate-suppressed" "$(cat "$STDERR_FILE")" "M: причина в логе" || return 1
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
run_test "J: dedup-ключ = ISO-неделя (фикс #2159)"        test_J_iso_week_key
run_test "K: outcome=no-real-defect → нет kanban-карточки" test_K_no_real_defect_no_card
run_test "L: JSONL append-only валиден"                   test_L_jsonl_valid
run_test "M: outcome=duplicate-suppressed → нет kanban-карточки" test_M_fingerprint_dedup

printf '\n[==========] %d tests, %d passed, %d failed\n' \
    "$TESTS_TOTAL" "$TESTS_PASSED" "$TESTS_FAILED"
if [ "$TESTS_FAILED" -ne 0 ]; then
    printf '[  FAILED  ] %s\n' "${FAILED_NAMES[@]}"
    printf 'artifacts: %s\n' "$TEST_TMP"
    exit 1
fi
rm -rf "$TEST_TMP"
exit 0
