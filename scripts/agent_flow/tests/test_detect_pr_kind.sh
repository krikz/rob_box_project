#!/usr/bin/env bash
# ============================================================================
# test_detect_pr_kind.sh — ретро 24.08 t_388bb652
#
# Регресс-гард: detect_pr_kind в agent-flow-{e2e-process,merge-gate}.sh
# корректно классифицирует:
#   - docs(adr-XXXX) / docs(architecture) → lint (ADR-черновики, docs-only)
#   - wip(arch ...) / wip(infra ...) / wip(voice-core ...) → lint (WIP-черновики)
#
# До этого фикса PR #1577/#1559/#1580/#1581/#1578 (открытые на 24.08 02:23 CEST)
# шли в e2e-очередь как functional и залипали с e2e:rejected (cold-start
# wake-gate no_wake_word из t_d9e70587), потому что detect_pr_kind
# не распознавал docs/wip префиксы conventional-commit.
#
# Run:
#   bash scripts/agent_flow/tests/test_detect_pr_kind.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_LIB_DIR/.." && pwd)"  # scripts/agent_flow/, где лежат *.sh

# Color codes (если stdout — терминал)
if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; BLU=$'\033[34m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; BLU=''; END=''
fi

TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

# ---- Извлечение detect_pr_kind из реальных production-скриптов -------------
# Чтобы тест не зависел от copy-paste: парсим функцию из исходников через awk
# и eval'им в текущий scope. Если в будущем кто-то поправит detect_pr_kind
# в одном файле и забудет про второй — этот тест сразу скажет.
extract_func() {  # $1=script_path $2=func_signature $3=out_var
    local script="$1" sig="$2" outvar="$3" body
    # Сигнатура: has_label() или detect_pr_kind() с опциональным комментарием
    body="$(awk -v sig="$sig" '
        $0 ~ "^" sig "[[:space:]]*\\(\\)" {flag=1}
        flag {print}
        flag && /^}/ {flag=0; exit}
    ' "$script")"
    if [ -z "$body" ]; then
        printf 'FAIL: func %s не найдена в %s\n' "$sig" "$script" >&2
        return 1
    fi
    printf -v "$outvar" '%s' "$body"
}

# После дедупа 30.08 detect_pr_kind и has_label живут в ОДНОМ месте —
# lib_agent_flow_common.sh, который сорсят и e2e-process, и merge-gate.
# Расходиться копиям больше негде, поэтому тест:
#   1) проверяет, что ни один из двух скриптов не завёл локальную копию
#      обратно (иначе drift, от которого этот гард и написан, возвращается);
#   2) берёт единственную реализацию из библиотеки и вешает на неё оба имени
#      (detect_pr_kind_e2e / detect_pr_kind_mg) — кейсы ниже не менялись.
LIB_COMMON="$REPO_ROOT/lib_agent_flow_common.sh"
[ -f "$LIB_COMMON" ] || { printf 'FAIL: нет %s\n' "$LIB_COMMON" >&2; exit 1; }

for _script in agent-flow-e2e-process.sh agent-flow-merge-gate.sh; do
    if grep -qE '^detect_pr_kind\(\)' "$REPO_ROOT/$_script"; then
        printf 'FAIL: %s снова держит СВОЮ копию detect_pr_kind (дедуп 30.08 откатан)\n' \
            "$_script" >&2
        exit 1
    fi
    if ! grep -q 'lib_agent_flow_common\.sh' "$REPO_ROOT/$_script"; then
        printf 'FAIL: %s не сорсит lib_agent_flow_common.sh\n' "$_script" >&2
        exit 1
    fi
done

extract_func "$LIB_COMMON" "detect_pr_kind" LIB_BODY
extract_func "$LIB_COMMON" "has_label"      LIB_HAS_LABEL

# Подставь NO_E2E_LABEL — detect_pr_kind обращается к глобальной переменной.
# shellcheck disable=SC2034
NO_E2E_LABEL="no-e2e-required"

eval "$LIB_HAS_LABEL" >/dev/null  # определяет has_label в текущем scope
unset -f detect_pr_kind detect_pr_kind_e2e detect_pr_kind_mg 2>/dev/null || true
eval "$(printf '%s' "$LIB_BODY" | sed 's/^detect_pr_kind()/detect_pr_kind_e2e()/')" >/dev/null
eval "$(printf '%s' "$LIB_BODY" | sed 's/^detect_pr_kind()/detect_pr_kind_mg()/')" >/dev/null

# ---- Test runner ----------------------------------------------------------
run_test() {
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    printf '%s[ RUN     ]%s %s\n' "$BLU" "$END" "$name"
    if "$fn"; then
        TESTS_PASSED=$((TESTS_PASSED+1))
        printf '%s[   PASS  ]%s %s\n' "$GRN" "$END" "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED+1))
        FAILED_NAMES+=("$name")
        printf '%s[   FAIL  ]%s %s\n' "$RED" "$END" "$name"
    fi
}

assert_kind() {  # $1=impl $2=title $3=want $4=label (опц.)
    local impl="$1" title="$2" want="$3" label="${4:-}" got fn
    fn="detect_pr_kind_${impl}"
    got=$(${fn} "$label" "$title")
    if [ "$got" != "$want" ]; then
        printf '    %sassert fail:%s impl=%s title=%q want=%s got=%s\n' \
            "$RED" "$END" "$impl" "$title" "$want" "$got" >&2
        return 1
    fi
}

# ============================================================================
# POSITIVE: новые префиксы → lint (оба варианта реализации)
# ============================================================================
test_docs_adr_lint() {
    assert_kind e2e "docs(adr-0027): systemic wake-gate no_wake_word blocker" lint
    assert_kind mg  "docs(adr-0027): systemic wake-gate no_wake_word blocker" lint
}

test_docs_architecture_lint() {
    assert_kind e2e "docs(architecture): meta-quest WebXR layout" lint
    assert_kind mg  "docs(architecture): meta-quest WebXR layout" lint
}

test_wip_arch_lint() {
    assert_kind e2e "wip(arch #1506 t_228de99c): verdict v3" lint
    assert_kind mg  "wip(arch #1506 t_228de99c): verdict v3" lint
}

test_wip_infra_lint() {
    assert_kind e2e "wip(infra): networking runbook update"    lint
    assert_kind mg  "wip(infra): networking runbook update"    lint
}

test_wip_voice_core_lint() {
    assert_kind e2e "wip(voice-core): verification suite for e2e_routes" lint
    assert_kind mg  "wip(voice-core): verification suite for e2e_routes" lint
}

# ============================================================================
# POSITIVE: регрессия — ранее существовавшие lint-префиксы остались lint
# ============================================================================
test_bracket_tags_lint() {
    assert_kind e2e "[lint] foo"      lint
    assert_kind mg  "[lint] foo"      lint
    assert_kind e2e "[refactor] bar"  lint
    assert_kind mg  "[refactor] bar"  lint
}

test_fix_agent_flow_lint() {
    assert_kind e2e "fix(agent-flow e2e): python-пайпы неубиваемые" lint
    assert_kind mg  "fix(agent-flow e2e): python-пайпы неубиваемые" lint
    assert_kind e2e "fix(agent_flow merge): ret"                  lint
    assert_kind mg  "fix(agent_flow merge): ret"                  lint
}

test_no_e2e_label_lint() {
    assert_kind e2e "feat(voice-core): что-то" lint "no-e2e-required,hermes"
    assert_kind mg  "feat(voice-core): что-то" lint "no-e2e-required,hermes"
}

# ============================================================================
# NEGATIVE: feat/fix(voice)/etc остаются functional (e2e обязательно)
# ============================================================================
test_feat_functional() {
    assert_kind e2e "feat(voice-core): new wake word detection" functional
    assert_kind mg  "feat(voice-core): new wake word detection" functional
}

test_fix_voice_functional() {
    assert_kind e2e "fix(voice): STT pipeline race" functional
    assert_kind mg  "fix(voice): STT pipeline race" functional
}

# Граничный кейс: "doc(...)" (без 's') — НЕ должен матчиться.
test_doc_no_s_functional() {
    assert_kind e2e "doc(api): endpoint typo" functional
    assert_kind mg  "doc(api): endpoint typo" functional
}

# Граничный кейс: "wip(nonexistent)" — НЕ должен матчиться.
test_wip_unknown_functional() {
    assert_kind e2e "wip(marketing): campaign draft"  functional
    assert_kind mg  "wip(marketing): campaign draft"  functional
}

# ============================================================================
# Case-sensitivity: обе реализации case-insensitive (title и labels
# нормализуются в lowercase внутри detect_pr_kind). Выравнено ревью 28.08 —
# раньше merge-gate был case-sensitive и расходился с e2e-process.
# ============================================================================
test_e2e_process_uppercase_lint() {
    assert_kind e2e "DOCS(ADR-0027): SOMETHING" lint
    assert_kind e2e "WIP(Arch #1506): verdict"  lint
}

test_mg_uppercase_lint() {
    assert_kind mg "DOCS(ADR-0027): SOMETHING" lint
    assert_kind mg "WIP(Arch #1506): verdict"  lint
}

# ============================================================================
# Real-world кейсы из задачи: PR #1577/#1559/#1580/#1581/#1578.
# Это «живые» title, которые сейчас залипли с e2e:rejected.
# ============================================================================
test_real_pr_1577_lint() {
    # PR #1577 (на 24.08 02:23 CEST): docs(adr-0027): systemic wake-gate
    assert_kind e2e "docs(adr-0027): systemic wake-gate no_wake_word blocker" lint
    assert_kind mg  "docs(adr-0027): systemic wake-gate no_wake_word blocker" lint
}

test_real_pr_1559_lint() {
    # PR #1559 (на 24.08 02:23 CEST): wip(arch #1506 t_228de99c): verdict v3
    assert_kind e2e "wip(arch #1506 t_228de99c): verdict v3" lint
    assert_kind mg  "wip(arch #1506 t_228de99c): verdict v3" lint
}

# ============================================================================
# Run all tests
# ============================================================================
run_test "POS: docs(adr-XXXX) → lint"           test_docs_adr_lint
run_test "POS: docs(architecture) → lint"       test_docs_architecture_lint
run_test "POS: wip(arch ...) → lint"            test_wip_arch_lint
run_test "POS: wip(infra) → lint"               test_wip_infra_lint
run_test "POS: wip(voice-core) → lint"          test_wip_voice_core_lint
run_test "REG: [lint]/[refactor] tags → lint"   test_bracket_tags_lint
run_test "REG: fix(agent-flow) → lint"          test_fix_agent_flow_lint
run_test "REG: no-e2e-required label → lint"    test_no_e2e_label_lint
run_test "NEG: feat(voice) → functional"        test_feat_functional
run_test "NEG: fix(voice) → functional"         test_fix_voice_functional
run_test "NEG: doc(singular) → functional"      test_doc_no_s_functional
run_test "NEG: wip(unknown) → functional"       test_wip_unknown_functional
run_test "CASE: e2e-process uppercase → lint"    test_e2e_process_uppercase_lint
run_test "CASE: merge-gate uppercase → lint"     test_mg_uppercase_lint
run_test "REAL: PR #1577 (docs/adr-0027)"       test_real_pr_1577_lint
run_test "REAL: PR #1559 (wip/arch verdict)"    test_real_pr_1559_lint

printf '\n%s==== Summary ====%s\n' "$YEL" "$END"
printf 'total:  %d\n' $TESTS_TOTAL
printf '%spassed: %d%s\n' "$GRN" "$TESTS_PASSED" "$END"
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf '%sfailed: %d%s\n' "$RED" "$TESTS_FAILED" "$END"
    printf 'failures:\n'
    for n in "${FAILED_NAMES[@]}"; do printf '  - %s\n' "$n"; done
    exit 1
fi
exit 0
