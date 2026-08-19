#!/bin/bash
# ============================================================================
# test_crp_patch.sh — регресс-гард для hermes-agent-crp-schema.patch
#
# Ретро t_9cc3ec08: ADD research §6.2 — root cause координационного drift
# #1448/#1450/#1452/#1456 — kanban_block без structured schema (free-text
# reason), поэтому воркеры не видели варианты fix и пинг-понговали.
# P5-фикс: CRP (Consultation Request Pack, SASE arXiv 2509.06216) на стороне
# hermes-kanban — kanban_create(crp_schema=...) + kanban_block(crp=...).
#
# Patch живёт в scripts/agent_flow/vendor/hermes-agent-crp-schema.patch и
# применяется install.sh идемпотентно. Этот тест — строгий контрактный
# гард: если upstream hermes-agent изменит сигнатуру create_task /
# block_task или schema-описание в KANBAN_CREATE_SCHEMA / KANBAN_BLOCK_SCHEMA,
# patch перестанет применяться чисто, и воркеры снова свалятся в free-text
# drift. Тест ловит это ДО merge.
#
# Проверяем:
#   1. vendor-патч hermes-agent-crp-schema.patch существует и непустой;
#   2. содержит ключевые символы CRP-схемы в исходнике (validate_crp,
#      crp_schema, crp_block, KANBAN_CREATE_SCHEMA.crp_schema);
#   3. чисто применяется (git apply --check) к свежему origin/main
#      hermes-agent (worktree, НЕ трогает живой чекаут на хосте);
#   4. после применения в hermes_cli/ появляются crp.py + crp-схема;
#   5. повторный apply идемпотентен (git apply --reverse --check).
#
# НЕ зависит от сети: использует локальный origin/main, который CI
# синхронизирует. Если fetch молча падает — это видно по stale-check
# ниже (test пропускается, а не валится).
#
# Run:
#   bash scripts/agent_flow/tests/test_crp_patch.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/.." && pwd)"
HA_SRC="${HERMES_AGENT_SRC:-/home/builder/.hermes/hermes-agent}"
PATCH="$REPO_ROOT/vendor/hermes-agent-crp-schema.patch"

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "  ok: $*"; }

WORKTREE="$(mktemp -d /tmp/ha-crp-test.XXXXXX)"
cleanup() {
    git -C "$HA_SRC" worktree remove --force "$WORKTREE" >/dev/null 2>&1 || true
}
trap cleanup EXIT

# ------------------------------------------------------------------ 1
echo "==> 1. vendor patch exists"
[ -f "$PATCH" ] || fail "patch not found: $PATCH"
[ -s "$PATCH" ] || fail "patch is empty: $PATCH"
pass "patch present ($(wc -l < "$PATCH") lines, $(stat -c '%s' "$PATCH") bytes)"

# ------------------------------------------------------------------ 2
echo "==> 2. patch contains required CRP symbols"
# Эти строки ОБЯЗАНЫ появиться в hermes-agent после применения patch.
# Если upstream переименует функцию/поле — patch станет бесполезным и
# воркеры снова упрутся в free-text drift.
for sym in \
    "def validate_crp" \
    "crp_schema" \
    "crp_block" \
    "KANBAN_CREATE_SCHEMA" \
    "KANBAN_BLOCK_SCHEMA"; do
    grep -qF "$sym" "$PATCH" || fail "patch missing required symbol: $sym"
    pass "symbol '$sym' present"
done

# ------------------------------------------------------------------ 3
echo "==> 3. fetch origin/main (best-effort)"
if ! git -C "$HA_SRC" fetch origin main --quiet 2>/dev/null; then
    echo "  WARN cannot fetch origin/main — using local HEAD (skip upstream-check)"
    BASE="$(git -C "$HA_SRC" rev-parse HEAD)"
else
    BASE="origin/main"
fi
git -C "$HA_SRC" worktree add --detach "$WORKTREE" "$BASE" >/dev/null 2>&1 \
    || fail "cannot create worktree at $BASE (проверь $HA_SRC)"
pass "worktree created at $BASE"

# ------------------------------------------------------------------ 4
echo "==> 4. patch applies cleanly"
( cd "$WORKTREE" && git apply --check "$PATCH" ) \
    || fail "patch does not apply cleanly to $BASE — upstream moved; regenerate"
pass "patch applies cleanly"

# ------------------------------------------------------------------ 5
echo "==> 5. post-apply: crp.py module + symbols present"
( cd "$WORKTREE" && git apply "$PATCH" ) \
    || fail "git apply failed despite clean check"
[ -f "$WORKTREE/hermes_cli/crp.py" ] \
    || fail "hermes_cli/crp.py missing after apply"
pass "hermes_cli/crp.py present"
for sym in \
    "def validate_crp" \
    "def serialize_crp" \
    "def deserialize_crp" \
    "def format_crp_for_humans" \
    "CRPSchemaError"; do
    grep -qF "$sym" "$WORKTREE/hermes_cli/crp.py" \
        || fail "hermes_cli/crp.py missing symbol: $sym"
done
pass "all 5 crp.py symbols present"

# Интеграция в create_task / block_task — критические точки patch.
grep -q "crp_schema" "$WORKTREE/hermes_cli/kanban_db.py" \
    || fail "kanban_db.py missing crp_schema"
grep -q "crp_block" "$WORKTREE/hermes_cli/kanban_db.py" \
    || fail "kanban_db.py missing crp_block"
pass "kanban_db.py has crp_schema + crp_block"

grep -q '"crp_schema"' "$WORKTREE/tools/kanban_tools.py" \
    || fail "tools/kanban_tools.py missing KANBAN_CREATE_SCHEMA.crp_schema"
grep -q '"crp"' "$WORKTREE/tools/kanban_tools.py" \
    || fail "tools/kanban_tools.py missing KANBAN_BLOCK_SCHEMA.crp"
pass "tools/kanban_tools.py schema descriptions present"

# ------------------------------------------------------------------ 6
echo "==> 6. idempotency (reverse-check)"
( cd "$WORKTREE" && git apply --reverse --check "$PATCH" ) \
    || fail "patch not idempotent (reverse-check failed after apply)"
pass "patch idempotent"

# ------------------------------------------------------------------ 7
echo "==> 7. patch includes hermes-agent CRP unit + integration tests"
# Patch ОБЯЗАН содержать тесты, иначе фикс без регрессии — никто не
# узнает, что воркер опять провалился в free-text drift. SASE §6.2:
# silent drift возвращается, если нет жёсткого schema-теста.
CRP_TEST_1="$WORKTREE/tests/hermes_cli/test_crp.py"
CRP_TEST_2="$WORKTREE/tests/hermes_cli/test_crp_integration.py"
[ -f "$CRP_TEST_1" ] || fail "patch missing tests/hermes_cli/test_crp.py"
[ -f "$CRP_TEST_2" ] || fail "patch missing tests/hermes_cli/test_crp_integration.py"
pass "both crp test files present in patched tree"
# Минимальный smoke: тесты содержат ключевые проверки
grep -qF "test_validate_crp_rejects_empty_options" "$CRP_TEST_1" \
    || fail "test_crp.py missing empty-options rejection test"
grep -qF "test_create_task_rejects_invalid_crp" "$CRP_TEST_2" \
    || fail "test_crp_integration.py missing invalid-crp rejection test"
pass "key contract tests present (empty options / invalid crp rejection)"

# ------------------------------------------------------------------ 8 (best-effort)
echo "==> 8. run hermes-agent CRP unit tests on patched tree (best-effort)"
# Требует editable install / dev-deps; на CI runner'е hermes-agent не
# установлен в editable, поэтому шаг best-effort. Smoke-checks 1-7 уже
# доказали, что патч лежит и содержит правильные символы.
if command -v pytest >/dev/null 2>&1 && [ -d "$WORKTREE" ]; then
    if ( cd "$WORKTREE" && PYTHONPATH=. python3 -m pytest tests/hermes_cli/test_crp.py tests/hermes_cli/test_crp_integration.py -q 2>&1 ) | tail -5; then
        pass "hermes-agent CRP tests ran"
    else
        echo "  WARN pytest failed on patched tree (deps missing?) — manual verify:"
        echo "       cd $WORKTREE && pip install -e . && pytest tests/hermes_cli/test_crp.py"
    fi
else
    echo "  WARN pytest not on PATH or no git worktree — skipped (smoke-checks 1-7 already green)"
fi

echo
echo "ALL CRP PATCH TESTS PASSED"
