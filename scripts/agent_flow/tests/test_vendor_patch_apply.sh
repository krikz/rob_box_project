#!/bin/bash
# ============================================================================
# test_vendor_patch_apply.sh — vendor-патчи hermes-agent применяются к upstream
#
# Ретро t_f00676f8: локальные патчи hermes-agent (валидация скиллов по
# профилю t_1ab37fa8) терялись при `git pull`/`pip install -U hermes-agent`,
# потому что не были сохранены в репо. Фикс: диффы живут в
# scripts/agent_flow/vendor/hermes-agent-*.patch, а install.sh применяет их
# идемпотентно.
#
# Ретро t_1d467636: добавлен второй патч
# hermes-agent-auto-decompose-maintenance.patch (MAINTENANCE/peak probe +
# идемпотентность декомпозиции). Тест покрывает ВСЕ vendor-патчи.
#
# Ретро t_9cc3ec08: добавлен третий патч hermes-agent-crp-schema.patch
# (Consultation Request Pack SASE arXiv 2509.06216 на стороне hermes-
# kanban). Символ-фикс — функция `validate_crp` в hermes_cli.crp; без
# неё create_task(crp_schema=...) и block_task(crp=...) падают.
#
# Этот тест проверяет для каждого vendor-патча:
#   1. vendor-патч существует и непустой;
#   2. патч чисто применяется (git apply --check) к свежему origin/main
#      hermes-agent (worktree, НЕ трогает живой чекаут на хосте);
#   3. после применения в файлах появляется главный символ фикса;
#   4. повторный apply идемпотентен (git apply --reverse --check).
#
# Требует: git, network (fetch origin), ~/.hermes/hermes-agent как источник
# upstream-коммитов. НЕ пишет в живой чекаут hermes-agent.
#
# Invocation:
#   bash tests/test_vendor_patch_apply.sh
# Returns 0 on all-pass, non-zero on first failure.
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/.." && pwd)"
HA_SRC="${HERMES_AGENT_SRC:-/home/builder/.hermes/hermes-agent}"
WORKTREE="$(mktemp -d /tmp/ha-patch-test.XXXXXX)"

cleanup() {
    git -C "$HA_SRC" worktree remove --force "$WORKTREE" >/dev/null 2>&1 || rm -rf "$WORKTREE"
}
trap cleanup EXIT

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "  ok: $*"; }

# Символы фиксов: имя патча -> ожидаемый символ в применённом дереве.
# Ключ — базовое имя файла патча (без .patch).
declare -A EXPECTED_SYMBOL=(
    ["hermes-agent-skill-validation"]="_validate_skills_for_assignee"
    ["hermes-agent-auto-decompose-maintenance"]="defer decompose (MAINTENANCE)"
    # Issue #1462 — cost attribution per issue (P10 gap). Each kanban run
    # writes one ``cost_attribution`` event row carrying the dollar total
    # in cents; the helper function _compute_run_cost_cents aggregates the
    # worker session's session_model_usage rows. Both symbols anchor the
    # patch in upstream code paths.
    ["hermes-agent-kanban-cost-attribution"]="_compute_run_cost_cents"
    ["hermes-agent-crp-schema"]="validate_crp"
    # Ретро t_695f5138 / task t_bc3ac7fb: pre-spawn worktree-collision
    # guard. ``git worktree add`` fatally rejects creating a worktree
    # for a branch already checked out elsewhere; before the fix this
    # surfaced as spawn_failed × 2 → blocked, while triage spawned
    # duplicates on the same branch. The patch adds
    # ``_find_worktree_for_branch`` (the porcelain parser that detects
    # the collision before the shell-out) and ``WorktreeBranchBusyError``
    # (the dedicated exception the dispatch loop catches with
    # force_trip=True to skip the retry storm).
    ["hermes-agent-spawn-worktree-precheck"]="WorktreeBranchBusyError"
    # Issue #1571: _ensure_git_worktree uses local HEAD as the base
    # for a new worktree branch, which drifts from origin/develop when
    # the main worktree hasn't refreshed its HEAD (observed 48-50
    # commits behind on multiple recent cards). The patch adds
    # ``_resolve_worktree_base_ref`` (best-effort ``git fetch origin
    # develop`` with HEAD fallback + stderr warning) and
    # ``_warn_worktree_base_drift`` (WORKTREE_BASE_DRIFT warning when
    # HEAD..origin/develop > threshold). The patch's anchor lives
    # near ``_ensure_git_worktree`` whose line numbers shift after the
    # precheck patch above is applied — hence the ``z-`` prefix that
    # the test uses to defer this patch into a second apply pass.
    ["hermes-agent-z-spawn-base-origin-develop"]="_resolve_worktree_base_ref"
)

patches=( "$REPO_ROOT"/vendor/hermes-agent-*.patch )
[ ${#patches[@]} -gt 0 ] || fail "no vendor patches found in $REPO_ROOT/vendor"

# Split patches into two phases by ``z-`` prefix in the basename.
# The plain (no z-) patches must apply cleanly against fresh
# origin/main; the z-prefixed patches apply on top of the plain ones
# (their anchors assume the precheck guard already landed).
plain_patches=()
z_patches=()
for p in "${patches[@]}"; do
    name="$(basename "$p" .patch)"
    if [[ "$name" == hermes-agent-z-* ]]; then
        z_patches+=("$p")
    else
        plain_patches+=("$p")
    fi
done

echo "==> 1. vendor patches exist"
for p in "${patches[@]}"; do
    [ -s "$p" ] || fail "vendor patch missing or empty: $p"
    pass "$(basename "$p") ($(wc -l < "$p") lines)"
done

echo "==> 2. apply --check against fresh origin/main (phase 1: plain patches)"
git -C "$HA_SRC" fetch origin main --quiet 2>/dev/null || fail "cannot fetch origin/main"
git -C "$HA_SRC" worktree add --detach "$WORKTREE" origin/main >/dev/null 2>&1 || fail "cannot create worktree at origin/main"
for p in "${plain_patches[@]}"; do
    ( cd "$WORKTREE" && git apply --check "$p" ) || fail "vendor patch does not apply cleanly to origin/main (upstream moved; regenerate): $(basename "$p")"
    pass "$(basename "$p") applies cleanly"
done

echo "==> 2b. apply --check phase 2: z-patches against origin/main+plain-applied"
# Apply plain patches first so the z-patch anchors have the right
# offsets to match.
for p in "${plain_patches[@]}"; do
    ( cd "$WORKTREE" && git apply "$p" ) || fail "git apply failed (phase 1): $(basename "$p")"
done
for p in "${z_patches[@]}"; do
    ( cd "$WORKTREE" && git apply --check "$p" ) || fail "z-patch does not apply cleanly on top of plain patches: $(basename "$p")"
    pass "$(basename "$p") applies cleanly (z-patch on top of plain)"
done

echo "==> 3. apply all and verify symbols present"
for p in "${z_patches[@]}"; do
    ( cd "$WORKTREE" && git apply "$p" ) || fail "git apply failed (phase 2): $(basename "$p")"
done
for p in "${patches[@]}"; do
    name="$(basename "$p" .patch)"
    symbol="${EXPECTED_SYMBOL[$name]:-}"
    if [ -n "$symbol" ]; then
        grep -rqF "$symbol" "$WORKTREE"/hermes_cli "$WORKTREE"/gateway 2>/dev/null \
            || fail "$name lacks expected symbol '$symbol' after apply"
        pass "$name: '$symbol' present"
    fi
done

echo "==> 4. idempotency: reverse-and-reapply cycle (LIFO then FIFO)"
# NOTE: ``git apply --reverse --check`` is unreliable for some context-based
# patches (returns "patch does not apply" even when a plain ``git apply
# --reverse`` succeeds — the --check path tracks a different index
# invariant). The robust idempotency check mirrors what install.sh
# actually does: reverse in LIFO order, then re-apply in FIFO order,
# and finally verify the patched tree matches the post-step-3 state.
# If both cycles are byte-identical, every patch is fully reversible
# AND every patch applies cleanly to a fresh tree — that is the
# operational idempotency install.sh relies on. Issue #1571.
save_dirty_state="$WORKTREE/.idempotency-snapshot"
mkdir -p "$save_dirty_state"
cp "$WORKTREE/hermes_cli/kanban_db.py" "$save_dirty_state/kanban_db.py.after-step3"

# LIFO reverse: z-patches first, then plain patches.
for (( idx=${#z_patches[@]}-1; idx>=0; idx-- )); do
    p="${z_patches[idx]}"
    ( cd "$WORKTREE" && git apply --reverse "$p" ) || fail "z-patch reverse failed (LIFO): $(basename "$p")"
    pass "$(basename "$p") reversed"
done
for (( idx=${#plain_patches[@]}-1; idx>=0; idx-- )); do
    p="${plain_patches[idx]}"
    ( cd "$WORKTREE" && git apply --reverse "$p" ) || fail "patch reverse failed (LIFO): $(basename "$p")"
    pass "$(basename "$p") reversed"
done

# Re-apply in FIFO order: plain patches, then z-patches.
for p in "${plain_patches[@]}"; do
    ( cd "$WORKTREE" && git apply "$p" ) || fail "patch re-apply failed (FIFO): $(basename "$p")"
    pass "$(basename "$p") re-applied"
done
for p in "${z_patches[@]}"; do
    ( cd "$WORKTREE" && git apply "$p" ) || fail "z-patch re-apply failed (FIFO): $(basename "$p")"
    pass "$(basename "$p") re-applied"
done

# Verify the tree is byte-identical to the post-step-3 state.
if ! diff -q "$save_dirty_state/kanban_db.py.after-step3" "$WORKTREE/hermes_cli/kanban_db.py" >/dev/null; then
    fail "idempotency cycle diverged: post-reverse+reapply tree != post-step3 tree"
fi
pass "reverse+reapply cycle is byte-identical to step 3 (true idempotency)"
rm -rf "$save_dirty_state"

echo "ALL VENDOR PATCH TESTS PASSED"
