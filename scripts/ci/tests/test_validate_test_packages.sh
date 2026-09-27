#!/usr/bin/env bash
# Regression test for scripts/ci/validate_test_packages.py (issue #2232).
#
# This test asserts that the validator:
#   1. Returns 0 on the current G-Run Tests.yml (post-fix).
#   2. Returns 1 when a known package is removed from --packages-up-to.
#   3. Returns 1 when a known package is missing its pytest/colcon header.
#   4. Reports the offending package name in stderr (so CI logs surface
#      what regressed).
#
# Run:
#   bash scripts/ci/tests/test_validate_test_packages.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts/ci/tests/test_*.sh lives two levels under repo root.
# Inside a worktree, ``../..`` may land on the main checkout rather than
# the worktree itself — resolve via ``git rev-parse --show-toplevel``.
REPO_ROOT="$SCRIPT_DIR"
if command -v git >/dev/null 2>&1; then
    if wt_root="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"; then
        REPO_ROOT="$wt_root"
    else
        REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
    fi
fi
SCRIPT="$REPO_ROOT/scripts/ci/validate_test_packages.py"
WORKFLOW="$REPO_ROOT/.github/workflows/G-Run Tests.yml"

if [ ! -x "$SCRIPT" ]; then
    echo "❌ $SCRIPT not executable"; exit 1
fi

if [ ! -f "$WORKFLOW" ]; then
    echo "❌ $WORKFLOW missing"; exit 1
fi

BACKUP="$(mktemp)"
trap 'cp "$BACKUP" "$WORKFLOW" && rm -f "$BACKUP"' EXIT
cp "$WORKFLOW" "$BACKUP"

# Case 1: pre-fix state (remove rob_box_quest from --packages-up-to)
echo "--- Case 1: rob_box_quest missing from --packages-up-to ---"
sed -i 's| rob_box_quest| |g' "$WORKFLOW"
set +e
"$SCRIPT" >/dev/null 2>err.txt
rc=$?
set -e
if [ "$rc" -eq 0 ]; then
    echo "❌ expected non-zero exit, got 0"; cat err.txt; exit 1
fi
if ! grep -q "rob_box_quest" err.txt; then
    echo "❌ stderr doesn't mention rob_box_quest"; cat err.txt; exit 1
fi
echo "  ✅ returned non-zero + mentions rob_box_quest"

cp "$BACKUP" "$WORKFLOW"

# Case 2: pre-fix state (remove === pytest: rob_box_core === header)
echo "--- Case 2: rob_box_core missing pytest header ---"
sed -i '/=== pytest: rob_box_core/d' "$WORKFLOW"
set +e
"$SCRIPT" >/dev/null 2>err.txt
rc=$?
set -e
if [ "$rc" -eq 0 ]; then
    echo "❌ expected non-zero exit, got 0"; cat err.txt; exit 1
fi
if ! grep -q "rob_box_core" err.txt; then
    echo "❌ stderr doesn't mention rob_box_core"; cat err.txt; exit 1
fi
echo "  ✅ returned non-zero + mentions rob_box_core"

cp "$BACKUP" "$WORKFLOW"

# Case 3: current (post-fix) state must pass
echo "--- Case 3: post-fix state must pass ---"
"$SCRIPT" >/dev/null
echo "  ✅ exit=0 on current workflow"

# Case 4: only-out-of-scope-package is not flagged
# (we keep our EXPECTED set conservative, so harness/llm/mcp_tools are
# informational only and don't fail the script).
echo "--- Case 4: out-of-scope packages don't fail the script ---"
"$SCRIPT" 2>&1 | grep -E "out of #2232 scope" >/dev/null \
    || { echo "❌ informational line missing"; exit 1; }
echo "  ✅ informational line present"

echo
echo "✅ validate_test_packages.py regression test PASSED"