#!/bin/bash
# Re-pin editable finder mappings before each test run, since parallel
# agents on the board keep re-installing their own rob_box_core.
set -e

WORKTREE="$(cd "$(dirname "$0")/.." && pwd)"
VENV_PKG_DIR="/home/builder/.hermes/hermes-agent/venv/lib/python3.11/site-packages"

# Patch editable finder mappings for packages AV-21 depends on.
patch_finder() {
    local pkg="$1"
    local worktree_subdir="$2"
    local mapping_file="$VENV_PKG_DIR/__editable___${pkg//./_}_0_1_0_finder.py"
    [ -f "$mapping_file" ] || return 0
    sed -i "s|'${pkg}': '[^']*'|'${pkg}': '${WORKTREE}/${worktree_subdir}'|" "$mapping_file"
}

patch_finder rob_box_core src/rob_box_core/rob_box_core
patch_finder rob_box_harness src/rob_box_harness/rob_box_harness
patch_finder rob_box_llm src/rob_box_llm/rob_box_llm
patch_finder rob_box_mcp_tools src/rob_box_mcp_tools/rob_box_mcp_tools

# Reinstall rob_box_core from this worktree (also sets up say/play_animation
# from this branch's catalog). Use --no-deps for speed.
pip install --no-deps -e "$WORKTREE/src/rob_box_core/" >/dev/null 2>&1 || true

# Run pytest.
python -m pytest "$@"
