#!/usr/bin/env bash
# agent-flow-regen-vendor-patch.sh — regenerate a hermes-agent vendor patch
# from the LIVE working tree state at $HERMES_AGENT_DIR.
#
# Use when the original vendor patch in scripts/agent_flow/vendor/ stops
# applying (line offsets drifted because hermes-agent moved). Symptom in
# install.sh / sot-sync log:
#   ERROR patch does not apply cleanly to /home/builder/.hermes/hermes-agent
#     — upstream moved; regenerate vendor patch from current diff
#
# Strategy:
#   1. Snapshot LIVE hermes-agent working tree (the partially-applied state).
#   2. Apply the patch's "added" lines into a synthetic copy of the LIVE files
#      at the right insertion anchors:
#        - hermes_cli/kanban_db.py: insert _profile_skill_names +
#          _validate_skills_for_assignee between _canonical_assignee and
#          create_task, then insert call site after `skills_list = cleaned`.
#        - hermes_cli/profiles.py + tests/hermes_cli/test_kanban_db.py:
#          carry over as-is (hunks already present in live tree from previous
#          apply; live tree IS the "applied" state for these files).
#   3. git diff between LIVE and PATCHED → fresh patch with proper diff
#      headers. Applies cleanly to BOTH the live tree AND clean upstream main.
#
# Usage:
#   bash scripts/agent_flow/agent-flow-regen-vendor-patch.sh <patch-file>
#
# Example:
#   bash scripts/agent_flow/agent-flow-regen-vendor-patch.sh \
#     scripts/agent_flow/vendor/hermes-agent-skill-validation.patch
#
# Output:
#   - Writes <patch-file>.new next to the original
#   - Verifies the new patch applies cleanly (live + upstream main)
#   - Prints the diff and instructions to replace the patch
#
# This script is intentionally hermes-agent-agnostic — it works for any
# vendor patch that follows the skill-validation pattern (the only pattern
# we currently ship). If the patch touches different files or requires
# different insertion logic, extend the python helper below.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HERMES_AGENT_DIR="${HERMES_AGENT_DIR:-/home/builder/.hermes/hermes-agent}"

if [ $# -ne 1 ]; then
    echo "usage: $0 <patch-file>" >&2
    echo "  e.g. $0 scripts/agent_flow/vendor/hermes-agent-skill-validation.patch" >&2
    exit 2
fi

PATCH_IN="$1"
if [ ! -f "$PATCH_IN" ]; then
    echo "ERROR: patch file not found: $PATCH_IN" >&2
    exit 2
fi

if ! git -C "$HERMES_AGENT_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "ERROR: $HERMES_AGENT_DIR is not a git checkout; cannot snapshot" >&2
    exit 2
fi

# Detect which files the patch touches. We support the canonical 3-file set.
PATCH_FILES=$(grep -E '^diff --git ' "$PATCH_IN" | sed -E 's|^diff --git a/([^ ]+) b/.*|\1|' | sort -u)
echo "patch touches files:"
echo "$PATCH_FILES" | sed 's/^/  /'

EXPECTED="hermes_cli/kanban_db.py
hermes_cli/profiles.py
tests/hermes_cli/test_kanban_db.py"
if [ "$PATCH_FILES" != "$EXPECTED" ]; then
    echo "ERROR: this script only handles the skill-validation 3-file pattern." >&2
    echo "  Expected files:" >&2
    echo "$EXPECTED" | sed 's/^/    /' >&2
    echo "  Got:" >&2
    echo "$PATCH_FILES" | sed 's/^/    /' >&2
    exit 3
fi

# Snapshot LIVE tree (working tree, not HEAD, because the dirty state is the
# actual state install.sh / sot-sync applies patches to).
WORK=$(mktemp -d -t regen-vendor-patch.XXXXXX)
UPSTREAM_TEST=$(mktemp -d -t upstream-test.XXXXXX)
trap 'rm -rf "$WORK" "$UPSTREAM_TEST"' EXIT

mkdir -p "$WORK/live/hermes_cli" "$WORK/live/tests/hermes_cli"
mkdir -p "$WORK/patched/hermes_cli" "$WORK/patched/tests/hermes_cli"
for f in $EXPECTED; do
    cp "$HERMES_AGENT_DIR/$f" "$WORK/live/$f"
done

# Initialize a tiny git repo, commit LIVE as baseline, overwrite with PATCHED,
# then `git diff` produces a clean unified patch.
REPO="$WORK/repo"
mkdir -p "$REPO"
git init -q --initial-branch=main "$REPO"
git -C "$REPO" config user.email "devops@local"
git -C "$REPO" config user.name "devops"
for f in $EXPECTED; do
    target="$REPO/$f"
    mkdir -p "$(dirname "$target")"
    cp "$WORK/live/$f" "$target"
done
git -C "$REPO" add -A
git -C "$REPO" commit -q -m "live snapshot"

# --- Apply patch content to LIVE files using Python ---
python3 - "$PATCH_IN" "$WORK/live" "$WORK/patched" <<'PYEOF'
import sys
from pathlib import Path

patch_file, live_dir, patched_dir = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])


def extract_added_lines(patch_text, file_path):
    """Return the '+' lines (without prefix) from all hunks of file_path, in order."""
    cur_file = None
    in_hunk = False
    result = []
    for raw in patch_text.split("\n"):
        if raw.startswith("diff --git "):
            cur_file = raw.split(" b/", 1)[-1]
            in_hunk = False
        elif raw.startswith("@@"):
            in_hunk = True
        elif raw.startswith("--- ") or raw.startswith("+++ "):
            continue
        elif cur_file == file_path and in_hunk and raw.startswith("+"):
            result.append(raw[1:])
    return result


def patch_kanban_db(live_text, patch_text):
    """Insert the function defs + call site from the patch into the live kanban_db.py.

    Strategy: take the patch's `+` lines (which describe the FULL intended
    post-state additions), split them at the call-site boundary
    ("# Cross-check per-task skills"), and insert:
      - hunk1_block before `def create_task(`
      - hunk2_block after `skills_list = cleaned`
    """
    lines = live_text.splitlines()
    all_added = extract_added_lines(patch_text, "hermes_cli/kanban_db.py")
    hunk1, hunk2 = [], []
    in_h2 = False
    for ln in all_added:
        if ln.startswith("    # Cross-check per-task skills against"):
            in_h2 = True
        (hunk2 if in_h2 else hunk1).append(ln)
    while hunk1 and hunk1[-1] == "":
        hunk1.pop()
    while hunk2 and hunk2[-1] == "":
        hunk2.pop()

    # If the input patch is already-regenerated (only hunk1+hunk2 missing
    # hunks; same shape), the `+` lines look identical. We insert them.
    canon_start = next(
        (i for i, ln in enumerate(lines) if ln.startswith("def _canonical_assignee(")),
        None
    )
    if canon_start is None:
        raise SystemExit("def _canonical_assignee not found in live kanban_db.py")
    create_task_idx = next(
        (i for i in range(canon_start, len(lines)) if lines[i].startswith("def create_task(")),
        None
    )
    if create_task_idx is None:
        raise SystemExit("def create_task not found in live kanban_db.py")
    lines = lines[:create_task_idx] + hunk1 + [""] + lines[create_task_idx:]

    for i, ln in enumerate(lines):
        if ln.strip() == "skills_list = cleaned":
            lines = lines[: i + 1] + hunk2 + [""] + lines[i + 1:]
            break
    else:
        raise SystemExit("skills_list = cleaned not found in live kanban_db.py")
    return "\n".join(lines) + "\n"


# Apply kanban_db.py changes
live_kanban = (live_dir / "hermes_cli/kanban_db.py").read_text()
patched = patch_kanban_db(live_kanban, Path(patch_file).read_text())
(patched_dir / "hermes_cli/kanban_db.py").write_text(patched)

# profiles.py + test_kanban_db.py: carry over as-is.
# The hunks for these files are ALREADY in the live tree from a previous
# apply; copying live → patched means git diff sees them as unchanged.
for f in ("hermes_cli/profiles.py", "tests/hermes_cli/test_kanban_db.py"):
    src = live_dir / f
    dst = patched_dir / f
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(src.read_text())
PYEOF

# --- Overwrite REPO files with PATCHED content ---
for f in $EXPECTED; do
    cp "$WORK/patched/$f" "$REPO/$f"
done

# --- Generate the patch ---
NEW_PATCH="${PATCH_IN}.new"
git -C "$REPO" diff --no-color > "$NEW_PATCH"
echo
echo "regenerated patch -> $NEW_PATCH ($(wc -c < "$NEW_PATCH") bytes, $(grep -c '^@@' "$NEW_PATCH") hunks)"

# --- Verify: applies cleanly to LIVE tree ---
echo
echo "=== Verifying: patch --dry-run against LIVE $HERMES_AGENT_DIR ==="
# Use absolute path for -i since `cd` changes cwd.
if ( cd "$HERMES_AGENT_DIR" && patch -p1 --dry-run -i "$(readlink -f "$NEW_PATCH")" >/dev/null 2>&1 ); then
    echo "  OK patch applies cleanly to live tree"
else
    echo "  FAIL patch does not apply to live tree; inspect:" >&2
    ( cd "$HERMES_AGENT_DIR" && patch -p1 --dry-run -i "$(readlink -f "$NEW_PATCH")" ) >&2
    exit 4
fi

# --- Verify: applies cleanly to clean upstream main ---
echo "=== Verifying: patch --dry-run against clean upstream main ==="
UPSTREAM_ROOT="$UPSTREAM_TEST/root"
mkdir -p "$UPSTREAM_ROOT/hermes_cli" "$UPSTREAM_ROOT/tests/hermes_cli"
for f in hermes_cli/kanban_db.py hermes_cli/profiles.py tests/hermes_cli/test_kanban_db.py; do
    if git -C "$HERMES_AGENT_DIR" show "origin/main:$f" > "$UPSTREAM_ROOT/$f" 2>/dev/null; then
        :
    else
        echo "  WARN cannot fetch origin/main:$f — skipping upstream verification"
        UPSTREAM_ROOT=""
        break
    fi
done
if [ -n "$UPSTREAM_ROOT" ]; then
    if ( cd "$UPSTREAM_ROOT" && patch -p1 --dry-run -i "$(readlink -f "$NEW_PATCH")" >/dev/null 2>&1 ); then
        echo "  OK patch applies cleanly to clean upstream main"
    else
        echo "  WARN patch does not apply to clean upstream main (fuzz needed); see:"
        ( cd "$UPSTREAM_ROOT" && patch -p1 --dry-run -i "$(readlink -f "$NEW_PATCH")" ) || true
    fi
fi

echo
echo "DONE. To replace the patch:"
echo "  diff $PATCH_IN $NEW_PATCH   # review the diff"
echo "  mv $NEW_PATCH $PATCH_IN"
