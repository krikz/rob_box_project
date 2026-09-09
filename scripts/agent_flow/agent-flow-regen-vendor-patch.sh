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
# History / why this script exists:
#   - Originally (t_f00676f8): a single 3-file skill-validation patch.
#   - Helper (t_49c2b63f): hardcoded the 3-file pattern (kanban_db.py +
#     profiles.py + test_kanban_db.py).
#   - Patch evolved (33f967c, 920b0d6): added kanban.py for --force-scope
#     CLI flag and dropped profiles.py / test_kanban_db.py (their hunks
#     were already upstreamed). New pattern: 2 files (kanban.py +
#     kanban_db.py).
#   - Helper was too rigid: it rejected anything that wasn't the original
#     3-file list (issue #2332).
#
# Strategy (N-file generic):
#   1. Snapshot LIVE hermes-agent working tree (the partially-applied state).
#   2. For each file the patch touches:
#      - If a custom inserter is registered for that path, run it
#        (currently: hermes_cli/kanban_db.py and hermes_cli/kanban.py).
#      - Otherwise: carry live content over wholesale (the hunks for this
#        file are ALREADY in the live tree from a previous apply; copy
#        live → patched and git diff sees them as unchanged).
#   3. git diff between LIVE and PATCHED → fresh patch with proper diff
#      headers. Applies cleanly to BOTH the live tree AND clean upstream main.
#
# Custom inserters for kanban_db.py / kanban.py are needed because those
# files have a known shape split into "function-definition block" +
# "call-site block" (ADR-0036 §4.1) that can't be done with a naive
# `+` line insertion. Adding new patchable files is one function each.
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
# This script is hermes-agent-agnostic — it works for any vendor patch
# whose files are either (a) carry-over from live (whole-file copy) or
# (b) anchor-based inserters listed in this script. To support a new
# anchor-based file, add a Python helper below.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HERMES_AGENT_DIR="${HERMES_AGENT_DIR:-/home/builder/.hermes/hermes-agent}"

# --baseline=<live|upstream> (default: upstream when reachable, else live)
# - upstream: regenerate patch against origin/main baseline (the place where
#   patches should ideally apply; preferred for new patches).
# - live: regenerate against the current HERMES_AGENT_DIR working tree
#   (useful when upstream moved so much that anchor-matching against
#   origin/main fails — patch becomes "live-tree-aware").
BASELINE_MODE="auto"
POSITIONAL=()
for arg in "$@"; do
    case "$arg" in
        --baseline=*) BASELINE_MODE="${arg#--baseline=}" ;;
        -h|--help)
            cat <<USAGE
usage: $0 <patch-file> [--baseline=upstream|live|auto]

Regenerates a vendor patch against a baseline so it applies cleanly:
  upstream (default if origin/main reachable) — origin/main of HERMES_AGENT_DIR
  live                                     — current HERMES_AGENT_DIR working tree
  auto                                     — upstream if reachable, else live

The regenerated patch is written to <patch-file>.new.
USAGE
            exit 0 ;;
        --baseline=*) BASELINE_MODE="${arg#--baseline=}" ;;
        *) POSITIONAL+=("$arg") ;;
    esac
done

if [ "${#POSITIONAL[@]}" -lt 1 ]; then
    echo "usage: $0 <patch-file> [--baseline=upstream|live|auto]" >&2
    echo "  e.g. $0 scripts/agent_flow/vendor/hermes-agent-skill-validation.patch" >&2
    exit 2
fi

# Normalize to absolute path so patch(1) finds it regardless of subshell cwd
# (issue #2332 debug: relative -i + cd inside subshell = empty stdin →
# vacuous OK).
PATCH_IN="$(cd "$(dirname "${POSITIONAL[0]}")" && pwd)/$(basename "${POSITIONAL[0]}")"
if [ ! -f "$PATCH_IN" ]; then
    echo "ERROR: patch file not found: $PATCH_IN" >&2
    exit 2
fi

if ! git -C "$HERMES_AGENT_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "ERROR: $HERMES_AGENT_DIR is not a git checkout; cannot snapshot" >&2
    exit 2
fi

# Detect which files the patch touches. Supports any N >= 1.
PATCH_FILES=$(grep -E '^diff --git ' "$PATCH_IN" | sed -E 's|^diff --git a/([^ ]+) b/.*|\1|' | sort -u)
PATCH_FILE_COUNT=$(echo "$PATCH_FILES" | grep -c '.' || true)
echo "patch touches $PATCH_FILE_COUNT file(s):"
echo "$PATCH_FILES" | sed 's/^/  /'

# Custom inserters. Each Python function takes (live_text, patch_text) and
# returns the post-state text. If no inserter is registered for a file, the
# carry-over path is used (live → patched wholesale, git diff sees no
# change for that file → no hunk for it in the new patch). That is the
# intended behavior for files whose hunks are already in the live tree
# from a previous apply.
declare -A HAS_INSERTER=(
    ["hermes_cli/kanban_db.py"]=1
    ["hermes_cli/kanban.py"]=1
)

INSERTER_FILES=()
CARRYOVER_FILES=()
while IFS= read -r f; do
    [ -z "$f" ] && continue
    if [ "${HAS_INSERTER[$f]:-}" = "1" ]; then
        INSERTER_FILES+=("$f")
    else
        CARRYOVER_FILES+=("$f")
    fi
done <<< "$PATCH_FILES"
echo "  inserter-based: ${INSERTER_FILES[*]:-<none>}"
echo "  carry-over    : ${CARRYOVER_FILES[*]:-<none>}"

# Snapshot LIVE tree (working tree, not HEAD, because the dirty state is the
# actual state install.sh / sot-sync applies patches to).
WORK=$(mktemp -d -t regen-vendor-patch.XXXXXX)
UPSTREAM_TEST=$(mktemp -d -t upstream-test.XXXXXX)
trap 'rm -rf "$WORK" "$UPSTREAM_TEST"' EXIT

# Try to snapshot baseline according to --baseline flag. upstream-mode
# (default if reachable) is preferred because patches should ideally apply
# to fresh origin/main; live-mode is the fallback when upstream moved so
# much that anchor-matching against origin/main fails (issue #2332).
USE_UPSTREAM=false
case "$BASELINE_MODE" in
    upstream)
        USE_UPSTREAM=true ;;
    live)
        USE_UPSTREAM=false ;;
    auto)
        # Default to LIVE baseline: the patch's anchors are tailored to the
        # files in HERMES_AGENT_DIR, and that's where install.sh applies it.
        # Operators who specifically want a patch that applies to a fresh
        # upstream main can pass --baseline=upstream.
        USE_UPSTREAM=false
        ;;
    *)
        echo "ERROR: unknown --baseline value: $BASELINE_MODE (valid: upstream|live|auto)" >&2
        exit 2
        ;;
esac

if $USE_UPSTREAM; then
    BASELINE_LABEL="origin/main"
    BASELINE_DIR="$WORK/upstream"
    mkdir -p "$BASELINE_DIR"
    while IFS= read -r f; do
        [ -z "$f" ] && continue
        mkdir -p "$BASELINE_DIR/$(dirname "$f")"
        if ! git -C "$HERMES_AGENT_DIR" show "origin/main:$f" > "$BASELINE_DIR/$f" 2>/dev/null; then
            # File doesn't exist in origin/main — treat as empty (will create it)
            : > "$BASELINE_DIR/$f"
        fi
    done <<< "$PATCH_FILES"
else
    BASELINE_LABEL="LIVE working tree"
    BASELINE_DIR="$WORK/live"
    mkdir -p "$BASELINE_DIR"
    while IFS= read -r f; do
        [ -z "$f" ] && continue
        mkdir -p "$BASELINE_DIR/$(dirname "$f")"
        cp "$HERMES_AGENT_DIR/$f" "$BASELINE_DIR/$f"
    done <<< "$PATCH_FILES"
fi

# Initialize a tiny git repo, commit BASELINE as the initial state, overwrite
# with PATCHED, then `git diff` produces a clean unified patch.
REPO="$WORK/repo"
mkdir -p "$REPO"
git init -q --initial-branch=main "$REPO"
git -C "$REPO" config user.email "devops@local"
git -C "$REPO" config user.name "devops"
while IFS= read -r f; do
    [ -z "$f" ] && continue
    target="$REPO/$f"
    mkdir -p "$(dirname "$target")"
    cp "$BASELINE_DIR/$f" "$target" 2>/dev/null || : > "$target"
done <<< "$PATCH_FILES"
git -C "$REPO" add -A
git -C "$REPO" commit -q -m "baseline ($BASELINE_LABEL)"

# --- Apply patch content to BASELINE files using Python ---
python3 - "$PATCH_IN" "$BASELINE_DIR" "$WORK/patched" <<'PYEOF'
import sys
from pathlib import Path

patch_file, base_dir, patched_dir = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])


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


def find_anchor(lines, predicate, label):
    """Return first index i where lines[i] matches predicate; raise with helpful error if not."""
    idx = next((i for i, ln in enumerate(lines) if predicate(ln)), None)
    if idx is None:
        raise SystemExit(
            f"anchor not found in base {label!r}: predicate {predicate!r} did not match.\n"
            f"  This usually means upstream refactored the file so much that the\n"
            f"  vendor patch's anchor points no longer exist. Options:\n"
            f"  1. Run this script with --baseline=live to regenerate against the\n"
            f"     live working tree (where anchors are known to exist).\n"
            f"  2. Manually port the patch to the new file layout (kanban.py may\n"
            f"     have moved to hermes_cli.kanban_parser.py in upstream main).\n"
            f"  3. Drop the patch entirely if the fix is now upstream."
        )
    return idx


def patch_kanban_db(base_text, patch_text):
    """Insert the function defs + call site from the patch into base kanban_db.py.

    Strategy: take the patch's `+` lines (which describe the FULL intended
    post-state additions), split them at the call-site boundary
    ("# Cross-check per-task skills"), and insert:
      - hunk1_block before `def create_task(`
      - hunk2_block after `skills_list = cleaned`
    """
    lines = base_text.splitlines()
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

    create_task_idx = find_anchor(
        lines,
        lambda ln: ln.startswith("def create_task("),
        "kanban_db.py",
    )
    lines = lines[:create_task_idx] + hunk1 + [""] + lines[create_task_idx:]

    skills_list_idx = find_anchor(
        lines,
        lambda ln: (
            ln.strip() == "skills_list = cleaned"
            or ln.strip() == "skills_list = _normalize_task_skills(skills)"
        ),
        "kanban_db.py (call-site)",
    )
    lines = lines[: skills_list_idx + 1] + hunk2 + [""] + lines[skills_list_idx + 1 :]
    return "\n".join(lines) + "\n"


def patch_kanban_py(base_text, patch_text):
    """Apply --force-scope additions to base kanban.py.

    Strategy: take the patch's `+` lines for kanban.py, split them at the
    call-site boundary (the `_cmd_create` forwarding line carrying
    `force_scope=bool(getattr(args, "force_scope", False))`), and insert:
      - hunk1_block right BEFORE the existing `--json` add_argument
      - hunk2_block right AFTER the `goal_max_turns=...` forwarding in
        _cmd_create

    If the patch no longer carries --force-scope (upstream included it),
    carry-over is the safe fallback — `extract_added_lines` returns [],
    both inserts are no-ops, and the regenerated patch carries no hunks
    for kanban.py (effectively dropping the file from the patch).
    """
    lines = base_text.splitlines()
    added = extract_added_lines(patch_text, "hermes_cli/kanban.py")
    if not added:
        return base_text

    # Split at second occurrence of `force_scope=bool(getattr(...))`.
    hunk1, hunk2 = [], []
    seen_first = False
    for ln in added:
        if "force_scope=bool(getattr(args, \"force_scope\"" in ln:
            if not seen_first:
                seen_first = True
                hunk1.append(ln)
            else:
                hunk2.append(ln)
        else:
            (hunk2 if seen_first else hunk1).append(ln)
    while hunk1 and hunk1[-1] == "":
        hunk1.pop()
    while hunk2 and hunk2[-1] == "":
        hunk2.pop()

    if hunk1:
        # Anchor: `--json` add_argument inside the `create` subparser. In
        # older hermes-agent this lives in `hermes_cli/kanban.py`; in
        # newer upstream main it has been moved to `hermes_cli/kanban_parser.py`.
        # We search both forms:
        #   - `p_create.add_argument("--json"`   (older — kanban.py)
        #   - `_arg("--json", dest="json"`      (newer — kanban_parser.py)
        json_anchor = None
        for i, ln in enumerate(lines):
            if (
                'p_create.add_argument("--json"' in ln
                or '_arg("--json", dest="json"' in ln
            ):
                json_anchor = i
                break
        if json_anchor is None:
            raise SystemExit(
                "anchor not found in base kanban.py: '--json' add_argument "
                "neither 'p_create.add_argument(\"--json\")' (legacy) nor "
                "'_arg(\"--json\", dest=\"json\")' (modern kanban_parser.py) "
                "found. Upstream has refactored the create subparser beyond "
                "this helper's anchor knowledge — manual port required."
            )
        lines = lines[:json_anchor] + hunk1 + [""] + lines[json_anchor:]

    if hunk2:
        gmt_anchor = None
        for i, ln in enumerate(lines):
            if 'goal_max_turns=getattr(args, "goal_max_turns"' in ln:
                gmt_anchor = i + 1
                break
        if gmt_anchor is None:
            raise SystemExit(
                "anchor not found in base kanban.py: 'goal_max_turns=getattr(args, ...)' "
                "missing — upstream may have refactored _cmd_create body"
            )
        lines = lines[:gmt_anchor] + hunk2 + [""] + lines[gmt_anchor:]

    return "\n".join(lines) + "\n"


INSERTERS = {
    "hermes_cli/kanban_db.py": patch_kanban_db,
    "hermes_cli/kanban.py": patch_kanban_py,
}


patch_text = Path(patch_file).read_text()

# Apply inserter-based patches against BASE.
for fpath, fn in INSERTERS.items():
    base_path = base_dir / fpath
    patched_path = patched_dir / fpath
    if not base_path.exists():
        continue
    base_text = base_path.read_text()
    patched = fn(base_text, patch_text)
    patched_path.parent.mkdir(parents=True, exist_ok=True)
    patched_path.write_text(patched)

# Carry over every other file wholesale (their hunks already exist in live).
import os
for root, _dirs, files in os.walk(base_dir):
    rel = os.path.relpath(root, base_dir)
    for fname in files:
        rel_path = rel if rel == "." else os.path.join(rel, fname)
        src = base_dir / rel_path
        dst = patched_dir / rel_path
        dst.parent.mkdir(parents=True, exist_ok=True)
        # If an inserter already wrote this file, skip.
        if dst.exists() and dst.stat().st_mtime >= src.stat().st_mtime:
            continue
        dst.write_text(src.read_text())
PYEOF

# --- Overwrite REPO files with PATCHED content ---
while IFS= read -r f; do
    [ -z "$f" ] && continue
    if [ -f "$WORK/patched/$f" ]; then
        mkdir -p "$(dirname "$REPO/$f")"
        cp "$WORK/patched/$f" "$REPO/$f"
    else
        mkdir -p "$(dirname "$REPO/$f")"
        cp "$WORK/live/$f" "$REPO/$f"
    fi
done <<< "$PATCH_FILES"

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

# --- Verify: applies cleanly to clean upstream main (best-effort, may fail with fuzz) ---
# Note: this is a soft check. We try with `-F 0` (no fuzz allowed) and warn
# if patch(1) reports fuzzed lines or outright failure. Without `-F 0`, patch
# silently accepts fuzzy matches and reports OK, which is a false positive.
echo "=== Verifying: patch --dry-run against clean upstream main (-F 0, no fuzz) ==="
UPSTREAM_ROOT="$UPSTREAM_TEST/root"
UPSTREAM_OK=unknown
while IFS= read -r f; do
    [ -z "$f" ] && continue
    mkdir -p "$UPSTREAM_ROOT/$(dirname "$f")"
    if git -C "$HERMES_AGENT_DIR" show "origin/main:$f" > "$UPSTREAM_ROOT/$f" 2>/dev/null; then
        :
    else
        echo "  WARN cannot fetch origin/main:$f — skipping upstream verification"
        UPSTREAM_ROOT=""
        break
    fi
done <<< "$PATCH_FILES"
if [ -n "$UPSTREAM_ROOT" ]; then
    if ( cd "$UPSTREAM_ROOT" && patch -p1 --dry-run -F 0 -i "$(readlink -f "$NEW_PATCH")" >/dev/null 2>&1 ); then
        echo "  OK patch applies cleanly to clean upstream main (no fuzz)"
        UPSTREAM_OK=yes
    else
        echo "  WARN patch needs fuzz (or fails) on upstream main — see:"
        ( cd "$UPSTREAM_ROOT" && patch -p1 --dry-run -F 0 -i "$(readlink -f "$NEW_PATCH")" ) 2>&1 | head -10 || true
        echo "         (non-fatal: install.sh's sentinel-detect will skip patch on hosts where upstream already includes the fix)"
        UPSTREAM_OK=no
    fi
fi

echo
echo "DONE. To replace the patch:"
echo "  diff $PATCH_IN $NEW_PATCH   # review the diff"
echo "  mv $NEW_PATCH $PATCH_IN"
