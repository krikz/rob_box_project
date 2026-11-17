#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
    -I"$ROOT/scripts/hil/diagnostics" \
    "$ROOT/scripts/hil/tests/pathcalc_snapshot_test.cpp" \
    -o "$WORK/pathcalc_snapshot_test"
"$WORK/pathcalc_snapshot_test" "$WORK"
