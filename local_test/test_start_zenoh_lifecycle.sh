#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_FILE="/tmp/test_start_zenoh_lifecycle.out"

cleanup() {
    pkill -f rmw_zenohd 2>/dev/null || true
}

trap cleanup EXIT

cleanup

set +e
timeout 8 "$SCRIPT_DIR/start_zenoh.sh" >"$OUTPUT_FILE" 2>&1
STATUS=$?
set -e

if [ "$STATUS" -ne 124 ]; then
    echo "Expected start_zenoh.sh to stay alive until timeout, got exit code $STATUS"
    echo "=== start_zenoh.sh output ==="
    cat "$OUTPUT_FILE"
    exit 1
fi

if ! grep -q "Роутер PID=" "$OUTPUT_FILE"; then
    echo "Router startup confirmation missing from output"
    echo "=== start_zenoh.sh output ==="
    cat "$OUTPUT_FILE"
    exit 1
fi

echo "start_zenoh.sh stays attached to the router process as expected"