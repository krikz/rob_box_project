#!/usr/bin/env bash
# Advisory health probe contract: failures are represented in JSON and never
# become the functional e2e exit status.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
source "$ROOT/.github/workflows/scripts/e2e_voice_lib.sh"
probe="$(ROBOT_SSH=true observe_step unit 2>&1)"
python3 - "$probe" <<'PY'
import json, sys
snapshot = json.loads(sys.argv[1])
assert snapshot["step"] == "unit"
assert "raw" in snapshot
PY
printf 'health probe contract: PASS\n'
