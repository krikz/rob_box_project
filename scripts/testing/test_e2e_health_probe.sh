#!/usr/bin/env bash
# Advisory health probe contract: failures are represented in JSON and never
# become the functional e2e exit status.
# set -e обязателен: без него `printf 'health probe contract: PASS'` в конце
# выполнялся ВСЕГДА — в том числе когда python3 недоступен и обе питоновские
# части свалились. Тест печатал PASS, ничего не проверив (наблюдалось на
# dev-машине под Windows 22.09.2026). Это ровно тот «красивый PASS», против
# которого ADR-0018.
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
command -v python3 >/dev/null 2>&1 || {
    printf 'health probe contract: SKIP — нет python3 (проба его требует)\n' >&2
    exit 1
}
source "$ROOT/.github/workflows/scripts/e2e_voice_lib.sh"
probe="$(ROBOT_SSH=true observe_step unit 2>&1)"
python3 - "$probe" <<'PY'
import json, sys
snapshot = json.loads(sys.argv[1])
assert snapshot["step"] == "unit"
assert "raw" in snapshot
PY
printf 'health probe contract: PASS\n'
