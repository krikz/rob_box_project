#!/usr/bin/env bash
# Тест marathon_has_marker на фрагменте настоящего лога акта 2 (run 39934329778, #3934).
set -u
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../marathon_markers.sh
. "$D/../marathon_markers.sh"
F="$D/fixtures/act2_39934329778_fragment.log"
fail=0
check() { # name expected(0/1) markers
    marathon_has_marker "$F" "$3"; local rc=$?
    if [ "$rc" = "$2" ]; then echo "PASS $1"; else echo "FAIL $1 (rc=$rc, ожидали $2)"; fail=1; fi
}
check "эхо E2E_NO_REACTION не маркер" 1 'E2E_NO_REACTION|E2E_INFRA_FAIL'
check "настоящий E2E_FEATURE_FAIL найден" 0 'E2E_FEATURE_FAIL'
check "настоящий E2E_VERDICT найден" 0 'E2E_VERDICT'
check "E2E_SUMMARY (>>>) найден" 0 'E2E_SUMMARY'
T="$(mktemp)"
printf '2026-10-23T09:56:30.7653489Z E2E_NO_REACTION\n' > "$T"
if marathon_has_marker "$T" 'E2E_NO_REACTION|E2E_INFRA_FAIL'; then
    echo "PASS настоящий NO_REACTION найден"
else
    echo "FAIL настоящий NO_REACTION не найден"; fail=1
fi
rm -f "$T"
exit "$fail"
