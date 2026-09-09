#!/usr/bin/env bash
# tools/ffmpeg_decode/run_all.sh — run every scenario, summarise.
#
# Usage:
#     bash tools/ffmpeg_decode/run_all.sh
#
# Exit code is the number of scenarios that failed (0 = all pass, capped
# at 255 by the shell). Per-scenario exit codes are reported in the
# summary table and emitted by each individual scenario when run on its
# own (see tools/ffmpeg_decode/README.md for the code map).
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
SCENARIOS=(pcm wav mp3 ogg)

declare -a names=()
declare -a results=()
declare -a durations=()

failed=0
total=${#SCENARIOS[@]}
t0=$(date +%s.%N)

for sc in "${SCENARIOS[@]}"; do
    echo "================================================================"
    echo " Scenario: ${sc}"
    echo "================================================================"
    script="${HERE}/scenario_${sc}.sh"
    if [[ ! -x "${script}" ]]; then
        chmod +x "${script}" 2>/dev/null || true
    fi
    ts=$(date +%s.%N)
    if bash "${script}"; then
        te=$(date +%s.%N)
        names+=("${sc}")
        results+=("OK")
        durations+=("$(python3 -c "print(f'{$te - $ts:.2f}s')")")
    else
        code=$?
        te=$(date +%s.%N)
        names+=("${sc}")
        results+=("FAIL(${code})")
        durations+=("$(python3 -c "print(f'{$te - $ts:.2f}s')")")
        failed=$((failed + 1))
    fi
done

t1=$(date +%s.%N)
echo
echo "================================================================"
echo " Summary"
echo "================================================================"
printf "  %-8s %-12s %-10s\n" "scenario" "result" "elapsed"
printf "  %-8s %-12s %-10s\n" "--------" "------" "-------"
for i in "${!names[@]}"; do
    printf "  %-8s %-12s %-10s\n" "${names[$i]}" "${results[$i]}" "${durations[$i]}"
done
echo
echo "  total elapsed: $(python3 -c "print(f'{$t1 - $t0:.2f}s')")"
echo "  passed: $((total - failed)) / ${total}"

if [[ ${failed} -gt 0 ]]; then
    if [[ ${failed} -gt 255 ]]; then failed=255; fi
    exit ${failed}
fi
exit 0
