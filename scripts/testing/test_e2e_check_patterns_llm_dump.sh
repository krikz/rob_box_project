#!/usr/bin/env bash
# Unit-тест check_patterns: дамп LLM-запроса не считается доказательством (issue #3270).
#
# Зачем: лог каждого LLM-запроса печатает «  tools(61): ..., music_library, ...»
# и «  [N] system: ...» с текстом промпта. Голый grep -E по такому логу находил
# имя тула, которого никто не вызывал (ml06: PATTERN_OK music_library).
#
# Случаи:
#   1. имя тула только в tools(N) -> PATTERN_MISS, rc=1 (ml06-подобный);
#   2. имя тула только в тексте промпта «[0] system:» / истории tool_calls -> MISS;
#   3. настоящая строка вызова «Запрос выполнения: <tool>» -> OK, даже когда
#      рядом есть дамп с тем же именем;
#   4. паттерн, живущий вне дампа (не-тул) -> OK;
#   5. обрезанный дамп без LLM REQUEST END не съедает остаток лога.
#
# Офлайн: source e2e_voice_lib.sh + check_patterns (вырезана awk'ом из
# e2e_voice_test.sh) со стабом ROBOT_SSH.
# Запуск: bash scripts/testing/test_e2e_check_patterns_llm_dump.sh
# Exit:   0 = PASS, N>0 = N failed assertions.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LIB="$REPO_ROOT/.github/workflows/scripts/e2e_voice_lib.sh"
HARNESS="$REPO_ROOT/.github/workflows/scripts/e2e_voice_test.sh"

[ -f "$LIB" ] && [ -f "$HARNESS" ] || { echo "FATAL: нет lib/harness"; exit 1; }
# shellcheck disable=SC1090
source "$LIB" 2>/dev/null
type strip_llm_request_dump >/dev/null 2>&1 || { echo "FATAL: strip_llm_request_dump не определена"; exit 2; }

FN="$(awk '/^check_patterns\(\) \{/,/^\}/' "$HARNESS")"
[ -n "$FN" ] || { echo "FATAL: check_patterns не найдена в harness"; exit 3; }
eval "$FN"

PASS=0
FAIL=0
ok()  { printf 'OK   %s\n' "$1"; PASS=$((PASS + 1)); }
bad() { printf 'FAIL %s\n' "$1"; FAIL=$((FAIL + 1)); }

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT
fake_ssh() { cat "$WORKDIR/docker_logs.txt"; }
ROBOT_SSH="fake_ssh"

DUMP="🔎 LLM REQUEST START provider=minimax mode=stream
  [0] system: 'Ты Робби. Для трека зови music_library и save_waypoint'
  [1] assistant: '' tool_calls=[{\"name\": \"memory_save\", \"args\": {}}]
  tools(3): add_music_material, music_library, speak_text
🔎 LLM REQUEST END"

run() { # $1=лог, остальное=паттерны; вывод в OUT, код в RC
    printf '%s\n' "$1" > "$WORKDIR/docker_logs.txt"; shift
    OUT="$(check_patterns "t0" "$@")"; RC=$?
}

# 1. имя тула только в tools(N)
run "$DUMP" 'music_library'
{ [ "$RC" = 1 ] && printf '%s' "$OUT" | grep -q 'PATTERN_MISS: music_library'; } \
    && ok "1: имя тула в tools(N) -> MISS" || bad "1: ждали MISS, rc=$RC out=$OUT"

# 2. имя в промпте и в tool_calls истории
run "$DUMP" 'save_waypoint' 'memory_save'
[ "$RC" = 1 ] && [ "$(printf '%s' "$OUT" | grep -c PATTERN_MISS)" = 2 ] \
    && ok "2: имя в промпте/истории дампа -> MISS" || bad "2: rc=$RC out=$OUT"

# 3. реальный вызов рядом с дампом
run "$DUMP
Запрос выполнения: music_library с параметрами {}" 'music_library'
[ "$RC" = 0 ] && ok "3: строка вызова -> OK" || bad "3: rc=$RC out=$OUT"

# 4. не-тул паттерн вне дампа
run "$DUMP
[backlog] flushed to LLM backlog_handled=true" 'flushed to LLM'
[ "$RC" = 0 ] && ok "4: паттерн вне дампа -> OK" || bad "4: rc=$RC out=$OUT"

# 5. обрезанный дамп (без END) не съедает хвост лога
run "🔎 LLM REQUEST START provider=minimax mode=stream
  [0] system: 'x'
Запрос выполнения: stop_music с параметрами {}" 'stop_music'
[ "$RC" = 0 ] && ok "5: обрезанный дамп, хвост цел" || bad "5: rc=$RC out=$OUT"

# 6. issue #3860: дамп v2 с launch-префиксом ``[dialog_node-3]`` и recent_actions с именем тула
PREFIXED="[dialog_node-3]   [17] user: '<system_context>\\n  <recent_actions>[{\"tool\":\"register_speaker\",\"status\":\"done\"}]</recent_actions>'
[dialog_node-3]   tools(2): register_speaker, stop_music"
run "$PREFIXED" 'register_speaker' '"register_speaker"'
[ "$RC" = 1 ] && [ "$(printf '%s' "$OUT" | grep -c PATTERN_MISS)" = 2 ] \
    && ok "6: дамп с launch-префиксом -> MISS (#3860)" || bad "6: rc=$RC out=$OUT"

# 7. тот же дамп + настоящая строка вызова
run "$PREFIXED
Запрос выполнения: register_speaker с параметрами {}" 'register_speaker'
[ "$RC" = 0 ] && ok "7: префиксный дамп + реальный вызов -> OK" || bad "7: rc=$RC out=$OUT"

printf '\nPASS=%s FAIL=%s\n' "$PASS" "$FAIL"
exit "$FAIL"
