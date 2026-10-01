#!/usr/bin/env bash
# Unit-тест restore_robot_tts_default() (issue #3248).
#
# Зачем: voice_core_suite_v1 mv01 переключает робота на Yandex «alena», и
# после run 36775544782 голос так и остался из акта — set_provider
# персистится в /data/tts_provider_state.json. Харнесс в trap EXIT обязан
# вернуть дефолт set_voice в обход LLM, но ТОЛЬКО если за прогон голос
# реально меняли (строки вызова, а не имя тула в дампе каталога tools(N)).
#
# Покрытые случаи:
#   1. E2E_RUN_BEFORE пуст -> ничего не делаем;
#   2. в логе только дамп каталога «tools(61): …set_voice…» -> робот не трогаем;
#   3. «[set_voice] voice=…» в логе -> set_voice(провайдер из /tts_node provider)
#      через /mcp/execute, успех громко в логе;
#   4. то же, но /mcp/result с success=false -> громкое «НЕ возвращён»;
#   5. /tts_node provider не прочитан -> громко, вызова нет;
#   6. код, уходящий в контейнер base64-строкой, компилируется и берёт
#      провайдера из argv[1].
#
# Офлайн: source e2e_voice_lib.sh + свои robot_ros()/log()/ROBOT_SSH.
# Запуск: bash scripts/testing/test_e2e_tts_restore.sh
# Exit:   0 = PASS, N>0 = N failed assertions.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LIB="$REPO_ROOT/.github/workflows/scripts/e2e_voice_lib.sh"

[ -f "$LIB" ] || { printf 'FATAL: отсутствует %s\n' "$LIB"; exit 1; }
# shellcheck disable=SC1090
source "$LIB" 2>/dev/null
type restore_robot_tts_default >/dev/null 2>&1 || { echo "FATAL: restore_robot_tts_default не определена"; exit 2; }

PASS=0
FAIL=0
ok()  { printf 'OK   %s\n' "$1"; PASS=$((PASS + 1)); }
bad() { printf 'FAIL %s\n' "$1"; FAIL=$((FAIL + 1)); }

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

log() { printf '%s\n' "$*" >> "$WORKDIR/log.txt"; }
fake_ssh() { cat "$WORKDIR/docker_logs.txt"; }
ROBOT_SSH="fake_ssh"
# Настоящий robot_ros берёт ОДНУ строку и отдаёт её eval'у на роботе — стаб
# разбирает её тем же eval'ом в argv и записывает по аргументу на строку:
# так тест ловит и поломанное квотирование (а не только содержимое).
robot_ros() {
    case "$1" in
        "python3 -c "*)
            eval "set -- $1"
            printf '%s\n' "$@" > "$WORKDIR/py_args.txt"
            cat "$WORKDIR/mcp_out.txt"
            return 0
            ;;
    esac
    case "$*" in
        *"ros2 param get '/tts_node' 'provider'"*)
            [ -s "$WORKDIR/provider.txt" ] && echo "String value is: $(cat "$WORKDIR/provider.txt")"
            ;;
    esac
}

reset_case() {
    : > "$WORKDIR/log.txt"
    rm -f "$WORKDIR/py_args.txt"
    printf '%s' "$1" > "$WORKDIR/docker_logs.txt"
    printf '%s' "$2" > "$WORKDIR/provider.txt"
    printf '%s\n' "$3" > "$WORKDIR/mcp_out.txt"
}

CATALOG_DUMP="[dialogue_node-4]   tools(61): add_music_material, set_voice, set_tts_provider, speak_text"
SET_VOICE_LINE="[mcp_server-10] [INFO] [1790801748.271554909] [mcp_server]: [set_voice] [set_voice] voice='alena' provider=yandex default=anton switched=True"
OK_JSON='{"tool_name": "set_voice", "request_id": "e2e-tts-restore-x", "result": {"success": true, "data": {"status": "ok", "voice_set": "male-qn-qingse", "provider": "minimax", "provider_switched": true}}}'
FAIL_JSON='{"tool_name": "set_voice", "request_id": "e2e-tts-restore-x", "result": {"success": false, "error": "voice_unavailable"}}'

# 1. Нет E2E_RUN_BEFORE.
reset_case "$SET_VOICE_LINE" "minimax" "$OK_JSON"
E2E_RUN_BEFORE="" restore_robot_tts_default
[ ! -f "$WORKDIR/py_args.txt" ] && ok "1: без E2E_RUN_BEFORE робот не трогаем" || bad "1: вызов при пустом E2E_RUN_BEFORE"

E2E_RUN_BEFORE="2026-09-30T20:51:56Z"

# 2. Только дамп каталога — не доказательство смены голоса.
reset_case "$CATALOG_DUMP" "minimax" "$OK_JSON"
restore_robot_tts_default
if [ ! -f "$WORKDIR/py_args.txt" ] && grep -q "не меняли" "$WORKDIR/log.txt"; then
    ok "2: дамп tools(N) с set_voice не запускает восстановление"
else
    bad "2: восстановление по имени тула в каталоге (тавтология)"
fi

# 3. Реальный set_voice -> вызов с провайдером из конфига, успех в логе.
reset_case "$CATALOG_DUMP
$SET_VOICE_LINE" "minimax" "zenoh INFO noise line
$OK_JSON"
restore_robot_tts_default
if [ -f "$WORKDIR/py_args.txt" ] && [ "$(tail -1 "$WORKDIR/py_args.txt")" = "minimax" ]; then
    ok "3: set_voice зовётся с провайдером /tts_node provider (minimax)"
else
    bad "3: нет вызова или не тот провайдер: $(cat "$WORKDIR/py_args.txt" 2>/dev/null | tail -1)"
fi
if grep -q "возвращён дефолт.*provider=minimax voice=male-qn-qingse" "$WORKDIR/log.txt"; then
    ok "3: успех залогирован с provider/voice из /mcp/result"
else
    bad "3: нет строки успеха: $(cat "$WORKDIR/log.txt")"
fi

# 4. /mcp/result success=false -> громко.
reset_case "$SET_VOICE_LINE" "minimax" "$FAIL_JSON"
restore_robot_tts_default
grep -q "НЕ возвращён (voice_unavailable)" "$WORKDIR/log.txt" \
    && ok "4: провал set_voice громко в логе" \
    || bad "4: провал не залогирован: $(cat "$WORKDIR/log.txt")"

# 4b. Таймаут/пустой ответ -> тоже громко.
reset_case "$SET_VOICE_LINE" "minimax" ""
restore_robot_tts_default
grep -q "НЕ возвращён" "$WORKDIR/log.txt" \
    && ok "4b: пустой ответ /mcp/result громко в логе" \
    || bad "4b: пустой ответ не залогирован: $(cat "$WORKDIR/log.txt")"

# 5. Провайдер не прочитан -> громко, без вызова.
reset_case "$SET_VOICE_LINE" "" "$OK_JSON"
restore_robot_tts_default
if [ ! -f "$WORKDIR/py_args.txt" ] && grep -q "не прочитан" "$WORKDIR/log.txt"; then
    ok "5: без /tts_node provider — громкий отказ, без вызова"
else
    bad "5: ожидали громкий отказ без вызова"
fi

# 6. Код для контейнера компилируется и берёт провайдера из argv[1].
reset_case "$SET_VOICE_LINE" "yandex" "$OK_JSON"
restore_robot_tts_default
[ "$(wc -l < "$WORKDIR/py_args.txt" | tr -d ' ')" = "4" ] \
    && ok "6: eval разбирает команду ровно в 4 аргумента (python3 -c <код> <провайдер>)" \
    || bad "6: квотирование сломано: $(cat "$WORKDIR/py_args.txt" | head -c 300)"
code_arg="$(sed -n '3p' "$WORKDIR/py_args.txt")"
b64="$(printf '%s' "$code_arg" | sed -E "s/.*b64decode\('([^']*)'\).*/\1/")"
if printf '%s' "$b64" | base64 -d 2>/dev/null | python3 -c 'import sys
src = sys.stdin.read()
compile(src, "<e2e_tts_restore>", "exec")
assert "sys.argv[1]" in src and "\"set_voice\"" in src and "sender=\"harness\"" in src
' 2>/dev/null; then
    ok "6: base64-код компилируется, зовёт set_voice от sender=harness, провайдер из argv[1]"
else
    bad "6: base64-код не компилируется или не тот контракт"
fi
[ "$(tail -1 "$WORKDIR/py_args.txt")" = "yandex" ] \
    && ok "6: провайдер берётся из конфига, не хардкодом" \
    || bad "6: провайдер не из конфига"

printf '\n%d passed, %d failed\n' "$PASS" "$FAIL"
exit "$FAIL"
