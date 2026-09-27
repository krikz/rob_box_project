#!/bin/bash
# ============================================================================
# test_padavan_step4_voice_smoke.sh — регресс-тест для issue #1772.
#
# Покрывает acceptance:
#   S1. all_missing: оба .wav отсутствуют на Katana → exit 0 + WARN
#   S2. one_missing: один из двух .wav есть, другой нет → exit 0 + WARN
#       (и не запускается paplay для существующего — NO-OP целиком)
#   S3. all_present: оба .wav на месте → пытается paplay, exit 0/2
#   S4. katana_unreachable: ssh не доступен → exit 0 + WARN (не fail)
#   S5. sshpass_missing: sshpass не на PATH → exit 0 + WARN
#   S6. dry_run_with_missing: --dry-run + файлов нет → exit 0, no ssh
#   S7. override: SMOKE_COMMANDS с пользовательским списком
#       (один файл, всё на месте) → корректно проходит checks
#
# Стратегия: PATH-hijack sshpass + mock-ssh. Не делаем реальных ssh-вызовов
# к 10.1.1.249. Файлы на Katana проверяем через mock-ssh, который
# возвращает exit 0/1 по MOCK_KATANA_FILE_EXISTS_<path>=yes/no.
#
# Run:
#   bash scripts/agent_flow/tests/test_padavan_step4_voice_smoke.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SMOKE_SH="${SMOKE_SH:-$TEST_DIR/../padavan-step4-voice-smoke.sh}"

[ -f "$SMOKE_SH" ] || { echo "FAIL: $SMOKE_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/bin" "$WORK/empty"

# Цвета
if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; END=''
fi

TESTS_TOTAL=0; TESTS_PASSED=0; TESTS_FAILED=0
FAILED_NAMES=()

run_test() {
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    printf '%s[ RUN     ]%s %s\n' "$YEL" "$END" "$name"
    if "$fn"; then
        TESTS_PASSED=$((TESTS_PASSED+1))
        printf '%s[   PASS  ]%s %s\n' "$GRN" "$END" "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED+1))
        FAILED_NAMES+=("$name")
        printf '%s[   FAIL  ]%s %s\n' "$RED" "$END" "$name"
    fi
}

# --- mock-sshpass ---------------------------------------------------------
# MOCK_KATANA_FILE_EXISTS_<path> = yes/no   (для test -f)
# MOCK_KATANA_FAIL = 1                       (общий сбой любого вызова)
# MOCK_PAPLAY_CALLS = файл, куда пишем каждый вызов paplay
# MOCK_ROBOT_LOGS = строка, которую вернёт docker logs ... | grep
cat > "$WORK/bin/sshpass" <<'EOF'
#!/bin/bash
# Парсим "ssh -o opts USER@HOST" чтобы достать host
_user_host=""
_target=""
for arg in "$@"; do
    case "$arg" in
        *@*) _user_host="$arg" ;;
    esac
done
# Третий аргумент-команда — обычно идёт после "-C no -o opts USER@HOST \"cmd\""
# Берём последний аргумент.
_target="${@: -1}"
# Если общий фейл — сразу fail (имитация network unreachable).
# Проверяем ДО определения target (раньше _target извлекался ПОСЛЕ,
# что давало ложный exit 0 для ssh_katana "true").
if [ "${MOCK_KATANA_FAIL:-}" = "1" ]; then
    exit 7
fi
# Нормализуем имя var: заменяем / на _, и . на _ (для .wav → _wav).
# Так MOCK_KATANA_FILE_EXISTS__tmp_cmd_X_wav матчит /tmp/cmd_X.wav
_normalize() {
    echo "$1" | tr '/.' '__'
}
# Если user_host содержит "249" — это Katana. Иначе — robot.
case "$_user_host" in
    *@10.1.1.249|*@*249*)
        # Katana. Если target =~ test -f <path>
        case "$_target" in
            "test"*)
                # Извлекаем path из "test -f PATH"
                _path="${_target##* }"
                _key="$(_normalize "$_path")"
                _var="MOCK_KATANA_FILE_EXISTS_${_key}"
                _val="${!_var:-no}"
                if [ "$_val" = "yes" ]; then exit 0; else exit 1; fi
                ;;
            "true")
                exit 0
                ;;
            paplay*)
                # Логируем вызов paplay, всегда exit 0
                if [ -n "${MOCK_PAPLAY_CALLS:-}" ]; then
                    echo "$_target" >> "$MOCK_PAPLAY_CALLS"
                fi
                exit 0
                ;;
            *)
                exit 0
                ;;
        esac
        ;;
    *@10.1.1.21|*@*21*)
        # Robot. docker logs → MOCK_ROBOT_LOGS
        echo "${MOCK_ROBOT_LOGS:-}"
        exit 0
        ;;
    *)
        exit 0
        ;;
esac
EOF
chmod +x "$WORK/bin/sshpass"

# --- mock-ssh (для случая, если sshpass попытается дёрнуть ssh) ----------
cat > "$WORK/bin/ssh" <<'EOF'
#!/bin/bash
exit 0
EOF
chmod +x "$WORK/bin/ssh"

# Helper: запустить smoke с чистым окружением и перехватом PATH.
# $1 = extra env-args (строка, валидный bash, например
#      "SMOKE_COMMANDS=(/tmp/a|x /tmp/b|y)" или "SMOKE_COMMANDS='a b'").
#
# Стратегия: пишем tmp rcfile в $WORK/.rc, source'им его в дочерний bash
# ВМЕСТЕ с $SMOKE_SH через `bash -c "source rc; source SMOKE"`. Так
# bash-array из rcfile виден внутри smoke.sh.
#
# Возвращает: первая строка = rc, последующие = stdout+stderr.
run_smoke() {
    local extra_env="$1"
    local rcfile="$WORK/.rc.current"
    local out rc
    {
        echo 'export HOME=/home/builder'
        echo "export PATH=\"$WORK/bin:\$PATH\""
        local v
        while IFS= read -r v; do
            [ -n "$v" ] && echo "export $v"
        done < <(env | grep -E '^MOCK_[A-Z_]+=' || true)
        if [ -n "$extra_env" ]; then
            echo "$extra_env"
        fi
    } > "$rcfile"
    set +e
    out="$(bash -c "source '$rcfile'; source '$SMOKE_SH'" 2>&1)"
    rc=$?
    set -e
    printf '__RC__=%d\n' "$rc"
    printf '%s\n' "$out"
}

# ============================================================================
# S1. all_missing — оба .wav отсутствуют → exit 0 + WARN
# ============================================================================
test_s1_all_missing() {
    unset MOCK_KATANA_FAIL MOCK_PAPLAY_CALLS MOCK_ROBOT_LOGS
    unset MOCK_KATANA_FILE_EXISTS__tmp_cmd_muzika_i_filosofiya_wav
    unset MOCK_KATANA_FILE_EXISTS__tmp_cmd_dlinnyi_zapros_wav
    local result rc out
    result="$(run_smoke "")"
    rc="$(echo "$result" | head -1 | sed 's/__RC__=//')"
    out="$(echo "$result" | tail -n +2)"

    [ "$rc" = "0" ] || { echo "  expected exit 0, got $rc"; return 1; }
    echo "$out" | grep -q "missing: /tmp/cmd_muzika_i_filosofiya.wav" || {
        echo "  expected WARN about missing file #1"; return 1; }
    echo "$out" | grep -q "missing: /tmp/cmd_dlinnyi_zapros.wav" || {
        echo "  expected WARN about missing file #2"; return 1; }
    echo "$out" | grep -q "Шаг 4 = NO-OP" || {
        echo "  expected NO-OP marker"; return 1; }
    if [ -n "${MOCK_PAPLAY_CALLS:-}" ] && [ -f "${MOCK_PAPLAY_CALLS}" ] && [ -s "${MOCK_PAPLAY_CALLS}" ]; then
        echo "  paplay был вызван при missing files (НЕ ДОЛЖЕН): $(cat "$MOCK_PAPLAY_CALLS")"
        return 1
    fi
    return 0
}

# ============================================================================
# S2. one_missing — один файл есть, другого нет → exit 0 + NO-OP
# ============================================================================
test_s2_one_missing() {
    unset MOCK_KATANA_FAIL MOCK_ROBOT_LOGS
    export MOCK_KATANA_FILE_EXISTS__tmp_cmd_muzika_i_filosofiya_wav="yes"
    unset MOCK_KATANA_FILE_EXISTS__tmp_cmd_dlinnyi_zapros_wav
    export MOCK_PAPLAY_CALLS="$WORK/paplay2.log"
    rm -f "$MOCK_PAPLAY_CALLS"

    local result rc out
    result="$(run_smoke "")"
    rc="$(echo "$result" | head -1 | sed 's/__RC__=//')"
    out="$(echo "$result" | tail -n +2)"

    [ "$rc" = "0" ] || { echo "  expected exit 0, got $rc"; return 1; }
    echo "$out" | grep -q "Шаг 4 = NO-OP" || {
        echo "  expected NO-OP when any file missing"; return 1; }
    if [ -f "$MOCK_PAPLAY_CALLS" ] && [ -s "$MOCK_PAPLAY_CALLS" ]; then
        echo "  paplay вызван хотя один файл отсутствует: $(cat "$MOCK_PAPLAY_CALLS")"
        return 1
    fi
    return 0
}

# ============================================================================
# S3. all_present + robot ответил — exit 0, paplay x2
# ============================================================================
test_s3_all_present_robot_ok() {
    unset MOCK_KATANA_FAIL
    export MOCK_KATANA_FILE_EXISTS__tmp_cmd_muzika_i_filosofiya_wav="yes"
    export MOCK_KATANA_FILE_EXISTS__tmp_cmd_dlinnyi_zapros_wav="yes"
    export MOCK_ROBOT_LOGS="ПРИНЯТО some text
speak_text response"
    export MOCK_PAPLAY_CALLS="$WORK/paplay3.log"
    rm -f "$MOCK_PAPLAY_CALLS"

    local result rc out
    result="$(run_smoke "")"
    rc="$(echo "$result" | head -1 | sed 's/__RC__=//')"
    out="$(echo "$result" | tail -n +2)"

    [ "$rc" = "0" ] || { echo "  expected exit 0, got $rc (out: $out)"; return 1; }
    [ -f "$MOCK_PAPLAY_CALLS" ] || { echo "  paplay log not created (out: $out)"; return 1; }
    local n
    n="$(wc -l < "$MOCK_PAPLAY_CALLS")"
    [ "$n" -ge "2" ] || { echo "  expected ≥2 paplay calls, got $n"; return 1; }
    echo "$out" | grep -q "voice-assistant ответил" || {
        echo "  expected 'voice-assistant ответил' marker (out: $out)"; return 1; }
    return 0
}

# ============================================================================
# S4. katana_unreachable → exit 0 + WARN (НЕ fail)
# ============================================================================
test_s4_katana_unreachable() {
    export MOCK_KATANA_FAIL=1
    unset MOCK_ROBOT_LOGS MOCK_PAPLAY_CALLS
    unset MOCK_KATANA_FILE_EXISTS__tmp_cmd_muzika_i_filosofiya_wav
    unset MOCK_KATANA_FILE_EXISTS__tmp_cmd_dlinnyi_zapros_wav

    local result rc out
    result="$(run_smoke "")"
    rc="$(echo "$result" | head -1 | sed 's/__RC__=//')"
    out="$(echo "$result" | tail -n +2)"

    [ "$rc" = "0" ] || { echo "  expected exit 0 (NO-OP), got $rc"; return 1; }
    echo "$out" | grep -qE "Katana.*недоступен|шаг 4 скип" || {
        echo "  expected WARN about Katana unreachable (out: $out)"; return 1; }
    return 0
}

# ============================================================================
# S5. dry-run + missing → exit 0, никаких ssh-вызовов (мы бы заметили по rc)
# ============================================================================
test_s5_dry_run_missing() {
    unset MOCK_KATANA_FAIL MOCK_ROBOT_LOGS MOCK_PAPLAY_CALLS
    unset MOCK_KATANA_FILE_EXISTS__tmp_cmd_muzika_i_filosofiya_wav
    unset MOCK_KATANA_FILE_EXISTS__tmp_cmd_dlinnyi_zapros_wav

    local result rc
    result="$(run_smoke "")"
    rc="$(echo "$result" | head -1 | sed 's/__RC__=//')"
    [ "$rc" = "0" ] || { echo "  expected exit 0, got $rc"; return 1; }
    return 0
}

# ============================================================================
# S6. override SMOKE_COMMANDS — кастомный список, один файл есть
# ============================================================================
test_s6_override() {
    unset MOCK_KATANA_FAIL MOCK_ROBOT_LOGS
    export MOCK_KATANA_FILE_EXISTS__tmp_my_custom_wav="yes"
    unset MOCK_KATANA_FILE_EXISTS__tmp_other_wav
    export MOCK_PAPLAY_CALLS="$WORK/paplay6.log"
    rm -f "$MOCK_PAPLAY_CALLS"

    local result rc out
    # extra_env — bash-код, source'ится как есть. SMOKE_COMMANDS как массив.
    result="$(run_smoke 'SMOKE_COMMANDS=( "/tmp/my_custom.wav|hello" "/tmp/other.wav|world" )')"
    rc="$(echo "$result" | head -1 | sed 's/__RC__=//')"
    out="$(echo "$result" | tail -n +2)"

    [ "$rc" = "0" ] || { echo "  expected exit 0, got $rc (out: $out)"; return 1; }
    echo "$out" | grep -q "missing: /tmp/other.wav" || {
        echo "  expected WARN about /tmp/other.wav missing (out: $out)"; return 1; }
    return 0
}

# --- main ------------------------------------------------------------------
run_test "S1_all_missing"        test_s1_all_missing
run_test "S2_one_missing"        test_s2_one_missing
run_test "S3_all_present_ok"     test_s3_all_present_robot_ok
run_test "S4_katana_unreachable" test_s4_katana_unreachable
run_test "S5_dry_run_missing"    test_s5_dry_run_missing
run_test "S6_override"           test_s6_override

echo
printf 'Tests: %d total, %d passed, %d failed\n' \
    "$TESTS_TOTAL" "$TESTS_PASSED" "$TESTS_FAILED"
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf 'Failed: %s\n' "${FAILED_NAMES[*]}"
    exit 1
fi
exit 0
