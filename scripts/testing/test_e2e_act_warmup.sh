#!/usr/bin/env bash
# Unit-тест warm_act_audio_path() — issue #3886 (потеря «Робот,» в первой фразе акта 2).
# Офлайн: ffmpeg/paplay/sleep подменены функциями-стабами, робот не нужен.
# Запуск: bash scripts/testing/test_e2e_act_warmup.sh   Exit: 0 = PASS, N = число упавших.
set -u
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LIB="$SCRIPT_DIR/../../.github/workflows/scripts/e2e_voice_lib.sh"
HARNESS="$SCRIPT_DIR/../../.github/workflows/scripts/e2e_voice_test.sh"
[ -f "$LIB" ] || { echo "нет $LIB"; exit 1; }
# shellcheck disable=SC1090
source "$LIB" 2>/dev/null
type warm_act_audio_path >/dev/null 2>&1 || { echo "FATAL: warm_act_audio_path не определена"; exit 2; }

FAIL=0
ok()  { echo "OK   $1"; }
bad() { echo "FAIL $1"; FAIL=$((FAIL + 1)); }

CALLS=""
ffmpeg() { CALLS="$CALLS ffmpeg"; : > "${@: -1}"; return "${STUB_FFMPEG_RC:-0}"; }
paplay() { CALLS="$CALLS paplay"; return "${STUB_PAPLAY_RC:-0}"; }
sleep()  { CALLS="$CALLS sleep:$1"; return 0; }
tmp="$(mktemp -d)"

# 1. по умолчанию: тишина через sink, потом пауза 15 с
CALLS=""; unset E2E_ACT_WARMUP_SEC
warm_act_audio_path "$tmp" >/dev/null
case "$CALLS" in *ffmpeg*paplay*sleep:15*) ok "default: ffmpeg -> paplay -> sleep 15" ;; *) bad "default: calls=[$CALLS]" ;; esac

# 2. выключено
CALLS=""; E2E_ACT_WARMUP_SEC=0 warm_act_audio_path "$tmp" >/dev/null
[ -z "$CALLS" ] && ok "E2E_ACT_WARMUP_SEC=0: ничего не делает" || bad "off: calls=[$CALLS]"

# 3. paplay упал -> пауза всё равно, rc=0, в логе честно сказано
CALLS=""; STUB_PAPLAY_RC=1 E2E_ACT_WARMUP_SEC=3 warm_act_audio_path "$tmp" >"$tmp/o"; rc=$?
if [ "$rc" = 0 ] && grep -q "не вышло" "$tmp/o" && case "$CALLS" in *sleep:3*) true ;; *) false ;; esac; then
    ok "paplay FAIL: rc=0, пауза есть, лог честный"
else
    bad "paplay fail: rc=$rc calls=[$CALLS] out=$(cat "$tmp/o")"
fi

# 4. мусор в переменной -> дефолт 15
CALLS=""; E2E_ACT_WARMUP_SEC=abc warm_act_audio_path "$tmp" >/dev/null
case "$CALLS" in *sleep:15*) ok "мусор в E2E_ACT_WARMUP_SEC -> 15" ;; *) bad "garbage: [$CALLS]" ;; esac

# 5. харнесс реально вызывает прогрев после остановки музыки, только для v2
if grep -A3 'ensure_music_stopped_for_act "\$_music_inherits"' "$HARNESS" \
    | grep -q 'E2E_DIALOG_ENGINE:-v1}" = "v2" \] && warm_act_audio_path'; then
    ok "харнесс зовёт прогрев после музыки для v2"
else
    bad "вызов прогрева в харнессе не найден"
fi

rm -rf "$tmp"
echo "failed=$FAIL"
exit "$FAIL"
