#!/usr/bin/env bash
# Регресс-тест scripts/robot_dev/hotpatch.sh: dirty-флаг chuangong (фаза C, ADR-0168 §2.4).
#
# Ни робота, ни сети: SSH подменён массивом/фейком в PATH. Флаг пишется в
# CHUANGONG_DIRTY_FLAG (временный путь), не в /var/lib на машине разработчика.
#
# Что проверяется:
#   1. bash -n hotpatch.sh;
#   2. set_chuangong_dirty_flag через подмену SSH создаёт файл;
#   3. clear_chuangong_dirty_flag удаляет файл;
#   4. ветка push (PACKAGES= NO_RESTART=1) вызывает запись флага;
#   5. ветка revert снимает флаг;
#   6. status и logs флаг не ставят и не снимают;
#   7. ROBOT по умолчанию не сменён (ros2@10.1.1.22).
#
# Запуск: bash scripts/robot_dev/tests/test_hotpatch.sh

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
HOTPATCH="$REPO_ROOT/scripts/robot_dev/hotpatch.sh"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

pass=0
failures=0
ok() { echo "PASS: $1"; pass=$((pass + 1)); }
ko() { echo "FAIL: $1"; failures=$((failures + 1)); }
check() { if eval "$2"; then ok "$1"; else ko "$1"; fi; }

# ── 1. синтаксис ──────────────────────────────────────────────────────────
if bash -n "$HOTPATCH"; then ok "bash -n hotpatch.sh"; else ko "bash -n hotpatch.sh"; fi

# ── фейки ─────────────────────────────────────────────────────────────────
mkdir -p "$WORK/bin" "$WORK/flag-dir" "$WORK/remote" "$WORK/compose"
FLAG="$WORK/flag-dir/dirty"
export FAKE_SSH_LOG="$WORK/ssh.log"
export FAKE_DOCKER_LOG="$WORK/docker.log"
export FAKE_STATE="$WORK/remote"
: >"$FAKE_SSH_LOG"
: >"$FAKE_DOCKER_LOG"

cat >"$WORK/bin/ssh" <<'SSEOF'
#!/usr/bin/env bash
while [ "$#" -gt 0 ]; do
  case "$1" in -o) shift 2 ;; *) break ;; esac
done
host="$1"; shift
echo "$host|$*" >>"$FAKE_SSH_LOG"
exec bash -c "$*"
SSEOF
chmod +x "$WORK/bin/ssh"

cat >"$WORK/bin/docker" <<'DEOF'
#!/usr/bin/env bash
echo "docker $*" >>"$FAKE_DOCKER_LOG"
S="${FAKE_STATE:-/tmp}"
case "${1:-}" in
  inspect)
    if [[ "$*" == *Health.Status* ]]; then
      echo healthy
    elif [[ "$*" == *com.docker.compose.service* ]]; then
      echo voice-assistant
    else
      echo "img started=t health=healthy"
    fi
    exit 0
    ;;
  exec)
    shift
    [ "${1:-}" = "-i" ] && shift
    container="${1:-}"; shift || true
    if [ "${1:-}" = "test" ] && [ "${2:-}" = "-d" ]; then
      mkdir -p "$S$3"
      exit 0
    fi
    if [ "${1:-}" = "sh" ] && [ "${2:-}" = "-c" ]; then
      snippet="${3:-}"
      snippet="${snippet//\/ws/$S\/ws}"
      mkdir -p "$S/ws"
      bash -c "$snippet"
      exit $?
    fi
    exit 0
    ;;
  restart|compose|logs)
    [ "${1:-}" = logs ] && echo "fake log line"
    exit 0
    ;;
esac
exit 0
DEOF
chmod +x "$WORK/bin/docker"

# sleep no-op: revert ждёт 8с перед recreate voice-assistant
cat >"$WORK/bin/sleep" <<'ZEOF'
#!/usr/bin/env bash
exit 0
ZEOF
chmod +x "$WORK/bin/sleep"

export PATH="$WORK/bin:$PATH"
export CHUANGONG_DIRTY_FLAG="$FLAG"
export ROBOT="ros2@10.1.1.22"
export CONTAINER="voice-assistant"
export COMPOSE_DIR="$WORK/compose"
export NO_RESTART=1

# ── 2–3. функции set/clear с подменой SSH ─────────────────────────────────
SSH=("$WORK/bin/ssh" -o BatchMode=yes "$ROBOT")
export HOTPATCH_LIB_ONLY=1
# shellcheck disable=SC1090
source "$HOTPATCH"

rm -f "$FLAG"
: >"$FAKE_SSH_LOG"
set_chuangong_dirty_flag
check "set_chuangong_dirty_flag создаёт файл" '[ -f "$FLAG" ]'
check "set: содержимое hotpatch" 'grep -qx hotpatch "$FLAG"'
check "set: ssh ходил на ROBOT" 'grep -q "^ros2@10.1.1.22|" "$FAKE_SSH_LOG"'

: >"$FAKE_SSH_LOG"
clear_chuangong_dirty_flag
check "clear_chuangong_dirty_flag удаляет флаг" '[ ! -f "$FLAG" ]'
check "clear: ssh ходил на ROBOT" 'grep -q "^ros2@10.1.1.22|" "$FAKE_SSH_LOG"'

# ── 4. ветка push ставит флаг ─────────────────────────────────────────────
rm -f "$FLAG"
: >"$FAKE_SSH_LOG"
: >"$FAKE_DOCKER_LOG"
PACKAGES="" NO_RESTART=1 hotpatch_main push "$REPO_ROOT" >"$WORK/push.out" 2>"$WORK/push.err" || true
cat "$WORK/push.out" "$WORK/push.err" >"$WORK/push.log"
check "push создаёт dirty-флаг" '[ -f "$FLAG" ]'
check "push пишет метку (лога)" 'grep -q "метка:" "$WORK/push.log"'
check "push использует подмену SSH (log не пуст)" '[ -s "$FAKE_SSH_LOG" ]'
check "push ssh-host только ROBOT-строка" \
  '! grep -vE "^ros2@10\\.1\\.1\\.22\\|" "$FAKE_SSH_LOG" | grep -q .'

# ── 5. ветка revert снимает флаг ──────────────────────────────────────────
printf 'hotpatch\n' >"$FLAG"
: >"$FAKE_SSH_LOG"
hotpatch_main revert >"$WORK/revert.out" 2>"$WORK/revert.err" || true
check "revert снимает dirty-флаг" '[ ! -f "$FLAG" ]'
check "revert ходил через SSH" '[ -s "$FAKE_SSH_LOG" ]'

# ── 6. status/logs не трогают флаг ────────────────────────────────────────
printf 'keep\n' >"$FLAG"
: >"$FAKE_SSH_LOG"
hotpatch_main status >"$WORK/status.out" 2>&1 || true
check "status не удаляет флаг" '[ -f "$FLAG" ] && grep -qx keep "$FLAG"'
check "status не перезаписывает флаг содержимым hotpatch" '! grep -qx hotpatch "$FLAG"'

: >"$FAKE_SSH_LOG"
hotpatch_main logs 1m >"$WORK/logs.out" 2>&1 || true
check "logs не удаляет флаг" '[ -f "$FLAG" ] && grep -qx keep "$FLAG"'
check "logs не ставит hotpatch в флаг" '! grep -qx hotpatch "$FLAG"'

# ── 7. контракт в тексте скрипта ──────────────────────────────────────────
# grep -F по фиксированным фрагментам (без eval-кавычечной каши).
if grep -F 'ROBOT="${ROBOT:-ros2@10.1.1.22}"' "$HOTPATCH" >/dev/null; then
  ok "ROBOT default ros2@10.1.1.22 в hotpatch.sh"
else
  ko "ROBOT default ros2@10.1.1.22 в hotpatch.sh"
fi
if grep -F 'CHUANGONG_DIRTY_FLAG="${CHUANGONG_DIRTY_FLAG:-/var/lib/chuangong/dirty}"' "$HOTPATCH" >/dev/null; then
  ok "CHUANGONG_DIRTY_FLAG default /var/lib/chuangong/dirty"
else
  ko "CHUANGONG_DIRTY_FLAG default /var/lib/chuangong/dirty"
fi

# push/revert ветки вызывают set/clear; status/logs — нет.
push_block="$(awk '/^[[:space:]]*push\)/,/^[[:space:]]*;;/' "$HOTPATCH")"
revert_block="$(awk '/^[[:space:]]*revert\)/,/^[[:space:]]*;;/' "$HOTPATCH")"
status_block="$(awk '/^[[:space:]]*status\)/,/^[[:space:]]*;;/' "$HOTPATCH")"
logs_block="$(awk '/^[[:space:]]*logs\)/,/^[[:space:]]*;;/' "$HOTPATCH")"
if printf '%s\n' "$push_block" | grep -q set_chuangong_dirty_flag; then
  ok "push вызывает set_chuangong_dirty_flag"
else
  ko "push вызывает set_chuangong_dirty_flag"
fi
if printf '%s\n' "$revert_block" | grep -q clear_chuangong_dirty_flag; then
  ok "revert вызывает clear_chuangong_dirty_flag"
else
  ko "revert вызывает clear_chuangong_dirty_flag"
fi
if printf '%s\n' "$status_block" | grep -q chuangong_dirty_flag; then
  ko "status не вызывает set/clear dirty"
else
  ok "status не вызывает set/clear dirty"
fi
if printf '%s\n' "$logs_block" | grep -q chuangong_dirty_flag; then
  ko "logs не вызывает set/clear dirty"
else
  ok "logs не вызывает set/clear dirty"
fi

echo
echo "result: pass=$pass fail=$failures"
if [ "$failures" -ne 0 ]; then
  echo "---- debug ----"
  echo "FLAG=$FLAG exists=$([ -f "$FLAG" ] && echo yes || echo no)"
  echo "push.log:"; cat "$WORK/push.log" 2>/dev/null || true
  echo "revert:"; cat "$WORK/revert.out" 2>/dev/null; cat "$WORK/revert.err" 2>/dev/null || true
  echo "ssh.log:"; cat "$FAKE_SSH_LOG" 2>/dev/null || true
  exit 1
fi
exit 0
