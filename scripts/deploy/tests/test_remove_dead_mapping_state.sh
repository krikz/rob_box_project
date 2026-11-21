#!/usr/bin/env bash
# Регресс-тест issue #5045: деплой Main Pi убирает мёртвый mapping_state.json
# из тома карт и пишет об этом в лог. rtabmap.db не трогает.
#
# 1. scripts/deploy/remove_dead_mapping_state.sh:
#    - файла нет / каталога нет ⇒ exit 0, строка «нет» в вывод;
#    - файл есть ⇒ exit 0, файл исчез, строка «удалён» в вывод, соседний
#      rtabmap.db на месте;
#    - повторный запуск ⇒ exit 0 (идемпотентно);
#    - цель — симлинк или каталог ⇒ exit 1, ничего не удалено;
#    - без аргумента ⇒ exit 1.
#    - rm получает Permission denied, файл остаётся, docker exec rtabmap
#      с mount /maps на этот каталог удаляет его ⇒ exit 0, docker вызван
#      узко (exec rtabmap rm -f /maps/mapping_state.json), docker run не зван;
#    - тот же rm, но контейнера нет, а локальный образ есть ⇒ exit 0 через
#      docker run --pull never, без pull;
#    - rm и docker не смогли ⇒ exit 1, файл на месте.
# 2. ".github/workflows/L-Deploy and Verify.yml": шаг чистки стоит на Main Pi
#    ДО «[Main Pi] Stop Containers», зовёт этот скрипт и не маскируется
#    `|| echo` / continue-on-error.
#
# Запуск: bash scripts/deploy/tests/test_remove_dead_mapping_state.sh

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SCRIPT="$REPO_ROOT/scripts/deploy/remove_dead_mapping_state.sh"
WORKFLOW="$REPO_ROOT/.github/workflows/L-Deploy and Verify.yml"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

pass=0
failures=0
ok() { echo "PASS: $1"; pass=$((pass + 1)); }
ko() { echo "FAIL: $1"; failures=$((failures + 1)); }

# --- скрипт: нет файла -------------------------------------------------------
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && grep -q 'нет' <<<"$out"; then
    ok "каталога нет — exit 0 и строка в лог"
else
    ko "каталога нет: rc=$rc out=$out"
fi

mkdir -p "$WORK/maps"
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && grep -q 'отсутствует' <<<"$out"; then
    ok "файла нет — exit 0 и строка «отсутствует»"
else
    ko "файла нет: rc=$rc out=$out"
fi

# --- скрипт: файл есть, сосед жив -------------------------------------------
printf '%s\n' '{"map_name":"квартира","mode":"localization"}' > "$WORK/maps/mapping_state.json"
printf '%s\n' 'rtabmap-db-sentinel' > "$WORK/maps/rtabmap.db"
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && [ ! -e "$WORK/maps/mapping_state.json" ] && grep -q 'удалён' <<<"$out"; then
    ok "файл был — удалён, exit 0, строка «удалён»"
else
    ko "файл был: rc=$rc exists=$([ -e "$WORK/maps/mapping_state.json" ] && echo yes || echo no) out=$out"
fi
if [ "$(cat "$WORK/maps/rtabmap.db")" = "rtabmap-db-sentinel" ]; then
    ok "rtabmap.db не тронут"
else
    ko "rtabmap.db изменён или пропал"
fi

# --- идемпотентность ---------------------------------------------------------
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && grep -q 'отсутствует' <<<"$out"; then
    ok "повторный запуск — exit 0, нечего удалять"
else
    ko "повторный запуск: rc=$rc out=$out"
fi

# --- не обычный файл: не трогаем --------------------------------------------
ln -s rtabmap.db "$WORK/maps/mapping_state.json"
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 1 ] && [ -L "$WORK/maps/mapping_state.json" ]; then
    ok "симлинк — exit 1, ссылка на месте"
else
    ko "симлинк: rc=$rc out=$out"
fi
rm -f "$WORK/maps/mapping_state.json"

mkdir "$WORK/maps/mapping_state.json"
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 1 ] && [ -d "$WORK/maps/mapping_state.json" ]; then
    ok "каталог с тем же именем — exit 1, на месте"
else
    ko "каталог-цель: rc=$rc out=$out"
fi
rmdir "$WORK/maps/mapping_state.json"

# --- rm Permission denied: фолбэк через docker (фейки в PATH, без демона) ---
# Фейковый rm удаляет, только если рядом нет файла .rm-deny. Фейковый docker
# пишет argv в docker.log и удаляет файл на хосте, только если сценарий
# положил .docker-exec-ok / .docker-run-ok.
FAKE="$(mktemp -d)"
cat > "$FAKE/rm" << 'EOF'
#!/usr/bin/env bash
# Фейк rm: -f <путь>. Если рядом .rm-deny — Permission denied, файл не трогаем.
set -u
target=""
for arg in "$@"; do
    case "$arg" in
        -*) ;;
        *) target="$arg" ;;
    esac
done
dir="$(dirname "$target")"
if [ -n "$target" ] && [ -f "$dir/.rm-deny" ]; then
    echo "rm: cannot remove '$target': Permission denied" >&2
    exit 1
fi
if [ -n "$target" ]; then
    /bin/rm -f -- "$target"
fi
exit 0
EOF
cat > "$FAKE/docker" << 'EOF'
#!/usr/bin/env bash
# Фейк docker. Сценарий лежит в $FAKE_SCENARIO:
#   running=true|false
#   source=<путь Source для Destination=/maps, пусто — mount нет>
#   image=<Config.Image или пусто>
#   image_present=yes|no
#   images_line=<строка docker images или пусто>
#   exec_removes=yes|no
#   run_removes=yes|no
set -u
log="${FAKE_DOCKER_LOG:?}"
printf '%s\n' "$*" >> "$log"
scenario="${FAKE_SCENARIO:?}"
val() { sed -n "s/^$1=//p" "$scenario" | head -1; }

cmd="${1:-}"
shift || true
case "$cmd" in
    inspect)
        fmt=""
        while [ $# -gt 0 ]; do
            case "$1" in
                -f) fmt="$2"; shift 2 ;;
                *) shift ;;
            esac
        done
        name="${1:-}"
        case "$fmt" in
            *State.Running*)
                printf '%s\n' "$(val running)"
                ;;
            *Destination*)
                src="$(val source)"
                if [ -n "$src" ]; then
                    printf '/config\n/config-src\n/maps\n%s\n' "$src"
                fi
                ;;
            *Config.Image*)
                img="$(val image)"
                if [ -n "$img" ]; then
                    printf '%s\n' "$img"
                fi
                ;;
            *)
                exit 1
                ;;
        esac
        exit 0
        ;;
    "image")
        sub="${1:-}"
        shift || true
        if [ "$sub" = "inspect" ]; then
            if [ "$(val image_present)" = "yes" ]; then
                exit 0
            fi
            echo "Error: No such image" >&2
            exit 1
        fi
        exit 1
        ;;
    images)
        line="$(val images_line)"
        if [ -n "$line" ]; then
            printf '%s\n' "$line"
        fi
        exit 0
        ;;
    exec)
        # Ждём узкий вызов: exec rtabmap rm -f /maps/mapping_state.json
        if [ "$(val exec_removes)" = "yes" ]; then
            host="$(val host_target)"
            /bin/rm -f -- "$host"
        fi
        exit 0
        ;;
    run)
        if [ "$(val run_removes)" = "yes" ]; then
            host="$(val host_target)"
            /bin/rm -f -- "$host"
        fi
        exit 0
        ;;
    *)
        echo "fake docker: неизвестная команда $cmd" >&2
        exit 1
        ;;
esac
EOF
chmod +x "$FAKE/rm" "$FAKE/docker"

write_scenario() {
    local file="$1"
    shift
    : > "$file"
    for kv in "$@"; do
        printf '%s\n' "$kv" >> "$file"
    done
}

run_with_fakes() {
    local scenario="$1"
    local maps="$2"
    FAKE_SCENARIO="$scenario" FAKE_DOCKER_LOG="$FAKE/docker.log" \
        PATH="$FAKE:$PATH" bash "$SCRIPT" "$maps"
}

# (в) rm Permission denied + docker exec с mount на этот каталог
exec_maps="$WORK/exec-maps"
mkdir -p "$exec_maps"
printf '%s\n' '{"map_name":"квартира"}' > "$exec_maps/mapping_state.json"
printf '%s\n' 'rtabmap-db-sentinel' > "$exec_maps/rtabmap.db"
: > "$exec_maps/.rm-deny"
canon_exec="$(cd "$exec_maps" && pwd -P)"
: > "$FAKE/docker.log"
write_scenario "$FAKE/scenario" \
    "running=true" \
    "source=$canon_exec" \
    "image=ghcr.io/krikz/rob_box:rtabmap-humble-local" \
    "image_present=yes" \
    "images_line=" \
    "exec_removes=yes" \
    "run_removes=no" \
    "host_target=$exec_maps/mapping_state.json"
out=$(run_with_fakes "$FAKE/scenario" "$exec_maps" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && [ ! -e "$exec_maps/mapping_state.json" ] && grep -q 'удалён' <<<"$out"; then
    ok "rm denied + docker exec — exit 0, файл исчез"
else
    ko "rm denied + docker exec: rc=$rc exists=$([ -e "$exec_maps/mapping_state.json" ] && echo yes || echo no) out=$out"
fi
if [ "$(cat "$exec_maps/rtabmap.db")" = "rtabmap-db-sentinel" ]; then
    ok "при docker exec соседний rtabmap.db не тронут"
else
    ko "при docker exec rtabmap.db изменён"
fi
if grep -qx 'exec rtabmap rm -f /maps/mapping_state.json' "$FAKE/docker.log"; then
    ok "docker exec вызван узко: rtabmap rm -f /maps/mapping_state.json"
else
    ko "узкий docker exec не найден в логе: $(cat "$FAKE/docker.log")"
fi
if grep -q '^run ' "$FAKE/docker.log"; then
    ko "docker run вызван, хотя exec уже убрал файл: $(cat "$FAKE/docker.log")"
else
    ok "docker run не вызван, когда exec справился"
fi

# mount /maps указывает на другой каталог — exec не зовём, идём в docker run
other_maps="$WORK/other-maps"
mkdir -p "$other_maps"
printf '%s\n' '{"map_name":"квартира"}' > "$other_maps/mapping_state.json"
: > "$other_maps/.rm-deny"
: > "$FAKE/docker.log"
write_scenario "$FAKE/scenario" \
    "running=true" \
    "source=/tmp/not-this-maps" \
    "image=ghcr.io/krikz/rob_box:rtabmap-humble-local" \
    "image_present=yes" \
    "images_line=" \
    "exec_removes=yes" \
    "run_removes=yes" \
    "host_target=$other_maps/mapping_state.json"
out=$(run_with_fakes "$FAKE/scenario" "$other_maps" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && [ ! -e "$other_maps/mapping_state.json" ]; then
    ok "чужой mount /maps — exec не зван, docker run убрал файл"
else
    ko "чужой mount: rc=$rc out=$out log=$(cat "$FAKE/docker.log")"
fi
if grep -q '^exec ' "$FAKE/docker.log"; then
    ko "docker exec вызван при чужом Source: $(cat "$FAKE/docker.log")"
else
    ok "docker exec не вызван, когда Source не этот каталог"
fi
if grep -q -- '--pull never' "$FAKE/docker.log" && grep -q '/maps/mapping_state.json' "$FAKE/docker.log"; then
    ok "docker run с --pull never и узким /maps/mapping_state.json"
else
    ko "docker run не узкий: $(cat "$FAKE/docker.log")"
fi

# контейнера нет, образ берём из docker images
images_maps="$WORK/images-maps"
mkdir -p "$images_maps"
printf '%s\n' '{"map_name":"квартира"}' > "$images_maps/mapping_state.json"
: > "$images_maps/.rm-deny"
: > "$FAKE/docker.log"
write_scenario "$FAKE/scenario" \
    "running=false" \
    "source=" \
    "image=" \
    "image_present=no" \
    "images_line=ghcr.io/krikz/rob_box:rtabmap-humble-cached" \
    "exec_removes=no" \
    "run_removes=yes" \
    "host_target=$images_maps/mapping_state.json"
out=$(run_with_fakes "$FAKE/scenario" "$images_maps" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && [ ! -e "$images_maps/mapping_state.json" ] \
    && grep -q 'rtabmap-humble-cached' "$FAKE/docker.log"; then
    ok "контейнера нет — docker run из локального docker images, exit 0"
else
    ko "docker images fallback: rc=$rc out=$out log=$(cat "$FAKE/docker.log")"
fi

# (г) rm и docker провалились — exit 1, файл на месте
fail_maps="$WORK/fail-maps"
mkdir -p "$fail_maps"
printf '%s\n' '{"map_name":"квартира"}' > "$fail_maps/mapping_state.json"
: > "$fail_maps/.rm-deny"
: > "$FAKE/docker.log"
write_scenario "$FAKE/scenario" \
    "running=false" \
    "source=" \
    "image=" \
    "image_present=no" \
    "images_line=" \
    "exec_removes=no" \
    "run_removes=no" \
    "host_target=$fail_maps/mapping_state.json"
out=$(run_with_fakes "$FAKE/scenario" "$fail_maps" 2>&1)
rc=$?
if [ "$rc" -eq 1 ] && [ -f "$fail_maps/mapping_state.json" ] && grep -q '::error' <<<"$out"; then
    ok "rm и docker провалились — exit 1, файл на месте, ::error"
else
    ko "двойной провал: rc=$rc exists=$([ -f "$fail_maps/mapping_state.json" ] && echo yes || echo no) out=$out"
fi

# docker exec вызван, но файл не исчез, образа нет — тоже exit 1
stuck_maps="$WORK/stuck-maps"
mkdir -p "$stuck_maps"
printf '%s\n' '{"map_name":"квартира"}' > "$stuck_maps/mapping_state.json"
: > "$stuck_maps/.rm-deny"
canon_stuck="$(cd "$stuck_maps" && pwd -P)"
: > "$FAKE/docker.log"
write_scenario "$FAKE/scenario" \
    "running=true" \
    "source=$canon_stuck" \
    "image=" \
    "image_present=no" \
    "images_line=" \
    "exec_removes=no" \
    "run_removes=no" \
    "host_target=$stuck_maps/mapping_state.json"
out=$(run_with_fakes "$FAKE/scenario" "$stuck_maps" 2>&1)
rc=$?
if [ "$rc" -eq 1 ] && [ -f "$stuck_maps/mapping_state.json" ] \
    && grep -qx 'exec rtabmap rm -f /maps/mapping_state.json' "$FAKE/docker.log"; then
    ok "exec не убрал файл и образа нет — exit 1, файл на месте"
else
    ko "exec-не-убрал: rc=$rc out=$out log=$(cat "$FAKE/docker.log")"
fi

rm -rf "$FAKE"

# --- без аргумента -----------------------------------------------------------
out=$(bash "$SCRIPT" 2>&1)
rc=$?
if [ "$rc" -eq 1 ]; then
    ok "без аргумента — exit 1"
else
    ko "без аргумента: rc=$rc out=$out"
fi

# --- шаг в workflow ----------------------------------------------------------
step_line=$(grep -n 'Remove dead mapping_state.json (#5045)' "$WORKFLOW" | head -1 | cut -d: -f1)
stop_line=$(grep -n '\[Main Pi\] Stop Containers' "$WORKFLOW" | head -1 | cut -d: -f1)
if [ -n "$step_line" ] && [ -n "$stop_line" ] && [ "$step_line" -lt "$stop_line" ]; then
    ok "шаг чистки (#5045) стоит до [Main Pi] Stop Containers ($step_line < $stop_line)"
else
    ko "порядок шагов: cleanup=$step_line stop=$stop_line"
fi

if grep -q 'scripts/deploy/remove_dead_mapping_state.sh' "$WORKFLOW"; then
    ok "workflow зовёт remove_dead_mapping_state.sh"
else
    ko "workflow не зовёт скрипт"
fi

# Блок шага: от его имени до следующего шага «- name:» на том же отступе.
# Не должен глотать ошибку своим || echo или continue-on-error.
block=$(awk '/Remove dead mapping_state.json \(#5045\)/{p=1; next} p && /^      - name: /{exit} p' "$WORKFLOW")
if grep -q '|| echo' <<<"$block" || grep -q 'continue-on-error' <<<"$block"; then
    ko "шаг чистки маскирует ошибку: $block"
else
    ok "шаг чистки не маскирует ошибку (нет || echo / continue-on-error)"
fi

echo "---"
echo "PASS=$pass FAIL=$failures"
[ "$failures" -eq 0 ]
