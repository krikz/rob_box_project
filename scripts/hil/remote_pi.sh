#!/usr/bin/env bash
# Помощник hil.sh на стороне Pi (Main или Vision). hil.sh копирует его в /tmp/hil/
# и вызывает по ssh одной подкомандой — так в ssh не уходят многоэтажные кавычки.
# План: docs/research/2026-11-04-hil-robot-on-gazebo-data.md §1.3, эпик #4371.
#
#   remote_pi.sh lock-take <файл> <текст>    атомарно создать замок; есть — код 4
#   remote_pi.sh lock-show <файл>            содержимое замка или «нет»
#   remote_pi.sh lock-release <файл>         снять замок
#   remote_pi.sh pin <compose-каталог> <out> оверлей image: <образ контейнера> для всех
#                                            сервисов compose-проекта этого каталога
#   remote_pi.sh stop <контейнер>...         docker stop; проверка, что не работает
#   remote_pi.sh state <контейнер>...        «имя running|exited|absent USE_SIM_TIME=…»
#   remote_pi.sh compose <каталог> <args>... docker compose в каталоге
#   remote_pi.sh env <контейнер> <ПЕРЕМЕННАЯ> значение env контейнера или «unset»
#   remote_pi.sh maps-dump <контейнер> <БД>  SELECT * FROM maps ORDER BY id (строки a|b|c)
#                                            из SQLite внутри контейнера, ТОЛЬКО чтение
#                                            (mode=ro); нет таблицы/файла — код 7
#
# Боевые данные (/data, ./maps) скрипт не пишет. Читает одно: maps-dump — боевой реестр
# карт Sound Pi для сверки «до on / после off» (#4392), открывается только на чтение.
set -euo pipefail

cmd="${1:-}"
shift || true

state_of() {
    local name="$1" st
    st=$(docker inspect -f '{{.State.Status}}' "$name" 2>/dev/null) || st=absent
    local simt=""
    if [ "$st" != absent ]; then
        simt=$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$name" \
            | sed -n 's/^USE_SIM_TIME=//p' | tail -n 1)
    fi
    echo "$name $st USE_SIM_TIME=${simt:-unset}"
}

case "$cmd" in
    lock-take)
        lock="$1"; text="$2"
        if ! ( set -C; printf '%s\n' "$text" > "$lock" ) 2>/dev/null; then
            echo "замок $lock уже есть: $(cat "$lock" 2>/dev/null)" >&2
            exit 4
        fi
        echo "замок $lock взят"
        ;;
    lock-show)
        if [ -e "$1" ]; then cat "$1"; else echo "нет"; fi
        ;;
    lock-release)
        rm -f -- "$1"
        echo "замок $1 снят"
        ;;
    pin)
        dir="$1"; out="$2"
        mkdir -p "$(dirname "$out")"
        docker ps -a --filter "label=com.docker.compose.project.working_dir=$dir" \
            --format '{{.Label "com.docker.compose.service"}} {{.Image}}' \
            | awk 'BEGIN { print "services:" } NF == 2 { printf "  %s:\n    image: \"%s\"\n", $1, $2 }' \
            > "$out"
        n=$(grep -c 'image:' "$out" || true)
        if [ "$n" -eq 0 ]; then
            echo "в $dir нет контейнеров compose — нечего закреплять (стек не поднят?)" >&2
            exit 5
        fi
        echo "закреплено образов: $n ($out)"
        ;;
    stop)
        for name in "$@"; do
            docker stop --time 20 "$name" >/dev/null 2>&1 || true
            st=$(docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null || echo absent)
            if [ "$st" = true ]; then
                echo "❌ $name всё ещё работает после docker stop" >&2
                exit 6
            fi
            echo "$name остановлен ($st)"
        done
        ;;
    state)
        for name in "$@"; do state_of "$name"; done
        ;;
    compose)
        dir="$1"; shift
        cd "$dir"
        docker compose "$@"
        ;;
    env)
        name="$1"; var="$2"
        val=$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$name" 2>/dev/null \
            | sed -n "s/^$var=//p" | tail -n 1)
        echo "${val:-unset}"
        ;;
    maps-dump)
        name="$1"; db="$2"
        # Файла нет — sqlite в mode=ro не создаёт его, а падает: «нечего сверять» не путается с «пусто».
        if ! docker exec "$name" python3 -c '
import sqlite3, sys
conn = sqlite3.connect("file:" + sys.argv[1] + "?mode=ro", uri=True)
for row in conn.execute("SELECT * FROM maps ORDER BY id"):
    print("|".join("" if v is None else str(v) for v in row))
' "$db"; then
            echo "❌ maps-dump: не прочитал maps из $db в $name" >&2
            exit 7
        fi
        ;;
    *)
        sed -n '2,20p' "$0" >&2
        exit 2
        ;;
esac
