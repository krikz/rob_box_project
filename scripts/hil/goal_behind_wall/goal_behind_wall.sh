#!/usr/bin/env bash
# Сценарий HIL «цель за стеной» (#5003).
#
#   scripts/hil/goal_behind_wall/goal_behind_wall.sh
#
# Мир — штатный dead_end (worlds_catalog): спавн (0, 0) лицом в торец (+X),
# зазор до торца 0.25 м. Цель — сразу за торцом, (0.75, 0), курс 0: внутри
# стены, не в 3 м пустоты за ней. Это не карман
# 0.05 м: назад от спавна 4.5 м коридора, боковой зазор корпуса на спавне
# 0.2475 м (выше stop_m 0.20 и выше проекции BackUp 0.05 м).
#
# Скрипт цель не шлёт и Gazebo не поднимает. Он только проверяет изоляцию и,
# если она есть, печатает команду прогона. Иначе выход 2, до любой цели:
#   * мир не Gazebo (HIL_SIM_WORLD не dead_end, контейнер gazebo не running,
#     в графе не ровно один /controller_manager, у cmd_vel_unstamped не 1
#     подписчик) — боевой контроллер ещё в графе;
#   * боевой ros2-control работает;
#   * на can0 есть командный кадр VESC (команда 0..8). Чтение candump, ничего
#     не шлёт. Кадров нет (candump пуст или его нет) — тоже выход 2: «ноль
#     команд» без чтения шины не доказан.
#
# HIL включает scripts/hil/hil.sh, не этот скрипт. Живой прогон — отдельное
# окно, не этот файл.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../../.." && pwd)"
if [ -n "${HIL_CONF:-}" ]; then
    # shellcheck source=/dev/null
    . "$HIL_CONF"
fi
# shellcheck disable=SC1091
. "$REPO_ROOT/scripts/hil/hil.conf"

: "${HIL_GBW_CAN_SECS:=2}"
: "${HIL_GBW_GOAL_X:=0.75}"
: "${HIL_GBW_GOAL_Y:=0.0}"
: "${HIL_GBW_GOAL_YAW:=0.0}"
: "${HIL_GBW_SPAWN:=0 0 0}"

GZ="${HIL_SIM_PREFIX}-gazebo"
SSHO=(-o BatchMode=yes -o ConnectTimeout=10)
CAN_WATCH="$REPO_ROOT/scripts/hil/speed/can_watch.py"
# Тот же топик, что hil.sh:433. В hil.conf его нет, set -u иначе падает.
CMD_VEL_UNSTAMPED="/diff_drive_controller/cmd_vel_unstamped"

fail() { printf 'goal-behind-wall: %s\n' "$*" >&2; exit 2; }
# $2 — текст команды для удалённой стороны; кавычки вокруг неё не дают
# локальному shell раскрыть $() внутри.
# shellcheck disable=SC2029
rsh() { ssh "${SSHO[@]}" "$1" "$2" </dev/null; }

# state_of печатает «имя running|exited|absent USE_SIM_TIME=…».
container_state() {
    rsh "$1" "docker inspect -f '{{.State.Status}}' '$2' 2>/dev/null || echo absent"
}

# candump -L: «(t) if id#data». Команда VESC — старший байт id. 0..8 — команда
# на мотор, 9 и выше — статус. Тот же порог, что can_watch.py (STATUS_1 = 9).
# mawk strtonum не знает, поэтому hex считает python, как can_watch.py.
vesc_command_frames() {
    python3 -c '
import sys
n = 0
for line in sys.stdin:
    ident = next((p for p in line.split() if "#" in p), "")
    if not ident:
        continue
    try:
        cmd = (int(ident.split("#", 1)[0], 16) >> 8) & 0xFF
    except ValueError:
        continue
    if cmd < 9:
        n += 1
print(n)
'
}

main() {
    [ "${HIL_SIM_WORLD}" = "dead_end" ] \
        || fail "мир HIL_SIM_WORLD=${HIL_SIM_WORLD}, ждали dead_end (не polygon и не карман 0.05 м)"

    st=$(container_state "$HIL_MAIN_HOST" ros2-control) || fail "нет ssh до $HIL_MAIN_HOST, состояние ros2-control не проверено"
    case "$st" in
        running*) fail "боевой ros2-control работает ($st)" ;;
        exited|absent) ;;
        *) fail "состояние ros2-control не доказано ($st)" ;;
    esac

    gz=$(container_state "$HIL_SANDBOX_HOST" "$GZ") || fail "нет ssh до $HIL_SANDBOX_HOST, gazebo не проверен"
    [ "$gz" = "running" ] || fail "gazebo не работает ($gz) — контроллер сима не единственный"

    status=$(rsh "$HIL_SANDBOX_HOST" \
        "docker exec '$GZ' /entrypoint.sh ros2 topic info '$CMD_VEL_UNSTAMPED' --verbose 2>/dev/null; echo CM_MARK; docker exec '$GZ' /entrypoint.sh ros2 node list 2>/dev/null" \
        ) || fail "проба графа gazebo не ответила"
    cm=$(printf '%s\n' "$status" | awk '/^CM_MARK$/{p=1;next} p && $0=="/controller_manager"{n++} END{print n+0}')
    [ "$cm" = "1" ] || fail "/controller_manager в графе: $cm (ждали 1, иначе боевой контроллер ещё здесь)"
    subs=$(printf '%s\n' "$status" | sed -n '1,/^CM_MARK$/p' | awk '/Subscription count:/{print $3; exit}')
    [ "$subs" = "1" ] || fail "$CMD_VEL_UNSTAMPED подписчиков: ${subs:-нет} (ждали 1)"

    # Только чтение. timeout оборвёт candump; кадр команды 0..8 — отказ.
    sample=$(rsh "$HIL_MAIN_HOST" \
        "timeout '$HIL_GBW_CAN_SECS' candump -t a -L can0 2>/dev/null || true" \
        ) || fail "can0 не прочитан (ssh до $HIL_MAIN_HOST не ответил)"
    [ -n "$sample" ] || fail "can0 пуст за ${HIL_GBW_CAN_SECS} с — ноль команд VESC не доказан"
    cmds=$(printf '%s\n' "$sample" | vesc_command_frames)
    [ "$cmds" = "0" ] || fail "на can0 $cmds командных кадров VESC (команда 0..8) — цель не шлю"

    # can_watch.py — тот же разбор, которым HIL доказывает «на моторы ничего не ушло».
    # Здесь он не запускается: шина уже прочитана, повторный candump не нужен.
    [ -f "$CAN_WATCH" ] || fail "нет $CAN_WATCH — разбор команд VESC нечем сверить"

    read -r sx sy syaw <<<"$HIL_GBW_SPAWN"
    printf 'goal-behind-wall: изоляция есть (gazebo running, ros2-control %s, /controller_manager 1, VESC команд 0)\n' "$st"
    printf 'мир: dead_end, спавн %s %s %s, цель за торцом %s %s %s\n' "$sx" "$sy" "$syaw" \
        "$HIL_GBW_GOAL_X" "$HIL_GBW_GOAL_Y" "$HIL_GBW_GOAL_YAW"
    printf 'прогон (этот скрипт его не запускает):\n'
    printf '  HIL_SIM_WORLD=dead_end scripts/hil/hil.sh on\n'
    printf '  # gazebo: world:=/sim_worlds/dead_end.sdf x:=%s y:=%s yaw:=%s\n' "$sx" "$sy" "$syaw"
    printf '  # цель NavigateToPose (%s, %s, yaw %s) — за торцом, не в кармане\n' \
        "$HIL_GBW_GOAL_X" "$HIL_GBW_GOAL_Y" "$HIL_GBW_GOAL_YAW"
    return 0
}

main "$@"
