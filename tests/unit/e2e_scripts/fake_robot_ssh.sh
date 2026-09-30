#!/bin/bash
# fake_robot_ssh.sh — подставной робот для ROBOT_SSH_OVERRIDE (ADR-0057, PR #3211).
#
# e2e_voice_test.sh зовёт ``${ROBOT_SSH} "<команда для удалённого shell>"``
# (один аргумент). Этот фейк исполняет ту же строку локальным bash, но
# ``docker`` и ``ros2`` в нём — фейки с СОСТОЯНИЕМ в каталоге
# $FAKE_ROBOT_STATE. Строка разбирается так же, как её разбирал бы shell
# робота, поэтому robot_ros() (printf %q + eval, ADR-0129) проходит
# настоящие два уровня разбора, а не матчится подстрокой.
#
# Семантика параметров — та же, что у FAKE_ROBOT в
# test_issue_2890_act_isolation.py (изоляция акта #2890, 54d1a6f8):
#   * параметр узла лежит файлом $FAKE_ROBOT_STATE/param_<node><name>;
#   * ``ros2 param set <node> e2e_mode true`` при прежнем false пишет в
#     $FAKE_ROBOT_STATE/events строку ``WIPE <node>`` (узел стирает e2e-базу);
#   * ``ros2 param set /dialogue_node e2e_session_reset_token <t>`` пишет
#     ``RESET <node> <t>`` и сохраняет токен; при FAKE_DIALOGUE_REJECT=1
#     значение ОТКЛОНЯЕТСЯ, как отклонил бы parameters_callback при
#     провале сброса (печать «Set parameter failed», токен не меняется) —
#     харнесс обязан прочитать токен обратно и упасть E2E_FATAL;
#   * ``ros2 param get`` печатает то, что печатает ros2cli:
#     «Boolean value is: True|False» / «String value is: <v>».
#
# Логи контейнера voice-assistant (``docker logs voice-assistant ...``) —
# содержимое $FAKE_ROBOT_LOG (если задан). ``echo X > /proc/1/fd/1`` внутри
# ``docker exec voice-assistant`` дописывается в $FAKE_ROBOT_STATE/pid1_stdout
# (НЕ в /proc/1/fd/1 машины, на которой идёт тест). Каждая строка вызова
# журналируется в $FAKE_ROBOT_STATE/ssh_calls.
#
# Всё, что фейк не умеет, — громкая ошибка (rc=97 + stderr), а не тихий успех.
#
# Env:
#   FAKE_ROBOT_STATE     — каталог состояния (обязателен)
#   FAKE_ROBOT_LOG       — файл «docker logs voice-assistant» (необязателен)
#   FAKE_DIALOGUE_REJECT — 1: dialogue_node отклоняет e2e_session_reset_token
set -u
: "${FAKE_ROBOT_STATE:?fake_robot_ssh.sh: FAKE_ROBOT_STATE не задан}"
mkdir -p "$FAKE_ROBOT_STATE"
printf '%s\n' "$*" >> "$FAKE_ROBOT_STATE/ssh_calls"

_unsupported() {
    echo "fake_robot_ssh: не поддержано: $*" >&2
    echo "UNSUPPORTED $*" >> "$FAKE_ROBOT_STATE/events"
    return 97
}

_pfile() {  # $1=node $2=name
    printf '%s/param_%s\n' "$FAKE_ROBOT_STATE" "$(printf '%s%s' "$1" "$2" | tr '/' '_')"
}

ros2() {  # ros2 param set|get <node> <name> [value] [--no-daemon]
    [ "${1:-}" = "param" ] || { _unsupported "ros2 $*"; return; }
    local verb="${2:-}" node="${3:-}" name="${4:-}" value="${5:-}" f was
    f="$(_pfile "$node" "$name")"
    case "$verb:$name" in
        set:e2e_mode)
            was="$(cat "$f" 2>/dev/null || echo false)"
            if [ "$was" = "false" ] && [ "$value" = "true" ]; then
                echo "WIPE $node" >> "$FAKE_ROBOT_STATE/events"
            fi
            echo "$value" > "$f"
            echo "Set parameter successful"
            ;;
        set:e2e_session_reset_token)
            if [ "${FAKE_DIALOGUE_REJECT:-0}" = "1" ]; then
                echo "Set parameter failed: issue 2890: e2e session reset failed"
                return 0
            fi
            echo "RESET $node $value" >> "$FAKE_ROBOT_STATE/events"
            echo "$value" > "$f"
            echo "Set parameter successful"
            ;;
        get:e2e_mode)
            if [ "$(cat "$f" 2>/dev/null || echo false)" = "true" ]; then
                echo "Boolean value is: True"
            else
                echo "Boolean value is: False"
            fi
            ;;
        get:e2e_session_reset_token)
            echo "String value is: $(cat "$f" 2>/dev/null)"
            ;;
        *)
            _unsupported "ros2 $*"
            ;;
    esac
}

docker() {
    local sub="${1:-}" container="${2:-}"
    case "$sub:$container" in
        logs:voice-assistant)
            [ -n "${FAKE_ROBOT_LOG:-}" ] && cat "$FAKE_ROBOT_LOG"
            return 0
            ;;
        exec:voice-assistant)
            # docker exec voice-assistant bash -lc|-c <cmd>
            [ "${3:-}" = "bash" ] || { _unsupported "docker $*"; return; }
            local cmd="${5:-}"
            cmd="${cmd//\/proc\/1\/fd\/1/$FAKE_ROBOT_STATE/pid1_stdout}"
            # source ROS-окружения на роботе — в фейке no-op (вызывается
            # из $cmd через eval, shellcheck этого не видит).
            # shellcheck disable=SC2329
            ( source() { :; }; eval "$cmd" )
            ;;
        logs:* | exec:* | ps:*)
            # Других контейнеров (telegram-bot и т.п.) на фейк-роботе нет.
            return 1
            ;;
        *)
            _unsupported "docker $*"
            ;;
    esac
}

eval "$1"
