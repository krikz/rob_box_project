#!/usr/bin/env bash
# studio_ro_gate.sh — forced command для ключа студии канала (issue #4492, решение Шифу 06.11).
#
# Ставится в authorized_keys хоста строкой
#   command="$HOME/.local/bin/studio_ro_gate.sh",no-pty,no-port-forwarding,no-agent-forwarding,
#   no-X11-forwarding,restrict ssh-ed25519 AAAA… studio
# (см. scripts/studio/install_ro_key.sh и docs/runbooks/studio-ro-access.md).
#
# sshd запускает гейт вместо любой команды клиента; что просил клиент — в $SSH_ORIGINAL_COMMAND.
# Гейт пропускает только чтение по белому списку и ничего больше:
#
#   ls [-lahtr1RS] <путь>…          stat <путь>…        cat <путь>…        sha256sum <путь>…
#   head [-n N] <путь>…             tail [-n N] <путь>…                     (N ≤ 100000, без -f)
#   scp [-p] [-v] -f <файл>         выдача одного файла клиенту (scp -O host:файл .)
#   docker logs [--timestamps] --tail N <контейнер>                         (N ≤ 2000)
#   git -C <репо> log|show|status [опции из белого списка] [ревизии] [-- пути]
#
# Конфиг на хосте (один элемент на строку, # — комментарий, абсолютные пути):
#   ~/.config/studio_ro/roots       корни, внутри которых разрешено чтение файлов;
#   ~/.config/studio_ro/containers  имена контейнеров для docker logs;
#   ~/.config/studio_ro/repos       git-репозитории для git -C.
# Лог: ~/.local/state/studio_ro/gate.log — «время<TAB>allow|deny<TAB>причина<TAB>клиент<TAB>команда».
#
# Как разбирается команда: никакого eval. Сначала проверка набора символов (только [A-Za-z0-9_./:=@+,-]
# и пробел), то есть кавычки, ; & | $ ` < > * ? \ ~ перевод строки отбиваются до разбора. Потом
# `read -ra` в массив (globbing выключен) и разбор по позициям. Каждый путь — realpath -e, затем
# проверка «внутри корня» и жёсткий запрет приватного (/data, *.db, ключи, .env), даже если такой
# корень по ошибке попал в конфиг. Команда исполняется над разрешённым путём, а не над исходной строкой.

set -uo pipefail
set -f
umask 077

readonly MAX_CMD_LEN=2048
readonly MAX_LOG_TAIL=2000
readonly MAX_HEAD_LINES=100000

CONF_DIR="$HOME/.config/studio_ro"
STATE_DIR="$HOME/.local/state/studio_ro"
LOG_FILE="$STATE_DIR/gate.log"
RAW_CMD="${SSH_ORIGINAL_COMMAND-}"
CLIENT="${SSH_CLIENT-}"
CLIENT="${CLIENT%% *}"
CLIENT="${CLIENT:-local}"

ROOTS=()
REPOS=()
CONTAINERS=()
RESOLVED=""
EXEC=()

# ── лог и решения ─────────────────────────────────────────────────────────

log_line() {
    # $1 — allow|deny, $2 — причина. Непечатаемые символы команды в логе заменяются на «?».
    local shown="${RAW_CMD:0:$MAX_CMD_LEN}"
    shown="${shown//[^[:print:]]/?}"
    mkdir -p "$STATE_DIR" 2>/dev/null || return 1
    printf '%s\t%s\t%s\t%s\t%s\n' "$(date -u +%FT%TZ)" "$1" "$2" "$CLIENT" "$shown" >>"$LOG_FILE" 2>/dev/null
}

deny() {
    log_line deny "$1" || true
    echo "studio_ro_gate: deny: $1" >&2
    exit 126
}

allow_and_exec() {
    # Без записи в лог команда не исполняется (fail closed).
    log_line allow "$1" || {
        echo "studio_ro_gate: deny: лог $LOG_FILE недоступен" >&2
        exit 126
    }
    exec "${EXEC[@]}"
}

# ── конфиг ────────────────────────────────────────────────────────────────

read_list() {
    # Печатает непустые строки файла $1 без комментариев и пробелов по краям.
    local line
    [[ -r "$1" ]] || return 0
    while IFS= read -r line || [[ -n "$line" ]]; do
        line="${line%%#*}"
        line="${line#"${line%%[![:space:]]*}"}"
        line="${line%"${line##*[![:space:]]}"}"
        [[ -n "$line" ]] && printf '%s\n' "$line"
    done <"$1"
}

is_private() {
    # Жёсткий запрет независимо от конфига: люди мастерской, секреты, системное.
    local p="$1" base="${1##*/}"
    case "$p" in
        / | /data | /data/* | */data/faces* | */data/voices* | /root | /root/* | /etc | /etc/* | \
            /proc | /proc/* | /sys | /sys/* | /dev | /dev/* | /run | /run/* | */.ssh | */.ssh/* | \
            */.config/studio_ro | */.config/studio_ro/* | */.local/state/studio_ro | */.local/state/studio_ro/*)
            return 0 ;;
    esac
    case "$base" in
        *.db | *.db-wal | *.db-shm | *.sqlite | *.sqlite3 | .env | .env.* | *.env | *.pem | *.key | \
            id_rsa* | id_ed25519* | id_ecdsa* | authorized_keys* | *.npy | *.wav | *.ogg | *.flac)
            return 0 ;;
    esac
    return 1
}

load_dirs() {
    # $1 — файл конфига, $2 — имя массива-приёмника. Берёт только существующие абсолютные каталоги,
    # нормализованные realpath, и не приватные. Плохая строка молча пропускается (видно в install).
    local line rp
    local -n out="$2"
    out=()
    while IFS= read -r line; do
        [[ "$line" == /* ]] || continue
        rp="$(realpath -e -- "$line" 2>/dev/null)" || continue
        [[ -d "$rp" ]] || continue
        is_private "$rp" && continue
        out+=("$rp")
    done < <(read_list "$1")
}

load_config() {
    load_dirs "$CONF_DIR/roots" ROOTS
    load_dirs "$CONF_DIR/repos" REPOS
    local line
    CONTAINERS=()
    while IFS= read -r line; do
        [[ "$line" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$ ]] && CONTAINERS+=("$line")
    done < <(read_list "$CONF_DIR/containers")
}

# ── проверки путей ────────────────────────────────────────────────────────

under_any() {
    # $1 — нормализованный путь, остальные — нормализованные корни.
    local p="$1" r
    shift
    for r in "$@"; do
        [[ "$p" == "$r" || "$p" == "$r/"* ]] && return 0
    done
    return 1
}

resolve_path() {
    # Пишет в RESOLVED нормализованный путь или отказывает. $1 — путь из команды.
    local p="$1"
    [[ "$p" == /* ]] || deny "путь не абсолютный: $p"
    is_private "$p" && deny "приватный путь: $p"
    RESOLVED="$(realpath -e -- "$p" 2>/dev/null)" || deny "нет такого пути: $p"
    is_private "$RESOLVED" && deny "приватный путь: $RESOLVED"
    ((${#ROOTS[@]})) || deny "нет разрешённых корней ($CONF_DIR/roots)"
    under_any "$RESOLVED" "${ROOTS[@]}" || deny "вне разрешённых корней: $p -> $RESOLVED"
    [[ -f "$RESOLVED" || -d "$RESOLVED" ]] || deny "не файл и не каталог: $RESOLVED"
}

# ── команды ───────────────────────────────────────────────────────────────

cmd_files() {
    # ls/stat/cat/head/tail/sha256sum: флаги по белому списку, дальше только пути.
    local tool="$1" a
    shift
    EXEC=("$tool")
    while (($#)); do
        a="$1"
        case "$tool:$a" in
            ls:-[lahtr1RS]*)
                [[ "$a" =~ ^-[lahtr1RS]+$ ]] || deny "флаг ls не разрешён: $a"
                EXEC+=("$a") ;;
            head:-n | tail:-n)
                if ! [[ "${2-}" =~ ^[0-9]{1,6}$ ]] || ((10#$2 > MAX_HEAD_LINES)); then
                    deny "$tool -n: нужно число ≤ $MAX_HEAD_LINES"
                fi
                EXEC+=(-n "$((10#$2))")
                shift ;;
            *:-*) deny "флаг не разрешён: $tool $a" ;;
            *) break ;;
        esac
        shift
    done
    (($#)) || deny "$tool без пути"
    EXEC+=(--)
    for a in "$@"; do
        [[ "$a" == -* ]] && deny "флаг после пути: $a"
        resolve_path "$a"
        [[ "$tool" != ls && -d "$RESOLVED" ]] && deny "$tool по каталогу: $RESOLVED"
        EXEC+=("$RESOLVED")
    done
    allow_and_exec "$tool"
}

cmd_scp() {
    # Только режим источника (-f): клиент скачивает один обычный файл. -t (загрузка), -r, -d — нет.
    local a mode="" file=""
    EXEC=(scp)
    shift
    for a in "$@"; do
        case "$a" in
            -f) mode=f ;;
            -p | -v) EXEC+=("$a") ;;
            -t | -d | -r | -[a-zA-Z]*) deny "scp: флаг не разрешён: $a" ;;
            --) ;;
            *) [[ -z "$file" ]] || deny "scp: больше одного файла"; file="$a" ;;
        esac
    done
    [[ "$mode" == f ]] || deny "scp: разрешена только выдача (-f)"
    [[ -n "$file" ]] || deny "scp: нет файла"
    resolve_path "$file"
    [[ -f "$RESOLVED" ]] || deny "scp: не обычный файл: $RESOLVED"
    EXEC+=(-f -- "$RESOLVED")
    allow_and_exec "scp -f"
}

cmd_docker() {
    # docker logs [--timestamps] --tail N <контейнер>
    local ts=() n name
    [[ "${2-}" == logs ]] || deny "docker: разрешён только logs"
    shift 2
    if [[ "${1-}" == --timestamps || "${1-}" == -t ]]; then
        ts=(--timestamps)
        shift
    fi
    (($# == 3)) && [[ "$1" == --tail ]] || deny "docker logs: формат --tail N <контейнер>"
    n="$2"
    name="$3"
    if ! [[ "$n" =~ ^[0-9]{1,6}$ ]] || ((10#$n < 1 || 10#$n > MAX_LOG_TAIL)); then
        deny "docker logs: N от 1 до $MAX_LOG_TAIL, а не $n"
    fi
    local c ok=0
    for c in "${CONTAINERS[@]}"; do
        [[ "$c" == "$name" ]] && ok=1
    done
    ((ok)) || deny "контейнер не в белом списке: $name"
    EXEC=(docker logs "${ts[@]}" --tail "$((10#$n))" -- "$name")
    allow_and_exec "docker logs"
}

git_opt_ok() {
    # Опции git log/show/status, которые только меняют вид вывода. --output, --ext-diff, -O и прочее —
    # мимо белого списка.
    case "$1" in
        --oneline | --stat | --shortstat | --name-only | --name-status | --no-patch | -s | -p | --patch | \
            --no-color | --decorate | --no-decorate | --graph | --first-parent | --no-merges | --merges | \
            --short | --branch | --porcelain | --untracked-files=no | -uno | --abbrev-commit | --date=iso | \
            --date=short | --date=relative)
            return 0 ;;
    esac
    [[ "$1" =~ ^(-n|--max-count=)?-?[0-9]{1,5}$ ]] && [[ "$1" == -* ]] && return 0
    [[ "$1" =~ ^--(since|until|after|before)=[0-9][0-9:T.-]{0,24}$ ]] && return 0
    return 1
}

cmd_git() {
    # git -C <репо> log|show|status …
    [[ "${2-}" == -C ]] || deny "git: нужен формат git -C <репо> log|show|status"
    local repo="${3-}" sub="${4-}" rp a seen_dd=0
    [[ "$repo" == /* ]] || deny "git: репо не абсолютный путь: $repo"
    rp="$(realpath -e -- "$repo" 2>/dev/null)" || deny "git: нет репо: $repo"
    local r ok=0
    for r in "${REPOS[@]}"; do
        [[ "$r" == "$rp" ]] && ok=1
    done
    ((ok)) || deny "git: репо не в белом списке: $rp"
    case "$sub" in log | show | status) ;; *) deny "git: подкоманда не разрешена: $sub" ;; esac
    shift 4
    EXEC=(git --no-pager -C "$rp" "$sub")
    while (($#)); do
        a="$1"
        if ((seen_dd)); then
            [[ "$a" =~ ^[A-Za-z0-9_][A-Za-z0-9_./-]*$ && "$a" != *..* ]] || deny "git: путь не разрешён: $a"
        elif [[ "$a" == -n ]]; then
            [[ "${2-}" =~ ^[0-9]{1,5}$ ]] || deny "git -n: нужно число"
            EXEC+=(-n "$2")
            shift 2
            continue
        elif [[ "$a" == -- ]]; then
            seen_dd=1
        elif [[ "$a" == -* ]]; then
            git_opt_ok "$a" || deny "git: опция не разрешена: $a"
        else
            [[ "$a" =~ ^[A-Za-z0-9][A-Za-z0-9_./:-]{0,199}$ ]] || deny "git: ревизия не разрешена: $a"
        fi
        EXEC+=("$a")
        shift
    done
    allow_and_exec "git $sub"
}

# ── main ──────────────────────────────────────────────────────────────────

main() {
    [[ -n "$RAW_CMD" ]] || deny "пустая команда (интерактивный вход закрыт)"
    ((${#RAW_CMD} <= MAX_CMD_LEN)) || deny "команда длиннее $MAX_CMD_LEN"
    [[ "$RAW_CMD" =~ ^[A-Za-z0-9_./:=@+,\ -]+$ ]] || deny "запрещённые символы (разрешены буквы, цифры, _ . / : = @ + , - и пробел)"

    local -a argv
    IFS=' ' read -r -a argv <<<"$RAW_CMD"
    ((${#argv[@]})) || deny "пустая команда"
    load_config

    case "${argv[0]}" in
        ls | stat | cat | head | tail | sha256sum) cmd_files "${argv[@]}" ;;
        scp) cmd_scp "${argv[@]}" ;;
        docker) cmd_docker "${argv[@]}" ;;
        git) cmd_git "${argv[@]}" ;;
        # sftp-подсистема приходит сюда как /usr/lib/openssh/sftp-server или internal-sftp — отказ.
        *) deny "команда не в белом списке: ${argv[0]}" ;;
    esac
}

main
