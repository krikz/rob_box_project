#!/usr/bin/env bash
# install_ro_key.sh — ставит ключ студии канала на хост с доступом только на чтение (issue #4492).
#
#   scripts/studio/install_ro_key.sh <host> <pubkey-файл> [--profile P] [--user U] [--apply]
#
# Без --apply ничего не делает: печатает строку authorized_keys и что куда будет скопировано.
# С --apply по ssh-ключу координатора (BatchMode, пароли не используются):
#   1. кладёт гейт в ~/.local/bin/studio_ro_gate.sh (0755) и конфиги профиля в ~/.config/studio_ro/ (0644);
#   2. дописывает в ~/.ssh/authorized_keys строку
#      command="<home>/.local/bin/studio_ro_gate.sh",no-pty,no-port-forwarding,no-agent-forwarding,
#      no-X11-forwarding,restrict <ключ>
#      ТОЛЬКО если её там ещё нет. Если тот же ключ уже стоит с другими опциями (например, без гейта),
#      скрипт останавливается: такую строку координатор убирает руками, иначе гейт обходится.
#
# Профиль (ro_config/<профиль>) выбирается по адресу хоста: 10.1.1.250 → sandbox, 10.1.1.249 → katana,
# 10.1.1.10/20 → main, 10.1.1.11/21 → vision, 10.1.1.12/22 → sound; иначе нужен --profile.
# Запуск с --apply — координатор после «да» Шифу в чате приложения. Runbook: docs/runbooks/studio-ro-access.md.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GATE_SRC="$HERE/studio_ro_gate.sh"
OPTS="no-pty,no-port-forwarding,no-agent-forwarding,no-X11-forwarding,restrict"

usage() {
    sed -n '2,8p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
    exit 2
}
die() {
    echo "install_ro_key: $*" >&2
    exit 1
}

host="" pubfile="" profile="" user="ros2" apply=0
while (($#)); do
    case "$1" in
        --apply) apply=1 ;;
        --profile) profile="${2-}"; shift ;;
        --user) user="${2-}"; shift ;;
        -h | --help) usage ;;
        -*) die "неизвестный флаг: $1" ;;
        *)
            if [[ -z "$host" ]]; then host="$1"
            elif [[ -z "$pubfile" ]]; then pubfile="$1"
            else die "лишний аргумент: $1"; fi ;;
    esac
    shift
done
[[ -n "$host" && -n "$pubfile" ]] || usage
[[ "$host" =~ ^[A-Za-z0-9][A-Za-z0-9.-]*$ ]] || die "плохой хост: $host"
[[ "$user" =~ ^[a-z_][a-z0-9_-]*$ ]] || die "плохой пользователь: $user"

if [[ -z "$profile" ]]; then
    case "$host" in
        10.1.1.250) profile=sandbox ;;
        10.1.1.249) profile=katana ;;
        10.1.1.10 | 10.1.1.20) profile=main ;;
        10.1.1.11 | 10.1.1.21) profile=vision ;;
        10.1.1.12 | 10.1.1.22) profile=sound ;;
        *) die "профиль для $host не известен — укажи --profile sandbox|katana|main|vision|sound" ;;
    esac
fi
CONF_SRC="$HERE/ro_config/$profile"
[[ -d "$CONF_SRC" ]] || die "нет профиля $CONF_SRC"
for f in roots containers repos; do [[ -f "$CONF_SRC/$f" ]] || die "нет $CONF_SRC/$f"; done

[[ -r "$pubfile" ]] || die "не читается файл ключа: $pubfile"
key="$(tr -d '\r' <"$pubfile" | sed -n '1p')"
# Только сам ключ, без опций: тип, base64 и необязательный комментарий из безопасных символов.
[[ "$key" =~ ^(ssh-ed25519|ecdsa-sha2-nistp256|ecdsa-sha2-nistp384|ssh-rsa)\ [A-Za-z0-9+/]+={0,3}(\ [A-Za-z0-9_.@-]{1,64})?$ ]] ||
    die "это не публичный ключ OpenSSH (или в нём опции/странный комментарий): $pubfile"
key_body="$(cut -d' ' -f1-2 <<<"$key")"

remote_home="/home/$user"
gate_dst="$remote_home/.local/bin/studio_ro_gate.sh"
line="command=\"$gate_dst\",$OPTS $key"

echo "Хост:     $user@$host (профиль $profile)"
echo "Гейт:     $GATE_SRC -> $gate_dst"
echo "Конфиги:  $CONF_SRC/{roots,containers,repos} -> $remote_home/.config/studio_ro/"
echo "Строка authorized_keys (добавится, только если её ещё нет):"
echo "$line"
if ((!apply)); then
    echo "Сухой прогон: ничего не сделано. Для установки — тот же вызов с --apply."
    exit 0
fi

SSH=(ssh -o BatchMode=yes -o PasswordAuthentication=no -o KbdInteractiveAuthentication=no
    -o ConnectTimeout=10 "$user@$host")

# shellcheck disable=SC2016 # $HOME раскрывается на хосте, а не здесь
got_home="$("${SSH[@]}" 'printf %s "$HOME"')"
[[ "$got_home" == "$remote_home" ]] || die "HOME на хосте $got_home, ожидался $remote_home — укажи --user"

put() { # put <локальный файл> <удалённый путь> <права>; CRLF чекаута на Windows срезается
    tr -d '\r' <"$1" |
        "${SSH[@]}" "mkdir -p \"\$(dirname '$2')\" && cat > '$2.tmp' && chmod $3 '$2.tmp' && mv -f '$2.tmp' '$2'"
}
put "$GATE_SRC" "$gate_dst" 755
for f in roots containers repos; do put "$CONF_SRC/$f" "$remote_home/.config/studio_ro/$f" 644; done

# Ключ и строка — из проверенного набора символов, в одинарных кавычках они безопасны.
"${SSH[@]}" "set -e; ak=\"\$HOME/.ssh/authorized_keys\"; mkdir -p \"\$HOME/.ssh\"; chmod 700 \"\$HOME/.ssh\"; touch \"\$ak\"; chmod 600 \"\$ak\"
if grep -qxF '$line' \"\$ak\"; then echo 'строка уже есть — ничего не добавлено'; exit 0; fi
if grep -qF '$key_body' \"\$ak\"; then echo 'ключ уже стоит с другими опциями — убери ту строку руками' >&2; exit 3; fi
printf '%s\n' '$line' >> \"\$ak\"; echo 'строка добавлена'"
echo "Готово на $host. Проверка: docs/runbooks/studio-ro-access.md §3."
