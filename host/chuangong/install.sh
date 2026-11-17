#!/usr/bin/env bash
# Установка агента chuangong на одну Pi (ADR-0168, фаза B). Идемпотентно: повторный запуск ничего не ломает.
#
#   sudo bash host/chuangong/install.sh --node main|vision|sound [--robot-id RBXU100001] [--enable] [--pubkey FILE]
#
# Что делает:
#   * apt: python3, python3-yaml, python3-cryptography (minisign не нужен — подпись проверяет Python);
#   * код агента → /opt/rob_box/chuangong/chuangong (заменяется целиком), обёртка /usr/local/bin/chuangong;
#   * каталоги /opt/rob_box/releases, /var/lib/chuangong (владелец — пользователь агента), /etc/chuangong;
#   * /etc/chuangong/chuangong.env из примера — только если файла ещё нет (правки на хосте не затираются);
#     режим observe: агент смотрит и докладывает, ничего не скачивает и не переключает;
#   * публичный ключ minisign → /etc/chuangong/release.pub, если передан --pubkey (без ключа агент честно
#     докладывает fetch_error: no_pubkey);
#   * юниты chuangong.service/.timer → /etc/systemd/system. Таймер НЕ включается без --enable.
#   sudoers не нужен: в фазе B агент работает от пользователя с группой docker и пишет только в свои каталоги.
#
# Переменные для тестов и чистой ОС: CHUANGONG_ROOT (префикс корня ФС, по умолчанию пусто),
# CHUANGONG_USER (по умолчанию ros2 или SUDO_USER), CHUANGONG_SKIP_APT=1, CHUANGONG_SYSTEMCTL (по умолчанию systemctl).
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${CHUANGONG_ROOT:-}"
AGENT_USER="${CHUANGONG_USER:-${SUDO_USER:-ros2}}"
SYSTEMCTL="${CHUANGONG_SYSTEMCTL:-systemctl}"
NODE=""
ROBOT_ID="${ROBOT_ID:-RBXU100001}"
ENABLE=0
PUBKEY=""

usage() { sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        --node) NODE="${2:-}"; shift 2 ;;
        --robot-id) ROBOT_ID="${2:-}"; shift 2 ;;
        --enable) ENABLE=1; shift ;;
        --pubkey) PUBKEY="${2:-}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "chuangong install: неизвестный аргумент $1" >&2; usage >&2; exit 2 ;;
    esac
done

case "$NODE" in
    main|vision|sound) ;;
    *) echo "chuangong install: нужен --node main|vision|sound" >&2; exit 2 ;;
esac
[[ "$ROBOT_ID" =~ ^[A-Z0-9]{4,32}$ ]] || { echo "chuangong install: ROBOT_ID '$ROBOT_ID' не похож на RBXU100001" >&2; exit 2; }

log() { echo "[chuangong] $*"; }

if [ "${CHUANGONG_SKIP_APT:-0}" != 1 ]; then
    missing=""
    for pkg in python3 python3-yaml python3-cryptography; do
        dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q '^install ok installed$' || missing="$missing $pkg"
    done
    if [ -n "$missing" ]; then
        log "apt install:$missing"
        DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends $missing
    else
        log "python3, python3-yaml, python3-cryptography уже стоят"
    fi
fi

owner="$AGENT_USER"
if ! id "$owner" >/dev/null 2>&1; then
    log "пользователя $owner нет — каталоги остаются за root"
    owner="root"
fi

install -d -m 0755 "$ROOT/opt/rob_box" "$ROOT/opt/rob_box/chuangong" "$ROOT/etc/chuangong" "$ROOT/usr/local/bin" \
    "$ROOT/etc/systemd/system"
install -d -m 0755 -o "$owner" -g "$(id -gn "$owner")" "$ROOT/opt/rob_box/releases" "$ROOT/var/lib/chuangong"

# Код агента — целиком из репо: новая копия рядом, затем подмена каталога.
new="$ROOT/opt/rob_box/chuangong/.chuangong.new"
rm -rf "$new"
install -d -m 0755 "$new"
for f in "$SRC"/chuangong/*.py; do
    install -m 0644 "$f" "$new/"
done
rm -rf "$ROOT/opt/rob_box/chuangong/chuangong"
mv "$new" "$ROOT/opt/rob_box/chuangong/chuangong"
log "код агента: $ROOT/opt/rob_box/chuangong/chuangong ($(ls "$ROOT/opt/rob_box/chuangong/chuangong" | wc -l) файлов)"

cat > "$ROOT/usr/local/bin/chuangong.tmp" << 'WRAPEOF'
#!/bin/sh
# Обёртка агента chuangong (ставит host/chuangong/install.sh): конфиг из /etc/chuangong/chuangong.env.
if [ -r /etc/chuangong/chuangong.env ]; then
    set -a
    . /etc/chuangong/chuangong.env
    set +a
fi
cd /opt/rob_box/chuangong && exec env PYTHONDONTWRITEBYTECODE=1 python3 -m chuangong "$@"
WRAPEOF
chmod 0755 "$ROOT/usr/local/bin/chuangong.tmp"
mv "$ROOT/usr/local/bin/chuangong.tmp" "$ROOT/usr/local/bin/chuangong"

envf="$ROOT/etc/chuangong/chuangong.env"
if [ -f "$envf" ]; then
    log "$envf уже есть — не трогаю (узел в нём: $(sed -n 's/^CHUANGONG_NODE=//p' "$envf"))"
else
    sed -e "s/^CHUANGONG_ROBOT_ID=.*/CHUANGONG_ROBOT_ID=$ROBOT_ID/" -e "s/^CHUANGONG_NODE=.*/CHUANGONG_NODE=$NODE/" \
        -e "s/^CHUANGONG_MODE=.*/CHUANGONG_MODE=observe/" "$SRC/systemd/chuangong.env.example" > "$envf.tmp"
    chmod 0644 "$envf.tmp"
    mv "$envf.tmp" "$envf"
    log "$envf создан: robot_id=$ROBOT_ID node=$NODE mode=observe"
fi

if [ -n "$PUBKEY" ]; then
    install -m 0644 "$PUBKEY" "$ROOT/etc/chuangong/release.pub"
    log "публичный ключ: $ROOT/etc/chuangong/release.pub"
elif [ ! -f "$ROOT/etc/chuangong/release.pub" ]; then
    log "публичного ключа нет (--pubkey) — агент будет докладывать fetch_error: no_pubkey"
fi

changed=0
for unit in chuangong.service chuangong.timer; do
    dst="$ROOT/etc/systemd/system/$unit"
    if [ -f "$dst" ] && cmp -s "$SRC/systemd/$unit" "$dst"; then
        continue
    fi
    install -m 0644 "$SRC/systemd/$unit" "$dst"
    changed=1
done
if [ "$changed" = 1 ]; then
    log "юниты обновлены"
    [ -n "$ROOT" ] || "$SYSTEMCTL" daemon-reload
fi

if [ "$ENABLE" = 1 ]; then
    [ -n "$ROOT" ] || "$SYSTEMCTL" enable --now chuangong.timer
    log "таймер chuangong.timer включён (--enable)"
else
    log "таймер НЕ включён (включить: sudo systemctl enable --now chuangong.timer)"
fi
log "готово. Проверка: chuangong run --dry-run"
