#!/bin/bash
# Раскладка скриптов студии в профиль Hermes (~/.hermes/profiles/studio/scripts/).
#
# Зачем отдельный скрипт: крон ищет скрипты только в папке своего профиля, а исходники
# живут в репозитории.
#
# ПОЧЕМУ ОБЁРТКИ, А НЕ HARDLINK: крон допускает только файл внутри папки профиля
# (hermes-agent, cron/scheduler_script.py::_resolve_script_path — симлинк наружу не
# проходит). Hardlink вроде подходит, но рвётся, когда исходник пересоздают
# (patch/write_file пересоздают файл) — и в профиле молча остаётся СТАРАЯ версия.
# Именно так крон студии пять минут подряд падал с «TG_STUDIO_BOT_TOKEN не задан».
# Обёртка же всегда зовёт репозиторий: правка в репо видна сразу, без перераскладки.
#
# Запуск: bash scripts/telegram/studio_install.sh
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
PROFILE="${STUDIO_PROFILE:-studio}"
# HERMES_HOME в окружении Hermes указывает на папку ПРОФИЛЯ, а не на корень,
# поэтому корень берём явно: профили лежат в ~/.hermes/profiles/<имя>.
GLOBAL_HOME="${STUDIO_HERMES_HOME:-/home/builder/.hermes}"
SCRIPTS_DIR="$GLOBAL_HOME/profiles/$PROFILE/scripts"
SRC_DIR="$REPO_ROOT/scripts/telegram"

SCRIPTS=(studio_poll.sh studio_publish.sh studio_session.sh studio_facts.sh)

if [ ! -d "$SCRIPTS_DIR" ]; then
    echo "[studio-install] нет папки профиля: $SCRIPTS_DIR" >&2
    echo "[studio-install] создай профиль: hermes profile create $PROFILE --clone-from architect" >&2
    exit 1
fi

mkdir -p "$SCRIPTS_DIR"
for name in "${SCRIPTS[@]}"; do
    src="$SRC_DIR/$name"
    [ -f "$src" ] || { echo "[studio-install] нет источника: $src" >&2; exit 1; }
    dst="$SCRIPTS_DIR/$name"
    # Пишем во временный файл и переименовываем, а не перезаписываем на месте.
    # Если в профиле ещё лежит hardlink на исходник, запись «>» идёт через общий
    # inode — и обёртка записывается в САМ РЕПОЗИТОРИЙ. Одно «rm» и «mv» этого не дают.
    before=$(cksum "$src" | cut -d' ' -f1,2)
    tmp="$dst.tmp.$$"
    cat > "$tmp" <<EOF
#!/bin/bash
# Обёртка студии: зовёт скрипт из репозитория.
# Создаётся scripts/telegram/studio_install.sh — руками не править.
exec bash "$src" "\$@"
EOF
    chmod +x "$tmp"
    rm -f "$dst"
    mv -f "$tmp" "$dst"
    after=$(cksum "$src" | cut -d' ' -f1,2)
    if [ "$before" != "$after" ]; then
        echo "[studio-install] ВНИМАНИЕ: исходник $src изменился при раскладке" >&2
        exit 1
    fi
    echo "[studio-install] $name → $dst"
done

echo "[studio-install] готово: ${#SCRIPTS[@]} обёрток в профиле $PROFILE"
echo "[studio-install] правки скриптов видны сразу — перераскладка не нужна"
