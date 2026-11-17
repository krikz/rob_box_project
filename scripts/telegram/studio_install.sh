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

# Корень репозитория: из расположения этого скрипта, а не зашитый путь.
# Зашитый /home/builder ломался бы на другой машине и после переезда клона.
REPO_ROOT="${REPO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
PROFILE="${STUDIO_PROFILE:-studio}"
# HERMES_HOME в окружении Hermes указывает на папку ПРОФИЛЯ, а не на корень,
# поэтому корень берём явно: профили лежат в ~/.hermes/profiles/<имя>.
GLOBAL_HOME="${STUDIO_HERMES_HOME:-$HOME/.hermes}"
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
    #
    # Контрольную сумму исходника снимаем ДО записи: после записи уже нечем
    # проверить, что исходник не пострадал.
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

# Расписание кронов лежит в репозитории (docker/studio/cron.json) — это
# источник истины, а не строчка в runbook.
CRON_SPEC="$REPO_ROOT/docker/studio/cron.json"
if [ -f "$CRON_SPEC" ]; then
    echo "[studio-install] расписание кронов: $CRON_SPEC"
    python3 - "$CRON_SPEC" <<'PYEOF'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as handle:
    spec = json.load(handle)
for job in spec.get("jobs", []):
    print(f"  {job.get('name')}: {job.get('schedule')} {job.get('timezone', '')} → {job.get('script')}")
PYEOF
fi

# Кнопки Шифу — постоянный сервис, а не крон (Шифу 06.11: «нажал — а оно уходит курить»).
# Юнит берётся из репо, @REPO_ROOT@ заменяется на корень клона.
UNIT_SRC="$REPO_ROOT/docker/studio/studio-watch.service"
UNIT_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
if [ -f "$UNIT_SRC" ]; then
    mkdir -p "$UNIT_DIR"
    sed "s#@REPO_ROOT@#$REPO_ROOT#g" "$UNIT_SRC" > "$UNIT_DIR/studio-watch.service.tmp.$$"
    mv -f "$UNIT_DIR/studio-watch.service.tmp.$$" "$UNIT_DIR/studio-watch.service"
    echo "[studio-install] юнит: $UNIT_DIR/studio-watch.service"
    if [ "${STUDIO_ENABLE_SERVICE:-0}" = "1" ]; then
        systemctl --user daemon-reload
        systemctl --user enable --now studio-watch.service
        systemctl --user restart studio-watch.service
        echo "[studio-install] сервис studio-watch включён и перезапущен"
    else
        echo "[studio-install] включить: systemctl --user daemon-reload && systemctl --user enable --now studio-watch"
    fi
    echo "[studio-install] крон «studio: опрос кнопок» должен быть снят: hermes --profile studio cron list"
fi

# Синхронизация клона студии с origin/develop (ADR-0172, issue #4703).
# Юниты берём из репо, @REPO_ROOT@ заменяется на корень клона — так же,
# как у studio-watch. Таймер раз в 5 минут: фикс из develop доезжает до
# боевой студии без ручного git pull.
for unit in studio-sync.service studio-sync.timer; do
    UNIT_SRC="$REPO_ROOT/docker/studio/$unit"
    [ -f "$UNIT_SRC" ] || { echo "[studio-install] нет юнита: $UNIT_SRC" >&2; continue; }
    mkdir -p "$UNIT_DIR"
    sed "s#@REPO_ROOT@#$REPO_ROOT#g" "$UNIT_SRC" > "$UNIT_DIR/$unit.tmp.$$"
    mv -f "$UNIT_DIR/$unit.tmp.$$" "$UNIT_DIR/$unit"
    echo "[studio-install] юнит: $UNIT_DIR/$unit"
done
if [ "${STUDIO_ENABLE_SERVICE:-0}" = "1" ]; then
    systemctl --user daemon-reload
    systemctl --user enable --now studio-sync.timer
    echo "[studio-install] таймер studio-sync включён (синхронизация клона раз в 5 минут)"
else
    echo "[studio-install] включить: systemctl --user daemon-reload && systemctl --user enable --now studio-sync.timer"
fi
