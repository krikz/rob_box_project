#!/usr/bin/env bash
# CRLF → LF в копии репо на песочнице (вызывает sandbox_prep.sh; докер-контейнеры пишут в out/ от root).
#   fix_crlf.sh <каталог>
# Не заходит в каталоги результатов и кэша (out*, fuel). Чинит только файлы с \r; файл, который
# нельзя перезаписать, — предупреждение в stderr, не провал: иначе set -e в sandbox_prep.sh
# не дойдёт до шагов Depot и patch_compose.py.
set -euo pipefail
DIR="${1:?каталог}"
cd "$DIR"
find . \( -path ./fuel -o -path './out*' \) -prune -o -type f \
    \( -name '*.sh' -o -name '*.py' -o -name '*.yaml' -o -name '*.json5' -o -name '*.sdf' \) -print0 \
    | while IFS= read -r -d '' f; do
        [ "$(tr -d '\r' < "$f" | wc -c)" -ne "$(wc -c < "$f")" ] || continue
        sed -i 's/\r$//' "$f" || echo "fix_crlf: не удалось исправить $f (нет прав?)" >&2
    done
