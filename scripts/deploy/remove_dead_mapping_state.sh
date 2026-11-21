#!/usr/bin/env bash
# ============================================================================
# remove_dead_mapping_state.sh — убрать мёртвый mapping_state.json (issue #5045).
#
# /maps/mapping_state.json на Main Pi не читается с коммита 4f5baba18
# (2026-03-06): start_rtabmap.sh больше не смотрит в файл, MCP-тулы держат
# состояние в памяти, имя активной карты — строка maps.is_active в БД (#4392).
# Файл остался в томе карт и называет «квартиру», хотя робот стоит в мастерской.
#
# Запускается на Main Pi шагом «[Main Pi] Remove dead mapping_state.json»
# в ".github/workflows/L-Deploy and Verify.yml", ДО Stop Containers.
# Аргумент — каталог тома карт (на роботе ~/rob_box_project/docker/main/maps,
# в контейнере rtabmap это /maps). Каталога нет — нечего чистить, выход 0.
#
# Идемпотентно и узко: удаляет только файл с именем mapping_state.json.
# rtabmap.db и всё остальное в каталоге не трогает. Нет файла — тихий выход 0
# и строка «нет файла» в лог. Есть — удаляет и пишет, что удалил.
# Код возврата: 0 — файла нет (не было или удалён); 1 — удалить не вышло.
# ============================================================================

set -uo pipefail

maps_dir="${1:-}"

if [ -z "$maps_dir" ]; then
    echo "::error title=Dead mapping_state.json (#5045)::не задан каталог тома карт" >&2
    exit 1
fi

target="$maps_dir/mapping_state.json"

if [ ! -d "$maps_dir" ]; then
    echo "ℹ️  #5045: каталога $maps_dir нет — чистить нечего"
    exit 0
fi

if [ ! -e "$target" ] && [ ! -L "$target" ]; then
    echo "ℹ️  #5045: $target отсутствует — мёртвый файл уже убран"
    exit 0
fi

if [ -L "$target" ] || [ ! -f "$target" ]; then
    echo "::error title=Dead mapping_state.json (#5045)::$target не обычный файл — не трогаю, разберитесь вручную" >&2
    exit 1
fi

rm -f -- "$target" || {
    echo "::error title=Dead mapping_state.json (#5045)::не удалось удалить $target" >&2
    exit 1
}

if [ -e "$target" ]; then
    echo "::error title=Dead mapping_state.json (#5045)::$target остался после rm" >&2
    exit 1
fi

echo "🧹 #5045: удалён мёртвый $target (не читается с 4f5baba18, имя карты — maps.is_active в БД)"
exit 0
