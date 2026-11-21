#!/usr/bin/env bash
# Регресс-тест issue #5045: деплой Main Pi убирает мёртвый mapping_state.json
# из тома карт и пишет об этом в лог. rtabmap.db не трогает.
#
# 1. scripts/deploy/remove_dead_mapping_state.sh:
#    - файла нет / каталога нет ⇒ exit 0, строка «нет» в вывод;
#    - файл есть ⇒ exit 0, файл исчез, строка «удалён» в вывод, соседний
#      rtabmap.db на месте;
#    - повторный запуск ⇒ exit 0 (идемпотентно);
#    - цель — симлинк или каталог ⇒ exit 1, ничего не удалено;
#    - без аргумента ⇒ exit 1.
# 2. ".github/workflows/L-Deploy and Verify.yml": шаг чистки стоит на Main Pi
#    ДО «[Main Pi] Stop Containers», зовёт этот скрипт и не маскируется
#    `|| echo` / continue-on-error.
#
# Запуск: bash scripts/deploy/tests/test_remove_dead_mapping_state.sh

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SCRIPT="$REPO_ROOT/scripts/deploy/remove_dead_mapping_state.sh"
WORKFLOW="$REPO_ROOT/.github/workflows/L-Deploy and Verify.yml"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

pass=0
failures=0
ok() { echo "PASS: $1"; pass=$((pass + 1)); }
ko() { echo "FAIL: $1"; failures=$((failures + 1)); }

# --- скрипт: нет файла -------------------------------------------------------
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && grep -q 'нет' <<<"$out"; then
    ok "каталога нет — exit 0 и строка в лог"
else
    ko "каталога нет: rc=$rc out=$out"
fi

mkdir -p "$WORK/maps"
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && grep -q 'отсутствует' <<<"$out"; then
    ok "файла нет — exit 0 и строка «отсутствует»"
else
    ko "файла нет: rc=$rc out=$out"
fi

# --- скрипт: файл есть, сосед жив -------------------------------------------
printf '%s\n' '{"map_name":"квартира","mode":"localization"}' > "$WORK/maps/mapping_state.json"
printf '%s\n' 'rtabmap-db-sentinel' > "$WORK/maps/rtabmap.db"
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && [ ! -e "$WORK/maps/mapping_state.json" ] && grep -q 'удалён' <<<"$out"; then
    ok "файл был — удалён, exit 0, строка «удалён»"
else
    ko "файл был: rc=$rc exists=$([ -e "$WORK/maps/mapping_state.json" ] && echo yes || echo no) out=$out"
fi
if [ "$(cat "$WORK/maps/rtabmap.db")" = "rtabmap-db-sentinel" ]; then
    ok "rtabmap.db не тронут"
else
    ko "rtabmap.db изменён или пропал"
fi

# --- идемпотентность ---------------------------------------------------------
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && grep -q 'отсутствует' <<<"$out"; then
    ok "повторный запуск — exit 0, нечего удалять"
else
    ko "повторный запуск: rc=$rc out=$out"
fi

# --- не обычный файл: не трогаем --------------------------------------------
ln -s rtabmap.db "$WORK/maps/mapping_state.json"
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 1 ] && [ -L "$WORK/maps/mapping_state.json" ]; then
    ok "симлинк — exit 1, ссылка на месте"
else
    ko "симлинк: rc=$rc out=$out"
fi
rm -f "$WORK/maps/mapping_state.json"

mkdir "$WORK/maps/mapping_state.json"
out=$(bash "$SCRIPT" "$WORK/maps" 2>&1)
rc=$?
if [ "$rc" -eq 1 ] && [ -d "$WORK/maps/mapping_state.json" ]; then
    ok "каталог с тем же именем — exit 1, на месте"
else
    ko "каталог-цель: rc=$rc out=$out"
fi
rmdir "$WORK/maps/mapping_state.json"

# --- без аргумента -----------------------------------------------------------
out=$(bash "$SCRIPT" 2>&1)
rc=$?
if [ "$rc" -eq 1 ]; then
    ok "без аргумента — exit 1"
else
    ko "без аргумента: rc=$rc out=$out"
fi

# --- шаг в workflow ----------------------------------------------------------
step_line=$(grep -n 'Remove dead mapping_state.json (#5045)' "$WORKFLOW" | head -1 | cut -d: -f1)
stop_line=$(grep -n '\[Main Pi\] Stop Containers' "$WORKFLOW" | head -1 | cut -d: -f1)
if [ -n "$step_line" ] && [ -n "$stop_line" ] && [ "$step_line" -lt "$stop_line" ]; then
    ok "шаг чистки (#5045) стоит до [Main Pi] Stop Containers ($step_line < $stop_line)"
else
    ko "порядок шагов: cleanup=$step_line stop=$stop_line"
fi

if grep -q 'scripts/deploy/remove_dead_mapping_state.sh' "$WORKFLOW"; then
    ok "workflow зовёт remove_dead_mapping_state.sh"
else
    ko "workflow не зовёт скрипт"
fi

# Блок шага: от его имени до следующего шага «- name:» на том же отступе.
# Не должен глотать ошибку своим || echo или continue-on-error.
block=$(awk '/Remove dead mapping_state.json \(#5045\)/{p=1; next} p && /^      - name: /{exit} p' "$WORKFLOW")
if grep -q '|| echo' <<<"$block" || grep -q 'continue-on-error' <<<"$block"; then
    ko "шаг чистки маскирует ошибку: $block"
else
    ok "шаг чистки не маскирует ошибку (нет || echo / continue-on-error)"
fi

echo "---"
echo "PASS=$pass FAIL=$failures"
[ "$failures" -eq 0 ]
