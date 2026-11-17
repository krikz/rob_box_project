#!/usr/bin/env bash
# Набор «предельная скорость»: дихотомия по классам мест, каждый шаг выбирает код (speed_verdict.py next).
#
#   SPEED_OUT=<каталог> scripts/hil/speed/sweep.sh [tight|open ...]   # по умолчанию оба класса по очереди
#
# Журнал $SPEED_OUT/ledger.jsonl переживает обрыв: повторный запуск продолжит с того же места.
# HIL уже включён; замок стенда держит оператор. Стоп: touch $SPEED_OUT/STOP (после текущего прогона).
set -uo pipefail
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Windows-python не понимает /d/… при MSYS_NO_PATHCONV=1
PYHERE="$(cygpath -m "$HERE" 2>/dev/null || echo "$HERE")"
: "${SPEED_OUT:?SPEED_OUT — каталог результатов}"
: "${SPEED_START:=1.0}"
classes=("$@")
[ ${#classes[@]} -gt 0 ] || classes=(tight open)
for cls in "${classes[@]}"; do
    while :; do
        [ -f "$SPEED_OUT/STOP" ] && { echo "SWEEP_STOPPED $(date -u +%FT%TZ)"; exit 0; }
        line=$(python "$PYHERE/speed_verdict.py" next --ledger "$SPEED_OUT/ledger.jsonl" --cls "$cls" --start "$SPEED_START")
        echo "$(date -u +%H:%M:%SZ) $line"
        read -r tag _ v scene rep _ <<<"$line"
        [ "$tag" = NEXT ] || { echo "SWEEP_ERROR: speed_verdict next не дал строку NEXT"; exit 1; }
        [ "$v" = DONE ] && break
        bash "$HERE/run_scene.sh" "$scene" "$v" "$rep" || echo "$(date -u +%H:%M:%SZ) run_scene rc=$? ($scene $v $rep)"
    done
done
echo "SWEEP_DONE $(date -u +%FT%TZ)"
