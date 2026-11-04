#!/usr/bin/env bash
# Деплой не идёт, пока на Main Pi включён HIL (эпик #4371).
#
# scripts/hil/hil.sh on кладёт замок /tmp/hil.lock на Main Pi и останавливает
# ros2-control (моторы). Деплой делает `compose down` + `up -d` и поднял бы
# ros2-control — второй /controller_manager рядом с Gazebo, команды Nav2 ушли бы
# и на настоящие моторы. Поэтому при замке деплой падает с понятной ошибкой ДО
# любых остановок на Pi. Снять: scripts/hil/hil.sh off (вернёт робота и замок).
#
#   check_hil_lock.sh <команда до Main Pi>...
#   пример: check_hil_lock.sh ssh -o BatchMode=yes ros2@10.1.1.10
# Последним аргументом команде добавляется проверка на стороне Pi.
#
# Выход: 0 — замка нет; 1 — HIL включён; 2 — Main Pi не ответил (тоже стоп:
# состояние неизвестно, деплоить вслепую нельзя).
set -uo pipefail

LOCK="${HIL_LOCK:-/tmp/hil.lock}"
[ "$#" -gt 0 ] || { echo "usage: $0 <команда до Main Pi>..." >&2; exit 2; }

out=$("$@" "if [ -e '$LOCK' ]; then echo HIL_LOCKED; cat '$LOCK'; else echo HIL_FREE; fi" 2>&1)
rc=$?
case "$out" in
    HIL_FREE*)
        echo "✅ HIL выключен: замка $LOCK на Main Pi нет"
        exit 0
        ;;
    HIL_LOCKED*)
        echo "::error::На Main Pi включён HIL (замок $LOCK): ${out#HIL_LOCKED}"
        echo "❌ Деплой остановлен до каких-либо действий на Pi. ros2-control и lslidar сейчас"
        echo "   намеренно выключены, стек навигации едет по Gazebo. Деплой поднял бы моторы."
        echo "   Сначала: scripts/hil/hil.sh off  (план docs/research/2026-11-04-hil-robot-on-gazebo-data.md §8)"
        exit 1
        ;;
    *)
        echo "::error::Не удалось проверить замок HIL на Main Pi (rc=$rc): $out"
        exit 2
        ;;
esac
