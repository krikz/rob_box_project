#!/usr/bin/env bash
# =============================================================================
# cleanup_dead_voice_dbs.sh — уборка мёртвых/пустых БД в /data (issue #2751).
#
# ⚠️ ТОЛЬКО ЧЕЛОВЕК ЗАПУСКАЕТ ЭТО НА РОБОТЕ. Скрипт создан агентом как
# ПРЕДЛОЖЕНИЕ (issue #2751 п.3: "уборка — только скриптом/инструкцией в
# репозитории плюс список в PR, файлы на роботе не трогать") — сам он ни
# разу не запускался ни в CI, ни на Vision Pi. Дефолт — dry-run: без
# --apply только печатает план и ничего не удаляет и не архивирует.
#
# Инвентаризация (issue #2751, замер 22.09.2026, перепроверено вручную в
# рамках PR read-only через ssh: `ls -la ~/rob_box_project/docker/vision/data/voice/`):
#
#   ФАЙЛ                    РАЗМЕР        ПОСЛЕДНЯЯ ЗАПИСЬ   ПОЧЕМУ МЁРТВ
#   memory.db               0 байт        09.08              никогда не писался
#   rob_box_voice.db        0 байт        13.04               —"—
#   voice_assistant.db      0 байт        12.06               —"—
#   waypoints.db            0 байт        01.03               —"—
#   voice.db                0 байт        22.09 01:32         пересоздан кем-то
#                                                              этой ночью — ПРИЧИНА
#                                                              НЕ НАЙДЕНА (см. PR),
#                                                              контейнеры перезапущены
#                                                              22.09 06:41 — ПОЗЖЕ
#                                                              создания файла, то есть
#                                                              не текущий процесс.
#   operator_memory.db      4 КБ + 57 КБ  08.09               УПРАЗДНЁН ADR-0083 §E —
#                           WAL                               supervisor_node.py:173
#                                                              прямо говорит "упразднён",
#                                                              оба агента (личность +
#                                                              ТАРС) делят harness_voice.db
#                                                              через колонку agent.
#
# speakers.db.bak-* (issue #2750) — СЮДА НЕ ВХОДЯТ. Это не мусор, а
# единственная сохранившаяся копия 44 профилей с февраля (бэкап от
# 21.09.2026). Их разбор — отдельное решение владельца (issue #2750 п.4),
# не блокируется этим скриптом.
#
# Что делает --apply
# -------------------
# 1. Для КАЖДОГО файла из списка выше заново проверяет актуальный размер
#    на диске (issue #2751 писался неделю назад — состояние могло
#    измениться) и ОТКАЗЫВАЕТСЯ трогать файл, если он больше не пустой
#    (кроме operator_memory.db — у него ожидаемый ненулевой размер, см. §2).
# 2. Пустые (0 байт) файлы — удаляет напрямую: терять нечего.
# 3. operator_memory.db (+ его -shm/-wal) — переносит в
#    <DATA_DIR>/_archive_2751/, не удаляет: там формально есть байты
#    (даже если, по коду, ничего не читает), архивная копия дешевле, чем
#    сожалеть, если ADR-0083 §E окажется прочитан неверно.
# 4. Всегда печатает, что сделал — ни одной молчаливой операции.
#
# Использование
# -------------
#   # план, ничего не трогает (дефолт)
#   DATA_DIR=~/rob_box_project/docker/vision/data/voice \
#       bash scripts/maintenance/cleanup_dead_voice_dbs.sh
#
#   # применить (только человек, только после дефолт-прогона выше)
#   DATA_DIR=~/rob_box_project/docker/vision/data/voice \
#       bash scripts/maintenance/cleanup_dead_voice_dbs.sh --apply
# =============================================================================
set -u

DATA_DIR="${DATA_DIR:-/data}"
APPLY=0
[ "${1:-}" = "--apply" ] && APPLY=1

log() { printf '[cleanup-dead-voice-dbs] %s\n' "$*"; }

if [ ! -d "$DATA_DIR" ]; then
    echo "FATAL: DATA_DIR не найден: $DATA_DIR" >&2
    exit 2
fi

# name → expected_max_bytes (0 = должен быть строго пуст)
EMPTY_FILES="memory.db rob_box_voice.db voice_assistant.db waypoints.db voice.db"
ARCHIVE_FILES="operator_memory.db operator_memory.db-shm operator_memory.db-wal"

log "DATA_DIR=$DATA_DIR  режим=$([ "$APPLY" = 1 ] && echo APPLY || echo DRY-RUN)"
log ""
log "=== пустые файлы (0 байт) — кандидаты на прямое удаление ==="
for f in $EMPTY_FILES; do
    path="$DATA_DIR/$f"
    if [ ! -e "$path" ]; then
        log "  $f — уже отсутствует, пропуск"
        continue
    fi
    size="$(wc -c < "$path" 2>/dev/null | tr -d ' ')"
    if [ "${size:-1}" != "0" ]; then
        log "  ⚠️ $f — сейчас $size байт (НЕ 0, как в issue #2751) — ПРОПУСКАЮ, не трогаю без ручного разбора"
        continue
    fi
    if [ "$APPLY" = 1 ]; then
        rm -f -- "$path" && log "  ✅ удалён: $f (был 0 байт)" \
            || log "  ❌ не удалось удалить: $f"
    else
        log "  [DRY-RUN] rm -f -- '$path'"
    fi
done

log ""
log "=== operator_memory.db (упразднён ADR-0083 §E) — архивируется, не удаляется ==="
archive_dir="$DATA_DIR/_archive_2751"
any_present=0
for f in $ARCHIVE_FILES; do
    [ -e "$DATA_DIR/$f" ] && any_present=1
done
if [ "$any_present" = 0 ]; then
    log "  operator_memory.db (и -shm/-wal) уже отсутствуют — нечего архивировать"
else
    if [ "$APPLY" = 1 ]; then
        mkdir -p "$archive_dir"
        for f in $ARCHIVE_FILES; do
            [ -e "$DATA_DIR/$f" ] || continue
            mv -- "$DATA_DIR/$f" "$archive_dir/$f" \
                && log "  ✅ перенесён в архив: $f → _archive_2751/$f" \
                || log "  ❌ не удалось перенести: $f"
        done
    else
        for f in $ARCHIVE_FILES; do
            [ -e "$DATA_DIR/$f" ] || continue
            log "  [DRY-RUN] mv -- '$DATA_DIR/$f' '$archive_dir/$f'"
        done
    fi
fi

log ""
log "=== НЕ трогается этим скриптом (осознанно) ==="
log "  speakers.db, speakers.db.bak-* — issue #2750, решение за владельцем"
log "  harness_voice.db, voice_memory.db, music_library/index.db — живые"
log ""
if [ "$APPLY" = 0 ]; then
    log "Это был dry-run. Повторите с --apply, чтобы применить."
fi
