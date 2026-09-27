#!/usr/bin/env bash
# ============================================================================
# check_container_status.sh — регрессионный скрипт (issue #2122)
#
# Источник истины: <repo>/.github/scripts/check_container_status.sh
# Копия на runner (self-hosted) обновляется через checkout — НЕ через
# hardlink/symlink. Шаблон дистрибуции совпадает с deployment_issue_dedup.py
# (вызывается из .github/workflows/L-Deploy and Verify.yml).
#
# Ретро run #34153246013 (2026-09-07, staging): deploy отрапортовал success
# при контейнере rob-box-quest в бесконечном "Restarting (1)" loop, потому что
# старая проверка брала ОДИН снимок `docker compose ps --format json` сразу
# после фиксированного `sleep 30` и сравнивала `.State != "running"`.
# Контейнер под restart-policy в активном crash-loop ЛЕГИТИМНО показывает
# State=="running" на доли секунды между падениями — единичный неудачно
# попавший замер читает «здоров». Эмпирический repro (busybox crash-loop
# с ~1s «работы» перед exit 1, локально): 9 из 20 снятых раз в 400ms
# единичных снимков докладывали "running", хотя контейнер непрерывно
# падал. Тем самым run #34153246013 отрапортовал success с роботом в
# бесконечном «Restarting (1)» + мёртвыми ROS2-топиками.
#
# Фикс (двухзамерная проверка через `docker inspect`, см. коммит
# 70e863026 + 7f8939d28 в develop) вынесен из workflow в этот скрипт:
#
#   1. status=exited/dead                       → провал сразу
#   2. status=restarting, подтверждён ОБОИМИ     → провал
#   3. Health.Status=unhealthy в любом замере   → провал
#   4. RestartCount вырос между замерами        → провал (та самая гонка)
#
# Контрактно важно: правило «exited → провал» НЕ применяется к
# one-shot init-контейнерам (voice-resources-init и подобные) с
# ненулевым ExitCode=0, потому что они штатно завершаются с кодом 0 на
# ЗДОРОВОМ роботе. Провал только при ненулевом ExitCode. Проверено после
# успешного деплоя 2026-09-07.
#
# Этот скрипт:
#   * читает JSON-данные `docker inspect` через ssh на Pi;
#   * принимает SSH-параметры через env (PI_HOST, PI_USER, PI_PASSWORD,
#     PI_COMPOSE_DIR, PI_SSH_OPTS);
#   * печатает JSON-результат `{healthy: bool, failed_count: int,
#     findings: [...]}` в stdout — workflow его парсит и пишет outputs.
#   * в stderr — диагностика (как в workflow).
#   * в FINDINGS_FILE (env) — JSONL одна запись на каждый failed-контейнер
#     (для совместимости с остальным deploy-verify pipeline).
#
# Для тестов: docker-inspect мокается через PATH (`docker` → test wrapper).
# ============================================================================
set -euo pipefail

CONFIRM_INTERVAL="${CONFIRM_INTERVAL:-15}"

# Required env
: "${PI_HOST:?PI_HOST must be set (e.g. 10.1.1.11)}"
: "${PI_USER:?PI_USER must be set (e.g. ros2)}"
: "${PI_PASSWORD:?PI_PASSWORD must be set (sshpass -p)}"
: "${PI_COMPOSE_DIR:?PI_COMPOSE_DIR must be set (e.g. ~/rob_box_project/docker/vision)}"
: "${PI_SSH_OPTS:?PI_SSH_OPTS must be set (ssh options string)}"

# Optional env
SUMMARY_FILE="${SUMMARY_FILE:-/tmp/${PI_HOST//./_}_status_summary.txt}"
HEALTHY_OUTPUT="${HEALTHY_OUTPUT:-true}"  # default, переписывается ниже
FAILED_COUNT_OUTPUT="${FAILED_COUNT_OUTPUT:-0}"

# --- helpers ----------------------------------------------------------------
log() { printf '[check_container_status] %s %s\n' "$(date -Iseconds)" "$*" >&2; }

# Снять JSON `docker inspect` для всех контейнеров compose-проекта.
# Возвращает JSON-массив объектов {name, status, restarting, health,
# restarts, exitcode} (по одному на контейнер).
#
# Команда `docker inspect` берётся из PATH — для тестов подменяется
# mock-скриптом, который читает заранее заданные JSON из fixture-файла.
sample_containers() {
    local json
    json="$(sshpass -p "$PI_PASSWORD" ssh $PI_SSH_OPTS "${PI_USER}@${PI_HOST}" \
        "cd ${PI_COMPOSE_DIR} && for cid in \$(docker compose ps -aq); do docker inspect --format '{{json .}}' \"\$cid\"; done" \
        2>/dev/null || true)"
    [ -n "$json" ] || return 0

    # Проекция полей через jq; не падаем если jq пустой.
    printf '%s\n' "$json" | jq -c '{
        name: (.Name | ltrimstr("/")),
        status: .State.Status,
        restarting: .State.Restarting,
        health: (.State.Health.Status // "none"),
        restarts: .RestartCount,
        exitcode: .State.ExitCode
    }' 2>/dev/null || true
}

# --- main -------------------------------------------------------------------
log "🔍 Checking container status on ${PI_HOST} (compose: ${PI_COMPOSE_DIR})"

log "📸 Sample #1 (t=0s)..."
SAMPLE1="$(sample_containers)"
[ -n "$SAMPLE1" ] || log "(empty sample #1)"
# Печатаем каждую запись в STDERR (логи, не данные) — чтобы stdout оставался
# «чистым» для вызывающего (subshell вытаскивает оттуда JSON).
if [ -n "$SAMPLE1" ]; then
    echo "$SAMPLE1" | jq -r '"  \(.name): status=\(.status) restarting=\(.restarting) health=\(.health) restarts=\(.restarts)"' >&2 2>/dev/null || log "(jq projection failed for sample #1)"
fi

log "⏳ Waiting ${CONFIRM_INTERVAL}s before confirming (avoid a mid-crash-loop 'running' snapshot)..."
sleep "$CONFIRM_INTERVAL"

log "📸 Sample #2 (t=${CONFIRM_INTERVAL}s)..."
SAMPLE2="$(sample_containers)"
[ -n "$SAMPLE2" ] || log "(empty sample #2)"
if [ -n "$SAMPLE2" ]; then
    echo "$SAMPLE2" | jq -r '"  \(.name): status=\(.status) restarting=\(.restarting) health=\(.health) restarts=\(.restarts)"' >&2 2>/dev/null || log "(jq projection failed for sample #2)"
fi

FAILED_COUNT=0
: > "$SUMMARY_FILE"

NAMES="$(echo "$SAMPLE2" | jq -r '.name' 2>/dev/null | sort -u || true)"
[ -n "$NAMES" ] || log "(no containers in sample #2)"

FINDINGS_JSONL="${FINDINGS_FILE:-/dev/null}"

for name in $NAMES; do
    [ -n "$name" ] || continue
    S1="$(echo "$SAMPLE1" | jq -c --arg n "$name" 'select(.name==$n)' 2>/dev/null || true)"
    S2="$(echo "$SAMPLE2" | jq -c --arg n "$name" 'select(.name==$n)' 2>/dev/null || true)"
    [ -z "$S2" ] && continue
    [ -z "$S1" ] && S1='{}'

    ST1="$(echo "$S1" | jq -r '.status // "unknown"' 2>/dev/null || echo unknown)"
    ST2="$(echo "$S2" | jq -r '.status' 2>/dev/null || echo unknown)"
    HE1="$(echo "$S1" | jq -r '.health // "none"' 2>/dev/null || echo none)"
    HE2="$(echo "$S2" | jq -r '.health' 2>/dev/null || echo none)"
    RC1="$(echo "$S1" | jq -r '.restarts // empty' 2>/dev/null || true)"
    RC2="$(echo "$S2" | jq -r '.restarts' 2>/dev/null || true)"

    REASON=""
    EC2="$(echo "$S2" | jq -r '.exitcode // 0' 2>/dev/null || echo 0)"

    # One-shot init-контейнеры (voice-resources-init и подобные) штатно
    # заканчивают работу с кодом 0 и остаются в `exited` на ЗДОРОВОМ роботе —
    # проверено после успешного деплоя 2026-09-07. Валить деплой по факту
    # `exited` без разбора кода возврата — значит валить КАЖДЫЙ деплой.
    # Провал только при ненулевом коде.
    if [ "$ST2" = "exited" ] && [ "$EC2" != "0" ]; then
        REASON="State.Status=exited (ExitCode=$EC2)"
    elif [ "$ST2" = "dead" ]; then
        REASON="State.Status=dead"
    elif [ "$ST1" = "restarting" ] && [ "$ST2" = "restarting" ]; then
        REASON="State.Status=restarting (confirmed at t=0s and t=${CONFIRM_INTERVAL}s)"
    elif [ "$HE2" = "unhealthy" ] || [ "$HE1" = "unhealthy" ]; then
        REASON="Health.Status=unhealthy"
    elif [ -n "$RC1" ] && [ "$RC2" -gt "$RC1" ] 2>/dev/null; then
        REASON="RestartCount grew during the ${CONFIRM_INTERVAL}s confirmation window: $RC1 -> $RC2 (container crashed while being checked)"
    fi

    if [ -n "$REASON" ]; then
        FAILED_COUNT=$((FAILED_COUNT + 1))
        log "❌ $name: $REASON"
        {
            echo "=== $name ==="
            echo "Reason: $REASON"
            echo "State.Status: sample1=${ST1:-unknown} sample2=$ST2"
            echo "Health.Status: sample1=${HE1:-none} sample2=$HE2"
            echo "RestartCount: sample1=${RC1:-unknown} sample2=$RC2"
            echo ""
        } >> "$SUMMARY_FILE"

        # Findings для остального pipeline (формат совпадает с тем, что
        # workflow пишет руками в *_findings.jsonl).
        if [ "$FINDINGS_JSONL" != "/dev/null" ]; then
            jq -nc \
                --arg container "$name" \
                --arg kind "container_status" \
                --arg severity "critical" \
                --arg summary "Container is unhealthy or restart-looping" \
                --arg raw_text "$REASON (State.Status: sample1=${ST1:-unknown} sample2=$ST2, Health.Status: sample1=${HE1:-none} sample2=$HE2, RestartCount: sample1=${RC1:-unknown} sample2=$RC2)" \
                '{environment:env.ENVIRONMENT, scope:env.SCOPE, container:$container, kind:$kind, severity:$severity, summary:$summary, raw_text:$raw_text}' \
                >> "$FINDINGS_JSONL" 2>/dev/null || true
        fi
    fi
done

# Печатаем JSON для workflow (он парсит через jq).
HEALTHY_BOOL="true"
[ "$FAILED_COUNT" -gt 0 ] && HEALTHY_BOOL="false"

jq -nc \
    --argjson healthy "$( [ "$HEALTHY_BOOL" = "true" ] && echo true || echo false )" \
    --argjson failed_count "$FAILED_COUNT" \
    '{healthy: $healthy, failed_count: $failed_count}'

# Через env, переданный через source, отдаём обратно в workflow.
# Workflow вызывает `source` и читает HEALTHY_RESULT/FAILED_COUNT_RESULT.
HEALTHY_RESULT="$HEALTHY_BOOL"
FAILED_COUNT_RESULT="$FAILED_COUNT"
export HEALTHY_RESULT FAILED_COUNT_RESULT

# Дефолтный exit code = 0 (workflow проверяет HEALTHY_RESULT). В тестах
# удобно: скрипт не падает на каждом failed-контейнере, а сообщает
# через JSON.
exit 0
