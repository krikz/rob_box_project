#!/usr/bin/env bash
# ============================================================================
# check_container_status.sh — регрессионный скрипт (issue #2122)
#
# Источник истины: <repo>/scripts/ci/check_container_status.sh
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
#   3. Health.Status=unhealthy в любом замере   → провал,
#      КРОМЕ окна start_period (issue #5012): Docker сам ещё не считает
#      пробы этого окна провалом (docs: failures within start_period are
#      not counted towards the maximum number of retries), но Status уже
#      пишет "unhealthy". Возраст контейнера (State.StartedAt) короче
#      Config.Healthcheck.StartPeriod — не провал. После окна — провал,
#      как раньше. Поднимать start_period «на глаз» нельзя: лечит один
#      сервис и прячет долгий старт.
#   4. Health.Status=starting в любом замере    → не здоровье (issue #5114).
#      Внутри start_period Docker ещё не обязан вынести вердикт. Скрипт сам
#      ждёт конца самого позднего окна (started_at + start_period + slack) и
#      мерит снова, пока pending не опустеет или не кончится
#      MAX_PENDING_WAIT_S. После окна starting — провал
#      (reason=starting_after_start_period): run 45019034598 счёл nav2
#      здоровым, пока health=starting на обоих замерах, а controller_server
#      уже умер SIGILL. Нет start_period или StartedAt не разобрался —
#      льготы нет, starting остаётся провалом (как unhealthy без окна).
#      Ждать должен сам скрипт, а не workflow: Vision и Sound зовут его же,
#      и healthy=false на pending красил бы зелёный деплой (приёмка #5118).
#   5. RestartCount вырос между замерами        → провал (та самая гонка)
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
#     pending_start: bool, pending: [{name, started_at, start_period_ns}]}`
#     в stdout — workflow его парсит и пишет outputs. К финальному JSON
#     pending_start=false в нормальном случае: скрипт сам дождался конца
#     окон и перемерил (issue #5114, приёмка #5118). Поле остаётся для
#     диагностики — true, только если ожидание упёрлось в
#     MAX_PENDING_WAIT_S, а контейнер всё ещё starting внутри окна.
#   * в stderr — диагностика (как в workflow).
#   * в FINDINGS_FILE (env) — JSONL одна запись на каждый failed-контейнер
#     (для совместимости с остальным deploy-verify pipeline).
#   * EXPECTED_CONTAINERS (env, опционально) — имена, обязанные присутствовать
#     в compose ps -a; отсутствующий засчитывается как failed (ADR-0159, Sound Pi).
#
# Для тестов: docker-inspect мокается через PATH (`docker` → test wrapper).
# ============================================================================
set -euo pipefail

CONFIRM_INTERVAL="${CONFIRM_INTERVAL:-15}"
# Ожидание конца start_period, когда замер поймал health=starting внутри окна
# (issue #5114, приёмка #5118). PROBE_SLACK_S — запас после конца окна: первая
# проба healthcheck, которая уже имеет право красить контейнер, идёт с
# interval (у nav2 15s) плюс timeout. MAX_PENDING_WAIT_S — потолок суммарного
# ожидания; окно nav2 90s, oak-d 90s, sound 60s, больше 180s ждать нечего.
# Тесты ставят оба в 0: реальные секунды прогону не нужны.
PROBE_SLACK_S="${PROBE_SLACK_S:-25}"
MAX_PENDING_WAIT_S="${MAX_PENDING_WAIT_S:-180}"
PENDING_POLL_S="${PENDING_POLL_S:-5}"

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
# restarts, exitcode, started_at, start_period_ns} (по одному на контейнер).
# started_at — State.StartedAt (RFC3339Nano); start_period_ns —
# Config.Healthcheck.StartPeriod в наносекундах (0, если healthcheck нет).
#
# Команда `docker inspect` берётся из PATH — для тестов подменяется
# mock-скриптом, который читает заранее заданные JSON из fixture-файла.
sample_containers() {
    local json
    json="$(sshpass -p "$PI_PASSWORD" ssh $PI_SSH_OPTS "${PI_USER}@${PI_HOST}" \
        "cd ${PI_COMPOSE_DIR} && for cid in \$(docker compose ps -aq); do docker inspect --format '{{json .}}' \"\$cid\"; done" \
        2>/dev/null || true)"
    [ -n "$json" ] || return 0

    # -s, не потоковый jq: docker inspect пишет один JSON на строку, но мок и
    # редкий inspect отдают один объект без завершающего перевода строки, и
    # потоковый jq тогда молча ничего не печатает. Перемер читал бы пустоту.
    printf '%s\n' "$json" | jq -sc '.[] | {
        name: (.Name | ltrimstr("/")),
        status: .State.Status,
        restarting: .State.Restarting,
        health: (.State.Health.Status // "none"),
        restarts: .RestartCount,
        exitcode: .State.ExitCode,
        started_at: (.State.StartedAt // ""),
        start_period_ns: (.Config.Healthcheck.StartPeriod // 0)
    }' 2>/dev/null || true
}

# true, если замер unhealthy попал ВНУТРЬ start_period контейнера
# (issue #5012): возраст от State.StartedAt до момента замера короче
# Config.Healthcheck.StartPeriod. Нет StartedAt, нет start_period или
# дата не разобралась — не льгота, unhealthy остаётся провалом.
within_start_period() {  # $1=started_at (RFC3339) $2=start_period_ns $3=epoch секунд замера
    local started_at="$1" period_ns="$2" sampled_epoch="$3"
    [ -n "$started_at" ] && [ -n "$period_ns" ] && [ "$period_ns" -gt 0 ] 2>/dev/null || return 1
    local started_epoch
    started_epoch="$(date -d "$started_at" +%s 2>/dev/null)" || return 1
    [ -n "$started_epoch" ] || return 1
    local age_s=$((sampled_epoch - started_epoch))
    local period_s=$((period_ns / 1000000000))
    [ "$age_s" -lt "$period_s" ]
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
PENDING_FILE="$(mktemp)"
trap 'rm -f "$PENDING_FILE"' EXIT
: > "$SUMMARY_FILE"

NAMES="$(echo "$SAMPLE2" | jq -r '.name' 2>/dev/null | sort -u || true)"
[ -n "$NAMES" ] || log "(no containers in sample #2)"

FINDINGS_JSONL="${FINDINGS_FILE:-/dev/null}"

# Классификация одной пары замеров. Вынесена в функцию: при pending_start
# скрипт ждёт конца окна и прогоняет её снова на свежем замере (issue #5114).
# Читает и пишет глобальные SAMPLE1/SAMPLE2/NAMES/FAILED_COUNT/PENDING_FILE.
classify_samples() {
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
    # StartPeriod/StartedAt берём из более позднего замера: рестарт между
    # замерами двигает StartedAt, и льгота должна считаться от нового старта.
    SA="$(echo "$S2" | jq -r '.started_at // empty' 2>/dev/null || true)"
    [ -n "$SA" ] || SA="$(echo "$S1" | jq -r '.started_at // empty' 2>/dev/null || true)"
    SP="$(echo "$S2" | jq -r '.start_period_ns // 0' 2>/dev/null || echo 0)"

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
    elif [ -n "$RC1" ] && [ "$RC2" -gt "$RC1" ] 2>/dev/null; then
        # Раньше health-веток: рост RestartCount — падение между замерами,
        # даже если контейнер ещё starting/unhealthy внутри start_period.
        REASON="RestartCount grew during the ${CONFIRM_INTERVAL}s confirmation window: $RC1 -> $RC2 (container crashed while being checked)"
    elif { [ "$HE2" = "unhealthy" ] || [ "$HE1" = "unhealthy" ]; } \
        && ! within_start_period "$SA" "$SP" "$(date +%s)"; then
        # issue #5012: unhealthy внутри start_period — не провал (Docker ещё
        # не считает пробы окна). После окна и без данных об окне — провал.
        REASON="Health.Status=unhealthy"
    elif { [ "$HE2" = "starting" ] || [ "$HE1" = "starting" ]; }; then
        # issue #5114: starting — не здоровье ни внутри окна, ни после.
        # Внутри окна это не провал (Docker ещё не вынес вердикт); после окна
        # и без данных об окне — провал, reason=starting_after_start_period.
        if within_start_period "$SA" "$SP" "$(date +%s)"; then
            log "⏳ $name: Health.Status=starting inside start_period (not healthy, not failed yet)"
            jq -nc --arg name "$name" --arg started_at "$SA" --argjson start_period_ns "$SP" \
                '{name:$name, started_at:$started_at, start_period_ns:$start_period_ns}' \
                >> "$PENDING_FILE"
        else
            REASON="reason=starting_after_start_period (Health.Status=starting after Config.Healthcheck.StartPeriod)"
        fi
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
# Внутри classify_samples: перемер после ожидания окна обнуляет FAILED_COUNT,
# и проверка обязана пройтись по свежему списку, а не по первому.
# EXPECTED_CONTAINERS (опционально, через пробел) — container_name'ы, которые
# обязаны быть в `docker compose ps -a` узла. Без этого узел, где `compose up`
# ничего не поднял (или стек исчез), отчитывается `healthy: true` по пустому
# списку — ровно та «зелёная галочка без evidence», которую ADR-0018 запрещает.
# Задаёт только вызывающий, у которого есть источник истины (для Sound Pi — список
# сервисов без profile в docker/sound/docker-compose.yaml, сверяется тестом).
for expected in ${EXPECTED_CONTAINERS:-}; do
    if printf '%s\n' "$NAMES" | grep -Fxq -- "$expected"; then
        continue
    fi
    REASON="expected container is absent from 'docker compose ps -a' (never created or removed)"
    FAILED_COUNT=$((FAILED_COUNT + 1))
    log "❌ $expected: $REASON"
    {
        echo "=== $expected ==="
        echo "Reason: $REASON"
        echo ""
    } >> "$SUMMARY_FILE"
    if [ "$FINDINGS_JSONL" != "/dev/null" ]; then
        jq -nc \
            --arg container "$expected" \
            --arg kind "container_status" \
            --arg severity "critical" \
            --arg summary "Expected container is missing" \
            --arg raw_text "$REASON" \
            '{environment:env.ENVIRONMENT, scope:env.SCOPE, container:$container, kind:$kind, severity:$severity, summary:$summary, raw_text:$raw_text}' \
            >> "$FINDINGS_JSONL" 2>/dev/null || true
    fi
done
}

classify_samples

# starting внутри окна — не провал и не здоровье (issue #5114). Ждать конца
# окна должен сам скрипт: workflow пишет *_healthy из этого JSON сразу, и
# pending с healthy=false красит зелёный деплой (приёмка #5118 — та же дыра
# на Vision и Sound, окна там 60-90s). Провал рядом глушит ожидание: ждать
# уже нечего. После ожидания перемериваем; starting вне окна становится
# провалом на следующем проходе classify.
waited_s=0
while [ "$FAILED_COUNT" -eq 0 ] && [ -s "$PENDING_FILE" ] && [ "$waited_s" -lt "$MAX_PENDING_WAIT_S" ]; do
    # Конец самого позднего окна: started_at + start_period + slack. Сон не
    # длиннее шага опроса и не длиннее остатка cap: контейнер, ставший healthy
    # раньше дедлайна, не должен держать деплой до конца окна.
    now_epoch="$(date +%s)"
    latest_deadline=0
    while IFS= read -r row; do
        [ -n "$row" ] || continue
        row_started="$(printf '%s' "$row" | jq -r '.started_at // empty')"
        row_period="$(printf '%s' "$row" | jq -r '.start_period_ns // 0')"
        row_epoch="$(date -d "$row_started" +%s 2>/dev/null || echo 0)"
        row_deadline=$((row_epoch + row_period / 1000000000 + PROBE_SLACK_S))
        [ "$row_deadline" -gt "$latest_deadline" ] && latest_deadline="$row_deadline"
    done < "$PENDING_FILE"
    until_deadline=$((latest_deadline - now_epoch))
    [ "$until_deadline" -lt 0 ] && until_deadline=0
    remaining=$((MAX_PENDING_WAIT_S - waited_s))
    # PENDING_POLL_S=0 — перемер сразу, без сна (тесты). Ноль не значит
    # «не ждать»: иначе pending так и не перемерится.
    wait_s="$PENDING_POLL_S"
    [ "$until_deadline" -lt "$wait_s" ] && wait_s="$until_deadline"
    [ "$wait_s" -gt "$remaining" ] && wait_s="$remaining"
    [ "$wait_s" -lt 0 ] && wait_s=0
    log "⏳ pending start window; waiting ${wait_s}s before re-measuring (waited ${waited_s}s of ${MAX_PENDING_WAIT_S}s)"
    [ "$wait_s" -gt 0 ] && sleep "$wait_s"
    # Cap двигается и при нуле сна, иначе poll=0 крутит цикл, не исчерпав cap.
    step_s="$wait_s"
    [ "$step_s" -lt 1 ] && step_s=1
    waited_s=$((waited_s + step_s))
    # Свежий замер становится обоими сэмплами: двухзамерное подтверждение
    # между ними уже было, здесь нужен вердикт после конца окна.
    SAMPLE1="$(sample_containers)"
    SAMPLE2="$SAMPLE1"
    : > "$PENDING_FILE"
    : > "$SUMMARY_FILE"
    FAILED_COUNT=0
    NAMES="$(echo "$SAMPLE2" | jq -r '.name' 2>/dev/null | sort -u || true)"
    classify_samples
done

# Печатаем JSON для workflow (он парсит через jq).
# К этому месту pending в нормальном случае пуст: окна дождались и перемерили.
# pending_start=true остаётся только диагностикой — ожидание упёрлось в
# MAX_PENDING_WAIT_S, а контейнер всё ещё starting внутри окна. Это не
# здоровье: healthy=false, но failed_count=0 (провала нет, вердикта тоже).
HEALTHY_BOOL="true"
PENDING_START="false"
[ "$FAILED_COUNT" -gt 0 ] && HEALTHY_BOOL="false"
if [ "$FAILED_COUNT" -eq 0 ] && [ -s "$PENDING_FILE" ]; then
    HEALTHY_BOOL="false"
    PENDING_START="true"
fi
PENDING_JSON="$(jq -sc '.' "$PENDING_FILE")"

jq -nc \
    --argjson healthy "$( [ "$HEALTHY_BOOL" = "true" ] && echo true || echo false )" \
    --argjson failed_count "$FAILED_COUNT" \
    --argjson pending_start "$( [ "$PENDING_START" = "true" ] && echo true || echo false )" \
    --argjson pending "$PENDING_JSON" \
    '{healthy: $healthy, failed_count: $failed_count, pending_start: $pending_start, pending: $pending}'

# Через env, переданный через source, отдаём обратно в workflow.
# Workflow вызывает `source` и читает HEALTHY_RESULT/FAILED_COUNT_RESULT.
HEALTHY_RESULT="$HEALTHY_BOOL"
FAILED_COUNT_RESULT="$FAILED_COUNT"
export HEALTHY_RESULT FAILED_COUNT_RESULT

# Дефолтный exit code = 0 (workflow проверяет HEALTHY_RESULT). В тестах
# удобно: скрипт не падает на каждом failed-контейнере, а сообщает
# через JSON.
exit 0
