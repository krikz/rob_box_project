#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/voice_init_wait.sh
#
# Каноническая версия живёт в репо. На хост раскладывается через
#   bash <repo>/scripts/agent_flow/install.sh
# который создаёт hardlink-копии в:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
#
# Правка: редактируем <repo>/scripts/agent_flow/voice_init_wait.sh,
# commit, merge. На хост: bash <repo>/scripts/agent_flow/install.sh.
# ============================================================================
# voice_init_wait.sh — фикс race condition между voice-resources-init (Exited)
# и последующим health-check / docker compose ps (issue #2095, deploy #34144648195).
#
# КОНТЕКСТ
#   voice-resources-init — одноразовый init-контейнер из docker/vision/
#   docker-compose.yaml:183-191. Копирует renardo samples в shared volume,
#   после чего Exited (restart: "no"). supercollider и voice-assistant
#   зависят от него через `depends_on: { condition: service_completed_successfully }`.
#
#   ПРОБЛЕМА (CI log deploy #34144648195, шаг [Vision Pi] Start Containers):
#     16:46:44  Container voice-resources-init Exited
#     16:46:50  Container supercollider Healthy
#     16:46:50  Error response from daemon: No such container: 6fc9913da...
#     16:46:50  ##[error]Process completed with exit code 1
#
#   docker daemon удерживает name-slot для свеже-exited контейнера ~5-6 сек
#   (ретро 18.08 t_9d35468d / round-130). `docker compose up -d` после того
#   как init-container Exited — пытается resolve exited ID в своём internal
#   reconcile → "No such container" → exit 1.
#
# ФИКС (принцип — k3s-style "wait for the sidecar to settle")
#   1. `docker wait <name>` — блокирует ровно до exit init-контейнера.
#      Если контейнер уже Exited (race с parallel deps) — docker wait
#      возвращает exit code мгновенно.
#   2. `sleep <DAEMON_RELEASE_WAIT_SECONDS>` — даём daemon освободить
#      name-slot. 8 сек — консервативно с запасом (эмпирически ~5-6 сек).
#   3. `docker rm -f <name>` — гарантированно удалить exited-контейнер,
#      чтобы daemon закрыл name-slot принудительно.
#
# ПОЧЕМУ НЕ --remove-orphans (acceptance #2 из issue)
#   --remove-orphans удаляет контейнеры, не упомянутые в текущем compose-
#   файле, а НЕ автоматически cleanup'ит exited init-контейнеры. В
#   rob_box compose есть volumes без сервисов и alias-сервисы — флаг
#   непредсказуем. Явный wait+sleep+rm проще и audit-friendly.
#
# ПОЧЕМУ ОТДЕЛЬНЫМ СКРИПТОМ (а не inline в workflow)
#   - Тестируемость: unit-тест с mock docker CLI (см. tests/test_voice_init_wait.sh).
#   - SOT-конвенция agent_flow: всякий часто-используемый bash — в scripts/.
#   - Возможность переиспользования вне deploy workflow (manual SSH recovery,
#     deploy-sweep, post-merge-build и т.п.).
#
# Usage:
#   voice_init_wait.sh [--container NAME] [--wait SECONDS] [--timeout SECONDS]
# Env (для тестов):
#   DOCKER_CMD   — путь к docker/mock (default: docker)
#   INIT_CONTAINER_NAME, DAEMON_RELEASE_WAIT_SECONDS, WAIT_TIMEOUT_SECONDS
#
# Exit codes:
#   0  — успешно (init дождался, name-slot освобождён, контейнер удалён)
#   1  — init не существует (не запускался) — noop, тоже success-soft
#   2  — usage (неверные аргументы)
#   3  — timeout при docker wait (init не завершился за WAIT_TIMEOUT_SECONDS)
# ============================================================================
set -euo pipefail

PREFIX="[voice_init_wait]"
INIT_CONTAINER_NAME="${INIT_CONTAINER_NAME:-voice-resources-init}"
DAEMON_RELEASE_WAIT_SECONDS="${DAEMON_RELEASE_WAIT_SECONDS:-8}"
WAIT_TIMEOUT_SECONDS="${WAIT_TIMEOUT_SECONDS:-120}"
DOCKER_CMD="${DOCKER_CMD:-docker}"

usage() {
    cat <<EOF
Usage: $0 [--container NAME] [--wait SECONDS] [--timeout SECONDS]

  --container NAME   Имя init-контейнера (default: voice-resources-init)
  --wait SECONDS     Сколько спать после exit для освобождения name-slot
                     (default: 8, эмпирически ~5-6 сек требуется docker daemon)
  --timeout SECONDS  Сколько ждать exit init-контейнера через docker wait
                     (default: 120)

Env overrides: INIT_CONTAINER_NAME, DAEMON_RELEASE_WAIT_SECONDS,
               WAIT_TIMEOUT_SECONDS, DOCKER_CMD.
EOF
}

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --container)
            INIT_CONTAINER_NAME="$2"
            shift 2
            ;;
        --wait)
            DAEMON_RELEASE_WAIT_SECONDS="$2"
            shift 2
            ;;
        --timeout)
            WAIT_TIMEOUT_SECONDS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "$PREFIX [FATAL] unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

# Sanity: numeric args
for v in "$DAEMON_RELEASE_WAIT_SECONDS" "$WAIT_TIMEOUT_SECONDS"; do
    if ! [[ "$v" =~ ^[0-9]+$ ]]; then
        echo "$PREFIX [FATAL] --wait/--timeout must be non-negative integers, got: $v" >&2
        exit 2
    fi
done

# Sanity: docker CLI доступен (или его mock для тестов)
if ! command -v "$DOCKER_CMD" >/dev/null 2>&1; then
    echo "$PREFIX [FATAL] docker command not found: $DOCKER_CMD" >&2
    exit 2
fi

log() { echo "$PREFIX $*"; }

log "init_container=$INIT_CONTAINER_NAME daemon_release_wait=${DAEMON_RELEASE_WAIT_SECONDS}s wait_timeout=${WAIT_TIMEOUT_SECONDS}s"

# Шаг 0: проверить что контейнер существует. Если нет — noop (init не запускался
# в этом деплое, нечего чистить). Не fail — это нормальный кейс (например,
# profile=quest отключает voice-assistant).
if ! "$DOCKER_CMD" ps -a --filter "name=^/${INIT_CONTAINER_NAME}\$" --format '{{.Names}}' \
        | grep -qx "${INIT_CONTAINER_NAME}"; then
    log "container '$INIT_CONTAINER_NAME' not found — nothing to wait (noop)"
    exit 0
fi

# Шаг 1: дождаться exit init-контейнера. `docker wait` блокирует до завершения.
# Если уже Exited — возвращает мгновенно.
#
# Exit code `timeout`:
#   - 124: timeout сработал, процесс был убит (SIGTERM, процесс не перехватил)
#   - 137: timeout сработал, процесс убит SIGKILL (128+9)
#   - иное: ошибка самого timeout или docker wait
# Проверяем оба (124/137), потому что некоторые sleep-loop'ы внутри docker
# wait ловят SIGTERM и выходят штатно — тогда exit 124; в редких случаях
# (process группа) — SIGKILL, exit 137.
#
# NB: используем явный `[[ ... ]]` вместо `if !` — `if ! cmd; then EXIT_CODE=$?`
# даёт $?==0 (exit code самого `!`), не cmd. Поэтому сохраняем в переменную ДО
# проверки.
log "step 1/3: docker wait '$INIT_CONTAINER_NAME' (timeout=${WAIT_TIMEOUT_SECONDS}s)..."
WAIT_START=$(date +%s)
WAIT_RC=0
timeout "${WAIT_TIMEOUT_SECONDS}" "$DOCKER_CMD" wait "${INIT_CONTAINER_NAME}" || WAIT_RC=$?
if [[ $WAIT_RC -ne 0 ]]; then
    if [[ $WAIT_RC -eq 124 ]] || [[ $WAIT_RC -eq 137 ]]; then
        log "[FATAL] timeout (${WAIT_TIMEOUT_SECONDS}s) waiting for '$INIT_CONTAINER_NAME' to exit"
        log "        container likely hung — check 'docker logs $INIT_CONTAINER_NAME'"
        exit 3
    fi
    log "[FATAL] docker wait failed with exit $WAIT_RC"
    exit 1
fi
WAIT_ELAPSED=$(( $(date +%s) - WAIT_START ))
log "step 1/3 done (${WAIT_ELAPSED}s)"

# Шаг 2: дать daemon освободить name-slot. docker daemon удерживает name-slot
# ~5-6 сек после exit; ждём с запасом.
log "step 2/3: sleep ${DAEMON_RELEASE_WAIT_SECONDS}s (daemon name-slot release)..."
sleep "${DAEMON_RELEASE_WAIT_SECONDS}"
log "step 2/3 done"

# Шаг 3: явно удалить exited-контейнер. К этому моменту daemon уже должен
# был забыть ID, но rm -f гарантирует cleanup для downstream команд
# (`docker compose ps`, health-check и т.п.).
log "step 3/3: docker rm -f '$INIT_CONTAINER_NAME'..."
if "$DOCKER_CMD" rm -f "${INIT_CONTAINER_NAME}" 2>/dev/null; then
    log "step 3/3 done"
else
    log "step 3/3 [WARN] rm failed (container already gone — non-fatal)"
fi

log "voice_init_wait: SUCCESS"
exit 0