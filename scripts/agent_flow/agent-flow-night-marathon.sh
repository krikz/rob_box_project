#!/bin/bash
# ============================================================================
# agent-flow-night-marathon.sh — ночной голосовой марафон как отдельный раунд.
#
# Что это
# -------
# Cron-обёртка (no_agent job, every 1h) над scripts/e2e/run_night_marathon.sh.
# Раз в сутки, в заданный ночной час, прогоняет 117-шаговый голосовой марафон
# (10 актов, docs/e2e/night-voice-marathon.md) и заканчивает ДО ночного ревью,
# чтобы его результаты попали в дайджест этих же суток, а не следующих.
#
# Почему «every 1h» + внутренний гейт по часу, а не cron на 21:00
# ---------------------------------------------------------------
# Ровно как у agent-flow-nightly-review.sh: hermes-scheduler умеет
# schedule.kind=interval, а не «cron в 21:00». Час проверяем внутри, дедуп
# за сутки — sentinel-файлом. Тик вне окна стоит один `date` и exit 0.
#
# Взаимное исключение с e2e-ротацией
# ----------------------------------
# Робот один: ротация (agent-flow-e2e-process.sh, cron every 20m) и марафон
# играют команды в ОДИН динамик и слушают ОДИН микрофон. Параллельный запуск
# не замедляет прогоны, а обнуляет их смысл — harness ловит в
# `docker logs voice-assistant` реакции на чужие фразы («✅ ПОЛНЫЙ ЦИКЛ +
# PATTERN_MISS»). Развязка — sentinel $HERMES_HOME/state/robot-busy:
# run_night_marathon.sh его пишет, gate G3.5 в e2e-process.sh его читает и
# пропускает тик. Файл самопротухает (expected_end + ROBOT_BUSY_MAX_AGE),
# поэтому упавший ночью марафон не морозит ротацию до утра.
#
# Порядок ночи (local TZ, дефолты)
# --------------------------------
#   21:00  старт марафона              (NIGHT_MARATHON_HOUR)
#   ~01:00 марафон закончился, sentinel снят
#   02:00  дедлайн: новые акты не стартуют  (NIGHTLY_REVIEW_HOUR)
#   02:00  agent-flow-nightly-review.sh собирает дайджест за «вчера 00:00 →
#          сейчас» — акты марафона попадают в него как runs «L: E2E Voice Test»
#   04:00  agents_sleep PEAK, MAINTENANCE — всё спит
#
# ENV
# ---
#   NIGHT_MARATHON_HOUR          час старта окна, local TZ      (default 21)
#   NIGHT_MARATHON_WINDOW_HOURS  ширина окна в часах            (default 2)
#   NIGHT_MARATHON_DEADLINE_HOUR час, после которого новые акты не стартуют
#                                (default = NIGHTLY_REVIEW_HOUR, т.е. 2)
#   NIGHT_MARATHON_ACTS          подмножество актов, «1,2,3»    (default все)
#   NIGHT_MARATHON_REF           ветка сценариев                (default develop)
#   NIGHT_MARATHON_MAX_RUNTIME   потолок работы, сек            (default 21600)
#   NIGHT_MARATHON_FORCE=true    игнорировать окно и дедуп (ручной запуск)
#   NIGHT_MARATHON_DRY_RUN=1     показать план и выйти
# ============================================================================
set -uo pipefail   # без -e: ошибка прогона не должна убивать cron-job

PREFIX="[agent-flow-night-marathon]"

# HERMES_HOME/HOME нужны ДО af_load_profile_env — библиотека читает
# ${HERMES_HOME} напрямую, а cron зовёт нас без них (под set -u это падение).
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
export HOME="${HOME:-/home/builder}"
REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
GH_BIN="${GH_BIN:-gh}"
export GH_BIN

LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-night-marathon.lock}"
STATE_DIR="${NIGHT_MARATHON_STATE_DIR:-$HERMES_HOME/state}"

NIGHT_MARATHON_HOUR="${NIGHT_MARATHON_HOUR:-21}"
NIGHT_MARATHON_WINDOW_HOURS="${NIGHT_MARATHON_WINDOW_HOURS:-2}"
# Дедлайн по умолчанию — старт ночного ревью: марафон обязан закончиться до
# того, как ревью соберёт дайджест, иначе его акты попадут в ревью следующих
# суток (то есть через день после того, как что-то сломалось).
NIGHTLY_REVIEW_HOUR="${NIGHTLY_REVIEW_HOUR:-2}"
NIGHT_MARATHON_DEADLINE_HOUR="${NIGHT_MARATHON_DEADLINE_HOUR:-$NIGHTLY_REVIEW_HOUR}"
NIGHT_MARATHON_ACTS="${NIGHT_MARATHON_ACTS:-}"
NIGHT_MARATHON_REF="${NIGHT_MARATHON_REF:-develop}"
NIGHT_MARATHON_MAX_RUNTIME="${NIGHT_MARATHON_MAX_RUNTIME:-21600}"   # 6h
FORCE="${NIGHT_MARATHON_FORCE:-false}"
DRY_RUN="${NIGHT_MARATHON_DRY_RUN:-0}"

# Названия актов кириллические — под POSIX-локалью python в раннере падал бы
# с UnicodeEncodeError и тик умирал бы молча.
export PYTHONIOENCODING="${PYTHONIOENCODING:-utf-8}"

log() { printf '%s %s %s\n' "$PREFIX" "$(date -Iseconds)" "$*" >&2; }

# --- shared library bootstrap ------------------------------------------------
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=lib_agent_flow_common.sh
. "$_LIB_DIR_HERE/lib_agent_flow_common.sh"

af_load_profile_env ""

command -v python3 >/dev/null 2>&1 || { log "python3 not on PATH — skip"; exit 0; }
command -v "$GH_BIN" >/dev/null 2>&1 || { log "gh not on PATH — skip"; exit 0; }

# --- gates -------------------------------------------------------------------
af_flock_guard_or_exit "$LOCK_FILE"

if [ "${NIGHT_MARATHON_TEST_MODE:-0}" != "1" ]; then
    af_maintenance_gate_or_exit
fi

mkdir -p "$STATE_DIR" 2>/dev/null || true

# Марафонные сутки = сегодняшняя локальная дата на момент СТАРТА окна.
# Окно 21:00-23:00 не переходит через полночь, поэтому просто `date +%F`.
MARATHON_DATE="${NIGHT_MARATHON_DATE:-$(date +%F)}"
SENTINEL="$STATE_DIR/agent-flow-night-marathon.${MARATHON_DATE}.done"

_hour_now="$(date +%-H)"
_hour_end=$((NIGHT_MARATHON_HOUR + NIGHT_MARATHON_WINDOW_HOURS))
if [ "$FORCE" != "true" ]; then
    if [ "$_hour_now" -lt "$NIGHT_MARATHON_HOUR" ] || [ "$_hour_now" -ge "$_hour_end" ]; then
        log "вне окна [${NIGHT_MARATHON_HOUR}:00, ${_hour_end}:00) — сейчас ${_hour_now}:xx, skip"
        exit 0
    fi
    if [ -f "$SENTINEL" ]; then
        log "марафон за ${MARATHON_DATE} уже прогонялся (sentinel ${SENTINEL}) — skip"
        exit 0
    fi
fi

# --- материализация сценариев из origin/develop ------------------------------
# Рабочее дерево REPO_DIR ротация таскает по раундам, и на нём может не быть
# ни .github/e2e/scenarios/night/, ни свежего раннера. Берём и то и другое
# прямо из origin/<ref> в отдельный временный каталог — тогда прогон не
# зависит от того, на каком коммите застряло рабочее дерево.
if [ ! -d "$REPO_DIR/.git" ]; then
    log "REPO_DIR не git-репозиторий: $REPO_DIR — skip"
    exit 0
fi
if ! timeout 120 git -C "$REPO_DIR" fetch --quiet origin "$NIGHT_MARATHON_REF" 2>/dev/null; then
    log "⚠️ git fetch origin ${NIGHT_MARATHON_REF} не прошёл — работаю на том, что уже есть локально"
fi

WORK_DIR="$(mktemp -d "/tmp/night-marathon-${MARATHON_DATE}.XXXXXX")"
cleanup_work() { rm -rf "$WORK_DIR" 2>/dev/null || true; }
trap 'cleanup_work' EXIT

if ! git -C "$REPO_DIR" archive "origin/${NIGHT_MARATHON_REF}" \
        scripts/e2e .github/e2e/scenarios/night 2>/dev/null \
        | tar -x -C "$WORK_DIR" 2>/dev/null; then
    log "❌ не удалось выгрузить сценарии из origin/${NIGHT_MARATHON_REF}"
    log "   Ожидались scripts/e2e/ и .github/e2e/scenarios/night/ — проверь, что марафон смержен в ${NIGHT_MARATHON_REF}"
    exit 0
fi

RUNNER="$WORK_DIR/scripts/e2e/run_night_marathon.sh"
if [ ! -f "$RUNNER" ]; then
    log "❌ раннер не найден после выгрузки: $RUNNER — skip"
    exit 0
fi

REPORT_DIR="${NIGHT_MARATHON_REPORT:-$STATE_DIR/night-marathon/${MARATHON_DATE}}"
mkdir -p "$REPORT_DIR" 2>/dev/null || true

log "старт марафона: ref=${NIGHT_MARATHON_REF} акты=${NIGHT_MARATHON_ACTS:-все} дедлайн=${NIGHT_MARATHON_DEADLINE_HOUR}:00 отчёт=${REPORT_DIR}"

if [ "$DRY_RUN" = "1" ]; then
    DRY_RUN=1 ACTS="$NIGHT_MARATHON_ACTS" REF="$NIGHT_MARATHON_REF" \
        REPORT="$REPORT_DIR" bash "$RUNNER"
    log "NIGHT_MARATHON_DRY_RUN=1 — sentinel не ставлю"
    exit 0
fi

# Sentinel ставим ПЕРЕД прогоном, а не после: тик длится часами, а cron
# продолжает тикать каждый час внутри окна. Без раннего sentinel второй тик
# в 22:00 запустил бы второй марафон поверх первого — flock его бы отбил, но
# полагаться на один только flock хрупко (перезапуск scheduler'а снимает
# блокировку). Провал прогона фиксируем в самом sentinel, не в его отсутствии:
# «марафон за эти сутки состоялся» — факт, а не оценка.
printf 'started_at=%s\nref=%s\nacts=%s\nreport=%s\n' \
    "$(date -Iseconds)" "$NIGHT_MARATHON_REF" "${NIGHT_MARATHON_ACTS:-all}" "$REPORT_DIR" \
    > "$SENTINEL"

start_epoch="$(date +%s)"
timeout "$NIGHT_MARATHON_MAX_RUNTIME" env \
    ACTS="$NIGHT_MARATHON_ACTS" \
    REF="$NIGHT_MARATHON_REF" \
    REPORT="$REPORT_DIR" \
    DEADLINE_HOUR="$NIGHT_MARATHON_DEADLINE_HOUR" \
    bash "$RUNNER" 2>&1 | tee -a "$REPORT_DIR/runner.log"
rc="${PIPESTATUS[0]}"
elapsed=$(( $(date +%s) - start_epoch ))

case "$rc" in
    0)   verdict="all_green" ;;
    3)   verdict="robot_busy_skipped" ;;
    124) verdict="timeout_${NIGHT_MARATHON_MAX_RUNTIME}s" ;;
    *)   verdict="some_acts_red" ;;
esac
printf 'finished_at=%s\nelapsed_s=%s\nrc=%s\nverdict=%s\n' \
    "$(date -Iseconds)" "$elapsed" "$rc" "$verdict" >> "$SENTINEL"

log "марафон завершён за $((elapsed / 60)) мин: rc=${rc} verdict=${verdict}"
if [ -f "$REPORT_DIR/summary.tsv" ]; then
    log "итог по актам:"
    sed 's/^/    /' "$REPORT_DIR/summary.tsv" >&2
fi

# rc=3 (робот занят) — не провал марафона, а корректная уступка: снимаем
# sentinel суток, чтобы следующий тик внутри окна попробовал снова.
if [ "$rc" = "3" ]; then
    rm -f "$SENTINEL" 2>/dev/null || true
    log "робот был занят — sentinel суток снят, следующий тик в окне попробует снова"
fi

exit 0
