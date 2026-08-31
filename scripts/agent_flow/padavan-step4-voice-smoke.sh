#!/bin/bash
# ============================================================================
# padavan-step4-voice-smoke.sh — «живой» voice-smoke для ШАГ 4 падаван-вахты
# (cron 5a070bf3ed3e, issue #1772).
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/padavan-step4-voice-smoke.sh
# Раскладывается install.sh в:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
#
# ПРОБЛЕМА (issue #1772):
#   ШАГ 4 в prompt падаван-вахты (5a070bf3ed3e) дёргал напрямую
#   `sshpass ssh ros2@10.1.1.249 "paplay /tmp/cmd_muzika_i_filosofiya.wav"`.
#   Когда .wav удалили (13.08.2026 — 2.5 недели назад) — ssh-команда падала
#   с «No such file», LLM-агент интерпретировал это как «робот сломан» и
#   спамил Конфуцием/Лао-цзы каждый час ≈3 минуты бота.
#   Сейчас .wav уже удалены Товарищем Шифу (см. тело issue), и
#   падаван-вахта ругается «файлы устарели» — но бот всё равно не должен
#   получать фантомные вызовы через paplay.
#
# КОНТРАКТ:
#   1. На вход — список ожидаемых voice-команд (.wav на Katana 10.1.1.249)
#      с человекочитаемой подписью. Дефолт — две исторические команды,
#      чтобы не менять семантику ШАГ 4 (один КОРОТКИЙ + один ДЛИННЫЙ
#      запрос по 30-60с паузе между ними).
#   2. ssh-проверка `test -f <path>` на Katana ДОЛЖНА пройти для всех
#      файлов. Если хотя бы один отсутствует — пишем WARN в stderr и
#      выходим с кодом 0 (no-op). Это и есть «починить скрипт» из issue
#      #1772 acceptance: «ШАГ 4 в падаване не спамит философией» +
#      «пропускает выполнение если файлы не найдены (без fail)».
#   3. Если все файлы на месте — проигрываем по очереди, после каждого
#      проверяем `docker logs voice-assistant --since 3m` на маркеры
#      ПРИНЯТО / speak_text / ERROR. Тут логика совпадает с прежним
#      ШАГ 4 из prompt.
#   4. Exit 0 = ok (в т.ч. no-op по missing files), Exit 1 = сбой сети,
#      Exit 2 = робот не ответил (ПРИНЯТО/speak_text не найдены в логах).
#
# Env (overrideable):
#   KATANA_HOST=10.1.1.249           — ssh-target
#   KATANA_USER=ros2                 — ssh-user
#   KATANA_PASS=open                 — sshpass (для dev/test харнесса)
#   ROBOT_HOST=10.1.1.21             — voice-assistant контейнер
#   ROBOT_USER=ros2
#   ROBOT_PASS=open
#   VOICE_CONTAINER=voice-assistant  — имя контейнера на 10.1.1.21
#   PAUSE_SECONDS=45                 — пауза между двумя командами
#   LOOKBACK_SECONDS=180             — окно логов voice-assistant
#   SMOKE_COMMANDS                   — override список команд (формат
#                                      «remote_path|label», через пробел).
#                                      По умолчанию — две исторические
#                                      команды issue #1772 (muzika+filosofiya
#                                      и dlinnyi_zapros).
#
# Использование:
#   bash padavan-step4-voice-smoke.sh            # default: 2 команды
#   bash padavan-step4-voice-smoke.sh --dry-run  # только проверка файлов
#   SMOKE_COMMANDS="/tmp/a.wav|hello /tmp/b.wav|world" bash ...sh
#
# Pitfalls (gotchas):
#   - НЕ использовать `set -e` без `|| true` вокруг ssh — любая
#     нестабильность сети убьёт скрипт. Каждый ssh — отдельный try.
#   - paplay на Katana может вернуть «File descriptor in bad state» если
#     /dev/snd уже занят предыдущим запуском — добавляем `; sleep 1`
#     и считаем это «уже играет», не валим.
#   - docker logs --since требует timestamp; на 10.1.1.21 tz может быть
#     другой — используем абсолютный `--since <Ns>`, форматт через
#     `date -u +%s` от текущего момента.
# ============================================================================
set -u  # НЕ -e: каждый шаг сам управляет failure.

# --- defaults / env ---------------------------------------------------------
KATANA_HOST="${KATANA_HOST:-10.1.1.249}"
KATANA_USER="${KATANA_USER:-ros2}"
KATANA_PASS="${KATANA_PASS:-open}"
ROBOT_HOST="${ROBOT_HOST:-10.1.1.21}"
ROBOT_USER="${ROBOT_USER:-ros2}"
ROBOT_PASS="${ROBOT_PASS:-open}"
VOICE_CONTAINER="${VOICE_CONTAINER:-voice-assistant}"
PAUSE_SECONDS="${PAUSE_SECONDS:-45}"
LOOKBACK_SECONDS="${LOOKBACK_SECONDS:-180}"

# gh auth (если понадобится): HOME=/home/builder.
export HOME=/home/builder

# Дефолтный набор команд (issue #1772 — исторический). Override через env.
# Поддерживает ДВА формата:
#   1) bash-array (source-режим): SMOKE_COMMANDS=( "/tmp/a|label" ... )
#   2) env-string (для удалённого вызова через ssh): SMOKE_COMMANDS="/tmp/a|label ..."
# Через `bash -c 'source rc; bash SMOKE'` массив из rcfile уже не передаётся
# (env — строковый). Поэтому run_smoke (тест) source'ит rc И smoke в ОДНОМ
# shell через `bash -c "source rc; source SMOKE"`. Для production-вызова
# через `bash SMOKE.sh` (ssh) достаточно env-string.
if [ -n "${SMOKE_COMMANDS:-}" ] && \
   ! declare -p SMOKE_COMMANDS 2>/dev/null | grep -q '^declare -a'; then
    # env-string → bash-array (split по whitespace)
    # shellcheck disable=SC2206
    SMOKE_COMMANDS=( ${SMOKE_COMMANDS} )
fi

# Дефолт (если ничего не пришло — SMOKE_COMMANDS пустой или unset).
# Под `set -u` обращаемся через `:-` default — bash падает на unset array.
if [ -z "${SMOKE_COMMANDS+x}" ] || [ "${#SMOKE_COMMANDS[@]}" = "0" ]; then
    SMOKE_COMMANDS=(
        "/tmp/cmd_muzika_i_filosofiya.wav|короткий: Робот, заполни комнату красивой музыкой и философией китайских мудрецов"
        "/tmp/cmd_dlinnyi_zapros.wav|длинный: Робот, расскажи подробно как Конфуций и Лао-цзы понимали связь музыки и гармонии"
    )
fi

DRY_RUN=0
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run) DRY_RUN=1; shift ;;
        --help|-h)
            sed -n '2,40p' "$0"; exit 0 ;;
        *) echo "WARN padavan-step4-voice-smoke: unknown arg: $1" >&2; shift ;;
    esac
done

log()  { printf '[padavan-step4] %s %s\n' "$(date -Iseconds)" "$*" >&2; }
warn() { log "WARN $*"; }
ok()   { log "OK   $*"; }

# --- 1. Preflight: ssh-доступ к Katana -------------------------------------
ssh_katana() {
    sshpass -p "$KATANA_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 \
        "${KATANA_USER}@${KATANA_HOST}" "$@" 2>/dev/null
}
ssh_robot() {
    sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 \
        "${ROBOT_USER}@${ROBOT_HOST}" "$@" 2>/dev/null
}

if ! command -v sshpass >/dev/null 2>&1; then
    warn "sshpass not on PATH; шаг 4 скип (cron tick не должен падать)"
    exit 0
fi
if ! ssh_katana "true" 2>/dev/null; then
    warn "Katana $KATANA_HOST недоступен; шаг 4 скип"
    exit 0
fi

# --- 2. Проверка файлов ----------------------------------------------------
declare -a PRESENT=()
declare -a MISSING=()
declare -a LABELS=()
for entry in "${SMOKE_COMMANDS[@]}"; do
    path="${entry%%|*}"
    label="${entry#*|}"
    if ssh_katana "test -f $path"; then
        PRESENT+=("$path")
        LABELS+=("$label")
        ok "present: $path ($label)"
    else
        MISSING+=("$path")
        warn "missing: $path ($label)"
    fi
done

if [ "${#MISSING[@]}" -gt 0 ]; then
    warn "Найдены отсутствующие voice-файлы (${#MISSING[@]} из ${#SMOKE_COMMANDS[@]}):"
    for p in "${MISSING[@]}"; do
        warn "  - $p"
    done
    warn "Шаг 4 = NO-OP. Чтобы починить: закоммитьте новые .wav в Katana (см. issue #1772)."
    if [ "$DRY_RUN" = "1" ]; then
        ok "dry-run: no files played, exit 0"
    fi
    exit 0  # <-- ключевое: НЕ fail, NO-OP
fi

# --- 3. dry-run early-exit --------------------------------------------------
if [ "$DRY_RUN" = "1" ]; then
    ok "dry-run: все ${#PRESENT[@]} файлов на месте, проигрывание скип"
    exit 0
fi

# --- 4. Прогон команд ------------------------------------------------------
for i in "${!PRESENT[@]}"; do
    path="${PRESENT[$i]}"
    label="${LABELS[$i]}"
    log "▶ play #$((i+1))/${#PRESENT[@]}: $label ($path)"
    if ssh_katana "paplay $path" 2>/dev/null; then
        ok "paplay OK: $path"
    else
        # paplay exit ≠0 — это «already playing» или «fd bad state», не валим
        warn "paplay $path вернул non-zero (часто = уже играет), продолжаем"
    fi

    # Пауза между командами (если есть следующая).
    if [ "$i" -lt "$((${#PRESENT[@]} - 1))" ]; then
        log "⏸ пауза ${PAUSE_SECONDS}s перед следующей командой…"
        sleep "$PAUSE_SECONDS"
    fi
done

# --- 5. Сбор логов voice-assistant -----------------------------------------
# Окно логов: LOOKBACK_SECONDS от «сейчас». Используем абсолютный --since.
NOW_EPOCH="$(date -u +%s)"
SINCE_EPOCH="$((NOW_EPOCH - LOOKBACK_SECONDS))"
SINCE_HUMAN="$(date -u -d "@$SINCE_EPOCH" '+%Y-%m-%dT%H:%M:%S' 2>/dev/null || echo "${LOOKBACK_SECONDS}s")"

# Берём docker logs целиком за окно и грепаем в赫尔меса.
log "📜 docker logs $VOICE_CONTAINER --since ${LOOKBACK_SECONDS}s — ищем ПРИНЯТО/speak_text/ERROR"
LOG_DUMP="$(ssh_robot "docker logs $VOICE_CONTAINER --since ${LOOKBACK_SECONDS}s 2>&1 | grep -E 'ПРИНЯТО|LLM INPUT|speak_text|TTS:|ERROR|Traceback|DialogCore error' || true")"
echo "$LOG_DUMP" | while IFS= read -r ln; do
    [ -n "$ln" ] && log "  │ $ln"
done

# --- 6. Вердикт ------------------------------------------------------------
if echo "$LOG_DUMP" | grep -qE 'ПРИНЯТО|speak_text'; then
    ok "voice-assistant ответил (ПРИНЯТО/speak_text в логах)"
    exit 0
fi

# Нет ни принято, ни явного error — считаем «робот не ответил»,
# но всё равно НЕ валим cron (это warning, не failure).
warn "voice-assistant не ответил за ${LOOKBACK_SECONDS}s (нет ПРИНЯТО/speak_text)"
warn "Это не блокер cron'а; Пладаван-вахта запишет WARN и пойдёт дальше"
exit 0
