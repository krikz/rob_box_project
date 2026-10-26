#!/usr/bin/env bash
# =============================================================================
# run_night_marathon.sh — ночной голосовой марафон (117 шагов, 10 актов).
#
# Зачем отдельный раннер, а не один workflow_dispatch
# ---------------------------------------------------
# «L-E2E Voice Test.yml» имеет timeout-minutes: 45, а шаг стоит 60-180 секунд
# (замер по run 34385886254: 11 шагов = 26.5 минуты). 117 шагов в один job
# не влезают физически. Марафон — это ОДНА история, поэтому акты
# запускаются СТРОГО ПО ПОРЯДКУ и на одном и том же роботе: акт 3 опирается
# на профили голосов, зарегистрированные в акте 2, акт 9 — на факт «не ем
# лук», сохранённый в акте 2, акт 10 — на то, что треки убраны в актах 5 и 7.
#
# Использование
# -------------
#   bash scripts/e2e/run_night_marathon.sh                 # все 10 актов
#   ACTS=3 bash scripts/e2e/run_night_marathon.sh          # только акт 3
#   ACTS=1,2,3 bash scripts/e2e/run_night_marathon.sh      # подмножество
#   REF=develop bash scripts/e2e/run_night_marathon.sh
#   DRY_RUN=1 bash scripts/e2e/run_night_marathon.sh       # показать план
#
# ENV:
#   REF        (default develop)  — ветка, с которой берётся workflow+сценарии
#   ACTS       (default all)      — список актов через запятую
#   REPORT     (default tests/e2e/_artifacts/night_<UTC>/)
#   POLL_SEC   (default 60)       — период опроса статуса
#   MAX_WAIT   (default 3600)     — потолок ожидания одного акта, сек
#   TTS_PROVIDER (default пусто = auto воркфлоу) — кто озвучивает команды на
#              билд-машине: auto|minimax|silero|yandex. Многодикторные акты —
#              minimax: auto выбирает неизмеренный Yandex, и Борис звучит как Саша.
#   E2E_RELEASE_CMD (default пусто) — команда снятия режима e2e на роботе: раннер выполнит её в trap,
#              если марафон оборвался, не дойдя до последнего акта (#3972). Пусто - режим снимет таймер
#              владельца speaker_id_node (~3 ч от начала последнего акта).
#   Режим e2e (БД дикторов e2e) держит раннер, а не акт (#3972): первый акт плана включает его и стирает
#   e2e-БД ОДИН раз (phase=first), остальные подхватывают без стирания (middle), последний выключает (last).
#   План из одного акта или начинающийся с наследующего акта идёт по-старому (#3904).
#   STOP_ON_FOUNDATION_FAIL (default 1) — прервать марафон, если акт 1 или 2
#              красный: без wake-word и без профилей голосов остальные акты
#              не диагностичны, они просто утонут в no_accept.
#
# ПЕРЕД ПРОГОНОМ для акта 8 ничего выставлять не нужно: политика перебивания
# одна, параметра нет (ADR-0150 PR-5 #3694).
#   ⚠️ если нужен ros2 param set — только через docker exec в контейнере
#   (ROS_ENV='source /opt/ros/humble/setup.bash; source /ws/install/setup.bash;'):
#   на ХОСТЕ Pi ros2 нет вообще. Форма `ssh <robot> 'ros2 ...'` молча
#   возвращает rc=127 — из-за этого изоляция БД дикторов из #2759 не
#   срабатывала ни разу с момента мержа, см. issue #2763.
# =============================================================================
set -u

REPO_SLUG="${REPO_SLUG:-krikz/rob_box_project}"
REF="${REF:-develop}"
WORKFLOW="${WORKFLOW:-L-E2E Voice Test.yml}"
POLL_SEC="${POLL_SEC:-60}"
MAX_WAIT="${MAX_WAIT:-3600}"
DRY_RUN="${DRY_RUN:-0}"
STOP_ON_FOUNDATION_FAIL="${STOP_ON_FOUNDATION_FAIL:-1}"
TTS_PROVIDER="${TTS_PROVIDER:-}"
TTS_ARGS=()
[ -n "$TTS_PROVIDER" ] && TTS_ARGS=(-f tts_provider="$TTS_PROVIDER")

# --- robot-busy sentinel (взаимное исключение с e2e-ротацией) ---------------
# Робот один: ротация (agent-flow-e2e-process.sh, cron every 20m) и марафон
# играют команды в ОДИН динамик и слушают ОДИН микрофон. Параллельный запуск
# не замедляет прогоны — он делает их бессмысленными: harness видит в логах
# реакции на чужие фразы («✅ ПОЛНЫЙ ЦИКЛ + PATTERN_MISS»).
# Контракт файла — одна строка: <owner> <started_epoch> <expected_end_epoch> <note>
# Ротация читает его в гейте G3.5 и пропускает тик, пока мы держим робота.
HERMES_HOME="${HERMES_HOME:-$HOME/.hermes}"
ROBOT_BUSY_SENTINEL="${ROBOT_BUSY_SENTINEL:-$HERMES_HOME/state/robot-busy}"
SKIP_SENTINEL="${SKIP_SENTINEL:-0}"
# Сколько часов резервируем робота под марафон. По замеру run 34385886254
# (11 шагов = 26.5 мин) 117 шагов — это 2.5-5 часов; 6 даёт запас на ретраи.
SENTINEL_HOURS="${SENTINEL_HOURS:-6}"

# --- дедлайн (default: не позже ночного ревью) ------------------------------
# agent-flow-nightly-review.sh стартует в NIGHTLY_REVIEW_HOUR (default 2:00
# local) и собирает дайджест за окно «вчера 00:00 → сейчас». Марафон обязан
# ЗАКОНЧИТЬСЯ до этого, иначе его акты попадут в ревью следующих суток —
# то есть через день после того, как что-то сломалось.
# DEADLINE_HOUR="" отключает дедлайн (ручной прогон без спешки).
DEADLINE_HOUR="${DEADLINE_HOUR:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
# shellcheck source=marathon_markers.sh
. "$SCRIPT_DIR/marathon_markers.sh"
MANIFEST="$ROOT/.github/e2e/scenarios/night/night_marathon_manifest.json"

if [ ! -f "$MANIFEST" ]; then
    echo "FATAL: манифест не найден: $MANIFEST" >&2
    echo "       сгенерируй: py -3 scripts/e2e/gen_night_marathon.py" >&2
    exit 2
fi

RUN_TS="$(date -u +%Y%m%d_%H%M%S)"
REPORT="${REPORT:-$ROOT/tests/e2e/_artifacts/night_${RUN_TS}}"
mkdir -p "$REPORT"
SUMMARY="$REPORT/summary.tsv"
# #3989: recovered — шагов, прошедших после восстановления харнессом (не чистый PASS); act_recover — что харнесс
# чинил на старте акта (утечка прошлого акта: music, silence). Оба — из строки E2E_SUMMARY лога акта.
printf 'act\ttitle\tstability\tsteps\trun_id\tconclusion\turl\trecovered\tact_recover\n' > "$SUMMARY"

log() { printf '[night %s] %s\n' "$(date -u +%H:%M:%S)" "$*"; }

# --- какие акты гоняем ------------------------------------------------------
ACTS_FILTER="${ACTS:-}"

# Манифест читаем питоном (jq на билд-машинах есть не всегда, python3 — есть).
# python3 на Windows-хостах бывает WindowsApps-заглушкой: она есть в PATH,
# но при запуске уходит в Microsoft Store. Проверяем ЗАПУСКОМ, а не наличием.
PY=""
for _cand in "${PY_BIN:-}" python3 python py; do
    [ -z "$_cand" ] && continue
    if "$_cand" -c 'import sys' >/dev/null 2>&1; then PY="$_cand"; break; fi
done
if [ -z "$PY" ]; then
    echo "FATAL: не найден рабочий python3 (пробовал python3/python/py)" >&2
    exit 2
fi
plan="$("$PY" - "$MANIFEST" "$ACTS_FILTER" "$ROOT" <<'PYEOF'
import json, os, sys
# Названия актов кириллические; на Windows-хосте stdout по умолчанию cp1252
# и print() падает с UnicodeEncodeError ещё до первого диспатча.
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass
manifest, flt, root = sys.argv[1], sys.argv[2], sys.argv[3]
acts = json.load(open(manifest, encoding="utf-8"))["acts"]
if flt.strip():
    want = {int(x) for x in flt.replace(" ", "").split(",") if x}
    acts = [a for a in acts if a["act"] in want]


def inherits(act):
    with open(os.path.join(root, act["scenario_file"]), encoding="utf-8") as fh:
        return json.load(fh).get("inherits_previous_act", False) is True


# Issue #3972: режим e2e живёт весь марафон и принадлежит раннеру: first включает (стирает e2e-БД один раз),
# middle подхватывает без стирания, last подхватывает и выключает. Нужны >= 2 актов, и первый не должен
# наследовать: наследующий акт без режима обязан упасть на E2E_PRECONDITION (ACTS=3,4 отдельно), а не
# получить свежий пустой каст. Иначе (один акт / план с наследующего) - старое поведение #3904: акт держит
# режим для следующего, только если следующий акт ЭТОГО плана наследует (inherits_previous_act).
n = len(acts)
phased = n >= 2 and not inherits(acts[0])
holds = [i + 1 < n and inherits(acts[i + 1]) for i in range(n)]
for i, (a, hold) in enumerate(zip(acts, holds)):
    phase = ("first" if i == 0 else "last" if i == n - 1 else "middle") if phased else "-"
    print("\t".join([
        str(a["act"]), a["title"], a["stability"], str(a["steps"]),
        a["scenario_file"], a["acceptance_file"], phase,
        "hold" if hold and not phased else "-",
    ]))
PYEOF
)"

# Python на Windows-хосте печатает \r\n: \r прилипал к последней колонке плана
# (a_hold="hold\r" — акт не держал режим e2e для наследующего, #3904).
plan="$(printf '%s' "$plan" | tr -d '\r')"
if [ -z "$plan" ]; then
    echo "FATAL: ни один акт не выбран (ACTS=$ACTS_FILTER)" >&2
    exit 2
fi

n_acts="$(printf '%s\n' "$plan" | wc -l | tr -d ' ')"
log "REF=$REF  актов=$n_acts  tts_provider=${TTS_PROVIDER:-auto}  отчёт=$REPORT"
printf '%s\n' "$plan" | while IFS=$'\t' read -r a_n a_title a_stab a_steps a_scn a_acc a_phase a_hold; do
    log "  акт $a_n  «$a_title»  ($a_steps шагов, $a_stab)$([ "$a_phase" != - ] && echo "  → режим e2e марафона: $a_phase")$([ "$a_hold" = hold ] && echo '  → режим e2e держит для следующего (наследует)')"
done

if [ "$DRY_RUN" = "1" ]; then
    log "DRY_RUN=1 — ничего не запускаю"
    exit 0
fi

# --- дедлайн: во сколько прекратить запускать новые акты ---------------------
DEADLINE_EPOCH=0
if [ -n "$DEADLINE_HOUR" ]; then
    DEADLINE_EPOCH="$(date -d "today ${DEADLINE_HOUR}:00" +%s 2>/dev/null || echo 0)"
    # Дедлайн ночью после старта вечером — это уже завтрашние сутки.
    if [ "$DEADLINE_EPOCH" -gt 0 ] && [ "$DEADLINE_EPOCH" -le "$(date +%s)" ]; then
        DEADLINE_EPOCH="$(date -d "tomorrow ${DEADLINE_HOUR}:00" +%s 2>/dev/null || echo 0)"
    fi
    [ "$DEADLINE_EPOCH" -gt 0 ] \
        && log "дедлайн: $(date -d "@$DEADLINE_EPOCH" +'%F %H:%M') — после него новые акты не стартуют"
fi

# --- режим e2e на роботе держит раннер (#3972) ---------------------------------
# Включает/стирает e2e-БД акт first, снимает акт last (его trap в харнессе). Раннер сам до робота не
# ходит (он диспатчит workflow), поэтому при обрыве (Ctrl-C, break, дедлайн, dispatch_failed) режим
# остаётся на таймере владельца speaker_id_node (e2e_release_after_s, скользящий, ~3 ч от начала
# последнего акта). Здесь - громкая запись об этом и необязательный E2E_RELEASE_CMD (команда снятия
# режима, например ssh на робота), которую раннер выполнит в trap, если last не отработал.
PHASE_STARTED=0
PHASE_RELEASED=0
E2E_RELEASE_CMD="${E2E_RELEASE_CMD:-}"
E2E_MARATHON_HOLD_S="${E2E_MARATHON_HOLD_S:-10800}"
release_e2e_mode_on_exit() {
    [ "$PHASE_STARTED" = "1" ] && [ "$PHASE_RELEASED" != "1" ] || return 0
    PHASE_RELEASED=1
    log "⚠️ марафон закончился, не дойдя до акта last: режим e2e на роботе мог остаться включённым (БД дикторов e2e, живые люди не узнаются)"
    if [ -n "$E2E_RELEASE_CMD" ]; then
        log "   снимаю режим: E2E_RELEASE_CMD"
        bash -c "$E2E_RELEASE_CMD" </dev/null || log "   ❌ E2E_RELEASE_CMD не отработала - режим снимется сам по таймеру владельца (~$((E2E_MARATHON_HOLD_S / 60)) мин от начала последнего акта)"
    else
        log "   сам снимется по таймеру владельца (~$((E2E_MARATHON_HOLD_S / 60)) мин от начала последнего акта); сразу: ros2 param set /speaker_id_node e2e_mode false (docker exec voice-assistant на Sound Pi)"
    fi
}
on_exit() { release_e2e_mode_on_exit; release_sentinel; }
trap 'on_exit' EXIT
trap 'on_exit; exit 130' INT
trap 'on_exit; exit 143' TERM

# --- захват робота ----------------------------------------------------------
SENTINEL_TAKEN=0
release_sentinel() {
    [ "$SENTINEL_TAKEN" = "1" ] || return 0
    rm -f "$ROBOT_BUSY_SENTINEL" 2>/dev/null || true
    SENTINEL_TAKEN=0
    log "robot-busy sentinel снят — ротация свободна"
}
if [ "$SKIP_SENTINEL" = "1" ]; then
    log "⚠️ SKIP_SENTINEL=1 — робот НЕ резервируется, ротация может влезть в прогон"
else
    mkdir -p "$(dirname "$ROBOT_BUSY_SENTINEL")" 2>/dev/null || true
    if [ -f "$ROBOT_BUSY_SENTINEL" ]; then
        _cur="$(head -1 "$ROBOT_BUSY_SENTINEL" 2>/dev/null || true)"
        _cur_owner="$(printf '%s' "$_cur" | awk '{print $1}')"
        _cur_end="$(printf '%s' "$_cur" | awk '{print $3}')"
        case "${_cur_end:-}" in ''|*[!0-9]*) _cur_end=0 ;; esac
        if [ "$_cur_end" -gt "$(date +%s)" ]; then
            log "❌ робот уже занят (owner=${_cur_owner:-?}, до $(date -d "@$_cur_end" +'%H:%M')) — не лезу"
            log "   Если это протухший sentinel: rm $ROBOT_BUSY_SENTINEL"
            exit 3
        fi
        log "⚠️ найден просроченный robot-busy sentinel (owner=${_cur_owner:-?}) — перехватываю"
    fi
    _end_epoch=$(( $(date +%s) + SENTINEL_HOURS * 3600 ))
    # Если задан дедлайн — резервируем ровно до него, не дольше: иначе
    # упавший марафон держал бы ротацию заблокированной ещё несколько часов.
    if [ "$DEADLINE_EPOCH" -gt 0 ] && [ "$DEADLINE_EPOCH" -lt "$_end_epoch" ]; then
        _end_epoch="$DEADLINE_EPOCH"
    fi
    printf 'night-marathon %s %s %s\n' "$(date +%s)" "$_end_epoch" "run_night_marathon.sh acts=$n_acts ref=$REF" \
        > "$ROBOT_BUSY_SENTINEL"
    SENTINEL_TAKEN=1
    log "robot-busy sentinel взят до $(date -d "@$_end_epoch" +'%F %H:%M') — ротация будет пропускать тики"
fi

overall=0
while IFS=$'\t' read -r a_n a_title a_stab a_steps a_scn a_acc a_phase a_hold; do
    [ -z "$a_n" ] && continue

    # Дедлайн проверяем ПЕРЕД диспатчем: акт живёт до 45 минут, и стартовать
    # его за десять минут до ночного ревью бессмысленно — результат приедет
    # уже после того, как дайджест собран.
    if [ "$DEADLINE_EPOCH" -gt 0 ] && [ "$(date +%s)" -ge "$DEADLINE_EPOCH" ]; then
        log "⏰ дедлайн $(date -d "@$DEADLINE_EPOCH" +'%H:%M') прошёл — акт $a_n и последующие не запускаю"
        printf '%s\t%s\t%s\t%s\t-\tskipped_deadline\t-\t-\t-\n' \
            "$a_n" "$a_title" "$a_stab" "$a_steps" >> "$SUMMARY"
        continue
    fi

    log "=== АКТ $a_n «$a_title» ($a_steps шагов, $a_stab) ==="
    # Issue #3904 — следующий акт наследует этот: режим e2e на роботе не
    # снимать в trap этого акта (снимет последний акт или таймер владельца).
    HOLD_ARGS=()
    [ "$a_hold" = "hold" ] && HOLD_ARGS=(-f e2e_hold_for_next_act=true)
    # Issue #3972 - режим e2e держит раннер: одно включение/стирание на марафон (first), остальные акты
    # его только подхватывают, last выключает. Без этого акты 2/2b/2c стирали каст друг друга.
    if [ "$a_phase" != "-" ]; then
        HOLD_ARGS+=(-f e2e_marathon_phase="$a_phase")
        PHASE_STARTED=1
    fi

    # Метка времени ДО диспатча: по ней ищем свой run среди чужих.
    before_iso="$(date -u -d '-2 minutes' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
                  || date -u +%Y-%m-%dT%H:%M:%SZ)"

    if ! gh workflow run "$WORKFLOW" \
            --repo "$REPO_SLUG" \
            --ref "$REF" \
            -f environment=test \
            -f scenario_file="$a_scn" \
            -f acceptance_file="$a_acc" \
            -f triggered_by_script="run_night_marathon.sh" \
            -f triggered_by_reason="night marathon act $a_n: $a_title" \
            ${TTS_ARGS[@]+"${TTS_ARGS[@]}"} \
            ${HOLD_ARGS[@]+"${HOLD_ARGS[@]}"} \
            >/dev/null 2>&1; then
        log "  ❌ dispatch не прошёл (акт $a_n) — пропускаю"
        printf '%s\t%s\t%s\t%s\t-\tdispatch_failed\t-\t-\t-\n' \
            "$a_n" "$a_title" "$a_stab" "$a_steps" >> "$SUMMARY"
        overall=1
        continue
    fi

    # Ищем run_id: свежайший run воркфлоу, созданный после before_iso.
    run_id=""
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        sleep "${DISPATCH_WAIT_SEC:-6}"
        run_id="$(gh run list --repo "$REPO_SLUG" --workflow "$WORKFLOW" \
                    --limit 5 --json databaseId,createdAt,headBranch \
                    --jq "[.[] | select(.createdAt > \"$before_iso\")] | .[0].databaseId // empty" \
                    2>/dev/null || echo '')"
        [ -n "$run_id" ] && break
    done
    if [ -z "$run_id" ]; then
        log "  ⚠️ run_id не определился — акт запущен, но статус не отследить"
        printf '%s\t%s\t%s\t%s\t-\tunknown\t-\t-\t-\n' \
            "$a_n" "$a_title" "$a_stab" "$a_steps" >> "$SUMMARY"
        overall=1
        continue
    fi
    url="https://github.com/$REPO_SLUG/actions/runs/$run_id"
    log "  run_id=$run_id  $url"

    # Ждём завершения. gh run watch иногда рвёт соединение на длинных
    # прогонах — поэтому простой polling с потолком MAX_WAIT.
    waited=0
    status="in_progress"
    conclusion=""
    while [ "$waited" -lt "$MAX_WAIT" ]; do
        sleep "$POLL_SEC"
        waited=$((waited + POLL_SEC))
        read -r status conclusion <<<"$(gh run view "$run_id" --repo "$REPO_SLUG" \
            --json status,conclusion --jq '"\(.status) \(.conclusion // "-")"' 2>/dev/null \
            || echo 'unknown -')"
        [ "$status" = "completed" ] && break
        log "  ... $((waited / 60)) мин, статус=$status"
    done
    if [ "$status" != "completed" ]; then
        conclusion="timeout_${MAX_WAIT}s"
        log "  ⚠️ акт $a_n не завершился за ${MAX_WAIT}s"
    fi
    log "  вердикт акта $a_n: $conclusion"
    # Акт last отработал до конца: его trap в харнессе вернул робота на боевые БД.
    [ "$a_phase" = "last" ] && [ "$status" = "completed" ] && PHASE_RELEASED=1

    # Лог акта в отчёт — по нему потом читают, ЧТО именно упало (и из него же — колонки recovered/act_recover).
    act_log="$REPORT/act${a_n}_${run_id}.log"
    gh run view "$run_id" --repo "$REPO_SLUG" --log > "$act_log" 2>/dev/null || true
    recovered="$(marathon_summary_value "$act_log" recovered)"
    act_recover="$(marathon_summary_value "$act_log" act_recover)"
    [ "$recovered" != "-" ] && [ "$recovered" != "0" ] \
        && log "  восстановлено харнессом шагов: $recovered (OK recovered_*, не чистый PASS)"
    [ "$act_recover" != "-" ] && log "  на старте акта харнесс чинил утечку прошлого акта: $act_recover"

    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$a_n" "$a_title" "$a_stab" "$a_steps" "$run_id" "$conclusion" "$url" "$recovered" "$act_recover" >> "$SUMMARY"

    [ "$conclusion" != "success" ] && overall=1

    # Акты 1-2 — фундамент: без wake-word и без профилей голосов остальное
    # утонет в no_accept и диагностической ценности не даст.
    #
    # Но красный красному рознь, и различать их обязательно. Харнесс сам
    # печатает причину отказа отдельным маркером:
    #   E2E_NO_REACTION / E2E_INFRA_FAIL — робот не отвечает или упал синтез.
    #     Дальше идти бессмысленно: следующие девять актов дадут девять
    #     страниц no_accept и ни одного факта.
    #   E2E_FEATURE_FAIL — робот ответил, но не выполнил ассерт. Это НАХОДКА,
    #     ради которой марафон и запускают, и она не мешает актам 2-10:
    #     wake-word жив, значит остальное диагностично.
    # Первый прогон акта 1 (run 34408526453) прошёл 9 шагов из 10 и упал
    # ровно на фиче — останавливать там марафон было бы ошибкой.
    if [ "$STOP_ON_FOUNDATION_FAIL" = "1" ] && [ "$conclusion" != "success" ] \
       && { [ "$a_n" = "1" ] || [ "$a_n" = "2" ]; }; then
        if marathon_has_marker "$act_log" 'E2E_NO_REACTION|E2E_INFRA_FAIL'; then
            log "❌ фундаментальный акт $a_n не отвечает (no_reaction/infra) — останавливаю марафон"
            log "   (STOP_ON_FOUNDATION_FAIL=0 чтобы продолжать вопреки)"
            break
        fi
        log "⚠️ акт $a_n красный по фиче (E2E_FEATURE_FAIL) — это находка, продолжаю марафон"
    fi
done <<< "$plan"

log "=== ИТОГ ==="
cat "$SUMMARY"
log "отчёт: $REPORT"
exit "$overall"
