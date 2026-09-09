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
#   STOP_ON_FOUNDATION_FAIL (default 1) — прервать марафон, если акт 1 или 2
#              красный: без wake-word и без профилей голосов остальные акты
#              не диагностичны, они просто утонут в no_accept.
#
# ПЕРЕД ПРОГОНОМ (иначе акт 8 не диагностичен):
#   ssh <robot> 'ros2 param set /dialogue_node barge_in_policy classify'
# =============================================================================
set -u

REPO_SLUG="${REPO_SLUG:-krikz/rob_box_project}"
REF="${REF:-develop}"
WORKFLOW="${WORKFLOW:-L-E2E Voice Test.yml}"
POLL_SEC="${POLL_SEC:-60}"
MAX_WAIT="${MAX_WAIT:-3600}"
DRY_RUN="${DRY_RUN:-0}"
STOP_ON_FOUNDATION_FAIL="${STOP_ON_FOUNDATION_FAIL:-1}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
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
printf 'act\ttitle\tstability\tsteps\trun_id\tconclusion\turl\n' > "$SUMMARY"

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
plan="$("$PY" - "$MANIFEST" "$ACTS_FILTER" <<'PYEOF'
import json, sys
# Названия актов кириллические; на Windows-хосте stdout по умолчанию cp1252
# и print() падает с UnicodeEncodeError ещё до первого диспатча.
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass
manifest, flt = sys.argv[1], sys.argv[2]
acts = json.load(open(manifest, encoding="utf-8"))["acts"]
if flt.strip():
    want = {int(x) for x in flt.replace(" ", "").split(",") if x}
    acts = [a for a in acts if a["act"] in want]
for a in acts:
    print("\t".join([
        str(a["act"]), a["title"], a["stability"], str(a["steps"]),
        a["scenario_file"], a["acceptance_file"],
    ]))
PYEOF
)"

if [ -z "$plan" ]; then
    echo "FATAL: ни один акт не выбран (ACTS=$ACTS_FILTER)" >&2
    exit 2
fi

n_acts="$(printf '%s\n' "$plan" | wc -l | tr -d ' ')"
log "REF=$REF  актов=$n_acts  отчёт=$REPORT"
printf '%s\n' "$plan" | while IFS=$'\t' read -r a_n a_title a_stab a_steps a_scn a_acc; do
    log "  акт $a_n  «$a_title»  ($a_steps шагов, $a_stab)"
done

if [ "$DRY_RUN" = "1" ]; then
    log "DRY_RUN=1 — ничего не запускаю"
    exit 0
fi

overall=0
while IFS=$'\t' read -r a_n a_title a_stab a_steps a_scn a_acc; do
    [ -z "$a_n" ] && continue
    log "=== АКТ $a_n «$a_title» ($a_steps шагов, $a_stab) ==="

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
            >/dev/null 2>&1; then
        log "  ❌ dispatch не прошёл (акт $a_n) — пропускаю"
        printf '%s\t%s\t%s\t%s\t-\tdispatch_failed\t-\n' \
            "$a_n" "$a_title" "$a_stab" "$a_steps" >> "$SUMMARY"
        overall=1
        continue
    fi

    # Ищем run_id: свежайший run воркфлоу, созданный после before_iso.
    run_id=""
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        sleep 6
        run_id="$(gh run list --repo "$REPO_SLUG" --workflow "$WORKFLOW" \
                    --limit 5 --json databaseId,createdAt,headBranch \
                    --jq "[.[] | select(.createdAt > \"$before_iso\")] | .[0].databaseId // empty" \
                    2>/dev/null || echo '')"
        [ -n "$run_id" ] && break
    done
    if [ -z "$run_id" ]; then
        log "  ⚠️ run_id не определился — акт запущен, но статус не отследить"
        printf '%s\t%s\t%s\t%s\t-\tunknown\t-\n' \
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

    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$a_n" "$a_title" "$a_stab" "$a_steps" "$run_id" "$conclusion" "$url" >> "$SUMMARY"

    # Лог акта в отчёт — по нему потом читают, ЧТО именно упало.
    gh run view "$run_id" --repo "$REPO_SLUG" --log > "$REPORT/act${a_n}_${run_id}.log" 2>/dev/null || true

    [ "$conclusion" != "success" ] && overall=1

    # Акты 1-2 — фундамент: без wake-word и без профилей голосов остальное
    # утонет в no_accept и диагностической ценности не даст.
    if [ "$STOP_ON_FOUNDATION_FAIL" = "1" ] && [ "$conclusion" != "success" ] \
       && { [ "$a_n" = "1" ] || [ "$a_n" = "2" ]; }; then
        log "❌ фундаментальный акт $a_n красный — останавливаю марафон"
        log "   (STOP_ON_FOUNDATION_FAIL=0 чтобы продолжать вопреки)"
        break
    fi
done <<< "$plan"

log "=== ИТОГ ==="
cat "$SUMMARY"
log "отчёт: $REPORT"
exit "$overall"
