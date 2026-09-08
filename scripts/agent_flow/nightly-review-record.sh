#!/bin/bash
# ============================================================================
# nightly-review-record.sh — персистентность находок ночного/компонентного
# ревью (ADR-0079, follow-up ADR-0049, issue #2159).
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/nightly-review-record.sh
# Раскладка — как у остальных agent_flow скриптов, через install.sh.
#
# ПРОБЛЕМА (найдено на ревью 08.09, до мержа #2177): agent-flow-nightly-
# review.sh (`no_agent`, cron) создаёт kanban-карточку ДО того, как кто-либо
# посмотрел на код — он физически не может знать, найдёт ли ревьюер дефект.
# Первая версия ADR-0079 пыталась решить это, читая переменную
# NIGHTLY_REVIEW_OUTCOME в том же прогоне, который создаёт карточку — мёртвый
# код: в проде эту переменную некому было выставить ДО запуска ревьюера
# (тесты гоняли скрипт, вручную подставляя её через env, и от этого выглядели
# зелёными, ничего не проверяя про реальный cron-путь).
#
# РЕШЕНИЕ: тот же паттерн, что ADR-0077 использует для обычных worker-
# отчётов — не механический pre-dispatch скрипт пишет находки, а сам
# ревьюер, своим последним шагом, перед `kanban_complete`. Он (и только он)
# знает, что нашёл. Этот скрипт даёт стабильный формат хранения (append-only
# JSONL, коммитится в git worktree воркера — переживает архивацию карточки)
# и бонус: fingerprint-дедуп находок между ночами (калька с GitHub SARIF
# `partialFingerprints`, см. docs/adr/0079-nightly-review-persistence.md).
#
# ЧТО ДЕЛАЕТ:
#   1. Валидирует --outcome (open-issue-<N> | no-real-defect |
#      duplicate-suppressed:<fp>) и --finding (JSON, обязателен при outcome
#      != no-real-defect).
#   2. Считает fingerprint = sha1(type:file:line:symbol)[:12] для каждой
#      находки, если он не передан явно.
#   3. Сканирует <reports-dir>/nightly-review/*.jsonl за LOOKBACK_DAYS на
#      совпадение fingerprint с записью outcome=open-issue-* — если есть,
#      печатает WARNING в stderr (fail-open: решение открывать новый issue
#      или сослаться на существующий — за воркером, не за скриптом).
#   4. Дописывает ОДНУ строку JSON в
#      <reports-dir>/nightly-review/<review-date>.jsonl.
#
# ЧЕГО НЕ ДЕЛАЕТ:
#   - НЕ коммитит и НЕ пушит файл — как kanban-report-write.sh (ADR-0077),
#     это работа воркера (`git add`+`commit`+`push` в СВОЮ ветку).
#   - НЕ создаёт и НЕ трогает kanban-карточки или GitHub issues. Открывать
#     issue на подтверждённую находку — отдельный шаг воркера (`gh issue
#     create`), этот скрипт только персистит факт, что находка была.
#   - НЕ блокирует ничего при дубле — WARNING, не exit != 0. Решение
#     остаётся за воркером/Шифу, как везде в этом репо (fail-open).
#
# Использование:
#   nightly-review-record.sh --task-id t_xxx --component <slug> \
#       --outcome open-issue-123 \
#       --finding '{"type":"dead-code","severity":"medium","file":"a.py","line":42,"symbol":"foo","raw":"..."}' \
#       [--files-changed a.py,b.py] [--review-date YYYY-MM-DD] [--reports-dir DIR]
#
# Формат JSONL-строки:
#   {ts, review_date, iso_week, task_id, component, files_changed[],
#    findings[{type, severity, file, line, symbol, fingerprint, raw}], outcome}
#
# ENV:
#   NIGHTLY_REVIEW_REPORTS_DIR   — корень отчётов (default docs/reports;
#                                  файл ложится в <root>/nightly-review/)
#   NIGHTLY_REVIEW_LOOKBACK_DAYS — окно дедуп-сканирования (default 30)
#
# Exit codes: 0 = ok (даже при WARNING-дубле), 2 = usage/validation error.
# ============================================================================
set -euo pipefail

TASK_ID=""
COMPONENT=""
OUTCOME=""
FINDINGS=()
FILES_CHANGED=""
REVIEW_DATE="$(date -u +%F)"
REPORTS_DIR="${NIGHTLY_REVIEW_REPORTS_DIR:-docs/reports}"
LOOKBACK_DAYS="${NIGHTLY_REVIEW_LOOKBACK_DAYS:-30}"

usage() {
    cat <<'EOF'
usage: nightly-review-record.sh --task-id T --component C --outcome OUTCOME
       [--finding JSON]... [--files-changed f1,f2,...]
       [--review-date YYYY-MM-DD] [--reports-dir DIR]

  --task-id T        kanban task id этой ревью-карточки (t_xxxxxxxx)
  --component C      slug компонента ('nightly' для карточки-дайджеста,
                      иначе тот же slug, что в title карточки)
  --outcome OUTCOME  open-issue-<N> | no-real-defect | duplicate-suppressed:<fp>
  --finding JSON     JSON-объект находки (повторяемый флаг), поля:
                      type, severity, file, line, symbol, raw.
                      Обязателен хотя бы один, если outcome != no-real-defect.
  --files-changed    список файлов через запятую (опционально)
  --review-date      YYYY-MM-DD (default: сегодня UTC)
  --reports-dir DIR  корень отчётов (default docs/reports, или
                      NIGHTLY_REVIEW_REPORTS_DIR)
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --task-id)       TASK_ID="${2:?}"; shift 2 ;;
        --component)     COMPONENT="${2:?}"; shift 2 ;;
        --outcome)       OUTCOME="${2:?}"; shift 2 ;;
        --finding)       FINDINGS+=("${2:?}"); shift 2 ;;
        --files-changed) FILES_CHANGED="${2:?}"; shift 2 ;;
        --review-date)   REVIEW_DATE="${2:?}"; shift 2 ;;
        --reports-dir)   REPORTS_DIR="${2:?}"; shift 2 ;;
        -h|--help)       usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

[ -n "$TASK_ID" ]   || { echo "ERROR: --task-id required" >&2; exit 2; }
[ -n "$COMPONENT" ] || { echo "ERROR: --component required" >&2; exit 2; }
[ -n "$OUTCOME" ]   || { echo "ERROR: --outcome required" >&2; exit 2; }

case "$OUTCOME" in
    open-issue-*|no-real-defect|duplicate-suppressed:*) ;;
    *)
        echo "ERROR: --outcome must be open-issue-<N> | no-real-defect | duplicate-suppressed:<fp>, got: $OUTCOME" >&2
        exit 2
        ;;
esac

case "$OUTCOME" in
    open-issue-*|duplicate-suppressed:*)
        [ "${#FINDINGS[@]}" -gt 0 ] || {
            echo "ERROR: --outcome=$OUTCOME requires at least one --finding" >&2
            exit 2
        }
        ;;
esac

command -v python3 >/dev/null 2>&1 || { echo "ERROR: python3 not on PATH" >&2; exit 2; }

ISO_WEEK="$(date -d "$REVIEW_DATE" +%G-W%V 2>/dev/null || date +%G-W%V)"
NOW_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
JSONL_DIR="${REPORTS_DIR}/nightly-review"
JSONL_FILE="${JSONL_DIR}/${REVIEW_DATE}.jsonl"
mkdir -p "$JSONL_DIR"

# --- собрать findings[], добить fingerprint = sha1(type:file:line:symbol)[:12] ---
FINDINGS_JSON="[]"
if [ "${#FINDINGS[@]}" -gt 0 ]; then
    FINDINGS_JSON="$(printf '%s\n' "${FINDINGS[@]}" | python3 -c '
import hashlib, json, sys

out = []
for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    try:
        f = json.loads(line)
    except Exception as e:
        sys.stderr.write("ERROR: --finding не JSON: %s (%s)\n" % (line[:120], e))
        sys.exit(2)
    if not f.get("fingerprint"):
        basis = "%s:%s:%s:%s" % (f.get("type", ""), f.get("file", ""), f.get("line", ""), f.get("symbol", ""))
        f["fingerprint"] = hashlib.sha1(basis.encode("utf-8")).hexdigest()[:12]
    out.append(f)
print(json.dumps(out, ensure_ascii=False))
')"
fi

# --- files_changed[] ---------------------------------------------------------
FILES_JSON="[]"
if [ -n "$FILES_CHANGED" ]; then
    FILES_JSON="$(printf '%s' "$FILES_CHANGED" | python3 -c '
import json, sys
print(json.dumps([f for f in sys.stdin.read().strip().split(",") if f], ensure_ascii=False))
')"
fi

# --- dedup lookup: та же fingerprint уже трекается открытым issue? ----------
# Окно — mtime файла *.jsonl (дёшево, достаточно для warning, не для гейта).
if [ "$FINDINGS_JSON" != "[]" ] && [ -d "$JSONL_DIR" ]; then
    printf '%s' "$FINDINGS_JSON" | JSONL_DIR="$JSONL_DIR" LOOKBACK_DAYS="$LOOKBACK_DAYS" python3 -c '
import glob, json, os, sys, time

new_findings = json.load(sys.stdin)
new_fps = {f["fingerprint"] for f in new_findings if f.get("fingerprint")}
if not new_fps:
    sys.exit(0)

cutoff = time.time() - int(os.environ["LOOKBACK_DAYS"]) * 86400
jsonl_dir = os.environ["JSONL_DIR"]
for path in glob.glob(os.path.join(jsonl_dir, "*.jsonl")):
    try:
        if os.path.getmtime(path) < cutoff:
            continue
    except OSError:
        continue
    try:
        with open(path, encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except Exception:
                    continue
                if not str(rec.get("outcome", "")).startswith("open-issue-"):
                    continue
                for f in rec.get("findings", []):
                    fp = f.get("fingerprint")
                    if fp in new_fps:
                        sys.stderr.write(
                            "WARNING: находка fingerprint=%s уже трекается (%s, %s:%s) — "
                            "%s: не заводи новый issue, сошлись на существующий\n"
                            % (fp, os.path.basename(path), f.get("file", "?"), f.get("line", "?"), rec.get("outcome"))
                        )
    except OSError:
        continue
'
fi

# --- собрать и дописать строку -----------------------------------------------
LINE="$(NOW_UTC="$NOW_UTC" REVIEW_DATE="$REVIEW_DATE" ISO_WEEK="$ISO_WEEK" \
    TASK_ID="$TASK_ID" COMPONENT="$COMPONENT" OUTCOME="$OUTCOME" \
    FINDINGS_JSON="$FINDINGS_JSON" FILES_JSON="$FILES_JSON" python3 -c '
import json, os
rec = {
    "ts": os.environ["NOW_UTC"],
    "review_date": os.environ["REVIEW_DATE"],
    "iso_week": os.environ["ISO_WEEK"],
    "task_id": os.environ["TASK_ID"],
    "component": os.environ["COMPONENT"],
    "files_changed": json.loads(os.environ["FILES_JSON"]),
    "findings": json.loads(os.environ["FINDINGS_JSON"]),
    "outcome": os.environ["OUTCOME"],
}
print(json.dumps(rec, ensure_ascii=False))
')"

printf '%s\n' "$LINE" >> "$JSONL_FILE"
echo "RECORDED $JSONL_FILE (task_id=$TASK_ID outcome=$OUTCOME findings=${#FINDINGS[@]})"
