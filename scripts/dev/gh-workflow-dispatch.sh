#!/usr/bin/env bash
# gh-workflow-dispatch.sh — запуск workflow_dispatch через REST вместо `gh workflow run` (issue #4476).
#
# С 05.11 23:23 UTC `gh workflow run` отвечает HTTP 500 на любой workflow, а прямой
# POST .../actions/workflows/<id>/dispatches проходит (204). Тот же приём, что у
# gh-pr-create-via-gh-api.sh: REST, а не обёртка.
#
# Использование:
#   scripts/dev/gh-workflow-dispatch.sh <workflow файл или имя> <ref> [k=v ...]
#   scripts/dev/gh-workflow-dispatch.sh "L-E2E Voice Test.yml" develop environment=test scenario_file=x.json
#
# ENV:
#   REPO_SLUG          (default krikz/rob_box_project)
#   DISPATCH_RETRY_SEC (default 10) — пауза перед единственным повтором при 5xx
#
# Выход: 0 — диспатч принят (HTTP 204); иначе ≠0, в stderr — HTTP-код и тело ответа.
set -u

REPO_SLUG="${REPO_SLUG:-krikz/rob_box_project}"
DISPATCH_RETRY_SEC="${DISPATCH_RETRY_SEC:-10}"

if [ "$#" -lt 2 ]; then
    echo "usage: $0 <workflow file|name> <ref> [k=v ...]" >&2
    exit 64
fi
WF="$1"
REF="$2"
shift 2

INPUT_ARGS=()
for kv in "$@"; do
    case "$kv" in
        *=*) INPUT_ARGS+=(-f "inputs[${kv%%=*}]=${kv#*=}") ;;
        *) echo "gh-workflow-dispatch: input '$kv' не в форме k=v" >&2; exit 64 ;;
    esac
done

# id workflow: по имени файла (путь оканчивается на /<WF>) или по name; \r от Windows-gh срезаем.
errf="$(mktemp)"
wf_id="$(gh api "repos/$REPO_SLUG/actions/workflows" --paginate \
    --jq ".workflows[] | select((.path | endswith(\"/$WF\")) or .name == \"$WF\") | .id" 2>"$errf")"
lookup_rc=$?
wf_id="$(printf '%s' "$wf_id" | tr -d '\r' | head -1)"
if [ "$lookup_rc" -ne 0 ] || [ -z "$wf_id" ]; then
    echo "gh-workflow-dispatch: не нашёл workflow '$WF' в $REPO_SLUG (rc=$lookup_rc): $(tr -d '\r' < "$errf" 2>/dev/null)" >&2
    rm -f "$errf"
    exit 1
fi
rm -f "$errf"

attempt=1
while :; do
    # -i: первая строка ответа — «HTTP/2.0 204 No Content»; тело после пустой строки.
    resp="$(gh api -i "repos/$REPO_SLUG/actions/workflows/$wf_id/dispatches" --method POST \
        -f ref="$REF" ${INPUT_ARGS[@]+"${INPUT_ARGS[@]}"} 2>&1)"
    rc=$?
    resp="${resp//$'\r'/}"
    code="$(printf '%s\n' "$resp" | grep -m1 -oE '^HTTP/[0-9.]+ [0-9]{3}' | grep -oE '[0-9]{3}$')"
    # gh при ошибке может не печатать заголовки: тогда код берём из «(HTTP 500)».
    [ -z "$code" ] && code="$(printf '%s\n' "$resp" | grep -m1 -oE 'HTTP [0-9]{3}' | grep -oE '[0-9]{3}$')"
    if [ "$rc" -eq 0 ] && [ "${code:-204}" = "204" ]; then
        exit 0
    fi
    case "${code:-}" in
        5??)
            if [ "$attempt" -lt 2 ]; then
                echo "gh-workflow-dispatch: HTTP $code (workflow $WF, id=$wf_id), повтор через ${DISPATCH_RETRY_SEC}s" >&2
                sleep "$DISPATCH_RETRY_SEC"
                attempt=$((attempt + 1))
                continue
            fi
            ;;
    esac
    echo "gh-workflow-dispatch: диспатч '$WF' (id=$wf_id, ref=$REF) не прошёл: HTTP ${code:-?} (gh rc=$rc), попыток $attempt" >&2
    printf '%s\n' "$resp" >&2
    exit "$([ "$rc" -ne 0 ] && echo "$rc" || echo 1)"
done
