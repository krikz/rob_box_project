#!/bin/bash
# ============================================================================
# test_runtime_for_retro.sh — ретро t_10b51b22 (overshoot-kill архит-карточек)
#
# Регресс-гард для расширения runtime_for() и max_retries_for() в
# agent-flow-triage.sh: ретро-карточки архитектора (title начинается с
# «ретро:» / body начинается с «ретро:») автоматически получают
# AGENT_FLOW_MAX_RUNTIME_LARGE (3600s) и 3 ретрая вместо 2 — иначе
# watchdog-overshoot (4*max_rt = 14400s при max_rt=3600 — 4ч!) успевает
# убить карточку до завершения (см. тик 2026-09-14 19:55 CEST, t_a5488e86).
#
# Кейсы (acceptance criteria):
#   T1. test_retro_in_title_large:
#       labels="agent:architect", title="ретро: foo" → 3600
#   T2. test_retro_in_body_large:
#       labels="agent:architect", body starts with "ретро:" → 3600
#   T3. test_retro_default_role_large:
#       labels="" (default = architect), title="ретро: foo" → 3600
#   T4. test_no_retro_default_runtime:
#       labels="agent:architect", title="обычная задача" → 1800
#   T5. test_retro_in_body_but_not_assignee_architect_still_default:
#       body="ретро: ..." но assignee=devops (роль определена по labels)
#       → 1800 (НЕ large: ретро-правило только для архитектора)
#   T6. test_p0_label_still_large:
#       regression: priority:P0 продолжает работать → 3600
#   T7. test_long_body_still_large:
#       regression: body > 2000 chars → 3600
#   T8. test_retro_in_title_retries_3:
#       labels="agent:architect", title="ретро: foo" → max_retries_for = 3
#   T9. test_no_retro_retries_2:
#       labels="agent:architect", title="обычная" → max_retries_for = 2
#
# Run:
#   bash scripts/agent_flow/tests/test_runtime_for_retro.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRIAGE_SH="${TRIAGE_SH:-$TEST_DIR/../agent-flow-triage.sh}"

# Source triage script in stub-mode: делаем так, чтобы main не запустился.
# Подход: переопределяем env-флаги, чтобы main-фаза сразу вышла с skip, плюс
# оборачиваем в подоболочку и `tail -1` нам не нужен — source просто определит
# функции. Проверяем: main вызывает flock + gh auth + issue list — без
# подготовки упадёт на первой же. Поэтому тест изолирует функции через `set`:
# переопределим env, потом переопределим main-функции, которые мешают.
# Проще: source с stdin-substitution через `</dev/null` и мокаем ВСЁ нужное.
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

[ -f "$TRIAGE_SH" ] || fail "triage script not found: $TRIAGE_SH"

# Извлекаем определения функций runtime_for и max_retries_for (если есть)
# из triage-скрипта. runtime_for определена ~строка 871. max_retries_for — там,
# куда мы её добавим. Подход: source'им нужный кусок через awk.
#
# Triage — большой файл (~2644 строки), main при source'е не запускается
# (все вызовы за `if`-guard'ами в main()). Однако ради перестраховки вынесем
# только нужные функции в standalone-источник.

extract_funcs() {
    # Извлекаем ТЕЛА runtime_for(), max_retries_for() и _is_retro_architect()
    # по-отдельности. Для каждой: нашли строку «^name()», читаем до первой
    # standalone-строки «}» (закрытие функции). Это надёжно, потому что
    # внутри функций heredoc/case может съесть конец строки, но «}» на
    # собственной строке — однозначный маркер конца.
    local out="" body
    for fn in runtime_for max_retries_for _is_retro_architect; do
        body="$(awk -v fn="$fn" '
            $0 ~ "^" fn "\\(\\)" { f=1 }
            f { print }
            f && /^\}/ { exit }
        ' "$TRIAGE_SH")"
        out+="$body"$'\n'
    done
    printf '%s' "$out"
}

FUNC_BODY="$(extract_funcs)"
[ -n "$FUNC_BODY" ] || fail "could not extract runtime_for / max_retries_for from $TRIAGE_SH"

# Подгружаем функции в текущий shell. Сначала задаём defaults, как в triage.sh.
AGENT_FLOW_MAX_RUNTIME="${AGENT_FLOW_MAX_RUNTIME:-1800}"
AGENT_FLOW_MAX_RUNTIME_LARGE="${AGENT_FLOW_MAX_RUNTIME_LARGE:-3600}"
AGENT_FLOW_LARGE_BODY_CHARS="${AGENT_FLOW_LARGE_BODY_CHARS:-2000}"
AGENT_FLOW_MAX_RETRIES="${AGENT_FLOW_MAX_RETRIES:-2}"
AGENT_FLOW_MAX_RETRIES_LARGE="${AGENT_FLOW_MAX_RETRIES_LARGE:-3}"

eval "$FUNC_BODY"

# Sanity: функции должны быть определены
type runtime_for >/dev/null 2>&1 || fail "runtime_for not loaded"
type max_retries_for >/dev/null 2>&1 || fail "max_retries_for not loaded"

# --- helpers ----------------------------------------------------------------
short_body() { printf 'обычное тело issue, %d chars\n' "${#1}" ; }

run() {
    local name="$1" labels="$2" title="$3" body="$4" want_rt="$5" want_retries="$6"
    local got_rt got_retries
    got_rt="$(runtime_for "$labels" "$body" "$title" 2>&1)" || fail "$name: runtime_for failed"
    got_retries="$(max_retries_for "$labels" "$body" "$title" 2>&1)" || fail "$name: max_retries_for failed"
    if [ "$got_rt" != "$want_rt" ]; then
        fail "$name: runtime_for labels='$labels' title='$title' → got '$got_rt', want '$want_rt'"
    fi
    if [ "$got_retries" != "$want_retries" ]; then
        fail "$name: max_retries_for labels='$labels' title='$title' → got '$got_retries', want '$want_retries'"
    fi
    pass "$name: rt=$got_rt retries=$got_retries"
}

# --- T1-T9 -------------------------------------------------------------------
T1_LABELS="agent:architect"
T1_TITLE="ретро: foo"
T1_BODY="обычное тело без меток"
run "T1_retro_in_title" "$T1_LABELS" "$T1_TITLE" "$T1_BODY" 3600 3 || exit 1

T2_LABELS="agent:architect"
T2_TITLE="обычное название"
T2_BODY="ретро: body starts with this
## Факты
- старые логи
"
run "T2_retro_in_body" "$T2_LABELS" "$T2_TITLE" "$T2_BODY" 3600 3 || exit 1

T3_LABELS=""
T3_TITLE="ретро: default-role"
T3_BODY="обычное тело"
run "T3_retro_default_role" "$T3_LABELS" "$T3_TITLE" "$T3_BODY" 3600 3 || exit 1

T4_LABELS="agent:architect"
T4_TITLE="обычная задача без ретро"
T4_BODY="обычное тело"
run "T4_no_retro_default" "$T4_LABELS" "$T4_TITLE" "$T4_BODY" 1800 2 || exit 1

T5_LABELS="agent:devops"
T5_TITLE="обычная задача"
T5_BODY="ретро: this body starts with retro but assignee=devops → should NOT be large"
run "T5_retro_devops_not_large" "$T5_LABELS" "$T5_TITLE" "$T5_BODY" 1800 2 || exit 1

T6_LABELS="priority:P0,agent:architect"
T6_TITLE="обычное название"
T6_BODY="обычное тело"
run "T6_p0_label_still_large" "$T6_LABELS" "$T6_TITLE" "$T6_BODY" 3600 3 || exit 1

T7_BODY="$(python3 -c 'print("." * 2500, end="")')"   # 2500 chars
run "T7_long_body_still_large" "agent:architect" "обычное название" "$T7_BODY" 3600 3 || exit 1

T8_LABELS="agent:architect"
T8_TITLE="ретро: счётчик ретраев"
T8_BODY="обычное тело"
run "T8_retro_retries_3" "$T8_LABELS" "$T8_TITLE" "$T8_BODY" 3600 3 || exit 1

T9_LABELS="agent:architect"
T9_TITLE="обычная задача"
T9_BODY="обычное тело"
run "T9_no_retro_retries_2" "$T9_LABELS" "$T9_TITLE" "$T9_BODY" 1800 2 || exit 1

echo
echo "All tests passed (T1-T9)."