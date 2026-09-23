#!/usr/bin/env bash
# Unit-тест для apply_node_params()/restore_node_params() (issue #2809,
# E2E-харнесс "переспрос личности", follow-up PR #2818).
#
# Зачем: акт night_marathon_act2b_identity_question форсирует
# name_confidence_band_high=0.99 на время прогона через top-level поле
# сценария node_params — иначе синтетические голоса (0.87-0.96) почти
# никогда не попадают в зону сомнения, и переспрос не тестируется.
# Контракт (тот же принцип, что e2e_mode/#2750): значение НЕ ВЕРИТСЯ на
# слово exit-коду `ros2 param set` — читается ОБРАТНО; восстановление в
# конце идёт К ИСХОДНОМУ значению, прочитанному ДО изменения (не к
# хардкоду). Три сценария, которые обязаны быть покрыты:
#   1. apply нормально ставит + проверяет + запоминает оригинал;
#   2. apply ловит "узел вернул не то, что просили" -> E2E_FATAL (exit 2);
#   3. restore возвращает ИМЕННО то значение, что было ДО apply (не 0.80
#      хардкодом — если бы оригинал уже был другим, напр. 0.75).
#
# Тест офлайновый: source'ит e2e_voice_lib.sh (единственный файл, безопасный
# для source вне главного скрипта — см. его шапку) и подсовывает СВОЙ
# robot_ros()/log(), эмулирующий состояние параметра узла в переменной
# оболочки (файловый "реестр параметров" — тот же приём, что реальный
# robot_ros делает через ssh+ros2, только без сети).
#
# Запуск: bash scripts/testing/test_e2e_node_params.sh
# Exit:   0 = PASS, N>0 = N failed assertions.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LIB="$REPO_ROOT/.github/workflows/scripts/e2e_voice_lib.sh"

if [ -t 1 ]; then
    RED=$'\033[0;31m'; GRN=$'\033[0;32m'; NC=$'\033[0m'
else
    RED=""; GRN=""; NC=""
fi

[ -f "$LIB" ] || { printf '❌ отсутствует: %s\n' "$LIB"; exit 1; }

# shellcheck disable=SC1090
source "$LIB" 2>/dev/null

for fn in apply_node_params restore_node_params _ros2_param_get_raw _ros2_param_value _node_param_values_match; do
    if ! type "$fn" >/dev/null 2>&1; then
        printf '%sFATAL: %s не определена после source %s%s\n' "$RED" "$fn" "$LIB" "$NC"
        exit 2
    fi
done

PASS=0
FAIL=0
ok()  { printf '%sOK%s   %s\n' "$GRN" "$NC" "$1"; PASS=$((PASS + 1)); }
bad() { printf '%sFAIL%s %s\n' "$RED" "$NC" "$1"; FAIL=$((FAIL + 1)); }

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

# --- фейковый "реестр параметров" узла --------------------------------------
# Файл WORKDIR/registry.tsv: node\tparam\tvalue. robot_ros() — единственная
# точка входа, которую реальный харнесс подменяет через ROBOT_SSH_OVERRIDE;
# здесь эмулируем на уровень выше (сама robot_ros — не ssh, а функция),
# потому что apply_node_params/restore_node_params вызывают именно её.
REGISTRY="$WORKDIR/registry.tsv"
: > "$REGISTRY"

_registry_get() {  # $1=node $2=param
    awk -F'\t' -v n="$1" -v p="$2" '$1==n && $2==p {print $3; found=1} END{if(!found) exit 1}' "$REGISTRY"
}
_registry_set() {  # $1=node $2=param $3=value
    local tmp="$WORKDIR/registry.tmp"
    awk -F'\t' -v n="$1" -v p="$2" -v v="$3" '
        BEGIN{OFS="\t"; done=0}
        $1==n && $2==p {print n,p,v; done=1; next}
        {print}
        END{if(!done) print n,p,v}
    ' "$REGISTRY" > "$tmp"
    mv "$tmp" "$REGISTRY"
}

# robot_ros() — стаб вместо реального ssh+docker-exec+ros2. Разбирает те же
# команды, что реальный apply_node_params/restore_node_params шлют:
#   ros2 param get '<node>' '<param>' --no-daemon
#   ros2 param set '<node>' '<param>' '<value>' --no-daemon
robot_ros() {
    local cmd="$1"
    case "$cmd" in
        "ros2 param get "*)
            local rest node param val
            rest="${cmd#ros2 param get \'}"
            node="${rest%%\'*}"
            rest="${rest#*\' \'}"
            param="${rest%%\'*}"
            if val="$(_registry_get "$node" "$param")"; then
                printf "  Double value is: %s\n" "$val"
            else
                printf "Parameter not set.\n"
                return 1
            fi
            ;;
        "ros2 param set "*)
            local rest node param val
            rest="${cmd#ros2 param set \'}"
            node="${rest%%\'*}"
            rest="${rest#*\' \'}"
            param="${rest%%\'*}"
            rest="${rest#*\' \'}"
            val="${rest%%\'*}"
            if [ "$REGISTRY_REJECT_SET" = "1" ]; then
                # Симулирует parameters_callback, который отклонил значение
                # (issue #2809: узел НЕ перехватывает этот параметр, либо
                # валидация не прошла) — регистр остаётся со старым значением.
                return 0
            fi
            _registry_set "$node" "$param" "$val"
            ;;
        *)
            return 1
            ;;
    esac
}

log() { :; }  # тихий stub — не засоряет вывод теста

# =============================================================================
# CASE 1: apply нормально применяет + запоминает оригинал; restore возвращает
#         ИМЕННО прочитанный оригинал (не хардкод 0.80 из calibration).
# =============================================================================
: > "$REGISTRY"
_registry_set "/speaker_id_node" "name_confidence_band_high" "0.73"
REGISTRY_REJECT_SET=0
E2E_NODE_PARAM_ORIGINALS_FILE="$WORKDIR/originals_case1.tsv"
: > "$E2E_NODE_PARAM_ORIGINALS_FILE"

SCENARIO1="$WORKDIR/scenario1.json"
cat > "$SCENARIO1" <<'JSON'
{"node_params": {"/speaker_id_node": {"name_confidence_band_high": 0.99}}}
JSON

if apply_node_params "$SCENARIO1"; then
    if [ "$(_registry_get "/speaker_id_node" "name_confidence_band_high")" = "0.99" ]; then
        ok "apply_node_params применил запрошенное значение (0.99)"
    else
        bad "apply_node_params НЕ применил значение (реестр: $(_registry_get "/speaker_id_node" "name_confidence_band_high" || echo '<нет>'))"
    fi
    if grep -qE $'^/speaker_id_node\tname_confidence_band_high\t0\\.73$' "$E2E_NODE_PARAM_ORIGINALS_FILE"; then
        ok "apply_node_params запомнил ИСХОДНОЕ значение 0.73 (не дефолт 0.80)"
    else
        bad "apply_node_params не запомнил оригинал корректно: $(cat "$E2E_NODE_PARAM_ORIGINALS_FILE")"
    fi
else
    bad "apply_node_params неожиданно завершился с ошибкой на валидном сценарии"
fi

restore_node_params
if [ "$(_registry_get "/speaker_id_node" "name_confidence_band_high")" = "0.73" ]; then
    ok "restore_node_params вернул ИСХОДНОЕ значение 0.73 (не хардкод 0.80)"
else
    bad "restore_node_params не восстановил оригинал: реестр=$(_registry_get "/speaker_id_node" "name_confidence_band_high" || echo '<нет>')"
fi

# =============================================================================
# CASE 2: пустой/отсутствующий node_params — no-op, не трогает реестр.
# =============================================================================
: > "$REGISTRY"
_registry_set "/speaker_id_node" "name_confidence_band_high" "0.80"
E2E_NODE_PARAM_ORIGINALS_FILE="$WORKDIR/originals_case2.tsv"
: > "$E2E_NODE_PARAM_ORIGINALS_FILE"

SCENARIO2="$WORKDIR/scenario2.json"
cat > "$SCENARIO2" <<'JSON'
{"steps": [{"label": "s1", "text": "Робот, привет"}]}
JSON

apply_node_params "$SCENARIO2"
if [ "$(_registry_get "/speaker_id_node" "name_confidence_band_high")" = "0.80" ] \
    && [ ! -s "$E2E_NODE_PARAM_ORIGINALS_FILE" ]; then
    ok "сценарий без node_params — no-op, реестр и originals-файл не тронуты"
else
    bad "сценарий без node_params неожиданно что-то поменял"
fi
restore_node_params  # тоже должен быть no-op (originals-файл пуст)
if [ "$(_registry_get "/speaker_id_node" "name_confidence_band_high")" = "0.80" ]; then
    ok "restore_node_params на пустом originals-файле — no-op"
else
    bad "restore_node_params на пустом originals-файле что-то испортил"
fi

# =============================================================================
# CASE 3: узел ОТКЛОНИЛ значение (parameters_callback вернул successful=False,
# либо не перехватывает этот параметр вообще) — apply_node_params обязан
# ЧЕСТНО провалиться (E2E_FATAL, exit 2), а не поверить exit-коду ros2cli.
# =============================================================================
: > "$REGISTRY"
_registry_set "/speaker_id_node" "some_uncaptured_param" "0.80"
REGISTRY_REJECT_SET=1
E2E_NODE_PARAM_ORIGINALS_FILE="$WORKDIR/originals_case3.tsv"
: > "$E2E_NODE_PARAM_ORIGINALS_FILE"

SCENARIO3="$WORKDIR/scenario3.json"
cat > "$SCENARIO3" <<'JSON'
{"node_params": {"/speaker_id_node": {"some_uncaptured_param": 0.99}}}
JSON

# apply_node_params вызывает `exit 2` при провале верификации — запускаем в
# subshell, чтобы не убить сам тестовый раннер.
( apply_node_params "$SCENARIO3" ) 2> "$WORKDIR/case3_stderr.txt"
case3_rc=$?
REGISTRY_REJECT_SET=0

if [ "$case3_rc" = "2" ]; then
    ok "apply_node_params падает с exit 2, когда узел отклонил значение"
else
    bad "apply_node_params должен был упасть с exit 2 на отклонённом значении, получили rc=$case3_rc"
fi
if grep -q "E2E_FATAL" "$WORKDIR/case3_stderr.txt"; then
    ok "провал верификации даёт понятный E2E_FATAL, а не тихий exit"
else
    bad "нет E2E_FATAL в выводе провала: $(cat "$WORKDIR/case3_stderr.txt")"
fi
if [ "$(_registry_get "/speaker_id_node" "some_uncaptured_param")" = "0.80" ]; then
    ok "реестр остался на исходном значении после провала (0.80, не 0.99)"
else
    bad "реестр изменился, хотя apply_node_params должен был провалиться"
fi

printf '\n%d passed, %d failed\n' "$PASS" "$FAIL"
[ "$FAIL" -eq 0 ]
