#!/usr/bin/env bash
# Контракт деплоя атомарного e2e-харнесса.
#
# ЗАЧЕМ. Харнесс .github/workflows/scripts/e2e_voice_test.sh запускается НЕ в
# чекауте репозитория: workflow "L: E2E Voice Test" копирует его одиночным
# файлом в /tmp на билд-машину 10.1.1.249 и дёргает через ssh, где CWD —
# домашний каталог ros2. Всё, что харнесс source'ит или зовёт по пути, обязано
# быть скопировано туда же. Каждый пропущенный файл даёт МОЛЧАЛИВУЮ деградацию:
# скрипт не падает целиком, а теряет ровно одну функцию или ровно один артефакт,
# и прогон краснеет с диагнозом, указывающим на робота вместо харнесса.
#
# Этот класс бага ловили уже шесть раз:
#   1. e2e_voice_lib.sh        — не копировался, source падал (t_cca7c074, 19.08)
#   2. e2e_voice_wake_gate.sh  — не копировался, classify_step_expect молча
#                                резолвилась в пустоту (issue #1735)
#   3. e2e_tool_match.py       — не копировался, acceptance-чекеры падали на
#                                ImportError (run 34408526453, 09.09)
#   4. observe_step()          — функция утеряна мёржем ace46a372 (две копии
#                                map_tts_voice вместо неё) → на каждом прогоне
#                                `line 1788: observe_step: command not found`,
#                                health_snapshot.json пустой (run 35658231116)
#   5. e2e_audio_metrics.py    — repo-relative путь + не копировался →
#   6. e2e_baseline_diff.py      audio_metrics.json ВСЕГДА был 36-байтной
#                                заглушкой {"error":"audio_metrics.py failed"}
#                                (run 35658231116, 22.09)
#
# Тест дешёвый и офлайновый: ни ssh, ни робота, ни билд-машины не требует.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SCRIPTS_DIR="$ROOT/.github/workflows/scripts"
HARNESS="$SCRIPTS_DIR/e2e_voice_test.sh"
LIB="$SCRIPTS_DIR/e2e_voice_lib.sh"
WAKE="$SCRIPTS_DIR/e2e_voice_wake_gate.sh"
WORKFLOW="$ROOT/.github/workflows/L-E2E Voice Test.yml"

fails=0
fail() { printf '  ❌ %s\n' "$1"; fails=$((fails + 1)); }
ok()   { printf '  ✅ %s\n' "$1"; }

for f in "$HARNESS" "$LIB" "$WAKE" "$WORKFLOW"; do
    [ -f "$f" ] || { printf '❌ отсутствует: %s\n' "$f"; exit 1; }
done

# ---------------------------------------------------------------------------
# CHECK 1. Каждый файл из $SCRIPT_DIR_E2E, который харнесс source'ит или зовёт,
#          должен копироваться на 249 scp-командой в workflow.
# ---------------------------------------------------------------------------
printf 'CHECK 1: файлы харнесса копируются на билд-машину\n'
referenced="$(grep -oE '\$SCRIPT_DIR_E2E/[A-Za-z0-9_.-]+' "$HARNESS" \
    | sed 's#^\$SCRIPT_DIR_E2E/##' | sort -u)"
[ -n "$referenced" ] || fail "в харнессе не нашлось ни одной ссылки \$SCRIPT_DIR_E2E/* — тест сломан?"

while IFS= read -r dep; do
    [ -n "$dep" ] || continue
    if [ ! -f "$SCRIPTS_DIR/$dep" ]; then
        fail "$dep: харнесс зовёт его, но файла нет в $SCRIPTS_DIR"
        continue
    fi
    if grep -q "scp .*scripts/$dep ros2@10\.1\.1\.249:/tmp/$dep" "$WORKFLOW"; then
        ok "$dep — копируется на 249"
    else
        fail "$dep: харнесс зовёт его через \$SCRIPT_DIR_E2E, но workflow НЕ копирует его на 249 (добавь scp в шаг «Push atomic harness»)"
    fi
done <<< "$referenced"

# ---------------------------------------------------------------------------
# CHECK 2. Ни один python-скрипт харнесса не зовётся repo-relative путём.
#          На 249 нет чекаута — такой путь не разрешается никогда.
# ---------------------------------------------------------------------------
printf 'CHECK 2: нет repo-relative вызовов внутри харнесса\n'
if bad="$(grep -nE 'python3?[[:space:]]+\.github/workflows/scripts/' "$HARNESS")"; then
    while IFS= read -r line; do
        fail "repo-relative вызов (на 249 не разрешится): $line"
    done <<< "$bad"
else
    ok "repo-relative вызовов нет"
fi

# ---------------------------------------------------------------------------
# CHECK 3. Каждая функция, которую харнесс зовёт из либ, реально определена
#          после source. Ловит «мёрж съел функцию» (observe_step, ace46a372).
# ---------------------------------------------------------------------------
printf 'CHECK 3: функции либ определены (регресс observe_step / ace46a372)\n'
# shellcheck source=/dev/null
source "$LIB"
# shellcheck source=/dev/null
source "$WAKE"
for fn in $(grep -hoE '^[a-z_][a-z0-9_]*\(\)' "$LIB" "$WAKE" | tr -d '()' | sort -u); do
    if grep -qE "(^|[^A-Za-z0-9_])${fn}[[:space:]]" "$HARNESS"; then
        if declare -F "$fn" >/dev/null 2>&1; then
            ok "$fn — определена и используется харнессом"
        else
            fail "$fn: харнесс её зовёт, но после source она не определена"
        fi
    fi
done
# Обратная проверка: функции, которые харнесс зовёт, но нигде не определены.
# Список вызовов берём по «голым» командам в начале строки — этого хватает,
# чтобы поймать observe_step, и не даёт ложных срабатываний на встроенных.
while IFS= read -r fn; do
    [ -n "$fn" ] || continue
    declare -F "$fn" >/dev/null 2>&1 && continue
    grep -qE "^[[:space:]]*${fn}\(\)" "$HARNESS" && continue
    command -v "$fn" >/dev/null 2>&1 && continue
    fail "$fn: зовётся в харнессе, но не определена ни в нём, ни в либах, и это не внешняя команда"
done <<< "$(grep -oE '^[[:space:]]*(observe_step|classify_step_expect|safe_label|map_tts_voice|run_wake_gate_preflight|wake_gate_cleared_since)[[:space:]]' "$HARNESS" | sed 's/[[:space:]]//g' | sort -u)"

# ---------------------------------------------------------------------------
# CHECK 4. Ни одна функция не определена в либе дважды. Дубль — это след
#          плохого разрешения конфликта (ace46a372) и мина для будущих правок:
#          правку внесут в одну копию, а победит текстово последняя.
# ---------------------------------------------------------------------------
printf 'CHECK 4: нет дублей определений функций в либах\n'
for f in "$LIB" "$WAKE"; do
    dupes="$(grep -oE '^[a-z_][a-z0-9_]*\(\)' "$f" | sort | uniq -d)"
    if [ -n "$dupes" ]; then
        while IFS= read -r d; do
            fail "$(basename "$f"): функция $d определена более одного раза"
        done <<< "$dupes"
    else
        ok "$(basename "$f") — дублей нет"
    fi
done

# ---------------------------------------------------------------------------
# CHECK 5. Склейка профилей дикторов не проходит как успешная регистрация.
#
# bug(живой прогон 35667281570, акт 2, 22.09.2026). Шаг n204_boris_intro_long
# получил OK: acceptance проверяет только факт вызова register_speaker. А в
# логах робота за то же окно:
#     user_input='[Spkr:Саша] ... Меня зовут Борис ...'
#     Speaker 'Борис' merged into existing profile (id=dc417cef)
#     [issue 1077] Speaker registered: 'Борис' id=dc417cef
# Голос Бориса опознан как Саша, Борис склеен в профиль Саши, профиль
# ПЕРЕИМЕНОВАН — Саша исчез. В /data/speakers.db после акта остался ОДИН
# диктор «Борис» с двумя эмбеддингами вместо двух дикторов. Текст шага при
# этом просит «Запомни мой голос ОТДЕЛЬНО от Сашиного, ... я не хочу, чтобы
# ты нас путал». Зелёный шаг поверх потерянной личности — ровно тот красивый
# PASS, против которого ADR-0018.
# ---------------------------------------------------------------------------
printf 'CHECK 5: склейка профиля диктора не считается успешной регистрацией\n'
if grep -q 'merged into existing profile' "$HARNESS"; then
    ok "харнесс ищет маркер склейки профиля"
    if grep -q 'speaker_merged' "$HARNESS"; then
        ok "склейка отражается в причине провала шага (speaker_merged)"
    else
        fail "склейка найдена, но не попадает в причину провала — шаг останется зелёным"
    fi
    if grep -q 'speaker_merges.log' "$HARNESS"; then
        ok "склейка пишется в артефакт speaker_merges.log"
    else
        fail "склейка не пишется в артефакты — ретро-инженер её не увидит"
    fi
    if grep -qF 'register_speaker*)' "$HARNESS"; then
        ok "проверка включается только для шагов, ожидающих register_speaker"
    else
        fail "проверка не привязана к register_speaker — сработает там, где не надо"
    fi
else
    fail "харнесс НЕ проверяет склейку профилей: шаг с register_speaker остаётся зелёным, даже когда профиль слился с чужим и был переименован (run 35667281570)"
fi

printf '\n'
if [ "$fails" -ne 0 ]; then
    printf 'e2e harness deploy contract: FAIL (%d проблем)\n' "$fails"
    exit 1
fi
printf 'e2e harness deploy contract: PASS\n'
