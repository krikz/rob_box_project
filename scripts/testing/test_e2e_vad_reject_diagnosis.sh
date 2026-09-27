#!/usr/bin/env bash
# Unit-тест: шаг, который не доехал до STT, обязан называть настоящую причину.
#
# РЕГРЕСС, который ловим (живой прогон 35658231116, 22.09.2026).
# Шаги n303_bg_boris / n304_bg_grisha_unknown отчитались так:
#
#   >>> STEP n303_bg_boris: ❌ backlog accumulation не подтверждён после 2 попыток
#   E2E_STEP n303_bg_boris FAIL backlog_miss
#
# то есть обвинили фичу бэклога в dialogue_node. В docker logs робота за то
# же окно лежало:
#
#   [audio_node] ❌ Речь отклонена: 17.37с (min=0.3, max=15.0)
#
# audio_node выбросил фразу ПО ДЛИНЕ — до STT, до wake-gate, до бэклога.
# Реплики этих шагов в синтезе minimax звучат 16.09-17.37с против
# speech_max_duration=15.0. Соседний n305 прошёл на 14.81с, то есть сценарий
# был лотереей с зазором 0.19с. Диагноз «backlog_miss» отправлял разбор в
# dialogue_node, где регресса нет.
#
# ADR-0018 «честный FAIL лучше красивого PASS»: FAIL, указывающий не на ту
# подсистему, — это тот же обман, только автоматический.
#
# Тест офлайновый: ни ssh, ни робота, ни билд-машины. ROBOT_SSH подменяется
# заглушкой, которая печатает фикстуру вместо docker logs.
#
# Запуск: bash scripts/testing/test_e2e_vad_reject_diagnosis.sh
# Exit:   0 = PASS, 1 = есть проваленные проверки.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HARNESS="$REPO_ROOT/.github/workflows/scripts/e2e_voice_test.sh"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

fails=0
ok()   { printf '  ✅ %s\n' "$1"; }
bad()  { printf '  ❌ %s\n' "$1"; fails=$((fails + 1)); }

[ -f "$HARNESS" ] || { printf '❌ нет харнесса: %s\n' "$HARNESS"; exit 1; }

# --- вырезаем тестируемые функции из харнесса -------------------------------
# Харнесс целиком source'ить нельзя — у него есть main flow (запись, ssh,
# paplay). Берём ровно две функции из его РЕАЛЬНОГО текста, чтобы тест не
# разошёлся с продакшеном.
awk '/^vad_reject_reason\(\)/,/^}/' "$HARNESS" > "$TMP/funcs.sh"
awk '/^emit_step_fail_or_vad\(\)/,/^}/' "$HARNESS" >> "$TMP/funcs.sh"
for fn in vad_reject_reason emit_step_fail_or_vad; do
    grep -q "^${fn}()" "$TMP/funcs.sh" \
        || { printf '❌ не нашёл %s() в харнессе — фикс откатили?\n' "$fn"; exit 1; }
done

# --- окружение-заглушка -----------------------------------------------------
OUT_DIR="$TMP/out"; mkdir -p "$OUT_DIR"
FIXTURE="$TMP/logs.txt"
EMITTED="$TMP/emitted.txt"
E2E_FAIL_KIND=""
: > "$EMITTED"

# ROBOT_SSH зовётся как `${ROBOT_SSH} "docker logs ..."` — заглушка молча
# игнорирует аргумент и печатает фикстуру.
fake_ssh() { cat "$FIXTURE"; }
ROBOT_SSH=fake_ssh

log()        { printf '%s\n' "$*" >> "$TMP/log.txt"; }
emit_step()  { printf '%s\n' "$1" >> "$EMITTED"; }
# Настоящий mark_fail_kind — чтобы проверить и приоритет kind'ов.
eval "$(awk '/^mark_fail_kind\(\)/,/^}/' "$HARNESS")"

# shellcheck source=/dev/null
source "$TMP/funcs.sh"

reset_case() { : > "$EMITTED"; : > "$TMP/log.txt"; E2E_FAIL_KIND=""; rm -f "$OUT_DIR/vad_rejects.log"; }

# ===========================================================================
# CASE 1. Длинная фраза (точная строка из прогона 35658231116, n303_bg_boris).
#         Ожидаем vad_rejected + kind=infra, а НЕ backlog_miss/feature.
# ===========================================================================
printf 'CASE 1: фраза длиннее speech_max_duration → vad_rejected, не backlog_miss\n'
reset_case
cat > "$FIXTURE" <<'EOF'
[audio_node-1] [INFO] [1790026828.374] [audio_node]: 🎙️  VAD: речь
[audio_node-1] [INFO] [1790026841.268] [audio_node]: 🎙️  VAD: тишина
[audio_node-1] [WARN] [1790026844.276] [audio_node]: ❌ Речь отклонена: 17.37с (min=0.3, max=15.0)
EOF
emit_step_fail_or_vad n303_bg_boris "2026-09-21T21:40:00Z" "FAIL backlog_miss" feature
if grep -q 'n303_bg_boris FAIL vad_rejected' "$EMITTED"; then
    ok "маркер шага: FAIL vad_rejected"
else
    bad "ожидал 'FAIL vad_rejected', получил: $(cat "$EMITTED")"
fi
if grep -q 'backlog_miss' "$EMITTED"; then
    bad "маркер всё ещё обвиняет бэклог (backlog_miss) — регресс вернулся"
else
    ok "backlog_miss больше не публикуется, когда звук не доехал"
fi
if [ "$E2E_FAIL_KIND" = "infra" ]; then
    ok "fail_kind=infra (не feature — робот-логика не исполнялась)"
else
    bad "fail_kind='$E2E_FAIL_KIND', ожидал 'infra'"
fi
if grep -q '17.37с' "$TMP/log.txt"; then
    ok "в логе шага есть измеренная длительность и лимит"
else
    bad "в логе шага нет измеренной длительности — диагноз не доказуем"
fi
if grep -q 'НЕ регресс робота' "$TMP/log.txt"; then
    ok "в логе прямо сказано, что это не регресс робота"
else
    bad "нет явной пометки «это не регресс робота»"
fi
if [ -s "$OUT_DIR/vad_rejects.log" ]; then
    ok "причина записана в артефакт vad_rejects.log"
else
    bad "vad_rejects.log пуст — ретро-инженер не увидит причину в артефактах"
fi

# ===========================================================================
# CASE 2. Никакого VAD-отказа не было → сохраняем ИСХОДНЫЙ диагноз.
#         Гарантия, что фикс не превращает любой фейл в «инфру» (это был бы
#         обратный обман — красивый «не наша вина» вместо настоящего регресса).
# ===========================================================================
printf 'CASE 2: без VAD-отказа → исходный диагноз сохраняется\n'
reset_case
cat > "$FIXTURE" <<'EOF'
[audio_node-1] [INFO] [1790027210.276] [audio_node]: ✅ Речь распознана: 14.81с
[stt_node-6] [INFO] [1790027215.287] [stt_node]: ✅ ПРИНЯТО (respeaker): Робот, как меня зовут?
EOF
emit_step_fail_or_vad n306_who_was_talking "2026-09-21T21:40:00Z" "FAIL backlog_miss" feature
if grep -q 'n306_who_was_talking FAIL backlog_miss' "$EMITTED"; then
    ok "исходный маркер backlog_miss сохранён"
else
    bad "исходный маркер потерян: $(cat "$EMITTED")"
fi
if [ "$E2E_FAIL_KIND" = "feature" ]; then
    ok "fail_kind=feature (настоящий регресс не замаскирован под инфру)"
else
    bad "fail_kind='$E2E_FAIL_KIND', ожидал 'feature'"
fi

# ===========================================================================
# CASE 3. no_accept + VAD-отказ → тоже vad_rejected (вторая точка вердикта).
# ===========================================================================
printf 'CASE 3: no_accept при VAD-отказе → vad_rejected\n'
reset_case
printf '[audio_node-1] [WARN] ❌ Речь отклонена: 16.09с (min=0.3, max=15.0)\n' > "$FIXTURE"
emit_step_fail_or_vad n304_bg_grisha "2026-09-21T21:40:00Z" "FAIL no_accept" no_reaction
if grep -q 'FAIL vad_rejected' "$EMITTED"; then
    ok "no_accept-путь тоже отдаёт vad_rejected"
else
    bad "no_accept-путь не перехвачен: $(cat "$EMITTED")"
fi

# ===========================================================================
# CASE 4. Мёртвый захват микрофона (ретро 17.09: «простоял ночь — не слышит»).
#         `Речь отклонена: 0.00с` — пустой буфер, лечится restart'ом, и это
#         тоже не регресс кода.
# ===========================================================================
printf 'CASE 4: пустой буфер (0.00с) → vad_rejected с подсказкой про restart\n'
reset_case
printf '[audio_node-1] [WARN] ❌ Речь отклонена: 0.00с (min=0.3, max=15.0)\n' > "$FIXTURE"
reason="$(vad_reject_reason "2026-09-21T21:40:00Z")"; rc=$?
if [ "$rc" = "0" ] && printf '%s' "$reason" | grep -q 'docker restart voice-assistant'; then
    ok "распознан мёртвый захват, в причине есть лечение"
else
    bad "не распознан пустой буфер (rc=$rc, reason='$reason')"
fi

# ===========================================================================
# CASE 5. ReSpeaker не найден на старте.
# ===========================================================================
printf 'CASE 5: ReSpeaker не найден → vad_rejected (инфра)\n'
reset_case
printf '[audio_node-1] [ERROR] ❌ ReSpeaker аудио устройство не найдено!\n' > "$FIXTURE"
if vad_reject_reason "2026-09-21T21:40:00Z" | grep -q 'ReSpeaker'; then
    ok "распознано отсутствие ReSpeaker"
else
    bad "не распознано отсутствие ReSpeaker"
fi

# ===========================================================================
# CASE 6. Чистый лог без отказов → vad_reject_reason возвращает 1 и молчит.
# ===========================================================================
printf 'CASE 6: чистый лог → проба молчит и возвращает 1\n'
reset_case
printf '[audio_node-1] [INFO] ✅ Речь распознана: 9.95с\n' > "$FIXTURE"
out="$(vad_reject_reason "2026-09-21T21:40:00Z")"; rc=$?
if [ "$rc" != "0" ] && [ -z "$out" ]; then
    ok "ложных срабатываний нет"
else
    bad "ложное срабатывание на чистом логе (rc=$rc, out='$out')"
fi

printf '\n'
if [ "$fails" -ne 0 ]; then
    printf 'vad reject diagnosis: FAIL (%d проверок провалено)\n' "$fails"
    exit 1
fi
printf 'vad reject diagnosis: PASS\n'
