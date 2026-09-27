#!/usr/bin/env bash
# Контракт «формат лога ноды ↔ паттерны e2e-харнесса».
#
# ЗАЧЕМ (живой прогон 35658231116, 22.09.2026).
# stt_node.py:923 печатает маркер приёма фразы так:
#     self.get_logger().info(f"✅ ПРИНЯТО ({source}): {text}")
# то есть в логе робота стоит `✅ ПРИНЯТО (respeaker): Робот, ...`.
# Тег источника добавлен в 6e016325f (wake-роутер, #2011). Четыре места в
# e2e-харнессе искали строку БЕЗ тега и не находили ничего:
#
#   1. e2e_voice_test.sh:parse_transcript — grep '✅ ПРИНЯТО:' →
#      "recognized" в transcript.json пуст на каждом прогоне, а в лог шага
#      шло «TRANSCRIPT[...]: STT не вернул фразу (нет '✅ ПРИНЯТО')» ДАЖЕ
#      там, где строкой выше харнесс отчитался «✅ ПОЛНЫЙ ЦИКЛ». Два
#      взаимно противоречащих утверждения в одном логе.
#   2. e2e_voice_test.sh, python-чекер acceptance — то же regex.
#   3. e2e_voice_wake_gate.sh:wake_gate_cleared_since — grep -F
#      "✅ ПРИНЯТО: Робот". На живом логе: 0 совпадений против 5 у
#      исправленного паттерна. wake-gate НИКОГДА не считался пройденным;
#      симптом не всплывал только потому, что сам preflight был мёртвым
#      кодом. После его починки это дало бы вечный SKIP всех шагов
#      expect="wake-gated" — тихо «зелёный» прогон, не проверяющий ничего.
#   4. e2e_timing.py — recognized в timing.json и step summary пуст.
#
# Ни один из четырёх случаев не ронял прогон. Именно поэтому дрейф жил
# месяцами: сломанный grep выглядит как «события не было».
#
# КАК ПРОВЕРЯЕМ. Строка-образец рендерится из f-string, взятого ИЗ САМОГО
# stt_node.py. Bash-паттерны проверяются НАСТОЯЩИМ grep/sed, а не переводом
# в другой движок регулярок: первая версия этого теста как раз и соврала на
# переводе `[^[:space:]]` в питоновский синтаксис, объявив рабочий паттерн
# сломанным. Питоновские паттерны проверяются питоновским re.
#
# Запуск: bash scripts/testing/test_e2e_log_marker_contract.sh
# Exit:   0 = PASS, 1 = паттерны разошлись с форматом логов.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NODE="$REPO_ROOT/src/rob_box_voice/rob_box_voice/stt_node.py"
HARNESS="$REPO_ROOT/.github/workflows/scripts/e2e_voice_test.sh"
WAKE="$REPO_ROOT/.github/workflows/scripts/e2e_voice_wake_gate.sh"
TIMING="$REPO_ROOT/.github/workflows/scripts/e2e_timing.py"

for f in "$NODE" "$HARNESS" "$WAKE" "$TIMING"; do
    [ -f "$f" ] || { printf '❌ отсутствует: %s\n' "$f"; exit 1; }
done

fails=0
ok()  { printf '  ✅ %s\n' "$1"; }
bad() { printf '  ❌ %s\n' "$1"; fails=$((fails + 1)); }

SAMPLE_TEXT="Робот, как меня зовут?"

# ---------------------------------------------------------------------------
# CHECK 1. Формат маркера берём из самой ноды и рендерим строку лога.
# ---------------------------------------------------------------------------
printf 'CHECK 1: формат маркера ПРИНЯТО читается из stt_node.py\n'
FMT="$(grep -oE 'f"✅ ПРИНЯТО[^"]*"' "$NODE" | head -1 | sed -E 's/^f"//; s/"$//')"
if [ -z "$FMT" ]; then
    bad "stt_node.py: не нашёл f-string с «✅ ПРИНЯТО». Маркер переименовали? Тогда правь и паттерны в e2e_voice_test.sh / e2e_voice_wake_gate.sh / e2e_timing.py — иначе e2e перестанет видеть приём фразы и НЕ упадёт."
    FMT='✅ ПРИНЯТО ({source}): {text}'
else
    ok "stt_node.py печатает: $FMT"
fi
RENDERED="$(printf '%s' "$FMT" | sed "s/{source}/respeaker/; s/{text}/$SAMPLE_TEXT/")"
LINE="[stt_node-6] [INFO] [1790027215.287338166] [stt_node]: $RENDERED"
LEGACY="[stt_node-6] [INFO] [1790027215.287] [stt_node]: ✅ ПРИНЯТО: $SAMPLE_TEXT"
printf '  строка-образец: %s\n' "$RENDERED"

# ---------------------------------------------------------------------------
# CHECK 2. parse_transcript: настоящий grep -oE + sed из харнесса.
#          Проверяем не «совпало», а «извлекло РОВНО текст фразы» — обрезанный
#          или съехавший захват так же врёт, только тише.
# ---------------------------------------------------------------------------
printf 'CHECK 2: parse_transcript извлекает фразу настоящим grep/sed\n'
GREP_PAT="$(grep -oE "grep -oE '✅ ПРИНЯТО[^']*'" "$HARNESS" | head -1 | sed -E "s/^grep -oE '//; s/'$//")"
SED_PAT="$(grep -oE "sed -E 's\^\\^✅ ПРИНЯТО[^']*'" "$HARNESS" | head -1 || true)"
if [ -z "$GREP_PAT" ]; then
    bad "e2e_voice_test.sh: не нашёл grep -oE с «✅ ПРИНЯТО» — фикс откатили?"
else
    printf '  паттерн: %s\n' "$GREP_PAT"
    for variant in "живой формат с тегом:$LINE" "старый формат без тега:$LEGACY"; do
        vname="${variant%%:*}"; vline="${variant#*:}"
        got="$(printf '%s\n' "$vline" | grep -oE "$GREP_PAT" | head -1 \
               | sed -E 's/^✅ ПРИНЯТО( \([^)]*\))?:[[:space:]]*//' | tr -d '\r')"
        if [ "$got" = "$SAMPLE_TEXT" ]; then
            ok "$vname — извлёк «$got»"
        elif [ -z "$got" ]; then
            bad "$vname — НЕ поймал строку ноды (паттерн: $GREP_PAT)"
        else
            bad "$vname — извлёк «$got», ожидалось «$SAMPLE_TEXT»: в transcript.json попадёт не та фраза"
        fi
    done
fi

# ---------------------------------------------------------------------------
# CHECK 3. wake-gate: grep -qE из самого файла, против живой строки.
# ---------------------------------------------------------------------------
printf 'CHECK 3: wake-gate видит ПРИНЯТО с wake-словом\n'
if grep -q 'grep -qF "✅ ПРИНЯТО:' "$WAKE"; then
    bad "e2e_voice_wake_gate.sh: вернулся grep -qF \"✅ ПРИНЯТО: ...\" — фиксированная строка без тега источника. На живом логе даёт 0 совпадений → wake-gate никогда не пройден → вечный SKIP wake-gated шагов (тихий «зелёный» прогон)."
fi
WAKE_PAT="$(grep -oE 'grep -qE "✅ ПРИНЯТО[^"]*Робот"' "$WAKE" | head -1 | sed -E 's/^grep -qE "//; s/"$//')"
if [ -z "$WAKE_PAT" ]; then
    bad "e2e_voice_wake_gate.sh: не нашёл grep -qE с «✅ ПРИНЯТО ... Робот»"
else
    printf '  паттерн: %s\n' "$WAKE_PAT"
    if printf '%s\n' "$LINE" | grep -qE "$WAKE_PAT"; then
        ok "живой формат с тегом — wake-gate засчитан"
    else
        bad "живой формат с тегом — wake-gate НЕ засчитан: все wake-gated шаги уйдут в SKIP"
    fi
    if printf '%s\n' "$LEGACY" | grep -qE "$WAKE_PAT"; then
        ok "старый формат без тега — тоже засчитан"
    else
        bad "старый формат без тега не читается: прогон против необновлённого робота ослепнет"
    fi
fi

# ---------------------------------------------------------------------------
# CHECK 4. Питоновские паттерны (acceptance-чекер в харнессе + e2e_timing.py).
# ---------------------------------------------------------------------------
printf 'CHECK 4: питоновские паттерны извлекают фразу\n'
python3 - "$HARNESS" "$TIMING" "$LINE" "$LEGACY" "$SAMPLE_TEXT" <<'PY' || fails=$((fails + 1))
import pathlib, re, sys

for stream in (sys.stdout, sys.stderr):
    try:
        stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

harness, timing, line, legacy, sample = sys.argv[1:6]
bad = 0
for label, path in (("e2e_voice_test.sh acceptance", harness), ("e2e_timing.py", timing)):
    src = pathlib.Path(path).read_text(encoding="utf-8")
    m = re.search(r're\.search\(r"(✅ ПРИНЯТО[^"]*)"', src)
    if not m:
        print(f"  ❌ {label}: не нашёл re.search с «✅ ПРИНЯТО»")
        bad += 1
        continue
    pattern = m.group(1)
    try:
        rx = re.compile(pattern)
    except re.error as exc:
        print(f"  ❌ {label}: паттерн не компилируется ({exc}): {pattern}")
        bad += 1
        continue
    for vname, vline in (("живой формат с тегом", line), ("старый формат без тега", legacy)):
        hit = rx.search(vline)
        if not hit or not hit.groups():
            print(f"  ❌ {label} / {vname}: не извлёк фразу (паттерн: {pattern})")
            bad += 1
            continue
        got = (hit.group(1) or "").strip()
        if got != sample:
            print(f"  ❌ {label} / {vname}: извлёк «{got}», ожидалось «{sample}»")
            bad += 1
            continue
        print(f"  ✅ {label} / {vname} — извлёк «{got}»")
sys.exit(1 if bad else 0)
PY

printf '\n'
if [ "$fails" -ne 0 ]; then
    printf 'e2e log marker contract: FAIL (%d)\n' "$fails"
    exit 1
fi
printf 'e2e log marker contract: PASS\n'
