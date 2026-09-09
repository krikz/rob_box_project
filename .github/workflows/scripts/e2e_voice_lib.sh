#!/usr/bin/env bash
# BUG-B (t_f0612a43) — общие helper'ы для e2e voice-скриптов.
#
# Этот файл source'ится и из основного harness (.github/workflows/scripts/
# e2e_voice_test.sh), и из unit-тестов (scripts/testing/test_e2e_voice_*).
# НЕ выполняет ничего, кроме определения функций — никакого main flow,
# никакого чтения ENV.
#
# Добавляй сюда только pure-функции (без side-effects).

# safe_label() — транслитерация + ASCII slug для использования в именах файлов.
# Использование: имя wav файла строится из ${label}, и если label содержит
# кириллицу/запятые/знаки вопроса — имя становится невалидным для shell
# (pathname expansion ломает '?', heredoc-Python synth_yandex путает аргументы).
# До фикса (см. git log BUG-B t_f0612a43) synth_yandex падал с YANDEX_EMPTY
# / permission denied и шаг помечался FAIL synth без реального запуска.
#
# Примеры:
#   safe_label "ml01_generate_romantic"             → "ml01_generate_romantic"
#   safe_label "Робот, что в библиотеке?"           → "robot_chto_v_biblioteke"
#   safe_label "Робот, сохрани этот трек для Ивана." → "robot_sokhrani_etot_trek_dlya_ivana"
#   safe_label "  ml/03:search-rain?  "              → "ml_03_search_rain"
safe_label() {
    # $1=label. Печатает ASCII-slug в stdout.
    python3 - "$1" <<'PY'
import sys, re
label = sys.argv[1]
# Транслитерация (все буквы → lowercase, чтобы slug был каноническим).
# Строчные: а→a, б→b, ... Заглавные: А→a, Б→b, ... (тот же lowercase выход).
TRANS = {
    "а":"a","б":"b","в":"v","г":"g","д":"d","е":"e","ё":"yo","ж":"zh","з":"z",
    "и":"i","й":"y","к":"k","л":"l","м":"m","н":"n","о":"o","п":"p","р":"r",
    "с":"s","т":"t","у":"u","ф":"f","х":"kh","ц":"ts","ч":"ch","ш":"sh","щ":"shch",
    "ъ":"" , "ы":"y","ь":"","э":"e","ю":"yu","я":"ya",
    "А":"a","Б":"b","В":"v","Г":"g","Д":"d","Е":"e","Ё":"yo","Ж":"zh","З":"z",
    "И":"i","Й":"y","К":"k","Л":"l","М":"m","Н":"n","О":"o","П":"p","Р":"r",
    "С":"s","Т":"t","У":"u","Ф":"f","Х":"kh","Ц":"ts","Ч":"ch","Ш":"sh","Щ":"shch",
    "Ъ":"" , "Ы":"y","Ь":"","Э":"e","Ю":"yu","Я":"ya",
    " ": "_", ",": "_", ".": "", "?": "", "!": "", ";": "", ":": "",
    "«": "", "»": "", "—": "-", "–": "-", "/": "_", "\\": "_",
    "(": "", ")": "", "[": "", "]": "", "{": "", "}": "",
    '"': "", "'": "", "`": "", "&": "and", "@": "at", "#": "", "%": "", "*": "",
}
slug = "".join(TRANS.get(c, c) for c in label).lower()
slug = re.sub(r"[^a-z0-9_\-]", "_", slug)
slug = re.sub(r"_+", "_", slug).strip("_")
slug = slug[:80] or "step"
print(slug)
PY
}

# observe_step() — advisory robot-health snapshot; never affects PASS/FAIL.
observe_step() {
    local step="${1:-functional}" output rc=0
    output="$(STEP_NAME="$step" ROBOT_SSH="${ROBOT_SSH:-}" bash -c '
        set +e
        printf "{\\\"step\\\":\\\"%s\\\",\\\"checked_at\\\":\\\"%s\\\",\\\"docker_ps\\\":\\\"" "$STEP_NAME" "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
        if [ -n "$ROBOT_SSH" ]; then $ROBOT_SSH "docker ps --format '\''{{.Names}}|{{.Status}}'\''" 2>&1; else printf "ROBOT_SSH_UNSET"; fi
        printf "\\\",\\\"scan_hz\\\":\\\""
        if [ -n "$ROBOT_SSH" ]; then $ROBOT_SSH "timeout 8 ros2 topic hz /scan --window 3" 2>&1; else printf "ROBOT_SSH_UNSET"; fi
        printf "\\\",\\\"odom_hz\\\":\\\""
        if [ -n "$ROBOT_SSH" ]; then $ROBOT_SSH "timeout 8 ros2 topic hz /odom --window 3" 2>&1; else printf "ROBOT_SSH_UNSET"; fi
        printf "\\\"}\\n"
    ' 2>&1)" || rc=$?
    python3 - "$step" "$rc" "$output" <<'PY2'
import json, sys
step, rc, raw = sys.argv[1], int(sys.argv[2]), sys.argv[3]
try: snapshot = json.loads(raw)
except json.JSONDecodeError: snapshot = {"step": step, "probe_error": raw[-4000:], "probe_rc": rc}
print(json.dumps(snapshot, ensure_ascii=False, indent=2))
PY2
    return 0
}


# observe_step() — advisory robot-health snapshot; never affects PASS/FAIL.
observe_step() {
    local step="${1:-functional}" output rc=0
    output="$(printf '%s' "$ROBOT_SSH" | sed 's/[[:space:]]*$//' >/dev/null; \
        ${ROBOT_SSH:-true} "docker ps --format '{{.Names}}|{{.Status}}'" 2>&1; \
        ${ROBOT_SSH:-true} "timeout 8 ros2 topic hz /scan --window 3" 2>&1; \
        ${ROBOT_SSH:-true} "timeout 8 ros2 topic hz /odom --window 3" 2>&1)" || rc=$?
    python3 - "$step" "$rc" "$output" <<'PY2'
import json, sys
step, rc, raw = sys.argv[1], int(sys.argv[2]), sys.argv[3]
print(json.dumps({"step": step, "probe_rc": rc, "raw": raw[-12000:]}, ensure_ascii=False, indent=2))
PY2
    return 0
}
