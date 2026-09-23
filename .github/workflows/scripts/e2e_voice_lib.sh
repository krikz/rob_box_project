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

# map_tts_voice() — голос сценария → голос выбранного TTS-провайдера.
#
# Сценарии (.github/e2e/scenarios/*.json) называют голоса ПО-ЯНДЕКСОВСКИ
# ("anton"/"ermil"/"zahar"/"filipp") — это исторический контракт, менять
# его в 250+ шагах нельзя. Когда команда синтезируется не Yandex'ом
# (E2E_TTS_PROVIDER=silero|minimax, см. e2e_voice_test.sh), имя голоса надо
# перевести в каталог целевого провайдера, иначе провайдер молча возьмёт
# свой дефолт и ВСЕ шаги зазвучат одним голосом.
#
# Почему это важно именно для e2e: act2/act3 night-marathon проверяют
# диаризацию (speaker_tag A vs B) — там в одном сценарии живут все четыре
# яндексовских голоса, и они ОБЯЗАНЫ остаться четырьмя разными голосами
# после перевода.
#
# ИЗМЕРЕНО 22.09.2026 (до этого таблица утверждала «гарантированно
# различимые», не имея ни одного замера, — и это было неправдой).
# Методика и сырые данные: scripts/e2e/measure_tts_voice_distinctness.py,
# evidence/tts-voice-distinctness-2026-09-22/. «Различимы» = max-cos
# resemblyzer-эмбеддингов НИЖЕ порога опознания IDENTIFY_THRESHOLD=0.72
# (порог слияния профилей REGISTER_MATCH_THRESHOLD=0.75 — см.
# src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py). Для
# ориентира: один и тот же голос на двух разных фразах даёт 0.95-0.98.
#
#   minimax, СТАРАЯ таблица: anton/ermil = 0.739, ermil/filipp = 0.810 —
#     НЕ различимы. На живом роботе та же пара anton/ermil дала 0.846
#     (микрофонный канал поднимает косинус), и акт 2 склеил Сашу с Борисом
#     в один профиль — run 35667281570.
#   minimax, таблица ниже: худшая пара 0.684, anton/ermil = 0.527.
#   silero, СТАРАЯ таблица: zahar/filipp (baya/xenia) = 0.722 — ровно на
#     границе, baya/kseniya = 0.788. Ниже baya отдан alena, а zahar получил
#     kseniya: худшая пара 0.678 (anton/ermil = aidar/eugene).
#   yandex — НЕ ИЗМЕРЕН: ключ на роботе отвечает PERMISSION_DENIED (та же
#     архивная папка, из-за которой падает synth_yandex). Мерить, когда
#     ключ починят: measure_tts_voice_distinctness.py --provider yandex.
#
# Подкрутку питча/темпа как альтернативу смене голосов ПРОВЕРИЛИ И
# ОТКЛОНИЛИ. Фиксированный сдвиг питча за говорящим действительно разводит
# эмбеддинги (anton[+6] vs ermil: 0.70 → 0.42, intra-voice не страдает,
# 0.95+), но ломает распознавание: vosk на тех же файлах даёт WER 0.79 при
# pitch=-6 и 0.26 при pitch=+3 против 0.05-0.16 без сдвига. Менять голос
# дешевле, чем менять диагноз с «не различил дикторов» на «не расслышал».
#
# Гендер намеренно НЕ сохраняется: различимость важнее совпадения пола,
# робот всё равно слышит синтетику. У Silero так было с самого начала,
# теперь так же и у minimax (Борису достался женский голос).
#
# Каталоги-источники — src/rob_box_voice/rob_box_voice/tts_voice_registry.py
# (PROVIDER_VOICES). Голос, который уже native для провайдера, не трогаем:
# так можно задать --voice aidar / --voice Russian_CrazyQueen напрямую.
#
# Примеры:
#   map_tts_voice yandex  anton  → anton
#   map_tts_voice silero  anton  → aidar
#   map_tts_voice silero  zahar  → kseniya
#   map_tts_voice minimax ermil  → Russian_PessimisticGirl
#   map_tts_voice silero  aidar  → aidar      (уже native)
#   map_tts_voice silero  ""     → aidar      (дефолт провайдера)
map_tts_voice() {
    # $1=provider $2=voice. Печатает голос провайдера в stdout.
    local provider="$1" voice="$2"
    case "$provider" in
        yandex)
            # Yandex — исходный каталог сценариев, перевод не нужен.
            printf '%s' "${voice:-anton}"
            return 0
            ;;
        silero)
            case "$voice" in
                aidar|baya|kseniya|xenia|eugene) printf '%s' "$voice"; return 0 ;;
                anton|kostya|"")                 printf 'aidar' ;;
                ermil|madirus)                   printf 'eugene' ;;
                zahar|arina)                     printf 'kseniya' ;;
                filipp|jane)                     printf 'xenia' ;;
                alena)                           printf 'baya' ;;
                omazh|rush)                      printf 'xenia' ;;
                *)                               printf 'aidar' ;;
            esac
            return 0
            ;;
        minimax)
            case "$voice" in
                Russian_*|male-qn-qingse|female-shaonv) printf '%s' "$voice"; return 0 ;;
                anton|"")        printf 'Russian_ReliableMan' ;;
                ermil|madirus)   printf 'Russian_PessimisticGirl' ;;
                zahar|arina)     printf 'Russian_CrazyQueen' ;;
                filipp|kostya)   printf 'Russian_AttractiveGuy' ;;
                alena)           printf 'Russian_BrightHeroine' ;;
                jane)            printf 'Russian_AmbitiousWoman' ;;
                arina)           printf 'Russian_PessimisticGirl' ;;
                omazh|rush)      printf 'Russian_Bad-temperedBoy' ;;
                *)               printf 'Russian_ReliableMan' ;;
            esac
            return 0
            ;;
        *)
            printf '%s' "$voice"
            return 0
            ;;
    esac
}

# observe_step() — advisory health snapshot робота (issue #2313).
#
# Утеряна при разрешении конфликта в мёрже ace46a372 ("Merge branch
# 'restore/develop' into develop"): та же правка оставила ДВЕ копии
# map_tts_voice и не оставила ни одной observe_step. Харнесс звал её на
# e2e_voice_test.sh:1788 и каждый прогон падал на
# `line 1788: observe_step: command not found` (run 35658231116) —
# health_snapshot.json оставался пустым. Регресс закрыт тестом
# scripts/testing/test_e2e_lib_contract.sh (все функции, которые харнесс
# зовёт из либы, обязаны быть определены после source).
#
# Контракт (scripts/testing/test_e2e_health_probe.sh): всегда печатает
# валидный JSON с полями step/probe_rc/raw и ВСЕГДА возвращает 0 —
# проба advisory и не имеет права влиять на PASS/FAIL прогона.
observe_step() {
    local step="${1:-functional}" output rc=0
    output="$(${ROBOT_SSH:-true} "docker ps --format '{{.Names}}|{{.Status}}'" 2>&1; \
        ${ROBOT_SSH:-true} "timeout 8 ros2 topic hz /scan --window 3" 2>&1; \
        ${ROBOT_SSH:-true} "timeout 8 ros2 topic hz /odom --window 3" 2>&1)" || rc=$?
    python3 - "$step" "$rc" "$output" <<'PY2'
import json, sys
step, rc, raw = sys.argv[1], int(sys.argv[2]), sys.argv[3]
print(json.dumps({"step": step, "probe_rc": rc, "raw": raw[-12000:]}, ensure_ascii=False, indent=2))
PY2
    return 0
}

# --- issue #2809 — переспрос личности: pure-функции условного шага/node_params
#
# Обе группы функций ниже переиспользуют harness-контракт e2e_mode
# (ros2 param set, читаем ОБРАТНО, не верим exit-коду) и уже существующий
# канал robot_speech()/keyword_hit() (e2e_tool_match.py, issue #2764/#2779).
# Вынесены сюда (а не в главный e2e_voice_test.sh), потому что этот файл —
# единственный, который безопасно source'ить из unit-тестов (см. шапку файла:
# "никакого main flow, никакого чтения ENV").

# when_robot_asked_matches() — issue #2809: "спросил ли робот вопрос-гипотезу
# на ПРЕДЫДУЩЕМ шаге". $1=речь робота за окно предыдущего шага (уже
# ПРОФИЛЬТРОВАННАЯ через robot_speech() — вызывающий отвечает за канал, эта
# функция сама логов не читает), $2=grep -E паттерн из scenario.json
# (when_robot_asked). Контракт идентичен must_not_say/expected_keywords по
# регистронезависимости, но БЕЗ поддержки "|"-альтернации как отдельного
# разбора — сам паттерн ЯВЛЯЕТСЯ ERE (grep -E), значит "Саш.*это ты|это
# ты.*Саш" уже валидный regex сам по себе, доп. парсинг не нужен.
# Пустой паттерн намеренно считается "не совпало" (return 1) — пустая строка
# в grep -E совпадает с ЧЕМ УГОДНО (включая тишину), это был бы тихий баг,
# из-за которого шаг без when_robot_asked ошибочно попал бы в эту ветку.
when_robot_asked_matches() {  # $1=prev_robot_speech $2=pattern (ERE-подобный)
    local prev_speech="$1" pattern="$2"
    [ -z "$pattern" ] && return 1
    # НЕ `grep -qiE` — регистронезависимость bash-grep для кириллицы зависит
    # от LC_CTYPE хоста, и под C/C.UTF-8 (частый дефолт минимального образа,
    # напр. билд-машина katana) она НЕ РАБОТАЕТ: glibc C.UTF-8 декодирует
    # UTF-8 байты, но НЕ содержит таблиц регистра для не-ASCII — "САША" не
    # сворачивается к "саша". Живой замер на этой задаче (issue #2809):
    # `grep -qiE` пропустил "САША, ЭТО ТЫ?" против паттерна в нижнем
    # регистре под LC_CTYPE=C.UTF-8. Python re.IGNORECASE Unicode-корректен
    # независимо от локали ОС — тот же принцип, что уже используют
    # keyword_hit()/tool_invoked() в e2e_tool_match.py (issue #2764/#2779),
    # не полагаясь на локаль байтового текстового процессора.
    PREV_SPEECH="$prev_speech" WRA_PATTERN="$pattern" python3 -c '
import os, re, sys
pattern = os.environ.get("WRA_PATTERN", "")
text = os.environ.get("PREV_SPEECH", "")
try:
    hit = bool(re.search(pattern, text, re.IGNORECASE))
except re.error:
    hit = False
sys.exit(0 if hit else 1)
'
}

# _ros2_param_get_raw() — сырой вывод ``ros2 param get <node> <param>``.
# Требует функцию robot_ros() в вызывающем окружении (главный скрипт её
# определяет; unit-тест подменяет своим стабом ДО вызова).
_ros2_param_get_raw() {  # $1=node $2=param
    robot_ros "ros2 param get '$1' '$2' --no-daemon" 2>/dev/null
}

# _ros2_param_value() — вытащить значение из вывода ``ros2 param get``,
# независимо от типа (Boolean/Integer/Double/String value is: ...).
_ros2_param_value() {  # $1=raw ros2-param-get output
    printf '%s' "$1" | grep -aoE '(Boolean|Integer|Double|String) value is: .*' | tail -1 \
        | sed -E 's/^(Boolean|Integer|Double|String) value is: //'
}

# _node_param_values_match() — сравнение значения ПОСЛЕ set с запрошенным.
# Строковое равенство ловит bool/str; числовое — доп. попытка через awk,
# потому что ``ros2 param get`` может вернуть "0.99" на запрос "0.990000"
# (или наоборот) — не разные значения, разное форматирование.
_node_param_values_match() {  # $1=actual $2=requested
    local a="$1" b="$2"
    [ "$a" = "$b" ] && return 0
    awk -v a="$a" -v b="$b" 'BEGIN{ exit !(a+0 == b+0) }' 2>/dev/null
}

# apply_node_params() — читает top-level ``node_params`` сценария, применяет
# КАЖДЫЙ параметр, проверяет ЧТЕНИЕМ (не exit-кодом), копит исходные значения
# в $E2E_NODE_PARAM_ORIGINALS_FILE для restore_node_params(). Пустой/
# отсутствующий ``node_params`` — no-op. Требует в окружении: robot_ros(),
# log(), E2E_NODE_PARAM_ORIGINALS_FILE (путь к файлу для restore).
apply_node_params() {  # $1=scenario_file
    local scenario_file="$1"
    local node_params_tsv
    node_params_tsv="$(python3 - "$scenario_file" <<'NPPY'
import json, sys
sc = json.load(open(sys.argv[1], encoding="utf-8"))
for node, params in (sc.get("node_params", {}) or {}).items():
    for k, v in (params or {}).items():
        print("%s\t%s\t%s" % (node, k, v))
NPPY
)"
    [ -z "$node_params_tsv" ] && return 0

    local node param value
    while IFS=$'\t' read -r node param value; do
        [ -z "$node" ] && continue
        local before_raw before_value after_raw after_value
        before_raw="$(_ros2_param_get_raw "$node" "$param")"
        before_value="$(_ros2_param_value "$before_raw")"
        if [ -z "$before_value" ]; then
            echo "E2E_FATAL: node_params — не удалось прочитать исходное значение ${node} ${param} (без него нечего восстанавливать после акта)" >&2
            echo "           проверь вручную: ssh <robot> \"docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 param get ${node} ${param} --no-daemon'\"" >&2
            exit 2
        fi
        robot_ros "ros2 param set '${node}' '${param}' '${value}' --no-daemon" >/dev/null 2>&1
        after_raw="$(_ros2_param_get_raw "$node" "$param")"
        after_value="$(_ros2_param_value "$after_raw")"
        if ! _node_param_values_match "$after_value" "$value"; then
            echo "E2E_FATAL: node_params — ${node} ${param} не применился: сценарий просил '${value}', узел вернул '${after_value}' (parameters_callback отклонил значение либо не перехватывает этот параметр — см. issue #2809)" >&2
            exit 2
        fi
        printf '%s\t%s\t%s\n' "$node" "$param" "$before_value" >> "$E2E_NODE_PARAM_ORIGINALS_FILE"
        log "🧪 node_params: ${node} ${param} = ${value} (было ${before_value}) — применено и подтверждено чтением"
    done <<< "$node_params_tsv"
}

# restore_node_params() — вызывается ТОЛЬКО из trap EXIT, всегда, независимо
# от PASS/FAIL/обрыва. Восстанавливает КАЖДЫЙ параметр к значению, прочитанному
# в apply_node_params ДО изменения (не к хардкоду в этом файле). Провал
# восстановления — НЕ фатал (акт уже закончился), но обязан быть громким в
# логе прогона (ADR-0018 — молчать о сбое нельзя).
restore_node_params() {
    [ -s "$E2E_NODE_PARAM_ORIGINALS_FILE" ] || return 0
    local node param value after_raw after_value
    while IFS=$'\t' read -r node param value; do
        [ -z "$node" ] && continue
        robot_ros "ros2 param set '${node}' '${param}' '${value}' --no-daemon" >/dev/null 2>&1
        after_raw="$(_ros2_param_get_raw "$node" "$param")"
        after_value="$(_ros2_param_value "$after_raw")"
        if _node_param_values_match "$after_value" "$value"; then
            log "🧪 node_params: ${node} ${param} восстановлен к исходному ${value}"
        else
            log "❌ ВНИМАНИЕ: node_params — ${node} ${param} НЕ восстановлен (просили ${value}, узел вернул ${after_value:-<пусто>}) — робот остался на форсированном значении!"
            log "   почини вручную: ssh <robot> \"docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 param set ${node} ${param} ${value} --no-daemon'\""
        fi
    done < "$E2E_NODE_PARAM_ORIGINALS_FILE"
}
