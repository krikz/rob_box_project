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

# --- issue #2809/#2824 — единая точка разбора scenario.json в TSV ---------
#
# ОБЕ стороны — главный scenario-цикл (e2e_voice_test.sh) И регресс-тест
# (scripts/testing/test_e2e_scenario_tsv_row.sh) — обязаны читать РОВНО
# ЭТОТ код, не копии. Issue #2824: регресс-тест #2823 проверял python-
# парсер и bash `while read` по отдельности (каждый своей копией), и не
# поймал баг, который жил ИМЕННО НА ГРАНИЦЕ между ними: python печатал
# корректный TSV, bash `read` его коверкал. Общий источник истины делает
# такой класс регрессов невозможным — тест ФИЗИЧЕСКИ не может разойтись
# с тем, что реально исполняется в прогоне.
#
# E2E_SCENARIO_ROW_READ — команда `read`, разбирающая одну строку TSV на
# именованные переменные. Вызывается через
# `IFS=$'\x1f' eval "$E2E_SCENARIO_ROW_READ"` (не напрямую `read ...` —
# eval нужен, чтобы этот файл оставался единственным местом, где
# перечислены имена полей; изменить список — значит поменять одну строку
# здесь, а не грепать все места, где он продублирован).
# issue #2902 — next_when_robot_asked: when_robot_asked СЛЕДУЮЩЕГО шага. Если
# речь проваленной попытки уже совпала с ним, робот задал вопрос, на который
# отвечает следующий шаг, и повтор этой реплики проверял бы другое — ретрай
# запрещается (retry_block_reason в e2e_tool_match.py). Поле ПОСЛЕДНЕЕ:
# разделитель \x1f сохраняет позиции, прежние поля не сдвигаются.
E2E_SCENARIO_ROW_READ='read -r idx label text voice patterns_json acceptance_json expect_raw retry_acceptance when_robot_asked required_question sleep_before_sec next_when_robot_asked barge_in_json next_barge_in_json silent_ok'

# parse_scenario_to_tsv() — читает scenario.json, пишет TSV-подобный файл
# (на самом деле \x1f-separated, не таб — см. ниже) с одной строкой на шаг,
# в порядке полей E2E_SCENARIO_ROW_READ. $1=scenario_file $2=out_file.
#
# issue #2824 (регресс от #2809, живой прогон 35851587044, develop
# b2f5560e5): разделитель \t НЕЛЬЗЯ использовать с bash `read` — \t
# относится к "IFS whitespace" НЕЗАВИСИМО от того, что реально записано в
# IFS. Даже `IFS=$'\t' read` схлопывает подряд идущие табы и стрижёт
# пустые поля по краям — так же, как обычный $IFS по умолчанию (пробел/
# таб/перевод строки). У шага БЕЗ when_robot_asked это поле пустое ('') —
# на выходе python оно просто исчезает из строки, все поля ПОСЛЕ него
# сдвигаются на одну позицию: в required_question приезжает то, что
# должно быть в sleep_before_sec, а сам when_robot_asked получает "0"
# (сдвинутое required_question). Непустой ("0") when_robot_asked не
# матчит речь предыдущего шага — ВСЕ обычные шаги без этого поля уходили
# в SKIP robot_did_not_ask (прогон 35851587044: steps=0/19, ни одна
# реплика не проиграна — сломан был КАЖДЫЙ сценарий, не только новый акт).
#
# \x1f (ASCII Unit Separator) — не whitespace для bash ни при каких
# обстоятельствах, пустые поля (в том числе подряд/по краям) сохраняются
# позиционно (проверено вручную: `printf 'a\x1f\x1f\x1fb\x1fc\n' |
# { IFS=$'\x1f' read -r a b c d e; ...}` даёт a=a b='' c='' d=b e=c —
# именно то поведение, которого ждали от \t и не получили). Значения
# JSON-полей (patterns_json/acceptance_json) экранированы json.dumps
# (control-байты <0x20 всегда \uXXXX), поэтому литеральный 0x1f туда
# никогда не просочится — разделитель не может столкнуться с данными.
parse_scenario_to_tsv() {  # $1=scenario_file $2=out_file
    local scenario_file="$1" out_file="$2"
    python3 - "$scenario_file" <<'PY' > "$out_file"
import json, sys
# Windows dev-машины: sys.stdout по умолчанию транслирует \n -> \r\n в
# текстовом режиме (Linux CI-раннер этого не делает). \r в последнем поле
# ломает сравнение строк в bash ("0.0" != "0.0\r") -- держим вывод
# бинарно-предсказуемым на обеих платформах.
try:
    sys.stdout.reconfigure(newline="\n")
except Exception:
    pass
FS = "\x1f"


def norm_barge(raw):
    """barge_in шага -> компактный JSON {after,n,delay_sec} или '' (поля нет). Неизвестный after
    остаётся как есть: run_step назовёт его ошибкой сценария, а не подменит молча."""
    if not isinstance(raw, dict):
        return ''
    try:
        n = max(1, int(raw.get('n', 1) or 1))
    except (TypeError, ValueError):
        n = 1
    try:
        delay = max(0.0, float(raw.get('delay_sec', 0) or 0))
    except (TypeError, ValueError):
        delay = 0.0
    after = str(raw.get('after') or 'speech_started')
    return json.dumps({'after': after, 'n': n, 'delay_sec': delay}, ensure_ascii=False, separators=(',', ':'))


sc = json.load(open(sys.argv[1], encoding="utf-8"))
steps = sc.get("steps", [])
for i, s in enumerate(steps):
    pats = s.get('patterns', [])
    acc = s.get('acceptance', {})
    exp = s.get('expect', 'cycle')
    retry = s.get('retry_acceptance', 0)
    try:
        retry = int(retry or 0)
    except (TypeError, ValueError):
        retry = 0
    # issue #2824 -- sanitize against the field separator itself (defense
    # in depth: json.dumps already escapes it, but when_robot_asked is a
    # raw scenario-author string, not JSON-encoded on this line).
    when_asked = str(s.get('when_robot_asked') or '').replace(FS, ' ').replace('\n', ' ')
    required_q = 1 if s.get('required_question') else 0
    try:
        sleep_before = float(s.get('sleep_before_sec', 0) or 0)
    except (TypeError, ValueError):
        sleep_before = 0.0
    nxt = steps[i + 1] if i + 1 < len(steps) and isinstance(steps[i + 1], dict) else {}
    next_asked = str(nxt.get('when_robot_asked') or '').replace(FS, ' ').replace('\n', ' ')
    # issue #3857 -- barge_in: намеренное перебивание (JSON after/n/delay_sec). next_barge_in_json --
    # то же у СЛЕДУЮЩЕГО шага: предыдущий шаг ждёт не конца ответа, а якоря перебивания.
    # silent_ok -- ход без речи допустим (Tier-1 new_session: llm_calls=0, речи нет).
    barge = norm_barge(s.get('barge_in'))
    next_barge = norm_barge(nxt.get('barge_in'))
    silent_ok = 1 if (s.get('silent_ok') or exp == 'silent_ok') else 0
    fields = [str(i), s.get('label', f's{i+1}'), s.get('text', ''), s.get('voice', 'anton'),
              json.dumps(pats), json.dumps(acc, ensure_ascii=False), exp, str(retry),
              when_asked, str(required_q), str(sleep_before), next_asked,
              barge, next_barge, str(silent_ok)]
    print(FS.join(fields))
PY
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
    # issue #2824 -- \x1f (ASCII Unit Separator), не \t: та же категория бага,
    # что в главном scenario-цикле (\t -- IFS-whitespace для bash `read`
    # независимо от значения IFS, схлопывает пустые/подряд идущие поля).
    # node/param/value здесь пустыми не бывают на практике, но разделитель
    # держим единообразным во ВСЕХ местах, где харнесс генерирует TSV и
    # читает его обратно через `read` -- именно так по этому классу багов
    # просят исправлять и остальные вызовы (issue #2824).
    node_params_tsv="$(python3 - "$scenario_file" <<'NPPY'
import json, sys
FS = "\x1f"
sc = json.load(open(sys.argv[1], encoding="utf-8"))
for node, params in (sc.get("node_params", {}) or {}).items():
    for k, v in (params or {}).items():
        print(FS.join([str(node), str(k), str(v)]))
NPPY
)"
    [ -z "$node_params_tsv" ] && return 0

    local node param value
    while IFS=$'\x1f' read -r node param value; do
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
        printf '%s\x1f%s\x1f%s\n' "$node" "$param" "$before_value" >> "$E2E_NODE_PARAM_ORIGINALS_FILE"
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
    while IFS=$'\x1f' read -r node param value; do
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

# --- issue #3248 — голос/провайдер TTS не должны переживать акт ----------------
# voice_core_suite_v1 mv01 «говори голосом Алены» → set_voice(alena, yandex):
# tts_node ставит Yandex первым в цепочку и ПЕРСИСТИТ это в
# /data/tts_provider_state.json, а VoiceStateStore в mcp_server помнит голос.
# После run 36775544782 робот так и остался на Yandex (ermil/anton) — следующий
# акт и живые люди слышали чужой голос; марафон 29-30.09 упал 11/12 на той же
# утечке. Вернуть голос фразой («верни голос») — снова LLM, тот же недетерминизм.
#
# Поэтому в trap EXIT: если за прогон РЕАЛЬНО звали set_voice / переключали
# провайдера (строки mcp_server «[set_voice] voice=» / tts_node
# «[issue 1765] set_provider: '»), харнесс сам, в обход LLM, вызывает тул
# set_voice(voice=<дефолт провайдера>, provider=<tts_node.provider из
# конфига>) — тем же путём, что LLM (/mcp/execute, подпись sender=harness,
# как scripts/music/live_check_mcp_call.py). Цель — КОНФИГУРАЦИОННЫЙ дефолт
# (param /tts_node provider + default_voice_for), а не «что было до акта»:
# голос, выбранный set_voice, живёт только в памяти mcp_server и снаружи не
# читается. Провал — не фатал (акт уже закончился), но громко (ADR-0018).
#
# Требует в окружении: robot_ros(), log(), ROBOT_SSH, E2E_RUN_BEFORE.

# Код, исполняемый ВНУТРИ контейнера voice-assistant. Передаётся base64-строкой
# (python3 -c "exec(b64decode(...))"), чтобы в командной строке ssh не было
# переводов строк и $'…'-квотинга. Печатает одну строку JSON.
#
# ОДНА реализация подписанного вызова /mcp/execute (sender=harness, как
# scripts/music/live_check_mcp_call.py): общая «шапка» _e2e_mcp_client_py + тело
# под задачу (_e2e_tts_restore_py: set_voice после акта; _e2e_music_stop_py:
# стоп музыки ПЕРЕД актом, issue #3871). Вторую подпись не писать.
_e2e_mcp_client_py() {
    cat <<'MCPPY'
import json, sys, time, uuid
from rob_box_mcp_tools.mcp_auth import RequestAuthenticator
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class McpClient:
    """Подписанные вызовы тулов через /mcp/execute -> /mcp/result (как у LLM)."""

    def __init__(self, node_name, rid_prefix):
        self.rid_prefix = rid_prefix
        self.auth = RequestAuthenticator.from_env(sender="harness")
        self.can_sign = bool(self.auth.can_sign)
        self.results = {}
        self.node = None
        if not self.can_sign:
            return
        rclpy.init()
        self.node = rclpy.create_node(node_name)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.node.create_subscription(String, "/mcp/result", self._on_result, qos)
        self.pub = self.node.create_publisher(String, "/mcp/execute", qos)
        self.spin_until(lambda: self.pub.get_subscription_count() > 0, 10.0)

    def _on_result(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(payload, dict) and payload.get("request_id"):
            self.results[payload["request_id"]] = payload

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return bool(predicate())

    def call(self, tool_name, parameters, timeout):
        """Ответ /mcp/result целиком либо None (таймаут)."""
        rid = self.rid_prefix + uuid.uuid4().hex[:12]
        request = {"tool_name": tool_name, "parameters": parameters, "request_id": rid}
        self.auth.sign(request)  # подпись прямо перед публикацией (окно clock_skew)
        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        self.pub.publish(msg)
        self.spin_until(lambda: rid in self.results, timeout)
        return self.results.get(rid)

    def close(self):
        if self.node is not None:
            self.node.destroy_node()
            rclpy.shutdown()
MCPPY
}

# argv[1] = провайдер. Ответ /mcp/result на set_voice либо {"error": ...}.
_e2e_tts_restore_py() {
    _e2e_mcp_client_py
    cat <<'TTSPY'
provider = sys.argv[1]
from rob_box_voice.tts_voice_registry import default_voice_for
voice = default_voice_for(provider)
if not voice:
    print(json.dumps({"error": "no default voice for provider %r" % provider}))
    sys.exit(3)
mcp = McpClient("e2e_tts_restore", "e2e-tts-restore-")
if not mcp.can_sign:
    print(json.dumps({"error": "no /mcp/execute secret to sign with"}))
    sys.exit(3)
payload = mcp.call("set_voice", {"voice": voice, "provider": provider}, 20.0)
mcp.close()
print(json.dumps(payload if payload is not None else {"error": "timeout: no /mcp/result for set_voice"}, ensure_ascii=False))
sys.exit(0 if payload is not None else 2)
TTSPY
}

# --- issue #3871 — музыка не должна переживать прогон ---------------------------
# Утечка состояния между актами/прогонами: замер акта 1 (run 39849541397, 22.10) шёл
# под DJ-сет, запущенный из Telegram за час до того (во всех ходах
# <music_state>{"playing":"yes",...,"dj":"on"}); харнесс перед актом делал только
# сброс сессии диалога. Снимок берём у владельца плеера — latched /voice/music/state
# (парсер и «играет» — rob_box_voice.core.music_player_state, не своя копия), стоп —
# теми же подписанными тулами, что у голосового «хватит» (dj_set stop + stop_music).
# Состояние читается маленьким рабочим процессом внутри voice-assistant (rclpy,
# TRANSIENT_LOCAL), а не `ros2 topic echo` в сервисном контейнере (OOM).
# argv[1] = сколько секунд ждать idle после стопа. Печатает одну строку JSON:
# {"verdict": "was_idle"|"stopped"|"stuck", "before": {...}, "after": {...}, "stop_calls": {...}}.
_e2e_music_stop_py() {
    _e2e_mcp_client_py
    cat <<'MUSICPY'
idle_timeout = float(sys.argv[1])
from rob_box_voice.core.music_player_state import MUSIC_STATE_TOPIC, parse_music_state
mcp = McpClient("e2e_music_precondition", "e2e-music-stop-")
if not mcp.can_sign:
    print(json.dumps({"error": "no /mcp/execute secret to sign with"}))
    sys.exit(3)
box = {}
def on_state(msg):
    snap = parse_music_state(msg.data)
    if snap is not None:
        box["snap"] = snap
state_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
mcp.node.create_subscription(String, MUSIC_STATE_TOPIC, on_state, state_qos)
def busy():
    snap = box.get("snap")
    return snap is not None and (snap.is_playing() or bool(snap.dj))
def describe():
    snap = box.get("snap")
    if snap is None:
        return {"snapshot": False}
    return {"snapshot": True, "state": snap.state, "playing": snap.is_playing(), "dj": bool(snap.dj), "track_id": snap.track_id}
mcp.spin_until(lambda: "snap" in box, 5.0)  # latched: у живого плеера снимок приходит сразу
report = {"before": describe()}
if not busy():
    report["verdict"] = "was_idle"
else:
    calls = {}
    for tool, params in (("dj_set", {"action": "stop"}), ("stop_music", {})):
        answer = mcp.call(tool, params, 8.0)
        result = answer.get("result") if isinstance(answer, dict) and isinstance(answer.get("result"), dict) else {}
        calls[tool] = "timeout" if answer is None else ("ok" if result.get("success") is True else "fail:%s" % (result.get("error") or result.get("message") or "success!=true"))
    t0 = time.monotonic()
    mcp.spin_until(lambda: not busy(), idle_timeout)
    report["stop_calls"] = calls
    report["after"] = describe()
    report["idle_after_s"] = round(time.monotonic() - t0, 1)
    report["verdict"] = "stuck" if busy() else "stopped"
mcp.close()
print(json.dumps(report, ensure_ascii=False))
sys.exit(0)
MUSICPY
}

# ensure_music_stopped_for_act INHERITS -- предусловие акта (issue #3871).
# INHERITS=yes -- акт наследует окно диалога прошлого (inherits_previous_act=true): в этом
# окне музыка может быть частью разговора -- не трогаем, но говорим об этом.
# Лог ``E2E_PRECONDITION music=stopped|was_idle|skipped``; играет и не гаснет за
# E2E_MUSIC_IDLE_WAIT (10 с) -- ``E2E_FATAL: перед актом играет музыка``, exit 2 (trap EXIT
# вернёт узлы на боевые БД). Не удалось даже прочитать -- тоже FATAL: тишину не доказали.
# Требует: robot_ros(), log().
ensure_music_stopped_for_act() {
    local inherits="${1:-no}" wait_s="${E2E_MUSIC_IDLE_WAIT:-10}" code_b64 out result_line parsed verdict detail
    if [ "$inherits" = "yes" ]; then
        echo "E2E_PRECONDITION music=skipped"
        log "🎵 музыка: акт наследует окно диалога (inherits_previous_act=true) — состояние плеера не трогаем"
        return 0
    fi
    code_b64="$(_e2e_music_stop_py | base64 | tr -d '\n\r')"
    out="$(robot_ros "python3 -c \"import base64;exec(base64.b64decode('${code_b64}').decode())\" '${wait_s}'" 2>/dev/null)"
    result_line="$(printf '%s\n' "$out" | grep -a '^{' | tail -1)"
    parsed="$(printf '%s' "$result_line" | python3 -c 'import json, sys
try:
    p = json.load(sys.stdin)
except Exception:
    print("error\tнет JSON-ответа"); sys.exit(0)
if not isinstance(p, dict) or "verdict" not in p:
    print("error\t%s" % ((p.get("error") if isinstance(p, dict) else None) or "ответ без verdict")); sys.exit(0)
b, a = p.get("before") or {}, p.get("after") or {}
def fmt(d):
    return "snapshot=нет" if not d.get("snapshot", True) else "playing=%s dj=%s track_id=%s" % (d.get("playing"), d.get("dj"), d.get("track_id"))
print("%s\tбыло: %s; стоп: %s; после (%s с): %s" % (p["verdict"], fmt(b), p.get("stop_calls", {}), p.get("idle_after_s", "-"), fmt(a) if a else "-"))
' 2>/dev/null)"
    verdict="$(printf '%s' "$parsed" | cut -f1)"
    detail="$(printf '%s' "$parsed" | cut -f2-)"
    case "$verdict" in
        was_idle)
            echo "E2E_PRECONDITION music=was_idle"
            log "🎵 музыка: перед актом тихо (${detail})"
            return 0
            ;;
        stopped)
            echo "E2E_PRECONDITION music=stopped"
            log "🎵 музыка: перед актом играла — остановлена через /mcp/execute dj_set stop + stop_music (${detail})"
            return 0
            ;;
        stuck)
            echo "E2E_FATAL: перед актом играет музыка — стоп отправлен, но плеер не стал idle за ${wait_s} с (${detail}) — замер под музыкой нечитаем (issue #3871)" >&2
            exit 2
            ;;
        *)
            echo "E2E_FATAL: перед актом не удалось проверить /voice/music/state (${detail:-<пусто>}) — тишину не доказали, замер мог пойти под музыку (issue #3871)" >&2
            printf '%s\n' "$out" | tail -3 >&2
            exit 2
            ;;
    esac
}

# Issue #3270: убирает из лога voice-assistant дамп LLM-запроса
# (deepseek.py::_dump_llm_request): строки START/END, «  [i] role: '...'» (текст
# промпта, история и tool_calls прошлых ходов) и «  tools(N): имя, имя, ...».
# Паттерн сценария должен находить СЛЕДЫ ВЫПОЛНЕНИЯ, а не имя тула в каталоге,
# который печатается на каждом ходе (ml06: PATTERN_OK music_library при
# невызванном туле). Читает stdin, пишет stdout. Режем по форме строки, а не
# блоком START..END: при обрезке --since END может отсутствовать, и блочный
# фильтр съел бы весь остальной лог.
strip_llm_request_dump() {
    # Issue #3860: в выводе launch у строки дампа есть префикс ``[dialog_node-3]`` --
    # якорь ``^`` без необязательного префикса не видел ``[17] user: '...<recent_actions>
    # [{"tool":"register_speaker"...`` и давал ложные ``forbidden tool calls``.
    # Тот же шаблон держит e2e_tool_match.strip_llm_request_dump (acceptance).
    grep -vE '(LLM REQUEST (START|END))|^(\[[A-Za-z0-9_]+-[0-9]+\])?[[:space:]]*\[[0-9]+\] [A-Za-z_]+: |^(\[[A-Za-z0-9_]+-[0-9]+\])?[[:space:]]*tools\([0-9]+\): ' || true
}

# --- issue #3857 (эпик #3683) -- ждём СОБЫТИЙ робота, а не таймеров -----------
#
# Раньше: после play -- ``sleep E2E_REACTION_WINDOW`` (40 с) при любом ответе; перед шагом --
# 20 с окно лога + 15 с «тишины» по grep строк синтеза. Теперь харнесс каждые 2 с читает лог
# робота и спрашивает e2e_events.py (решает код по маркерам, не оператор по таймеру):
#   * ответ готов -- v2: ``📊 [turn] ... stage=done`` хода + все ``SpeechEvent`` реплик закрыты
#     (finished/cancelled/rejected по speech_id); v1: ``Публикую TTS finished event`` по speech_id;
#   * робот ещё говорит -- ``started`` без парного закрытия; после конца речи +2 с (хвост ReSpeaker);
#   * потолок ожидания -- прежний E2E_REACTION_WINDOW / E2E_SILENCE_WAIT_MAX: в худшем случае
#     поведение то же, что было. E2E_EVENT_WAITS=0 возвращает фиксированный sleep.
_E2E_EVENT_GREP='ПРИНЯТО|SpeechEvent|\[turn\]|LLM REQUEST START|TTS finished event|🔊 TTS:'
_E2E_SPEECH_GREP='SpeechEvent|TTS finished event|🔊 TTS:'

_events_py() { printf '%s' "${E2E_EVENTS_PY:-${SCRIPT_DIR_E2E:-.}/e2e_events.py}"; }

# Первая строка вывода -- часы робота (date +%s.%N), дальше отфильтрованный docker logs.
# ``; true``: пустой grep не должен выглядеть как сбой SSH -- сбой виден по отсутствию числа в 1-й строке.
_events_fetch() {  # $1=since (RFC3339 или 600s) $2=egrep
    ${ROBOT_SSH} "date +%s.%N; docker logs voice-assistant --since '$1' 2>&1 | grep -aE '$2'; true" 2>/dev/null
}

_ev_kv() {  # stdin = key=value строки; $1=key
    sed -n "s/^$1=//p" | head -1 | tr -d '\r'
}

# wait_reply_done BEFORE -- ждёт «ответ состоялся и робот замолчал». rc=0 -- дождались,
# rc=1 -- потолок (дальше решает check_cycle, как и раньше). Пишет E2E_REPLY_AFTER (с) и
# E2E_REPLY_NO_SPEECH=1, если ход закончился без речи (silent_ok шага либо льгота E2E_NO_SPEECH_GRACE).
wait_reply_done() {
    local since="$1" poll="${E2E_EVENT_POLL:-2}" ceiling="${E2E_REACTION_WINDOW:-40}"
    local grace="${E2E_NO_SPEECH_GRACE:-8}" tail_s="${E2E_SPEECH_TAIL:-2}"
    local t0 now out verdict sig quiet prev_sig="" stable=0 nospeech_at=0 silent_flag=""
    [ "${E2E_STEP_SILENT_OK:-0}" = "1" ] && silent_flag="--silent-ok"
    E2E_REPLY_NO_SPEECH=0
    t0="$(date +%s)"
    while :; do
        out="$(_events_fetch "$since" "$_E2E_EVENT_GREP" | python3 "$(_events_py)" state --engine "${E2E_DIALOG_ENGINE:-v1}" $silent_flag)"
        verdict="$(printf '%s\n' "$out" | _ev_kv verdict)"
        sig="$(printf '%s\n' "$out" | _ev_kv sig)"
        quiet="$(printf '%s\n' "$out" | _ev_kv quiet_for)"
        now="$(date +%s)"
        case "$verdict" in
            done|silent)
                if [ "$sig" = "$prev_sig" ]; then stable=$((stable + 1)); else stable=1; fi
                prev_sig="$sig"
                if awk "BEGIN{exit !(${quiet:--1} >= $tail_s)}" 2>/dev/null || [ "$stable" -ge 2 ]; then
                    [ "$verdict" = "silent" ] && E2E_REPLY_NO_SPEECH=1
                    E2E_REPLY_AFTER=$((now - t0)); return 0
                fi ;;
            nospeech)
                [ "$nospeech_at" = "0" ] && nospeech_at="$now"
                if [ $((now - nospeech_at)) -ge "$grace" ]; then
                    E2E_REPLY_NO_SPEECH=1; E2E_REPLY_AFTER=$((now - t0)); return 0
                fi ;;
            *) stable=0; prev_sig=""; nospeech_at=0 ;;
        esac
        if [ $((now - t0)) -ge "$ceiling" ]; then
            E2E_REPLY_AFTER=$((now - t0)); return 1
        fi
        sleep "$poll"
    done
}

# warm_act_audio_path -- issue #3886: прогрев звукового тракта перед ПЕРВОЙ фразой акта.
# Акт 2 марафона (22-23.10) два прогона подряд терял «Робот,» в первой фразе n201: после
# пред-синтеза (#3875) PLAY идёт через ~2 с после сброса сессии/e2e-БД, а раньше между ними
# лежали ~20 с живого синтеза. В файле «Робот,» есть (1.1 с речи после 1.5 с adelay), VAD
# робота его видит; значит режется на пути звук/STT-канал после простоя. Прогрев: 2 с ЧИСТОЙ
# тишины через тот же sink (будит sink, VAD не триггерит) + пауза E2E_ACT_WARMUP_SEC (по
# умолчанию 15; 0 = выключено), которую раньше неявно давал синтез. Только v2-акт.
# Возвращает 0 всегда (прогрев -- не повод ронять акт), в лог пишет что реально сделано.
warm_act_audio_path() {  # $1=каталог для временного wav
    local dir="${1:-/tmp}" wait_s="${E2E_ACT_WARMUP_SEC:-15}" wav
    case "$wait_s" in ''|*[!0-9]*) wait_s=15 ;; esac
    if [ "$wait_s" = "0" ]; then
        echo ">>> прогрев звукового тракта: выключен (E2E_ACT_WARMUP_SEC=0)"
        return 0
    fi
    wav="$dir/.warmup_silence.wav"
    if ffmpeg -nostdin -y -f lavfi -i "anullsrc=r=16000:cl=mono" -t 2 -ac 1 -ar 16000 "$wav" >/dev/null 2>&1         && paplay "$wav" >/dev/null 2>&1; then
        echo ">>> прогрев звукового тракта: 2 с тишины через sink сыграны, пауза ${wait_s}s"
    else
        echo ">>> прогрев звукового тракта: тишину через sink сыграть не вышло (ffmpeg/paplay) -- только пауза ${wait_s}s"
    fi
    sleep "$wait_s"
    return 0
}

# wait_robot_quiet -- v2: «робот ещё говорит» = SpeechEvent started/queued без пары finished/
# cancelled/rejected (по speech_id; «протухшие» без событий > 60 с не считаются). Тихо --
# +2 с после последнего события речи. Пишет E2E_STEP_SILENCE_SEC. rc=0 тихо, rc=1 потолок.
wait_robot_quiet() {
    local poll="${E2E_EVENT_POLL:-2}" max="${E2E_SILENCE_WAIT_MAX:-90}" tail_s="${E2E_SPEECH_TAIL:-2}"
    local t0 now out nopen sig quiet fetch prev_sig="" stable=0
    t0="$(date +%s)"
    while :; do
        out="$(_events_fetch "${E2E_QUIET_LOOKBACK:-600s}" "$_E2E_SPEECH_GREP" | python3 "$(_events_py)" quiet --engine "${E2E_DIALOG_ENGINE:-v1}")"
        fetch="$(printf '%s\n' "$out" | _ev_kv fetch)"
        nopen="$(printf '%s\n' "$out" | _ev_kv open)"
        sig="$(printf '%s\n' "$out" | _ev_kv sig)"
        quiet="$(printf '%s\n' "$out" | _ev_kv quiet_for)"
        now="$(date +%s)"
        if [ "$sig" = "$prev_sig" ]; then stable=$((stable + 1)); else stable=1; fi
        prev_sig="$sig"
        if [ "$fetch" = "1" ] && [ "${nopen:-1}" = "0" ] \
           && { [ "$sig" = "0" ] || awk "BEGIN{exit !(${quiet:--1} >= $tail_s)}" 2>/dev/null || [ "$stable" -ge 2 ]; }; then
            E2E_STEP_SILENCE_SEC=$((now - t0)); return 0
        fi
        if [ $((now - t0)) -ge "$max" ]; then
            E2E_STEP_SILENCE_SEC=$((now - t0)); return 1
        fi
        sleep "$poll"
    done
}

# barge_in шага: JSON {"after":"speech_started|llm_request|segment_started","n":N,"delay_sec":X}.
_barge_field() {  # $1=json $2=поле
    BARGE_JSON="$1" python3 -c 'import json, os, sys
try:
    print(json.loads(os.environ["BARGE_JSON"]).get(sys.argv[1], ""))
except Exception:
    print("")' "$2"
}

# wait_barge_anchor SINCE JSON -- ждёт якорь перебивания в логе с SINCE (BEFORE фразы, которую
# перебиваем). rc=0 -- якорь наступил: E2E_ANCHOR_TS (метка лога робота, с), E2E_ANCHOR_ID,
# E2E_ANCHOR_NOW (часы робота в момент наблюдения). rc=1 -- за потолок не наступил.
wait_barge_anchor() {
    local since="$1" spec="$2" after n t0 now out
    after="$(_barge_field "$spec" after)"; n="$(_barge_field "$spec" n)"
    case "$after" in
        speech_started|llm_request|segment_started) ;;
        *) E2E_ANCHOR_ERROR="неизвестный barge_in.after='${after}'"; return 2 ;;
    esac
    t0="$(date +%s)"
    while :; do
        out="$(_events_fetch "$since" "$_E2E_EVENT_GREP" | python3 "$(_events_py)" anchor --engine "${E2E_DIALOG_ENGINE:-v1}" --after "$after" --n "${n:-1}")"
        if [ "$(printf '%s\n' "$out" | _ev_kv found)" = "1" ]; then
            E2E_ANCHOR_TS="$(printf '%s\n' "$out" | _ev_kv ts)"
            E2E_ANCHOR_ID="$(printf '%s\n' "$out" | _ev_kv speech_id)"
            E2E_ANCHOR_NOW="$(printf '%s\n' "$out" | _ev_kv now)"
            awk "BEGIN{exit !($E2E_ANCHOR_TS > 0)}" 2>/dev/null || E2E_ANCHOR_TS="$E2E_ANCHOR_NOW"
            return 0
        fi
        now="$(date +%s)"
        [ $((now - t0)) -ge "${E2E_REACTION_WINDOW:-40}" ] && return 1
        sleep "${E2E_BARGE_POLL:-1}"
    done
}

# step_wait_after_play SINCE -- что делать после play команды. E2E_WAIT_MODE:
#   fixed  -- E2E_EVENT_WAITS=0 (слепой sleep, как раньше) либо якорь не наступил за потолок;
#   anchor -- СЛЕДУЮЩИЙ шаг перебивает (barge_in): ждали его якорь, конца ответа не ждём;
#   event  -- ждали событий (wait_reply_done).
# E2E_REPLY_AFTER -- сколько секунд ждали (идёт в лог шага ``reply_after=``).
step_wait_after_play() {  # $1=since
    E2E_WAIT_MODE="fixed"; E2E_REPLY_AFTER="${E2E_REACTION_WINDOW:-40}"; E2E_REPLY_NO_SPEECH=0
    if [ "${E2E_EVENT_WAITS:-1}" = "0" ]; then
        sleep "${E2E_REACTION_WINDOW:-40}"
        return 0
    fi
    if [ -n "${E2E_STEP_NEXT_BARGE_IN:-}" ]; then
        local _t0
        _t0="$(date +%s)"
        if wait_barge_anchor "$1" "$E2E_STEP_NEXT_BARGE_IN"; then
            E2E_WAIT_MODE="anchor"; E2E_REPLY_AFTER=$(( $(date +%s) - _t0 ))
            E2E_PENDING_ANCHOR_SINCE="$1"; E2E_PENDING_ANCHOR_SPEC="$E2E_STEP_NEXT_BARGE_IN"
        fi
        return 0   # якорь не наступил: потолок уже выжат; шаг-перебиватель получит SKIP
    fi
    wait_reply_done "$1"
    E2E_WAIT_MODE="event"
    return 0
}

# barge_hold SPEC -- выдержать delay_sec от якоря по часам робота (якорь наблюдали раньше, чем
# дошли до play). Печатает фактическую задержку якорь -> начало play (с).
barge_hold() {
    local spec="$1" delay robot_now remaining
    delay="$(_barge_field "$spec" delay_sec)"
    robot_now="$(${ROBOT_SSH} "date +%s.%N" 2>/dev/null | tr -d '\r')"
    case "$robot_now" in ''|*[!0-9.]*) robot_now="$E2E_ANCHOR_NOW" ;; esac
    remaining="$(awk -v d="${delay:-0}" -v n="$robot_now" -v a="$E2E_ANCHOR_TS" 'BEGIN{r=d-(n-a); printf "%.2f", (r>0?r:0)}')"
    awk "BEGIN{exit !($remaining > 0)}" 2>/dev/null && sleep "$remaining"
    awk -v n="$robot_now" -v a="$E2E_ANCHOR_TS" -v r="$remaining" 'BEGIN{printf "%.1f", (n-a)+r}'
}

restore_robot_tts_default() {
    [ -n "${E2E_RUN_BEFORE:-}" ] || return 0
    local logs provider code_b64 out result_line verdict
    # Строки, доказывающие, что за прогон голос/провайдер МЕНЯЛИ (а не что
    # тул есть в каталоге tools(N) — тот лежит в логе на каждом ходе).
    local markers_re="\[set_voice\] voice=|\[issue 1765\] set_provider: '"
    logs="$(${ROBOT_SSH} "docker logs voice-assistant --since '${E2E_RUN_BEFORE}' 2>&1" 2>/dev/null || echo '')"
    if ! printf '%s' "$logs" | grep -qE "$markers_re"; then
        log "🎙️ TTS: за прогон голос/провайдер не меняли — робот не трогаем"
        return 0
    fi
    provider="$(_ros2_param_value "$(_ros2_param_get_raw /tts_node provider)")"
    case "$provider" in
        yandex|minimax|silero) ;;
        *)
            log "❌ ВНИМАНИЕ: TTS — акт менял голос, но /tts_node provider не прочитан ('${provider:-<пусто>}') — голос НЕ возвращён, робот говорит голосом из акта!"
            return 0
            ;;
    esac
    code_b64="$(_e2e_tts_restore_py | base64 | tr -d '\n\r')"
    # robot_ros принимает ОДНУ строку команды (её разбирает eval на роботе,
    # как у «ros2 param get '…'» выше) — кавычки ставим сами. В строке только
    # base64-алфавит и имя провайдера из белого списка выше.
    out="$(robot_ros "python3 -c \"import base64;exec(base64.b64decode('${code_b64}').decode())\" '${provider}'" 2>/dev/null)"
    result_line="$(printf '%s\n' "$out" | grep -a '^{' | tail -1)"
    verdict="$(printf '%s' "$result_line" | python3 -c 'import json, sys
try:
    p = json.load(sys.stdin)
except Exception:
    print("FAIL\tнет JSON-ответа"); sys.exit(0)
r = p.get("result") if isinstance(p.get("result"), dict) else {}
d = r.get("data") if isinstance(r.get("data"), dict) else {}
if r.get("success") is True:
    print("OK\t%s\t%s" % (d.get("provider", ""), d.get("voice_set", "")))
else:
    print("FAIL\t%s" % (p.get("error") or r.get("error") or r.get("message") or "success!=true"))
' 2>/dev/null)"
    case "$verdict" in
        OK*)
            log "🎙️ TTS: акт менял голос — возвращён дефолт set_voice в обход LLM: provider=$(printf '%s' "$verdict" | cut -f2) voice=$(printf '%s' "$verdict" | cut -f3)"
            ;;
        *)
            log "❌ ВНИМАНИЕ: TTS — дефолтный голос НЕ возвращён ($(printf '%s' "$verdict" | cut -f2-)), робот говорит голосом из акта!"
            log "   почини вручную: фразой «Робот, переключись на ${provider} и говори голосом по умолчанию» или set_voice через scripts/music/live_check_mcp_call.py"
            ;;
    esac
}
