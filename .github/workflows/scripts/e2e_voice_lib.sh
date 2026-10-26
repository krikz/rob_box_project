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
#   minimax, таблица 22.09: худшая пара 0.684, anton/ermil = 0.527 (но ermil и zahar
#     оба женские: Гриша ≈ Борис, n210). Таблица ниже — от 23.10, см. #3931.
#   silero, СТАРАЯ таблица: zahar/filipp (baya/xenia) = 0.722 — ровно на
#     границе, baya/kseniya = 0.788. Ниже baya отдан alena, а zahar получил
#     kseniya: худшая пара 0.678 (anton/ermil = aidar/eugene).
#   yandex — НЕ ИЗМЕРЕН: ключ на роботе отвечает PERMISSION_DENIED (та же
#     архивная папка, из-за которой падает synth_yandex). Мерить, когда
#     ключ починят: measure_tts_voice_distinctness.py --provider yandex.
#
# Подкрутку питча/темпа как ЗАМЕНУ смены голоса проверили и отклонили (22.09):
# фиксированный сдвиг питча разводит эмбеддинги (anton[+6] vs ermil: 0.70 → 0.42,
# intra-voice не страдает), но ломает распознавание: vosk на тех же файлах даёт
# WER 0.79 при pitch=-6 и 0.26 при pitch=+3 против 0.05-0.16 без сдвига.
# 23.10 (#3931) питч/темп возвращали («голоса далеко друг от друга, как у реальных людей»):
# питч +5/темп 1.2 у Бориса, -5/0.85 у Гриши. Замер на ЖИВОМ канале (катана -> воздух ->
# ReSpeaker -> Yandex STT + resemblyzer, 254 проигрыша) это опроверг:
#   * вейк «Робот» терялся: RMc 6/8, BTc 3/8 (62% потерь), HCc 6/8 против 16-19 из 18-22 без просодии;
#   * разводит питч не голоса, а регистр: cos между голосами одного регистра 0.8-0.9 при любом
#     питче/темпе/сдвиге формант (ReliableMan с формантами +-20% к себе же 0.84-0.91).
# Итог 23.10: просодия снята (0 / 1.0), остался питч +2 у Бориса. Саша/Гриша/Валера на Russian_* - один
# низкий мужской регистр, между собой 0.9: РОБОТ ИХ НЕ РАЗЛИЧАЕТ (docs/dialog/role_voices_biometrics_2026-10-23.md).
#
# 26.10 (#3992, решение Шифу: «голоса разнесены по эмбеддингам далеко, чтоб он не путался»; вариант Б - Гриша и
# Валера другого регистра): подбор по ЖИВОМУ каналу (колонка katana -> воздух -> ReSpeaker -> resemblyzer) среди
# 332 системных голосов MiniMax (русских 8, но речь на русском тянет любой голос: STT понимает) и Yandex/Silero.
# Выбор - четыре голоса из НЕрусского каталога MiniMax, читающие русский текст; отчёт, матрицы, команды:
# docs/dialog/role_voices_biometrics_2026-10-26.md, evidence/role-voices-live-2026-10-26/.
#   Саша   anton          English_Jovialman                  мужской, вейк 21/21
#   Борис  ermil/madirus  English_WiseScholar                мужской, вейк 18/21
#   Гриша  zahar          Chinese (Mandarin)_Gentle_Senior   ЖЕНСКИЙ (пожилой), вейк 20/21
#   Валера filipp/kostya  English_Whispering_girl            ЖЕНСКИЙ (шёпот), вейк 17/21; реплики Валеры без «Робот,»
# К чужим по записям (макс / p95 / доля пар выше порога опознания 0.72): худшая Саша-Борис 0.76 / 0.70 / 2.7%,
# остальные 0.66-0.74 / 0.61-0.69 / 0-0.8%; свой к своей мин 0.79-0.92. Критерий «макс < 0.72» СТРОГО не достигнут
# (0.74-0.76 у трёх пар), по p95 и по решению самого speaker_id (4 профиля, 0 склеек на 28 фразах) - да.
# Yandex и Silero на этом канале дают те же 0.8-0.9 внутри мужского/женского регистра: четвёрку не собирают.
# Гендер: у Гриши и Валеры теперь женские голоса - реплики в сценариях обязаны быть в женском роде.
#
# Каталоги-источники — src/rob_box_voice/rob_box_voice/tts_voice_registry.py
# (PROVIDER_VOICES). Голос, который уже native для провайдера, не трогаем:
# так можно задать --voice aidar / --voice Russian_CrazyQueen напрямую.
#
# Примеры:
#   map_tts_voice yandex  anton  → anton
#   map_tts_voice silero  anton  → aidar
#   map_tts_voice silero  zahar  → kseniya
#   map_tts_voice minimax ermil  → English_WiseScholar:0:1.0       (voice:pitch:speed)
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
                # voice:pitch:speed (#3931). Роли разнесены не только голосом, но и
                # питчем/темпом, как разные люди. Ровно эту строку получают
                # synth_minimax (разбирает) и ключ кэша (хранит): смена питча/темпа
                # не вернёт старые wav. Замер: docs/dialog/role_voices_biometrics_2026-10-23.md
                # Живой канал (колонка -> ReSpeaker), 23.10, #3931: по всем MiniMax cos между
                # голосами одного регистра 0.8-0.9 (питч/темп/сдвиг формант не разводят), между
                # низким мужским и высоким 0.65-0.75. Вейк «Робот» без просодии теряется в 10-15%
                # фраз, с питчем +5/темпом 1.2 - в 38-62%. Поэтому просодия снята (питч +2 у Бориса жил до 26.10).
                # Каст 26.10 (#3992), живой канал; ПРОСОДИЯ 0/1.0 (питч/темп не разводят, а вейк ломают).
                anton|"")        printf 'English_Jovialman:0:1.0' ;;                 # Саша: мужской (вейк 21/21)
                ermil|madirus)   printf 'English_WiseScholar:0:1.0' ;;               # Борис: мужской (вейк 18/21; к Саше макс 0.76, p95 0.70)
                zahar)           printf 'Chinese (Mandarin)_Gentle_Senior:0:1.0' ;;  # Гриша: ЖЕНСКИЙ пожилой (вейк 20/21)
                filipp|kostya)   printf 'English_Whispering_girl:0:1.0' ;;           # Валера: ЖЕНСКИЙ шёпот (вейк 17/21; его реплики без «Робот,»)
                # Запасной: Portuguese_StrictBoss (женский; к ролям макс 0.69-0.73, n=6, ВЕЙК НЕ МЕРИЛИ). Второго запасного нет:
                # presenter_female и Russian_PessimisticGirl (старые запасные: arina/rita) к Грише 0.82-0.84, а ko_ConsiderateSenior
                # к Борису 0.83. Имя rita поэтому уходит в голос по умолчанию (ниже), а не в голос, который склеится с Гришей.
                arina)           printf 'Portuguese_StrictBoss:0:1.0' ;;
                alena)           printf 'Russian_BrightHeroine' ;;
                jane)            printf 'Russian_AmbitiousWoman' ;;
                omazh|rush)      printf 'Russian_CrazyQueen' ;;
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
# INHERITS=yes -- акт наследует окно диалога прошлого (inherits_previous_act=true). Раньше плеер такого акта не
# трогали, и утечка доезжала: n712 «всё, глуши» не остановил трек, и он играл весь акт 8 марафона 23.10 (#4027).
# Каждый акт марафона кончается стопом музыки, значит играющая на старте наследующего акта музыка -- утечка: стоп
# тем же путём и честная пометка акта ``E2E_RECOVER music at_act_start`` (E2E_ACT_RECOVERED, сводка).
# Лог ``E2E_PRECONDITION music=stopped|was_idle``; играет и не гаснет за
# E2E_MUSIC_IDLE_WAIT (10 с) -- ``E2E_FATAL: перед актом играет музыка``, exit 2 (trap EXIT
# вернёт узлы на боевые БД). Не удалось даже прочитать -- тоже FATAL: тишину не доказали.
# Требует: robot_ros(), log().
ensure_music_stopped_for_act() {
    local inherits="${1:-no}" wait_s="${E2E_MUSIC_IDLE_WAIT:-10}" code_b64 out result_line parsed verdict detail
    code_b64="$(_e2e_music_stop_py | base64 | tr -d '\n\r')"
    out="$(robot_ros "python3 -c \"import base64;exec(base64.b64decode('${code_b64}').decode())\" '${wait_s}'" 2>/dev/null)"
    result_line="$(printf '%s\n' "$out" | grep -a '^{' | tail -1 | sed 's/^[^{]*//; s/}[^}]*$/}/')"
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
            if [ "$inherits" = "yes" ]; then
                echo "E2E_RECOVER music at_act_start (${detail})"
                E2E_ACT_RECOVERED="${E2E_ACT_RECOVERED:+${E2E_ACT_RECOVERED},}music"
                log "🎵 музыка: прошлый акт оставил её играть — утечка состояния (#4027), остановлена через /mcp/execute dj_set stop + stop_music (${detail})"
                return 0
            fi
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
#   * #3989: фраза ушла в бэклог без вейка (``[backlog] accumulated reason=no_wake``) или ход проглотила тишина
#     (``stage=silenced``) -- ждать нечего, run_step решает о восстановлении шага (step_recover).
_E2E_EVENT_GREP='ПРИНЯТО|SpeechEvent|\[turn\]|LLM REQUEST START|TTS finished event|🔊 TTS:|\[backlog\] accumulated|tier1 kind='
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

# wait_reply_done BEFORE [BARGE] -- ждёт «ответ состоялся и робот замолчал». v2 (#3954): единственный судья
# шага -- сам e2e_events.py; check_cycle для v2 не зовётся. rc=0 -- дождались (ответ или silent_ok),
# rc=1 -- потолок при принятой фразе (ход не кончился), rc=2 -- ход кончился сбоем/отменой/без речи (FAIL сразу,
# E2E_REPLY_REASON), rc=3 -- ПРИНЯТО нет за E2E_ACCEPT_WINDOW (переигрывать сразу, не ждать потолок),
# rc=4 -- до хода не дошло: no_wake (вейк потерян, фраза в бэклоге) / silenced (тишина), #3989.
# Пишет E2E_REPLY_AFTER (с), E2E_REPLY_VERDICT и E2E_REPLY_NO_SPEECH=1, если ход закончился без речи
# (silent_ok шага либо льгота E2E_NO_SPEECH_GRACE; у v1 льгота ведёт к rc=0, как раньше).
wait_reply_done() {
    local since="$1" barge_flag="" poll="${E2E_EVENT_POLL:-2}" ceiling="${E2E_REACTION_WINDOW:-40}"
    local accept_win=0 reason
    [ "${2:-0}" = "1" ] && barge_flag="--barge"
    [ "${E2E_DIALOG_ENGINE:-v1}" = "v2" ] && accept_win="${E2E_ACCEPT_WINDOW:-15}"
    E2E_REPLY_VERDICT="wait"; E2E_REPLY_REASON="-"
    local grace="${E2E_NO_SPEECH_GRACE:-8}" tail_s="${E2E_SPEECH_TAIL:-2}"
    local t0 now out verdict sig quiet prev_sig="" stable=0 nospeech_at=0 silent_flag=""
    [ "${E2E_STEP_SILENT_OK:-0}" = "1" ] && silent_flag="--silent-ok"
    E2E_REPLY_NO_SPEECH=0
    t0="$(date +%s)"
    while :; do
        now="$(date +%s)"
        out="$(_events_fetch "$since" "$_E2E_EVENT_GREP" | python3 "$(_events_py)" state --engine "${E2E_DIALOG_ENGINE:-v1}" $silent_flag $barge_flag --waited "$((now - t0))" --accept-window "$accept_win")"
        verdict="$(printf '%s\n' "$out" | _ev_kv verdict)"
        sig="$(printf '%s\n' "$out" | _ev_kv sig)"
        quiet="$(printf '%s\n' "$out" | _ev_kv quiet_for)"
        reason="$(printf '%s\n' "$out" | _ev_kv reason)"
        now="$(date +%s)"
        case "$verdict" in
            failed|cancelled)
                E2E_REPLY_VERDICT="$verdict"; E2E_REPLY_REASON="$reason"
                E2E_REPLY_AFTER=$((now - t0)); return 2 ;;
            no_accept)
                E2E_REPLY_VERDICT="no_accept"; E2E_REPLY_AFTER=$((now - t0)); return 3 ;;
            no_wake|silenced)
                E2E_REPLY_VERDICT="$verdict"; E2E_REPLY_REASON="$reason"
                E2E_REPLY_AFTER=$((now - t0)); return 4 ;;
            done|silent)
                if [ "$sig" = "$prev_sig" ]; then stable=$((stable + 1)); else stable=1; fi
                prev_sig="$sig"
                if awk "BEGIN{exit !(${quiet:--1} >= $tail_s)}" 2>/dev/null || [ "$stable" -ge 2 ]; then
                    [ "$verdict" = "silent" ] && E2E_REPLY_NO_SPEECH=1
                    E2E_REPLY_VERDICT="$verdict"; E2E_REPLY_AFTER=$((now - t0)); return 0
                fi ;;
            nospeech)
                [ "$nospeech_at" = "0" ] && nospeech_at="$now"
                if [ $((now - nospeech_at)) -ge "$grace" ]; then
                    E2E_REPLY_NO_SPEECH=1; E2E_REPLY_VERDICT="nospeech"; E2E_REPLY_AFTER=$((now - t0))
                    # v2: ход кончился, речи не было и шаг тишину не разрешал -- FAIL сразу, без повторов.
                    [ "${E2E_DIALOG_ENGINE:-v1}" = "v2" ] && { E2E_REPLY_REASON="no_speech"; return 2; }
                    return 0
                fi ;;
            *) stable=0; prev_sig=""; nospeech_at=0 ;;
        esac
        if [ $((now - t0)) -ge "$ceiling" ]; then
            E2E_REPLY_VERDICT="ceiling"; E2E_REPLY_AFTER=$((now - t0)); return 1
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
step_wait_after_play() {  # $1=since $2=1, если шаг сам перебивает (barge_in)
    E2E_WAIT_MODE="fixed"; E2E_REPLY_AFTER="${E2E_REACTION_WINDOW:-40}"; E2E_REPLY_NO_SPEECH=0
    E2E_REPLY_VERDICT="fixed"; E2E_REPLY_REASON="-"; E2E_WAIT_RC=0
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
    wait_reply_done "$1" "${2:-0}"
    E2E_WAIT_RC=$?
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

# --- issue #3989 — восстановление шага: политика сценария и фразы -------------------------------------------
# Шаг ждёт ответа, а судья e2e_events.py сказал no_wake (вейк потерян в STT, фраза ушла в бэклог) или silenced
# (ход проглотила тишина). Что сказать роботу и повторять ли фразу шага -- RECOVERY, кому можно -- recover_kinds:
# поле ``recover`` сценария/шага (true|false|список видов), шаг-перебиватель и шаг, чья фраза и есть восстановление,
# не восстанавливаются. Здесь -- только чтение решений e2e_events.py; сам шаг -- step_recover в e2e_voice_test.sh.
E2E_RECOVER_POLICY_FILE="${E2E_RECOVER_POLICY_FILE:-}"
load_recover_policy() {  # $1 = файл сценария; поле recover не разобрано -- E2E_FATAL до первого шага
    local err total allowed
    E2E_RECOVER_POLICY_FILE="${OUT_DIR:-/tmp}/recover_policy.tsv"
    if ! err="$(python3 "$(_events_py)" policy --scenario "$1" 2>&1 >"$E2E_RECOVER_POLICY_FILE")"; then
        echo "E2E_FATAL: поле recover сценария не разобрано: ${err:-нет ответа} (issue #3989)" >&2
        exit 2
    fi
    total="$(grep -c . "$E2E_RECOVER_POLICY_FILE" 2>/dev/null)"
    allowed="$(awk -F'\t' '$2 != "-" && $2 != "-\r"' "$E2E_RECOVER_POLICY_FILE" | grep -c .)"
    log "🩹 восстановление шагов (#3989): разрешено ${allowed:-0} из ${total:-0} шагов (поле recover, barge_in) — $E2E_RECOVER_POLICY_FILE"
}
step_recover_kinds() {  # $1 = label шага -> «no_wake,silenced» или пусто
    [ -n "${E2E_RECOVER_POLICY_FILE:-}" ] && [ -f "$E2E_RECOVER_POLICY_FILE" ] || return 0
    awk -F'\t' -v l="$1" '$1 == l { sub(/\r$/, "", $2); if ($2 != "-") print $2; exit }' "$E2E_RECOVER_POLICY_FILE"
}
recover_allowed() {  # $1 = вид; разрешён ли он текущему шагу (E2E_STEP_RECOVER_KINDS ставит scenario-цикл)
    case ",${E2E_STEP_RECOVER_KINDS:-}," in
        *",$1,"*) return 0 ;;
    esac
    return 1
}
recovery_field() {  # $1 = вид, $2 = phrase|replay -> значение из RECOVERY (e2e_events.py)
    python3 "$(_events_py)" recovery --kind "$1" | _ev_kv "$2"
}

# --- issue #3989 п.5 — тишина не переживает акт ---------------------------------------------------------
# «Замолчи» ставит сцену Silence на 10 минут, снимает её только «Робот, говори» (решение Шифу 23.10). Акт, оборванный
# между ними, оставлял следующему молчащего робота, и все шаги, ждущие ответа, краснели «робот молчит». В начале
# КАЖДОГО v2-акта (и наследующего) харнесс читает latched /dialog/session (пишет dialog_node), решает
# e2e_events.session_report; тишина -- реплика снятия (фраза -- RECOVERY в e2e_events.py) в /voice/stt/utterance, как
# от STT, и новый снимок. Маркер «E2E_RECOVER silence at_act_start», пометка акта в сводке (E2E_ACT_RECOVERED). Не
# снялась -- E2E_FATAL: акт молчащего робота нечитаем. Не прочитать -- громко, но не фатал: шагам поможет step_recover.
# Рабочий процесс внутри voice-assistant (rclpy), как у музыки ниже. argv: read|unsilence, фраза (base64), сек ждать
# снимков после реплики. Печатает одну строку JSON: snapshot, scenes, [after], now.
_e2e_session_py() {
    cat <<'SESSIONPY'
import base64, json, sys, time
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rob_box_dialog_msgs.msg import SessionState, Utterance
mode, wait_s = sys.argv[1], float(sys.argv[3])
rclpy.init()
node = rclpy.create_node("e2e_act_session")
got = []
latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
node.create_subscription(SessionState, "/dialog/session", got.append, latched)
def spin_until(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        rclpy.spin_once(node, timeout_sec=0.2)
def scenes():
    if not got:
        return None
    return [{"kind": s.kind, "set_by": s.set_by, "ttl_until": s.ttl_until.sec + s.ttl_until.nanosec / 1e9}
            for s in got[-1].scenes]
spin_until(lambda: got, 5.0)  # latched: у живой ноды снимок приходит сразу
report = {"snapshot": bool(got), "scenes": scenes()}
if mode == "unsilence":
    pub = node.create_publisher(Utterance, "/voice/stt/utterance", 10)
    spin_until(lambda: pub.get_subscription_count() > 0, 5.0)
    msg = Utterance()
    msg.text = base64.b64decode(sys.argv[2]).decode("utf-8").strip()
    msg.source, msg.provider = Utterance.SOURCE_RESPEAKER, "e2e_harness"
    pub.publish(msg)
    spin_until(lambda: False, wait_s)  # снимков может быть несколько (собеседник, сцена): берём последний
    report["after"] = scenes()
report["now"] = time.time()
node.destroy_node()
rclpy.shutdown()
print(json.dumps(report, ensure_ascii=False))
SESSIONPY
}

_session_report() {  # $1 = вывод рабочего процесса -> key=value e2e_events.session_report
    printf '%s\n' "$1" | grep -a '^{' | tail -1 | python3 "$(_events_py)" session
}

ensure_not_silenced_for_act() {
    [ "${E2E_DIALOG_ENGINE:-v1}" = "v2" ] || return 0   # сцена Silence -- только у dialog v2
    local code_b64 phrase phrase_b64 out rep detail wait_s="${E2E_UNSILENCE_WAIT:-8}"
    code_b64="$(_e2e_session_py | base64 | tr -d '\n\r')"
    out="$(robot_ros "python3 -c \"import base64;exec(base64.b64decode('${code_b64}').decode())\" read - 0" 2>/dev/null)"
    rep="$(_session_report "$out")"
    detail="$(printf '%s\n' "$rep" | _ev_kv detail)"
    if [ "$(printf '%s\n' "$rep" | _ev_kv fetch)" != "1" ]; then
        echo "E2E_PRECONDITION silence=unknown"
        log "⚠️ тишина: /dialog/session не прочитан ($(printf '%s\n' "$out" | tail -1)) — старт акта не проверен, шагам поможет восстановление (#3989)"
        return 0
    fi
    if [ "$(printf '%s\n' "$rep" | _ev_kv silenced)" != "1" ]; then
        echo "E2E_PRECONDITION silence=off"
        log "🤫 тишина: перед актом робот не в тишине (${detail})"
        return 0
    fi
    phrase="$(recovery_field silenced phrase)"
    phrase_b64="$(printf '%s' "$phrase" | base64 | tr -d '\n\r')"
    out="$(robot_ros "python3 -c \"import base64;exec(base64.b64decode('${code_b64}').decode())\" unsilence '${phrase_b64}' '${wait_s}'" 2>/dev/null)"
    rep="$(_session_report "$out")"
    if [ "$(printf '%s\n' "$rep" | _ev_kv after)" = "0" ]; then
        echo "E2E_RECOVER silence at_act_start (было: ${detail})"
        E2E_ACT_RECOVERED="${E2E_ACT_RECOVERED:+${E2E_ACT_RECOVERED},}silence"
        log "🤫 тишина: прошлый акт оставил робота в тишине (${detail}) — снята репликой «${phrase}» в /voice/stt/utterance (#3989 п.5)"
        return 0
    fi
    echo "E2E_FATAL: перед актом робот в тишине (${detail}), реплика «${phrase}» не сняла её за ${wait_s} с — шаги акта проглотила бы тишина (issue #3989)" >&2
    printf '%s\n' "$out" | tail -3 >&2
    exit 2
}

# --- issue #4027 — скорость и тон TTS не переживают акт ---------------------------------------------------
# Акт 4 (n409-n411 «помедленнее», «тараторь», «басом») двигает yandex_speed/pitch_shift (после T1 -- и
# minimax_speed), а возврат set_voice ниже их не трогает: 23.10 yandex_speed=1.2 и pitch_shift=0.8 доехали до акта 10.
# Как restore_node_params: значения ЧИТАЮТСЯ до акта (snapshot_tts_prosody) и возвращаются к ним, не к хардкоду;
# запись проверяется чтением. Чтение трёх параметров -- один ssh.
E2E_TTS_PROSODY_PARAMS="yandex_speed minimax_speed pitch_shift"
E2E_TTS_PROSODY_FILE="${E2E_TTS_PROSODY_FILE:-}"
_tts_prosody_read() {  # -> «параметр<US>значение» по каждому прочитанному параметру /tts_node
    robot_ros "for p in ${E2E_TTS_PROSODY_PARAMS}; do echo \"@\$p\"; ros2 param get /tts_node \$p --no-daemon; done" 2>/dev/null \
        | awk '/^@/ { p = substr($0, 2); sub(/\r$/, "", p); next }
               p != "" && /Double value is: / { sub(/.*Double value is: /, ""); sub(/\r$/, ""); printf "%s\037%s\n", p, $0; p = "" }'
}
_tts_prosody_value() {  # $1 = вывод _tts_prosody_read, $2 = параметр
    printf '%s\n' "$1" | awk -F'\037' -v p="$2" '$1 == p { print $2; exit }'
}
snapshot_tts_prosody() {  # $1 = файл: «параметр<US>значение до акта» для restore_robot_tts_default
    E2E_TTS_PROSODY_FILE="$1"
    _tts_prosody_read > "$1" 2>/dev/null || : > "$1"
    if [ -s "$1" ]; then
        log "🎙️ TTS: скорость/тон до акта: $(tr '\037\n' '= ' < "$1")"
    else
        log "⚠️ TTS: скорость/тон /tts_node до акта не прочитаны — после акта их вернуть будет не к чему (#4027)"
    fi
}
_restore_tts_prosody() {
    local file="${E2E_TTS_PROSODY_FILE:-}" now after cmd="" changed="" bad="" param value cur
    [ -n "$file" ] && [ -s "$file" ] || return 0
    now="$(_tts_prosody_read)"
    while IFS=$'\x1f' read -r param value; do
        [ -n "$param" ] || continue
        cur="$(_tts_prosody_value "$now" "$param")"
        _node_param_values_match "$cur" "$value" && continue
        case "$value" in *.*) ;; *) value="${value}.0" ;; esac   # Double, а не Integer: иначе ros2 param set откажет
        cmd="${cmd}ros2 param set /tts_node ${param} ${value} --no-daemon >/dev/null; "
        changed="${changed:+${changed}, }${param} ${cur:-?} -> ${value}"
    done < "$file"
    if [ -z "$cmd" ]; then
        log "🎙️ TTS: скорость и тон за акт не менялись"
        return 0
    fi
    robot_ros "$cmd" >/dev/null 2>&1
    after="$(_tts_prosody_read)"
    while IFS=$'\x1f' read -r param value; do
        [ -n "$param" ] || continue
        _node_param_values_match "$(_tts_prosody_value "$after" "$param")" "$value" \
            || bad="${bad:+${bad}, }${param}=$(_tts_prosody_value "$after" "$param") (ждали ${value})"
    done < "$file"
    if [ -z "$bad" ]; then
        log "🎙️ TTS: скорость/тон возвращены к значениям до акта: ${changed}"
    else
        log "❌ ВНИМАНИЕ: TTS — скорость/тон НЕ возвращены (${bad}), робот говорит с параметрами из акта!"
        log "   почини вручную: ssh <robot> \"docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 param set /tts_node <параметр> <значение> --no-daemon'\""
    fi
}

# --- issue #3710 п.2 — e2e-БД дикторов акта в артефакты ------------------------------------------------
# Разбор каста (клоны, отравленная галерея, «неопознанный») шёл по логам: сама e2e-БД оставалась в контейнере и
# стиралась следующим марафоном. В конце акта харнесс забирает её копию ОДНИМ ssh -- tar файла БД и его -wal/-shm
# (/data видна только из контейнера), только чтение. Боевую /data/speakers.db не трогает: путь обязан кончаться на
# .e2e.db. Артефакты: speakers_e2e_db.tar (на билд-машине, в OUT_DIR акта) и speakers_e2e_db.json -- таблицы, число
# строк и строки без BLOB (эмбеддинги не выносятся); json уходит в архив артефактов workflow (*.json).
E2E_SPEAKER_DB="${E2E_SPEAKER_DB:-/data/speakers.e2e.db}"
collect_e2e_speaker_db() {  # $1 = каталог артефактов акта
    local dir="$1" db="${E2E_SPEAKER_DB}" base tarf remote summary
    [ "${E2E_ACT_STARTED:-0}" = "1" ] || return 0
    case "$db" in
        /*.e2e.db) ;;
        *) log "❌ e2e-БД дикторов: путь '${db}' — не e2e-БД (*.e2e.db), боевую не копирую"; return 0 ;;
    esac
    base="${db##*/}"
    tarf="${dir}/speakers_e2e_db.tar"
    remote="cd '${db%/*}' && f=\$(ls -- '${base}' '${base}-wal' '${base}-shm' 2>/dev/null); [ -n \"\$f\" ] && tar -cf - \$f"
    ${ROBOT_SSH} "docker exec voice-assistant sh -c $(printf '%q' "$remote")" > "$tarf" 2>/dev/null
    if [ ! -s "$tarf" ]; then
        rm -f "$tarf"
        log "🗄️ e2e-БД дикторов: копии нет (в контейнере нет ${db} либо ssh/tar не ответили)"
        echo "E2E_SPEAKER_DB none"
        return 0
    fi
    summary="$(python3 - "$tarf" "$base" "${dir}/speakers_e2e_db.json" <<'DBPY'
import json, os, sqlite3, sys, tarfile, tempfile
tar_path, base, out_path = sys.argv[1:4]
report = {"db": base, "files": [], "tables": {}}
try:
    with tempfile.TemporaryDirectory() as tmp:
        with tarfile.open(tar_path) as tf:
            members = [m for m in tf.getmembers() if m.isfile() and m.name in (base, base + "-wal", base + "-shm")]
            report["files"] = [{"name": m.name, "bytes": m.size} for m in members]
            tf.extractall(tmp, members=members)
        con = sqlite3.connect(os.path.join(tmp, base))
        for (name,) in con.execute("SELECT name FROM sqlite_master WHERE type='table' ORDER BY name").fetchall():
            quoted = '"%s"' % name.replace('"', '""')
            cur = con.execute("SELECT * FROM %s LIMIT 200" % quoted)
            cols = [d[0] for d in cur.description]
            rows = [{c: v for c, v in zip(cols, row) if not isinstance(v, (bytes, memoryview))} for row in cur]
            report["tables"][name] = {"count": con.execute("SELECT COUNT(*) FROM %s" % quoted).fetchone()[0],
                                      "rows": rows}
        con.close()
except Exception as exc:  # копия есть и без сводки: разбор -- руками по tar
    report["error"] = "%s: %s" % (type(exc).__name__, exc)
with open(out_path, "w", encoding="utf-8") as fh:
    json.dump(report, fh, ensure_ascii=False, indent=1, default=str)
print(", ".join("%s=%d" % (k, v["count"]) for k, v in report["tables"].items()) or report.get("error", "таблиц нет"))
DBPY
)"
    log "🗄️ e2e-БД дикторов скопирована (только чтение): ${tarf} ($(wc -c < "$tarf" | tr -d ' ') байт), строки: ${summary}"
    echo "E2E_SPEAKER_DB copied ${tarf}"
}

restore_robot_tts_default() {
    _restore_tts_prosody   # #4027: скорость и тон -- к значениям до акта (своё чтение, без E2E_RUN_BEFORE)
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
    result_line="$(printf '%s\n' "$out" | grep -a '^{' | tail -1 | sed 's/^[^{]*//; s/}[^}]*$/}/')"
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
