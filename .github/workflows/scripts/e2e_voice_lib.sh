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
# после перевода. Поэтому таблица ниже — не «ближайший по полу», а
# «гарантированно различимый»: anton/ermil/zahar/filipp → четыре разных
# speaker'а у каждого провайдера (у Silero ради этого берутся и женские —
# различимость важнее совпадения пола, робот всё равно слышит синтетику).
#
# Каталоги-источники — src/rob_box_voice/rob_box_voice/tts_voice_registry.py
# (PROVIDER_VOICES). Голос, который уже native для провайдера, не трогаем:
# так можно задать --voice aidar / --voice Russian_CrazyQueen напрямую.
#
# Примеры:
#   map_tts_voice yandex  anton  → anton
#   map_tts_voice silero  anton  → aidar
#   map_tts_voice silero  zahar  → baya
#   map_tts_voice minimax ermil  → Russian_HandsomeChildhoodFriend
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
                zahar|arina)                     printf 'baya' ;;
                filipp|jane)                     printf 'xenia' ;;
                alena)                           printf 'kseniya' ;;
                omazh|rush)                      printf 'xenia' ;;
                *)                               printf 'aidar' ;;
            esac
            return 0
            ;;
        minimax)
            case "$voice" in
                Russian_*|male-qn-qingse|female-shaonv) printf '%s' "$voice"; return 0 ;;
                anton|"")        printf 'Russian_ReliableMan' ;;
                ermil|madirus)   printf 'Russian_HandsomeChildhoodFriend' ;;
                zahar)           printf 'Russian_Bad-temperedBoy' ;;
                filipp|kostya)   printf 'Russian_AttractiveGuy' ;;
                alena)           printf 'Russian_BrightHeroine' ;;
                jane)            printf 'Russian_AmbitiousWoman' ;;
                arina)           printf 'Russian_PessimisticGirl' ;;
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
# после перевода. Поэтому таблица ниже — не «ближайший по полу», а
# «гарантированно различимый»: anton/ermil/zahar/filipp → четыре разных
# speaker'а у каждого провайдера (у Silero ради этого берутся и женские —
# различимость важнее совпадения пола, робот всё равно слышит синтетику).
#
# Каталоги-источники — src/rob_box_voice/rob_box_voice/tts_voice_registry.py
# (PROVIDER_VOICES). Голос, который уже native для провайдера, не трогаем:
# так можно задать --voice aidar / --voice Russian_CrazyQueen напрямую.
#
# Примеры:
#   map_tts_voice yandex  anton  → anton
#   map_tts_voice silero  anton  → aidar
#   map_tts_voice silero  zahar  → baya
#   map_tts_voice minimax ermil  → Russian_HandsomeChildhoodFriend
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
                zahar|arina)                     printf 'baya' ;;
                filipp|jane)                     printf 'xenia' ;;
                alena)                           printf 'kseniya' ;;
                omazh|rush)                      printf 'xenia' ;;
                *)                               printf 'aidar' ;;
            esac
            return 0
            ;;
        minimax)
            case "$voice" in
                Russian_*|male-qn-qingse|female-shaonv) printf '%s' "$voice"; return 0 ;;
                anton|"")        printf 'Russian_ReliableMan' ;;
                ermil|madirus)   printf 'Russian_HandsomeChildhoodFriend' ;;
                zahar)           printf 'Russian_Bad-temperedBoy' ;;
                filipp|kostya)   printf 'Russian_AttractiveGuy' ;;
                alena)           printf 'Russian_BrightHeroine' ;;
                jane)            printf 'Russian_AmbitiousWoman' ;;
                arina)           printf 'Russian_PessimisticGirl' ;;
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
