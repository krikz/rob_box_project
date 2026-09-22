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
