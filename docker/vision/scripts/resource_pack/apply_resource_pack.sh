#!/usr/bin/env bash
# ============================================================================
# apply_resource_pack.sh — Ресурсный пак: привести файловую систему хоста
# в соответствие манифесту бинарных ресурсов Vision Pi.
# ============================================================================
# План: docs/plans/2026-09-15-resource-pack.md (§2 интерфейс, §3 манифест,
# §5 раскладка, §7 Этап 1). Манифест — manifest.yaml рядом с этим файлом.
#
# Заменяет собой (поэтапно) четыре разных способа доставки одного и того же
# класса вещей: два почти дословно одинаковых download_*_hef.sh и ступень
# скачивания STT/TTS-моделей в docker/vision/voice_base/Dockerfile.
#
# ---------------------------------------------------------------------------
# Контракт (план §2.3)
# ---------------------------------------------------------------------------
#  1. Идемпотентность. Файл на месте + sha256 совпал → no-op, без сети.
#  2. Целостность. Каждый open-ресурс с непустым sha256 проверяется ПОСЛЕ
#     скачивания и ДО установки в target. Скачанный не тот файл не
#     устанавливается никогда.
#  3. Версионирование. Единственный источник версии/URL/sha256 — манифест.
#  4. Поведение без сети. required: hard → exit 1 с объяснением, что именно
#     сломается. required: soft → предупреждение + продолжение, деградация
#     называется вслух (ADR-0018).
#  5. Открытые vs закрытые. open — скрипт качает сам. gated — скрипт только
#     проверяет (наличие + sha256) и говорит, чего не хватает и что делать;
#     кладёт файл человек, один раз (docs/deployment/hailo-vendor-artifacts.md).
#
# Несовпадение sha256 — ВСЕГДА hard-ошибка, даже для required: soft.
# Отсутствие ресурса это объявленная деградация; файл с другим содержимым —
# неизвестное состояние, и «тихо скачал не то» здесь запрещено by design.
#
# ---------------------------------------------------------------------------
# Использование
# ---------------------------------------------------------------------------
#   bash apply_resource_pack.sh                       # весь манифест
#   bash apply_resource_pack.sh --only yolov8n-hef    # одна запись
#   bash apply_resource_pack.sh --only a,b            # несколько
#   bash apply_resource_pack.sh --dry-run             # план действий, без сети
#   bash apply_resource_pack.sh --force               # перекачать даже валидное
#   bash apply_resource_pack.sh --host katana         # включить gated-записи
#
# Переменные окружения (план §2.2):
#   RESOURCE_PACK_MANIFEST — путь к манифесту (default: рядом со скриптом)
#   RESOURCE_PACK_ROOT     — корень раскладки (default: /opt/rob_box)
#   RESOURCE_PACK_FORCE    — 1 = как --force
#   RESOURCE_PACK_HOST     — vision-pi | katana, для gated-записей
#
# Exit codes:
#   0 — всё на месте (возможны объявленные soft-деградации, см. итог)
#   1 — hard-провал: не встал required: hard ресурс ИЛИ не сошёлся sha256
#   2 — ошибка использования / невалидный манифест
#
# Тест: tests/unit/docker/test_resource_pack.py (фикстура, без внешней сети)
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

MANIFEST="${RESOURCE_PACK_MANIFEST:-${SCRIPT_DIR}/manifest.yaml}"
PACK_ROOT="${RESOURCE_PACK_ROOT:-/opt/rob_box}"
FORCE="${RESOURCE_PACK_FORCE:-0}"
PACK_HOST="${RESOURCE_PACK_HOST:-}"
ONLY=""
DRY_RUN=0

TAG="[resource_pack]"

log()  { echo "${TAG} $*"; }
warn() { echo "${TAG} WARN: $*" >&2; }
err()  { echo "${TAG} ERROR: $*" >&2; }

usage() {
    # Печатаем ровно шапку-комментарий (до `set -euo pipefail`), чтобы --help
    # не начал вываливать код скрипта, если шапка когда-нибудь сдвинется.
    sed -n '2,/^set -euo pipefail$/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
}

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
while [ $# -gt 0 ]; do
    case "$1" in
        --only)
            ONLY="${2:-}"
            [ -n "$ONLY" ] || { err "--only требует аргумент (имена через запятую)"; exit 2; }
            shift 2 ;;
        --only=*)  ONLY="${1#--only=}"; shift ;;
        --manifest)
            MANIFEST="${2:-}"
            [ -n "$MANIFEST" ] || { err "--manifest требует путь"; exit 2; }
            shift 2 ;;
        --manifest=*) MANIFEST="${1#--manifest=}"; shift ;;
        --root)
            PACK_ROOT="${2:-}"
            [ -n "$PACK_ROOT" ] || { err "--root требует путь"; exit 2; }
            shift 2 ;;
        --root=*)  PACK_ROOT="${1#--root=}"; shift ;;
        --host)
            PACK_HOST="${2:-}"
            [ -n "$PACK_HOST" ] || { err "--host требует аргумент (vision-pi|katana)"; exit 2; }
            shift 2 ;;
        --host=*)  PACK_HOST="${1#--host=}"; shift ;;
        --force)   FORCE=1; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) err "неизвестный аргумент: $1"; echo "Подсказка: $0 --help" >&2; exit 2 ;;
    esac
done

[ -f "$MANIFEST" ] || { err "манифест не найден: ${MANIFEST}"; exit 2; }

case "$PACK_HOST" in
    ""|vision-pi|katana) : ;;
    *) err "--host: ожидается vision-pi | katana, получено '${PACK_HOST}'"; exit 2 ;;
esac

# ---------------------------------------------------------------------------
# Разбор манифеста
# ---------------------------------------------------------------------------
# Свой парсер, а не python+PyYAML: скрипт обязан работать на голой Vision Pi
# и на build-хосте без питоновских зависимостей (ровно как download_*_hef.sh
# сегодня — bash + coreutils + curl). Поддерживается ТОЛЬКО та подсхема,
# которую описывает manifest.yaml: список записей с плоскими скалярными
# полями, вложенные списки и folded-блоки (`>`), значения которых скрипту
# не нужны, пропускаются. Всё, что не распозналось, не «угадывается»:
# запись без обязательных полей роняет прогон с exit 2.
parse_manifest() {
    awk '
        # Незакрытый folded-блок (`key: >`) отдаём одной строкой: degrade_note
        # человеку нужен целиком, иначе в логе деплоя вместо «что именно
        # деградирует» будет пустота.
        function flush_fold() {
            if (fold_key == "") return
            gsub(/^[ \t]+|[ \t]+$/, "", fold_buf)
            print fold_idx "\t" fold_key "\t" fold_buf
            fold_key = ""; fold_buf = ""
        }
        function emit(k, v,   q) {
            gsub(/^[ \t]+|[ \t]+$/, "", k)
            sub(/^[ \t]+/, "", v)
            if (substr(v, 1, 1) == DQ) {
                q = index(substr(v, 2), DQ)
                v = (q > 0) ? substr(v, 2, q - 1) : substr(v, 2)
            } else if (substr(v, 1, 1) == SQ) {
                q = index(substr(v, 2), SQ)
                v = (q > 0) ? substr(v, 2, q - 1) : substr(v, 2)
            } else {
                sub(/[ \t]+#.*$/, "", v)
                sub(/[ \t]+$/, "", v)
                if (v == ">" || v == "|" || v == ">-" || v == "|-") {
                    fold_key = k; fold_idx = idx; fold_ind = cur_ind; fold_buf = ""
                    return
                }
            }
            print idx "\t" k "\t" v
        }
        function kv(s,   p) {
            p = index(s, ":")
            if (p == 0) return
            emit(substr(s, 1, p - 1), substr(s, p + 1))
        }
        BEGIN {
            idx = 0; in_res = 0; fold_key = ""; rec_ind = -1; fld_ind = -1
            DQ = sprintf("%c", 34); SQ = sprintf("%c", 39)
        }
        # CRLF: манифест, отредактированный на Windows (или скопированный
        # туда-обратно), приезжает с \r на конце каждой строки. Без этой
        # строки парсер молча разбирал ОДНУ запись вместо десяти — поймано
        # 21.09.2026 при прогоне на Vision Pi. Молчаливо неполный манифест
        # хуже явной ошибки: деплой разложил бы часть ресурсов и отчитался
        # об успехе.
        { sub(/\r$/, "") }
        /\t/ { print "0\t__PARSE_ERROR__\tтабуляция в отступе, строка " NR; exit 0 }
        {
            match($0, /^ */); ind = RLENGTH
            s = substr($0, ind + 1)
            sub(/[ \t]+$/, "", s)
            if (fold_key != "") {
                if (s == "") next                      # пустая строка внутри блока
                if (ind > fold_ind) { fold_buf = fold_buf " " s; next }
                flush_fold()
            }
            if (s == "" || substr(s, 1, 1) == "#") next
            cur_ind = ind
            # Отступ списка берём из файла, а не прибиваем к двум пробелам:
            # `resources:` + `- name:` на одном уровне — такой же валидный YAML
            # (так пишет yaml.safe_dump в тестовых фикстурах).
            if (substr(s, 1, 2) == "- ") {
                if (in_res == 0) next
                if (rec_ind < 0) { rec_ind = ind; fld_ind = ind + 2 }
                if (ind == rec_ind) { idx++; kv(substr(s, 3)); next }
                next                                   # элемент вложенного списка
            }
            if (ind == 0) {
                in_res = (s ~ /^resources:/) ? 1 : 0
                if (in_res == 1) { rec_ind = -1; fld_ind = -1 }
                next
            }
            if (in_res == 0) next
            if (rec_ind >= 0 && ind == fld_ind) { kv(s); next }
            next
        }
        END { flush_fold(); print "0\t__COUNT__\t" idx }
    ' "$1"
}

MANIFEST_KV="$(parse_manifest "$MANIFEST")"

if printf '%s\n' "$MANIFEST_KV" | grep -q "__PARSE_ERROR__"; then
    err "манифест не разобран: $(printf '%s\n' "$MANIFEST_KV" | awk -F'\t' '$2 == "__PARSE_ERROR__" { print $3 }')"
    exit 2
fi

COUNT="$(printf '%s\n' "$MANIFEST_KV" | awk -F'\t' '$2 == "__COUNT__" { print $3 }')"
if [ -z "$COUNT" ] || [ "$COUNT" -eq 0 ]; then
    err "в манифесте ${MANIFEST} нет ни одной записи под ключом resources:"
    exit 2
fi

field() {  # $1 = индекс записи, $2 = имя поля
    printf '%s\n' "$MANIFEST_KV" | awk -F'\t' -v i="$1" -v k="$2" '$1 == i && $2 == k { print $3; exit }'
}

# ---------------------------------------------------------------------------
# Утилиты
# ---------------------------------------------------------------------------
# Файл подаём на stdin, а не аргументом: coreutils экранирует «неудобные»
# имена (обратный слэш, перевод строки) и печатает хеш с ведущим «\» —
# сравнение с манифестом тогда не совпадает НИКОГДА при корректном файле.
# Тот же класс бага, что регистр хекса в download_retinaface_hef.sh (#2599).
sha256_of() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum < "$1" | cut -d' ' -f1 | tr '[:upper:]' '[:lower:]'
    elif command -v shasum >/dev/null 2>&1; then
        shasum -a 256 < "$1" | cut -d' ' -f1 | tr '[:upper:]' '[:lower:]'
    else
        echo "__no_sha_tool__"
    fi
}

lower() { printf '%s' "$1" | tr '[:upper:]' '[:lower:]'; }

# Абсолютный путь под /opt/rob_box переезжает в RESOURCE_PACK_ROOT (это и
# даёт тестируемость на фикстуре), любой другой абсолютный — как есть,
# относительный — от корня репозитория (build-context на katana).
resolve_target() {
    case "$1" in
        /opt/rob_box/*) printf '%s' "${PACK_ROOT}/${1#/opt/rob_box/}" ;;
        /*)             printf '%s' "$1" ;;
        *)              printf '%s' "${REPO_ROOT}/$1" ;;
    esac
}

file_size() {
    stat -c%s "$1" 2>/dev/null || stat -f%z "$1" 2>/dev/null || echo "?"
}

download_to() {  # $1 = url, $2 = dest
    if command -v curl >/dev/null 2>&1; then
        # -sS: без прогресс-бара (в логе деплоя по ssh он — мусор), но с
        # сообщением об ошибке. -f: HTTP 404 это провал, а не «скачал страницу».
        curl -fsSL --retry 3 --retry-delay 2 -o "$2" "$1"
    elif command -v wget >/dev/null 2>&1; then
        wget -q -O "$2" "$1"
    else
        err "ни curl, ни wget не найдены — скачать нечем"
        return 127
    fi
}

# Раньше zip распаковывался ВНУТРИ образа, где unzip стоял пакетом. На голой
# Vision Pi его может не быть (scripts/setup/*.sh его не ставят — проверено),
# а ронять деплой из-за отсутствующего архиватора, когда рядом лежит python3
# со штатным модулем zipfile, — глупо. Порядок: unzip, затем python3.
unpack_zip() {  # $1 = архив, $2 = каталог назначения
    if command -v unzip >/dev/null 2>&1; then
        unzip -q -o "$1" -d "$2" && return 0
        return 1
    fi
    if command -v python3 >/dev/null 2>&1; then
        warn "unzip не найден — распаковываю через python3 -m zipfile"
        python3 -m zipfile -e "$1" "$2" && return 0
        return 1
    fi
    err "ни unzip, ни python3 не найдены — распаковать нечем"
    return 1
}

selected() {  # $1 = name; пусто в --only → берём всё
    [ -z "$ONLY" ] && return 0
    printf '%s' ",${ONLY}," | grep -q ",$1,"
}

# ---------------------------------------------------------------------------
# Счётчики итога
# ---------------------------------------------------------------------------
HARD_FAILS=0
DEGRADED=""
NOOP=0
INSTALLED=0
SKIPPED=0

hard_fail() {  # $1 = name, $2... = причина
    local name="$1"; shift
    err "${name}: HARD FAIL — $*"
    HARD_FAILS=$((HARD_FAILS + 1))
}

degrade() {  # $1 = name, $2 = degrade_note, $3... = причина
    local name="$1" note="$2"; shift 2
    warn "${name}: ДЕГРАДАЦИЯ — $*"
    if [ -n "$note" ]; then
        warn "${name}: следствие — ${note}"
    fi
    # Последней строкой — присваивание: функция обязана возвращать 0, иначе
    # `set -e` у вызывающего превратит объявленную деградацию в внезапный
    # выход из скрипта на середине манифеста.
    DEGRADED="${DEGRADED} ${name}"
}

missing_resource() {  # $1 = name, $2 = required, $3 = degrade_note, $4... = причина
    local name="$1" required="$2" note="$3"; shift 3
    if [ "$required" = "hard" ]; then
        hard_fail "$name" "$* (required: hard → деплой обязан упасть)"
    else
        degrade "$name" "$note" "$*"
    fi
}

# ---------------------------------------------------------------------------
# open-ресурс
# ---------------------------------------------------------------------------
ensure_open() {  # $1 = индекс записи
    local i="$1"
    local name url sha unpack target required note verify tpath tdir expected actual tmp

    name="$(field "$i" name)"
    url="$(field "$i" url)"
    sha="$(lower "$(field "$i" sha256)")"
    unpack="$(field "$i" unpack)"
    [ -n "$unpack" ] || unpack="none"
    target="$(field "$i" target)"
    required="$(field "$i" required)"
    note="$(field "$i" degrade_note)"
    verify="$(field "$i" verify_file)"

    if [ -z "$url" ] || [ -z "$target" ] || [ -z "$required" ]; then
        err "${name}: запись type: open без url/target/required — манифест невалиден"
        exit 2
    fi
    case "$required" in hard|soft) : ;; *)
        err "${name}: required='${required}', ожидается hard|soft"; exit 2 ;;
    esac
    case "$unpack" in none|zip) : ;; *)
        err "${name}: unpack='${unpack}', поддерживается none|zip"; exit 2 ;;
    esac

    tpath="$(resolve_target "$target")"
    tdir="$(dirname "$tpath")"

    # --- уже на месте? ---
    local present=0
    if [ "$unpack" = "zip" ]; then
        if [ -d "$tpath" ] && { [ -z "$verify" ] || [ -f "${tpath}/${verify}" ]; }; then
            present=1
        fi
    elif [ -f "$tpath" ]; then
        present=1
    fi

    if [ "$present" -eq 1 ] && [ "$FORCE" != "1" ]; then
        if [ "$unpack" = "zip" ]; then
            log "${name}: OK no-op — ${tpath} распакован${verify:+, ${verify} на месте}"
            NOOP=$((NOOP + 1))
            return 0
        fi
        if [ -z "$sha" ]; then
            log "${name}: OK no-op — ${tpath} на месте ($(file_size "$tpath") байт)"
            warn "${name}: sha256 в манифесте ПУСТОЙ — целостность НЕ проверена (план §13.4)"
            NOOP=$((NOOP + 1))
            return 0
        fi
        actual="$(sha256_of "$tpath")"
        if [ "$actual" = "$sha" ]; then
            log "${name}: OK no-op — ${tpath} на месте, sha256 совпал"
            NOOP=$((NOOP + 1))
            return 0
        fi
        err "${name}: файл ${tpath} НА МЕСТЕ, но sha256 НЕ СОВПАЛ с манифестом."
        err "${name}:   ожидалось: ${sha}"
        err "${name}:   на диске:  ${actual}"
        err "${name}: это НЕ деградация, а неизвестное содержимое. Скрипт ничего не трогает."
        err "${name}: варианты: (а) сверить версию в манифесте; (б) осознанно перекачать —"
        err "${name}:           RESOURCE_PACK_FORCE=1 или --force."
        HARD_FAILS=$((HARD_FAILS + 1))
        return 0
    fi

    # --- надо качать ---
    if [ "$DRY_RUN" -eq 1 ]; then
        if [ "$present" -eq 1 ]; then
            log "${name}: DRY-RUN — на месте, но --force → перекачал бы ${url}"
        else
            log "${name}: DRY-RUN — скачал бы ${url} → ${tpath}"
        fi
        SKIPPED=$((SKIPPED + 1))
        return 0
    fi

    if ! mkdir -p "$tdir" 2>/dev/null; then
        missing_resource "$name" "$required" "$note" "не создать каталог ${tdir} (права?)"
        return 0
    fi

    # tmp кладём РЯДОМ с целью: mv внутри одной ФС атомарен, и контейнер
    # никогда не увидит полускачанный файл по целевому пути.
    tmp="$(mktemp "${tdir}/.${name}.XXXXXX")" || {
        missing_resource "$name" "$required" "$note" "не создать временный файл в ${tdir}"
        return 0
    }

    log "${name}: скачиваю ${url}"
    if ! download_to "$url" "$tmp"; then
        rm -f "$tmp"
        missing_resource "$name" "$required" "$note" "скачивание не удалось (сеть/URL/недоступен источник)"
        return 0
    fi

    if [ -n "$sha" ]; then
        actual="$(sha256_of "$tmp")"
        expected="$sha"
        if [ "$actual" != "$expected" ]; then
            rm -f "$tmp"
            err "${name}: sha256 скачанного файла НЕ СОВПАЛ — установка отменена."
            err "${name}:   ожидалось: ${expected}"
            err "${name}:   получено:  ${actual}"
            err "${name}:   источник:  ${url}"
            err "${name}: ничего не установлено: молча положить не тот файл нельзя (ADR-0018)."
            HARD_FAILS=$((HARD_FAILS + 1))
            return 0
        fi
        log "${name}: sha256 OK"
    else
        warn "${name}: sha256 в манифесте ПУСТОЙ — скачанное НЕ проверено (план §13.4)"
    fi

    if [ "$unpack" = "zip" ]; then
        if ! unpack_zip "$tmp" "$tdir"; then
            rm -f "$tmp"
            missing_resource "$name" "$required" "$note" "распаковка zip не удалась (ни unzip, ни python3 -m zipfile)"
            return 0
        fi
        rm -f "$tmp"
        if [ ! -d "$tpath" ]; then
            missing_resource "$name" "$required" "$note" "после распаковки нет каталога ${tpath} (архив другой структуры?)"
            return 0
        fi
        if [ -n "$verify" ] && [ ! -f "${tpath}/${verify}" ]; then
            missing_resource "$name" "$required" "$note" "после распаковки нет ${tpath}/${verify} — архив не тот"
            return 0
        fi
        log "${name}: OK установлен — ${tpath}${verify:+ (${verify} на месте)}"
        INSTALLED=$((INSTALLED + 1))
        return 0
    fi

    mv -f "$tmp" "$tpath"
    chmod 0644 "$tpath" 2>/dev/null || true
    log "${name}: OK установлен — ${tpath} ($(file_size "$tpath") байт)"
    INSTALLED=$((INSTALLED + 1))
}

# ---------------------------------------------------------------------------
# gated-ресурс: только проверка, скачать нельзя (нужен аккаунт Developer Zone)
# ---------------------------------------------------------------------------
ensure_gated() {  # $1 = индекс записи
    local i="$1"
    local name target target_host required note vendor_file version tpath actual sha

    name="$(field "$i" name)"
    target="$(field "$i" target)"
    target_host="$(field "$i" target_host)"
    required="$(field "$i" required)"
    note="$(field "$i" degrade_note)"
    vendor_file="$(field "$i" vendor_file)"
    version="$(field "$i" version)"
    sha="$(lower "$(field "$i" sha256)")"

    if [ -z "$target" ] || [ -z "$target_host" ] || [ -z "$required" ]; then
        err "${name}: запись type: gated без target/target_host/required — манифест невалиден"
        exit 2
    fi

    if [ -z "$PACK_HOST" ]; then
        log "${name}: пропущено — gated-запись для target_host=${target_host}, а --host/RESOURCE_PACK_HOST не задан."
        log "${name}:   /opt/rob_box/vendor на katana и на Vision Pi содержат РАЗНОЕ; угадывать хост скрипт не будет."
        SKIPPED=$((SKIPPED + 1))
        return 0
    fi
    if [ "$PACK_HOST" != "$target_host" ]; then
        log "${name}: пропущено — запись для ${target_host}, текущий хост объявлен как ${PACK_HOST}"
        SKIPPED=$((SKIPPED + 1))
        return 0
    fi

    tpath="$(resolve_target "$target")"

    if [ ! -f "$tpath" ]; then
        if [ "$DRY_RUN" -eq 1 ]; then
            log "${name}: DRY-RUN — ${tpath} отсутствует (скачать нельзя, кладёт человек)"
            SKIPPED=$((SKIPPED + 1))
            return 0
        fi
        err "${name}: НЕТ вендорного артефакта ${tpath}"
        err "${name}:   файл: ${vendor_file:-?} (HailoRT ${version:-?}), хост: ${target_host}"
        err "${name}:   скачать автоматически НЕЛЬЗЯ — нужен аккаунт Hailo Developer Zone."
        err "${name}:   процедура: docs/deployment/hailo-vendor-artifacts.md"
        missing_resource "$name" "$required" "$note" "вендорный артефакт не засеян на ${target_host}"
        return 0
    fi

    if [ -z "$sha" ]; then
        log "${name}: на месте — ${tpath} ($(file_size "$tpath") байт)"
        warn "${name}: sha256 в манифесте ПУСТОЙ — это НЕ проверка, а только test -f."
        warn "${name}: эталон заполняется вручную (Этап 4): sha256sum ${tpath}"
        NOOP=$((NOOP + 1))
        return 0
    fi

    actual="$(sha256_of "$tpath")"
    if [ "$actual" = "$sha" ]; then
        log "${name}: OK — ${tpath}, sha256 совпал (HailoRT ${version:-?})"
        NOOP=$((NOOP + 1))
        return 0
    fi
    err "${name}: sha256 вендорного артефакта НЕ СОВПАЛ."
    err "${name}:   ожидалось: ${sha}"
    err "${name}:   на диске:  ${actual}"
    err "${name}:   типичная причина — файл от другой версии HailoRT (манифест ждёт ${version:-?})."
    err "${name}:   что делать: docs/deployment/hailo-vendor-artifacts.md"
    HARD_FAILS=$((HARD_FAILS + 1))
}

# ---------------------------------------------------------------------------
# Главный цикл
# ---------------------------------------------------------------------------
log "манифест:  ${MANIFEST}"
log "корень:    ${PACK_ROOT}"
log "записей:   ${COUNT}${ONLY:+ (фильтр --only ${ONLY})}"
if [ "$DRY_RUN" -eq 1 ]; then
    log "режим:     DRY-RUN (ничего не скачивается и не пишется)"
fi
if [ "$FORCE" = "1" ]; then
    log "режим:     FORCE (перекачиваем даже валидное)"
fi

# --only с несуществующим именем — ошибка, а не тихий no-op: опечатка в
# workflow не должна выглядеть как успешный прогон.
if [ -n "$ONLY" ]; then
    ALL_NAMES="$(printf '%s\n' "$MANIFEST_KV" | awk -F'\t' '$2 == "name" { print $3 }')"
    IFS=',' read -r -a REQUESTED <<< "$ONLY"
    for want in "${REQUESTED[@]}"; do
        [ -n "$want" ] || continue
        if ! printf '%s\n' "$ALL_NAMES" | grep -qx "$want"; then
            err "--only ${want}: такой записи в манифесте нет. Доступны:"
            printf '%s\n' "$ALL_NAMES" | sed "s/^/${TAG}   /" >&2
            exit 2
        fi
    done
fi

i=1
while [ "$i" -le "$COUNT" ]; do
    r_name="$(field "$i" name)"
    r_type="$(field "$i" type)"

    if [ -z "$r_name" ]; then
        err "запись #${i} без поля name — манифест невалиден"
        exit 2
    fi

    if ! selected "$r_name"; then
        i=$((i + 1))
        continue
    fi

    case "$r_type" in
        open)  ensure_open  "$i" ;;
        gated) ensure_gated "$i" ;;
        build-time|git)
            log "${r_name}: пропущено — type: ${r_type}, доставляется не этим швом (план §6.3/§6.4)"
            SKIPPED=$((SKIPPED + 1)) ;;
        "")
            err "${r_name}: запись без поля type — манифест невалиден"
            exit 2 ;;
        *)
            err "${r_name}: неизвестный type='${r_type}' (open|gated|build-time|git)"
            exit 2 ;;
    esac
    i=$((i + 1))
done

# ---------------------------------------------------------------------------
# Итог
# ---------------------------------------------------------------------------
echo "${TAG} ────────────────────────────────────────"
log "итог: установлено ${INSTALLED}, без изменений ${NOOP}, пропущено ${SKIPPED}, hard-провалов ${HARD_FAILS}"
if [ -n "$DEGRADED" ]; then
    warn "объявленная деградация:${DEGRADED}"
    warn "это НЕ «всё хорошо» — потребители выше уйдут в stub (ADR-0018)."
fi

if [ "$HARD_FAILS" -gt 0 ]; then
    err "прогон неуспешен: ${HARD_FAILS} hard-провал(ов) выше."
    exit 1
fi
exit 0
