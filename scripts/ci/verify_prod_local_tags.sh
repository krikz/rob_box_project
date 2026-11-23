#!/usr/bin/env bash
# ============================================================================
# verify_prod_local_tags.sh — fail-closed проверка пинов .image-versions.<tag>
# по локальному registry ДО первого SSH на Pi (issue #5110, продолжение #4653).
#
# Зачем: деплой с registry_source=local берёт образы из локального registry по
# пинам `docker/{main,vision,sound}/.image-versions.<IMAGE_TAG>`. Если пина нет
# в registry (или пин — плавающий `latest`/`dev`), робот либо падает на
# `compose pull` уже после остановки контейнеров, либо молча остаётся на старом
# образе (тихая деградация, ADR-0018). Замер 23.11.2026: prod-пины
# `.image-versions.latest` = `latest-b2e7144`, но `teleop-humble-latest-b2e7144`
# и `quest-humble-latest-b2e7144` в registry НЕТ — только плавающие
# `teleop-humble-latest` / `quest-humble-latest`. Такой деплой обязан падать
# здесь, на runner'е, до SSH.
#
# Использование:
#   verify_prod_local_tags.sh [TAG]        # TAG по умолчанию — $IMAGE_TAG
#
# Окружение:
#   ROBBOX_REGISTRY_HOST — хост:порт локального registry (10.1.1.249:5000)
#   REGISTRY_REPO        — репозиторий в registry (krikz/rob_box)
#   VERIFY_IMAGE_SCRIPT  — путь к scripts/ci/verify-image-in-registry.sh
#                          (по умолчанию — рядом с этим скриптом)
#
# Правило «переменная пина → имя образа»: `<var>` без суффикса `_TAG`, в нижнем
# регистре, подчёркивания → дефисы, плюс `-<ROS_DISTRO>-`. То есть
# TELEOP_TAG → teleop-humble-<пин>. Правило совпадает с compose-файлами
# (`image: …:teleop-${ROS_DISTRO:-humble}-${TELEOP_TAG:-…}`); паритет держит
# test_verify_prod_local_tags.py::test_image_name_rule_matches_compose.
#
# Образ `supercollider` не входит в проверку: у него нет пина *_TAG, compose
# берёт `${IMAGE_TAG}` напрямую (`supercollider-dev`). Это отдельная известная
# дыра пинов, а не часть .image-versions.
#
# Коды возврата:
#   0 — все пины разрешаются, плавающих нет;
#   1 — есть отсутствующий/плавающий пин или registry недоступен (деплой
#       останавливается, на Pi ничего не тронуто);
#   2 — неверный вызов / нет файлов пинов для запрошенного тега.
# ============================================================================

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REGISTRY="${ROBBOX_REGISTRY_HOST:-10.1.1.249:5000}"
REPO="${REGISTRY_REPO:-krikz/rob_box}"
VERIFY_IMAGE="${VERIFY_IMAGE_SCRIPT:-$ROOT/scripts/ci/verify-image-in-registry.sh}"

TAG="${1:-${IMAGE_TAG:-}}"
if [ -z "$TAG" ]; then
    echo "usage: $(basename "$0") [TAG]  (TAG по умолчанию — \$IMAGE_TAG)" >&2
    exit 2
fi

if [ ! -x "$VERIFY_IMAGE" ] && [ ! -f "$VERIFY_IMAGE" ]; then
    echo "::error title=Deploy preflight (#5110)::не найден $VERIFY_IMAGE — проверить пины нечем, деплой остановлен"
    exit 1
fi

echo "🔎 preflight #5110: пины .image-versions.${TAG} против http://${REGISTRY}/v2/${REPO}"

# Registry должен отвечать: недоступный registry — это не «пин есть», а честный FAIL.
probe_code="$(curl -s -o /dev/null -w '%{http_code}' --max-time 15 "http://${REGISTRY}/v2/" || echo 000)"
case "$probe_code" in
    200|401) ;;
    *)
        echo "::error title=Deploy preflight (#5110)::registry http://${REGISTRY} недоступен (HTTP ${probe_code}) — проверить пины невозможно, деплой остановлен ДО SSH"
        exit 1
        ;;
esac

shopt -s nullglob
pin_files=("$ROOT"/docker/*/.image-versions."$TAG")
if [ "${#pin_files[@]}" -eq 0 ]; then
    echo "::error title=Deploy preflight (#5110)::не найдено docker/*/.image-versions.${TAG} в ${ROOT} — нечего проверять, деплой остановлен"
    exit 2
fi

floating=()
missing=()
checked=()
declare -A seen=()

for pin_file in "${pin_files[@]}"; do
    component="$(basename "$(dirname "$pin_file")")"
    ros_distro="humble"
    while IFS= read -r line || [ -n "$line" ]; do
        line="${line#"${line%%[![:space:]]*}"}"   # lstrip
        line="${line%"${line##*[![:space:]]}"}"   # rstrip
        case "$line" in
            ''|\#*) continue ;;
        esac
        key="${line%%=*}"
        value="${line#*=}"
        if [ "$key" = "ROS_DISTRO" ] && [ -n "$value" ]; then
            ros_distro="$value"
            continue
        fi
        case "$key" in
            *_TAG) ;;
            *) continue ;;
        esac
        if [ -z "$value" ]; then
            floating+=("${component}/.image-versions.${TAG}: ${key} пуст")
            continue
        fi
        case "$value" in
            latest|dev|test|local)
                floating+=("${component}/.image-versions.${TAG}: ${key}=${value} — плавающий тег, не пин")
                continue
                ;;
        esac
        name="${key%_TAG}"
        name="${name,,}"
        name="${name//_/-}"
        image="${name}-${ros_distro}-${value}"
        [ -n "${seen[$image]:-}" ] && continue
        seen[$image]=1
        if bash "$VERIFY_IMAGE" "$REGISTRY" "$image" "$REPO" >/dev/null 2>&1; then
            echo "   ✅ $image"
            checked+=("$image")
        else
            echo "   ❌ $image"
            missing+=("$image (${component}/${key}=${value})")
        fi
    done < "$pin_file"
done

if [ "${#floating[@]}" -gt 0 ]; then
    for item in "${floating[@]}"; do
        echo "::error title=Deploy preflight (#5110)::плавающий пин — ${item}. Пин обязан быть SHA-тегом сборки (например latest-b2e7144), иначе деплой молча выкачает чужой/старый образ"
    done
fi

if [ "${#missing[@]}" -gt 0 ]; then
    for item in "${missing[@]}"; do
        echo "::error title=Deploy preflight (#5110)::образа нет в registry ${REGISTRY} — ${item}. Деплой остановлен ДО SSH: контейнеры на Pi не тронуты"
    done
fi

if [ "${#floating[@]}" -gt 0 ] || [ "${#missing[@]}" -gt 0 ]; then
    echo "❌ preflight #5110: FAIL — плавающих пинов ${#floating[@]}, отсутствующих в registry ${#missing[@]} (проверено ${#checked[@]} образов)"
    exit 1
fi

echo "✅ preflight #5110: все ${#checked[@]} пинов .image-versions.${TAG} есть в registry ${REGISTRY} — можно деплоить"
exit 0
