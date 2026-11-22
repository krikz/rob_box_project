#!/usr/bin/env bash
# Регресс-тест issue #5089: уборка образов на борту не снимает тег, на
# который ссылается контейнер (running или stopped), мусор без контейнеров
# удаляет, --dry-run ничего не меняет, пропавший тег восстанавливается
# из Image контейнера без сети.
#
# Деплой №8 (run 44991405421): `docker image prune -af` снял локальные теги
# `*-dev-acd5a8d` у 8 сервисов Main Pi при живых контейнерах
# (Total reclaimed space: 0B). Следующий `compose up --pull never` упал
# «No such image».
#
# Фейковый docker в PATH (как scripts/deploy/tests/test_preflight_pull_images.sh).
# Состояние образов и контейнеров — файлы в $WORK, не живой демон.
#
# Запуск: bash scripts/deploy/tests/test_docker_prune_safe.sh

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SCRIPT="$REPO_ROOT/scripts/deploy/docker_prune_safe.sh"
WORKFLOW="$REPO_ROOT/.github/workflows/L-Deploy and Verify.yml"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

pass=0
failures=0
ok() { echo "PASS: $1"; pass=$((pass + 1)); }
ko() { echo "FAIL: $1"; failures=$((failures + 1)); }
check()  { local msg=$1; shift; if "$@"; then ok "$msg"; else ko "$msg"; fi; }
refute() { local msg=$1; shift; if "$@"; then ko "$msg"; else ok "$msg"; fi; }

# --- состояние фейкового демона --------------------------------------------
# images.tsv:  <id>\t<ref>     ref="-" — dangling (<none>)
# containers.tsv: <cid>\t<image-id>\t<config-image>\t<service>
# rmi.log:     ID, которые фейк реально удалил
# tag.log:     "<id> <ref>" восстановленных тегов
reset_state() {
    : > "$WORK/images.tsv"
    : > "$WORK/containers.tsv"
    : > "$WORK/rmi.log"
    : > "$WORK/tag.log"
    : > "$WORK/out.txt"
}

add_image() { printf '%s\t%s\n' "$1" "$2" >> "$WORK/images.tsv"; }
add_container() { printf '%s\t%s\t%s\t%s\n' "$1" "$2" "$3" "$4" >> "$WORK/containers.tsv"; }

has_ref() { awk -F '\t' -v ref="$1" '$2 == ref { found=1 } END { exit !found }' "$WORK/images.tsv"; }
has_id()  { awk -F '\t' -v id="$1" '$1 == id { found=1 } END { exit !found }' "$WORK/images.tsv"; }
rmi_logged() { grep -qxF -- "$1" "$WORK/rmi.log"; }
tag_logged() { grep -qxF -- "$1 $2" "$WORK/tag.log"; }

mkdir -p "$WORK/bin"

# Обёртка: FAKE_PS_RC!=0 роняет `docker ps`, остальное отдаёт фейковый
# демон scripts/deploy/tests/fake_docker_prune_safe.sh (отдельный файл,
# чтобы кавычки демона не склеились с этим тестом).
cp "$REPO_ROOT/scripts/deploy/tests/fake_docker_prune_safe.sh" "$WORK/bin/docker.real"
chmod +x "$WORK/bin/docker.real"
cat > "$WORK/bin/docker" << 'WRAP'
#!/usr/bin/env bash
if [ "${FAKE_PS_RC:-0}" -ne 0 ] && [ "${1:-}" = "ps" ]; then
    echo "Cannot connect to the Docker daemon" >&2
    exit 1
fi
exec "$(dirname "$0")/docker.real" "$@"
WRAP
chmod +x "$WORK/bin/docker"

NAV2_REF="ghcr.io/krikz/rob_box/nav2-humble-dev-acd5a8d"
NAV2_ID="sha256:f635eb037d87"
OLD_REF="ghcr.io/krikz/rob_box/nav2-humble-dev-faa402e"
OLD_ID="sha256:aaaaaaaaaaaa"
UNUSED_ID="sha256:bbbbbbbbbbbb"
DANGLING_ID="sha256:cccccccccccc"

run_script() {
    export FAKE_STATE="$WORK"
    PATH="$WORK/bin:$PATH" bash "$SCRIPT" "$@" > "$WORK/out.txt" 2>&1
}

# --- (а) образ контейнера сохраняет ВСЕ теги; мусор удаляется --------------
# Тот же ID несёт и текущий тег, и старый (как в деплое №8: один sha,
# два имени). Контейнер ссылается на него. Соседний ID не используется.
reset_state
add_image "$NAV2_ID" "$NAV2_REF"
add_image "$NAV2_ID" "$OLD_REF"
add_image "$UNUSED_ID" "ghcr.io/krikz/rob_box/old-humble-dev-1111111"
add_image "$DANGLING_ID" "-"
add_container "c_nav2" "$NAV2_ID" "$NAV2_REF" "nav2"
export FAKE_COMPOSE_RC=0 FAKE_COMPOSE_IMAGES="$NAV2_REF" FAKE_PS_RC=0

check "a: script exits 0" run_script
check "a: in-use tag kept" has_ref "$NAV2_REF"
check "a: second tag of in-use image kept" has_ref "$OLD_REF"
check "a: in-use id kept" has_id "$NAV2_ID"
refute "a: in-use id not passed to rmi" rmi_logged "$NAV2_ID"
check "a: unused image removed" rmi_logged "$UNUSED_ID"
refute "a: unused image gone from state" has_id "$UNUSED_ID"
check "a: dangling image removed" rmi_logged "$DANGLING_ID"
refute "a: no tag restore when tag present" tag_logged "$NAV2_ID" "$NAV2_REF"

# Остановленный контейнер защищает тег так же, как running (ps -aq, не ps -q).
reset_state
add_image "$NAV2_ID" "$NAV2_REF"
add_image "$UNUSED_ID" "ghcr.io/krikz/rob_box/old-humble-dev-2222222"
add_container "c_stopped" "$NAV2_ID" "$NAV2_REF" "nav2"
export FAKE_COMPOSE_RC=1 FAKE_COMPOSE_IMAGES=""
check "a-stopped: exits 0" run_script
check "a-stopped: stopped container keeps tag" has_ref "$NAV2_REF"
refute "a-stopped: in-use id not removed" rmi_logged "$NAV2_ID"
check "a-stopped: unused still removed" rmi_logged "$UNUSED_ID"

# --- (б) неиспользуемый образ удаляется, даже если тегов несколько ---------
reset_state
add_image "$UNUSED_ID" "ghcr.io/krikz/rob_box/old-humble-dev-3333333"
add_image "$UNUSED_ID" "ghcr.io/krikz/rob_box/old-humble-dev-4444444"
export FAKE_COMPOSE_RC=1 FAKE_COMPOSE_IMAGES=""
check "b: exits 0 with only garbage" run_script
check "b: unused id removed once" rmi_logged "$UNUSED_ID"
refute "b: both tags of unused image gone" has_id "$UNUSED_ID"
check "b: rmi called once for the id" test "$(grep -cF "$UNUSED_ID" "$WORK/rmi.log")" -eq 1

# --- (в) --dry-run ничего не удаляет и не тегирует -------------------------
reset_state
add_image "$NAV2_ID" "$NAV2_REF"
add_image "$UNUSED_ID" "ghcr.io/krikz/rob_box/old-humble-dev-5555555"
add_container "c_nav2" "$NAV2_ID" "$NAV2_REF" "nav2"
export FAKE_COMPOSE_RC=0 FAKE_COMPOSE_IMAGES="$NAV2_REF"
before="$(cat "$WORK/images.tsv")"
check "c: dry-run exits 0" run_script --dry-run
check "c: state unchanged" test "$before" = "$(cat "$WORK/images.tsv")"
refute "c: dry-run did not rmi" test -s "$WORK/rmi.log"
refute "c: dry-run did not tag" test -s "$WORK/tag.log"
check "c: dry-run names the unused id" grep -q "$UNUSED_ID" "$WORK/out.txt"

# Пропавший тег в dry-run: сообщить, не восстанавливать.
reset_state
add_image "$OLD_ID" "$OLD_REF"
add_container "c_nav2" "$OLD_ID" "$NAV2_REF" "nav2"
export FAKE_COMPOSE_RC=0 FAKE_COMPOSE_IMAGES="$NAV2_REF"
check "c-missing: dry-run exits 0" run_script --dry-run
refute "c-missing: dry-run did not restore tag" has_ref "$NAV2_REF"
refute "c-missing: dry-run did not tag" test -s "$WORK/tag.log"
check "c-missing: dry-run says it would restore" grep -q "DRY-RUN: восстановил бы тег" "$WORK/out.txt"

# --- (г) пропавший тег восстанавливается из ID контейнера ------------------
# Картина деплоя №8: тег *-dev-acd5a8d снят, тот же sha лежит под старым
# тегом, контейнер ссылается на ID и помнит Config.Image.
reset_state
add_image "$OLD_ID" "$OLD_REF"
add_container "c_nav2" "$OLD_ID" "$NAV2_REF" "nav2"
export FAKE_COMPOSE_RC=0 FAKE_COMPOSE_IMAGES="$NAV2_REF"
check "d: exits 0" run_script
check "d: missing tag restored" has_ref "$NAV2_REF"
# shellcheck disable=SC2016  # $1/$2 — поля awk, не переменные shell
check "d: restored tag points at container image id" \
    awk -F '\t' -v id="$OLD_ID" -v ref="$NAV2_REF" '$1 == id && $2 == ref { found=1 } END { exit !found }' "$WORK/images.tsv"
check "d: old tag kept" has_ref "$OLD_REF"
check "d: tag log records restore" tag_logged "$OLD_ID" "$NAV2_REF"
check "d: WARN names the service" grep -q "сервис=nav2" "$WORK/out.txt"
check "d: WARN names the ref" grep -q "$NAV2_REF" "$WORK/out.txt"
refute "d: script did not pull" grep -q "compose pull" "$WORK/out.txt"

# Восстановленный тег снова проходит image inspect (контракт compose up).
check "d: image inspect of restored ref passes" \
    env FAKE_STATE="$WORK" PATH="$WORK/bin:$PATH" docker image inspect "$NAV2_REF"

# Контейнер без compose-метки: WARN всё равно называет ref.
reset_state
add_image "$OLD_ID" "$OLD_REF"
add_container "c_plain" "$OLD_ID" "$NAV2_REF" ""
export FAKE_COMPOSE_RC=1 FAKE_COMPOSE_IMAGES=""
check "d-nolabel: exits 0" run_script
check "d-nolabel: tag restored" has_ref "$NAV2_REF"
check "d-nolabel: WARN falls back to ref" grep -q "сервис=unknown" "$WORK/out.txt"

# --- best-effort: docker ps упал — ничего не удаляем, exit 0 --------------
reset_state
add_image "$UNUSED_ID" "ghcr.io/krikz/rob_box/old-humble-dev-6666666"
add_image "$NAV2_ID" "$NAV2_REF"
export FAKE_PS_RC=1 FAKE_COMPOSE_RC=1 FAKE_COMPOSE_IMAGES=""
check "e: ps failure exits 0" run_script
check "e: ps failure removed nothing" test ! -s "$WORK/rmi.log"
check "e: ps failure keeps unused image" has_id "$UNUSED_ID"
check "e: ps failure warns" grep -q "уборка пропущена" "$WORK/out.txt"
export FAKE_PS_RC=0

# --- workflow: три борта зовут скрипт, prune -af образов больше нет --------
CLEANUP_STEP="[Vision Pi + Sound Pi + Main Pi] Cleanup Docker Artifacts (parallel)"
if python3 - "$WORKFLOW" "$CLEANUP_STEP" << 'PY'
import re, sys
text = open(sys.argv[1], encoding="utf-8").read()
step = sys.argv[2]
parts = re.split(r"(?m)^      - name: ", text)[1:]
body = ""
for part in parts:
    name = part.splitlines()[0].strip().strip('"')
    if name == step:
        body = part
        break
errors = []
if not body:
    errors.append(f"шаг не найден: {step}")
else:
    code = [ln for ln in body.splitlines() if not ln.strip().startswith("#")]
    code_text = "\n".join(code)
    if "docker image prune" in code_text:
        errors.append("шаг всё ещё вызывает docker image prune")
    if code_text.count("docker_prune_safe.sh") < 3:
        errors.append(
            "docker_prune_safe.sh должен вызываться на трёх бортах, найдено %d"
            % code_text.count("docker_prune_safe.sh")
        )
    if code_text.count("docker builder prune -af") < 3:
        errors.append("docker builder prune -af должен остаться на трёх бортах")
    for host in ("env.vision_pi_ip", "env.sound_pi_ip", "env.main_pi_ip"):
        if host not in body:
            errors.append("очистка не ходит на %s" % host)
    if "Warning: Docker cleanup failed" not in body:
        errors.append("провал уборки больше не маскируется warning-ом")
if errors:
    print("\n".join(errors))
    sys.exit(1)
PY
then ok "workflow: safe prune on 3 Pi, image prune -af gone, builder prune kept"
else ko "workflow wiring"
fi

# bash -n самого скрипта (синтаксис, без демона).
check "script: bash -n" bash -n "$SCRIPT"

echo "----"
echo "passed=$pass failed=$failures"
[ "$failures" -eq 0 ]
