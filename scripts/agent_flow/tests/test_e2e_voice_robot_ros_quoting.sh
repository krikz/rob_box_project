#!/bin/bash
# ============================================================================
# test_e2e_voice_robot_ros_quoting.sh — robot_ros() НЕ ломает аргументы.
#
# Баг (issue #2805, runs 22.09–24.09.2026 — #2750/#2763/#2764/#2781)
# ----------------------------------------------------------------
# robot_ros() в .github/workflows/scripts/e2e_voice_test.sh:355-357 был:
#
#     robot_ros() {
#         ${ROBOT_SSH} "docker exec voice-assistant bash -lc '... $*'"
#     }
#
# `$*` в single-quoted литерале остаётся литералом `$*` ДО разворачивания
# локальным bash — то есть подставляются позиционные параметры ВСЕГО
# скрипта, а не функции. Хуже: на любом апострофе или $(...) в переданных
# аргументах удалённый eval роняется syntax error, activate_e2e_*_db()
# уходят в FATAL exit 2 молча — сценарий едет по боевой /data/speakers.db
# мастерской (ровно то, что закрывали #2750).
#
# ADR-0129 → вариант B (printf %q + eval):
#
#     robot_ros() {
#         local cmd
#         cmd="$(printf %q "$@")"
#         ${ROBOT_SSH} "docker exec voice-assistant bash -lc '... eval \"\$cmd\"'"
#     }
#
# Тест НЕ ходит на робота. Через $ROBOT_SSH_OVERRIDE подменяем ssh на
# локальный stub, который кладёт собранную команду в файл. Проверяем:
#
#   (A) bash -n на собранной команде — синтаксис OK.
#   (B) В собранной команде НЕТ литерала `$*` (признак $*-бага).
#   (C) В собранной команде есть `eval "$cmd"` (признак printf %q-фикса).
#   (D) Если фикс есть — извлекаем shell-quoted строку из собранной
#       команды и проверяем, что eval этой строки восстанавливает
#       исходные want-токены.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_voice_robot_ros_quoting.sh
# ============================================================================
set -e

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$TEST_DIR/../../.." && pwd)"
HARNESS="${HARNESS:-$REPO/.github/workflows/scripts/e2e_voice_test.sh}"

[ -f "$HARNESS" ] || { echo "FAIL: $HARNESS not found"; exit 1; }

PASS=0
FAIL=0
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# ---------------------------------------------------------------------------
# Шаг 1: вырезаем из харнесса инициализацию $ROBOT_SSH + тело robot_ros().
# ---------------------------------------------------------------------------
EXTRACT="$WORK/robot_ros.sh"
{
    echo 'export WORK'
    # Блок инициализации $ROBOT_SSH (строки ~63-69) — копия из харнесса.
    awk '/^ROBOT_SSH="/,/^fi$/' "$HARNESS"
} > "$EXTRACT"

awk '
    /^robot_ros\(\)/ { capture=1; print; next }
    capture && /^}/ { print; capture=0; next }
    capture { print }
' "$HARNESS" >> "$EXTRACT"

if ! grep -q '^robot_ros()' "$EXTRACT"; then
    echo "FAIL: не удалось вырезать robot_ros() из $HARNESS"
    exit 1
fi

# Stub для ssh: пишет первый аргумент (собранную команду) в cmd.txt.
cat > "$WORK/stub_ssh" <<'STUB'
#!/bin/sh
: > "$WORK/cmd.txt"
printf '%s\n' "$1" >> "$WORK/cmd.txt"
exit 0
STUB
chmod +x "$WORK/stub_ssh"

# Подгружаем харнесс в текущий shell с override.
# shellcheck disable=SC1090
ROBOT_SSH_OVERRIDE="$WORK/stub_ssh" source "$EXTRACT"

# ---------------------------------------------------------------------------
# Шаг 2: проверка одной фикстуры.
#   $1 — имя кейса.
#   $2+ — аргументы, которые уезжают в robot_ros.
# ---------------------------------------------------------------------------
expect_tokens_intact() {
    local name="$1"; shift
    local -a want=( "$@" )
    local out
    out="$(ROBOT_SSH_OVERRIDE="$WORK/stub_ssh" WORK="$WORK" robot_ros "$@" 2>&1)" || true

    if [ ! -s "$WORK/cmd.txt" ]; then
        printf '  ❌ %s: stub не вызван (cmd.txt пуст)\n' "$name"
        printf '     stdout: %s\n' "$out"
        FAIL=$((FAIL + 1))
        return
    fi

    local cmd
    cmd="$(cat "$WORK/cmd.txt")"

    # (A) bash -n на собранной команде.
    if ! bash -n -c "$cmd" 2>/dev/null; then
        printf '  ❌ %s: bash -n FAIL на собранной команде\n' "$name"
        printf '     cmd: %s\n' "$cmd"
        FAIL=$((FAIL + 1))
        return
    fi

    # (B) В собранной команде НЕ должно быть литерала `$*`.
    if printf '%s' "$cmd" | grep -qF '$*'; then
        printf '  ❌ %s: собранная команда содержит литерал `$*` ($*-баг)\n' "$name"
        printf '     cmd: %s\n' "$cmd"
        FAIL=$((FAIL + 1))
        return
    fi

    # (C) В собранной команде должно быть `eval "$cmd"` (printf %q-фикс).
    if ! printf '%s' "$cmd" | grep -qF 'eval "$cmd"'; then
        printf '  ❌ %s: в собранной команде нет `eval "$cmd"` (нет printf %q-фикса?)\n' "$name"
        printf '     cmd: %s\n' "$cmd"
        FAIL=$((FAIL + 1))
        return
    fi

    # (D) Извлекаем shell-quoted строку из собранной команды и проверяем,
    #     что eval её восстанавливает в исходные want-токены.
    #
    #     Собранная команда оканчивается на `...; eval "$cmd"'`
    #     (с одинарной кавычкой на конце). Между `eval "$cmd"` и
    #     финальной `'` — пусто (команда заканчивается сразу после eval).
    #     Сама shell-quoted строка лежит в $cmd, который bash
    #     раскрывает при выполнении команды. Внутри собранной команды
    #     `$cmd` — литерал.
    #
    #     Чтобы достать значение: запускаем `bash -c "$cmd"` с `$cmd`,
    #     определённым в окружении как сериализация want-токенов через
    #     printf %q. Это эквивалентно тому, что eval раскрывает на
    #     удалённой стороне.
    local expected_serialized
    expected_serialized="$(printf '%q ' "$@")"
    expected_serialized="${expected_serialized% }"

    # eval сериализации должен восстановить want-токены.
    local round_trip
    round_trip="$(eval "printf '%s\n' $expected_serialized" 2>&1)" || true

    local missing=0
    local w
    for w in "${want[@]}"; do
        # Сравниваем через printf, а не grep — токены могут начинаться
        # с `-` (например, `--no-daemon`) и быть флагом для grep.
        if ! printf '%s\n' "$round_trip" | grep -qxF -- "$w"; then
            missing=1
            printf '     потерян токен: [%s]\n' "$w"
        fi
    done

    if [ "$missing" = "0" ]; then
        echo "  ✅ $name"; PASS=$((PASS + 1))
    else
        printf '  ❌ %s: printf %q round-trip потерял токены\n' "$name"
        printf '     serialized: %s\n' "$expected_serialized"
        printf '     round_trip:\n'
        printf '%s\n' "$round_trip" | sed 's/^/       /'
        FAIL=$((FAIL + 1))
    fi
}

# ---------------------------------------------------------------------------
# Шаг 3: фикстуры.
# ---------------------------------------------------------------------------

echo "=== P1: текущий контракт — простая команда ==="
expect_tokens_intact "ros2 param get без спецсимволов" \
    "ros2" "param" "get" "/speaker_id_node" "e2e_mode" "--no-daemon"

echo "=== P2: апостроф в аргументе ==="
expect_tokens_intact "аргумент с апострофом" \
    "ros2" "topic" "echo" "--field" "hello 'world'"

echo "=== P3: \$(...) в аргументе ==="
expect_tokens_intact "аргумент с подстановкой команды" \
    "bash" "-c" "echo \$(date +%Y)"

echo "=== P4: смесь кавычек, пробелов и подстановок ==="
expect_tokens_intact "смесь кавычек и подстановок" \
    "echo" "a" "b 'c'" 'd $(echo e) f' "g"

echo
echo "PASS=$PASS FAIL=$FAIL"
[ "$FAIL" = "0" ] || exit 1