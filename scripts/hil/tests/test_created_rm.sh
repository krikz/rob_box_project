#!/usr/bin/env bash
# Unit-тест created_containers_to_rm из hil.sh: какие ID удалять перед recreate.
# Фиктивный docker ps → функция; Up не трогаем. Запуск: bash scripts/hil/tests/test_created_rm.sh
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
HIL="$REPO_ROOT/scripts/hil/hil.sh"

pass=0
failures=0
ok() { echo "PASS: $1"; pass=$((pass + 1)); }
ko() { echo "FAIL: $1"; failures=$((failures + 1)); }
check() { if eval "$2"; then ok "$1"; else ko "$1"; fi; }

# Вытащить чистую функцию из hil.sh (без запуска case).
eval "$(sed -n '/^created_containers_to_rm()/,/^}/p' "$HIL")"
type created_containers_to_rm >/dev/null 2>&1 \
    || { echo "FAIL: created_containers_to_rm не найдена в $HIL"; exit 1; }

MAIN_SIM_TIME=(robot-state-publisher twist-mux proximity-guard followpath-watchdog rtabmap imu-filter nav2)

# Фикстура «docker ps -a --format '{{.ID}} {{.Names}} {{.Status}}'»
PS_LINES='aaa111 abe6c83d1f0a_imu-filter Created
bbb222 imu-filter Up 3 hours
ccc333 main-nav2-1 Created
ddd444 orphan_other Created
eee555 abe6c83d1f0a_nav2 Exited (0) 2 minutes ago
fff666 proximity-guard Created
ggg777 hash_twist-mux Up 1 second
hhh888 project-rtabmap-2 Created
iii999 followpath-watchdog Created'

got=$(printf '%s\n' "$PS_LINES" | created_containers_to_rm "${MAIN_SIM_TIME[@]}")
got_sorted=$(printf '%s\n' "$got" | sort | tr '\n' ' ')
got_sorted=${got_sorted% }

check "Created hash_imu-filter → rm aaa111" 'printf "%s\n" "$got" | grep -qx aaa111'
check "Up imu-filter → не rm bbb222" '! printf "%s\n" "$got" | grep -qx bbb222'
check "Created main-nav2-1 (v2 replica) → rm ccc333" 'printf "%s\n" "$got" | grep -qx ccc333'
check "Created чужой orphan_other → не rm" '! printf "%s\n" "$got" | grep -qx ddd444'
check "Exited hash_nav2 → не rm (только Created)" '! printf "%s\n" "$got" | grep -qx eee555'
check "Created proximity-guard (точное имя) → rm fff666" 'printf "%s\n" "$got" | grep -qx fff666'
check "Up hash_twist-mux → не rm ggg777" '! printf "%s\n" "$got" | grep -qx ggg777'
check "Created project-rtabmap-2 → rm hhh888" 'printf "%s\n" "$got" | grep -qx hhh888'
check "Created followpath-watchdog → rm iii999" 'printf "%s\n" "$got" | grep -qx iii999'
check "ровно 5 id: aaa111 ccc333 fff666 hhh888 iii999" \
    '[ "$got_sorted" = "aaa111 ccc333 fff666 hhh888 iii999" ]'

# Пустой ввод / нет Created — пустой stdout
empty=$(printf '' | created_containers_to_rm "${MAIN_SIM_TIME[@]}")
check "пустой ps → ничего не удалять" '[ -z "$empty" ]'
only_up=$(printf 'x1 nav2 Up 5 minutes\nx2 imu-filter Up Less than a second\n' \
    | created_containers_to_rm "${MAIN_SIM_TIME[@]}")
check "только Up → ничего не удалять" '[ -z "$only_up" ]'

# Сервис не из списка — не трогаем даже Created
not_list=$(printf 'z1 abe6c83d1f0a_ros2-control Created\n' | created_containers_to_rm "${MAIN_SIM_TIME[@]}")
check "Created ros2-control (не MAIN_SIM_TIME) → не rm" '[ -z "$not_list" ]'

echo
echo "итог: $pass PASS, $failures FAIL"
[ "$failures" -eq 0 ]
