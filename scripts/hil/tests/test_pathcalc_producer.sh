#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
COMMIT=a097086719c88f781aa59788eca29ac6ca5e56db
UPSTREAM="$WORK/upstream"
PACKAGE="$UPSTREAM/nav2_navfn_planner"
mkdir -p "$PACKAGE/src" "$PACKAGE/include/nav2_navfn_planner" "$WORK/stub/rclcpp"

for file in src/navfn.cpp src/navfn_planner.cpp include/nav2_navfn_planner/navfn.hpp; do
    curl -fsSL "https://raw.githubusercontent.com/ros-navigation/navigation2/$COMMIT/nav2_navfn_planner/$file" \
        -o "$PACKAGE/$file"
done
patch -d "$UPSTREAM" -p1 <"$ROOT/scripts/hil/diagnostics/nav2_pathcalc_snapshot.patch"
patch -d "$UPSTREAM" -p1 <"$ROOT/docker/main/nav2/navfn_gradient_origin.patch"
cp "$ROOT/scripts/hil/diagnostics/pathcalc_snapshot.hpp" "$PACKAGE/include/nav2_navfn_planner/"

cat >"$WORK/stub/rclcpp/rclcpp.hpp" <<'EOF'
#ifndef RCLCPP_RCLCPP_HPP_
#define RCLCPP_RCLCPP_HPP_
namespace rclcpp { struct Logger {}; inline Logger get_logger(const char *) { return {}; } }
#define RCLCPP_DEBUG(...) ((void)0)
#define RCLCPP_WARN(...) ((void)0)
#endif
EOF

g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
    -I"$WORK/stub" -I"$PACKAGE/include" \
    "$PACKAGE/src/navfn.cpp" "$ROOT/scripts/hil/tests/pathcalc_producer_test.cpp" \
    -o "$WORK/pathcalc_producer_test"
"$WORK/pathcalc_producer_test" "$WORK"
gzip -dc "$ROOT/scripts/hil/tests/fixtures/4682_pathcalc_snapshot.bin.gz" >"$WORK/4682_snapshot.bin"
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
    -I"$WORK/stub" -I"$PACKAGE/include" \
    "$PACKAGE/src/navfn.cpp" "$ROOT/scripts/hil/tests/pathcalc_snapshot_replay_test.cpp" \
    -o "$WORK/pathcalc_snapshot_replay_test"
"$WORK/pathcalc_snapshot_replay_test" "$WORK/4682_snapshot.bin"
grep -q 'costmap_->getSizeInCellsX() \* 4' "$PACKAGE/src/navfn_planner.cpp"
grep -q 'costmap_->getSizeInCellsY() \* 4' "$PACKAGE/src/navfn_planner.cpp"
echo 'PASS: pinned NavFn patch applies; max_cycles formula unchanged'
bash -n "$ROOT/scripts/hil/diagnostics/build_nav2_pathcalc_snapshot.sh"
grep -q "$COMMIT" "$ROOT/scripts/hil/diagnostics/build_nav2_pathcalc_snapshot.sh"
echo 'PASS: external build script syntax and pinned commit'
