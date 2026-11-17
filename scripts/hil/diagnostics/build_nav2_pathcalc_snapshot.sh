#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
COMMIT=a097086719c88f781aa59788eca29ac6ca5e56db
WORKSPACE="${PATHCALC_DIAG_WS:-/home/ros2/rob_box_hil/nav2_diag_4682_snapshot}"
SOURCE="$WORKSPACE/src/navigation2"
PACKAGE="$SOURCE/nav2_navfn_planner"

if [ -e "$WORKSPACE" ]; then
    echo "Refusing existing PATHCALC_DIAG_WS=$WORKSPACE; choose a fresh external workspace" >&2
    exit 2
fi
mkdir -p "$WORKSPACE/src"
curl -fsSL "https://github.com/ros-navigation/navigation2/archive/$COMMIT.tar.gz" | tar -xz -C "$WORKSPACE/src"
mv "$WORKSPACE/src/navigation2-$COMMIT" "$SOURCE"
cp "$ROOT/scripts/hil/diagnostics/pathcalc_snapshot.hpp" "$PACKAGE/include/nav2_navfn_planner/"
patch -d "$SOURCE" -p1 <"$ROOT/scripts/hil/diagnostics/nav2_pathcalc_snapshot.patch"

set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
set -u
cd "$WORKSPACE"
colcon build --symlink-install --base-paths "$PACKAGE" --packages-select nav2_navfn_planner
printf 'PATHCALC_DIAG_LIB=%s\n' "$WORKSPACE/install/nav2_navfn_planner/lib/libnav2_navfn_planner.so"
