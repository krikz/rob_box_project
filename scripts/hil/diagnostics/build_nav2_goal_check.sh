#!/usr/bin/env bash
# Build the HIL probe in the same Nav2 image that will load it.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
COMMIT=a097086719c88f781aa59788eca29ac6ca5e56db # Same pin as docker/main/nav2/Dockerfile.
: "${M1_GOAL_PROBE_DIR:?Set a fresh external workspace}"
: "${M1_GOAL_PROBE_IMAGE:?Set the local Nav2 image or image ID}"
image_id=$(docker image inspect -f '{{.Id}}' "$M1_GOAL_PROBE_IMAGE")
mkdir -- "$M1_GOAL_PROBE_DIR"
workspace="$(cd "$M1_GOAL_PROBE_DIR" && pwd)"
source="$workspace/navigation2-$COMMIT"
curl -fsSL "https://github.com/ros-navigation/navigation2/archive/$COMMIT.tar.gz" \
    | tar -xz -C "$workspace"
for patch in "$ROOT/docker/main/nav2/controller_goal_tf_fail.patch" \
    "$ROOT/docker/main/nav2/controller_progress_abort.patch" \
    "$ROOT/scripts/hil/diagnostics/nav2_goal_check.patch"; do
    patch -d "$source" -p1 <"$patch"
done

docker run --rm --pull never --network none --cpus=1 --memory=1300m --user "$(id -u):$(id -g)" \
    --mount "type=bind,src=$workspace,dst=/work" --workdir /work --entrypoint /bin/bash "$image_id" \
    -c 'set -eo pipefail
        source /opt/ros/humble/setup.bash
        cmake -S "/work/navigation2-$1/nav2_controller" -B /work/build -DCMAKE_BUILD_TYPE=Release
        cmake --build /work/build --target controller_server_core --parallel 1
        cp /work/build/libcontroller_server_core.so /work/libcontroller_server_core.so
        ldd -r /work/libcontroller_server_core.so > /work/ldd.log
        if grep -E "not found|undefined symbol" /work/ldd.log; then exit 1; fi' _ "$COMMIT"
printf '%s\n' "$image_id" >"$workspace/nav2_image_id"
(cd "$workspace" && sha256sum libcontroller_server_core.so >probe.sha256)
printf 'M1_GOAL_PROBE_DIR=%s\n' "$workspace"
