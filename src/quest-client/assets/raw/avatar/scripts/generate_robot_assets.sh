#!/usr/bin/env bash
# Генерация robot_body.glb + robot_animations.glb + smoke-test.
#
# Запуск:
#   ./generate_robot_assets.sh           # полный pipeline
#   ./generate_robot_assets.sh body      # только body
#   ./generate_robot_assets.sh anim      # только animations (требует body)
#   ./generate_robot_assets.sh test      # только smoke-test
#
# Требования:
#   - python3 + venv с trimesh/pygltflib/numpy/fast-simplification
#   - STL-меши в ../meshes/ (скопированы из rob_box_description)
#
# После Phase 2.0: добавится шаг gltf-transform (Draco + Meshopt + KTX2).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AVATAR_DIR="$(dirname "$SCRIPT_DIR")"
PY="${PY:-python3}"

step() { printf "\n=== %s ===\n" "$1"; }

run_body() {
  step "1/2: build robot_body.glb"
  "$PY" "$SCRIPT_DIR/generate_robot_body.py" --mesh-dir "$AVATAR_DIR/meshes" \
      --out "$AVATAR_DIR/robot_body.glb"
}

run_anim() {
  step "2/2: build robot_animations.glb"
  "$PY" "$SCRIPT_DIR/generate_robot_animations.py" \
      --body "$AVATAR_DIR/robot_body.glb" \
      --out "$AVATAR_DIR/robot_animations.glb"
}

run_test() {
  step "smoke-test"
  "$PY" "$SCRIPT_DIR/smoke_test.py" --dir "$AVATAR_DIR"
}

case "${1:-all}" in
  all)
    run_body
    run_anim
    run_test
    ;;
  body)
    run_body
    ;;
  anim)
    run_anim
    ;;
  test)
    run_test
    ;;
  *)
    echo "Usage: $0 [all|body|anim|test]"
    exit 1
    ;;
esac

echo
echo "Artifacts:"
ls -la "$AVATAR_DIR"/robot_body.glb "$AVATAR_DIR"/robot_animations.glb 2>/dev/null || true
