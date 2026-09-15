#!/usr/bin/env bash
# test_yaml_loader_env_override.sh — regression test for F-2.
#
# Проверяет контракт YAML-loader'а в start_vision_hailo.sh:
#   1. Если ENV KEY непусто → YAML SKIP'ается с WARN (ENV wins).
#   2. Если ENV KEY пусто/unset → YAML применяется (YAML wins as default).
#   3. Несколько ключей: каждый проверяется независимо.
#
# Usage:
#   bash scripts/testing/test_yaml_loader_env_override.sh
#
# Exit codes:
#   0 — все проверки прошли.
#   1 — хотя бы одна проверка провалилась.
#
# Это регрессионный тест для issue #2504 (F-2 from t_beba0869 review).
# Запускать на любой машине с bash + python3 + PyYAML.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SCRIPT_UNDER_TEST="${REPO_ROOT}/docker/vision/scripts/vision-hailo/start_vision_hailo.sh"

if [ ! -f "${SCRIPT_UNDER_TEST}" ]; then
    echo "FAIL: ${SCRIPT_UNDER_TEST} не найден."
    exit 1
fi

# Проверяем наличие python3 + yaml.
if ! command -v python3 >/dev/null 2>&1; then
    echo "SKIP: python3 не доступен — тест нельзя запустить."
    exit 0
fi
if ! python3 -c "import yaml" 2>/dev/null; then
    echo "SKIP: PyYAML не установлен — тест нельзя запустить."
    exit 0
fi

PASS=0
FAIL=0
TOTAL=0

pass() {
    TOTAL=$((TOTAL + 1))
    PASS=$((PASS + 1))
    echo "  PASS: $1"
}

fail() {
    TOTAL=$((TOTAL + 1))
    FAIL=$((FAIL + 1))
    echo "  FAIL: $1 — $2"
}

# Извлекаем YAML-loader-блок из start_vision_hailo.sh как функцию.
# Блок между "if [ -f \"\${HAILO_MODELS_YAML}\" ]..." и "fi" — это eval $().
# Для тестирования мы выносим его в отдельный скрипт, который принимает YAML path.
EXTRACT_BLOCK='
import os, sys, yaml
try:
    with open(sys.argv[1]) as f:
        cfg = yaml.safe_load(f) or {}
except Exception as exc:
    print(f"echo \"[start_vision_hailo] WARN: yaml parse failed: {exc}\" >&2")
    sys.exit(0)
node = (cfg.get("vision_hailo_node") or {})
def emit(k, v):
    if isinstance(v, bool):
        v = "true" if v else "false"
    print(f"export {k.upper()}=\"{v}\"")
for key in ("hailo_enabled", "hef_path", "stub_period_sec",
            "confidence_threshold", "input_topic", "output_topic"):
    if key not in node:
        continue
    env_name = key.upper()
    env_val = os.environ.get(env_name, "")
    if env_val:
        print(
            f"[start_vision_hailo] ENV override wins: {env_name} "
            f"(env=\"{env_val}\", yaml=\"{node[key]}\")",
            file=sys.stderr,
        )
        continue
    emit(key, node[key])
'

WORKDIR="$(mktemp -d)"
trap 'rm -rf "${WORKDIR}"' EXIT

# Helper: создать fixture YAML и запустить extract_block с заданным env.
run_block() {
    local label="$1"
    local yaml_content="$2"
    # env pairs: KEY=val KEY=val ...
    shift 2

    local yaml_file="${WORKDIR}/cfg-${RANDOM}.yaml"
    printf '%s\n' "${yaml_content}" > "${yaml_file}"

    local result
    result="$(env "$@" bash -c "
        eval \"\$(python3 - '${yaml_file}' <<'PY'
${EXTRACT_BLOCK}
PY
        )\"
        echo \"HAILO_ENABLED=\${HAILO_ENABLED-<<unset>>}\"
        echo \"HEF_PATH=\${HEF_PATH-<<unset>>}\"
        echo \"STUB_PERIOD_SEC=\${STUB_PERIOD_SEC-<<unset>>}\"
        echo \"CONFIDENCE_THRESHOLD=\${CONFIDENCE_THRESHOLD-<<unset>>}\"
        echo \"INPUT_TOPIC=\${INPUT_TOPIC-<<unset>>}\"
        echo \"OUTPUT_TOPIC=\${OUTPUT_TOPIC-<<unset>>}\"
    " 2>&1 1>/tmp/block-stdout.$$)"

    # Capture stderr separately.
    local stderr
    stderr="$(cat /tmp/block-stdout.$$ 2>/dev/null || echo "")"
    rm -f /tmp/block-stdout.$$

    echo "${result}"
    echo "STDERR:"
    echo "${stderr}"
}

assert_value() {
    local output="$1"
    local key="$2"
    local expected="$3"
    local got
    got="$(printf '%s\n' "${output}" | grep "^${key}=" | head -1 | cut -d= -f2-)"
    if [ "${got}" = "${expected}" ]; then
        pass "${key} == ${expected}"
    else
        fail "${key}" "expected '${expected}', got '${got}'"
    fi
}

assert_warn() {
    local stderr="$1"
    local key="$2"
    if printf '%s\n' "${stderr}" | grep -q "ENV override wins: ${key}"; then
        pass "WARN emitted for ${key}"
    else
        fail "WARN for ${key}" "expected 'ENV override wins: ${key}', stderr='${stderr}'"
    fi
}

assert_no_warn() {
    local stderr="$1"
    local key="$2"
    if printf '%s\n' "${stderr}" | grep -q "ENV override wins: ${key}"; then
        fail "no WARN for ${key}" "found unexpected WARN, stderr='${stderr}'"
    else
        pass "no WARN for ${key}"
    fi
}

# ---------- Test cases ----------

echo "=== Test 1: ENV not set, YAML provides defaults ==="
YAML='vision_hailo_node:
  hailo_enabled: false
  hef_path: ""
  stub_period_sec: 2.0
  confidence_threshold: 0.5
  input_topic: /yaml/input
  output_topic: /yaml/output'
out="$(env -i bash -c "
    eval \"\$(python3 - '/dev/stdin' '${WORKDIR}/dummy.yaml' <<'PY'
${EXTRACT_BLOCK}
PY
    )\"
    echo \"HAILO_ENABLED=\${HAILO_ENABLED-<<unset>>}\"
    echo \"INPUT_TOPIC=\${INPUT_TOPIC-<<unset>>}\"
" 2>/tmp/e1.stderr)"

printf '%s\n' "${YAML}" > "${WORKDIR}/dummy.yaml"
# Re-run with the actual file (heredoc trick doesn't allow both - and stdin file).
out="$(env -i bash -c "
    eval \"\$(python3 - '${WORKDIR}/dummy.yaml' <<'PY'
${EXTRACT_BLOCK}
PY
    )\"
    echo \"HAILO_ENABLED=\${HAILO_ENABLED-<<unset>>}\"
    echo \"HEF_PATH=\${HEF_PATH-<<unset>>}\"
    echo \"STUB_PERIOD_SEC=\${STUB_PERIOD_SEC-<<unset>>}\"
    echo \"CONFIDENCE_THRESHOLD=\${CONFIDENCE_THRESHOLD-<<unset>>}\"
    echo \"INPUT_TOPIC=\${INPUT_TOPIC-<<unset>>}\"
    echo \"OUTPUT_TOPIC=\${OUTPUT_TOPIC-<<unset>>}\"
" 2>/tmp/e1.stderr)"
stderr="$(cat /tmp/e1.stderr)"

assert_value "${out}" "HAILO_ENABLED" "false"
assert_value "${out}" "INPUT_TOPIC" "/yaml/input"
assert_value "${out}" "STUB_PERIOD_SEC" "2.0"
assert_no_warn "${stderr}" "HAILO_ENABLED"

echo
echo "=== Test 2: F-2 scenario — ENV=HAILO_ENABLED=true, YAML=hailo_enabled:false ==="
out="$(env -i HAILO_ENABLED=true bash -c "
    eval \"\$(python3 - '${WORKDIR}/dummy.yaml' <<'PY'
${EXTRACT_BLOCK}
PY
    )\"
    echo \"HAILO_ENABLED=\${HAILO_ENABLED-<<unset>>}\"
    echo \"INPUT_TOPIC=\${INPUT_TOPIC-<<unset>>}\"
" 2>/tmp/e2.stderr)"
stderr="$(cat /tmp/e2.stderr)"

assert_value "${out}" "HAILO_ENABLED" "true"
assert_warn "${stderr}" "HAILO_ENABLED"

echo
echo "=== Test 3: partial ENV override (only INPUT_TOPIC set) ==="
out="$(env -i INPUT_TOPIC=/env/input bash -c "
    eval \"\$(python3 - '${WORKDIR}/dummy.yaml' <<'PY'
${EXTRACT_BLOCK}
PY
    )\"
    echo \"HAILO_ENABLED=\${HAILO_ENABLED-<<unset>>}\"
    echo \"INPUT_TOPIC=\${INPUT_TOPIC-<<unset>>}\"
    echo \"OUTPUT_TOPIC=\${OUTPUT_TOPIC-<<unset>>}\"
" 2>/tmp/e3.stderr)"
stderr="$(cat /tmp/e3.stderr)"

assert_value "${out}" "INPUT_TOPIC" "/env/input"
assert_value "${out}" "OUTPUT_TOPIC" "/yaml/output"  # YAML provides because ENV unset.
assert_value "${out}" "HAILO_ENABLED" "false"  # YAML provides because ENV unset.
assert_warn "${stderr}" "INPUT_TOPIC"
assert_no_warn "${stderr}" "OUTPUT_TOPIC"
assert_no_warn "${stderr}" "HAILO_ENABLED"

echo
echo "=== Test 4: empty string in ENV for HAILO_ENABLED — treated as 'unset' (YAML wins) ==="
# Note: bash semantics — empty string IS a value, but our contract says
# "non-empty" wins. python sees os.environ.get('HAILO_ENABLED', '') = ''.
out="$(env -i HAILO_ENABLED= bash -c "
    eval \"\$(python3 - '${WORKDIR}/dummy.yaml' <<'PY'
${EXTRACT_BLOCK}
PY
    )\"
    echo \"HAILO_ENABLED=\${HAILO_ENABLED-<<unset>>}\"
" 2>/tmp/e4.stderr)"
stderr="$(cat /tmp/e4.stderr)"

# After bash's ${HAILO_ENABLED:-false} default-fill in the script (line 23),
# HAILO_ENABLED will be 'false' — which IS non-empty for python.
# But this test runs the EXTRACT_BLOCK in isolation, NOT the full script.
# In isolation, env_val='' → YAML wins → result is 'false' (from YAML).
assert_value "${out}" "HAILO_ENABLED" "false"
assert_no_warn "${stderr}" "HAILO_ENABLED"

echo
echo "=== Test 5: full script integration — HAILO_ENABLED=true wins over YAML=false ==="
# Build a shim of the script that hardcodes the fake setup.bash paths.
mkdir -p "${WORKDIR}/fake-ws/install" "${WORKDIR}/fake-ros/opt/ros/humble"
: > "${WORKDIR}/fake-ws/install/setup.bash"
: > "${WORKDIR}/fake-ros/opt/ros/humble/setup.bash"

sed -e "s|/ws/install/setup.bash|${WORKDIR}/fake-ws/install/setup.bash|g" \
    -e "s|/opt/ros/\${ROS_DISTRO:-humble}/setup.bash|${WORKDIR}/fake-ros/opt/ros/humble/setup.bash|g" \
    "${SCRIPT_UNDER_TEST}" > "${WORKDIR}/shim.sh"
chmod +x "${WORKDIR}/shim.sh"

# Test fixture: dev YAML.
cp "${WORKDIR}/dummy.yaml" "${WORKDIR}/fixture.yaml"

# Run with HAILO_ENABLED=true in ENV.
out="$(HAILO_ENABLED=true \
       HAILO_MODELS_YAML="${WORKDIR}/fixture.yaml" \
       HEF_PATH=/opt/rob_box/models/yolov8n.hef \
       bash "${WORKDIR}/shim.sh" 2>&1 || true)"

# The script will fail at "exec ros2 run" (no ros2 in PATH), so we capture
# everything up to that point. Look for the summary line.
if printf '%s\n' "${out}" | grep -q "ENV override wins: HAILO_ENABLED"; then
    pass "full script: ENV-wins WARN for HAILO_ENABLED"
else
    fail "full script ENV-wins" "WARN line not found in output"
fi
if printf '%s\n' "${out}" | grep -q "config: HAILO_ENABLED=true"; then
    pass "full script: summary shows HAILO_ENABLED=true"
else
    fail "full script summary" "expected 'HAILO_ENABLED=true' in summary, got: $(printf '%s\n' "${out}" | grep config)"
fi

# ---------- Summary ----------

echo
echo "================================"
echo "Tests: ${TOTAL}  Passed: ${PASS}  Failed: ${FAIL}"
echo "================================"

if [ "${FAIL}" -gt 0 ]; then
    exit 1
fi
exit 0
