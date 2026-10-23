#!/usr/bin/env bash
# Прогон юнит-тестов src/rob_box_mcp_tools/test/ — тот же скрипт в CI
# (G-Run Tests.yml, job «Unit Tests (rob_box_mcp_tools)») и локально.
#
# Зачем отдельный скрипт: до него весь test/ пакета не гонял ни один
# workflow. G-Run Tests собирал пакеты без rob_box_mcp_tools, а G-Lint Code
# (PR #2899) подключил ровно один файл. test_dialogue_register_speaker.py
# и ещё ~60 файлов в CI не исполнялись вообще.
#
# Как гоняем:
#   * прямо из checkout'а, без копии в test_ws — тесты, которые ходят
#     вверх до корня репо (migrations/, sound_pack/, docker/, tools/),
#     видят настоящее дерево, и никакой rsync-список не надо поддерживать;
#   * каждый файл — отдельным pytest-процессом, как rob_box_quest и
#     rob_box_telegram. Файлы ставят стабы rclpy/std_msgs/rob_box_* в
#     sys.modules на уровне модуля, и одним процессом на весь test/ они
#     портят друг другу импорт (86 failed + 54 errors против 4 failed
#     пофайлово на одном и том же коммите);
#   * exit 5 (ничего не собрано) — тоже провал: файл, из которого не
#     собрался ни один тест, в CI выглядел бы зелёным, а это ровно та
#     «декорация», из-за которой задача и появилась;
#   * MCP_TOOLS_JOBS файлов одновременно (по умолчанию 4 — столько vCPU у
#     раннера; локально можно 1) через xargs -P, #3937. Изоляция та же:
#     процесс на файл, не xdist --dist loadfile (воркер xdist гоняет много
#     файлов в одном интерпретаторе, и загрязнение вернулось бы). Вывод
#     файла идёт в его лог и печатается целиком после прогона, в
#     алфавитном порядке, — параллельные логи не перемешиваются.
#
# Использование:
#   [MCP_TOOLS_JOBS=N] scripts/testing/run_mcp_tools_unit_tests.sh [каталог-для-junitxml]
#
# Нужно: python3 с pytest (>=8), pyyaml, httpx, openai, msgpack, aiohttp.
# ROS-пакеты не обязательны, кроме nav2_msgs/action_msgs — без них
# test_animation.py и test_llm_integration.py падают на коллекции
# (rob_box_mcp_tools.tools.__init__ тянет navigation.py).

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PKG_DIR="$REPO_ROOT/src/rob_box_mcp_tools"
JUNIT_DIR="${1:-}"
PER_FILE_TIMEOUT="${MCP_TOOLS_PER_FILE_TIMEOUT:-300}"

# Соседние пакеты, которые импортируют тесты и сам rob_box_mcp_tools:
# rob_box_harness (slice_authority, mcp_server), rob_box_voice (music,
# dialogue), rob_box_core (utterance), rob_box_llm (harness), rob_box_dialog
# (знание диалога: его импортируют rob_box_voice.core и DSM харнесса).
export PYTHONPATH="$PKG_DIR:$REPO_ROOT/src/rob_box_harness:$REPO_ROOT/src/rob_box_llm:$REPO_ROOT/src/rob_box_core:$REPO_ROOT/src/rob_box_music:$REPO_ROOT/src/rob_box_voice:$REPO_ROOT/src/rob_box_dialog${PYTHONPATH:+:$PYTHONPATH}"

if [ -n "$JUNIT_DIR" ]; then
  mkdir -p "$JUNIT_DIR"
  JUNIT_DIR="$(cd "$JUNIT_DIR" && pwd)"
fi

mapfile -t TEST_FILES < <(cd "$PKG_DIR" && find test -name 'test_*.py' -not -path '*/fixtures/*' | sort)
if [ "${#TEST_FILES[@]}" -eq 0 ]; then
  echo "❌ rob_box_mcp_tools: не найдено ни одного test_*.py в $PKG_DIR/test"
  exit 1
fi
echo "=== pytest: rob_box_mcp_tools (${#TEST_FILES[@]} файлов, per-file, ${MCP_TOOLS_JOBS:-4} параллельно, timeout ${PER_FILE_TIMEOUT}s) ==="

JOBS="${MCP_TOOLS_JOBS:-4}"
LOG_DIR="$(mktemp -d)"
trap 'rm -rf "$LOG_DIR"' EXIT
export PKG_DIR JUNIT_DIR PER_FILE_TIMEOUT LOG_DIR

# run_one <test/файл.py>: гонит один файл отдельным pytest-процессом, лог — в
# $LOG_DIR/<ключ>.log, код выхода — в $LOG_DIR/<ключ>.rc. Всегда возвращает 0,
# чтобы xargs не прерывал остальные файлы: решение по коду принимается ниже.
run_one() {
  local f="$1" key rc=0
  key="$(echo "$f" | tr '/' '_' | sed 's/\.py$//')"
  local junit_args=()
  if [ -n "$JUNIT_DIR" ]; then
    junit_args=(--junitxml "$JUNIT_DIR/$key.xml")
  fi
  # -o addopts="" — pytest.ini пакета включает --cov, а pytest-cov в
  # раннере не гарантирован; -p no:* — apt-плагины ROS, собранные под
  # pytest 6, роняют pytest 9. Глушим по ИМЕНАМ entry point'ов
  # (launch_testing, launch_ros), а не по именам пакетов: первый CI-прогон
  # (run 35963939649) упал INTERNALERROR на всех 62 файлах — plugin
  # launch_testing_ros_pytest_entrypoint зарегистрирован как «launch_ros»,
  # и -p no:launch_testing_ros его не отключал.
  ( cd "$PKG_DIR" && timeout "$PER_FILE_TIMEOUT" python3 -m pytest \
      -p no:launch_testing -p no:launch_ros \
      -p no:ament_flake8 -p no:ament_pep257 \
      -p no:ament_copyright -p no:ament_xmllint \
      -p no:ament_lint -p no:colcon_core \
      -p no:cacheprovider \
      -o addopts="" \
      --tb=short -ra -v --no-header \
      "${junit_args[@]}" \
      "$f" ) > "$LOG_DIR/$key.log" 2>&1 || rc=$?
  echo "$rc" > "$LOG_DIR/$key.rc"
  return 0
}
export -f run_one

printf '%s\0' "${TEST_FILES[@]}" | xargs -r -0 -n 1 -P "$JOBS" bash -c 'run_one "$1"' _

FAILED=()
for f in "${TEST_FILES[@]}"; do
  key="$(echo "$f" | tr '/' '_' | sed 's/\.py$//')"
  echo "::group::rob_box_mcp_tools: $f"
  cat "$LOG_DIR/$key.log"
  echo "::endgroup::"
  rc="$(cat "$LOG_DIR/$key.rc" 2>/dev/null || echo missing)"
  case "$rc" in
    0) echo "✅ $f" ;;
    5) echo "❌ $f — ни одного теста не собрано (exit 5)"; FAILED+=("$f (exit 5: no tests collected)") ;;
    124) echo "❌ $f — timeout ${PER_FILE_TIMEOUT}s"; FAILED+=("$f (timeout)") ;;
    *) echo "❌ $f — exit $rc"; FAILED+=("$f (exit $rc)") ;;
  esac
done

if [ -n "$JUNIT_DIR" ]; then
  python3 - "$JUNIT_DIR" <<'PY'
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

totals = dict(tests=0, failures=0, errors=0, skipped=0)
for xml in sorted(Path(sys.argv[1]).glob("*.xml")):
    root = ET.parse(xml).getroot()
    suites = [root] if root.tag == "testsuite" else list(root.iter("testsuite"))
    for s in suites:
        for k in totals:
            totals[k] += int(s.attrib.get(k, 0))
passed = totals["tests"] - totals["failures"] - totals["errors"] - totals["skipped"]
print(
    f"=== rob_box_mcp_tools итог по junitxml: {totals['tests']} tests, "
    f"{passed} passed, {totals['failures']} failed, {totals['errors']} errors, "
    f"{totals['skipped']} skipped/xfailed ==="
)
PY
fi

if [ "${#FAILED[@]}" -ne 0 ]; then
  echo "❌ rob_box_mcp_tools: упало ${#FAILED[@]} из ${#TEST_FILES[@]} файлов:"
  printf '   - %s\n' "${FAILED[@]}"
  exit 1
fi
echo "✅ rob_box_mcp_tools: все ${#TEST_FILES[@]} файлов зелёные"
