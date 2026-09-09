#!/bin/bash
# ============================================================================
# test_workflow_no_scp_dotrex.sh — bug(workflow #1478) ретро 19.08
#
# Регрессия: `scp -r /path/. /target/` — deprecated синтаксис OpenSSH 9.0+,
# который runner'ы GitHub Actions на ubuntu-24.04 режут как
# `error: unexpected filename: .` на каждой итерации.
#
# Live evidence: run #32255219770 (round-162, job «Collect e2e artifacts from
# build machine») — 80+ ошибок за 25 секунд, артефакты НЕ скопированы,
# upload-artifact получил пустые папки → «No files were found».
#
# Acceptance:
#   - grep по .github/workflows/*.yml не находит `scp` + аргумент
#     в кавычках с трейлинг-`/.` (т.е. литерал `/."`)
#   - lock-in: добавление запрещённого паттерна в фиктивный файл
#     детектируется (используется sandbox-каталог, в репо ничего не пишем)
#   - lock-in: rsync-стиль НЕ триггерит детектор (false positive = 0)
#
# Run:
#   bash scripts/agent_flow/tests/test_workflow_no_scp_dotrex.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
WORKFLOWS_DIR="$REPO_ROOT/.github/workflows"

# --- Helpers ---------------------------------------------------------------
TESTS_TOTAL=0
TESTS_PASS=0
TESTS_FAIL=0
FAILS=()

pass() {
  TESTS_PASS=$((TESTS_PASS + 1))
  TESTS_TOTAL=$((TESTS_TOTAL + 1))
}
fail() {
  TESTS_FAIL=$((TESTS_FAIL + 1))
  TESTS_TOTAL=$((TESTS_TOTAL + 1))
  FAILS+=("$1")
  printf '  ✗ %s\n' "$1"
}

# Запрещённый паттерн: аргумент в кавычках, заканчивающийся на `/."`
# (трейлинг-slash + dot + закрывающая кавычка). Используем FIXED-string
# поиск, чтобы не зависеть от поведения shell escaping (см. devops-ретро
# 19.08 — backslash-escapes в `rg '\.'`/`grep '\.'` иногда не срабатывают
# на builder'е из-за особенностей локали).

# Поиск: для каждого yml-файла проверяем, что в одной строке
# присутствует и `scp ` (с пробелом), и трейлинг-`/.` в кавычках.
scan_for_dotrex() {
  local dir="$1"

  if command -v rg >/dev/null 2>&1; then
    # ripgrep: ищем файлы где в ОДНОЙ строке есть И 'scp ' И '/."'
    # (трейлинг-`/.` в кавычках). Используем --multiline-detect не нужно —
    # нам нужен матч в пределах одной строки, а ripgrep это делает по умолчанию.
    # Паттерн: scp(опц.флаги)(пробел) ... "/."  -- но ripgrep не умеет
    # AND внутри одного регекспа без lookahead. Поэтому делаем двумя
    # проходами: сначала 'scp ' строки, потом в каждой проверяем '/."'.
    local scp_lines
    scp_lines=$(rg -n -F --glob '*.yml' --glob '*.yaml' 'scp ' "$dir" || true)
    if [ -z "$scp_lines" ]; then
      return 0
    fi
    # Проверяем каждую строку с 'scp ' на наличие '/."'
    local bad
    bad=$(printf '%s\n' "$scp_lines" | awk -F: '
      {
        # последнее поле — содержимое строки; всё до — file:line[:col]
        # (но ":" в IP/URL ломает простую split, поэтому ищем подстроку
        # в полной исходной строке через индекс)
        line_text = $0
        # восстановим текст после последнего двоеточия которое идёт
        # за номером строки (формат rg: file:line:text)
        sub(/^[^:]+:[0-9]+:/, "", line_text)
        if (index(line_text, "/.\"")) {
          print $0
        }
      }
    ')
    if [ -n "$bad" ]; then
      printf '%s\n' "$bad"
      return 1
    fi
    return 0
  else
    # grep POSIX fallback: ищем файлы где есть ОБА токена на одной строке.
    local hits
    hits=$(grep -RHnF --include='*.yml' --include='*.yaml' -e 'scp ' -e '/."' "$dir" 2>/dev/null \
           | awk -F: '
             {
                line_text = $0
                sub(/^[^:]+:[0-9]+:/, "", line_text)
                if (index($0, "scp ") > 0 && index($0, "/.\"") > 0) {
                  print $0
                }
             }
           ')
    if [ -n "$hits" ]; then
      printf '%s\n' "$hits"
      return 1
    fi
    return 0
  fi
}

# --- Test 1: репозиторий чист от запрещённого паттерна ---------------------
echo "[1] workflows: сканируем $WORKFLOWS_DIR на наличие запрещённого паттерна scp ... /. ..."
if scan_for_dotrex "$WORKFLOWS_DIR" >/tmp/_dotrex_matches.$$ 2>&1; then
  echo "  ✓ не найдено"
  pass
else
  fail "найден запрещённый паттерн scp ... /. в workflow (см. /tmp/_dotrex_matches.$$)"
  cat /tmp/_dotrex_matches.$$ | sed 's/^/    /'
fi
rm -f /tmp/_dotrex_matches.$$

# --- Test 2: lock-in — позитивный кейс (детектор ловит инъекцию) -----------
echo "[2] lock-in: добавляем запрещённый паттерн в sandbox-файл, проверяем что детектор ловит"
SANDBOX="$(mktemp -d)"
# trap на EXIT, но rm -f /tmp/* не должен валить скрипт
trap 'rm -rf "$SANDBOX" 2>/dev/null || true' EXIT
cat >"$SANDBOX/bad.yml" <<'YAML'
name: bad
on: push
jobs:
  j:
    runs-on: ubuntu-latest
    steps:
      - name: collect
        run: |
          sshpass -e scp -r "ros2@host:$d/." "$ART_DIR/$bn/"
YAML

if scan_for_dotrex "$SANDBOX" >/dev/null 2>&1; then
  fail "lock-in: детектор НЕ сработал на заведомо плохом синтаксисе"
else
  echo "  ✓ детектор сработал"
  pass
fi

# --- Test 3: lock-in — негативный кейс (rsync-стиль НЕ детектируется) ------
echo "[3] lock-in: rsync-стиль НЕ должен триггерить детектор"
cat >"$SANDBOX/good.yml" <<'YAML'
name: good
on: push
jobs:
  j:
    runs-on: ubuntu-latest
    steps:
      - name: collect
        run: |
          sshpass -e rsync -a -e 'ssh -o StrictHostKeyChecking=no' "ros2@host:$d/" "$ART_DIR/$bn/"
YAML

if scan_for_dotrex "$SANDBOX" >/dev/null 2>&1; then
  fail "lock-in: детектор ложно сработал на rsync-стиль (false positive)"
else
  echo "  ✓ rsync-стиль чист"
  pass
fi

# --- Итог ------------------------------------------------------------------
echo ""
echo "  Итог: pass=$TESTS_PASS fail=$TESTS_FAIL total=$TESTS_TOTAL"
if [ "$TESTS_FAIL" -gt 0 ]; then
  echo ""
  echo "FAILED:"
  for f in "${FAILS[@]}"; do printf '  - %s\n' "$f"; done
  exit 1
fi
echo "ALL OK"