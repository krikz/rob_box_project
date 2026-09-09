#!/usr/bin/env bash
# test_validate_test_ws_dirs.sh — регресс-тест validate_test_ws_dirs.py.
#
# Строит синтетические мини-репо (workflow + тесты) и проверяет, что guard:
#   1) FAIL'ит непокрытый корневой каталог, читаемый от корня репо;
#   2) OK'ает, когда каталог добавлен в rsync-список ВСЕХ job'ов;
#   3) FAIL'ит, если каталог покрыт лишь в ОДНОМ job'е из двух (реальный
#      сценарий t_29b9ce36: починили integration, забыли unit);
#   4) НЕ ложно срабатывает на package-local каталог
#      (src/<pkg>/scripts/ — это не корневой scripts/);
#   5) даёт WARN (не FAIL) когда обращение защищено skipif, а --strict валит.
#
# Запуск:
#   bash scripts/agent_flow/tests/test_validate_test_ws_dirs.sh
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GUARD="$HERE/../validate_test_ws_dirs.py"
[ -f "$GUARD" ] || { echo "FATAL: guard не найден: $GUARD" >&2; exit 2; }

PASS=0
FAIL=0

_ok()   { echo "  ok   — $1"; PASS=$((PASS + 1)); }
_bad()  { echo "  FAIL — $1"; FAIL=$((FAIL + 1)); }

# make_repo <dir> <rsync_list_job1> <rsync_list_job2>
make_repo() {
    local d="$1" l1="$2" l2="$3"
    mkdir -p "$d/.github/workflows" "$d/src/pkg_a/test/unit" "$d/docker" "$d/docs"
    cat > "$d/.github/workflows/G-Run Tests.yml" <<YAML
jobs:
  unit-tests:
    steps:
      - name: Build workspace
        run: |
          for d in ${l1}; do
            [ -d "\$d" ] && rsync -av "\$d" \${{ github.workspace }}/test_ws/
          done
  integration-tests:
    steps:
      - name: Build workspace
        run: |
          for d in ${l2}; do
            [ -d "\$d" ] && rsync -av "\$d" \${{ github.workspace }}/test_ws/
          done
YAML
}

# добавить тест, читающий корневой <dir> (src/pkg_a/test/unit/x.py -> parents[4])
add_root_test() {
    local d="$1" dir="$2"
    mkdir -p "$d/$dir"
    cat > "$d/src/pkg_a/test/unit/test_root_ref.py" <<PY
from pathlib import Path
_ROOT = Path(__file__).resolve().parents[4]
_DATA = _ROOT / "${dir}" / "thing.py"
def test_reads_root_dir():
    assert _DATA
PY
}

run_guard() {  # run_guard <repo> [extra args...] -> печатает exit code
    local repo="$1"; shift
    ( cd "$repo" && python3 "$GUARD" "$@" >/tmp/guard_out.txt 2>&1; echo $? )
}

echo "== 1. непокрытый корневой каталог -> FAIL"
T1="$(mktemp -d)"
make_repo "$T1" "docker docs" "docker docs"
add_root_test "$T1" "scripts"
RC="$(run_guard "$T1")"
if [ "$RC" = "1" ] && grep -q "FAIL: 'scripts/'" /tmp/guard_out.txt; then
    _ok "guard поймал непокрытый scripts/ (exit 1)"
else
    _bad "ожидал exit 1 + FAIL scripts/, получил exit $RC"; cat /tmp/guard_out.txt
fi

echo "== 2. каталог покрыт в обоих job'ах -> OK"
T2="$(mktemp -d)"
make_repo "$T2" "docker docs scripts" "docker docs scripts"
add_root_test "$T2" "scripts"
RC="$(run_guard "$T2")"
if [ "$RC" = "0" ] && grep -q "validate_test_ws_dirs: OK" /tmp/guard_out.txt; then
    _ok "покрытый каталог даёт OK (exit 0)"
else
    _bad "ожидал exit 0 + OK, получил exit $RC"; cat /tmp/guard_out.txt
fi

echo "== 3. покрыт только в ОДНОМ job'е -> FAIL (регресс t_29b9ce36)"
T3="$(mktemp -d)"
make_repo "$T3" "docker docs" "docker docs scripts"
add_root_test "$T3" "scripts"
RC="$(run_guard "$T3")"
if [ "$RC" = "1" ] && [ "$(grep -c "FAIL: 'scripts/'" /tmp/guard_out.txt)" = "1" ]; then
    _ok "частичное покрытие (1 из 2 job) поймано ровно один раз"
else
    _bad "ожидал exit 1 + ровно 1 FAIL, получил exit $RC"; cat /tmp/guard_out.txt
fi

echo "== 4. package-local каталог -> НЕ ложное срабатывание"
T4="$(mktemp -d)"
make_repo "$T4" "docker docs" "docker docs"
mkdir -p "$T4/src/pkg_a/scripts" "$T4/scripts"
cat > "$T4/src/pkg_a/test/unit/test_pkg_local.py" <<'PY'
from pathlib import Path
# parents[1] от test/unit/ = test/ ... это НЕ корень репо
_SRC = Path(__file__).resolve().parents[1] / "scripts" / "node.py"
def test_local():
    assert _SRC
PY
RC="$(run_guard "$T4")"
if [ "$RC" = "0" ]; then
    _ok "package-local scripts/ не даёт ложный FAIL"
else
    _bad "ложное срабатывание на package-local (exit $RC)"; cat /tmp/guard_out.txt
fi

echo "== 5. skipif-guarded -> WARN, а с --strict -> FAIL"
T5="$(mktemp -d)"
make_repo "$T5" "docker docs" "docker docs"
mkdir -p "$T5/tools"
cat > "$T5/src/pkg_a/test/unit/test_skipif.py" <<'PY'
from pathlib import Path
import pytest
_ROOT = Path(__file__).resolve().parents[4]
_GEN = _ROOT / "tools" / "gen.py"
@pytest.mark.skipif(not _GEN.exists(), reason="генератор не найден")
def test_uses_generator():
    assert _GEN
PY
RC="$(run_guard "$T5")"
RC_STRICT="$(run_guard "$T5" --strict)"
if [ "$RC" = "0" ] && grep -q "WARN: 'tools/'" /tmp/guard_out.txt && [ "$RC_STRICT" = "1" ]; then
    _ok "skipif -> WARN (exit 0), --strict -> FAIL (exit 1)"
else
    _bad "ожидал WARN/exit0 и strict/exit1, получил $RC / $RC_STRICT"
    cat /tmp/guard_out.txt
fi

echo
echo "== итог: passed=$PASS failed=$FAIL"
[ "$FAIL" = "0" ] || exit 1
echo "test_validate_test_ws_dirs: ALL PASS"
