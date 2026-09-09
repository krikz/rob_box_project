#!/bin/bash
# ============================================================================
# test_retro_intent_extractor.sh — issue #2069
#
# Проверяет ТОЛЬКО извлечение (issue, pr, head, intent) из смерженных PR в
# ретро-пути merge-gate. Это самый маленький кусок, который решает судьбу
# карточки: если он выдаёт intent=close, дальше по цепочке карточка будет
# закрыта как COMPLETED.
#
# Зачем отдельный тест, если есть test_merge_gate_retro_path.sh:
# тот прогоняет merge-gate целиком, а merge-gate форсит HOME=/home/builder
# (agent-flow-merge-gate.sh:43) и требует моков gh/hermes — то есть он
# запускается только на агент-хосте. Этот тест не зависит ни от чего, кроме
# python3, и потому реально прогоняется на любой машине и в CI.
#
# Сценарии (все — реальные случаи из #2069):
#   1. body-only #N без closing-keyword  → ref   (PR #2047 → #1996)
#   2. wip-заголовок, даже с "closes #N" → ref   (PR #2014 → #2004)
#   3. #N в заголовке PR                 → close (PR #2039 → #2001)
#   4. closes #N в теле                  → close (обычный путь)
#   5. "see also #N" в теле              → ref
#   6. self-reference (#N == номер PR)   → не выводится вовсе
#
# Run:
#   bash scripts/agent_flow/tests/test_retro_intent_extractor.sh
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MERGE_GATE="${SCRIPT_DIR}/../agent-flow-merge-gate.sh"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# python3 на агент-хосте, python — на dev-машинах под Git Bash, где
# `python3` уходит в заглушку Microsoft Store. Тест обязан идти в обоих.
PY=""
for _c in python3 python; do
    if command -v "$_c" >/dev/null 2>&1 && "$_c" -c 'import sys; sys.exit(0 if sys.version_info[0]==3 else 1)' >/dev/null 2>&1; then
        PY="$_c"; break
    fi
done
if [ -z "$PY" ]; then
    echo "[   SKIP  ] python3 не найден — тест пропущен"
    exit 0
fi

pass=0
fail=0

assert_row() {  # $1=out $2=expected_row $3=label
    if printf '%s\n' "$1" | grep -qxF "$2"; then
        echo "[   PASS  ] $3"
        pass=$((pass + 1))
    else
        echo "[   FAIL  ] $3"
        echo "    ожидалась строка: $2"
        echo "    получено:"
        printf '%s\n' "$1" | sed 's/^/      /'
        fail=$((fail + 1))
    fi
}

assert_no_issue() {  # $1=out $2=issue $3=label
    if printf '%s\n' "$1" | cut -f1 | grep -qxF "$2"; then
        echo "[   FAIL  ] $3"
        echo "    issue $2 не должен был попасть в вывод"
        fail=$((fail + 1))
    else
        echo "[   PASS  ] $3"
        pass=$((pass + 1))
    fi
}

# --- вытащить python-блок экстрактора из merge-gate --------------------------
# Границы: строка `done < <(printf '%s' "$_retro_prs_json" | python3 -c '`
# и закрывающая `' "$_retro_since" 2>/dev/null)`.
sed -n "/^done < <(printf '%s' \"\$_retro_prs_json\" | python3 -c '/,/^' \"\$_retro_since\" 2>\/dev\/null)/p" \
    "$MERGE_GATE" \
    | sed "1s/^done < <(printf '%s' \"\$_retro_prs_json\" | python3 -c '//" \
    | sed '$d' >"$TMP/extract.py"

if [ ! -s "$TMP/extract.py" ]; then
    echo "[   FAIL  ] не удалось вытащить экстрактор из ${MERGE_GATE}"
    echo "    блок ретро-пути переписан? проверь якоря в этом тесте"
    exit 1
fi

cat >"$TMP/prs.json" <<'JSON'
[
 {"number":2047,
  "title":"design: pregenerate contract (ADR-0056, issue #2003)",
  "body":"## Зависимости / blockers\n- **#1996 ([operator-agent 07a]) ОТКРЫТ** — priority в tts_node.\n",
  "headRefName":"wt/t_201e2c64","mergedAt":"2026-09-07T11:20:00Z"},
 {"number":2014,
  "title":"wip(operator-agent verify #2004): статический разбор гипотез",
  "body":"closes #2004\n",
  "headRefName":"z/verify","mergedAt":"2026-09-07T07:10:00Z"},
 {"number":2039,
  "title":"[operator-agent 11] operator.admin (#2001)",
  "body":"Реализация среза.",
  "headRefName":"z/admin","mergedAt":"2026-09-07T09:00:00Z"},
 {"number":1143,
  "title":"fix retro",
  "body":"closes #1138\n",
  "headRefName":"z/x","mergedAt":"2026-08-12T14:14:05Z"},
 {"number":9001,
  "title":"chore: cleanup",
  "body":"see also #7777\n",
  "headRefName":"z/y","mergedAt":"2026-09-07T09:00:00Z"},
 {"number":9002,
  "title":"chore: self ref #9002",
  "body":"PR: #9002\n",
  "headRefName":"z/z","mergedAt":"2026-09-07T09:00:00Z"}
]
JSON

out="$("$PY" "$TMP/extract.py" '2026-08-01T00:00:00Z' <"$TMP/prs.json")"

echo "==== вывод экстрактора ===="
printf '%s\n' "$out"
echo "==========================="

assert_row "$out" "$(printf '1996\t2047\twt/t_201e2c64\tref')" \
    "1. body-only упоминание #1996 (секция blockers) → ref, не close"
assert_row "$out" "$(printf '2004\t2014\tz/verify\tref')" \
    "2. wip-заголовок PR #2014 → ref даже при 'closes #2004'"
assert_row "$out" "$(printf '2001\t2039\tz/admin\tclose')" \
    "3. #2001 в заголовке PR → close (ретро-путь не сломан)"
assert_row "$out" "$(printf '1138\t1143\tz/x\tclose')" \
    "4. 'closes #1138' в теле → close"
assert_row "$out" "$(printf '7777\t9001\tz/y\tref')" \
    "5. 'see also #7777' → ref"
assert_no_issue "$out" "9002" \
    "6. self-reference #9002 в своём же PR не выводится"

echo
echo "==== Summary ===="
echo "total:  $((pass + fail))"
echo "passed: $pass"
echo "failed: $fail"
[ "$fail" -eq 0 ]
