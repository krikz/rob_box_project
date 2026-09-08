#!/bin/bash
# ============================================================================
# test_card_defaults.sh — hermetic test для af_card_defaults_for
#                          в lib_agent_flow_common.sh (ADR-0080 #2162).
#
# Возвращает набор skills из card_defaults.yaml по assignee + labels.
# Тесты:
#   * hermetic HERMES_HOME с подложенным card_defaults.yaml
#   * cover всех групп (review/backend/devops/tester/architect/etc)
#   * label-based override (agent:pr-reviewer на backend → review_cards)
#   * неизвестный assignee → default_cards
#   * missing yaml → fallback на verification-before-completion
#
# Invocation:
#   bash scripts/agent_flow/tests/test_card_defaults.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB="$TEST_DIR/../lib_agent_flow_common.sh"
CARD_YAML_SRC="$TEST_DIR/../card_defaults.yaml"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

PASS=0; FAIL=0
pass() { PASS=$((PASS+1)); echo "  ✓ $1"; }
fail() { FAIL=$((FAIL+1)); echo "  ✗ $1 (${2:-})"; }

# Extract af_card_defaults_for (свежий код из LIB).
start="$(grep -n '^af_card_defaults_for()' "$LIB" | head -1 | cut -d: -f1)"
[ -n "$start" ] || { echo "FAIL: af_card_defaults_for not found in $LIB"; exit 1; }
end="$(awk -v s="$start" 'NR>=s && /^}$/{print NR; exit}' "$LIB")"
sed -n "${start},${end}p" "$LIB" > "$WORK/helper.sh"
# shellcheck disable=SC1091
. "$WORK/helper.sh"
_af_log() { :; }

# --- Подготовка: hermetic card_defaults.yaml + LIB_DIR ----------------------
mkdir -p "$WORK/lib"
cp "$CARD_YAML_SRC" "$WORK/lib/card_defaults.yaml"
LIB_DIR="$WORK/lib"

# Также подложим SOT yaml — af_card_defaults_for ищет рядом с LIB.
cp "$CARD_YAML_SRC" "$WORK/lib/card_defaults.yaml"

# helper.sh ссылается на свой BASH_SOURCE[0] — это "$WORK/helper.sh".
# Подменим, чтобы af_card_defaults_for нашёл yaml в $WORK/lib/.
cat > "$WORK/loader.sh" <<EOF
. "$WORK/helper.sh"
EOF
# helper.sh сейчас загружен выше, и af_card_defaults_for внутри использует
# \${BASH_SOURCE[0]:-/dev/null}. Так как BASH_SOURCE для helper.sh указывает
# на /tmp/.../helper.sh, не на lib/. Используем переменную CARD_DEFAULTS_YAML.
export CARD_DEFAULTS_YAML="$WORK/lib/card_defaults.yaml"

assert_eq() {  # $1=expected  $2=actual  $3=msg
    if [ "$1" = "$2" ]; then pass "$3"
    else fail "$3" "expected='$1' actual='$2'"; fi
}

# Helper: превращает многострочный stdout в | joined (без trailing |).
join() { paste -sd '|' -; }

# --- T1: devops → devops_cards ---------------------------------------------
assert_eq "verification-before-completion|agent-flow|code-review|resolving-merge-conflicts" \
    "$(af_card_defaults_for devops '' | join)" \
    "T1: devops → devops_cards"

# --- T2: backend → backend_fix_cards ---------------------------------------
assert_eq "verification-before-completion|test-driven-development|systematic-debugging|code-review" \
    "$(af_card_defaults_for backend '' | join)" \
    "T2: backend → backend_fix_cards"

# --- T3: pr-reviewer → review_cards ----------------------------------------
assert_eq "verification-before-completion|code-review|diff-review|architecture-doc-review" \
    "$(af_card_defaults_for pr-reviewer '' | join)" \
    "T3: pr-reviewer → review_cards"

# --- T4: architect → architect_cards ---------------------------------------
assert_eq "verification-before-completion|agent-flow|architecture-doc-review|to-tickets" \
    "$(af_card_defaults_for architect '' | join)" \
    "T4: architect → architect_cards"

# --- T5: tester → tester_cards ---------------------------------------------
assert_eq "verification-before-completion|test-driven-development|sdlc-review" \
    "$(af_card_defaults_for tester '' | join)" \
    "T5: tester → tester_cards"

# --- T6: agent-flow → agent_flow_cards -------------------------------------
assert_eq "verification-before-completion|agent-flow|code-review" \
    "$(af_card_defaults_for agent-flow '' | join)" \
    "T6: agent-flow → agent_flow_cards"

# --- T7: analyst → analyst_cards -------------------------------------------
assert_eq "verification-before-completion|test-driven-development" \
    "$(af_card_defaults_for analyst '' | join)" \
    "T7: analyst → analyst_cards"

# --- T8: неизвестный assignee → default_cards -------------------------------
assert_eq "verification-before-completion|agent-flow" \
    "$(af_card_defaults_for unknown-profile '' | join)" \
    "T8: unknown assignee → default_cards"

# --- T9: label-based override: agent:pr-reviewer на backend ----------------
assert_eq "verification-before-completion|code-review|diff-review|architecture-doc-review" \
    "$(af_card_defaults_for backend 'agent:pr-reviewer,priority:high' | join)" \
    "T9: label override agent:pr-reviewer → review_cards"

# --- T10: label-based override: agent:devops ------------------------------
assert_eq "verification-before-completion|agent-flow|code-review|resolving-merge-conflicts" \
    "$(af_card_defaults_for backend 'agent:devops' | join)" \
    "T10: label override agent:devops → devops_cards"

# --- T11: label-based override: type:adr → architect -----------------------
assert_eq "verification-before-completion|agent-flow|architecture-doc-review|to-tickets" \
    "$(af_card_defaults_for devops 'type:adr' | join)" \
    "T11: label override type:adr → architect_cards"

# --- T12: yaml отсутствует — fallback verification-before-completion -------
# Используем env-prefixed command — переменная не должна leak в последующие
# тесты (T13/T14 требуют валидный yaml).
( unset CARD_DEFAULTS_YAML
  result="$(af_card_defaults_for devops '' | join)"
  printf '%s\n' "$result"
) > /tmp/t12_out_$$
result="$(cat /tmp/t12_out_$$)"
rm -f /tmp/t12_out_$$
assert_eq "verification-before-completion" "$result" "T12: missing yaml → fallback verification-before-completion"

# --- T13: пустой assignee → default_cards (verification + agent-flow) ------
assert_eq "verification-before-completion|agent-flow" \
    "$(af_card_defaults_for '' '' | join)" \
    "T13: empty assignee → default_cards"

# --- T14: регистр меток — agent:DevOps (mixed case) → devops_cards ----------
assert_eq "verification-before-completion|agent-flow|code-review|resolving-merge-conflicts" \
    "$(af_card_defaults_for backend 'agent:DevOps' | join)" \
    "T14: case-insensitive label match"

# --- summary --------------------------------------------------------------
echo
echo "af_card_defaults_for: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ] || exit 1
exit 0