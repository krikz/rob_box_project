#!/bin/bash
# ============================================================================
# test_parse_body_skills.sh — hermetic test для parse_body_skills_section
#                               в lib_agent_flow_common.sh (ADR-0080 #2162).
#
# Парсер секции `## Skills` из body карточки:
#   * нумерованный "1. foo — desc" → foo
#   * bullet "- foo: desc" → foo
#   * голые "foo" / "foo — desc" → foo
#   * dedup в порядке появления
#   * секция заканчивается на следующем `## ` heading
#   * пустой вход → пустой stdout (fail-OPEN)
#
# Invocation:
#   bash scripts/agent_flow/tests/test_parse_body_skills.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB="$TEST_DIR/../lib_agent_flow_common.sh"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

PASS=0; FAIL=0
pass() { PASS=$((PASS+1)); echo "  ✓ $1"; }
fail() { FAIL=$((FAIL+1)); echo "  ✗ $1 (${2:-})"; }

# Extract parse_body_skills_section
start="$(grep -n '^parse_body_skills_section()' "$LIB" | head -1 | cut -d: -f1)"
[ -n "$start" ] || { echo "FAIL: parse_body_skills_section not found in $LIB"; exit 1; }
end="$(awk -v s="$start" 'NR>=s && /^}$/{print NR; exit}' "$LIB")"
sed -n "${start},${end}p" "$LIB" > "$WORK/helper.sh"
# shellcheck disable=SC1091
. "$WORK/helper.sh"
_af_log() { :; }

assert_eq() {  # $1=expected  $2=actual  $3=msg
    if [ "$1" = "$2" ]; then pass "$3"
    else fail "$3" "expected='$1' actual='$2'"; fi
}

# Helper: превращает многострочный stdin в pipe-joined строку для assert
# (без trailing pipe — `tr` оставляет последний \n → |).
join_lines() { paste -sd '|' -; }

# --- T1..T4: форматы входа -------------------------------------------------
b1="Some intro

## Skills (порядок)

1. code-review — двухосевое ревью diff
2. verification-before-completion — чеклист
3. architecture-doc-review — для ADR/PR

## Foo

more text"
assert_eq "code-review|verification-before-completion|architecture-doc-review" \
    "$(parse_body_skills_section "$b1" | join_lines)" \
    "T1: numbered format with em-dash comments"

# Bullet format
b2="## Skills

- code-review: ревью
* agent-flow — процесс
- ponytail

## Next
text"
assert_eq "code-review|agent-flow|ponytail" \
    "$(parse_body_skills_section "$b2" | join_lines)" \
    "T2: bullet format with mixed separators"

# Empty body → empty stdout
assert_eq "" "$(parse_body_skills_section '')" "T3: empty body → empty stdout"

# Body без секции ## Skills → empty
b4="Just body without skills section

## Acceptance
- foo"
assert_eq "" "$(parse_body_skills_section "$b4")" "T4: body without section → empty"

# --- T5..T7: dedup ---------------------------------------------------------
b5="## Skills
1. foo — desc
2. foo — dup
3. bar"
assert_eq "foo|bar" "$(parse_body_skills_section "$b5" | join_lines)" "T5: dedup (case-sensitive)"

b6="## Skills
- foo
- FOO
- Foo"
assert_eq "foo|FOO|Foo" "$(parse_body_skills_section "$b6" | join_lines)" "T6: dedup is case-sensitive"

# --- T8..T10: section ending on next heading -------------------------------
b7="## Skills
1. foo

## Other section
1. bar"
assert_eq "foo" "$(parse_body_skills_section "$b7" | join_lines)" "T7: section ends at next ## heading"

# --- T8: пробелы и странные разделители -----------------------------------
b8="## Skills
 1. foo — desc
  2. bar : note
   * baz | comment"
assert_eq "foo|bar|baz" "$(parse_body_skills_section "$b8" | join_lines)" "T8: whitespace + separators (en-dash, colon, pipe)"

# --- T9: пустые строки внутри секции --------------------------------------
b9="## Skills

1. foo

2. bar

"
assert_eq "foo|bar" "$(parse_body_skills_section "$b9" | join_lines)" "T9: blank lines between items"

# --- T10: '## Skills' префикс с разными написаниями -----------------------
b10="## skills (lower)
1. foo"
assert_eq "foo" "$(parse_body_skills_section "$b10" | join_lines)" "T10: '## skills' (lowercase) is recognized"

b11="##Skills no-space
1. foo"
assert_eq "foo" "$(parse_body_skills_section "$b11" | join_lines)" "T11: '##Skills' without space"

# --- T12: skill с точкой в имени -------------------------------------------
b12="## Skills
1. hermes.kanban-cli
2. foo.bar.baz"
assert_eq "hermes.kanban-cli|foo.bar.baz" "$(parse_body_skills_section "$b12" | join_lines)" "T12: skill names with dots preserved"

# --- T13: en-dash vs em-dash в разделителе ---------------------------------
b13="## Skills
1. foo — desc em-dash
2. bar – desc en-dash
3. baz: colon"
assert_eq "foo|bar|baz" "$(parse_body_skills_section "$b13" | join_lines)" "T13: em-dash, en-dash, colon all treated as separator"

# --- T14: ## Skills внутри code block — текущее поведение ------------------
# Парсер НЕ различает code blocks (fenced ```) — это документированное
# ограничение версии 1. Воркеры обычно не пишут '## Skills' внутри ``` блоков,
# а если и напишут — skill_view("```") просто ничего не загрузит (fail-OPEN).
# Тест фиксирует текущее поведение.
b14="## Skills
1. real-skill
\`\`\`
## Skills inside code
1. fake-skill
\`\`\`"
result="$(parse_body_skills_section "$b14" | paste -sd '|' -)"
expected="real-skill|\`\`\`|fake-skill"
assert_eq "$expected" "$result" "T14: code blocks NOT excluded (documented v1 limitation; skill_view will fail-open on '\`\`\`')"

# --- summary --------------------------------------------------------------
echo
echo "parse_body_skills_section: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ] || exit 1
exit 0