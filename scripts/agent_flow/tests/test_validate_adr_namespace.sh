#!/usr/bin/env bash
# ============================================================================
# test_validate_adr_namespace.sh — регресс-тест validate_adr_namespace.sh.
#
# Покрывает acceptance (карточка t_dd923f2a, ретро t_debcb647):
#   A. Negative: фиктивный 0040-collide.md на ветке с уже существующим
#      0040 в origin/develop → exit 1, в stderr collision-сообщение с
#      "Next free slot: 0043" (или NNN, зависящий от baseline).
#   B. Positive: фиктивный 0043-fresh.md (или NNN за пределами существующих)
#      → exit 0, "clean".
#   C. Edge: правка существующего ADR (--diff-filter=M, не Added) → clean
#      даже если номер совпадает (правка своего — не collision).
#   D. Edge: нет ни одного ADR в PR → clean.
#   E. Edge: --ref на несуществующую ссылку → exit 2 (usage error).
#   F. Edge: --ref на пустой ref без ADR-файлов → clean (collision невозможна).
#   G. Sanity: pipeline под set -euo pipefail не падает на пустом вводе
#      (ретро t_9435a3c5 — SIGPIPE → exit 1 без сообщения).
#   H. Slug-format: collision-сообщение содержит slug из baseline
#      (чтобы воркер сразу видел, кто сидит под этим номером).
#
# Стратегия: для каждого сценария создаём временный git-репозиторий
# с поддельным origin/develop и HEAD, копируем validate_adr_namespace.sh
# в этот репо, запускаем. Чистый tmp → чистая изоляция от develop.
#
# Run:
#   bash scripts/agent_flow/tests/test_validate_adr_namespace.sh
# ============================================================================

set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$TEST_DIR/.." && pwd)"
TARGET="$ROOT_DIR/validate_adr_namespace.sh"

if [ ! -x "$TARGET" ]; then
    echo "FAIL: $TARGET not executable" >&2
    exit 1
fi

WORK="$(mktemp -d -t adr-namespace-test-XXXXXX)"
trap 'rm -rf "$WORK"' EXIT

pass_count=0
fail_count=0
fail_log=""

# Вспомогалка: создать минимальный git-repo, имитирующий origin/develop.
#   $1 = work_dir (создаётся)
# Внутри:
#   - origin/develop: содержит NNNN-*.md файлы (default: 0039, 0040, 0041, 0042).
#   - main: HEAD ветки-кандидата (изначально == origin/develop).
setup_repo() {  # $1=work_dir
    local d="$1"
    mkdir -p "$d/docs/adr"
    cd "$d" || exit 2

    git init -q -b main
    git config user.email "test@test"
    git config user.name "test"
    git remote add origin "$d"

    # Baseline ADR-файлы: 0039, 0040, 0041, 0042 (как на 01.09 в origin/develop).
    cat > docs/adr/0039-openspec-integration.md <<'EOF'
# ADR-0039: openspec-integration
EOF
    cat > docs/adr/0040-e2e-process.md <<'EOF'
# ADR-0040: e2e-process-no-run-no-round
EOF
    cat > docs/adr/0041-unknown-assignee-silent-drop.md <<'EOF'
# ADR-0041: unknown-assignee-silent-drop-guard
EOF
    cat > docs/adr/0042-unknown-assignee-rollup.md <<'EOF'
# ADR-0042: unknown-assignee-rollup-guard
EOF
    git add . >/dev/null
    git commit -q -m "baseline: 4 ADR"
    # Создаём develop как отдельную ветку (как в реальном rob_box_project).
    git checkout -q -b develop
    # Синхронизируем origin/develop (для скрипта, который ходит в origin/develop).
    git push -q origin develop 2>/dev/null || true
    git fetch -q origin develop 2>/dev/null || true
}

# Запуск validate_adr_namespace.sh в контексте work_dir.
#   $1=work_dir $2=extra_args (опц.)
run_validate() {  # → stdout, stderr (объединённые), rc
    local d="$1"; shift
    (
        cd "$d" || exit 2
        bash "$TARGET" "$@"
    ) 2>&1
}

# === Сценарии ===

# --- A. Negative: collision ---
A_DIR="$WORK/A"
setup_repo "$A_DIR"
cat > "$A_DIR/docs/adr/0040-collide.md" <<'EOF'
# ADR-0040: collide
EOF
(
    cd "$A_DIR" || exit 2
    git add . >/dev/null
    git commit -q -m "collision ADR"
)
OUT_A="$(run_validate "$A_DIR")"
RC_A=$?
GOT_FREE="$(printf '%s' "$OUT_A" | grep -cE 'Next free (slot|в [A-Z]+-домене)' || true)"
GOT_COLLIDE_MSG="$(printf '%s' "$OUT_A" | grep -c 'ADR namespace collision detected' || true)"
# Slug извлекается sed'ом до первого `-` после номера. Файл 0040-e2e-process.md
# → slug = "e2e-process" (полный kebab до конца имени, без .md).
GOT_SLUG_E2E="$(printf '%s' "$OUT_A" | grep -c 'e2e-process' || true)"
GOT_SLUG_NEW="$(printf '%s' "$OUT_A" | grep -c 'collide' || true)"
if [ "$RC_A" -ne 1 ] \
    || [ "${GOT_FREE:-0}" -lt 1 ] \
    || [ "${GOT_COLLIDE_MSG:-0}" -lt 1 ] \
    || [ "${GOT_SLUG_E2E:-0}" -lt 1 ] \
    || [ "${GOT_SLUG_NEW:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [A: negative-collision]: expected rc=1 with collision msg + 'Next free slot:' + both slugs.
  rc=$RC_A
  free-slot-matches=$GOT_FREE
  collide-msg-matches=$GOT_COLLIDE_MSG
  baseline-slug-matches=$GOT_SLUG_E2E
  new-slug-matches=$GOT_SLUG_NEW
  output:
$OUT_A
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [A: negative-collision] (rc=$RC_A, next-free-detected)"
fi

# --- B. Positive: fresh 0043 ---
B_DIR="$WORK/B"
setup_repo "$B_DIR"
cat > "$B_DIR/docs/adr/0043-fresh.md" <<'EOF'
# ADR-0043: fresh
EOF
(
    cd "$B_DIR" || exit 2
    git add . >/dev/null
    git commit -q -m "fresh ADR"
)
OUT_B="$(run_validate "$B_DIR")"
RC_B=$?
GOT_CLEAN="$(printf '%s' "$OUT_B" | grep -c '^validate_adr_namespace: clean' || true)"
if [ "$RC_B" -ne 0 ] || [ "${GOT_CLEAN:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [B: positive-fresh]: expected rc=0 + 'clean' in stdout.
  rc=$RC_B
  clean-matches=$GOT_CLEAN
  output:
$OUT_B
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [B: positive-fresh] (rc=$RC_B, clean)"
fi

# --- C. Edge: правка существующего ADR ---
C_DIR="$WORK/C"
setup_repo "$C_DIR"
echo "extra line" >> "$C_DIR/docs/adr/0040-e2e-process.md"
(
    cd "$C_DIR" || exit 2
    git add . >/dev/null
    git commit -q -m "fix typo"
)
OUT_C="$(run_validate "$C_DIR")"
RC_C=$?
GOT_CLEAN_C="$(printf '%s' "$OUT_C" | grep -c '^validate_adr_namespace: clean' || true)"
if [ "$RC_C" -ne 0 ] || [ "${GOT_CLEAN_C:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [C: modify-existing-not-collision]: expected rc=0 (правка существующего — не collision).
  rc=$RC_C
  clean-matches=$GOT_CLEAN_C
  output:
$OUT_C
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [C: modify-existing-not-collision] (rc=$RC_C)"
fi

# --- D. Edge: пустой diff (ничего нового) ---
D_DIR="$WORK/D"
setup_repo "$D_DIR"
# Никаких ADR-файлов не добавляем — дифф develop...HEAD пустой.
OUT_D="$(run_validate "$D_DIR")"
RC_D=$?
GOT_CLEAN_D="$(printf '%s' "$OUT_D" | grep -c '^validate_adr_namespace: clean' || true)"
if [ "$RC_D" -ne 0 ] || [ "${GOT_CLEAN_D:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [D: no-new-adrs]: expected rc=0 + clean.
  rc=$RC_D
  clean-matches=$GOT_CLEAN_D
  output:
$OUT_D
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [D: no-new-adrs] (rc=$RC_D)"
fi

# --- E. Edge: --ref на несуществующую ссылку → exit 2 (usage error) ---
# Захватываем stdout/stderr отдельно. Скрипт работает под set -uo pipefail
# (без -e), поэтому $? после assignment корректно сохраняется.
E_DIR="$WORK/E"
setup_repo "$E_DIR"
OUT_E_FILE="$WORK/E.out"
( cd "$E_DIR" && bash "$TARGET" --ref origin/nonexistent ) >"$OUT_E_FILE" 2>&1
RC_E=$?
OUT_E="$(cat "$OUT_E_FILE")"
GOT_ERRMSG="$(printf '%s' "$OUT_E" | grep -c "baseline 'origin/nonexistent' не достижим" || true)"
if [ "$RC_E" -ne 2 ] || [ "${GOT_ERRMSG:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [E: missing-ref]: expected rc=2 + 'не достижим'.
  rc=$RC_E
  errmsg-matches=$GOT_ERRMSG
  output:
$OUT_E
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [E: missing-ref] (rc=$RC_E)"
fi

# --- F. Edge: пустой baseline (нет ADR в origin/develop) ---
# Хотим: HEAD содержит новый ADR 0001, baseline = предыдущий коммит
# (без каких-либо ADR). Тогда collision невозможна (baseline не имеет
# занятых номеров).
F_DIR="$WORK/F"
mkdir -p "$F_DIR/docs/adr"
cd "$F_DIR" || exit 2
git init -q -b main
git config user.email "test@test"
git config user.name "test"
git remote add origin "$F_DIR"
echo "no ADR here" > README.md
git add . >/dev/null
git commit -q -m "empty baseline"
# baseline = HEAD (этот коммит). Теперь добавим новый ADR и закоммитим.
cat > docs/adr/0001-fresh.md <<'EOF'
# ADR-0001: fresh
EOF
git add . >/dev/null
git commit -q -m "fresh ADR"
# Запустим с --ref HEAD~1 (предыдущий коммит без ADR).
OUT_F="$(run_validate "$F_DIR" --ref HEAD~1)"
RC_F=$?
GOT_CLEAN_F="$(printf '%s' "$OUT_F" | grep -c '^validate_adr_namespace: clean' || true)"
if [ "$RC_F" -ne 0 ] || [ "${GOT_CLEAN_F:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [F: empty-baseline]: expected rc=0 + clean (collision невозможна).
  rc=$RC_F
  clean-matches=$GOT_CLEAN_F
  output:
$OUT_F
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [F: empty-baseline] (rc=$RC_F)"
fi

# --- G. Sanity: pipeline под set -euo pipefail не падает на пустом вводе ---
# Ретро t_9435a3c5: SIGPIPE → exit 1 без сообщения. Это В-тест для бага,
# который мы починили в основном скрипте (добавили || true в конце
# pipeline). Здесь мы запускаем чистый сценарий и убеждаемся, что rc=0
# и есть 'clean' сообщение.
G_DIR="$WORK/G"
setup_repo "$G_DIR"
OUT_G="$(run_validate "$G_DIR")"
RC_G=$?
if [ "$RC_G" -ne 0 ] || ! printf '%s' "$OUT_G" | grep -q '^validate_adr_namespace:'; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [G: pipefail-no-silent-exit]: expected rc=0 + clean stdout на пустом diff.
  rc=$RC_G
  output:
$OUT_G
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [G: pipefail-no-silent-exit] (rc=$RC_G)"
fi

# --- H. Edge: --help → exit 0 ---
OUT_H="$(bash "$TARGET" --help 2>&1)" || true
RC_H=$?
GOT_TITLE="$(printf '%s' "$OUT_H" | grep -c 'validate_adr_namespace.sh — pre-PR check' || true)"
if [ "$RC_H" -ne 0 ] || [ "${GOT_TITLE:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [H: --help]: expected rc=0 + title.
  rc=$RC_H
  output:
$OUT_H
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [H: --help] (rc=$RC_H)"
fi

# --- I. Edge: collision на НЕСКОЛЬКИХ номерах (0040 и 0042) ---
I_DIR="$WORK/I"
setup_repo "$I_DIR"
cat > "$I_DIR/docs/adr/0040-second-collide.md" <<'EOF'
# ADR-0040: second collide
EOF
cat > "$I_DIR/docs/adr/0042-third-collide.md" <<'EOF'
# ADR-0042: third collide
EOF
(
    cd "$I_DIR" || exit 2
    git add . >/dev/null
    git commit -q -m "double collision"
)
OUT_I="$(run_validate "$I_DIR")"
RC_I=$?
GOT_BOTH="$(printf '%s' "$OUT_I" | grep -cE '^  RT:(40|42) ' || true)"
if [ "$RC_I" -ne 1 ] || [ "${GOT_BOTH:-0}" -lt 2 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [I: multi-collision]: expected rc=1 + обе collision-линии.
  rc=$RC_I
  both-matches=$GOT_BOTH
  output:
$OUT_I
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [I: multi-collision] (rc=$RC_I)"
fi

# --- J. --strict флаг существует и не ломает positive-case (для будущего расширения) ---
J_DIR="$WORK/J"
setup_repo "$J_DIR"
cat > "$J_DIR/docs/adr/0043-strict-fresh.md" <<'EOF'
# ADR-0043: strict fresh
EOF
(
    cd "$J_DIR" || exit 2
    git add . >/dev/null
    git commit -q -m "strict fresh"
)
OUT_J="$(run_validate "$J_DIR" --strict)"
RC_J=$?
GOT_CLEAN_J="$(printf '%s' "$OUT_J" | grep -c '^validate_adr_namespace: clean' || true)"
if [ "$RC_J" -ne 0 ] || [ "${GOT_CLEAN_J:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [J: --strict-on-clean]: expected rc=0 + clean (--strict не должен ломать clean-case).
  rc=$RC_J
  output:
$OUT_J
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [J: --strict-on-clean] (rc=$RC_J)"
fi

# --- K. AF-домен: collision в AF (issue #2076, Phase 2) ---
# Baseline: AF-0052-watchdog.md. New: AF-0052-dedup.md → collision в AF-домене.
# ВАЖНО: watchdog добавляется в baseline (push в origin/develop), dedup — в новый коммит.
K_DIR="$WORK/K"
mkdir -p "$K_DIR/docs/adr"
cd "$K_DIR" || exit 2
git init -q -b main
git config user.email "test@test"
git config user.name "test"
git remote add origin "$K_DIR"
# Baseline: один ADR в AF
cat > docs/adr/AF-0052-watchdog.md <<'EOF'
# ADR-AF-0052: watchdog
EOF
git add . >/dev/null
git commit -q -m "baseline AF"
git checkout -q -b develop
git push -q origin develop
# Новый файл с тем же AF-номером
cat > docs/adr/AF-0052-dedup.md <<'EOF'
# ADR-AF-0052: dedup
EOF
(
    cd "$K_DIR" || exit 2
    git add . >/dev/null
    git commit -q -m "AF-domain collision"
)
OUT_K="$(run_validate "$K_DIR")"
RC_K=$?
GOT_K="$(printf '%s' "$OUT_K" | grep -cE '^  AF:52 ' || true)"
GOT_K_FREE="$(printf '%s' "$OUT_K" | grep -c 'Next free в AF-домене' || true)"
if [ "$RC_K" -ne 1 ] || [ "${GOT_K:-0}" -lt 1 ] || [ "${GOT_K_FREE:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [K: af-domain-collision]: expected rc=1 + AF:52 line + AF next-free hint.
  rc=$RC_K
  af-line-matches=$GOT_K
  af-next-free-matches=$GOT_K_FREE
  output:
$OUT_K
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [K: af-domain-collision] (rc=$RC_K)"
fi

# --- L. Cross-domain: AF-0052 при RT-0052 в baseline → clean (разные домены) ---
L_DIR="$WORK/L"
mkdir -p "$L_DIR/docs/adr"
cd "$L_DIR" || exit 2
git init -q -b main
git config user.email "test@test"
git config user.name "test"
git remote add origin "$L_DIR"
echo "# ADR-0052: mcp slice" > docs/adr/0052-mcp-slice.md
git add . >/dev/null
git commit -q -m baseline
git checkout -q -b develop
git push -q origin develop
cat > docs/adr/AF-0052-watchdog.md <<'EOF'
# ADR-AF-0052: watchdog
EOF
(
    cd "$L_DIR" || exit 2
    git add . >/dev/null
    git commit -q -m "AF file, no collision expected"
)
OUT_L="$(run_validate "$L_DIR")"
RC_L=$?
GOT_L_CLEAN="$(printf '%s' "$OUT_L" | grep -c '^validate_adr_namespace: clean' || true)"
if [ "$RC_L" -ne 0 ] || [ "${GOT_L_CLEAN:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [L: cross-domain-no-collision]: AF-0052 при RT-0052 в baseline → clean.
  rc=$RC_L
  clean-matches=$GOT_L_CLEAN
  output:
$OUT_L
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [L: cross-domain-no-collision] (rc=$RC_L)"
fi

# --- M. AF fresh: AF-0053 в baseline пустом → clean ---
M_DIR="$WORK/M"
mkdir -p "$M_DIR/docs/adr"
cd "$M_DIR" || exit 2
git init -q -b main
git config user.email "test@test"
git config user.name "test"
git remote add origin "$M_DIR"
echo "no adr" > README.md
git add . >/dev/null
git commit -q -m empty
cat > docs/adr/AF-0001-fresh.md <<'EOF'
# ADR-AF-0001: fresh
EOF
git add . >/dev/null
git commit -q -m fresh
OUT_M="$(run_validate "$M_DIR" --ref HEAD~1)"
RC_M=$?
GOT_M_CLEAN="$(printf '%s' "$OUT_M" | grep -c '^validate_adr_namespace: clean' || true)"
if [ "$RC_M" -ne 0 ] || [ "${GOT_M_CLEAN:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [M: af-fresh-empty-baseline]: AF-0001 при пустом baseline → clean.
  rc=$RC_M
  clean-matches=$GOT_M_CLEAN
  output:
$OUT_M
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [M: af-fresh-empty-baseline] (rc=$RC_M)"
fi

# --- N. amendment-pattern: AMEND:14 не путается с RT:14 (issue #2601) ---
# Baseline: 0014-process.md + 0014-amendment-1-label-conflict.md.
# По СТАРОЙ логике (extract_keys без AMEND-ветки) это была бы коллизия RT:14.
# По НОВОЙ — это AMEND:14 (отдельный от RT:14), поэтому pre-PR guard
# проходит clean.
N_DIR="$WORK/N"
mkdir -p "$N_DIR/docs/adr"
cd "$N_DIR" || exit 2
git init -q -b main
git config user.email "test@test"
git config user.name "test"
git remote add origin "$N_DIR"
echo "# ADR-0014: process" > docs/adr/0014-process.md
echo "# ADR-0014 Amendment 1: label conflict" > docs/adr/0014-amendment-1-label-conflict.md
git add . >/dev/null
git commit -q -m "baseline: 0014-process + 0014-amendment"
git checkout -q -b develop
git push -q origin develop
# Новый ADR с другим slug'ом, который НЕ пересекается по номерам
cat > docs/adr/0015-fresh.md <<'EOF'
# ADR-0015: fresh
EOF
(
    cd "$N_DIR" || exit 2
    git add . >/dev/null
    git commit -q -m "fresh 0015 ADR"
)
OUT_N="$(run_validate "$N_DIR")"
RC_N=$?
GOT_N_CLEAN="$(printf '%s' "$OUT_N" | grep -c '^validate_adr_namespace: clean' || true)"
if [ "$RC_N" -ne 0 ] || [ "${GOT_N_CLEAN:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [N: amendment-not-rt-collision]: 0014-process + 0014-amendment не должны быть коллизией (AMEND:14 vs RT:14 — разные namespace).
  rc=$RC_N
  clean-matches=$GOT_N_CLEAN
  output:
$OUT_N
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [N: amendment-not-rt-collision] (rc=$RC_N)"
fi

# --- O. amendment-pattern: новый amendment под занятым AMEND:N → collision ---
# Baseline: 0022-amendment-1-a.md + 0022-amendment-2-b.md (AMEND:22 занят × 2,
# но они СЧИТАЮТСЯ как один ключ AMEND:22 с точки зрения duplicate-collision:
# ключ один, файлов много → коллизия в AMEND-домене).
# Pre-PR guard должен завалить, если PR добавляет ещё один 0022-amendment-*.
O_DIR="$WORK/O"
mkdir -p "$O_DIR/docs/adr"
cd "$O_DIR" || exit 2
git init -q -b main
git config user.email "test@test"
git config user.name "test"
git remote add origin "$O_DIR"
echo "# ADR-0022 Amendment 1: a" > docs/adr/0022-amendment-1-a.md
echo "# ADR-0022 Amendment 2: b" > docs/adr/0022-amendment-2-b.md
git add . >/dev/null
git commit -q -m "baseline: AMEND:22 уже занят"
git checkout -q -b develop
git push -q origin develop
# Новый PR пытается добавить ещё один amendment под тем же 0022
cat > docs/adr/0022-amendment-3-c.md <<'EOF'
# ADR-0022 Amendment 3: c
EOF
(
    cd "$O_DIR" || exit 2
    git add . >/dev/null
    git commit -q -m "another 0022 amendment — collision"
)
OUT_O="$(run_validate "$O_DIR")"
RC_O=$?
GOT_O_AMEND="$(printf '%s' "$OUT_O" | grep -cE 'AMEND:22.*занято' || true)"
GOT_O_COLLIDE="$(printf '%s' "$OUT_O" | grep -cE 'ADR namespace collision detected' || true)"
if [ "$RC_O" -ne 1 ] || [ "${GOT_O_AMEND:-0}" -lt 1 ] || [ "${GOT_O_COLLIDE:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [O: amendment-collision]: новый 0022-amendment при занятом AMEND:22 → collision.
  rc=$RC_O
  amend-matches=$GOT_O_AMEND
  collide-msg-matches=$GOT_O_COLLIDE
  output:
$OUT_O
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [O: amendment-collision] (rc=$RC_O)"
fi

# --- P. --full режим: clean (нет дублей в каталоге) ---
P_DIR="$WORK/P"
mkdir -p "$P_DIR/docs/adr"
cd "$P_DIR" || exit 2
git init -q -b main
git config user.email "test@test"
git config user.name "test"
git remote add origin "$P_DIR"
echo "# ADR-0001: a" > docs/adr/0001-a.md
echo "# ADR-0002: b" > docs/adr/0002-b.md
git add . >/dev/null
git commit -q -m "baseline: 2 ADR (clean)"
git checkout -q -b develop
git push -q origin develop
# Создаём ADR в develop (но без коллизий)
echo "# ADR-0003: c" > docs/adr/0003-c.md
(
    cd "$P_DIR" || exit 2
    git add . >/dev/null
    git commit -q -m "0003 in develop"
)
OUT_P="$(run_validate "$P_DIR" --full)"
RC_P=$?
GOT_P_FULL_CLEAN="$(printf '%s' "$OUT_P" | grep -cE 'validate_adr_namespace: --full clean' || true)"
if [ "$RC_P" -ne 0 ] || [ "${GOT_P_FULL_CLEAN:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [P: --full-clean]: --full в репо без коллизий → clean.
  rc=$RC_P
  full-clean-matches=$GOT_P_FULL_CLEAN
  output:
$OUT_P
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [P: --full-clean] (rc=$RC_P)"
fi

# --- Q. --full режим: коллизия (AF:52 × 2 файлов в каталоге) ---
Q_DIR="$WORK/Q"
mkdir -p "$Q_DIR/docs/adr"
cd "$Q_DIR" || exit 2
git init -q -b main
git config user.email "test@test"
git config user.name "test"
git remote add origin "$Q_DIR"
echo "# ADR-AF-0052: a" > docs/adr/AF-0052-a.md
echo "# ADR-AF-0052: b" > docs/adr/AF-0052-b.md
git add . >/dev/null
git commit -q -m "baseline: AF:52 × 2 (collision)"
git checkout -q -b develop
git push -q origin develop
OUT_Q="$(run_validate "$Q_DIR" --full)"
RC_Q=$?
GOT_Q_COLLIDE="$(printf '%s' "$OUT_Q" | grep -cE '^  AF:52 ' || true)"
GOT_Q_TITLE="$(printf '%s' "$OUT_Q" | grep -cE 'ERROR: --full mode found ADR namespace collision' || true)"
if [ "$RC_Q" -ne 1 ] || [ "${GOT_Q_COLLIDE:-0}" -lt 1 ] || [ "${GOT_Q_TITLE:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [Q: --full-collision]: AF:52 × 2 в develop → --full exit 1 + AF:52 line + ERROR header.
  rc=$RC_Q
  af-line-matches=$GOT_Q_COLLIDE
  title-matches=$GOT_Q_TITLE
  output:
$OUT_Q
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [Q: --full-collision] (rc=$RC_Q)"
fi

# --- R. --full режим: amendment НЕ считается коллизией с RT (тот же 0022) ---
R_DIR="$WORK/R"
mkdir -p "$R_DIR/docs/adr"
cd "$R_DIR" || exit 2
git init -q -b main
git config user.email "test@test"
git config user.name "test"
git remote add origin "$R_DIR"
echo "# ADR-0022: process" > docs/adr/0022-process.md
echo "# ADR-0022 Amendment 1: a" > docs/adr/0022-amendment-1-a.md
git add . >/dev/null
git commit -q -m "0022 process + amendment (разные namespace)"
git checkout -q -b develop
git push -q origin develop
OUT_R="$(run_validate "$R_DIR" --full)"
RC_R=$?
GOT_R_CLEAN="$(printf '%s' "$OUT_R" | grep -cE 'validate_adr_namespace: --full clean' || true)"
if [ "$RC_R" -ne 0 ] || [ "${GOT_R_CLEAN:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [R: --full-amendment-not-rt-collision]: 0022-process + 0022-amendment в --full → clean (AMEND:22 vs RT:22 — разные).
  rc=$RC_R
  full-clean-matches=$GOT_R_CLEAN
  output:
$OUT_R
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [R: --full-amendment-not-rt-collision] (rc=$RC_R)"
fi

# --- Итог ---
echo
echo "=== validate_adr_namespace test summary ==="
echo "PASS: $pass_count"
echo "FAIL: $fail_count"
if [ "$fail_count" -gt 0 ]; then
    printf '%s' "$fail_log"
    exit 1
fi
echo "ALL OK"
exit 0
