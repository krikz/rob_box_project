#!/bin/bash
# ============================================================================
# kanban-retro-create.sh — dedup-guard обёртка над `hermes kanban create`
# для РЕТРО-КАРТОЧЕК LLM-кронов процесса (архитектор-надзор 5c96a6eedf93,
# падаван-вахта 5a070bf3ed3e и любые другие, кто создаёт «ретро: ...» карточки).
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/kanban-retro-create.sh
# На хост раскладывается через `bash <repo>/scripts/agent_flow/install.sh`
# (hardlink в ~/.hermes/scripts/, profiles/agent-flow|architect|devops/scripts/).
#
# Ретро 13.08 (t_35ff29f1): надзор одним тиком создал ДВЕ одинаковые
# ретро-карточки (t_1e1fc3f0 + t_da3e0bd5, разница 4 сек) — повторный
# LLM-вызов `kanban create` в одном тике без проверки существующих карточек.
# Дубль сжёг прогон воркера (archived без complete), у дубля нет своего PR.
#
# ADR-0078 (follow-up ADR-0049, issue #2159): nightly-review создавал
# несколько карточек за тик (1 дайджест + N компонентных) с разными
# --key (nightly-review-<DATE> vs component-review-<slug>-<DATE>), и две
# параллельные карточки в одном окне обходили слой 1 (pre-check) и слой 2
# (idempotency-key). Добавлен слой 4 — **issue-label guard**: перед create
# для ключей `nightly-review-<YYYY-WW>` дополнительно проверяем, есть ли
# в текущей ISO-неделе открытый GitHub issue с label `nightly-review`.
# Если есть — SKIP (не плодим вторую канбан-карточку).
#
# Guard (четыре независимых слоя):
#   1. PRE-CHECK (механический): перед create читаем `kanban list --json` и
#      ищем НЕ-archived карточку, у которой в body есть маркер
#      `ретро-key: <KEY>` (тот же ключ) ЛИБО нормализованный title совпадает
#      с target. Совпадение → SKIP, create НЕ выполняется, exit 0.
#   2. IDEMPOTENCY-KEY (атомарный): create всегда идёт с
#      `--idempotency-key "retro:<KEY>"` — даже если два create вызовутся
#      одновременно (гонка в одном тике), второй вернёт id первого.
#   3. МАРКЕР В BODY: скрипт сам дописывает в конец body строку
#      `ретро-key: <KEY>` → следующий тик (та же аномалия, тот же --key)
#      находит карточку на шаге 1.
#   4. ISSUE-LABEL GUARD (ADR-0078): для ключей `nightly-review-*` дополнительно
#      проверяем `gh issue list --label nightly-review --state open --limit 5`;
#      если хотя бы одно открытое issue создано в текущей ISO-неделе
#      (`date -d "last monday" +%s` … now) — SKIP.
#      Покрывает гонку двух тиков с разными ключами в одном окне (issue #2159).
#
# КЛЮЧ = стабильный slug аномалии (БЕЗ дат/времён в ключе!), например
# `--key e2e-stop-build-runners`. Для одной и той же аномалии — один и тот же
# ключ во всех тиках; тогда повторные тики не плодят дубли.
# Исключение: nightly-review использует ISO-неделю (`nightly-review-<YYYY-WW>`),
# что вместе со слоем 4 даёт устойчивую дедупликацию на уровне недели.
#
# Вывод (stdout, готов для вставки в отчёт крона):
#   CREATED <id>                 — карточка создана (или возвращена существующая
#                                   по idempotency-key)
#   SKIP <id> (existing)         — карточка уже есть, create не выполнялся
#   SKIP (gh-issue-label guard)  — открытое issue с тем же label за ISO-неделю
#   WOULD_CREATE idempotency_key=... — режим --dry-run, create не выполнялся
#   ERROR ... (stderr, exit 2/3) — ошибка
#
# Exit codes: 0 = created/skipped (id на stdout), 2 = usage, 3 = kanban error.
# ============================================================================
set -euo pipefail

KANBAN_BIN="${KANBAN_BIN:-hermes}"
BOARD="robbox"
TITLE=""
BODY=""
ASSIGNEE=""
KEY=""
MAX_RUNTIME="1800"
SKILLS=()
DRY_RUN=false

usage() {
    cat <<'EOF'
usage: kanban-retro-create.sh --title TITLE --body BODY --assignee PROFILE
       [--key KEY] [--skill SKILL]... [--max-runtime SECS] [--board robbox]
       [--dry-run]

  --title TITLE        Заголовок ретро-карточки (обязательно)
  --body BODY          Тело карточки; скрипт допишет маркер 'ретро-key: <KEY>'
  --assignee PROFILE   Исполнитель (обязательно)
  --key KEY            Стабильный slug аномалии (БЕЗ дат/времён). Если не задан —
                       берётся из title. Один и тот же ключ для одной аномалии.
  --skill SKILL        Скил воркера (повторяемый, pass-through)
  --max-runtime SECS   Лимит рантайма воркера (default 1800)
  --board BOARD        Канбан-борд (default robbox)
  --dry-run            Только pre-check + вычисление ключа, create не вызывать
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --board)       BOARD="${2:?}"; shift 2 ;;
        --title)       TITLE="${2:?}"; shift 2 ;;
        --body)        BODY="${2:?}"; shift 2 ;;
        --assignee)    ASSIGNEE="${2:?}"; shift 2 ;;
        --key)         KEY="${2:?}"; shift 2 ;;
        --skill)       SKILLS+=("$2"); shift 2 ;;
        --max-runtime) MAX_RUNTIME="${2:?}"; shift 2 ;;
        --dry-run)     DRY_RUN=true; shift ;;
        -h|--help)     usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

[ -n "$TITLE" ]    || { echo "ERROR: --title required" >&2; exit 2; }
[ -n "$BODY" ]     || { echo "ERROR: --body required" >&2; exit 2; }
[ -n "$ASSIGNEE" ] || { echo "ERROR: --assignee required" >&2; exit 2; }

# --- skill validation УДАЛЁН (ADR-0023, 22.08) ------------------------------
# Раньше здесь был guard (ретро 19.08 t_8c8c7c69 / commit 6ceb43eb), который
# проверял --skill ТОЛЬКО в плоском skills/<name>/SKILL.md и резал категорийные
# скиллы (skills/<category>/<skill>/SKILL.md) с exit 2. Модель была неверной:
# dispatcher hermes-agent находит скиллы recursive через iter_skill_index_files,
# а fail-fast при unknown skill на `kanban create` уже даёт vendor-патч
# scripts/agent_flow/vendor/hermes-agent-skill-validation.patch
# (_validate_skills_for_assignee). Bash-guard был карго-культом, дублирующим
# эту логику с плоской (неверной) моделью — удалён по варианту A из ADR-0023.

# --- stable key -------------------------------------------------------------
# KEY: явный --key или slugify(title). slugify через python3 — корректно
# работает с кириллицей (lower, regex), в отличие от tr/awk.
if [ -z "$KEY" ]; then
    KEY="$(printf '%s' "$TITLE" | python3 -c '
import re, sys
s = sys.stdin.read().strip().lower()
s = re.sub(r"[^a-zа-яё0-9]+", "-", s).strip("-")
print((s[:60]) or "card")
')"
fi

IDEM_KEY="retro:${KEY}"

# --- pre-check (механический dedup) -----------------------------------------
# Читаем список карточек ОДИН раз и ищем: (a) маркер `ретро-key: <KEY>` в body,
# (b) нормализованный title == нормализованному target. Archived — пропускаем
# (archived карточка = мёртвая, для новой работы можно создавать, как в триаже).
# Если `hermes kanban list` падает (известный баг "closed database") — список
# пуст, полагаемся на idempotency-key (слой 2).
LIST_JSON="$("$KANBAN_BIN" kanban --board "$BOARD" list --json 2>/dev/null || echo '[]')"

MATCH="$(printf '%s' "$LIST_JSON" | python3 -c '
import json, re, sys
key = sys.argv[1]
target = sys.argv[2]

def norm(s):
    return re.sub(r"\s+", " ", (s or "").strip().lower())

target_norm = norm(target)
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
except Exception:
    tasks = []
marker = "ретро-key: " + key
for t in tasks:
    if t.get("status") == "archived":
        continue
    body = t.get("body") or ""
    if marker in body:
        print(t.get("id", "?"))
        sys.exit(0)
    if norm(t.get("title")) == target_norm:
        print(t.get("id", "?"))
        sys.exit(0)
' "$KEY" "$TITLE")"

if [ -n "$MATCH" ]; then
    echo "SKIP $MATCH (existing card, key=${KEY}; дубль не создаю)"
    exit 0
fi

# --- слой 4 (ADR-0078): issue-label guard для nightly-review-* -------------
# Ретро issue #2159: две карточки (nightly-review-<DATE> и
# component-review-<slug>-<DATE>) от разных тиков одного окна обходили
# слои 1-3 (разные --key → idempotency-key тоже разный). Теперь для любого
# ключа с префиксом `nightly-review-` дополнительно проверяем, есть ли
# открытый GitHub issue с label `nightly-review`, созданный в текущей
# ISO-неделе. Если есть — SKIP, карточка не нужна: дайджест уже ушёл
# через issue, читать Шифу удобнее там.
# Пустой GH_REPO / упавший gh / нет метки — fail-open (пропускаем слой 4,
# полагаемся на 1-3). Это обратная сторона observability: лучше дубль,
# который поймает merge-gate / следующий тик, чем false-negative в ночи.
case "$KEY" in
    nightly-review-*|component-review-*)
        if [ -n "${GH_REPO:-}" ] && command -v "$GH_BIN" >/dev/null 2>&1; then
            # Начало текущей ISO-недели (понедельник 00:00 UTC).
            # %u = 1..7 (Пн..Вс); отнимаем (%u - 1) чтобы получить понедельник.
            _dow=$(date -u +%u)
            _week_start=$(date -u -d "$(date -u +%Y-%m-%d) -$((_dow - 1)) days" +%s 2>/dev/null \
                || date -u +%s)
            _label_issues=$("$GH_BIN" issue list --repo "$GH_REPO" \
                --label nightly-review --state open --limit 5 \
                --json number,createdAt 2>/dev/null || echo '[]')
            _week_match=$(printf '%s' "$_label_issues" | _WEEK_START="$_week_start" python3 -c '
import json, os, sys
try:
    items = json.load(sys.stdin)
except Exception:
    sys.exit(0)
week_start = int(os.environ.get("_WEEK_START", "0"))
for it in items:
    ts = (it.get("createdAt") or "").strip()
    if not ts:
        continue
    try:
        from datetime import datetime, timezone
        # GitHub createdAt = "2026-09-08T12:00:12Z" (UTC) или с миллисекундами.
        # Нормализуем: Z → +00:00, обрезаем миллисекунды.
        ts_norm = ts.replace("Z", "+00:00")
        # Обрабатываем и вариант с ".123Z", и без.
        try:
            t = datetime.fromisoformat(ts_norm)
        except ValueError:
            # Fallback: первые 19 символов без TZ, добавляем UTC вручную.
            t = datetime.strptime(ts[:19], "%Y-%m-%dT%H:%M:%S").replace(tzinfo=timezone.utc)
        if int(t.timestamp()) >= week_start:
            print(it.get("number", "?"))
            sys.exit(0)
    except Exception:
        continue
sys.exit(0)
')
            if [ -n "$_week_match" ]; then
                echo "SKIP (gh-issue-label guard: nightly-review issue #${_week_match} открыт за текущую ISO-неделю)"
                exit 0
            fi
        fi
        ;;
esac

# --- create -----------------------------------------------------------------
# Маркер в конец body — чтобы следующий тик нашёл карточку на pre-check.
BODY_WITH_MARKER="${BODY}
ретро-key: ${KEY}"

if $DRY_RUN; then
    echo "WOULD_CREATE idempotency_key=${IDEM_KEY} assignee=${ASSIGNEE} board=${BOARD}"
    exit 0
fi

SKILL_ARGS=()
for s in "${SKILLS[@]}"; do
    SKILL_ARGS+=(--skill "$s")
done

# title — ПОЗИЦИОННЫЙ аргумент `hermes kanban create` (фикс 13.08 надзор:
# `--title` CLI не знает → exit 3, карточки не создавались через обёртку)
CREATE_OUT="$("$KANBAN_BIN" kanban --board "$BOARD" create "$TITLE" \
    --body "$BODY_WITH_MARKER" \
    --assignee "$ASSIGNEE" \
    --max-runtime "$MAX_RUNTIME" \
    --idempotency-key "$IDEM_KEY" \
    "${SKILL_ARGS[@]}" \
    --json 2>&1)" || {
        echo "ERROR: kanban create failed (exit $?): $CREATE_OUT" >&2
        exit 3
    }

TASK_ID="$(printf '%s' "$CREATE_OUT" | python3 -c '
import json, sys
try:
    print(json.load(sys.stdin).get("id", ""))
except Exception:
    print("")
')"

if [ -n "$TASK_ID" ]; then
    echo "CREATED $TASK_ID (key=${KEY}, assignee=${ASSIGNEE})"
else
    echo "ERROR: kanban create вернул не-JSON: $CREATE_OUT" >&2
    exit 3
fi
