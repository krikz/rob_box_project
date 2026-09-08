#!/bin/bash
# skills_usage_report.sh — ежедневная метрика: какие карточки реально
# использовали skills (через секцию `## Skills` в body), какие — нет.
#
# ADR-0080 / issue #2162: Шифу жалуется, что воркеры не используют скиллы.
# Этот скрипт даёт цифру: сколько карточек за окно имели секцию
# `## Skills` в body и сколько — нет. Если < threshold% — alerting в
# cron-лог (а не Slack/Telegram — задача говорит про метрику).
#
# Использование:
#   bash scripts/agent_flow/skills_usage_report.sh --since 1d
#   bash scripts/agent_flow/skills_usage_report.sh --since 7d
#   bash scripts/agent_flow/skills_usage_report.sh --since 1d --json
#   bash scripts/agent_flow/skills_usage_report.sh --help
#
# Exit codes:
#   0 — отчёт напечатан
#   2 — usage error
#
# Требования:
#   * hermes CLI в PATH (или $HERMES_BIN override)
#   * python3 (стандартный инструмент hermes-agent)
#
# Парсинг body: parse_body_skills_section из lib_agent_flow_common.sh
# (для согласованности с agent-flow-triage.sh — тесты ловят расхождения).
# Подсчёт делается в python (быстрее и проще с многострочным body, чем
# bash + awk + TSV).

set -u

# --- опции -----------------------------------------------------------------
SINCE="1d"
JSON_OUT=false
KANBAN_BOARD="${KANBAN_BOARD:-default}"
HERMES_BIN="${HERMES_BIN:-hermes}"
STATUS_FILTER="done"
THRESHOLD=50

usage() {
    cat <<'EOF'
skills_usage_report.sh — метрика использования skills в карточках kanban.

Использование:
  bash skills_usage_report.sh [--since <duration>] [--json] [--status STATUS]
                             [--board BOARD] [--threshold N]

Опции:
  --since DURATION   Период (default: 1d, поддерживает 1d/7d/30d/24h)
  --json             Машинный вывод (для дашбордов)
  --status STATUS    Статус карточек: done | cancelled | archived | all
                     (default: done)
  --board BOARD      Kanban board slug (default: default)
  --threshold N      Пороговая доля использования (0-100). Если usage%
                     < threshold — печатает ALERT line. Default: 50.
  --help             Эта справка

Выход (text):
  skills_usage: total=N with=M without=K pct=P (status=...)
  top skills: skill-a (N), skill-b (M), ...
  alert: low-skill-usage pct=P < threshold=T

Выход (json):
  {"total": N, "with": M, "without": K, "pct": P, "top_skills": {...}}
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --since) SINCE="${2:-}"; shift 2 ;;
        --json) JSON_OUT=true; shift ;;
        --status) STATUS_FILTER="${2:-done}"; shift 2 ;;
        --board) KANBAN_BOARD="${2:-default}"; shift 2 ;;
        --threshold) THRESHOLD="${2:-50}"; shift 2 ;;
        --help|-h) usage; exit 0 ;;
        *) echo "ERROR: unknown flag $1" >&2; usage; exit 2 ;;
    esac
done

# --- lib sourcing ---------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB="$SCRIPT_DIR/lib_agent_flow_common.sh"
if [ ! -f "$LIB" ]; then
    echo "ERROR: lib_agent_flow_common.sh not found at $LIB" >&2
    exit 2
fi
# shellcheck disable=SC1091
. "$LIB"

# Suppress _af_log noise in report mode
_af_log() { :; }

# --- собираем данные -------------------------------------------------------
if ! command -v "$HERMES_BIN" >/dev/null 2>&1; then
    echo "ERROR: hermes CLI not found (HERMES_BIN=$HERMES_BIN)" >&2
    exit 2
fi

# Достаём список карточек. hermes kanban list --json — контракт стабильный
# уже несколько релизов. Если поменяется — этот скрипт тоже придётся
# обновить, поэтому exit code 2 при пустом результате (а не 1).
RAW_LIST="$(HERMES_BIN="$HERMES_BIN" "$HERMES_BIN" kanban --board "$KANBAN_BOARD" list \
    --status "$STATUS_FILTER" --since "$SINCE" --json 2>/dev/null || true)"

if [ -z "$RAW_LIST" ]; then
    if $JSON_OUT; then
        printf '{"total":0,"with":0,"without":0,"pct":0,"top_skills":{},"error":"empty_list","since":"%s","status":"%s","board":"%s","threshold":%s}\n' \
            "$SINCE" "$STATUS_FILTER" "$KANBAN_BOARD" "$THRESHOLD"
    else
        echo "skills_usage: total=0 with=0 without=0 pct=0 (status=$STATUS_FILTER since=$SINCE board=$KANBAN_BOARD)"
        echo "  note: empty list returned by hermes kanban list — check HERMES_BIN/KANBAN_BOARD"
    fi
    exit 0
fi

# --- парсим в python -------------------------------------------------------
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# python делает всю обработку: JSON parse + проверка ## Skills +
# подсчёт top skills + emit строк для bash. parse_body_skills_section
# в bash тут не зовём — вместо этого парсим секцию напрямую в python
# (тот же контракт: heading "## Skills", нумерованные/bullet списки,
# dedup, первое слово после маркера — это skill name).
#
# python3 - <script> не позволяет передавать argv, поэтому передаём
# JSON через env-переменную CARDS_JSON (надёжно для бинарных данных)
# и пишем результат в STATS_OUT из python через file open().
CARDS_JSON="$RAW_LIST" STATS_OUT="$WORK/stats.tsv" python3 >/dev/null <<'PYEOF'
import json, os, sys, re, collections

raw = os.environ.get('CARDS_JSON', '')
try:
    data = json.loads(raw)
except Exception as e:
    sys.exit(f"error: cannot parse JSON: {e}")

# Разные версии hermes кладут карточки по-разному:
#   {"tasks": [...]} (kanban list)
#   {"cards": [...]} (legacy)
#   [...] (плоский массив)
items = []
if isinstance(data, list):
    items = data
elif isinstance(data, dict):
    for key in ('tasks', 'cards', 'items', 'data'):
        if key in data and isinstance(data[key], list):
            items = data[key]
            break

items = [c for c in items if isinstance(c, dict)]

# Парсим секцию ## Skills — соответствует parse_body_skills_section в bash.
SKILLS_HEADING_RE = re.compile(r'^##\s+Skills\b', re.MULTILINE)
LIST_ITEM_RE = re.compile(r'^\s*(?:\d+[.)]|[-*])\s+(\S+)', re.MULTILINE)
ANY_HEADING_RE = re.compile(r'^##\s+\S+', re.MULTILINE)

def parse_skills(body):
    if not body:
        return []
    m = SKILLS_HEADING_RE.search(body)
    if not m:
        return []
    # Ищем конец секции: следующий ## heading или конец body.
    section_start = m.end()
    section_end = len(body)
    for h in ANY_HEADING_RE.finditer(body, section_start):
        section_end = h.start()
        break
    section = body[section_start:section_end]
    # Достаём skill names, dedup в порядке появления.
    seen = set()
    out = []
    for item in LIST_ITEM_RE.finditer(section):
        # Берём первое слово (до пробела/em-dash/colon).
        name = item.group(1).split()[0]
        # Trim non-word chars (em-dash и т.п. остаются внутри []-класса).
        name = re.split(r'[\s—–:|]+', name)[0]
        if not name or name in seen:
            continue
        seen.add(name)
        out.append(name)
    return out

total = 0
with_skills = 0
without_skills = 0
top = collections.Counter()

for c in items:
    body = c.get('body') or c.get('description') or ''
    total += 1
    skills = parse_skills(body)
    if skills:
        with_skills += 1
        for s in skills:
            top[s] += 1
    else:
        without_skills += 1

pct = round((with_skills / total * 100), 1) if total else 0.0

# Output: tab-separated (для bash). 3 строки: stats, json, top10.
# stats line: total\twith\twithout\tpct
# top_skills JSON dict (one line)
# top lines: count\tname (one per line)
print(f"{total}\t{with_skills}\t{without_skills}\t{pct}")
print(json.dumps(dict(top.most_common(10)), ensure_ascii=False))
for name, count in top.most_common(10):
    print(f"{count}\t{name}")

# Дублируем в STATS_OUT для bash-парсинга (вдруг stdout уже занят).
out_path = os.environ.get('STATS_OUT', '/tmp/skills_usage_stats.tsv')
with open(out_path, 'w', encoding='utf-8') as f:
    f.write(f"{total}\t{with_skills}\t{without_skills}\t{pct}\n")
    f.write(json.dumps(dict(top.most_common(10)), ensure_ascii=False) + "\n")
    for name, count in top.most_common(10):
        f.write(f"{count}\t{name}\n")

sys.stderr.write(f"PY: total={total} with={with_skills} top={dict(top)}\n")
PYEOF

if [ ! -s "$WORK/stats.tsv" ]; then
    echo "ERROR: python stats produced no output" >&2
    exit 1
fi

# --- читаем результаты -----------------------------------------------------
STATS_LINE="$(sed -n '1p' "$WORK/stats.tsv")"
TOP_JSON="$(sed -n '2p' "$WORK/stats.tsv")"
TOTAL=$(printf '%s' "$STATS_LINE" | cut -f1)
WITH=$(printf '%s' "$STATS_LINE" | cut -f2)
WITHOUT=$(printf '%s' "$STATS_LINE" | cut -f3)
PCT=$(printf '%s' "$STATS_LINE" | cut -f4)

# --- output ---------------------------------------------------------------
if $JSON_OUT; then
    printf '{"total":%s,"with":%s,"without":%s,"pct":%s,"top_skills":%s,"since":"%s","status":"%s","board":"%s","threshold":%s}\n' \
        "$TOTAL" "$WITH" "$WITHOUT" "$PCT" "$TOP_JSON" "$SINCE" "$STATUS_FILTER" "$KANBAN_BOARD" "$THRESHOLD"
else
    echo "skills_usage: total=$TOTAL with=$WITH without=$WITHOUT pct=$PCT (status=$STATUS_FILTER since=$SINCE board=$KANBAN_BOARD threshold=$THRESHOLD)"
    # top skills (lines 3+)
    TOP_COUNT=$(($(wc -l < "$WORK/stats.tsv") - 2))
    if [ "$TOP_COUNT" -gt 0 ]; then
        echo "top skills:"
        sed -n '3,$p' "$WORK/stats.tsv" | while IFS=$'\t' read -r count name; do
            echo "  - $name ($count)"
        done
    fi
    # ALERT line — для cron-log scraper'а (легко grep'нуть).
    if awk -v pct="$PCT" -v thr="$THRESHOLD" 'BEGIN { exit !(pct+0 < thr+0) }'; then
        echo "alert: low-skill-usage pct=$PCT < threshold=$THRESHOLD"
    fi
fi
exit 0
