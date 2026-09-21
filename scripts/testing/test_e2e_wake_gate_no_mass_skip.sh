#!/usr/bin/env bash
# Гард: wake-gate не имеет права увести весь прогон в SKIP.
#
# ЗАЧЕМ (поймано 22.09.2026 на живом прогоне 35664554084, до того как
# испортило ночной марафон).
#
# ADR-0027 §5.2 даёт гард: шаг с expect="wake-gated" пропускается (SKIP), если
# cold-start wake-gate не пройден. Пробу делает run_wake_gate_preflight, и
# зовёт она wake_gate_cleared_since "$E2E_RUN_BEFORE", то есть
# `docker logs --since <старт прогона>`. В момент старта это окно ПУСТО по
# определению — значит ответ ВСЕГДА «cold-start NOT cleared».
#
# Пока сам preflight был мёртвым кодом (читал SCENARIO_FILE до парсинга
# аргументов), это никого не трогало. Как только его починили, гард начал
# резать всё подряд: в night-marathon 101 шаг из 125 не имеет явного expect и
# начинается с «Робот», а classify_step_expect авто-повышает такие шаги до
# wake-gated. Прогон отдал бы 101 SKIP, не покраснел бы — и не проверил
# НИЧЕГО. По ADR-0018 это худший исход: не честный FAIL, а тишина, которую
# читают как «нормально».
#
# Почему нельзя «просто пропускать»: акцепт, который откроет гейт, берётся
# ровно из отыгранных шагов. Пропустив первый wake-gated шаг, мы гарантируем,
# что следующая проба тоже будет пустой — и так до конца акта. Каскад.
#
# Контракт, который здесь закреплён:
#   1. Перед пропуском харнесс ПЕРЕПРАШИВАЕТ гейт (к этому моменту предыдущие
#      шаги уже отдали свои «✅ ПРИНЯТО ... Робот»).
#   2. ПЕРВЫЙ wake-gated шаг прогона играется всегда — он и есть cold-start
#      проба (тот же довод, что в single-text ветке preflight'а).
#   3. SKIP остаётся возможен — но только для ПОСЛЕДУЮЩИХ шагов, когда проба
#      уже израсходована и гейт всё равно не открылся. Фича ADR-0027 не
#      выключена, она перестала быть самоисполняющейся.
#
# Тест офлайновый: читает текст харнесса и сценарии, робота не требует.
#
# Запуск: bash scripts/testing/test_e2e_wake_gate_no_mass_skip.sh
# Exit:   0 = PASS, 1 = гард снова может обнулить прогон.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HARNESS="$REPO_ROOT/.github/workflows/scripts/e2e_voice_test.sh"
WAKE="$REPO_ROOT/.github/workflows/scripts/e2e_voice_wake_gate.sh"

for f in "$HARNESS" "$WAKE"; do
    [ -f "$f" ] || { printf '❌ отсутствует: %s\n' "$f"; exit 1; }
done

fails=0
ok()  { printf '  ✅ %s\n' "$1"; }
bad() { printf '  ❌ %s\n' "$1"; fails=$((fails + 1)); }

# ---------------------------------------------------------------------------
# CHECK 1. Перепроба перед пропуском есть, и она внутри skip-гарда.
# ---------------------------------------------------------------------------
printf 'CHECK 1: перед SKIP харнесс перепрашивает гейт\n'
guard="$(awk '/if \[ "\$expect" = "wake-gated" \]/,/^    fi$/' "$HARNESS")"
if [ -z "$guard" ]; then
    bad "не нашёл skip-гард wake-gated в харнессе — его убрали?"
else
    if printf '%s' "$guard" | grep -q 'wake_gate_cleared_since'; then
        ok "внутри гарда есть перепроба wake_gate_cleared_since"
    else
        bad "в skip-гарде НЕТ перепробы: решение принимается по разовому preflight'у, окно которого пусто по определению → массовый SKIP"
    fi
    if printf '%s' "$guard" | grep -q 'WAKE_GATE_PROBE_SPENT'; then
        ok "есть каскад-гард WAKE_GATE_PROBE_SPENT (первый шаг играется)"
    else
        bad "нет каскад-гарда: пропустив первый wake-gated шаг, акцепта не получить, и весь акт уйдёт в SKIP"
    fi
    if printf '%s' "$guard" | grep -q 'emit_step "${label} SKIP wake-gate-cold-start"'; then
        ok "SKIP по-прежнему возможен — фича ADR-0027 не выключена"
    else
        bad "SKIP исчез совсем: гард ADR-0027 §5.2 вырезан, а не исправлен"
    fi
fi

# ---------------------------------------------------------------------------
# CHECK 2. Флаг пробы инициализирован до первого использования.
# ---------------------------------------------------------------------------
printf 'CHECK 2: WAKE_GATE_PROBE_SPENT инициализирован\n'
if grep -qE '^WAKE_GATE_PROBE_SPENT=0' "$HARNESS"; then
    ok "инициализация есть (под set -u иначе упало бы на первом шаге)"
else
    bad "нет строки WAKE_GATE_PROBE_SPENT=0 на верхнем уровне"
fi

# ---------------------------------------------------------------------------
# CHECK 3. Масштаб: сколько шагов зависит от этого гарда.
#          Тест не требует «мало wake-gated шагов» — он лишь не даёт
#          забыть, что гард распространяется почти на весь марафон.
# ---------------------------------------------------------------------------
printf 'CHECK 3: масштаб влияния гарда на сценарии\n'
python3 - "$REPO_ROOT" <<'PY'
import glob, json, os, re, sys

for stream in (sys.stdout, sys.stderr):
    try:
        stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

root = sys.argv[1]
total = auto = 0
pattern = os.path.join(root, ".github", "e2e", "scenarios", "**", "*.json")
for path in sorted(glob.glob(pattern, recursive=True)):
    base = os.path.basename(path)
    if "acceptance" in base or "manifest" in base:
        continue
    try:
        data = json.load(open(path, encoding="utf-8"))
    except json.JSONDecodeError:
        continue
    for step in data.get("steps", []) or []:
        if not isinstance(step, dict):
            continue
        text = step.get("text") or ""
        if not text:
            continue
        total += 1
        # classify_step_expect: пустой expect + wake-префикс → wake-gated.
        if not step.get("expect") and re.match(r"^\s*(Робот|Робокс)", text, re.I):
            auto += 1
share = (100.0 * auto / total) if total else 0.0
print("  под авто-wake-gated: %d из %d реплик (%.0f%%)" % (auto, total, share))
if share > 50:
    print("  ⚠️  больше половины сценариев зависит от этого гарда — "
          "любая ошибка в нём обнуляет прогон целиком, а не портит один шаг")
PY

printf '\n'
if [ "$fails" -ne 0 ]; then
    printf 'wake-gate no mass skip: FAIL (%d)\n' "$fails"
    exit 1
fi
printf 'wake-gate no mass skip: PASS\n'
