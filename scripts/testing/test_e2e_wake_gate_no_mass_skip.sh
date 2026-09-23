#!/usr/bin/env bash
# Гард: wake-gate не имеет права увести весь прогон в SKIP.
#
# ЗАЧЕМ (поймано 22.09.2026 на живом прогоне 35664554084, до того как
# испортило ночной марафон).
#
# ADR-0029 §2.3 даёт гард: шаг с expect="wake-gated" пропускается (SKIP), если
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
#      уже израсходована и гейт всё равно не открылся. Фича ADR-0029 не
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
        ok "SKIP по-прежнему возможен — фича ADR-0029 не выключена"
    else
        bad "SKIP исчез совсем: гард ADR-0029 §2.3 вырезан, а не исправлен"
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
# CHECK 3. Цепочка «JSON → парсер → classify_step_expect» согласована.
#
# ВАЖНО, и я сам на этом ошибся 22.09.2026. Между scenario.json и
# classify_step_expect стоит парсер, и он подставляет отсутствующему expect
# ЛИТЕРАЛ 'cycle':
#     exp = s.get('expect', 'cycle')
# а classify_step_expect при ЯВНОМ 'cycle' намеренно НЕ авто-повышает шаг
# («trust caller'а»). Авто-повышение до wake-gated срабатывает только на
# ПУСТОЙ строке. Поэтому в scenario-режиме сейчас wake-gated не появляется
# вообще, и SKIP-гард не может сработать ни на одном сценарии репозитория.
#
# Я сначала посчитал «нет expect + префикс Робот → wake-gated» и заявил
# 101 шаг из 125 под SKIP. Это было НЕВЕРНО: дефолт парсера я пропустил.
# Тест теперь проверяет саму цепочку, а не пересказывает мою ошибку.
# ---------------------------------------------------------------------------
printf 'CHECK 3: цепочка JSON → парсер → classify_step_expect согласована\n'
PARSER_DEFAULT="$(grep -oE "s\.get\('expect', *'[a-z-]*'\)" "$HARNESS" | head -1 | grep -oE "'[a-z-]*'\)" | tr -d "')")"
if [ -z "$PARSER_DEFAULT" ]; then
    bad "не нашёл дефолт expect в парсере сценария — он нужен, чтобы понимать, включено ли авто-повышение"
else
    ok "парсер подставляет отсутствующему expect: '$PARSER_DEFAULT'"
fi
# shellcheck source=/dev/null
source "$WAKE"
CLS_DEFAULT="$(classify_step_expect "$PARSER_DEFAULT" "Робот, как дела")"
CLS_EMPTY="$(classify_step_expect "" "Робот, как дела")"
printf '  classify(%s, «Робот, ...») = %s   classify(пусто, «Робот, ...») = %s\n' \
    "'$PARSER_DEFAULT'" "$CLS_DEFAULT" "$CLS_EMPTY"
if [ "$CLS_EMPTY" != "wake-gated" ]; then
    bad "авто-повышение по wake-префиксу сломано: classify(пусто) = '$CLS_EMPTY', ждали wake-gated. Гард ADR-0029 §2.3 перестал быть достижимым вообще."
else
    ok "авто-повышение по wake-префиксу живо (для пустого expect)"
fi
if [ "$CLS_DEFAULT" = "wake-gated" ]; then
    # Дефолт парсера сменили на пустую строку — авто-повышение включилось
    # для сотен шагов сразу. Это ровно тот случай, ради которого написаны
    # перепроба и каскад-гард из CHECK 1.
    printf '  ⚠️  дефолт парсера теперь даёт wake-gated: авто-повышение включено для всех шагов без expect.\n'
    printf '      Перепроба и каскад-гард (CHECK 1) обязаны быть на месте, иначе прогон уйдёт в массовый SKIP.\n'
else
    ok "дефолт парсера '$PARSER_DEFAULT' не авто-повышает — SKIP-гард применим только к явному expect"
fi
python3 - "$REPO_ROOT" "$PARSER_DEFAULT" <<'PY'
import glob, json, os, re, sys

for stream in (sys.stdout, sys.stderr):
    try:
        stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

root, parser_default = sys.argv[1], sys.argv[2]
total = explicit_wg = latent = 0
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
        if not isinstance(step, dict) or not step.get("text"):
            continue
        total += 1
        raw = step.get("expect", parser_default)
        if raw in ("wake-gated", "wake_gated"):
            explicit_wg += 1
        elif raw == "" and re.match(r"^\s*(Робот|Робокс)", step["text"], re.I):
            latent += 1

print("  шагов всего: %d | явный wake-gated: %d | авто-повышаемых сейчас: %d"
      % (total, explicit_wg, latent))
if explicit_wg == 0 and latent == 0:
    print("  ℹ️  сейчас SKIP-гард недостижим ни на одном сценарии (все шаги -> cycle/backlog).")
    print("     Правки из CHECK 1 — защита на будущее: они снимают ловушку, в которой")
    print("     гард сам себя подтверждал, а не исправляют текущую поломку прогонов.")
PY

# ---------------------------------------------------------------------------
# CHECK 4. Агрегатный GATE-1 пропускается только при РЕАЛЬНЫХ пропусках шагов.
#
# bug(живой прогон 35665111906, 22.09.2026). Условие пропуска было
# `WAKE_GATE_CLEARED != 1`. Пока preflight был мёртвым кодом, он всегда уходил
# в else-ветку и форсил WAKE_GATE_CLEARED=1 — агрегатный GATE-1 (ADR-0022,
# главный гард против smoke-false-PASS) выполнялся всегда. Как только preflight
# заработал, scenario-ветка честно сказала «cold-start NOT cleared», и гейт
# стал пропускаться на КАЖДОМ scenario-прогоне. В логе акта 1 это видно как
# E2E_GATE1_SKIP_WAKE_GATE при 9/10 OK и НУЛЕ пропущенных шагов: оправдание
# применялось там, где оправдывать было нечего.
#
# Честное правило: оправдание действует, только если то, что оправдывают,
# действительно случилось.
# ---------------------------------------------------------------------------
printf 'CHECK 4: GATE-1 пропускается только при реальных wake-gate пропусках\n'
# Ищем строку `echo "E2E_GATE1_SKIP_WAKE_GATE"`, а не первое упоминание имени:
# маркер называется и в комментарии ВЫШЕ самого if, из-за чего окно поиска
# условия уезжало и тест ругался на верную правку.
gate_cond="$(grep -n 'echo "E2E_GATE1_SKIP_WAKE_GATE"' "$HARNESS" | head -1 | cut -d: -f1)"
if [ -z "$gate_cond" ]; then
    bad "не нашёл маркер E2E_GATE1_SKIP_WAKE_GATE — блок пропуска GATE-1 убрали?"
else
    # Условие if стоит на несколько строк выше маркера. Окно узкое намеренно:
    # в файле есть другие if'ы с WAKE_GATE_* (каскад-гард в run_step), и
    # широкий поиск подхватывал их — тест ругался на верную правку.
    cond_line="$(awk -v end="$gate_cond" 'NR<end && NR>end-20 && /^[[:space:]]*if \[ "\$\{WAKE_GATE_[A-Z_]*:-0\}"/ {l=$0} END{print l}' "$HARNESS")"
    printf '  условие: %s\n' "$(printf '%s' "$cond_line" | sed 's/^ *//')"
    if printf '%s' "$cond_line" | grep -q 'WAKE_GATE_SKIPPED_STEPS'; then
        ok "пропуск гейта завязан на счётчик реально пропущенных шагов"
    else
        bad "пропуск GATE-1 завязан НЕ на число пропущенных шагов: ADR-0022 гейт отключается на каждом scenario-прогоне, где гейт просто не прогрелся"
    fi
    if grep -qE '^WAKE_GATE_SKIPPED_STEPS=0' "$HARNESS"; then
        ok "счётчик WAKE_GATE_SKIPPED_STEPS инициализирован"
    else
        bad "нет инициализации WAKE_GATE_SKIPPED_STEPS=0 (под set -u упадёт)"
    fi
    if grep -q 'WAKE_GATE_SKIPPED_STEPS + 1' "$HARNESS"; then
        ok "счётчик инкрементируется на фактическом SKIP шага"
    else
        bad "счётчик нигде не увеличивается — он всегда 0, и гейт никогда не пропустится (обратная крайность)"
    fi
fi

printf '\n'
if [ "$fails" -ne 0 ]; then
    printf 'wake-gate no mass skip: FAIL (%d)\n' "$fails"
    exit 1
fi
printf 'wake-gate no mass skip: PASS\n'
