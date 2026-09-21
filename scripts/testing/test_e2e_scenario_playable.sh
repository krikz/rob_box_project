#!/usr/bin/env bash
# Гард: каждая реплика e2e-сценария обязана влезать в окно VAD робота.
#
# ЗАЧЕМ (живой прогон 35658231116, 22.09.2026).
# audio_node отбрасывает фразу по длине ДО STT:
#     ❌ Речь отклонена: 17.37с (min=0.3, max=15.0)
# Шаги night_marathon_act3 n303_bg_boris (136 симв) и n304_bg_grisha_unknown
# (129 симв) попали ровно в это и отчитались `FAIL backlog_miss` — то есть
# обвинили бэклог в dialogue_node, хотя dialogue_node этих фраз не видел.
# Соседний n305 (121 симв) в первой попытке отвалился, во второй прошёл на
# 14.81с — зазор 0.19с. Такой шаг не проверяет фичу, он бросает монетку.
#
# Хуже другое. Акт 2 «знакомство» — тот, где дикторы РЕГИСТРИРУЮТСЯ, — содержит
# реплики по 201-251 символу, это 26-32с синтеза. Они не проигрывались НИКОГДА.
# Поэтому speakers.db на роботе держит профили от 12.08.2026, identify-скоры
# всех дикторов лежат в 0.640-0.691 при identify_threshold=0.72, и весь акт 3
# видит `speaker='незнакомец'` вместо Саши/Бориса. Проверки act3 на известного
# диктора не могли пройти ни разу, а единственная проходившая проверка
# (`speaker='незнакомец'`) проходила ВАКУУМНО — она верна, когда распознавание
# мертво целиком. Красивый PASS поверх неработающей регистрации.
#
# ПОЧЕМУ ОФЛАЙНОВЫЙ ГАРД, если харнесс теперь мерит wav точно.
# Харнесс мерит факт (ffprobe на _eq.wav) и честно валит шаг — но уже на
# стенде, заняв единственный e2e-раннер на десятки минут. Этот тест ловит то
# же самое на PR, без робота и без синтеза.
#
# МОДЕЛЬ ОЦЕНКИ (калибровка на замерах того же прогона).
# Харнесс играет _eq.wav = adelay 1.5s + синтез. audio_node мерит не длину
# файла, а окно от первой речи до последней, с VAD-hangover'ом. Замеры
# (eq-длительность → что насчитал VAD):
#     n301  6.23 →  9.95     n302  9.81 → 12.51     n303 14.84 → 17.37 ❌
#     n304 13.39 → 16.09 ❌  n305 12.41 → 15.07 ❌  n306  9.13 → 13.28
#     n307  6.27 →  8.67     n308  7.42 →  9.95
# Накладка VAD лежит в 2.4-4.2с; берём минимум (2.5) — самый мягкий порог,
# который всё ещё правильно классифицирует все восемь шагов.
# Скорость синтеза: худшая наблюдённая 0.0981 с/символ (n303: 13.34с / 136).
# Итого бюджет на текст: speech_max_duration - VAD_OVERHEAD - ADELAY.
#
# Оценка сознательно пессимистична: она отвергнет пограничную реплику, которая
# «иногда проходит». Это и есть цель — шаг-лотерея не является проверкой.
#
# Запуск: bash scripts/testing/test_e2e_scenario_playable.sh
# Exit:   0 = PASS, 1 = есть непроигрываемые реплики.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HARNESS="$REPO_ROOT/.github/workflows/scripts/e2e_voice_test.sh"
CONFIG="$REPO_ROOT/docker/vision/config/voice_assistant/audio_node.yaml"

for f in "$HARNESS" "$CONFIG"; do
    [ -f "$f" ] || { printf '❌ отсутствует: %s\n' "$f"; exit 1; }
done

python3 - "$REPO_ROOT" "$HARNESS" "$CONFIG" <<'PY'
import json, pathlib, re, sys

# Вывод содержит кириллицу и эмодзи. На CI (Linux, UTF-8) это не проблема, но
# на dev-машине под Windows stdout — cp1252, и печать падала UnicodeEncodeError
# ещё до первой проверки. Тест обязан работать там, где его будут запускать.
for stream in (sys.stdout, sys.stderr):
    try:
        stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

repo = pathlib.Path(sys.argv[1])
harness = pathlib.Path(sys.argv[2]).read_text(encoding="utf-8")
config_text = pathlib.Path(sys.argv[3]).read_text(encoding="utf-8")

fails = []
notes = []

# --- параметры модели ------------------------------------------------------
ADELAY = 1.5          # харнесс добавляет adelay=1500 в EQ-фильтре
SEC_PER_CHAR = 0.0981 # худшая наблюдённая скорость синтеза (n303, minimax)
WARN_MARGIN = 1.5     # меньше этого запаса — шаг на грани

def yaml_float(text, key):
    m = re.search(rf"^\s*{re.escape(key)}\s*:\s*([0-9]+\.?[0-9]*)\s*$", text, re.M)
    return float(m.group(1)) if m else None

def harness_default(text, var):
    # Присваивание живёт внутри run_step, т.е. с отступом — якорь ^ без \s*
    # не находил его и тест ругался «фикс откатили?» на рабочем харнессе.
    m = re.search(rf'^\s*{re.escape(var)}="\$\{{{re.escape(var)}:-([0-9.]+)\}}"', text, re.M)
    return float(m.group(1)) if m else None

cfg_max = yaml_float(config_text, "speech_max_duration")
if cfg_max is None:
    fails.append("audio_node.yaml: не нашёл speech_max_duration — гард не может знать лимит")
    cfg_max = 15.0

# --- CHECK 1: дефолт в харнессе не разъехался с конфигом робота ------------
print("CHECK 1: дефолты харнесса синхронны с audio_node.yaml")
h_max = harness_default(harness, "E2E_VAD_MAX_DURATION")
h_ovh = harness_default(harness, "E2E_VAD_OVERHEAD")
if h_max is None:
    fails.append("харнесс: не нашёл дефолт E2E_VAD_MAX_DURATION — фикс откатили?")
elif abs(h_max - cfg_max) > 1e-9:
    fails.append(
        f"рассинхрон лимита: харнесс держит {h_max}, а робот грузит "
        f"speech_max_duration={cfg_max} (docker/vision/config/voice_assistant/"
        f"audio_node.yaml). Тест окна VAD проверял бы не то число."
    )
else:
    print(f"  ✅ speech_max_duration={cfg_max}s совпадает с дефолтом харнесса")
if h_ovh is None:
    fails.append("харнесс: не нашёл дефолт E2E_VAD_OVERHEAD")
else:
    print(f"  ✅ VAD-накладка {h_ovh}s задана явно")
overhead = h_ovh if h_ovh is not None else 2.5

# --- CHECK 2: adelay в EQ совпадает с моделью ------------------------------
print("CHECK 2: adelay в EQ-фильтре совпадает с моделью оценки")
adelays = set(re.findall(r"adelay=(\d+)", harness))
if adelays == {"1500"}:
    print(f"  ✅ adelay=1500ms ({ADELAY}s), как в модели")
else:
    fails.append(
        f"adelay в харнессе = {sorted(adelays)}, модель оценки считает {int(ADELAY*1000)}ms. "
        "Поправь ADELAY в этом тесте вместе с фильтром, иначе бюджет поедет."
    )

# --- CHECK 3: реплики сценариев влезают в бюджет ---------------------------
budget = cfg_max - overhead - ADELAY
max_chars = int(budget / SEC_PER_CHAR)
print(f"CHECK 3: реплики влезают в бюджет {budget:.2f}s текста (~{max_chars} символов)")
print(f"  (= speech_max_duration {cfg_max}s - VAD-накладка {overhead}s - adelay {ADELAY}s,"
      f" при {SEC_PER_CHAR} с/символ)")

scenarios = sorted(
    p for p in (repo / ".github" / "e2e" / "scenarios").rglob("*.json")
    if "acceptance" not in p.name and "manifest" not in p.name
)
if not scenarios:
    fails.append("не нашёл ни одного сценария — тест сломан?")

checked = over = warned = 0
for path in scenarios:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        fails.append(f"{path.relative_to(repo)}: битый JSON ({exc})")
        continue
    steps = data.get("steps")
    if not isinstance(steps, list):
        continue
    for idx, step in enumerate(steps):
        if not isinstance(step, dict):
            continue
        text = step.get("text") or ""
        if not text:
            continue
        checked += 1
        # Явный отказ от гарда — только осознанно, с причиной в самом сценарии.
        if step.get("vad_budget_waiver"):
            notes.append(
                f"{path.relative_to(repo)} шаг {step.get('label', idx)}: гард отключён "
                f"вручную (vad_budget_waiver: {step['vad_budget_waiver']})"
            )
            continue
        est = len(text) * SEC_PER_CHAR
        sid = step.get("label") or step.get("id") or step.get("name") or f"#{idx}"
        if est > budget:
            over += 1
            fails.append(
                f"{path.relative_to(repo)} шаг {sid}: {len(text)} символов "
                f"(~{est:.1f}s синтеза при бюджете {budget:.2f}s) — audio_node "
                f"отбросит фразу до STT. Разбей реплику на два шага или сократи "
                f"до ~{max_chars} символов."
            )
        elif budget - est < WARN_MARGIN:
            warned += 1
            notes.append(
                f"{path.relative_to(repo)} шаг {sid}: {len(text)} символов "
                f"(~{est:.1f}s, запас {budget - est:.1f}s) — на грани окна VAD"
            )

print(f"  проверено реплик: {checked}, за бюджетом: {over}, на грани: {warned}")

if notes:
    print("\nПредупреждения (не валят тест):")
    for n in notes:
        print(f"  ⚠️  {n}")

if fails:
    print("\nПроблемы:")
    for f in fails:
        print(f"  ❌ {f}")
    print(f"\ne2e scenario playable: FAIL ({len(fails)})")
    sys.exit(1)

print("\ne2e scenario playable: PASS")
PY
