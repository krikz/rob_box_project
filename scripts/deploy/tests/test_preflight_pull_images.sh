#!/usr/bin/env bash
# Регресс-тест issue #2930: деплой не должен останавливать/удалять работающие
# контейнеры, если нужных образов нет.
#
# 1. scripts/deploy/preflight_pull_images.sh с фейковым `docker` в PATH:
#    - pull упал / образа нет локально ⇒ exit 1 и НИ ОДНОГО down/rm/up;
#    - все образы на месте ⇒ exit 0 и дальше идут down/rm/up.
#    «Дальше» моделируется так же, как в workflow: preflight — отдельный шаг,
#    и следующий шаг (Stop/Start) выполняется только при его успехе.
# 2. Порядок шагов в ".github/workflows/L-Deploy and Verify.yml": для каждого
#    Pi шаг Pull (с preflight) стоит ДО Stop Containers, не маскируется
#    `|| echo`, не имеет continue-on-error, и нигде нет --ignore-pull-failures.
#
# Запуск: bash scripts/deploy/tests/test_preflight_pull_images.sh

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SCRIPT="$REPO_ROOT/scripts/deploy/preflight_pull_images.sh"
WORKFLOW="$REPO_ROOT/.github/workflows/L-Deploy and Verify.yml"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

pass=0
failures=0
ok() { echo "PASS: $1"; pass=$((pass + 1)); }
ko() { echo "FAIL: $1"; failures=$((failures + 1)); }

# --- фейковый docker -------------------------------------------------------
# Пишет каждый вызов в $FAKE_DOCKER_LOG. Поведение:
#   FAKE_PULL_RC        — код возврата `compose pull` (default 0)
#   FAKE_IMAGES         — вывод `compose config --images` (по строке)
#   FAKE_PRESENT        — образы, которые `image inspect` считает локальными
mkdir -p "$WORK/bin"
cat > "$WORK/bin/docker" <<'EOF'
#!/usr/bin/env bash
echo "docker $*" >> "$FAKE_DOCKER_LOG"
case "$1 $2" in
  "compose config")
    if [ "${3:-}" = "--images" ]; then printf '%s\n' "$FAKE_IMAGES"; fi
    exit 0 ;;
  "compose pull")
    exit "${FAKE_PULL_RC:-0}" ;;
  "image inspect")
    printf '%s\n' "$FAKE_PRESENT" | grep -qxF -- "$3" && exit 0
    exit 1 ;;
esac
exit 0
EOF
chmod +x "$WORK/bin/docker"

ALL_IMAGES=$'reg/rob_box:rtabmap-humble-dev\nreg/rob_box:nav2-humble-dev\neclipse/zenoh:1.6.2\nreg/rob_box:nav2-humble-dev'

# Моделирует шаги workflow: Pull(preflight) → [только при успехе] Stop → Start.
run_deploy() {
    export FAKE_DOCKER_LOG="$WORK/docker.log"
    : > "$FAKE_DOCKER_LOG"
    PATH="$WORK/bin:$PATH" bash "$SCRIPT" > "$WORK/out.txt" 2>&1
    local rc=$?
    if [ "$rc" -eq 0 ]; then
        PATH="$WORK/bin:$PATH" docker compose down --remove-orphans --timeout 20
        PATH="$WORK/bin:$PATH" docker rm -f abc
        PATH="$WORK/bin:$PATH" docker compose up -d --force-recreate --pull never
    fi
    return "$rc"
}

# check <msg> <cmd...> — PASS, если команда успешна; refute — наоборот.
check()  { local msg=$1; shift; if "$@"; then ok "$msg"; else ko "$msg"; fi; }
refute() { local msg=$1; shift; if "$@"; then ko "$msg"; else ok "$msg"; fi; }
logged() { grep -qE -- "$1" "$FAKE_DOCKER_LOG"; }
DESTRUCTIVE='^docker (compose (down|up|rm|stop)|rm|stop|container prune)'

# --- case 1: образа нет в реестре (pull падает), локально тоже нет ---------
export REGISTRY_SOURCE=local FAKE_IMAGES="$ALL_IMAGES" FAKE_PULL_RC=1
export FAKE_PRESENT=$'reg/rob_box:nav2-humble-dev\neclipse/zenoh:1.6.2'
refute "case1: pull failure ⇒ exit 1" run_deploy
refute "case1: no down/rm/up called" logged "$DESTRUCTIVE"
refute "case1: pull without --ignore-pull-failures" logged '--ignore-pull-failures'

# --- case 2: pull «успешен», но образа нет локально (напр. registry_source=skip)
export REGISTRY_SOURCE=skip FAKE_PULL_RC=0
refute "case2: missing image ⇒ exit 1" run_deploy
refute "case2: no down/rm/up called" logged "$DESTRUCTIVE"
refute "case2: skip ⇒ no pull" logged '^docker compose pull'
check "case2: missing image named in output" grep -q 'rtabmap-humble-dev' "$WORK/out.txt"

# --- case 3: всё на месте ⇒ идём дальше -----------------------------------
export REGISTRY_SOURCE=local FAKE_PULL_RC=0
export FAKE_PRESENT=$'reg/rob_box:rtabmap-humble-dev\nreg/rob_box:nav2-humble-dev\neclipse/zenoh:1.6.2'
check "case3: all images present ⇒ exit 0" run_deploy
check "case3: pulled with --policy always" logged '^docker compose pull --policy always$'
check "case3: stop proceeds (down)" logged '^docker compose down'
check "case3: start proceeds (up -d)" logged '^docker compose up -d'

# --- workflow order ---------------------------------------------------------
if python3 - "$WORKFLOW" <<'PY'
# Без PyYAML (на ubuntu-latest его может не быть): режем файл на шаги по
# строкам `      - name:` — у deploy-and-verify это единственный формат шага.
import re, sys
text = open(sys.argv[1], encoding="utf-8").read()
steps = []
for part in re.split(r"(?m)^      - name: ", text)[1:]:
    if_m = re.search(r"(?m)^        if: (.*)$", part)
    steps.append({
        "name": part.splitlines()[0].strip().strip('"'),
        "run": part,
        "if": if_m.group(1) if if_m else "",
        "continue-on-error": re.search(r"(?m)^        continue-on-error:", part) is not None,
    })
names = [s["name"] for s in steps]
errors = []
for pi in ("Vision Pi", "Main Pi"):
    idx = {k: names.index(f"[{pi}] {k}") for k in ("Pull Docker Images", "Stop Containers", "Start Containers")}
    pull = steps[idx["Pull Docker Images"]]
    if not idx["Pull Docker Images"] < idx["Stop Containers"] < idx["Start Containers"]:
        errors.append(f"{pi}: order must be Pull < Stop < Start, got {idx}")
    run = pull.get("run", "")
    if "preflight_pull_images.sh" not in run:
        errors.append(f"{pi}: Pull step does not call preflight_pull_images.sh")
    if "|| echo" in run or pull.get("continue-on-error"):
        errors.append(f"{pi}: Pull step failure is masked")
    if "skip" in str(pull.get("if", "")):
        errors.append(f"{pi}: Pull step must run for registry_source=skip too (local image check)")
for s in steps:
    code = [l for l in s["run"].splitlines() if not l.strip().startswith("#")]
    if any("--ignore-pull-failures" in l for l in code):
        errors.append(f"{s.get('name')}: uses --ignore-pull-failures")
if errors:
    print("\n".join(errors))
sys.exit(1 if errors else 0)
PY
then ok "workflow: Pull(preflight) before Stop for both Pi, not masked"; else ko "workflow order/masking"; fi

echo "----"
echo "passed=$pass failed=$failures"
[ "$failures" -eq 0 ]
