#!/bin/bash
# install.sh — раскладка процессных скриптов agent-flow по нужным директориям.
#
# Source of truth: <repo>/scripts/agent_flow/*.sh (эта папка).
# Копии (которые ищет cron при старте, при запуске воркера профиля, и т.п.):
#   1. /home/builder/.hermes/profiles/agent-flow/scripts/  — каноническое место
#   2. /home/builder/.hermes/profiles/architect/scripts/   — где cron сейчас ищет
#   3. /home/builder/.hermes/profiles/devops/scripts/      — devops-профиль
#   4. /home/builder/.hermes/profiles/backend/scripts/     — backend-профиль (с 01.09)
#   5. /home/builder/.hermes/profiles/analyst/scripts/     — analyst-профиль (с 01.09)
#   6. /home/builder/.hermes/scripts/                      — legacy (cron тоже стартует)
#
# Этот скрипт раскладывает **hardlink (cp -al) на канонические файлы из репо**.
# Hardlink — обычный файл с тем же inode, поэтому он гарантированно остаётся
# ВНУТРИ целевой директории (а не symlink наружу) и проходит guard
# `hermes-agent/cron/scheduler.py::_validate_script_path`, который требует
# `path.resolve()` находиться внутри `HERMES_HOME/scripts/`.
#
# Ретро 11.08 18:49 (kanban t_a6a236e0d9f0470e): cron 'Agent Cockpeat Watch Tock'
# (id 2fcf2ad1bd0b) упал на 50 тиков подряд (1ч42м даунтайма), потому что
# install.sh раскладывал симлинки `watchdog.sh -> .../repo/scripts/...`,
# а scheduler.py guard ресолвил их наружу `scripts_dir` и отклонял с
# "Blocked: script path resolves outside the scripts directory".
#
# Ретро 01.09 t_a3ba921e (DRIFT: 24KB merge-gate.sh отстал на backend/analyst
# после PR #1849 ADR-0035): TARGET_DIRS обновлён с 4 до 6 путей.
# Backend и analyst получили скрипты через profile-create.sh (snapshot с
# 31.08 — до MERGE PR #1849), а install.sh их не покрывал. Результат: 480
# строк (§4 stale-after-upstream-fix detector) у этих профилей — старая
# версия, ADR-0035 для половины профилей не работает. Чтобы такое не
# повторялось при следующем крупном merge в merge-gate (например, ADR-0044
# contract-drift bot сейчас в PR #1860), install.sh после раскладки делает
# ЖЁСТКУЮ md5-проверку (post_install_verify) — если хоть один TARGET_DIR
# отличается от SOT, скрипт валится с exit !=0 и пишет alert в alert.log.
#
# Резервные пути раскладки (если hardlink невозможен — cross-device):
#   1) cp -aL (копия содержимого) — fallback по умолчанию для symlink;
#   2) symbolic link — крайний случай, только для директорий вне `scripts_dir`;
#      для самой `~/.hermes/scripts/` symlink ЗАПРЕЩЁН (сломает guard).
#
# Гарантии:
#   - Все 6 путей ссылаются на одну и ту же inode-копию (hardlink) либо
#     на одинаковое содержимое (cp);
#   - Правка в репо (через PR/merge) автоматически расходится по всем путям
#     сразу при следующем запуске этого скрипта;
#   - Ни один файл в TARGET_DIRS не указывает за пределы своей директории
#     (anti-escape guard в конце);
#   - Post-install md5-сверка всех TARGET_DIRS (ретро 01.09 t_a3ba921e):
#     если хоть один файл отличается от SOT — exit 3 + alert в drift.alert.log;
#   - Нет дубликатов, нет drift (см. scripts/agent_flow/README.md).
#
# Запуск:
#   ./scripts/agent_flow/install.sh            # раскладка на хост
#   ./scripts/agent_flow/install.sh --dry-run  # только показать что сделает
#
# Идемпотентен — повторный запуск обновляет ссылки, ничего не ломает.
#
# Расширяемость: чтобы добавить новый целевой профиль, добавьте его путь в
# массив TARGET_DIRS ниже + пропишите комментарий в шапке. Все EXPECTED-файлы
# из этого скрипта будут разложены в новый профиль без дополнительных правок.

set -e

# Единый список процессных скриптов. Владелец списка — ЭТОТ файл:
#   - install.sh раскладывает EXPECTED по хостам;
#   - agent-flow-drift-detect.sh читает список через `install.sh --list-files`
#     (ретро 13.08 t_2cae75c0: раньше список дублировался в drift-detect.sh,
#     из-за чего kanban-retro-create.sh и ещё 2 скрипта не контролировались
#     drift-детектором; теперь источник один).
EXPECTED=(
    agent-flow-triage.sh
    agent-flow-merge-gate.sh
    agent-flow-completion-check.sh
    agent-flow-e2e-process.sh
    # Cron launcher для e2e-process (ретро 23.08 t_98bb3a1d): no-agent job
    # каждые 20 мин, подгружает env из .env и запускает e2e-process.sh.
    # SOT живёт в <repo>/scripts/agent_flow/, раскладывается install.sh.
    agent-flow-e2e-process-launcher.sh
    # Daily install.sh tick wrapper (ретро 28.08 t_7ebdfce0, kanban
    # t_36006bee): no-agent cron-job запускает install.sh каждый день в 03:00
    # чтобы гарантировать раскладку процессных скриптов по всем 4 target-папкам
    # без ручного запуска оператора. Wrapper-тонкий (10 строк), exec'ит
    # install.sh из репо (SOT) с REPO_DIR=$HERMES_SHARE/rob_box_project.
    # Раньше жил ТОЛЬКО в legacy ~/.hermes/scripts/ и не попадал в EXPECTED[],
    # из-за чего drift-detect его не контролировал и на profile-уровне
    # (devops/scripts/) его не было → cron-тик падал с «Script not found».
    agent-flow-install-daily.sh
    agent-flow-handoff.sh
    round_ensure.sh
    agent-flow-cleanup-249.sh
    agent-flow-deploy-sweep.sh
    agent-flow-unlabeled-sweep.sh
    agents_sleep.sh
    agents_sleep_schedule.conf
    cron-loop.sh
    watchdog.sh
    # Provider-exhaustion fast-tick guard (ретро 24.08 t_4c73490f):
    # 1-мин hot-path НЕОБХОДИМ, потому что watchdog.sh (2-мин scan) не
    # успевает среагировать до consecutive_failures=2 → gave_up.
    # Делит PROVIDER_MARKERS + ту же логику recovery-волны, плюс расширен
    # маркерами HTTP 401 / Authentication Fails (DeepSeek invalid api key).
    watchdog-provider-quick.sh
    agent-flow-drift-detect.sh
    kanban-retro-create.sh
    validate_honesty.sh
    # Pre-PR check на ADR namespace collision (ретро 01.09 t_debcb647):
    # дополняет validate_honesty.sh (claim-evidence) функцией проверки
    # ADR-номеров. Локальный запуск воркером ДО `gh pr create` ловит
    # collision раньше, чем merge-gate отвергнет PR. Запускается
    # руками (`bash scripts/agent_flow/validate_adr_namespace.sh`),
    # не блокер CI — воркер видит actionable ошибку и сам переименовывает
    # в next-free slot (вычисляется из max(origin/develop ADR number) + 1).
    validate_adr_namespace.sh
    # Post-merge build trigger (issue #1475, ADR-0022 extension): после
    # MERGED PR в develop/main — запускает L-Build-All-Services чтобы
    # .image-versions.dev получил свежие dev-<sha> теги.
    agent-flow-post-merge-build.sh
    # Shared library (дедуп 30.08): af_load_profile_env,
    # af_flock_guard_or_exit, af_maintenance_gate_or_exit,
    # gh_list_issues_by_label, has_label / has_label_json, slugify,
    # detect_pr_kind, free_stale_worktrees_for. Source'ится из merge-gate /
    # e2e-process / triage / deploy-sweep / unlabeled-sweep / handoff.
    # До этого то же самое жило копипастой по шести скриптам и успело
    # разъехаться дефолтами и текстами логов.
    lib_agent_flow_common.sh
    # Shared library (ретро 18.08 t_de6bea69): source'ится из e2e-process и
    # merge-gate для «user-unlabel respect» guard'а. Должен лежать рядом со
    # скриптами во всех профилях.
    lib_user_unlabel_check.sh
    # Shared library (issue #1540): source'ится из e2e-process и
    # post-merge-build для verify_recent_run() — общий контракт dedup'а
    # вместо копи-пасты. Должен лежать рядом со скриптами во всех профилях.
    lib_workflow_dedup.sh
    # Self-id / whoami helper (issue #1534): source'ится из 4 процессных
    # скриптов (merge-gate / triage / e2e-process / completion-check) чтобы
    # перед каждым side-effect на PR/issue писать «🤖 [agent:<role>]
    # script=… action=… reason=…» — чтобы в истории GitHub было видно КТО
    # это сделал (actor = krikz по GH-токену, иначе неразличимо).
    hermes_github.sh
    # Cross-task archive sweeper (ADR-0024 / ретро 22.08 t_d9b4c600): watchdog,
    # архивирующий blocked-карточки devops после успешного PR/issue.
    # Зависит от python3 helper'ов _cross_task_archive_sweeper_{scan,archive}.py
    # (должны лежать рядом — install.sh раскладывает только .sh, поэтому
    # python-файлы мы кладём соседним шагом `.bak` или тестовый
    # `cross-task-archive-sweeper.sh` запускается с явным `_LIB_DIR_HERE`
    # указывающим на repo). В dry-run install.sh их не трогает.
    cross-task-archive-sweeper.sh
    _cross_task_archive_sweeper_scan.py
    _cross_task_archive_sweeper_archive.py
    # Orphan blocked-card watchdog (ретро t_1d0426e3): no-agent job,
    # каждые 4h сканирует open issues с меткой needs-e2e в GH и закрывает
    # те, для которых найден MERGED PR (PATTERN «карточки-призраки»).
    # Регистрация cron-job делается в ensure_blocked_watchdog_cron ниже.
    agent-flow-blocked-watchdog.sh
# Fail-streak escalation watchdog (ретро 28.08 t_faac94b0): no-agent,
    # вызывается ИЗ launcher'а (после e2e-process.sh tick), не отдельным
    # cron-job. При streak ≥ WARN → issue-comment, при streak ≥ PAUSE →
    # sentinel-файл → e2e-process замораживает ротацию.
    agent-flow-e2e-fail-streak-watchdog.sh
    # Observability-вотчдоги (ретро 19.08 t_5cde0bc1 и 25.08 t_2d8cc9c4).
    # Метки НЕ меняют — только читают и возвращают exit-код для cron-алерта,
    # поэтому раскладка безопасна и без регистрации cron-job'а: файл на хосте
    # + drift-detect его контролирует, запуск — руками или через cron, когда
    # Шифу решит. До 30.08 оба лежали в репо ВНЕ этого списка, то есть на хост
    # не попадали вообще и запускаться физически не могли.
    #   drift    — сколько PR висят с e2e-done, пока их issue вернулась
    #              в ротацию (merge-gate reconcile сделал, но если не сработал —
    #              это единственный способ увидеть, что он не сработал);
    #   rotation — жива ли e2e-ротация: нет тиков и нет новых
    #              z-{e2e}/test-round-* за окно → ALERT.
    agent-flow-e2e-drift-watchdog.sh
    # One-shot cleanup для /tmp/agent-flow-e2e-* orphan mess (issue #1707,
    # ретро t_0ff29dcd): раскладывается install.sh, чтобы оператор мог
    # `bash scripts/agent_flow/agent-flow-e2e-wt-sweep.sh` с любой 3-target
    # директории (~/.hermes/scripts/ / ~/.hermes/profiles/<agent>/scripts/ /
    # hermes-share/...) без поиска SOT-пути в репо. Cron-job'ом НЕ
    # регистрируется — per-tick sweep в e2e-process.sh делает то же самое
    # при каждом запуске (issue #1707).
    agent-flow-e2e-wt-sweep.sh
    agent-flow-rotation-watchdog.sh
    # Padavan-vakhta STEP 4 voice-smoke (issue #1772): "живая" проверка
    # робота — проигрывает 2 .wav через динамик 10.1.1.249, читает логи
    # voice-assistant на 10.1.1.21. Устойчив к отсутствию .wav (NO-OP +
    # WARN, не валит cron tick). Вызывается LLM'ом в ШАГ 4 промпта
    # падаван-вахты (5a070bf3ed3e).
    padavan-step4-voice-smoke.sh
    # Cron-надзор mis-scope архитектурных карточек (ADR-0036 §4.3,
    # ретро t_aa585aa7): no-agent job, каждый час сканирует running-карточки
    # в kanban.db, для которых age > 4ч И assignee ≠ architect/devops И
    # body LIKE '%ADR-%', и пишет ОДИН auto-comment (idempotent через
    # task_comments). НЕ kill, НЕ reassign — Шифу принимает решение.
    # Регистрация cron-job делается в ensure_blocked_watchdog_scope_cron.
    agent-flow-blocked-watchdog-scope.sh
    # Ночной ревью-цикл (ADR-0049): no-agent job, раз в ночь собирает
    # дайджест за прошедшие сутки (merged PR / коммиты / issues /
    # красный CI / kanban) и заводит ОДНУ карточку «ночной ревью <дата>»
    # на architect + до COMPONENT_REVIEW_MAX карточек «ревью компонента:
    # <comp>» на analyst для компонентов, которые за сутки меняли (дубли /
    # глюки LLM / недоделки). Карточки создаются через
    # kanban-retro-create.sh (дедуп по key). Регистрация cron-job —
    # в ensure_nightly_review_cron ниже.
    agent-flow-nightly-review.sh
)

# Режим --list-files: печатает EXPECTED по одному имени на строку и выходит.
# Используется agent-flow-drift-detect.sh как единый источник списка.
if [ "${1:-}" = "--list-files" ]; then
    printf '%s\n' "${EXPECTED[@]}"
    exit 0
fi

DRY_RUN=false
[ "${1:-}" = "--dry-run" ] && DRY_RUN=true

# Если передан явный REPO_DIR (например на build host, где нет
# /home/builder/hermes-share), используем его:
#   bash install.sh /tmp/install_af_1107
#   bash install.sh --dry-run /tmp/install_af_1107
#   REPO_DIR=/tmp/install_af_1107 bash install.sh
# По умолчанию — путь dev-машины.
if [ "${1:-}" = "--dry-run" ]; then
    REPO_DIR="${REPO_DIR:-${2:-/home/builder/hermes-share/rob_box_project}}"
else
    REPO_DIR="${REPO_DIR:-${1:-/home/builder/hermes-share/rob_box_project}}"
fi
SCRIPT_DIR="$REPO_DIR/scripts/agent_flow"

# Канонические пути (все должны стать hardlink-ами на одну и ту же inode).
# Переопределяются INSTALL_TARGET_DIRS (colon-separated) для тестов и
# нестандартных хостов (см. tests/test_drift_detect_branch_active.sh).
#
# Ретро 01.09 t_a3ba921e: 01.09 в TARGET_DIRS добавлены backend/scripts и
# analyst/scripts (были пропущены с момента создания профилей 31.08, из-за
# чего merge-gate.sh отстал на 24KB от develop после PR #1849 ADR-0035).
# Полный список на 01.09 — 6 путей: основной cron (architect/agent-flow) +
# devops + backend + analyst + legacy ~/.hermes/scripts. При добавлении
# нового профиля: 1) допишите путь сюда, 2) обновите комментарий в шапке
# скрипта, 3) перепроверьте post_install_verify (md5 сверит все 6 путей).
if [ -n "${INSTALL_TARGET_DIRS:-}" ]; then
    IFS=':' read -r -a TARGET_DIRS <<< "$INSTALL_TARGET_DIRS"
else
    TARGET_DIRS=(
        "/home/builder/.hermes/profiles/agent-flow/scripts"
        "/home/builder/.hermes/profiles/architect/scripts"
        "/home/builder/.hermes/profiles/devops/scripts"
        "/home/builder/.hermes/profiles/backend/scripts"
        "/home/builder/.hermes/profiles/analyst/scripts"
        "/home/builder/.hermes/scripts"
    )
fi

# ~/.hermes/scripts/ проходит через guard в
# hermes-agent/cron/scheduler.py::_validate_script_path.
# Любой symlink наружу этой директории будет отклонён, поэтому
# для HERMES_SCRIPTS_DIR симлинки ЗАПРЕЩЕНЫ (см. ретро 11.08 t_a6a236e0d9f0470e).
# Переопределяется для тестов (fixture-директория вместо реального ~/.hermes).
HERMES_SCRIPTS_DIR="${HERMES_SCRIPTS_DIR:-/home/builder/.hermes/scripts}"

run() {
    if $DRY_RUN; then
        echo "  [DRY] $*"
    else
        "$@"
    fi
}

# _remove_existing dst — убирает существующий dst (regular file или symlink),
# оставляя .bak-версию, если dst был обычным файлом.
_remove_existing() {
    local dst="$1"
    local bak
    if [ -L "$dst" ]; then
        run rm -f "$dst"
    elif [ -e "$dst" ]; then
        bak="${dst}.bak.$(date -u +%Y%m%dT%H%M%SZ)"
        run mv "$dst" "$bak"
        echo "  BAK  $(basename "$dst") (was real file — saved as $bak)"
    fi
}

# _install_one src dst — раскладывает src в dst.
#   - сначала пробуем hardlink (cp -al) — это и есть основной режим;
#   - если cross-device (cp -al падает), делаем обычную копию (cp);
#   - симлинк — крайний случай, и ТОЛЬКО для директорий вне HERMES_SCRIPTS_DIR
#     (для HERMES_SCRIPTS_DIR симлинк ЗАПРЕЩЁН, см. anti-escape guard).
_install_one() {
    local src="$1"
    local dst="$2"
    local name
    name="$(basename "$dst")"

    # Уже hardlink на нужный src — ничего не делаем.
    if [ -f "$dst" ] && [ ! -L "$dst" ] \
        && [ "$(stat -c '%i' "$dst" 2>/dev/null)" = "$(stat -c '%i' "$src" 2>/dev/null)" ]; then
        echo "  OK   $name (already hardlink to src)"
        return 0
    fi

    # Уже regular file с тем же содержимым (cp-fallback от прошлого запуска) — оставляем.
    if [ -f "$dst" ] && [ ! -L "$dst" ] \
        && cmp -s "$dst" "$src" 2>/dev/null; then
        echo "  OK   $name (regular file copy, content matches)"
        return 0
    fi

    _remove_existing "$dst"

    # 1) hardlink (preferred)
    if $DRY_RUN || cp -al "$src" "$dst" 2>/dev/null; then
        [ -L "$dst" ] || [ -f "$dst" ] && echo "  HLINK $name -> $(stat -c '%i' "$dst" 2>/dev/null) inode of $src"
        return 0
    fi

    # 2) regular copy fallback (если hardlink упал — обычно cross-device)
    if cp -a "$src" "$dst" 2>/dev/null; then
        echo "  COPY $name (hardlink unavailable, used copy)"
        return 0
    fi

    # 3) symbolic link — крайний fallback, но ТОЛЬКО если dst не в HERMES_SCRIPTS_DIR.
    local dst_dir
    dst_dir="$(dirname "$dst")"
    if [ "$dst_dir" = "$HERMES_SCRIPTS_DIR" ]; then
        echo "  ERROR $name: cannot install (hardlink/copy failed AND symlink forbidden in $HERMES_SCRIPTS_DIR)" >&2
        return 1
    fi
    ln -sf "$src" "$dst"
    echo "  SLINK $name -> $src (last-resort fallback; NOT $HERMES_SCRIPTS_DIR, so guard not affected)"
}

echo "==> Source of truth: $SCRIPT_DIR"
if [ ! -d "$SCRIPT_DIR" ]; then
    echo "ERROR: source dir not found: $SCRIPT_DIR (clone the repo there?)"
    exit 1
fi

# sanity check — файлы на месте (EXPECTED объявлен в начале файла — единый
# список для раскладки и для agent-flow-drift-detect.sh --list-files)
for f in "${EXPECTED[@]}"; do
    if [ ! -f "$SCRIPT_DIR/$f" ]; then
        echo "ERROR: missing canonical file $SCRIPT_DIR/$f"
        exit 2
    fi
done

for target_dir in "${TARGET_DIRS[@]}"; do
    if [ ! -d "$target_dir" ]; then
        echo "  SKIP $target_dir (dir not present)"
        continue
    fi

    echo "==> $target_dir"
    run mkdir -p "$target_dir"
    for f in "${EXPECTED[@]}"; do
        src="$SCRIPT_DIR/$f"
        dst="$target_dir/$f"

        _install_one "$src" "$dst"
    done
done

# ---------------------------------------------------------------------------
# Применение vendor-патчей к hermes-agent (ретро t_f00676f8).
#
# Проблема: локальные фиксы hermes-agent (валидация скиллов по профилю
# t_1ab37fa8: _profile_skill_names/_validate_skills_for_assignee в
# hermes_cli/kanban_db.py, symlink-following подсчёт скиллов в
# hermes_cli/profiles.py) накладывались на хост руками БЕЗ сохранения в репо.
# При `git pull`/`pip install -U hermes-agent` патчи теряются, и регресс
# t_1ab37fa8 возвращается (карточки со скилами не из профиля падают).
#
# Решение: диффы хранятся в репо как scripts/agent_flow/vendor/
# hermes-agent-*.patch; этот скрипт применяет их идемпотентно
# (git apply --reverse --check => уже применён; git apply --check => можно
# применить). Вызывать ПОСЛЕ обновления hermes-agent.
HERMES_AGENT_DIR="${HERMES_AGENT_DIR:-/home/builder/.hermes/hermes-agent}"

apply_hermes_agent_patch() {
    local patch="$1"
    if ! git -C "$HERMES_AGENT_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        echo "  SKIP hermes-agent patch ($HERMES_AGENT_DIR not a git checkout)"
        return 0
    fi
    if [ ! -f "$patch" ]; then
        echo "  SKIP hermes-agent patch ($patch not found)"
        return 0
    fi
    echo "==> hermes-agent patch: $patch"
    # Уже применён?
    if ( cd "$HERMES_AGENT_DIR" && git apply --reverse --check "$patch" >/dev/null 2>&1 ); then
        echo "  OK   patch already applied (reverse-check clean)"
        return 0
    fi
    # Применится чисто?
    if ( cd "$HERMES_AGENT_DIR" && git apply --check "$patch" >/dev/null 2>&1 ); then
        if $DRY_RUN; then
            echo "  [DRY] would apply patch in $HERMES_AGENT_DIR"
            return 0
        fi
        if ( cd "$HERMES_AGENT_DIR" && git apply "$patch" ); then
            echo "  APPLIED hermes-agent patch (re-run install.sh after every hermes-agent update)"
            return 0
        fi
        echo "  ERROR applying hermes-agent patch" >&2
        return 1
    fi
    echo "  ERROR patch does not apply cleanly to $HERMES_AGENT_DIR — upstream moved" >&2
    echo "         Regenerate with: bash scripts/agent_flow/agent-flow-regen-vendor-patch.sh $patch" >&2
    echo "         See also: ретро t_f00676f8 (original) / t_49c2b63f (regen helper)" >&2
    return 1
}

# ---------------------------------------------------------------------------
# MAINTENANCE/peak probe для kanban auto-decomposer (ретро t_1d467636).
#
# Проблема: auto-decomposer создавал карточки даже внутри MAINTENANCE-окна
# DeepSeek peak (16.08 09:10Z = 12:10 MSK), потому что не проверял флаг,
# который знают merge-gate G1 и agents_sleep.sh. Фикс (vendor-патч
# hermes-agent-auto-decompose-maintenance.patch) добавляет в decompose_task
# проверку `kanban.maintenance_probe_command`: exit 0 = MAINTENANCE активна,
# декомпозиция откладывается («defer decompose (MAINTENANCE)»).
#
# Здесь мы прописываем эту команду в config.yaml (идемпотентно, один раз):
# probe = наличие файла MAINTENANCE на origin/develop в agents-sleep-repo —
# тот же источник истины, что у agents_sleep.sh.
ensure_kanban_maintenance_probe() {
    local cfg="${HERMES_CONFIG_YAML:-/home/builder/.hermes/config.yaml}"
    local repo="${AGENTS_SLEEP_REPO:-$HERMES_HOME/profiles/devops/agents-sleep-repo}"
    local probe_cmd="git -C $repo ls-tree origin/develop --name-only 2>/dev/null | grep -qx MAINTENANCE"
    if [ ! -f "$cfg" ]; then
        echo "  SKIP maintenance probe ($cfg not found)"
        return 0
    fi
    if grep -q "maintenance_probe_command" "$cfg" 2>/dev/null; then
        echo "  OK   maintenance_probe_command already present in $cfg"
        return 0
    fi
    if $DRY_RUN; then
        echo "  [DRY] would add kanban.maintenance_probe_command to $cfg"
        return 0
    fi
    # Тонко: добавляем секцию kanban: с maintenance_probe_command в конец
    # config.yaml, если kanban-секции ещё нет. Если секция есть — не
    # трогаем (чтобы не сломать пользовательские kanban-настройки).
    if grep -q "^kanban:" "$cfg" 2>/dev/null; then
        echo "  WARN kanban: section exists in $cfg but no maintenance_probe_command — add manually:"
        echo "       kanban:"
        echo "         maintenance_probe_command: \"$probe_cmd\""
        return 0
    fi
    printf '\nkanban:\n  maintenance_probe_command: "%s"\n' "$probe_cmd" >> "$cfg"
    echo "  ADDED kanban.maintenance_probe_command to $cfg (retro t_1d467636)"
}

echo
echo "==> Anti-escape guard: проверяю, что ни один файл в $HERMES_SCRIPTS_DIR не указывает наружу"
ANTI_ESCAPE_OK=true
for f in "${EXPECTED[@]}"; do
    f_hermes="$HERMES_SCRIPTS_DIR/$f"
    if [ ! -e "$f_hermes" ]; then
        if [ -d "$HERMES_SCRIPTS_DIR" ]; then
            echo "  WARN $f (missing in $HERMES_SCRIPTS_DIR)"
        fi
        continue
    fi
    # В dry-run НЕ трогаем файлы (раскладка ещё не применена) — просто
    # сообщаем, что будет исправлено.
    if $DRY_RUN; then
        if [ -L "$f_hermes" ]; then
            echo "  [DRY-WOULD-FIX] $f (symlink -> $(readlink "$f_hermes")); will become regular file"
            ANTI_ESCAPE_OK=false
        fi
        continue
    fi
    if [ -L "$f_hermes" ]; then
        # symlink в HERMES_SCRIPTS_DIR — гарантированно fail guard-а.
        # cp -L: переходим по симлинку и копируем содержимое. Потом атомарно
        # заменяем симлинк на regular copy (mv удаляет симлинк и переименовывает
        # копию). Так как скрипт работает в режиме set -e, нам нужен controlled
        # shell-scripting: обернём в условный блок.
        echo "  FIX  $f (was symlink in $HERMES_SCRIPTS_DIR -> $(readlink "$f_hermes"); converting to regular copy)"
        if cp -L "$f_hermes" "${f_hermes}.__tmpsym__" 2>/dev/null \
            && rm -f "$f_hermes" \
            && mv "${f_hermes}.__tmpsym__" "$f_hermes"; then
            chmod +x "$f_hermes" 2>/dev/null || true
            ANTI_ESCAPE_OK=false
        else
            # cleanup если что-то пошло не так
            rm -f "${f_hermes}.__tmpsym__" 2>/dev/null || true
            echo "  ERROR $f: failed to convert symlink to regular file" >&2
            ANTI_ESCAPE_OK=false
        fi
    fi
    # final check: readlink -f должен остаться внутри HERMES_SCRIPTS_DIR.
    resolved="$(readlink -f "$f_hermes" 2>/dev/null || true)"
    case "$resolved" in
        "$HERMES_SCRIPTS_DIR"/*)
            ;;
        *)
            if [ "$ANTI_ESCAPE_OK" = "true" ]; then
                echo "  ERROR $f: still resolves outside ($resolved)" >&2
                ANTI_ESCAPE_OK=false
            fi
            ;;
    esac
done
if [ "$ANTI_ESCAPE_OK" = "true" ]; then
    if $DRY_RUN; then
        echo "  OK all files in $HERMES_SCRIPTS_DIR would stay/copy inside it"
    else
        echo "  OK all files in $HERMES_SCRIPTS_DIR resolve inside it"
    fi
else
    if $DRY_RUN; then
        echo "  !! [DRY] some $HERMES_SCRIPTS_DIR files would be auto-healed"
    else
        echo "  !! auto-healed $HERMES_SCRIPTS_DIR files (now regular copies)"
    fi
fi

# ---------------------------------------------------------------------
# POST_INSTALL_VERIFY (ретро 01.09 t_a3ba921e).
#
# Проблема: install.sh раскладывал скрипты по 4 TARGET_DIRS, а backend и
# analyst получали их через profile-create.sh (snapshot с 31.08 — до MERGE
# PR #1849 ADR-0035). В результате merge-gate.sh отстал на 24KB на
# половине профилей, и ADR-0035 (stale-after-upstream-fix detector) для
# них не работал.
#
# Решение: после раскладки прогоняем md5sum по всем TARGET_DIRS-папкам и
# каждому EXPECTED-файлу. Если host-копия != source-of-truth — fail loud
# (exit 3) + alert в drift-log, чтобы ретро-карточка поднималась не на
# следующем 30-мин тике drift-detect, а сразу.
#
# В dry-run проверка тоже делается, но выходит без exit !=0 (чтобы можно
# было обкатывать изменения локально без ложных алертов).
#
# Что НЕ покрывается этой проверкой:
#   - drift-detect.sh дальше снимает более глубокую телеметрию (md5 vs
#     origin/develop, BRANCH_ACTIVE-handling). Здесь мы только фиксируем
#     факт «install.sh разложил всё, что должен был»;
#   - cron-watchdog (Agent Flow Scripts Drift, no_agent, every 30m)
#     ловит хост↔origin-develop drift независимо от install.sh.
POST_INSTALL_VERIFY_FAIL=0
POST_INSTALL_ALERT_LOG="${POST_INSTALL_ALERT_LOG:-/home/builder/.hermes/profiles/devops/cron/output/agent-flow-drift.alert.log}"
post_install_alert() {
    if $DRY_RUN; then
        return 0
    fi
    mkdir -p "$(dirname "$POST_INSTALL_ALERT_LOG")" 2>/dev/null || true
    printf '[%s] POST_INSTALL_VERIFY FAILED: %s\n' "$(date -Iseconds)" "$1" >> "$POST_INSTALL_ALERT_LOG" 2>/dev/null || true
}

echo
echo "==> Post-install md5 verification across ${#TARGET_DIRS[@]} target dirs (ретро 01.09 t_a3ba921e)"
# Ретро 01.09 t_a3ba921e: post_install_verify НАКАПЛИВАЕТ результат в
# POST_INSTALL_VERIFY_FAIL, но НЕ делает exit здесь. Итоговое решение
# принимается в самом конце скрипта (EXIT_AT_END блок ниже) — это важно
# для совместимости с drift-detect'овой branch_active_autofix(): она
# запускает `REPO_DIR=$wt bash $wt_install` без INSTALL_TARGET_DIRS
# override, install.sh берёт дефолтные реальные пути и verify делает
# всё, что может. Если verify повалится из-за тестовых WORK-папок
# (не относящихся к реальному хосту), мы НЕ должны ломать branch_active
# autofix midflight. Финальный exit 3 происходит после ВСЕХ операций.
post_install_verify() {
    local f t src_md5 dst_md5
    for f in "${EXPECTED[@]}"; do
        src="$SCRIPT_DIR/$f"
        [ -f "$src" ] || continue
        src_md5="$(md5sum "$src" | awk '{print $1}')"
        for t in "${TARGET_DIRS[@]}"; do
            if [ "$t" = "$SCRIPT_DIR" ]; then
                continue  # SOT против самого себя не сверяем
            fi
            if [ ! -d "$t" ]; then
                # Директория профиля просто не развёрнута — это SKIP, не FAIL.
                # (Раньше install.sh в этом случае тоже SKIP'ал — поведение
                # согласованное.)
                echo "  SKIP $f in $t (target dir not present)"
                continue
            fi
            if [ ! -f "$t/$f" ]; then
                # Файл отсутствует — раскладка должна была его положить.
                # Это уже баг (см. ретро 01.09).
                echo "  FAIL $f missing in $t"
                POST_INSTALL_VERIFY_FAIL=1
                post_install_alert "MISSING: $t/$f (expected after install.sh)"
                continue
            fi
            dst_md5="$(md5sum "$t/$f" | awk '{print $1}')"
            if [ "$dst_md5" != "$src_md5" ]; then
                echo "  FAIL $f differs in $t (source=$src_md5 dst=$dst_md5)"
                POST_INSTALL_VERIFY_FAIL=1
                post_install_alert "MD5 MISMATCH: src=$src_md5 dst=$dst_md5 file=$t/$f (post-install check)"
            else
                echo "  OK   $f in $t"
            fi
        done
    done
}
post_install_verify
if [ "$POST_INSTALL_VERIFY_FAIL" = "1" ]; then
    echo "  !! post-install verify reported FAIL — drift-devops карточка"
    echo "     может быть поднята drift-detect'ом; alert-лог:"
    echo "       $POST_INSTALL_ALERT_LOG"
else
    echo "  OK post-install verify passed: ${#TARGET_DIRS[@]} targets × $(printf '%s\n' "${EXPECTED[@]}" | wc -l) files"
fi
# NB: итоговый exit 3 происходит в EXIT_AT_END блоке в самом низу скрипта,
# а НЕ здесь — иначе branch_active_autofix в drift-detect ломается midflight.

# --- ensure_cron_job (дедуп 30.08) ------------------------------------------
# Общее тело трёх регистраторов ниже: они отличались только именем/скриптом/
# расписанием, но каждый нёс свою копию проверок и своё сообщение об ошибке
# (~30 строк ×3, python-guard был скопирован дословно дважды).
#
# Аргументы: <profile> <job_name> <job_script> <schedule> [guard]
#
# guard — как проверяем «джоб уже есть»:
#   interval — есть enabled-джоб на этот script с schedule.kind=interval.
#              Это правильная проверка: STALE once-job (state=completed,
#              enabled=false) её НЕ проходит, и рядом создаётся живой
#              interval-джоб (ретро 23.08+25.08 t_98bb3a1d/t_24e645e7 —
#              e2e-rotation простоял 60+ часов именно на таком once-джобе).
#   any      — есть ЛЮБОЙ джоб с этим script в jobs.json (исторический
#              вариант ensure_cleanup_cron с 13.08).
#              ⚠️ Слабее: completed once-джоб он засчитает как «живой», и
#              cleanup-249 останется незарегистрированным. Оставлен как есть,
#              чтобы дедуп не менял поведение крона на живом хосте; перевод
#              cleanup на interval-guard — отдельное решение, см. §5bis
#              docs/process-fix-roadmap.md.
#
# HERMES_PROFILES_ROOT переопределяется в тестах (см.
# tests/test_install_ensure_cleanup_cron.sh); дефолт — путь на хосте ротации.
ensure_cron_job() {  # $1=profile $2=job_name $3=job_script $4=schedule [$5=guard]
    local profile="$1" job_name="$2" job_script="$3" job_schedule="$4"
    local guard="${5:-interval}"
    local profiles_root="${HERMES_PROFILES_ROOT:-/home/builder/.hermes/profiles}"
    local jobs_file="$profiles_root/$profile/cron/jobs.json"
    local registered=1

    if ! command -v hermes >/dev/null 2>&1; then
        echo "  SKIP ensure-cron ($job_script): hermes CLI not on PATH (nothing to register)"
        return 0
    fi
    if [ ! -f "$jobs_file" ]; then
        echo "  SKIP ensure-cron ($job_script): $jobs_file not present ($profile profile not set up here)"
        return 0
    fi

    if [ "$guard" = "any" ]; then
        if grep -q "\"script\": \"$job_script\"" "$jobs_file"; then registered=0; fi
    else
        if python3 -c "
import json, sys
try:
    with open('$jobs_file') as f:
        d = json.load(f)
except Exception:
    # Нечитаемый/битый jobs.json — считаем «джоб есть» и НЕ создаём новый
    # (исходное поведение guard'а с 23.08: лучше не наплодить дублей).
    sys.exit(0)
for j in d.get('jobs', []):
    if j.get('script') == '$job_script' and j.get('schedule', {}).get('kind') == 'interval' and j.get('enabled'):
        sys.exit(0)
sys.exit(1)
" 2>/dev/null; then registered=0; fi
    fi

    if [ "$registered" -eq 0 ]; then
        echo "  OK   cron job '$job_name' already registered ($job_script)"
        return 0
    fi

    echo "  ADD  registering cron job '$job_name' ($profile, $job_schedule, no_agent)"
    if $DRY_RUN; then
        echo "  [DRY] hermes --profile $profile cron create '$job_schedule' --name '$job_name' --script '$job_script' --no-agent --deliver local --workdir '$REPO_DIR'"
        return 0
    fi
    if hermes --profile "$profile" cron create "$job_schedule" \
        --name "$job_name" \
        --script "$job_script" \
        --no-agent \
        --deliver local \
        --workdir "$REPO_DIR" >/dev/null 2>&1; then
        echo "  ADD  cron job created: $job_name ($job_script, $job_schedule)"
    else
        echo "  WARN cron job creation failed (non-fatal): $job_name — register manually:"
        echo "       hermes --profile $profile cron create '$job_schedule' --name '$job_name' --script '$job_script' --no-agent --deliver local --workdir $REPO_DIR"
    fi
}

echo
echo "==> Ensure cron job registration: agent-flow-cleanup-249.sh (ретро 13.08 t_04d73108)"
# Проблема: cleanup-249 раскладывался install.sh, но cron-job НЕ создавался —
# stale round-ветки (61-76/100-103) копились на origin. Регистрируем джоб
# идемпотентно в devops-профиле: every 6h, no_agent (скрипт = джоб).
# Регистрация переживает install.sh: каждый запуск (в т.ч. auto-fix из
# drift-detect) проверяет jobs.json и создаёт недостающий джоб.
ensure_cleanup_cron() {
    # guard=any — исторический (и более слабый) вариант, см. ensure_cron_job.
    ensure_cron_job devops "Agent Flow Cleanup 249" "agent-flow-cleanup-249.sh" "every 6h" any
}
ensure_cleanup_cron

echo
echo "==> Ensure cron job registration: e2e-process auto-rotation (ретро 23.08+25.08 t_98bb3a1d/t_24e645e7)"
# Проблема: agent-flow-e2e-process-launcher.sh раскладывался install.sh (commit
# bd7e509d), но cron-job НЕ создавался — он создавался вручную в тикете 23.08
# через `hermes cron create 'once in 20m'`. После первого тика джоб
# переходил в state=completed и больше НЕ запускался (в отличие от
# interval-расписания, once без повторов не self-reschedules). Результат:
# e2e-rotation простаивал 60+ часов, PR с label needs-e2e копился без
# подхвата (PR #1565 провисел 3.5ч+ на момент ретро).
#
# Решение: ensure_e2e_process_cron() — идемпотентная функция, регистрирующая
# interval-job (every 20m) в devops-профиле, no_agent (скрипт = launcher).
# Дубль-guard по двум критериям: (1) script-имя в jobs.json, (2) job с
# правильным расписанием. Это покрывает и кейс «старый once-job завис в
# jobs.json» — он enabled=false, новый interval-job будет зарегистрирован
# отдельно, и оба не конфликтуют.
#
# Регистрация переживает install.sh: каждый запуск (в т.ч. auto-fix из
# drift-detect) проверяет jobs.json и создаёт недостающий джоб.
ensure_e2e_process_cron() {
    ensure_cron_job devops "e2e-process auto-rotation" "agent-flow-e2e-process-launcher.sh" "every 20m" interval
}
ensure_e2e_process_cron

echo
echo "==> Ensure cron job registration: orphan blocked-watchdog (ретро t_1d0426e3)"
# Проблема: agent-flow-blocked-watchdog.sh раскладывается install.sh (commit
# от t_1d0426e3), но cron-job НЕ создаётся автоматически. Без него manual
# cleanup (t_547e17a7, t_3aa4c587, t_307bae4a) придётся повторять на каждом
# новом orphan — pattern «карточки-призраки» системный.
#
# Решение: ensure_blocked_watchdog_cron() — идемпотентная функция,
# регистрирующая interval-job (every 4h) в devops-профиле, no_agent
# (скрипт = watchdog). Дубль-guard по (script + interval + enabled).
#
# Регистрация переживает install.sh: каждый запуск (в т.ч. auto-fix из
# drift-detect) проверяет jobs.json и создаёт недостающий job.
ensure_blocked_watchdog_cron() {
    ensure_cron_job devops "Agent Flow Blocked Watchdog" "agent-flow-blocked-watchdog.sh" "every 4h" interval
}
ensure_blocked_watchdog_cron
echo "==> Ensure cron job registration: cron-надзор mis-scope карточек (ADR-0036 §4.3, ретро t_aa585aa7)"
# Проблема: agent-flow-blocked-watchdog-scope.sh раскладывается install.sh
# (commit от t_aa585aa7), но cron-job НЕ создаётся автоматически. Без него
# Шифу вынужден мониторить running-список сам — нарушает «не делай руками».
#
# Решение: ensure_blocked_watchdog_scope_cron() — идемпотентная функция,
# регистрирующая interval-job (every 1h) в devops-профиле, no_agent.
# Дубль-guard по (script + interval + enabled). Каждый тик сканирует все
# kanban-доски, для mis-scope running-карточек пишет ОДИН alert-комментарий
# (idempotent). Шифу eyeball'ит, решает kill/reassign/keep.
ensure_blocked_watchdog_scope_cron() {
    local profile_dir="/home/builder/.hermes/profiles/devops"
    local jobs_file="$profile_dir/cron/jobs.json"
    local job_name="Agent Flow Blocked Watchdog Scope (ADR-0036 §4.3)"
    local job_script="agent-flow-blocked-watchdog-scope.sh"
    local job_schedule="every 1h"

    if ! command -v hermes >/dev/null 2>&1; then
        echo "  SKIP ensure-blocked-scope-cron: hermes CLI not on PATH (nothing to register)"
        return 0
    fi
    if [ ! -f "$jobs_file" ]; then
        echo "  SKIP ensure-blocked-scope-cron: $jobs_file not present (devops profile not set up here)"
        return 0
    fi

    # Guard: уже есть interval-job на этот script.
    if python3 -c "
import json, sys
try:
    with open('$jobs_file') as f:
        d = json.load(f)
except Exception:
    sys.exit(0)
for j in d.get('jobs', []):
    if j.get('script') == '$job_script' and j.get('schedule', {}).get('kind') == 'interval' and j.get('enabled'):
        sys.exit(0)
sys.exit(1)
" 2>/dev/null; then
        echo "  OK   cron job '$job_name' already registered (interval, enabled)"
        return 0
    fi

    echo "  ADD  registering cron job '$job_name' (devops, $job_schedule, no_agent)"
    if $DRY_RUN; then
        echo "  [DRY] hermes --profile devops cron create '$job_schedule' --name '$job_name' --script '$job_script' --no-agent --deliver local --workdir '$REPO_DIR'"
        return 0
    fi
    if hermes --profile devops cron create "$job_schedule" \
        --name "$job_name" \
        --script "$job_script" \
        --no-agent \
        --deliver local \
        --workdir "$REPO_DIR" >/dev/null 2>&1; then
        echo "  ADD  cron job created: $job_name ($job_script, $job_schedule)"
    else
        echo "  WARN cron job creation failed (non-fatal): $job_name — register manually:"
        echo "       hermes --profile devops cron create '$job_schedule' --name '$job_name' --script '$job_script' --no-agent --deliver local --workdir $REPO_DIR"
    fi
}
ensure_blocked_watchdog_scope_cron

echo
echo "==> Ensure cron job registration: ночной ревью (ADR-0049)"
# Проблема: весь надзор конвейера реактивный и поштучный — никто не смотрит на
# день целиком и никто не перечитывает код, который воркеры за сутки написали.
# ADR-0049 закрывает этот пробел ночным ревью-циклом.
#
# Решение: ensure_nightly_review_cron() — идемпотентная регистрация
# interval-job (every 1h) в devops-профиле, no_agent. Час запуска НЕ зашит в
# расписание крона: сам скрипт пропускает тик вне окна
# [NIGHTLY_REVIEW_HOUR, +NIGHTLY_REVIEW_WINDOW_HOURS) и ставит sentinel на
# ревью-сутки. Ежечасный тик поэтому дешёвый (99% тиков = один `date` + exit
# 0), зато ревью не теряется, если хост лежал ровно в 02:00 или MAINTENANCE
# висел первый час окна.
ensure_nightly_review_cron() {
    ensure_cron_job devops "Agent Flow Nightly Review (ADR-0049)" "agent-flow-nightly-review.sh" "every 1h" interval
}
ensure_nightly_review_cron

echo
echo "==> md5sum verify: 6 copies of process-launcher / watchdog scripts are byte-identical (retro 25.08 t_24e645e7, extended 01.09 t_a3ba921e)"
# Проблема (ретро 25.08): agent-flow-*-launcher/watchdog раскладывается в N
# копий (agent-flow/, devops/, architect/, backend/, analyst/, + legacy
# ~/.hermes/scripts/). Если хотя бы одна копия отстала (drift между
# hardlink и copy, или вообще не донеслась через profile-create.sh →
# install.sh), cron может выполнять версию, не соответствующую SOT в репо.
# Verify-блок показывает md5 каждой копии и hard-fail'ит при расхождении —
# чтобы drift-devops карточка открывалась на ЭТОМ запуске install.sh, а не
# через 30 мин drift-detect.
#
# Ретро 01.09 t_a3ba921e: TARGET_DIRS расширен до 6 (добавлены backend и
# analyst). Эти 5 watchdog-файлов ВСЕ должны быть во ВСЕХ 6 копиях —
# иначе блокирующий файл (agent-flow-blocked-watchdog.sh,
# agent-flow-blocked-watchdog-scope.sh) у части профилей пустой → cron
# для тех профилей ничего не делает, накапливается drift. Поэтому
# verify_three_copies_md5sum заменён на единую итерацию по TARGET_DIRS для
# каждого из 5 watchdog-файлов.
verify_md5sum_copies() {
    local label="$1"
    shift
    local sums=()
    local paths=()
    local path
    for path in "$@"; do
        paths+=("$path")
        if [ ! -f "$path" ]; then
            # Один из TARGET_DIRS не донёс файл — это DRIFT, не WARN.
            # post_install_verify ниже поймает ровно эту ситуацию для всех
            # файлов, а здесь жёсткий hard-fail: нельзя разложить hardlink
            # для cron-запускаемого watchdog'а в 5 копий и пропустить 1.
            echo "  ERROR $label: missing $path (post-install verify failed)"
            return 1
        fi
        sums+=("$(md5sum "$path" 2>/dev/null | awk '{print $1}')")
    done
    local first="${sums[0]}"
    local s
    for s in "${sums[@]}"; do
        if [ "$s" != "$first" ]; then
            echo "  ERROR $label: md5sum drift detected across copies:"
            for p in "${paths[@]}"; do
                echo "         $(md5sum "$p" 2>/dev/null) $p"
            done
            echo "         Run: $REPO_DIR/scripts/agent_flow/install.sh (without --dry-run) to re-link"
            return 1
        fi
    done
    echo "  OK   $label: $first across ${#sums[@]} copies"
}

# Список 5 скриптов, для которых раньше verify был hardcoded на 3+1 пути.
# Берём имена из EXPECTED выше по якорю «watchdog/laucher» — надёжнее, чем
# перечислять, и автоматически подхватит будущие расширения.
_WATCHDOG_LAUNCHER_FILES=(
    agent-flow-e2e-process-launcher.sh
    agent-flow-blocked-watchdog.sh
    agent-flow-e2e-fail-streak-watchdog.sh
    padavan-step4-voice-smoke.sh
    agent-flow-blocked-watchdog-scope.sh
    agent-flow-nightly-review.sh
)

_md5_verify_fail=0
for f in "${_WATCHDOG_LAUNCHER_FILES[@]}"; do
    # Собираем пути для каждого TARGET_DIR (кроме SOT, иначе мы сравниваем
    # файл с самим собой). Совпадает с post_install_verify выше.
    _paths=()
    for t in "${TARGET_DIRS[@]}"; do
        [ "$t" = "$SCRIPT_DIR" ] && continue
        _paths+=("$t/$f")
    done
    if ! verify_md5sum_copies "$f" "${_paths[@]}"; then
        _md5_verify_fail=1
    fi
done
# NB: итоговый exit 4 (md5 verify FAIL) происходит в EXIT_AT_END блоке
# в самом низу скрипта, а НЕ здесь — по той же причине, что и
# post_install_verify: branch_active_autofix должен отработать штатно.


echo
echo "==> Telegram token sanity (retro 12.08 t_5af222ea): >1 active TELEGRAM_BOT_TOKEN = reconnect loop"
TOKEN_HOLDERS=()
for envf in /home/builder/.hermes/.env /home/builder/.hermes/profiles/*/.env; do
    [ -f "$envf" ] || continue
    while IFS= read -r ln; do
        case "$ln" in
            TELEGRAM_BOT_TOKEN=*)
                val="${ln#TELEGRAM_BOT_TOKEN=}"
                val="${val%\"}"; val="${val#\"}"
                if [ -n "$val" ]; then
                    TOKEN_HOLDERS+=("$(basename "$(dirname "$envf")")")
                fi
                break
                ;;
        esac
    done < "$envf"
done
if [ "${#TOKEN_HOLDERS[@]}" -gt 1 ]; then
    echo "  !! WARNING: ${#TOKEN_HOLDERS[@]} profiles hold an active TELEGRAM_BOT_TOKEN: ${TOKEN_HOLDERS[*]}"
    echo "     Telegram allows ONE getUpdates consumer per token; the rest will loop"
    echo "     'token already in use' forever. Keep the token only in the owner profile."
else
    echo "  OK  telegram token holders: ${TOKEN_HOLDERS[*]:-none}"
fi

# ---------------------------------------------------------------------------
# Sweep stale .bak skill directories (retro 23.08 t_ab1cc381).
#
# Проблема: hermes-agent/tools/skills_sync.py при обновлении skill-а
# использует shutil.move(dest, dest.with_suffix('.bak')) как rollback-механизм
# (tools/skills_sync.py:907). Если процесс прерывается между move и
# последующим rmtree('.bak') (строка 916), .bak-директория остаётся
# ВНУТРИ ~/.hermes/skills/<category>/ рядом с живым dest. После этого
# hermes-agent/tools/skills_tool.py::skill_view(name) считает оба
# (dest и dest.bak) как кандидатов и отказывается резолвить:
#   "Ambiguous skill name spike: 2 skills match"
# → воркеры падают с "Unknown skill(s): spike".
#
# Решение: install.sh при каждом запуске (в т.ч. drift-detect / deploy)
# подметает .bak-директории в skills/, оставляя живой dest как есть.
# Идемпотентно: если .bak нет — no-op.
#
# Scope:
#   - ~/.hermes/skills/<category>/<name>.bak/
#   - ~/.hermes/profiles/<profile>/skills/<category>/<name>.bak/
# НЕ трогаем обычные файлы *.bak.* (это метки времени от _remove_existing)
# и НЕ удаляем ничего внутри уже-установленных скиллов.
#
# Это НЕ лечит upstream-баг (он живёт в hermes-agent), но убирает
# симптомы на хосте при следующем install-цикле. Upstream-фикс в
# EXCLUDED_SKILL_DIRS / skill_utils.py отслеживается отдельно.
#
# Размещён ПЕРЕД vendor patches / MAINTENANCE probe — гарантирует выполнение
# даже если apply_hermes_agent_patch падает (pre-existing баг с
# устаревшими upstream-патчами, см. ретро t_f00676f8).
sweep_stale_skill_baks() {
    local roots=(
        "/home/builder/.hermes/skills"
        /home/builder/.hermes/profiles/*/skills
    )
    local removed=0
    local inspected=0
    local root bak
    for root in "${roots[@]}"; do
        # glob может не раскрыться, если нет profiles; тогда пропускаем.
        [ -d "$root" ] || continue
        # find depth-3: <root>/<category>/<name>.bak — ровно такая форма
        # генерируется dest.with_suffix('.bak') в skills_sync.py.
        while IFS= read -r bak; do
            inspected=$((inspected + 1))
            if $DRY_RUN; then
                echo "  [DRY] would sweep stale skill backup: ${bak#$root/}"
            else
                # Используем 'mv' в /tmp под именем с timestamp — если что-то
                # пойдёт не так, восстановимо вручную из той же папки.
                local trash="/tmp/hermes-skill-bak-sweep-$(date -u +%Y%m%dT%H%M%SZ)"
                mkdir -p "$trash"
                mv "$bak" "$trash/"
                echo "  SWEEP ${bak#$root/} -> $trash/ (recovery: $trash/${bak##*/})"
                removed=$((removed + 1))
            fi
        done < <(find "$root" -mindepth 2 -maxdepth 2 -type d -name '*.bak' 2>/dev/null)
    done
    if [ "$removed" -eq 0 ] && [ "$inspected" -eq 0 ]; then
        echo "  OK   no stale skill .bak dirs (clean)"
    elif [ "$removed" -eq 0 ]; then
        echo "  OK   inspected $inspected .bak dirs in dry-run (no actual removal)"
    else
        echo "  OK   swept $removed stale skill .bak dirs (inspected $inspected)"
    fi
}

echo
echo "==> Sweep stale skill .bak dirs (retro 23.08 t_ab1cc381)"
sweep_stale_skill_baks


echo
echo "==> hermes-agent vendor patches"
for _patch in "$SCRIPT_DIR"/vendor/hermes-agent-*.patch; do
    [ -f "$_patch" ] || continue
    apply_hermes_agent_patch "$_patch"
done
echo
echo "==> kanban MAINTENANCE probe config (retro t_1d467636)"
ensure_kanban_maintenance_probe


echo
echo "==> Done. Verify:"
if ! $DRY_RUN; then
    # Ретро 01.09 t_a3ba921e: список путей расширен с 4 до 6 (добавлены
    # profiles/backend и profiles/analyst). Список дублирует TARGET_DIRS
    # ниже в EXIT_AT_END через тот же пост-install verify. Здесь просто
    # печатаем итог в operator-friendly виде, в т.ч. inode/md5 по 6 путям.
    for f in "${EXPECTED[@]}"; do
        for fp in \
            "/home/builder/.hermes/profiles/agent-flow/scripts/$f" \
            "/home/builder/.hermes/profiles/architect/scripts/$f" \
            "/home/builder/.hermes/profiles/devops/scripts/$f" \
            "/home/builder/.hermes/profiles/backend/scripts/$f" \
            "/home/builder/.hermes/profiles/analyst/scripts/$f" \
            "/home/builder/.hermes/scripts/$f"; do
            if [ -e "$fp" ]; then
                inode="$(stat -c '%i' "$fp" 2>/dev/null)"
                kind="$( [ -L "$fp" ] && echo symlink || echo "reg(inode=$inode)" )"
                echo "  $fp -> $(readlink -f "$fp"): $(md5sum "$fp" 2>/dev/null | cut -c1-10) [$kind]"
            fi
        done
        echo "  ---"
    done
fi

# ===========================================================================
# EXIT_AT_END (ретро 01.09 t_a3ba921e)
#
# Финальный код возврата install.sh вычисляется здесь — ПОСЛЕ всех операций,
# чтобы branch_active_autofix в drift-detect мог отработать штатно (он
# вызывает install.sh из временного worktree на origin/develop, и если
# verify упадёт из-за несовпадения с реальным HERMES_HOME/профилями, mid-
# flight exit поломает self-healing flow).
#
# Exit codes:
#   0 — OK (все verify прошли)
#   3 — POST_INSTALL_VERIFY FAIL (host copy != source-of-truth)
#   4 — md5 verify failed для одного из watchdog/launcher (см. выше)
#   <другое> — другие error'ы (например, vendor patch fail)
# ===========================================================================
FINAL_EXIT=0
if [ "$POST_INSTALL_VERIFY_FAIL" = "1" ]; then
    echo
    echo "==> POST_INSTALL_VERIFY FAIL: host copy != source-of-truth"
    echo "    Alert:    $POST_INSTALL_ALERT_LOG"
    echo "    Manual:   bash $0 (re-run with INSTALL_TARGET_DIRS=$WORK override for tests)"
    if ! $DRY_RUN; then
        FINAL_EXIT=3
    fi
fi
if [ "$_md5_verify_fail" = "1" ]; then
    if [ "$FINAL_EXIT" = "0" ]; then
        echo "==> md5 verify FAIL: watchdog/launcher copies differ (см. ERROR выше)"
        if ! $DRY_RUN; then
            FINAL_EXIT=4
        fi
    fi
fi
if [ "$FINAL_EXIT" != "0" ]; then
    echo "==> exit $FINAL_EXIT"
    exit "$FINAL_EXIT"
fi
exit 0
