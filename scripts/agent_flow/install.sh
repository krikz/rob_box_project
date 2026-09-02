#!/bin/bash
# install.sh — раскладка процессных скриптов agent-flow по нужным директориям.
#
# Source of truth: <repo>/scripts/agent_flow/*.sh (эта папка).
# Копии (которые ищет cron при старте):
#   1. /home/builder/.hermes/profiles/agent-flow/scripts/   — каноническое место
#   2. /home/builder/.hermes/profiles/architect/scripts/    — где cron сейчас ищет
#   3. /home/builder/.hermes/scripts/                       — legacy (cron тоже стартует)
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
# Резервные пути раскладки (если hardlink невозможен — cross-device):
#   1) cp -aL (копия содержимого) — fallback по умолчанию для symlink;
#   2) symbolic link — крайний случай, только для директорий вне `scripts_dir`;
#      для самой `~/.hermes/scripts/` symlink ЗАПРЕЩЁН (сломает guard).
#
# Гарантии:
#   - Все 3 (или сколько есть) путей ссылаются на одну и ту же inode-копию
#     (hardlink) либо на одинаковое содержимое (cp);
#   - Правка в репо (через PR/merge) автоматически расходится по всем путям
#     сразу при следующем запуске этого скрипта;
#   - Ни один файл в TARGET_DIRS не указывает за пределы своей директории
#     (anti-escape guard в конце);
#   - Нет дубликатов, нет drift (см. scripts/agent_flow/README.md).
#
# Запуск:
#   ./scripts/agent_flow/install.sh            # раскладка на хост
#   ./scripts/agent_flow/install.sh --dry-run  # только показать что сделает
#
# Идемпотентен — повторный запуск обновляет ссылки, ничего не ломает.

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
    # Protocol-violation recovery watchdog (ретро t_52a6b973, 2026-09-02):
    # no-agent job, каждый час сканирует todo/ready-карточки assignee=agent-flow
    # с protocol_violation streak (worker exited rc=0 без kanban_complete),
    # ищет для issue MERGED PR в base branch, и вызывает kanban complete
    # от имени watchdog'а с verifier-summary. Закрывает карточки-призраки
    # типа t_0ed5689a, где вся работа сделана через дочерние задачи, но
    # корневая карточка висит в todo с consecutive_crashes=4.
    # Регистрация cron-job делается в ensure_pv_watchdog_cron ниже.
    agent-flow-protocol-violation-watchdog.sh
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
if [ -n "${INSTALL_TARGET_DIRS:-}" ]; then
    IFS=':' read -r -a TARGET_DIRS <<< "$INSTALL_TARGET_DIRS"
else
    TARGET_DIRS=(
        "/home/builder/.hermes/profiles/agent-flow/scripts"
        "/home/builder/.hermes/profiles/architect/scripts"
        "/home/builder/.hermes/profiles/devops/scripts"
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
echo "==> Ensure cron job registration: protocol-violation recovery (ретро t_52a6b973)"
# Проблема: карточки assignee=agent-flow могут застрять в todo с
# consecutive_crashes=4 (worker exited rc=0 без kanban_complete), хотя вся
# работа уже merged в develop через дочерние задачи (t_0ed5689a — типичный
# случай). Без watchdog'а Шифу вынужден вручную вызывать kanban complete.
#
# Решение: ensure_pv_watchdog_cron() — идемпотентная функция, регистрирующая
# interval-job (every 1h) в devops-профиле, no_agent. Сканирует todo/ready
# карточки assignee=agent-flow с protocol_violation streak, проверяет merged
# PR в base branch, вызывает kanban complete с verifier-summary.
# Дубль-guard по (script + interval + enabled).
ensure_pv_watchdog_cron() {
    local profile_dir="/home/builder/.hermes/profiles/devops"
    local jobs_file="$profile_dir/cron/jobs.json"
    local job_name="Agent Flow Protocol Violation Recovery (t_52a6b973)"
    local job_script="agent-flow-protocol-violation-watchdog.sh"
    local job_schedule="every 1h"

    if ! command -v hermes >/dev/null 2>&1; then
        echo "  SKIP ensure-pv-watchdog-cron: hermes CLI not on PATH"
        return 0
    fi
    if [ ! -f "$jobs_file" ]; then
        echo "  SKIP ensure-pv-watchdog-cron: $jobs_file not present"
        return 0
    fi

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
        echo "  OK   cron job '$job_name' already registered"
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
ensure_pv_watchdog_cron

echo
echo "==> md5sum verify: 3 copies of agent-flow scripts are byte-identical (retro 25.08 t_24e645e7)"
# Проблема: launcher agent-flow-e2e-process-launcher.sh раскладывается в 3
# копии (agent-flow/, devops/, architect/) + .hermes/scripts/. Если хотя бы
# одна копия отстала (drift между hardlink и copy), cron может выполнять
# версию, не соответствующую SOT в репо. Verify-блок в самом низу
# install.sh показывает md5sum каждой копии, но только при !DRY_RUN — здесь
# мы делаем явный hard-fail verify с читаемым выводом.
verify_three_copies_md5sum() {
    local label="$1"
    shift
    local sums=()
    local path
    for path in "$@"; do
        if [ ! -f "$path" ]; then
            echo "  WARN $label: missing $path (skipping md5sum check)"
            return 0
        fi
        sums+=("$(md5sum "$path" 2>/dev/null | awk '{print $1}')")
    done
    local first="${sums[0]}"
    local s
    for s in "${sums[@]}"; do
        if [ "$s" != "$first" ]; then
            echo "  ERROR $label: md5sum drift detected across copies:"
            for path in "$@"; do
                echo "         $(md5sum "$path" 2>/dev/null) $path"
            done
            echo "         Run: $REPO_DIR/scripts/agent_flow/install.sh (without --dry-run) to re-link"
            return 1
        fi
    done
    echo "  OK   $label: $first across ${#sums[@]} copies"
}
verify_three_copies_md5sum "agent-flow-e2e-process-launcher.sh" \
    "/home/builder/.hermes/profiles/agent-flow/scripts/agent-flow-e2e-process-launcher.sh" \
    "/home/builder/.hermes/profiles/architect/scripts/agent-flow-e2e-process-launcher.sh" \
    "/home/builder/.hermes/profiles/devops/scripts/agent-flow-e2e-process-launcher.sh" \
    "/home/builder/.hermes/scripts/agent-flow-e2e-process-launcher.sh"
verify_three_copies_md5sum "agent-flow-blocked-watchdog.sh" \
    "/home/builder/.hermes/profiles/agent-flow/scripts/agent-flow-blocked-watchdog.sh" \
    "/home/builder/.hermes/profiles/architect/scripts/agent-flow-blocked-watchdog.sh" \
    "/home/builder/.hermes/profiles/devops/scripts/agent-flow-blocked-watchdog.sh" \
    "/home/builder/.hermes/scripts/agent-flow-blocked-watchdog.sh"
verify_three_copies_md5sum "agent-flow-e2e-fail-streak-watchdog.sh" \
    "/home/builder/.hermes/profiles/agent-flow/scripts/agent-flow-e2e-fail-streak-watchdog.sh" \
    "/home/builder/.hermes/profiles/architect/scripts/agent-flow-e2e-fail-streak-watchdog.sh" \
    "/home/builder/.hermes/profiles/devops/scripts/agent-flow-e2e-fail-streak-watchdog.sh" \
    "/home/builder/.hermes/scripts/agent-flow-e2e-fail-streak-watchdog.sh"
verify_three_copies_md5sum "padavan-step4-voice-smoke.sh" \
    "/home/builder/.hermes/profiles/agent-flow/scripts/padavan-step4-voice-smoke.sh" \
    "/home/builder/.hermes/profiles/architect/scripts/padavan-step4-voice-smoke.sh" \
    "/home/builder/.hermes/profiles/devops/scripts/padavan-step4-voice-smoke.sh" \
    "/home/builder/.hermes/scripts/padavan-step4-voice-smoke.sh"
verify_three_copies_md5sum "agent-flow-blocked-watchdog-scope.sh" \
    "/home/builder/.hermes/profiles/agent-flow/scripts/agent-flow-blocked-watchdog-scope.sh" \
    "/home/builder/.hermes/profiles/architect/scripts/agent-flow-blocked-watchdog-scope.sh" \
    "/home/builder/.hermes/profiles/devops/scripts/agent-flow-blocked-watchdog-scope.sh" \
    "/home/builder/.hermes/scripts/agent-flow-blocked-watchdog-scope.sh"
verify_three_copies_md5sum "agent-flow-protocol-violation-watchdog.sh" \
    "/home/builder/.hermes/profiles/agent-flow/scripts/agent-flow-protocol-violation-watchdog.sh" \
    "/home/builder/.hermes/profiles/architect/scripts/agent-flow-protocol-violation-watchdog.sh" \
    "/home/builder/.hermes/profiles/devops/scripts/agent-flow-protocol-violation-watchdog.sh" \
    "/home/builder/.hermes/scripts/agent-flow-protocol-violation-watchdog.sh"

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
    for f in "${EXPECTED[@]}"; do
        for fp in \
            "/home/builder/.hermes/profiles/agent-flow/scripts/$f" \
            "/home/builder/.hermes/profiles/architect/scripts/$f" \
            "/home/builder/.hermes/profiles/devops/scripts/$f" \
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
