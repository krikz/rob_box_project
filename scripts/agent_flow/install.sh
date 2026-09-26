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
# Ретро 14.09 t_06956919: install.sh теперь раскладывает И САМОГО СЕБЯ.
# Раньше EXPECTED не включал install.sh, потому что считалось что
# install.sh на хосте — «мета», а не процессный скрипт. Это создало
# chicken-egg: drift-detect читает список через `bash install.sh --list-files`,
# поэтому install.sh не self-check'ился → 5 хостов катили со старой версией
# (1b767e5) пока origin/develop уже был на ec86158 (`vendor-patch
# upstream-move`). Когда оператор вручную тянул `git pull`, репо SOT
# обновлялось, но `bash scripts/agent_flow/install.sh` раскладывал
# EXPECTED (без install.sh) → на хостах оставалась старая версия.
# Решение: install.sh первый в EXPECTED → self-replace на каждом запуске.
# На первом запуске скрипт копирует себя под СТАРЫМ именем в TARGET_DIRS
# (содержимое одинаковое), затем в основном цикле `for f in EXPECTED`
# обрабатывает себя же как обычный файл (cp -al). Идемпотентно: при
# повторных запусках _install_one сравнивает inode/содержимое и пишет
# "OK already linked".
EXPECTED=(
    install.sh
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
    # Ночной голосовой марафон (docs/e2e/night-voice-marathon.md): no-agent
    # cron every 1h с внутренним гейтом по часу. Сценарии и раннер тянет из
    # origin/develop сам, но САМ скрипт должен лежать в scripts_dir — иначе
    # hermes scheduler отклонит job с «Script not found».
    agent-flow-night-marathon.sh
    agent-flow-handoff.sh
    round_ensure.sh
    # Round-formation module (issue #2299, 09.09.2026): единый владелец
    # ls-remote → max-N → freshness-check → create/reuse/recreate. Source'ится
    # из round_ensure.sh и agent-flow-e2e-process.sh. Должен лежать рядом
    # со скриптами во всех 4 target-папках.
    round_formation.sh
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
    # Cancel-stale-cards helper (ретро 15.09 t_197de62a): сканирует
    # kanban-доски, ловит карточки с provider-exhaust сигнатурой в
    # task_runs.summary ИЛИ tasks.last_failure_error, и блокирует их
    # (kind=capability, idempotent через sentinel-комментарий).
    # Дополняет watchdog-provider-quick.sh: тот реагирует на СВЕЖИЕ
    # маркеры (worker exit code = protocol violation), этот — на
    # исторические (когда worker crash-loop произошёл давно и стёрся
    # из свежего окна, но signal остался в summary/last_failure_error).
    # До этого фикса скрипт жил только в ~/.hermes/scripts/legacy и не
    # был в SOT репо → не раскладывался в профили, не контролировался
    # drift-detect'ом, и CRON НЕ регистрировался. Теперь — SOT +
    # cron every 5m в devops-профиле (см. ensure_cancel_provider_exhausted_cron).
    agent-flow-cancel-on-provider-exhausted.sh
    agent-flow-drift-detect.sh
    kanban-retro-create.sh
    # Worker-helper для контракта отчёта (ADR-0115, issue #2159, 2026-09-08):
    # воркер вызывает `bash scripts/agent_flow/kanban-report-write.sh $TASK_ID`
    # ПЕРЕД `kanban_complete` — скрипт собирает git diff/pytest/gh pr view и
    # пишет `docs/reports/kanban/<task_id>.md` (git tracked, переживает worktree GC).
    # Без него ретро/аудит через месяц невозможен — `kanban_complete` оставляет
    # только Result (~300 символов) + список путей без содержимого.
    kanban-report-write.sh
    # Worker-helpers для pre/post-work rebase protocol (issue #2438,
    # 2026-09-14): воркер вызывает `worker_pre_flight.sh` в самом начале
    # сессии (после claim, до кода) и `worker_post_flight.sh` перед
    # `kanban_complete`. Скрипты делают auto-rebase на origin/develop если
    # drift > MAX_BRANCH_BEHIND; при конфликте — пишут инструкцию в issue
    # и возвращают exit 1 (карточка остаётся в running). Без них воркеры
    # стартуют на устаревших ветках, PR diverged, merge-gate ловит
    # add/add конфликты (ретро PR #2351, #2363, issue #2438).
    worker_pre_flight.sh
    worker_post_flight.sh
    # Worker-helper для scope self-check (issue #2438, PR #2443, 2026-09-14):
    # воркер вызывает перед push/kanban_complete — сверяет working tree
    # (staged + unstaged + untracked) и committed diff vs origin/develop с
    # PR_ALLOWED_PREFIXES/PR_ALLOWED_GLOBS. Ловит «левые» файлы с чужих
    # worktree (PR #2443 ушёл с 4 чужими hailo/webxr файлами). Вызывается
    # автоматически из worker_post_flight.sh (opt-out SKIP_SCOPE_CHECK=true).
    worker_scope_check.sh
    validate_honesty.sh
    # Pre-PR check на ADR namespace collision (ретро 01.09 t_debcb647):
    # дополняет validate_honesty.sh (claim-evidence) функцией проверки
    # ADR-номеров. Локальный запуск воркером ДО `gh pr create` ловит
    # collision раньше, чем merge-gate отвергнет PR. Запускается
    # руками (`bash scripts/agent_flow/validate_adr_namespace.sh`),
    # не блокер CI — воркер видит actionable ошибку и сам переименовывает
    # в next-free slot (вычисляется из max(origin/develop ADR number) + 1).
    validate_adr_namespace.sh
    # Pre-PR check на молчаливый контракт test_ws (ретро 03.09 t_cfa21388):
    # G-Run Tests.yml копирует в test_ws/ только перечисленные в `for d in ...`
    # корневые каталоги. Тест, читающий корневой каталог вне списка, локально
    # зелёный, а на CI роняет ВЕСЬ батч пакета collect-error'ом. Так было с
    # docker/ (t_29b9ce36 -> PR #1874) и scripts/ (t_cfa21388, develop RED ~9ч,
    # 20+ PR). Guard сверяет список каждого job'а с реальными ссылками тестов.
    validate_test_ws_dirs.py
    # Post-PR gate на scope-drift (issue #2038, ADR-0055): блокирует push если
    # PR содержит файлы вне PR_ALLOWED_PREFIXES. Дополняет validate_branch_freshness.sh
    # (ADR-0045 freshness — commits behind): freshness ОК не спасает от scope-drift,
    # потому что base sha уже содержит «правильные» файлы — drift идёт через rebase
    # на старую ветку (ретро PR #1978/#1979/#2036: каждый раз тянули 12 файлов
    # webxr_client/* от AV-17). Воркер указывает scope в карточке; gate fail
    # → `kanban_block kind=drift-detected`, как требует acceptance в #2038.
    validate_pr_scope.sh
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
    # Push-via-gh-api wrapper (ретро 23.08 t_8abada71, t_43d5e94e,
    # t_cf3d17a0): secret policy маскирует любой реальный токен из keyring
    # → `git push` зависает на "could not read Password". Скрипт берёт токен
    # через явный GH_CONFIG_DIR (=/home/builder/.config/gh, проходит policy)
    # и подсовывает git'у через ОДНОРАЗОВЫЙ credential helper (token живёт
    # ТОЛЬКО в argv одного процесса). Идемпотентен (--dry-run по умолчанию).
    # Без раскладки в профили — воркеры devops/architect/developer тратят
    # итерации на попытки `git push` (issue #2061 t_fe8facbe).
    push-via-gh-api.sh
    # PR-create-via-gh-api wrapper (issue #2061, t_fe8facbe, t_332bdbb1):
    # `gh pr create` падает с exit 4 если ветка не запушена, а `git push`
    # падает с exit 1 из-за secret policy. После push через push-via-gh-api.sh
    # этот скрипт создаёт PR через REST POST (обходит интерактивный wizard
    # `gh pr create` и его terminal-guard --body flag). Идемпотентен:
    # если PR для head+base уже OPEN — возвращает его номер.
    gh-pr-create-via-gh-api.sh
    # Provisioning-failure watchdog (card t_c2ab8db9, retro t_34f33289):
    # upstream-loop guard — ловит карточки, попавшие в gate-by-giveup-pattern
    # (cf >= 3 за 30 мин + provider-exhausted signature + нет open PR) или
    # loop-no-progress (≥ 5 spawned + ≥ 1 gave_up за 1ч), БЛОКИРУЕТ их ДО
    # того, как dispatcher начнёт kill'ить через enforce_max_runtime SIGKILL.
    # Регистрация cron-job делается в ensure_runtime_overshoot_cron ниже
    # (every 2m — горячий цикл, потому что underlying spawn-loop может
    # съесть 1-2ч CPU/RAM менее чем за 30 мин на 5 параллельных карточках).
    agent-flow-runtime-overshoot-loop.sh
    # Cross-task archive sweeper (ADR-AF-0060 / ретро 22.08 t_d9b4c600): watchdog,
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
    # Periodic orphan-needs-e2e sweep (ретро t_78a6ffa3, 15.09.2026):
    # каждый час сканирует OPEN issues с label `needs-e2e` и закрывает
    # 3 категории сирот:
    #   (A) MERGED PR существует → close + archive card
    #   (B) пропускается, если есть OPEN PR (e2e-process разберётся)
    #   (C) нет ни OPEN ни MERGED PR за NEEDS_E2E_NO_PR_DAYS (default 7)
    #       → close reason=not_planned; НЕ трогает issues с `e2e:rejected`
    #   (D) MERGED PR есть, но last successful develop e2e run старше
    #       merge_date → relabel `needs-e2e` → `needs-e2e:recheck-develop`
    # Дополняет merge-gate.sh:archive_merged_card и e2e-process.sh
    # recovery-loop. Watchdog срабатывает даже если merge-gate пропустил
    # из-за rate-limit / transient error / mid-merge race.
    agent-flow-needs-e2e-orphan-watchdog.sh
    # Reactive conflict-sweep (ретро t_8fba04b9, issue #1977): no-agent
    # fallback на случай merge-gate silent path. Каждые 1h сканирует open
    # issues с ОБЕИМИ метками `needs-e2e` И `e2e-done` (data race: после
    # merge кто-то добавил needs-e2e обратно, ADR-0014 инвариант выполнен
    # но merge-gate не закрывает) И закрывает те, для которых найден
    # MERGED PR в develop. Также поддерживает one-shot mode
    # CONFLICT_SWEEP_ISSUE_NUM=NNN для cleanup уже разрешённого конфликта
    # где labels сняли руками. Регистрация cron-job делается в
    # ensure_conflict_sweep_cron ниже.
    agent-flow-conflict-sweep.sh
    # Daily PR-backlog digest для Шифа (PM-ретро t_cd2053b7, архитектор
    # t_d2ab84d7, devops-карточка t_f158469f). no_agent cron-job в 09:00
    # Europe/Berlin: один gh pr list запрос, группировка по mergeable +
    # mergeStateStatus (A = MERGEABLE+GREEN+e2e-done, B = MERGEABLE+GREEN
    # без e2e-done, C = CONFLICTING/DIRTY), cross-check issues со
    # stale-candidate (race case из t_d2ab84d7), один Telegram message
    # Шифу (chat_id=495039871). Dry-run через DIGEST_DRY_RUN=true.
    agent-flow-pr-backlog-digest.sh
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
    # Stale-blocked-after-prereq-merged watchdog (ретро 14.09 t_55c6c882,
    # pattern повторяет t_55ab37d4): no-agent, ежечасно сканирует все
    # kanban-доски, для blocked-карточек с PR#-references в body или
    # block-reason проверяет merged-статус каждого PR через REST
    # `gh api repos/.../pulls/N`. Если ВСЕ PR merged в develop И все
    # parent-карточки done → emit ОДИН alert-comment (idempotent через
    # marker `⚠️ stale-blocked: prerequisites merged`). НЕ auto-unblock —
    # это решение Шифу. Регистрация cron-job делается в
    # ensure_stale_blocked_watchdog_cron.
    agent-flow-stale-blocked-watchdog.sh
    # Stale-CONFLICTING PR watchdog (ретро 16.09 t_a7d642cd, pattern
    # wip-conflict-wave-after-cc-budget): no-agent, ежечасно сканирует
    # OPEN PR'ы с mergeableState='dirty' старше STALE_THRESHOLD_HOURS (4h)
    # И не имеющие активной running/todo kanban-карточки на rebase. Emit'ит
    # карточки `rebase PR #N` через kanban-retro-create.sh с idempotency-key
    # `retro:rebase-pr-<N>` (повторный тик = SKIP). НЕ rebase'ит сам,
    # НЕ merge'ит — только рекомендация, assignee=devops. Регистрация
    # cron-job делается в ensure_stale_conflicting_watchdog_cron ниже.
    agent-flow-stale-conflicting-watchdog.sh
    # E2E-rejected stale-watchdog (ретро 15.09 t_9251fd74): no-agent,
    # каждые 24ч сканирует GitHub Issues с меткой `e2e:rejected`. Для
    # issue старше STALE_DAYS (default 7) без нового PR — добавляет
    # assignee (по `agent:<role>` label или domain-keyword) и пишет
    # issue-comment "stale, нужна новая попытка или wontfix".
    # Для issue старше AUTO_CLOSE_DAYS (default 30) — auto-close +
    # label `closed:stale-rejected`. Регистрация cron-job делается в
    # ensure_e2e_rejected_watchdog_cron ниже.
    agent-flow-e2e-rejected-watchdog.sh
    # Orphan-watchdog detector (ретро 16.09 t_6687a024,
    # pattern stale-conflicting-watchdog-not-scheduled): no-agent,
    # каждые 24ч проверяет, что КАЖДЫЙ `agent-flow-*-watchdog.sh` из
    # EXPECTED[] install.sh имеет enabled interval-job в jobs.json
    # devops-профиля. Для orphan'ов emit'ит:
    #   1) строку в /tmp/agent-flow-drift.alert.log (общий канал с
    #      drift-detect), чтобы оператор увидел в утреннем обзоре;
    #   2) gh-issue (label `agent-flow-watchdog-orphan`) с перечнем
    #      пострадавших watchdog'ов — idempotent 24h dedup window;
    #   3) PR CI-guard (G-Agent-Flow-Process-Checks.yml)
    #      блокирует новые watchdog-сироты на merge-time.
    # Сам orphan-watchdog регистрируется в ensure_orphan_watchdog_cron
    # ниже (every 24h). Это страховка на случай CI-bypass / hotfix-push
    # в develop вне PR-flow (pattern повторялся уже 2 раза: t_197de62a
    # cancel-on-provider-exhausted + t_6687a024 stale-conflicting).
    # Имя файла выбрано с суффиксом -watchdog.sh ровно один раз в конце,
    # чтобы CI-guard (G-Agent-Flow-Process-Checks) корректно вывел
    # func_name=ensure_orphan_watchdog_cron по алгоритму
    # `${base#agent-flow-}` + `${slug%-watchdog.sh}` + replace -/_.
    agent-flow-orphan-watchdog.sh
    # Ночной ревью-цикл (ADR-0049): no-agent job, раз в ночь собирает
    # дайджест за прошедшие сутки (merged PR / коммиты / issues /
    # красный CI / kanban) и заводит ОДНУ карточку «ночной ревью <дата>»
    # на architect + до COMPONENT_REVIEW_MAX карточек «ревью компонента:
    # <comp>» на analyst для компонентов, которые за сутки меняли (дубли /
    # глюки LLM / недоделки). Карточки создаются через
    # kanban-retro-create.sh (дедуп по key). Регистрация cron-job —
    # в ensure_nightly_review_cron ниже.
    agent-flow-nightly-review.sh
    # Персистентность находок ночного ревью (ADR-0116):
    # НЕ cron-job — вызывается САМИМ LLM-ревьюером перед kanban_complete
    # (инструкция в теле карточки, см. agent-flow-nightly-review.sh),
    # тем же паттерном, что kanban-report-write.sh для ADR-0115.
    #
    # ВАЖНО про issue #2159 (2026-09-14, ретро t_11e6a7e7):
    # issue #2159 = «воркеры должны сохранять полные отчёты в
    # docs/reports/kanban/<task_id>.md» — это parent-issue для ADR-0115,
    # НЕ для ADR-0116. ADR-0116 ссылается на #2159 в §1.2 как нарративную
    # параллель (та же эпоха, та же боль «находки теряются»), но реальный
    # bug-триггер ADR-0116 — это дубль-карточки t_84434d4c + t_77f8ebd8
    # (см. ADR-0116 §1.2 и §2.2). Указание #2159 рядом с ADR-0116 тут —
    # историческое (PR #2177 буквально закрывал #2159 как umbrella), а не
    # bug-claim. Не путай с ownership: kanban-report-write.sh = ADR-0115
    # владеет #2159, nightly-review-record.sh = ADR-0116 свой собственный
    # fix-path.
    nightly-review-record.sh
    # Decomposed-children wake-up watchdog (ADR-AF-0052, nightly-review
    # t_bfd19ffb): no-agent job, каждые 4ч сканирует task_events.kind=
    # 'decomposed' и для детей со started_at=NULL, status ∈ {todo,triage},
    # decomposed >24ч назад — пишет ОДИН marker-коммент + priority += 1
    # (компенсирует баг «17/20 decomposed-рутов без task_links», из-за
    # которого dispatcher не поднимает детей). НЕ reassign, НЕ unblock,
    # НЕ создаёт карточки. Регистрация cron-job — в
    # ensure_decomposed_watchdog_cron ниже.
    agent-flow-decomposed-watchdog.sh
    # Orphan-cards audit telemetry (ретро t_3dbde205 / 15.09): no-agent job,
    # каждые 15м сканирует активные карточки (todo/ready/running/blocked),
    # группирует по (issue_number, repo) и для групп ≥2 → emit alert
    # (file/slack/gh_discussion) + INSERT task_events(kind='orphan_detected')
    # на КАЖДОЙ карточке-тёзке. Сам cancel делает merge-gate (G10c guard,
    # карточка t_e39afb1c). Метрика `agent_flow_orphan_cards_total`
    # экспортируется в $ORPHAN_METRICS_FILE + опционально pushgateway.
    # Идемпотентность: cooldown по (issue:repo:N) + (task_id, issue_key).
    # Регистрация cron-job делается в ensure_orphan_audit_cron ниже.
    agent-flow-orphan-audit.sh
    # Доставка repo-скиллов (.agents/skills) в профили воркеров (ретро
    # 05.09): af_skill_for_profile() маппит тип задачи (bug/feature/refactor)
    # на repo-скиллы, но без доставки профили их не видят. Вызывается из
    # install.sh best-effort после раскладки скриптов; идемпотентен.
    # Drift-detect контролирует наличие файла во всех профилях (EXPECTED).
    sync-skills.sh
    # Owner-setup helper для отдельного Telegram-канала `#devops-oncall`
    # (kanban t_d215c9e0, issue #2394 PM-default): через getUpdates
    # достаёт chat_id, пишет DEVOPS_ALERT_CHAT_ID в .env.249, опционально
    # notify-subscribe на тестовую таску. Вызывается владельцем вручную
    # ПОСЛЕ создания канала и добавления бота; НЕ cron-автомат. См.
    # docs/runbooks/devops-oncall-channel.md.
    setup-devops-oncall.sh
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
# Применение vendor-патчей к hermes-agent (ретро t_f00676f8, issue #2332).
#
# Проблема (оригинал, t_f00676f8): локальные фиксы hermes-agent
# (_profile_skill_names / _validate_skills_for_assignee в
# hermes_cli/kanban_db.py, --force-scope в hermes_cli/kanban.py,
# symlink-following подсчёт скиллов в hermes_cli/profiles.py) накладывались
# на хост руками БЕЗ сохранения в репо. При `git pull` / `pip install -U
# hermes-agent` патчи теряются, и регресс t_1ab37fa8 возвращается (карточки
# со скилами не из профиля падают).
#
# Проблема (issue #2332, 2026-09): когда upstream включает наш фикс
# в свой main (либо через ручной merge в dev-ветку как на этом хосте —
# commit 6c2be533d `feat(kanban): pre-create skill-validation + scope-hint`
# уже лежит в `z-devops/t_16a245cc-goal-mode-clean-exit-recovery`), patch
# перестаёт применяться чисто: ``git apply --check`` падает (offsets
# сдвинулись), ``git apply --reverse --check`` тоже падает (live tree
# не содержит точно тех хунков, что в patch'е — upstream их переписал).
# install.sh раньше выдавал ERROR и блокировал всю раскладку.
#
# Решение: трёхуровневый idempotent guard перед выходом в ERROR:
#   1) reverse-check → patch уже применён → OK;
#   2) SENTINEL-detect (live tree уже содержит сигнатуру фикса) → SKIP with
#      info (issue #2332): upstream включил / devops смержил руками;
#   3) forward-check + apply → чисто применяем.
# Если ничего не сработало → ERROR с подсказкой регенерировать patch.
# Только ЭТА функция валится с ERROR; остальная раскладка (TARGET_DIRS,
# drift-detect, cron registration) продолжается — patch-failure не должен
# блокировать весь install.
HERMES_AGENT_DIR="${HERMES_AGENT_DIR:-/home/builder/.hermes/hermes-agent}"

# Sentinel-detect (issue #2332): patch навывает "уже применён" если
# upstream включил наш фикс или dev-ветка содержит ручной merge.
# Sentinel — anchor-строка из канонического patch'а, который точно
# присутствует в live tree после ручного merge upstream'а.
#
# Формат: "<file-relative-to-HERMES_AGENT_DIR>|<grep -F pattern>"
# Первое совпадение → SKIP.
declare -A HERMES_AGENT_PATCH_SENTINELS=(
    # hermes-agent-skill-validation.patch:
    # upstream'овский merge commit 6c2be533d / наш фикс
    ["hermes-agent-skill-validation.patch"]="hermes_cli/kanban_db.py|def _profile_skill_names"
)

patch_already_in_live() {
    # $1 = patch basename (e.g. hermes-agent-skill-validation.patch)
    local name="$1"
    local sentinel="${HERMES_AGENT_PATCH_SENTINELS[$name]:-}"
    [ -n "$sentinel" ] || return 1
    local sentinel_file="${sentinel%|*}"
    local sentinel_grep="${sentinel#*|}"
    local full="$HERMES_AGENT_DIR/$sentinel_file"
    [ -f "$full" ] || return 1
    grep -qF -- "$sentinel_grep" "$full"
}

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
    local patch_name
    patch_name="$(basename "$patch")"
    echo "==> hermes-agent patch: $patch_name"
    # Уже применён (точная reverse-проверка)?
    if ( cd "$HERMES_AGENT_DIR" && git apply --reverse --check "$patch" >/dev/null 2>&1 ); then
        echo "  OK   patch already applied (reverse-check clean)"
        return 0
    fi
    # Уже применён через upstream / dev-merge? (sentinel-detect, issue #2332)
    if patch_already_in_live "$patch_name"; then
        echo "  SKIP patch already in live tree (sentinel-detect: upstream or dev-merge)"
        echo "         To re-apply, remove the sentinel from live tree and re-run install.sh"
        echo "         or regenerate the patch via:"
        echo "             bash scripts/agent_flow/agent-flow-regen-vendor-patch.sh $patch"
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
    echo "         See also: ретро t_f00676f8 (original) / t_49c2b63f (regen helper) / issue #2332" >&2
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
echo "==> Ensure cron job registration: cancel-on-provider-exhausted helper (ретро 15.09 t_197de62a)"
# Проблема: agent-flow-cancel-on-provider-exhausted.sh раскладывался вручную
# в ~/.hermes/scripts/legacy, НО в SOT <repo>/scripts/agent_flow/ его не было,
# и CRON-JOB НЕ регистрировался. Результат (ретро t_197de62a):
#   - 7 stale-карточек с task_runs.summary='провайдер исчерпан, ждать (402/429...)'
#     крутились в dispatcher crash-loop ready→running→crashed→ready 3+ цикла
#     подряд (watchdog-provider-quick UNBLOCK'ал их как только видел
#     providers_alive=True — а это могло быть от ЛЮБОГО живого воркера на
#     обычной задаче, не от самого провайдера);
#   - канбан-карточки блокировались только руками через
#     `bash agent-flow-cancel-on-provider-exhausted.sh --dry-run` →
#     `bash agent-flow-cancel-on-provider-exhausted.sh` (manual helper).
#
# Решение: ensure_cancel_provider_exhausted_cron() — interval-job (every 5m)
# в devops-профиле, no_agent (скрипт = watchdog). 5m — компромисс между
# свежестью (карточки не должны крутиться в crash-loop больше 5-10 мин)
# и нагрузкой (скрипт сканирует ВСЕ kanban-доски sqlite3 запросом, ~1 сек
# на доску). Дубль-guard по (script + interval + enabled).
#
# Каждый tick: сканирует ВСЕ kanban-доски, для каждой задачи в
# status IN (running, ready, todo) проверяет task_runs.summary И
# tasks.last_failure_error на provider-exhaust маркеры (HTTP 402/429,
# MiniMax 2056, "провайдер исчерпан", ...). Кандидаты: status NOT IN
# (blocked) И нет sentinel-marker'а → block kind=capability + comment
# с sentinel'ом (idempotent). Ретро t_197de62a: добавлена проверка
# last_failure_error (раньше смотрел только summary — воркеры часто
# crashed до записи summary).
#
# Регистрация переживает install.sh: каждый запуск (в т.ч. auto-fix из
# drift-detect) проверяет jobs.json и создаёт недостающий job.
ensure_cancel_provider_exhausted_cron() {
    ensure_cron_job devops "Agent Flow Cancel Provider Exhausted (ретро t_197de62a)" \
        "agent-flow-cancel-on-provider-exhausted.sh" "every 5m" interval
}
ensure_cancel_provider_exhausted_cron

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
echo "==> Ensure cron job registration: reactive conflict-sweep (ретро t_8fba04b9, issue #1977)"
# Проблема: merge-gate 5-min loop ТИХО не закрывает issues с конфликтом
# меток `needs-e2e + e2e-done` после merge (ADR-0014 инвариант выполнен
# но "Status: silent (empty output)" 50+ тиков подряд). Ручной cleanup
# (через gh issue close + gh issue edit) уже проведён для #1977, но
# pattern системный — нужна автоматизация.
#
# Решение: ensure_conflict_sweep_cron() — идемпотентная функция,
# регистрирующая interval-job (every 1h) в devops-профиле, no_agent
# (скрипт = sweep). Дубль-guard по (script + interval + enabled).
# Каждый тик сканирует open issues с ОБЕИМИ метками и закрывает те, для
# которых найден MERGED PR в develop. Дополнительно поддерживает
# one-shot mode через env CONFLICT_SWEEP_ISSUE_NUM для уже-разрешённых
# конфликтов.
#
# Регистрация переживает install.sh: каждый запуск (в т.ч. auto-fix из
# drift-detect) проверяет jobs.json и создаёт недостающий job.
ensure_conflict_sweep_cron() {
    ensure_cron_job devops "Agent Flow Conflict Sweep (ADR-0014 fallback)" "agent-flow-conflict-sweep.sh" "every 1h" interval
}
ensure_conflict_sweep_cron
echo "==> Ensure cron job registration: PR backlog digest (PM-ретро t_cd2053b7, devops t_f158469f)"
# Проблема: PM-шпаргалка `/tmp/t_cd2053b7/pr-backlog-2026-09-26.md` делалась
# вручную раз в ретро (17 OPEN PR, все MERGEABLE+GREEN, но Шифу не видел
# ежедневной сводки). Архитектор t_d2ab84d7 рекомендовал отдельный devops-job.
#
# Решение: ensure_pr_backlog_digest_cron() — идемпотентная функция,
# регистрирующая interval-job (every 24h, target 09:00 Europe/Berlin) в
# devops-профиле, no_agent (скрипт = agent-flow-pr-backlog-digest.sh).
# Сам скрипт дополнительно фильтрует окно по DIGEST_HOUR (default 9) и
# sentinel /tmp/agent-flow-pr-backlog-digest-YYYY-MM-DD.done, чтобы
# двойной cron-tick не слал 2 раза в день. Interval "every 24h" даёт
# cron-планировщику шанс запустить (09:00-09:59 local); скрипт сам решит,
# отправлять ли.
#
# Регистрация переживает install.sh: каждый запуск (в т.ч. auto-fix из
# drift-detect) проверяет jobs.json и создаёт недостающий job.
ensure_pr_backlog_digest_cron() {
    ensure_cron_job devops "Agent Flow PR Backlog Digest (Шифу daily)" "agent-flow-pr-backlog-digest.sh" "every 24h" interval
}
ensure_pr_backlog_digest_cron
echo "==> Ensure cron job registration: orphan needs-e2e sweep (ретро t_78a6ffa3)"
# Проблема: agent-flow-needs-e2e-orphan-watchdog.sh раскладывается install.sh,
# но cron-job НЕ создаётся автоматически. Без него паттерн «needs-e2e без PR»
# (11 issues на 14.09: 8 never-had-PR, 3 PR merged but orphan-stale) будет
# повторяться каждые сутки — Шифу придётся делать ручной cleanup по
# `gh issue close` для каждого нового orphan, что нарушает «не делай руками».
#
# Решение: ensure_needs_e2e_orphan_cron() — идемпотентная функция,
# регистрирующая interval-job (every 1h) в agent-flow профиле, no_agent
# (скрипт = sweep). Дубль-guard по (script + interval + enabled).
# Каждый тик сканирует OPEN issues с label `needs-e2e` и закрывает 3
# категории сирот: merged-PR → close; no-PR >= N дней → close reason=not_planned
# (НЕ трогает `e2e:rejected`); merged-PR + last-e2e-success < merge-date →
# relabel `needs-e2e:recheck-develop`.
#
# Регистрация переживает install.sh: каждый запуск (в т.ч. auto-fix из
# drift-detect) проверяет jobs.json и создаёт недостающий job.
ensure_needs_e2e_orphan_cron() {
    ensure_cron_job agent-flow "Agent Flow Needs-e2e Orphan Watchdog" "agent-flow-needs-e2e-orphan-watchdog.sh" "every 1h" interval
}
ensure_needs_e2e_orphan_cron
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
echo "==> Ensure cron job registration: stale-blocked-after-prereq-merged watchdog (ретро t_55c6c882)"
# Проблема: agent-flow-stale-blocked-watchdog.sh раскладывается install.sh,
# но cron-job НЕ создаётся автоматически. Без него stale-blocked карточки
# (pattern повторяет t_55ab37d4: prerequisites merged, но карточка остаётся
# blocked 8+ часов, потому что block_fn не имеет trigger на merged-PR'ы)
# накапливаются и обнаруживаются только руками Шифу. Ретро-карточка
# t_55c6c882 автоматизирует auto-detect → alert-comment, но без cron-job'а
# этот скрипт не запускается на проде.
#
# Решение: ensure_stale_blocked_watchdog_cron() — идемпотентная функция,
# регистрирующая interval-job (every 1h) в devops-профиле, no_agent
# (скрипт = watchdog). Дубль-guard по (script + interval + enabled).
# Каждый тик сканирует все kanban-доски, для blocked-карточек с PR#-ref'ами
# в body/block-reason проверяет merged-статус PR'ов через gh api REST, и
# если все PR merged + parents done → emit alert-comment (marker-based
# idempotency: один alert в сутки на карточку).
ensure_stale_blocked_watchdog_cron() {
    local profile_dir="/home/builder/.hermes/profiles/devops"
    local jobs_file="$profile_dir/cron/jobs.json"
    local job_name="Agent Flow Stale Blocked Watchdog (ретро t_55c6c882)"
    local job_script="agent-flow-stale-blocked-watchdog.sh"
    local job_schedule="every 1h"

    if ! command -v hermes >/dev/null 2>&1; then
        echo "  SKIP ensure-stale-blocked-watchdog-cron: hermes CLI not on PATH (nothing to register)"
        return 0
    fi
    if [ ! -f "$jobs_file" ]; then
        echo "  SKIP ensure-stale-blocked-watchdog-cron: $jobs_file not present (devops profile not set up here)"
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
ensure_stale_blocked_watchdog_cron
echo "==> Ensure cron job registration: stale-CONFLICTING PR watchdog (ретро t_a7d642cd)"
# Проблема (ретро 16.09 t_a7d642cd, wip-conflict-wave-after-cc-budget):
# После волны merge PR #2633/#2638/#2641/#2643 в develop 5+ воркерских
# `z-{agent}/` PR остаются в CONFLICTING 12-18 часов. Работник, который
# пушит, обычно не делает rebase перед push — wip-коммиты там остаются,
# PR создаётся, и никто не возвращается к rebase. merge-gate видит эти
# CONFLICTING, видит daily-report без видимого владельца, блокируется
# на ожидании.
#
# Решение: ensure_stale_conflicting_watchdog_cron() — every-1h no-agent
# job в devops-профиле. Сканирует OPEN PR с mergeableState='dirty'
# старше STALE_THRESHOLD_HOURS (4h) И без активной running/todo kanban-
# карточки на rebase → emit ОДНОЙ recommend-карточки на kanban (через
# kanban-retro-create.sh с idempotency-key `retro:rebase-pr-<N>`).
# Идемпотентность: на тике-повторе pre-check уже находит существующую
# карточку → SKIP. НЕ rebase'ит сам (assignee карточки выполняет rebase
# в worktree, у него контекст wip-коммитов).
ensure_stale_conflicting_watchdog_cron() {
    local profile_dir="/home/builder/.hermes/profiles/devops"
    local jobs_file="$profile_dir/cron/jobs.json"
    local job_name="Agent Flow Stale Conflicting Watchdog (ретро t_a7d642cd)"
    local job_script="agent-flow-stale-conflicting-watchdog.sh"
    local job_schedule="every 1h"

    if ! command -v hermes >/dev/null 2>&1; then
        echo "  SKIP ensure-stale-conflicting-watchdog-cron: hermes CLI not on PATH (nothing to register)"
        return 0
    fi
    if [ ! -f "$jobs_file" ]; then
        echo "  SKIP ensure-stale-conflicting-watchdog-cron: $jobs_file not present (devops profile not set up here)"
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
ensure_stale_conflicting_watchdog_cron

echo
echo "==> Ensure cron job registration: e2e-rejected watchdog (ретро 15.09 t_9251fd74)"
# Проблема: agent-flow-e2e-process ставит label `e2e:rejected` после
# неудачного прогона, но НЕ запускает process-cycle «rejected → новый
# fix-PR или closing-as-wontfix». Result: 6+ issue висят с этой меткой
# без assignee месяцами, юзер не видит «этот путь провалился, что дальше».
#
# Решение: ensure_e2e_rejected_watchdog_cron() — every-24h no-agent job в
# devops-профиле. Сканирует open issues с меткой `e2e:rejected`:
#   - age > 7d без нового PR → assignee + issue-comment «stale, нужна
#     новая попытка или wontfix-обоснование».
#   - age > 30d → close + label `closed:stale-rejected` (юзер может
#     переоткрыть).
# Идемпотентность: 24h-window на alert-comment; close идёт один раз по
# детекции отсутствия label. Log + exit-2 для cron-delivery.
ensure_e2e_rejected_watchdog_cron() {
    ensure_cron_job devops "Agent Flow E2E Rejected Watchdog (ретро t_9251fd74)" "agent-flow-e2e-rejected-watchdog.sh" "every 24h" interval
}
ensure_e2e_rejected_watchdog_cron

echo
echo "==> Ensure cron job registration: orphan-watchdog (ретро 16.09 t_6687a024)"
# Проблема: паттерн «PR вливает agent-flow-*-watchdog.sh в develop, но
# НЕ регистрирует cron-job в install.sh» повторялся уже 2 раза:
#   - t_197de62a — cancel-on-provider-exhausted.sh лежал orphan до ручного
#     фикса (добавили в install.sh);
#   - t_6687a024 — stale-conflicting-watchdog.sh провисел ~6ч без cron-job,
#     PR #2671 в CONFLICTING всё это время, никто не заметил.
# При этом PR CI-guard (G-Agent-Flow-Process-Checks.yml) ловит
# новые watchdog-sироты на merge-time, НО не покрывает:
#   - hotfix-push в develop вне PR-flow;
#   - случаи, когда CI отключён или bypass'нут;
#   - регрессии после merge (теоретически).
#
# Решение: ensure_orphan_watchdog_cron() — every-24h no-agent
# job в devops-профиле (страховка). Каждый tick:
#   1) читает EXPECTED[] install.sh через `bash install.sh --list-files`;
#   2) фильтрует `agent-flow-*-watchdog.sh`;
#   3) для каждого проверяет наличие enabled interval-job в jobs.json;
#   4) для orphan'ов пишет в /tmp/agent-flow-drift.alert.log + gh-issue
#      с label `agent-flow-watchdog-orphan` (idempotent 24h dedup).
# Сам orphan-watchdog регистрируется интервал-job'ом — interval-guard
# (по script+enabled), как и другие watchdog'и.
#
# Поведение по cron-доставке: exit 2 при missing → alert в
# cron-delivery; 24h-окно между повторными alert'ами.
ensure_orphan_watchdog_cron() {
    ensure_cron_job devops "Agent Flow Orphan Watchdog (ретро t_6687a024)" "agent-flow-orphan-watchdog.sh" "every 24h" interval
}
ensure_orphan_watchdog_cron

# Orphan-cards audit telemetry (ретро t_3dbde205 / 15.09): every-15m no-agent
# job в agent-flow профиле. Сканирует активные карточки канбана, группирует
# по (issue_number, repo) и для групп ≥2 → emit alert (ORPHAN_ALERT) +
# INSERT task_events(kind='orphan_detected') на КАЖДОЙ карточке-тёзке.
# Идемпотентность: cooldown-state в $ORPHAN_STATE_FILE (default 1h по issue).
# Сам cancel — в G10c guard merge-gate (карточка t_e39afb1c), этот watchdog
# только emit alert + событие для журнала.
ensure_orphan_audit_cron() {
    ensure_cron_job agent-flow "Agent Flow Orphan Audit (telemetry t_3dbde205)" "agent-flow-orphan-audit.sh" "every 15m" interval
}
ensure_orphan_audit_cron

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
echo "==> Ensure cron job registration: night voice marathon (docs/e2e/night-voice-marathon.md)"
# Проблема: 117-шаговый голосовой марафон (10 актов, ~4 часа) физически не
# влезает ни в один раунд ротации — «L: E2E Voice Test» имеет
# timeout-minutes: 45. Запускать его руками означает не запускать никогда.
#
# Решение: ensure_night_marathon_cron() — та же схема, что у ночного ревью:
# interval-job (every 1h) в devops-профиле, no_agent, а час старта зашит
# ВНУТРЬ скрипта (окно [NIGHT_MARATHON_HOUR, +WINDOW_HOURS), sentinel на
# сутки). Марафон стартует в 21:00 local и обязан закончиться до
# NIGHTLY_REVIEW_HOUR (02:00) — иначе его акты попадут в дайджест следующих
# суток, то есть через день после поломки.
#
# Развязка с ротацией — не через расписание, а через sentinel
# $HERMES_HOME/state/robot-busy (гейт G3.5 в agent-flow-e2e-process.sh):
# робот один, и параллельный e2e слушал бы чужие команды.
ensure_night_marathon_cron() {
    ensure_cron_job devops "Agent Flow Night Voice Marathon" "agent-flow-night-marathon.sh" "every 1h" interval
}
ensure_night_marathon_cron

echo
echo "==> Ensure cron job registration: runtime-overshoot-loop watchdog (t_c2ab8db9 / retro t_34f33289)"
# Карточка t_c2ab8db9: добавлен новый watchdog, который должен реагировать
# БЫСТРЕЕ чем остальные (every 2m), потому что underlying give-up-loop
# сжигает 1-2ч CPU/RAM менее чем за 30 мин на 5 параллельных карточках
# (retro t_34f33289: 120+ signal-9 SIGKILL после provider-exhausted).
# Регистрация interval-job в devops-профиле, no_agent, дубль-guard по
# (script + interval + enabled). Применяется идемпотентно.
ensure_runtime_overshoot_cron() {
    ensure_cron_job devops "Agent Flow Runtime Overshoot Loop (t_c2ab8db9)" "agent-flow-runtime-overshoot-loop.sh" "every 2m" interval
}
ensure_runtime_overshoot_cron

echo
echo "==> Ensure cron job registration: decomposed-children wake-up watchdog (ADR-AF-0052, ретро t_bfd19ffb)"
# Проблема (ADR-AF-0052 §1.1): декомпозиция эпика через kanban create оставляет
# детей со started_at=NULL, status=todo/triage, и dispatcher их не поднимает
# (17/20 последних decomposed-рутов не имеют записей в task_links — системный
# баг). Эпики висят мёртвым грузом (232ч на AV-11, 100ч на AV-27), Шифу
# узнаёт только из ретро.
#
# Решение: ensure_decomposed_watchdog_cron() — идемпотентная функция,
# регистрирующая interval-job (every 4h — компромисс между свежестью и
# нагрузкой; decomposed-алерт не hot-path) в devops-профиле, no_agent
# (скрипт = watchdog). Дубль-guard по (script + interval + enabled).
# Каждый тик сканирует все kanban-доски, для match'нутых детей пишет ОДИН
# marker-коммент в task_comments (idempotent через today_start_utc) и
# делает priority += 1 через прямой UPDATE.
ensure_decomposed_watchdog_cron() {
    ensure_cron_job devops "Agent Flow Decomposed Watchdog (ADR-AF-0052)" "agent-flow-decomposed-watchdog.sh" "every 4h" interval
}
ensure_decomposed_watchdog_cron

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
    agent-flow-decomposed-watchdog.sh
    agent-flow-stale-blocked-watchdog.sh
    agent-flow-cancel-on-provider-exhausted.sh
    # t_c2ab8db9 / ретро t_34f33289: hot-path (every 2m) watchdog для
    # upstream-loop guard (gate-by-giveup + loop-no-progress detectors).
    # md5-дrift по 6 копиям должен сразу ловиться (ставлен в этот список
    # явно, чтобы _md5_verify_fail детектил отставание host-копий).
    agent-flow-runtime-overshoot-loop.sh
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
# Ретро t_9e0760b9 (09.09.2026): install.sh 14 тиков подряд падал из-за
# устаревших vendor-патчей (hermes-agent upstream сдвинулся). Каждый patch
# failure валил set -e и весь install.sh → drift-detect cron не мог
# донести скрипты на хост (32 файла отставших). Делаем patch-приложение
# non-fatal: warning в лог, exit остаётся 0 (sync продолжается).
#
# Сводка выводится после цикла — видно сколько патчей applied/skipped/
# failed, чтобы cron-метрика ловила регрессии без false-positive падения.
_vpatches_applied=0
_vpatches_already=0
_vpatches_skipped=0
_vpatches_failed=0
for _patch in "$SCRIPT_DIR"/vendor/hermes-agent-*.patch; do
    [ -f "$_patch" ] || continue
    # .DISABLED файлы (например hermes-agent-spawn-worktree-precheck.patch.DISABLED)
    # не применяются — operator явно отключил их. Считаем как skipped.
    case "$_patch" in
        *.DISABLED)
            _vpatches_skipped=$((_vpatches_skipped + 1))
            echo "  SKIP patch disabled by operator: $(basename "$_patch")"
            continue
            ;;
    esac
    # Запускаем в subshell чтобы локальный exit не валил основной цикл.
    # set -e в основном скрипте остаётся — ошибка patch'ей логируется в
    # _vpatches_failed и не abort'ит install.sh.
    if (cd "$HERMES_AGENT_DIR" 2>/dev/null && git rev-parse --is-inside-work-tree >/dev/null 2>&1); then
        if (cd "$HERMES_AGENT_DIR" && git apply --reverse --check "$_patch" >/dev/null 2>&1); then
            _vpatches_already=$((_vpatches_already + 1))
            echo "  OK   patch already applied (reverse-check clean): $(basename "$_patch")"
        elif apply_hermes_agent_patch "$_patch" 2>/dev/null; then
            _vpatches_applied=$((_vpatches_applied + 1))
        else
            _vpatches_failed=$((_vpatches_failed + 1))
            # apply_hermes_agent_patch уже напечатал ERROR/Regenerate подсказку
            # в stderr (см. функцию выше); добавляем non-fatal маркер в stdout
            # чтобы cron-лог видел «patch не критичен, продолжили».
            echo "  WARN patch failed but install.sh continues (non-fatal): $(basename "$_patch")"
        fi
    else
        # Нет hermes-agent git checkout (например, CI runner без него) —
        # это не наша проблема, пропускаем.
        _vpatches_skipped=$((_vpatches_skipped + 1))
        echo "  SKIP hermes-agent not a git checkout: $(basename "$_patch")"
    fi
done
echo "  patch-summary: applied=$_vpatches_applied already=$_vpatches_already skipped=$_vpatches_skipped failed=$_vpatches_failed"
if [ "$_vpatches_failed" -gt 0 ]; then
    # Не валим install.sh (set -e сохраняется) — но помечаем факт в
    # alert.log чтобы drift-detect watchdog мог увидеть «X patches failed»
    # отдельным каналом, не как fatal failure всего install.sh.
    echo "  NOTE: failed patches should be regenerated via scripts/agent_flow/agent-flow-regen-vendor-patch.sh" >&2
fi
echo
echo "==> kanban MAINTENANCE probe config (retro t_1d467636)"
ensure_kanban_maintenance_probe

echo
echo "==> Sync repo skills to worker profiles (retro 05.09)"
# af_skill_for_profile() маппит тип задачи на repo-скиллы (systematic-
# debugging / test-driven-development / codebase-design / agent-flow).
# Без доставки в профили воркеров эти скиллы невидимы — sync-skills.sh
# раскладывает их hardlink-ами в skills/repo/<skill>/ (см. sync-skills.sh).
# Best-effort: сбой доставки НЕ валит install.sh (exit-код sync-skills.sh
# логируется отдельно; drift-detect тоже контролирует наличие файла).
if [ -f "$SCRIPT_DIR/sync-skills.sh" ]; then
    REPO_DIR="$REPO_DIR" HERMES_HOME="${HERMES_HOME:-${HOME}/.hermes}" \
        bash "$SCRIPT_DIR/sync-skills.sh" || echo "  WARN skills sync failed (non-fatal, see output above)"
else
    echo "  SKIP sync-skills.sh not present in $SCRIPT_DIR"
fi

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
