# Agent-flow scripts

## Source of Truth (SOT) — единственная точка правки

**`<repo>/scripts/agent_flow/`** (эта папка в репо). Если ты правишь
**здесь** + commit + merge в develop → изменения автоматически
расходятся по хосту через `install.sh`.

**НЕ править руками в:**
- `/home/builder/.hermes/scripts/`
- `/home/builder/.hermes/profiles/agent-flow/scripts/`
- `/home/builder/.hermes/profiles/architect/scripts/`

Эти 3 копии — **символические ссылки**, создаваемые `install.sh`.
Любая правка там уйдёт при следующем `install.sh` через `ln -sf`.

---

Процессные bash-скрипты, вызываемые по cron в Hermes-профилях
(`agent-flow-merge-gate`, `agent-flow-e2e-process`, `agent-flow-triage`,
`agent-flow-handoff`). Это **наша автоматизация** для issues → kanban →
agent-PR → e2e → close цикла (см. `docs/design/AGENT_FLOW_PROPOSAL.md`).

## ⚠️ КРИТИЧНО: drift между копиями (историческое)

Сейчас кроны стартуют эти скрипты по разным путям (через gateway
разных профилей, см. output `ps -eo pid,cmd | grep agent-flow`):

1. `/home/builder/.hermes/scripts/agent-flow-*.sh` — legacy, что-то стартует ещё
2. `/home/builder/.hermes/profiles/agent-flow/scripts/` — gateway agent-flow
3. `/home/builder/.hermes/profiles/architect/scripts/` — gateway architect

Чтобы избежать drift, **используй `install.sh` для раскладки hardlink-копий**:

```bash
bash <repo>/scripts/agent_flow/install.sh --dry-run   # только посмотреть
bash <repo>/scripts/agent_flow/install.sh             # реальная раскладка
```

После этого все пути (agent-flow / architect / devops profiles +
`~/.hermes/scripts`) — hardlink-копии (inode) или одинаковое содержимое.
Правка в репо видима везде после следующего `install.sh`.

**Контроль дрейфа: `agent-flow-drift-detect.sh`** (cron, every 30m).
Эталон — `origin/develop` (после `git fetch origin develop`), НЕ локальное
дерево: при local!=origin локальное дерево больше НЕ используется как
эталон (ретро 13.08 t_9a3f2e0c — слепота дрейфа host↔origin при
устаревшем local). Автофикс через `install.sh`; если не помог — сразу
создаётся kanban-карточка (create_drift_card), не ждём следующего тика.

## Скрипты

### `agent-flow-triage.sh` — no_agent=true, every 30m

Тикает каждые 30 минут. Берёт issues с лейблом `hermes`, заводит для
них kanban-карточки на доске `robbox`. Далее диспатчер `hermes gateway`
подхватывает карточки на `ready` и спавнит воркеров под нужный профиль.

### `agent-flow-merge-gate.sh` — no_agent=true, every 5m

Каждые 5 минут сканирует открытые PR с зелёным CI и label `needs-merge`,
мерджит подходящие в `develop`. **НЕ мерджит PR без human review** (Q22 —
только Шифу). Используется для clean-up очереди.

### `agent-flow-e2e-process.sh` — no_agent=true, every 60m

Главный e2e-процессор. Каждый час берёт issues с label `needs-e2e`,
мержит agent-PR в `z-{e2e}/test-round-N` (создаёт ветку если нет),
триггерит билд→деплой→e2e через `gh workflow run`, ждёт verdict
(`E2E_VERDICT PASS|FAIL` из атомарного харнесса), выставляет лейблы
`e2e-done` / `e2e:rejected` / `e2e:infra-fail`, комментит карточку.

**Содержит контракт `## e2e` блока в issue** — что воркеры должны
написать в body issue, чтобы процесс нашёл параметры теста (voice_text,
voice, scenario_file, patterns, volume и т.д.). Подробности —
`docs/design/E2E_TESTING_DESIGN_v2.md` §A.10.

**Пауза ротации при известном блокере (ретро 11.08 t_c26b73e7):**
перед созданием нового `test-round-N` процесс проверяет известные
блокеры — открытые issues, в title/body которых есть сигнатура из
`KNOWN_BLOCKER_SIGNATURES` (по умолчанию `no_wake_word` → #1117), и
робот-логи voice-assistant (best-effort, если задан `E2E_ROBOT_PASS`).
Если блокер найден — новый round НЕ создаётся, в каждый needs-e2e issue
публикуется коммент `e2e приостановлен: блокер #N` (идемпотентно), тик
завершается. Дополнительно: если у issue уже `BLOCKER_CONSECUTIVE_FAILS`
(по умолчанию 2) подряд однотипных FAIL с одной сигнатурой (маркер
`e2e-signature: <sig>` в докладах) — ставится `e2e:rejected` с указанием
блокера вместо нового round. Управляется env: `KNOWN_BLOCKER_SIGNATURES`,
`BLOCKER_ROBOT_LOG_SINCE` (default `6h`), `BLOCKER_CONSECUTIVE_FAILS`.

### `agent-flow-handoff.sh` — invoked manually / from kanban

Хелпер для хэндоффа между worker-профилями (например, devops →
architect, или backend → pr-reviewer). Используется редко, в основном
вручную.

### `kanban-retro-create.sh` — dedup-guard для ретро-карточек LLM-кронов (ретро 13.08 t_35ff29f1)

Единственная разрешённая точка создания «ретро: ...» карточек для
LLM-кронов (архитектор-надзор 5c96a6eedf93 и т.п.). Защищает от дублей:

1. **PRE-CHECK**: перед `create` читает `kanban list --json` и ищет
   НЕ-archived карточку с маркером `ретро-key: <key>` в body или точным
   нормализованным title → `SKIP <id>`, create не вызывается.
2. **IDEMPOTENCY-KEY**: create всегда идёт с
   `--idempotency-key "retro:<key>"` — повторный вызов в одном тике
   вернёт существующий id (атомарный гард от гонки).
3. **МАРКЕР**: скрипт дописывает `ретро-key: <key>` в конец body —
   следующий тик с тем же `--key` находит карточку на шаге 1.

`--key` — стабильный slug аномалии БЕЗ дат/времён (например
`e2e-stop-build-runners`); для одной аномалии — один ключ во всех тиках.
Вывод: `CREATED <id>` / `SKIP <id>` / `WOULD_CREATE` (--dry-run).

```bash
~/.hermes/scripts/kanban-retro-create.sh \
  --title "ретро: <аномалия>" --body "<факты+гипотеза+решение>" \
  --assignee <профиль> --skill <скил-из-профиля> --max-runtime 1800 \
  --key <стабильный-slug>
```

### `round_ensure.sh` — ручной валидационный e2e-раунд (ретро 11.08 t_26a6d362)

**Процессное правило:** ручные валидационные раунды devops (проверить
харнесс-фикс на живом роботе, прогнать конкретную команду) — **ТОЛЬКО
через этот скрипт** или `ROUND_ONLY=1` режим `agent-flow-e2e-process.sh`.

Скрипт берёт **тот же flock**, что и автоматическая ротация
(`/tmp/agent-flow-e2e-process.lock`): если e2e-process активен — выход с
ошибкой (`--wait N` ждёт до N секунд). Никогда не создавай round вручную
мимо него — параллельный ручной round + автоматическая ротация на одном
роботе жгут артефакты друг друга (11.08: round-49 FAIL из-за cleanup
артефактов, ложный вердикт #1077).

```bash
bash <repo>/scripts/agent_flow/round_ensure.sh            # печатает z-{e2e}/test-round-N
bash <repo>/scripts/agent_flow/round_ensure.sh --wait 300 # ждать до 5 мин
```

### `agent-flow-cleanup-249.sh` — безопасный cleanup /tmp на build-хосте (ретро 11.08 t_26a6d362)

Удаляет мусор прошлых e2e-ранов на `10.1.1.249` (`yandex_key_*`,
`build_*.log`, `dialog_e2e_*.wav`, `e2e_v2_*`, `voice_e2e_*.log`), но:

1. **не трогает файлы моложе `CLEANUP_MIN_AGE_MIN`** (default 30 мин) —
   активный e2e-прогон пишет свежие `/tmp/e2e_v2_*`;
2. **skip целиком, если e2e-process активен** (локальный flock
   `/tmp/agent-flow-e2e-process.lock` занят);
3. никогда не удаляет `e2e_voice_test.sh` (актуальный харнесс).

Дополнительно (ретро 12.08 t_d3aeaa9b): удаляет **stale round-ветки** на
remote (`z-{e2e}/test-round-N` без e2e-активности > `ROUND_STALE_HOURS`,
default **24ч**; e2e-активность = свежий коммит в ветке, e2e-process пушит
перед каждым прогоном). Guard: тот же flock e2e-process — активный round
не тронем.

**Cron (ретро 13.08 t_04d73108):** зарегистрирован в devops-профиле,
`every 6h`, no_agent=true. Регистрация идемпотентно пересоздаётся
`install.sh` (секция "Ensure cron job registration") — не потеряется при
переустановке.

```bash
bash <repo>/scripts/agent_flow/agent-flow-cleanup-249.sh --dry-run  # показать, что удалит
bash <repo>/scripts/agent_flow/agent-flow-cleanup-249.sh            # удалить (с guard'ами)
ROUND_STALE_HOURS=48 bash <repo>/scripts/agent_flow/agent-flow-cleanup-249.sh  # консервативный порог
```

### `agent-flow-deploy-sweep.sh` — авто-sweep stale deployment issues (ретро 12.08 t_d3e44336)

**Правило:** deployment issue без апдейтов > `STALE_HOURS` (default 72ч) →
авто-проверка актуальности на живых Pi (SSH) → resolved: close с
комментарием; актуально: авто-метка `hermes` (triage создаст карточку);
проверить нельзя: НЕ трогаем.

**Почему:** деплой-монитор (L-Deploy and Verify.yml) создавал issues
автоматически, но никто не верифицировал/закрывал — висели неделями
(9 штук Jul31–Aug7, ретро-триаж 12.08). Triage фильтрует только по метке
`hermes`, поэтому deployment issues не попадали в конвейер by design.

Идемпотентен: пропускает issues с меткой `hermes`/`e2e-done`/`e2e:rejected`
и свежие (< STALE_HOURS). Понимает `deploy-signature` из body issue
(`deploy-problem:env:scope:container:kind:digest`) и проверяет:
- `container_status` → `docker ps -a` (Up = resolved, restarting = actual)
- `critical_log` → `deployment_issue_dedup.py extract-log` (те же exclude-правила)
- `topic_check` → `ros2 topic list` в контейнере

```bash
bash <repo>/scripts/agent_flow/agent-flow-deploy-sweep.sh --dry-run          # показать, что сделает
STALE_HOURS=72 bash <repo>/scripts/agent_flow/agent-flow-deploy-sweep.sh     # реальный sweep
```

Рекомендуемый cron: `every 6h`, no_agent=true.

### `agent-flow-unlabeled-sweep.sh` — авто-sweep stale unlabeled issues (ретро 12.08 t_061d466e)

**Правило:** open issue БЕЗ process-меток (hermes/agent:*/needs-e2e/e2e-done/
e2e:rejected/no-e2e-required/needs-discussion) без апдейтов > `SWEEP_DAYS`
(default 2д) → эвристика по меткам/title/body определяет роль:
- voice/tts/music/audio/stt/vad → `agent:backend`
- ci/deploy/docker/build/workflow → `agent:devops`
- architecture/design/adr/refactor → `agent:architect`

Роль определена и возраст ≤ `MAX_AGE_DAYS` (default 21д) → авто-метки
`agent:<role>` + `hermes` (triage создаст kanban-карточку) + коммент.
Роль НЕ определена или issue слишком старая → только коммент-напоминание
(без `hermes` — не запускаем воркеров на потенциально неактуальные задачи).

**Build-failed issues** (метка build-failure / title «Build Failed») старше
`BUILD_FAILED_CLOSE_DAYS` (default 30д) → проверка, что L-Build Vision/Main Pi
на develop зелёные → resolved: close с комментарием; CI не зелёный → НЕ трогаем.

**Почему:** триаж фильтрует только по метке `hermes`; старые issues без неё
(#918 busy-loop 29.07, #929 OOM, #931/#933 TTS, #1016 музыка и др.) висели
неделями неразмеченными (ретро 12.08 t_061d466e).

Идемпотентен: пропускает process-issues, свежие (< SWEEP_DAYS), уже
размеченные; комментарии дедуплицируются (24h).

```bash
bash <repo>/scripts/agent_flow/agent-flow-unlabeled-sweep.sh --dry-run   # показать, что сделает
SWEEP_DAYS=2 bash <repo>/scripts/agent_flow/agent-flow-unlabeled-sweep.sh # реальный sweep
```

Рекомендуемый cron: `every 12h`, no_agent=true.

### `cron-loop.sh` — низкоуровневый цикл

Тонкая обёртка над cron-вызовами (используется как fallback когда
Hermes-cron недоступен). Маленький, 854 байт.

### `watchdog.sh` — мониторинг процессов

Сторожевой таймер для долгоиграющих процессов (e2e-build, deploy).
Запускается параллельно, проверяет живость по pid-файлу и heartbeat.

## Связь с cron-jobs

Скрипты регистрируются как `cronjob` через `hermes cron run --script
... --schedule "every Nm" --no-agent`. Управлять ими:

```bash
hermes cron list                  # какие кроны запущены
hermes cron pause <job_id>       # временно отключить
hermes cron resume <job_id>      # обратно включить
```

Для патча скрипта — см. **MAINTENANCE-процедуру**: pause → patch →
resume (порядок важен, см. skill `synthesis-tts-chain-debugging` §
«CRITICAL: pause → patch → resume для crons»).

## MAINTENANCE-flag

Скрипт `agent-flow-e2e-process.sh` первой строкой проверяет
наличие `MAINTENANCE`-файла в `origin/develop`. Если есть — тик
skip (exit 0). Включается через:

```bash
cd /home/builder/hermes-share/rob_box_project
git checkout develop
touch MAINTENANCE
git commit -m "maintenance: pause agent-flow crons"
git push origin develop
```

Снимается аналогично (`git rm MAINTENANCE`). Подробно — skill
`synthesis-tts-chain-debugging` § «MAINTENANCE-флаг».
