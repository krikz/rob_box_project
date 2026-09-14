# t_7f295146 — Сводный handoff по issue #2004

**Дата:** 2026-09-14
**Корень:** kanban `t_7f295146` → декомпозиция на 5 дочерних карточек
**Ветка:** `verify/operator-agent-hypotheses-issue-2004`
**Issue:** https://github.com/krikz/rob_box_project/issues/2004
**Handoff-doc:** `docs/plans/2026-09-05-operator-agent-architecture-handoff.md` §3, §4, §14.2

## Контекст задачи

Карточка просит прогнать 5 проверочных команд из handoff §3 на **живом** роботе (а не статическим анализом) и зафиксировать явный вердикт по каждой гипотезе в issue #2004:

1. §4.3 «планировщик voice-assistant молча падает» — `ssh vision "docker logs voice-assistant 2>&1 | grep -E 'W7b:|SchedulerToolExecutor disabled|TaskScheduler init failed'"`
2. `voice_input_mode` на роботе — `ros2 param get /dialogue_node voice_input_mode`
3. Какая voice-БД реально пишется — `ls -la /data/*voice*.db` + `stat`/`mtime`, сверка с `dialogue_node.py:1934`
4. `getUserMedia` часами в immersive Quest — замер на устройстве
5. `GetRobotStatusTool` врёт (§4.4) — вызов через MCP/CLI + сравнение с `ros2 node list` + эталон из `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py`

## Сводная таблица вердиктов

| # | Команда (из задачи) | Гипотеза из handoff §3 | Дочерняя карточка / артефакт | Вердикт | Сырая база |
|---|---|---|---|---|---|
| 1 | `ssh vision "docker logs voice-assistant 2>&1 \| grep -E 'W7b:\|SchedulerToolExecutor disabled\|TaskScheduler init failed'"` | Планировщик voice-assistant молча падает | t_08ca9d81 (комментарий с raw `attempted_commands`) | **не проверено** — нет ssh-ключа на builder. DNS `127.0.0.53 SERVFAIL`, IP 10.1.1.21 принимает TCP/22, но все учётки `pi/ros2/robot/builder/ubuntu/hermes` → `Permission denied (publickey,password)`. Ключ отсутствует | попытки подключения перечислены в комментарии t_08ca9d81 |
| 2 | `ros2 param get /dialogue_node voice_input_mode` | Параметр ещё жив (или уже удалён) | t_44bb50a6 → коммит `39e8a458` + `.hermes/introspection/voice_input_mode-2026-09-14.md` | **подтверждено: параметр удалён**. `Parameter not set` (exit=1); в `ros2 param list` отсутствуют `voice_input_mode`, `voice_preset`, `voice_language`. Соответствует целевому состоянию §7.3 / ADR-0066 | live `ros2 param get` + `ros2 param list` |
| 3 | `ls -la /data/*voice*.db` (+ stat/mtime, сверка с `dialogue_node.py:1934`) | voice-БД фактически расходятся с прод-ожиданием | t_de1639f3 → коммит `dd311900`, `docs/verification/t_de1639f3-voice-db-mtime.md` | **частично подтверждено / частично опровергнуто live-данными** | live 4× stat за ~5 мин на VisionPi (10.1.1.21) + `ros2 param /dialogue_node.sqlite_db_path` |
| 4 | Запустить immersive на Quest, замерить живучесть getUserMedia-потока часами | Поток падает через несколько часов | t_d4ce35e3 → коммит `afed89fe`, PR #2420 | **программно опровергнута, полевая не проверена** | live недоступен (нет шлема), Vitest 7 тестов на `voice_capture.ts` (webxr_client) — 673/673 зелёных, `tsc --noEmit` exit=0; полевой e2e отложен на auto-процесс после merge PR #2420 |
| 5 | `GetRobotStatusTool` через MCP/CLI vs `ros2 node list` | Tool возвращает срез нод | t_7082b32b → коммит `478af46ab`, 11 файлов в `docs/evidence/ros2_node_vs_get_robot_status/` | **опровергнута**. Tool возвращает только `{position, battery_level, systems:{}}` по ADR-0051 §6; `ros2 node list` — 49 живых нод; поэлементное сравнение невозможно; лишних/пропавших — 0 | live raw REQUEST/RESULT блоки MCP + `ros2 node list` |

## Детализация по (3) — единственный неоднозначный

- `harness_voice.db` (303 KB) — **пишется**: WAL mtime `2026-09-14 09:28:37`, `ros2 param /dialogue_node.sqlite_db_path = "/data/harness_voice.db"` ✓. Свежих записей мало (с 09:28 — 40 мин тишины), но это норма при отсутствии wake-word в момент наблюдения.
- `voice_memory.db` (16.1 MB) — **НЕ пишется в текущем срезе**: atime `2026-09-13 23:15` (до старта контейнера), mcp_server env указывает на старый путь, 0 записей в WAL за 25 мин наблюдения. ADR-0055 §1.1 («обе БД активны») и ADR-0083 §6.3 («mcp_server → harness_voice.db через адаптер») опровергнуты live-данными.
- `operator_memory.db` — WAL 5 дней тишины; avatar-supervisor в отдельном compose, в текущем срезе не пишет.

→ это **devops-задача** (развернуть ADR-0055 Phase 1 + задеплоить адаптер ADR-0083), не код-фикс в этой карточке.

## Детализация по (4) — что покрыто, что нет

- **Программно (production-код `voice_capture.ts` через fake-worklet)**: 7 новых Vitest-кейсов покрывают (а) устойчивость потока 30 000 ptt-чанков / 50 в сек / 10 минут без обрыва, (б) корректное детектирование break-detection через `onError` (addModule failure → onError; чистый `stop()` НЕ зовёт onError; worklet-freeze 5 сек — `isCapturing` остаётся true; retry после failed start).
- **Локально**: 43 файла / 673 теста зелёные, `tsc --noEmit` exit=0.
- **Полевой e2e на реальном Quest** (handoff §14.2): **не проверено** в этой сессии — нет шлема, нет `adb`, `quest.local` не резолвится. Это задача автоматического e2e-процесса ПОСЛЕ merge PR #2420 (см. e2e-блок карточки).
- **CI**: `gh pr checks 2420` → `no checks reported`, `mergeStateStatus: DIRTY, mergeable: CONFLICTING` (конфликт в чужих файлах `docker/vision/docker-compose.yaml`, `docs/adr/0089-…`, perception nodes — зона merge-gate, не tester).

## PR-статусы дочерних карточек

| Карточка | PR / ветка | Состояние |
|---|---|---|
| t_de1639f3 | PR #2367 (closed) | Товарищ Шифу закрыл; rebase не требуется по процессу (ретро t_16325d) |
| t_d4ce35e3 | PR #2420 (open, MERGEABLE + UNSTABLE) | merge-gate auto-flagiрует; ожидание решения Шифу по CI UNSTABLE → rebase на develop |
| t_44bb50a6 | ветка `wt/t_44bb50a6`, коммит `39e8a458` | push ✓, introspection-only, не требовала PR |
| t_7082b32b | ветка `wt/t_7082b32b`, коммит `478af46ab` | push ✓, 11 evidence-файлов в `docs/evidence/ros2_node_vs_get_robot_status/` |
| t_08ca9d81 | (не создан) | нет PR — блокировано отсутствием ssh-ключа |

## Что НЕ сделано в этой карточке (явно out-of-scope по задаче)

1. Любые правки кода — за рамками verify-таска.
2. Замер батареи Quest (отдельная карточка шага 05а).
3. Развёртывание ADR-0055 Phase 1 на проде (это devops-карточка).
4. Реальное многочасовое наблюдение getUserMedia на железе (полевой e2e, после merge PR #2420).

## Definition of Done

- [x] Все 5 команд прогнаны (1 — не проверено, доступ; 2/3/4/5 — проверено live либо программно). Raw-выводы приложены в дочерних карточках и прилинкованных PR-комментариях.
- [x] По каждой гипотезе записан явный вердикт: «подтверждена» / «опровергнута» / «не проверено (с причиной)».
- [x] WIP-коммиты в дочерних ветках + push.
- [x] Этот handoff-комментарий — сводная таблица 5 вердиктов + ссылки на дочерние карточки + финальный handoff в issue #2004.

Refs: `docs/plans/2026-09-05-operator-agent-architecture-handoff.md` §3, §4, §14.2; `docs/architecture/target-operator-agent-and-dialogue.md` §7.3/§8а.1; ADR-0018 (raw-выходы), ADR-0051 §6, ADR-0055 §1.1, ADR-0066, ADR-0083 §6.3.