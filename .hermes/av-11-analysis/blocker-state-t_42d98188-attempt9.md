# AV-11 Status — t_42d98188 attempt-9 (NEW finding beyond attempt-8)

**Дата:** 2026-08-28 ~12:50 UTC (fresh respawn, after night-padawan_tick respawn-guard respawn)
**Воркер:** backend
**Ветка:** `z-{agent}/1605-av-11-e2e-avatar-mixed-mode-quest-teleop` (HEAD=71b679d7)
**Карточка:** t_42d98188 — AV-11 e2e mixed-mode
**PR:** #1705 уже MERGED в `feature/avatar`

## TL;DR — что НОВОГО нашёл в этой попытке

**attempt-8 был неполный.** Диагноз attempt-8: «deployed supervisor Phase 1 monitor-only + needs rebuild после Phase 2 merge». Это НЕПРАВДА.

**Настоящий блокер (этот attempt-9):**

1. **Source-код `supervisor_node.py` на `feature/avatar` сам по себе Phase 1 monitor-only.** md5 deployed == md5 source (`a38dc341624e94a93fe4edba772cfb88`). Перебилд без изменения кода = то же самое поведение.

2. **`core/fsm.py`, `core/locks.py`, `core/dead_man.py`, `core/state.py` существуют и покрыты 86 unit-тестами**, но **НИ ОДИН из них не импортируется в `supervisor_node.py`**. Сервисные хэндлеры `_on_acquire_floor`/`_on_release_floor`/`_on_set_avatar_mode` всё ещё Phase 1 hardcoded `_fill_monitor_response()`.

3. **`supervisor_node.py:11-14`** явно говорит: «Phase 2 (active-режим) появится в отдельных карточках». Это был план изначально — отдельная работа.

4. **Issue #1706 (mной открытый в attempt-8) — INCORRECT.** Rebuild без code-merge не меняет поведение. Это было моей ошибкой. Нужна **новая карточка** на Phase 2 integration или явно поручить это существующей карточке.

5. **В дереве issues НЕТ карточки** «Phase 2: integrate ModeManager+LockManager into supervisor_node.py service handlers». Поискал по PR-ах, по git log origin/feature/avatar последние 5 дней — нет. Только Phase 1 monitor-only коммиты + bugfix-ы (#1644 RcutilsLogger-compat, #1645).

## Детальное доказательство

### 1. Network pre-flight (ЗЕЛЁНЫЙ, как и в attempt-8)

```
$ ping -c 2 -W 2 10.1.1.21
64 bytes from 10.1.1.21: icmp_seq=2 ttl=64 time=1.57 ms
$ sshpass -p open ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 hostname
VisionPi
```

### 2. Containers (те же 20 часов Up; supervisor образ НЕ пересобирался)

```
NAMES                 STATUS                IMAGE                                                       CREATED AT
avatar-supervisor     Up 20 hours (healthy) 10.1.1.249:5000/krikz/rob_box:supervisor-humble-test        2026-08-27 19:38:57 +0300 MSK
rob-box-quest         Up 20 hours (healthy) 10.1.1.249:5000/krikz/rob_box:quest-humble-test             2026-08-27 19:38:57 +0300 MSK
telegram-bot          Up 20 hours (healthy) 10.1.1.249:5000/krikz/rob_box:telegram-bot-humble-test      2026-08-27 19:38:57 +0300 MSK
voice-assistant       Up 20 hours (healthy) 10.1.1.249:5000/krikz/rob_box:voice-assistant-humble-test   2026-08-27 19:38:58 +0300 MSK
```

`docker history` подтверждает ENV `AVATAR_SUPERVISOR_MODE=monitor` (Phase 1).

### 3. Главное открытие: md5 deployed == md5 source

```
$ md5sum /home/builder/rob_box_project/.worktrees/t_42d98188/src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py
a38dc341624e94a93fe4edba772cfb88
$ ssh ... 'docker exec avatar-supervisor bash -lc "md5sum /ws/src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py"'
a38dc341624e94a93fe4edba772cfb88  /ws/src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py
$ ssh ... 'md5sum /ws/install/rob_box_supervisor/lib/python3.10/site-packages/rob_box_supervisor/supervisor_node.py'
a38dc341624e94a93fe4edba772cfb88
```

**Source = Deployed = 349 строк, Phase 1 monitor-only.**

### 4. Source `supervisor_node.py` — НОЛЬ интеграции FSM/LockManager

```
$ grep -nE "fsm|ModeManager|LockManager|core\." src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py
(empty — никаких упоминаний)
```

В отличие от:

```
$ grep -rn "ModeManager\|LockManager" src/rob_box_supervisor/rob_box_supervisor/
src/rob_box_supervisor/rob_box_supervisor/__init__.py:4:The actual FSM, LockManager, dispatcher and aggregator land in later cards
src/rob_box_supervisor/rob_box_supervisor/core/__init__.py:6:(:class:`ModeManager`, :class:`Mode`, :class:`FSMConflictError`).
src/rob_box_supervisor/rob_box_supervisor/core/__init__.py:24:    ModeManager,
src/rob_box_supervisor/rob_box_supervisor/core/__init__.py:32:    "ModeManager",
src/rob_box_supervisor/rob_box_supervisor/core/fsm.py:1:"""ModeManager — FSM режимов аватара (AV-3, ADR-0028 §4.1)...
src/rob_box_supervisor/rob_box_supervisor/core/locks.py:89:# === LockManager =====================================================
src/rob_box_supervisor/rob_box_supervisor/core/locks.py:92:class LockManager:
```

**ModeManager и LockManager экспортируются из `core/__init__.py`, но `supervisor_node.py` их не использует.**

### 5. Service handlers в source — Phase 1 hardcoded

```
def _on_acquire_floor(self, _request: Any, response: Any) -> Any:
    """``AcquireFloor`` — Phase 1 monitor: лог + monitor response."""
    self._log.info(f"AcquireFloor received (mode={self._mode}) — phase 1 monitor")
    return self._fill_monitor_response(response)

def _on_set_avatar_mode(self, _request: Any, response: Any) -> Any:
    """``SetAvatarMode`` — Phase 1 monitor (NOT_IMPLEMENTED для active)."""
    if self._mode != "monitor":
        # Phase 2: реальный FSM. Пока — refuse и остаёмся в monitor.
        self._log.warning(
            f"SetAvatarMode: mode={self._mode} запрошен, но Phase 2 не реализован "
            f"(AV-6 = monitor-only). Отвечаю success=true/applied=false/reason={MONITOR_MODE_REASON}"
        )
```

**Комментарий прямо в коде: «Phase 2: реальный FSM. Пока — refuse и остаёмся в monitor.»**

### 6. /acquire_floor в deployed supervisor (поведение)

```
$ docker exec -u root avatar-supervisor bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 service call /acquire_floor std_srvs/srv/Trigger"
response:
std_srvs.srv.Trigger_Response(success=True, message='{"applied": false, "reason": "supervisor_in_monitor_mode"}')
```

### 7. Issue/PR дерево по AV-N (проверка отсутствующей карточки)

| AV-N | Карточка | PR | Статус | Содержит интеграцию? |
|------|----------|-----|--------|----------------------|
| AV-2 | #1596 | #1608 | MERGED | package skeleton — нет |
| AV-3 | #1597 | #1632 | MERGED | FSM ModeManager logic — **да, но как separate pure-Python класс** |
| AV-4 | #1610 | #1610 | MERGED | LockManager logic — **да, но как separate pure-Python класс** |
| AV-5 | #1599 | #1622 | MERGED | /avatar/state msgpack — **НЕТ, реализован как-отдельная схема, не используется в supervisor_node.py** |
| AV-6 | #1600 | #1623 | MERGED | supervisor_node.py — **MONITOR-ONLY hardcoded** |
| AV-7 | done? | done? | merged | `voice_input_mode` parameter в dialogue_node — done |
| AV-8 | #1602 | #1607 | MERGED | docs-only (frame types 0x30-0x33) |
| AV-9 | done | #1633 | MERGED | docker-compose supervisor — **Phase 1 monitor-only** |
| AV-10 | done | #1621 | MERGED | Telegram client API refactor — done |
| **AV-11** | **#1605** | **#1700 closed, #1705 merged raw-evidence** | **OPEN, blocked** | **live e2e FAIL — no Phase 2 integration exists** |
| **AV-X (NEW)** | **нет** | **нет** | **—** | **Phase 2: wire up ModeManager+LockManager into supervisor_node.py service handlers** |

**Вывод:** разложение эпика Avatar из `docs/plans/2026-08-24-avatar-decomposition.md` §4 **не включало карточку на Phase 2 integration**. Это технический долг / пропущенный шаг в декомпозиции. Без него acceptance AV-11 физически недостижим.

## Что я НЕ могу сделать в этой backend-сессии

1. **Написать Phase 2 integration код** (~½-1 день, новый TDD для service handlers, изменение supervisor_node.py, изменение IDL сервисов — текущие std_srvs/Trigger без полей client_id/floor, нужен новый .srv по AV-5/AV-8 IDL). Это **новая worker-карточка по объёму**. Карточка AV-11 явно говорит «Без нового кода». Писать новый код = нарушать контракт этой карточки.

2. **Сделать docker push в registry 10.1.1.249:5000** — нет прав.

3. **Запустить `L: Build Vision Pi Services` + `L: Deploy and Verify`** — нет прав (только Шифу/devops).

4. **Подменить deployed supervisor вручную через `docker compose`** — даже если собрать локально Phase 2 node, без реестра Vision Pi его не развернёт (compose вытягивает `image: ${SERVICE_IMAGE_PREFIX:-ghcr.io/krikz/rob_box}:supervisor-${ROS_DISTRO}-${SUPERVISOR_TAG}`).

5. **Закрыть issue #1706 как resolved rebuild** — это будет ЛОЖЬ, потому что rebuild без code-merge не меняет behavior. После пересборки `/acquire_floor` всё ещё вернёт `supervisor_in_monitor_mode`.

## Что НУЖНО сделать (кому-то с правами + скоупом)

**ВАРИАНТ A (быстрый):** Backend-агент создаёт **новую карточку** `[AV-X] supervisor_node.py: wire up ModeManager+LockManager into service handlers (Phase 2 active mode)` от `feature/avatar`, объём ~1 день, TDD, или это идёт как расширение существующей AV-7/AV-8/AV-10 (что скорее невозможно, т.к. они все MERGED).

**ВАРИАНТ B (правильный, по ADR-0028 §4.5):** Шифу/architect принимает решение — пропустили ли мы Phase 2 integration при декомпозиции. Если пропустили — архитектор создаёт issue через `scripts/agent_flow/create_avatar_issues.sh` с правильным scope. Backend пишет код + TDD. Devops делает rebuild + deploy. Затем e2e-агент запускает AV-11 acceptance.

**ВАРИАНТ C (честная редукция scope AV-11):** Шифу решает что в текущих реалиях AV-11 живого e2e не достичь — переформулировать acceptance как «Phase 2 integration covered by N unit-тестов изолированно + bash harness готов для self-hosted runner + raw-evidence что deployed supervisor показывает monitor-mode reason». Закрыть AV-11 с этим acknowledgment.

**Любой из вариантов требует решения Шифу.** Эта backend-сессия может только сигнализировать блокер.

## Acceptance AV-11 — статус честно (через raw-evidence)

| # | Критерий | Результат | Где видно |
|---|----------|-----------|-----------|
| 1 | supervisor+quest+telegram запущены | OK | raw `docker ps` |
| 2 | WebXR + PIN auth | OK | Quest /healthz=200 |
| 3 | grip + twist linear=0.3 → движение | **FAIL** — `/acquire_floor` → `applied=false, reason="supervisor_in_monitor_mode"` | raw service-call |
| 4 | Telegram /say → TTS | **FAIL** — voice_floor acquisition тоже через `_fill_monitor_response` | raw `_on_set_voice_mode` |
| 5 | mode=mixed в логах | **FAIL** — /avatar/state содержит только Phase 1 monitor fields, никаких floors{} | raw /avatar/state decoder |
| 6 | Wi-Fi fail safe-stop ≤500ms | **FAIL** — нет LockManager в коде | source: `grep core.` returns empty |
| 7 | /forward → telegram_active | **FAIL** — same Phase 1 monitor | same as #3 |

**Все 7 шагов — FAIL по одной причине: supervisor integration Phase 2 не существует в исходниках.**

## Что сделано в этой сессии (raw, не догадки)

1. ✅ Ping Vision Pi, SSH, hostname, docker ps → все ЗЕЛЁНЫЕ.
2. ✅ md5 deployed vs source `supervisor_node.py` → ИДЕНТИЧНЫ (a38dc341624e94a93fe4edba772cfb88).
3. ✅ `grep ModeManager\|LockManager src/.../supervisor_node.py` → пусто.
4. ✅ `git log origin/feature/avatar --since="5 days ago" -- src/rob_box_supervisor/` → только Phase 1 monitor коммиты.
5. ✅ `gh search prs` + `git log --all` → нет in-progress PR на Phase 2 integration.
6. ✅ Issue #1706 → подтверждено incorrect (rebuild только не решает, нужен ещё code PR).
7. ✅ Failure-mode hypothesis: после моих открытий issue #1706 правильное решение — обновить его или создать новую карточку `[AV-X]`.
