# ADR-0081 — teleop_floor ↔ twist_mux: единое решение о блокировке

**Status:** accepted (2026-09-09, Шифу: GOODWORKRINKZ в PR #2200 + krikz в issue #2191)
**Amends:** ADR-0028 §4.2 (LockManager) + ADR-0028 §7 «Открытый вопрос 1»
**Scope:** `twist_mux` input gating + arbiter/Quest bridge
**Supersedes:** ADR-0080 (draft, 2026-09-08) — перенумерован из-за коллизии
с `docs/adr/0080-voice-and-headset-control-eight-seams.md` (PR #2221, merged).

> Документ фиксирует архитектурное решение и замер на роботе, **реализация
> — отдельной карточкой `t_9d33e8ca` (backend)**: `/teleop_lock` в
> twist_mux.yaml с priority=90, sticky (`timeout: 0.0`).
> Решения Q1–Q4 зафиксированы Шифу 2026-09-09 (PR #2200, комментарий
> «Решения владельца по Q1–Q4»).

## 1. Контекст

Арбитраж движения сейчас существует в трёх местах:

1. **in-process mutex `LocalAvatarArbiterClient`**
   (`src/rob_box_quest/rob_box_quest/core/avatar_arbiter.py`) — временный
   stub для Quest-WS внутри одного процесса.
2. **`LockManager`** в `avatar_arbiter`
   (`src/rob_box_supervisor/rob_box_supervisor/core/locks.py`) —
   единственный владелец `teleop_floor`/`voice_floor` (ADR-0028 §4.2,
   AV-12 IDL).
3. **приоритеты `twist_mux`**
   (`docker/main/config/twist_mux/twist_mux.yaml`) — emergency 255 /
   joystick 100 / web_ui 50 / quest 40 / voice 25 / nav2 10 + один lock
   `joystick_lock` (`timeout: 0.0`, sticky).

**Дыра (ADR-0028 §7 открытый вопрос 1):** держатель `teleop_floor`
(LockManager) никак не пробрасывается в twist_mux. Пока quest-оператор
держит floor:

- `cmd_vel_voice` (priority 25) и `cmd_vel` от nav2 (10) **всё равно
  проходят** в мультиплексор, если quest молчит дольше своего timeout
  (0.5 с) — а он молчит каждый раз, когда стик в нейтрали.
- Даже если twist_mux по timeout отбрасывает quest, ничего не блокирует
  web_ui / voice / nav2 от **параллельной** подачи команд в эти 0.5 с.
- У `joystick_control_node` аналогичной дыры нет — он публикует
  `/joystick_lock` пока ARMED
  (`src/rob_box_teleop/rob_box_teleop/joystick_control_node.py:115`),
  и twist_mux с `timeout: 0.0` блокирует нижестоящие источники sticky.

## 2. Решения (приняты Шифу 2026-09-09)

### Q1 — Один lock-topic: `/teleop_lock`, priority **90** (не 100)

**Решение:** Один общий `/teleop_lock` (Bool), приоритет **90** —
на 10 ниже аппаратуры.

Лестница приоритетов после принятия решения:

| источник | было | стало |
|---|---|---|
| `cmd_vel_emergency` | 255 | 255 |
| `cmd_vel_joy` (аппаратура) | 100 | 100 |
| **`cmd_vel_quest`** | **40** | **90** |
| `cmd_vel_web` (Telegram) | 50 | 50 |
| `cmd_vel_voice` (личность) | 25 | 25 |
| `cmd_vel` (nav2) | 10 | 10 |

**Почему lock=90, а не 100:** в `twist_mux` lock глушит источники со
**строго меньшим** приоритетом — это видно по текущему конфигу:
`joystick_lock` стоит на 100 и блокирует web(50)/voice(25)/nav2(10),
при этом сам `cmd_vel_joy` на 100 продолжает работать (источник и lock
**равны** по priority → блокировка не действует на источник с равным
приоритетом). Значит lock на 100 заглушил бы и quest(90) — то есть
самого держателя floor. Lock на 90 глушит web_ui(50)/voice(25)/
nav2(10) и оставляет живыми quest(90) и аппаратуру(100). Это ровно то,
что нужно.

⚠️ **Побочный эффект:** раньше quest(40) был **ниже** Telegram(50),
теперь выше. Telegram-оператор больше не перебивает quest-оператора
по приоритету. Это сознательная перестановка — VR-оператор «у меня руль»
так же, как аппаратура, и не должен уступать текстовому kill-switch-у.

**Резерв на замер (см. §5):** поведение twist_mux может быть не
`strict <`, а `<=` — тогда lock на 90 заглушит и сам quest. Это надо
проверить на роботе перед merge реализации (`t_9d33e8ca`). Если
`<=` — опустить lock до 89 или поднять quest до 91.

### Q2 — Watchdog **не нужен**, чистый sticky (`timeout: 0.0`)

**Решение:** топик `/teleop_lock_watchdog` и его ветка реализации
**удаляются**. Достаточно одного sticky `/teleop_lock` с
`timeout: 0.0`, priority 90.

**Обоснование (дословно Шифу):** «арбитр у нас только для квест-телеопа
и голосового управления из личности, у нас остаётся канал радио ещё,
так что при падении нам всё равно».

Аппаратура на 100 выше lock-а на 90 → залипший lock её не глушит →
при падении арбитра робот остаётся управляемым с пульта. Автоматика
разблокировки не нужна.

Это заодно снимает дефект, найденный в ранней редакции: watchdog
публиковался ровно один раз за жизнь ноды из-за de-dup — чинить нечего,
код уходит целиком.

### Q3 — Блокировать ли voice (`cmd_vel_voice`) при teleop_floor — закрыт через Q1

**Вопрос был:** голосовой команде «едь вперёд» (приоритет 25) — должно
ли она блокироваться, когда оператор держит teleop_floor?

**Суть проблемы:** приоритеты решают, кто победит, когда **оба**
источника публикуют. Но оператор шлёт `teleop_twist` не непрерывно —
отпустил стик, и в течение `timeout: 0.5` его канал считается молчащим.
В эту паузу человек рядом с роботом говорит «робот, езжай вперёд»,
личность дёргает инструмент движения — и робот едет по команде
постороннего, пока оператор в шлеме.

**Ответ следует из Q1 автоматически:** lock на 90 глушит voice(25).
Отдельный механизм не нужен.

⚠️ **Не путать с `voice_floor`:** голосовой канал (`/voice/in`,
`/voice/out`, TTS) НЕ блокируется — личность продолжает отвечать на
wake-word. Это отдельный домен (см. Q4 ниже и ADR-0027 §3.4).

### Q4 — `voice_floor` в twist_mux **не пробрасывается**

**Решение:** `voice_floor` — про голосовой поток, а не про `cmd_vel_*`.
Без изменений. Блокировка голоса решается на уровне dialogue_node
(выбор источника микрофона) и supervisor (FSM режимов), не на уровне
twist_mux. `/voice_lock` не вводим.

## 3. Итоговое решение (R1–R5)

| # | Решение | Обоснование |
|---|---|---|
| R1 | `/teleop_lock` (Bool) — новый twist_mux lock, priority=**90**, timeout=0.0 (sticky) | Зеркалит `/joystick_lock` (priority 100, sticky), но не глушит самого держателя (quest 90). |
| R2 | `/teleop_lock_watchdog` — **отменён**, не вводим | Аппаратура(100) выше lock-а(90) → при падении арбитра робот управляем с пульта. Автоматика разблокировки не нужна (решение Шифу). |
| R3 | Публикация `/teleop_lock` — **avatar_arbiter** (единственный владелец LockManager), не QuestBridge и не LocalAvatarArbiterClient | Один источник истины (ADR-0028 §4.2). |
| R4 | Блокируются **web_ui(50), voice(25), navigation(10)** — т.е. всё строго ниже priority=90. Не блокируется joystick(100), emergency(255), quest(90). | Семантика «у меня руль» = то же, что `/joystick_lock`, но не глушит держателя. |
| R5 | `voice_floor` НЕ пробрасывается в twist_mux | voice_floor — про голос, а twist_mux — про `cmd_vel_*`. Разделяем домены явно (Q4). |
| R6 | Приоритет `cmd_vel_quest` повышен 40 → **90** | VR-оператор наравне с аппаратурой (решение Шифу). Побочный эффект: Telegram(50) больше не перебивает quest(90). |

## 4. Альтернативы (рассмотренные и отклонённые)

- **Lock priority=100 (= joystick).** Отклонено (Шифу): заглушил бы
  quest(90), то есть самого держателя floor.
- **Lock priority=89 или quest=91.** Отступление на 1 — компромисс для
  случая, если замер на роботе покажет `<=` вместо `<` в twist_mux
  (см. §5). Держим как fallback.
- **Гибрид `/teleop_lock` + `/teleop_lock_watchdog` (C из draft v0).**
  Отклонено (Шифу): watchdog не нужен, аппаратура(100) выше lock-а.
- **Полностью убрать twist_mux и гонять `cmd_vel_*` через avatar_arbiter.**
  Отклонено: ADR-0028 §S7 явно требует, чтобы супервизор НЕ
  публиковал `cmd_vel_*` напрямую — только маршрутизация. Перенос
  маршрутизации в арбитр — это уже другая фаза (Phase 3), не вопрос
  #2191.
- **Неблокирующий арбитраж (только приоритеты twist_mux).** Отклонено:
  именно из-за дыры открыт issue #2191 — «оператор держит floor молча,
  а cmd_vel_voice всё равно проходит».
- **Один `/floor_lock` на оба floor-а.** Отклонено: разные домены
  (движение vs голос), разные держатели, разные требования к
  watchdog-у. См. Q4.
- **Per-source locks (`/quest_floor_lock` priority 40, отдельно
  блокируем voice/nav2).** Отклонено (Шифу): один lock проще, минимум
  конфига.

## 5. Замер на роботе (raw-evidence, обязателен до merge реализации)

Прежде чем коммитить `t_9d33e8ca` — замерить на железе два факта:

### 5.1 Поведение twist_mux: `strict <` или `<=`

Команда:
```bash
# Активный quest-телеоп (оператор в шлеме, стик подёргивает).
# Параллельно: оператор вручную публикует /teleop_lock = true.
ros2 topic pub /teleop_lock std_msgs/Bool "data: true" --once
# Наблюдаем /cmd_vel — едет ли от cmd_vel_quest?
ros2 topic echo /cmd_vel
```

Если `/cmd_vel` молчит → поведение `strict <`, lock 90 не глушит
источник 90 → ОК, реализуем как есть.

Если `/cmd_vel` едет → поведение `<=` (или twist_mux игнорирует
источник==lock) → fallback: поднять quest до 91 или опустить lock до 89.

### 5.2 Поведение twist_mux при «липком» lock в момент падения арбитра

(Definition of Done #2 из карточки, но watchdog отменён — теперь это
проверка именно того, что **отсутствие watchdog-а безопасно** при
падении арбитра. Поскольку аппаратура(100) выше lock-а(90) — залипший
lock не глушит пульт, но это надо подтвердить.)

```bash
# 1. Включить пульт (ARMED).
# 2. Поднять avatar_arbiter, он публикует /teleop_lock = true.
# 3. ros2 topic echo /cmd_vel_joy — видим движение от пульта.
# 4. SIGKILL avatar_arbiter.
# 5. Смотрим /cmd_vel — должен ехать от пульта, НЕ блокироваться.
ros2 topic echo /cmd_vel
```

### 5.3 Исходный замер из §5 draft v0 (текущая дыра)

```bash
# 1. Quest WS держит teleop_floor, стик в нейтрали (молчит > 0.5 с).
# 2. Параллельно шлём cmd_vel_voice через telega-бота / dialogue_node.
# 3. Смотрим, что реально едет на /cmd_vel (выход twist_mux).
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_voice
ros2 topic echo /avatar/state | grep -E "teleop_floor|voice_floor|mode"
```

Ожидаемо увидим до реализации: при quest-floor=HELD в `/avatar/state` в
`/cmd_vel` всё равно приходят движения от voice/nav2 — это и есть дыра.

После реализации — тот же сценарий, но `/cmd_vel` молчит пока
`/avatar/state.teleop_floor != null`. Это и есть приёмочный тест для
`t_9d33e8ca`.

## 6. Definition of Done для этой карточки

- [x] Решение записано как amendment к ADR-0028 §4.2 / §7 (этот файл).
- [ ] Замер на роботе: поведение `twist_mux` при «липком» lock в момент
      падения арбитра (raw-лог, фрагмент 30-50 строк). Делегировано
      в `t_9d33e8ca` (требует доступ к железу).
- [x] Заведена карточка `t_9d33e8ca` (backend) на реализацию с
      конкретной схемой lock-топиков и acceptance criteria, привязанными
      к этому ADR.

## 7. Что НЕ делаем в этой карточки

- Не пишем код avatar_arbiter / twist_mux.yaml / QuestBridge —
  это `t_9d33e8ca`.
- Не переименовываем существующие топики.
- Не трогаем FSM режимов (off/telegram_active/avatar_present/mixed) —
  это отдельный ADR.
- Не вводим `/teleop_lock_watchdog` (отменено, см. Q2).

## 8. История решений

| Дата | Событие | Решение |
|---|---|---|
| 2026-09-08 | ADR-0080 draft (t_8045c101, PR #2200) | Предложены (A)/(B)/(C) по Q1, (C) по Q2, рекомендация R5 по Q4. |
| 2026-09-09 | Коллизия номера с ADR-0080 «восемь швов» (PR #2221, merged) | Переименование 0080 → 0081. |
| 2026-09-09 | Решения Шифу в PR #2200 (krikz, «Решения владельца по Q1–Q4») | Q1: lock=90 (не 100), quest 40→90. Q2: watchdog отменён, чистый sticky. Q3: закрыт через Q1. Q4: без изменений. |

## Источники истины

- ADR-0028 §4.2 (LockManager), §4.4 S10 (dead-man), §7 открытый вопрос 1
- ADR-0027 §3.4 (voice_input_mode — Phase 2)
- ADR-0051 §2.2 (avatar_arbiter как отдельный сервис)
- ADR-0080 «восемь швов целевой архитектуры голоса и шлема» (PR #2221)
- docs/architecture/SYSTEM_OVERVIEW.md §5.4
- `docker/main/config/twist_mux/twist_mux.yaml`
- `src/rob_box_teleop/rob_box_teleop/joystick_control_node.py:115`
  (паттерн `/joystick_lock`)
- `src/rob_box_supervisor/rob_box_supervisor/core/locks.py` (LockManager API)
- `src/rob_box_supervisor/rob_box_supervisor/arbiter_node.py`
  (avatar_arbiter, ROS-узел)
- `src/rob_box_quest/rob_box_quest/core/avatar_arbiter.py`
  (LocalAvatarArbiterClient, stub)
- PR #2200 (issue #2191, комментарий «Решения владельца по Q1–Q4»)
