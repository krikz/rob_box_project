# ADR-0080 — teleop_floor ↔ twist_mux: единое решение о блокировке

**Status:** proposed (2026-09-08, t_8045c101 / issue #2191, agent:architect)
**Amends:** ADR-0028 §4.2 (LockManager) + ADR-0028 §7 «Открытый вопрос 1»
**Scope:** `twist_mux` input gating + arbiter/Quest bridge

> Драфт для обсуждения с Шифу. **Не реализация.** На этом этапе фиксируем
> архитектурное решение и замер на роботе, реализация — отдельной карточкой
> `voice-vr 06.5` (см. Definition of Done).

## 1. Контекст

Арбитраж движения сейчас существует в трёх местах:

1. **in-process mutex `LocalAvatarArbiterClient`** (`src/rob_box_quest/rob_box_quest/core/avatar_arbiter.py`) — временный stub для Quest-WS внутри одного процесса.
2. **`LockManager`** в `avatar_arbiter` (`src/rob_box_supervisor/rob_box_supervisor/core/locks.py`) — единственный владелец `teleop_floor`/`voice_floor` (ADR-0028 §4.2, AV-12 IDL).
3. **приоритеты `twist_mux`** (`docker/main/config/twist_mux/twist_mux.yaml`) — emergency 255 / joystick 100 / web_ui 50 / quest 40 / voice 25 / nav2 10 + один lock `joystick_lock` (`timeout: 0.0`, sticky).

**Дыра (ADR-0028 §7 открытый вопрос 1):** держатель `teleop_floor`
(LockManager) никак не пробрасывается в twist_mux. Пока quest-оператор
держит floor:

- `cmd_vel_voice` (priority 25) и `cmd_vel` от nav2 (10) **всё равно
  проходят** в мультиплексор, если quest молчит дольше своего timeout
  (0.5 с) — а он молчит каждый раз, когда стик в нейтрали.
- Даже если twist_mux по timeout отбрасывает quest, ничего не блокирует
  web_ui / voice / nav2 от **параллельной** подачи команд в эти 0.5 с.
- У `joystick_control_node` аналогичной дыры нет — он публикует
  `/joystick_lock` пока ARMED (`src/rob_box_teleop/rob_box_teleop/joystick_control_node.py:115`),
  и twist_mux с `timeout: 0.0` блокирует нижестоящие источники sticky.

## 2. Вопросы к решению

### Q1. Один lock-topic или по lock на каждый нижестоящий источник?

twist_mux поддерживает несколько locks. У каждого — свой `priority` и
своя группа блокируемых источников (всё, что ниже priority этого lock).
Перед нами:

- **(A) Один общий `/teleop_lock`** с приоритетом teleop-держателя.
  twist_mux блокирует всё, что ниже — web_ui / voice / nav2.
  - **Плюсы:** один topic, минимум конфига twist_mux, аналог
    `/joystick_lock` (понятная модель).
  - **Минусы:** «всё или ничего». Не различаем «оператор держит стик»
    (можно глушить автономию) и «оператор подключился, но не двинул
    стик» (нельзя — nav2 нужен для SLAM).
- **(B) Отдельный `/quest_floor_lock`** (приоритет ~40) + решаем, что
  блокируем.
  - **Плюсы:** можно сделать priority=40 → блочит voice(25)/nav2(10),
    но НЕ блочит joystick(100) и НЕ блочит web_ui(50) (web-UI часто
    используется как kill-switch).
  - **Минусы:** больше конфига, надо заранее знать, кто «ниже» кого.

**Рекомендация:** **(A) `/teleop_lock` с приоритетом 100** (наравне с
joystick) — потому что семантически teleop-держатель == «у меня руль»,
и блокировать нужно ровно то же, что блокирует `/joystick_lock`
(web_ui/voice/nav2). Это зеркалит существующий паттерн joystick и
избегает ad-hoc списков «кого блочим» в yaml.

### Q2. Что делать с «липким» lock при падении арбитра?

В `twist_mux.yaml` стоит `joystick_lock.timeout: 0.0` — «липкий»,
последнее значение сохраняется, если нода умрёт. Поведение
задокументировано как **«вооружён → автономия заблокирована до
восстановления»** (issue #1344 «поворот приходит сам»).

Для `/teleop_lock` есть три варианта:

- **(A) тоже sticky (`timeout: 0.0`)**. Арбитр умер → lock остался
  активным → автономия заблокирована. **Безопаснее для людей рядом с
  роботом**, но требует ручного сброса (`ros2 topic pub ... false`).
- **(B) короткий timeout (`timeout: 0.5`)**. Арбитр умер → через 0.5 с
  lock отпускается, автономия может работать. **Гибче**, но если
  арбитр умер во время того, как оператор реально держит floor — на
  0.5 с появится окно race с автономией.
- **(C) гибрид — sticky, но с watchdog-каналом** (отдельный
  `/teleop_lock_watchdog` с timeout 0.5, twist_mux И блокирует, если
  хоть один из двух true). Арбитр живой → публикует True в оба;
  Арбитр умер → через 0.5 с watchdog отпускается, основной lock
  остаётся True (если успел), автономия всё равно отпускается. Это
  позволяет отпустить автономию **без** ручного `ros2 topic pub`,
  ценой +1 топика.

**Рекомендация:** **(C) гибрид.** Базовый lock sticky — безопасность
как у joystick_lock; watchdog даёт auto-release при падении арбитра без
ручного вмешательства. Это та же модель, что у heartbeat→dead-man в
LockManager (ADR-0028 §4.4 S10), только на стороне twist_mux.

### Q3. Должен ли voice блокироваться при teleop_floor у оператора?

CONTEXT.md явно говорит: **«личность не видит оператора»** — голосовой
поток от ТАРС-личности не должен зависеть от того, есть ли VR-оператор
с гарнитурой. Это **отдельная орбита** (avatar/telegram_active/
mixed).

Но **сам по себе voice_floor и teleop_floor — разные floor-ы**, и
voice_lock **не должен** зависеть от teleop_lock. Т.е. когда
quest-оператор держит teleop_floor:

- voice_floor может оставаться свободным → ТАРС отвечает на wake-word.
- **`cmd_vel_voice`** (приоритет 25) — это НЕ голос, а движение от
  голосовой команды («едь вперёд»). Оно **должно** блокироваться
  teleop_lock, иначе «иди вперёд» из телеграм-бота перебьёт стик
  VR-оператора.

**Решение:** блокируем только `cmd_vel_voice` и `cmd_vel` (nav2), а
**голосовой канал (`/voice/in`, `/voice/out`, TTS-канал) не трогаем**.
Это и есть смысл «voice_floor» — он про голос, а не про движение.

### Q4. Нужен ли `voice_floor` собственный lock в twist_mux?

`voice_floor` — про голосовой поток (один держатель микрофона). У
twist_mux нет «голосовых приоритетов» — он про `cmd_vel_*`. Поэтому
**отдельный lock в twist_mux для `voice_floor` НЕ нужен** —
блокировка голоса решается на уровне dialogue_node (выбор источника
микрофона) и supervisor (FSM режимов), не на уровне twist_mux.

**Зафиксировать явно:** voice_floor → НЕ пробрасывается в twist_mux,
НЕ вводим `/voice_lock`. Это убирает путаницу между двумя разными
«floor-ами».

## 3. Предлагаемое решение (сводка)

| # | Решение | Обоснование |
|---|---|---|
| R1 | `/teleop_lock` (Bool) — новый twist_mux lock, priority=100, timeout=0.0 (sticky) | Зеркалит `/joystick_lock`: один источник движения, блочит всё ниже 100. |
| R2 | `/teleop_lock_watchdog` (Bool) — параллельный, priority=100, timeout=0.5 | Auto-release при падении арбитра, не требует ручного сброса. twist_mux блокирует пока ЛЮБОЙ из двух true (OR по конфигу twist_mux — уточнить в реализации). |
| R3 | Публикация lock-ов — **avatar_arbiter** (единственный владелец LockManager), не QuestBridge и не LocalAvatarArbiterClient | Один источник истины (ADR-0028 §4.2). |
| R4 | Блокируются **web_ui(50), voice(25), navigation(10)** — т.е. всё ниже priority=100. Не блокируется joystick(100), emergency(255). | Семантика «у меня руль» = то же, что `/joystick_lock`. |
| R5 | `voice_floor` НЕ пробрасывается в twist_mux (см. Q4). | voice_floor — про голос, а twist_mux — про `cmd_vel_*`. Разделяем домены явно. |

## 4. Альтернативы (рассмотренные и отклонённые)

- **Полностью убрать twist_mux и гонять `cmd_vel_*` через avatar_arbiter.**
  Отклонено: ADR-0028 §S7 явно требует, чтобы супервизор НЕ публиковал
  `cmd_vel_*` напрямую — только маршрутизация. Перенос маршрутизации в
  арбитр — это уже другая фаза (Phase 3), не вопрос #2191.
- **Неблокирующий арбитраж (только приоритеты twist_mux).** Отклонено:
  именно из-за дыры открыт issue #2191 — «оператор держит floor молча,
  а cmd_vel_voice всё равно проходит».
- **Один `/floor_lock` на оба floor-а.** Отклонено: разные домены
  (движение vs голос), разные держатели, разные требования к
  watchdog-у. См. Q4.

## 5. Замер на роботе (raw-evidence)

Прежде чем коммитить реализацию — замерить на железе текущее поведение
(Definition of Done #2):

```bash
# 1. Quest WS держит teleop_floor, стик в нейтрали (молчит > 0.5 с).
# 2. Параллельно шлём cmd_vel_voice через telega-бота / dialogue_node.
# 3. Смотрим, что реально едет на /cmd_vel (выход twist_mux).
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_voice
ros2 topic echo /avatar/state | grep -E "teleop_floor|voice_floor|mode"
```

Ожидаемо увидим: при quest-floor=HELD в `/avatar/state` в `/cmd_vel`
всё равно приходят движения от voice/nav2 — это и есть дыра.

После реализации — тот же сценарий, но `/cmd_vel` молчит пока
`/avatar/state.teleop_floor != null`. Это и есть приёмочный тест для
карточки реализации.

## 6. Definition of Done для этой карточки

- [x] Решение записано как amendment к ADR-0028 §4.2 / §7 (этот файл).
- [ ] Замер на роботе: поведение `twist_mux` при «липком» lock в момент
      падения арбитра (raw-лог, фрагмент 30-50 строк).
- [ ] Заведена карточка `voice-vr 06.5` на реализацию с конкретной
      схемой lock-топиков и acceptance criteria, привязанными к этому
      ADR.

## 7. Что НЕ делаем в этой карточке

- Не пишем код avatar_arbiter / twist_mux.yaml / QuestBridge.
- Не переименовываем существующие топики.
- Не трогаем FSM режимов (off/telegram_active/avatar_present/mixed) —
  это отдельный ADR.

## 8. Открытые вопросы для Шифу

1. **Q1: один `/teleop_lock` (A) или по lock на источник (B)?**
   Рекомендую (A) — зеркалит `/joystick_lock`, минимум конфига.
2. **Q2: sticky (A), timeout 0.5 (B), гибрид с watchdog (C)?**
   Рекомендую (C) — auto-release при падении арбитра.
3. **Q3: блокировать ли voice_floor в twist_mux?**
   Нет (R5) — это про голос, не про движение.
4. **Q4: priority=100 lock-а (наравне с joystick) или выше/ниже?**
   Рекомендую 100 (= joystick) — оба источника «у меня руль».

После ответа Шифу — этот ADR переходит в `accepted` и создаётся
карточка на реализацию.

## Источники истины

- ADR-0028 §4.2 (LockManager), §4.4 S10 (dead-man), §7 открытый вопрос 1
- ADR-0027 §3.4 (voice_input_mode — Phase 2)
- ADR-0051 §2.2 (avatar_arbiter как отдельный сервис)
- docs/architecture/SYSTEM_OVERVIEW.md §5.4
- `docker/main/config/twist_mux/twist_mux.yaml`
- `src/rob_box_teleop/rob_box_teleop/joystick_control_node.py:115` (паттерн `/joystick_lock`)
- `src/rob_box_supervisor/rob_box_supervisor/core/locks.py` (LockManager API)
- `src/rob_box_supervisor/rob_box_supervisor/arbiter_node.py` (avatar_arbiter, ROS-узел)
- `src/rob_box_quest/rob_box_quest/core/avatar_arbiter.py` (LocalAvatarArbiterClient, stub)
